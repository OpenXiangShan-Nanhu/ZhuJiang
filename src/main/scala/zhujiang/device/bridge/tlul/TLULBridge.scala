package zhujiang.device.bridge.tlul

import chisel3._
import chisel3.util._
import freechips.rocketchip.util.MaskGen
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.PickOneLow
import xs.utils.arb.VipArbiter
import zhujiang.ZJModule
import zhujiang.chi.FlitHelper.connIcn
import zhujiang.chi.{DatOpcode, DataFlit, ReqFlit, RespFlit}
import zhujiang.tilelink.{DOpcode, TLULBundle, TilelinkParams}

class TLULBridge(node: Node, busDataBits: Int, tagOffset: Int)(implicit p: Parameters) extends ZJModule {
  private val compareTagBits = 16
  require(node.nodeType == NodeType.HI || node.nodeType == NodeType.S)
  private val tlParams = TilelinkParams(sourceBits = log2Ceil(node.outstanding), dataBits = busDataBits)

  val icn = IO(new DeviceIcnBundle(node))
  val tl = IO(new TLULBundle(tlParams))
  val nodeId = IO(Input(UInt(niw.W)))
  val working = IO(Output(Bool()))

  private def compareTag(addr0: UInt, addr1: UInt): Bool = true.B

  icn.tx.req.get.valid := false.B
  icn.tx.req.get.bits := DontCare

  private val wakeups = Wire(Vec(node.outstanding, Valid(UInt(raw.W))))

  private val icnRspArb = Module(new VipArbiter(icn.tx.resp.get.bits.cloneType, node.outstanding))
  icn.tx.resp.get <> icnRspArb.io.out

  private val tlaArb = Module(new VipArbiter(tl.a.bits.cloneType, node.outstanding))
  tl.a <> tlaArb.io.out

  private val cms = for(idx <- 0 until node.outstanding) yield {
    val cm = Module(new TLULBridgeCtrlMachine(node, tlParams, node.outstanding, 64, compareTag))
    cm.suggestName(s"cm_$idx")
    cm.io.wakeupIns := wakeups.zipWithIndex.filterNot(_._2 == idx).map(_._1)
    wakeups(idx).valid := cm.io.wakeupOut.valid
    wakeups(idx).bits := cm.io.wakeupOut.bits
    cm.io.idx := idx.U
    icnRspArb.io.in(idx).valid := cm.icn.tx.resp.valid
    icnRspArb.io.in(idx).bits := cm.icn.tx.resp.bits.asTypeOf(icn.tx.resp.get.bits.cloneType)
    cm.icn.tx.resp.ready := icnRspArb.io.in(idx).ready
    tlaArb.io.in(idx) <> cm.tla
    cm
  }
  private val chiTxV = icn.tx.elements.values.map ({
    case chn: DecoupledIO[Data] => chn.valid
    case _ => false.B
  })
  working := RegNext(Cat(cms.map(_.io.info.valid) ++ chiTxV).orR)

  private val shouldBeWaited = cms.map(cm => cm.io.info.valid && !cm.io.wakeupOut.valid && cm.io.info.bits.isSnooped)
  private val cmAddrSeq = cms.map(cm => cm.io.info.bits.addr)
  private val req = icn.rx.req.get.bits.asTypeOf(new ReqFlit)
  private val reqTagMatchVec = VecInit(shouldBeWaited.zip(cmAddrSeq).map(elm => elm._1 && compareTag(elm._2, req.Addr)))
  private val reqTagMatchVecReg = RegEnable(reqTagMatchVec, icn.rx.req.get.fire)
  private val waitNum = PopCount(reqTagMatchVecReg)

  private val busyEntries = cms.map(_.io.info.valid)
  private val enqCtrl = PickOneLow(busyEntries)

  icn.rx.req.get.ready := enqCtrl.valid
  icn.rx.resp.get.ready := true.B
  icn.rx.data.get.ready := true.B

  for((cm, idx) <- cms.zipWithIndex) {
    cm.icn.rx.req.valid := icn.rx.req.get.valid && enqCtrl.bits(idx)
    cm.icn.rx.req.bits := icn.rx.req.get.bits.asTypeOf(new ReqFlit)
    cm.icn.rx.data.valid := icn.rx.data.get.valid && icn.rx.data.get.bits.asTypeOf(new DataFlit).TxnID === idx.U
    cm.icn.rx.data.bits := icn.rx.data.get.bits.asTypeOf(new DataFlit)
    cm.icn.rx.resp.foreach({r =>
      r.valid := icn.rx.resp.get.valid && icn.rx.resp.get.bits.asTypeOf(new RespFlit).TxnID === idx.U
      r.bits := icn.rx.resp.get.bits.asTypeOf(new RespFlit)
    })

    cm.io.readDataFire := tl.d.fire && tl.d.bits.source === idx.U
    cm.io.readDataLast := true.B
    cm.io.waitNum := waitNum
  }

  private val readDataPipe = Module(new Queue(gen = new DataFlit, entries = 1, pipe = true))
  private val ctrlVec = VecInit(cms.map(_.io.info.bits))
  private val ctrlSel = ctrlVec(tl.d.bits.source(log2Ceil(node.outstanding) - 1, 0))
  private val ackData = tl.d.bits.opcode === DOpcode.AccessAckData && tl.d.valid

  readDataPipe.io.enq.valid := ackData
  tl.d.ready := Mux(ackData, readDataPipe.io.enq.ready, true.B)

  readDataPipe.io.enq.bits := DontCare
  readDataPipe.io.enq.bits.Data := Fill(dw / busDataBits, tl.d.bits.data)
  readDataPipe.io.enq.bits.Opcode := DatOpcode.CompData
  if(dw == 512) {
    readDataPipe.io.enq.bits.DataID := 0.U
  } else if(dw == 256) {
    readDataPipe.io.enq.bits.DataID := Cat(ctrlSel.addr(5), false.B)
  } else if (dw == 128) {
    readDataPipe.io.enq.bits.DataID := ctrlSel.addr(5, 4)
  } else {
    require(requirement = false, s"illegal DW $dw")
  }
  readDataPipe.io.enq.bits.SrcID := 0.U
  readDataPipe.io.enq.bits.Resp := "b010".U
  readDataPipe.io.enq.bits.BE := MaskGen(ctrlSel.addr, ctrlSel.size, bew)
  if(node.nodeType == NodeType.S) {
    readDataPipe.io.enq.bits.TxnID := ctrlSel.returnTxnId.get
    readDataPipe.io.enq.bits.TgtID := ctrlSel.returnNid.get
    readDataPipe.io.enq.bits.HomeNID := ctrlSel.srcId
    readDataPipe.io.enq.bits.DBID := ctrlSel.txnId
    readDataPipe.io.enq.bits.QoS := ctrlSel.qos
  } else {
    readDataPipe.io.enq.bits.TxnID := ctrlSel.txnId
    readDataPipe.io.enq.bits.TgtID := ctrlSel.srcId
    readDataPipe.io.enq.bits.HomeNID := nodeId
    readDataPipe.io.enq.bits.DBID := tl.d.bits.source
    readDataPipe.io.enq.bits.QoS := ctrlSel.qos
  }

  connIcn(icn.tx.data.get, readDataPipe.io.deq)
}
