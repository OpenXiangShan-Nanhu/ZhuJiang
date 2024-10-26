package zhujiang.device.bridge.chi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}
import xs.utils.{PickOneLow, ResetRRArbiter}
import zhujiang.chi._
import zhujiang.ZJModule

class ChiSnBridge(node: Node)(implicit p: Parameters) extends ZJModule {
  private val compareTagBits = 16
  val ioDataBits = 64
  val tagOffset = log2Ceil(ioDataBits / 8)
  require(node.nodeType == NodeType.HI)
  private val snNode = Node(
    nodeType = NodeType.S,
    nidBits = node.nidBits,
    aidBits = node.aidBits,
    splitFlit = true
  )

  val icn = IO(new DeviceIcnBundle(node))
  val sn = IO(new IcnBundle(snNode))

  private def compareTag(addr0: UInt, addr1: UInt): Bool = {
    addr0(compareTagBits + tagOffset - 1, tagOffset) === addr1(compareTagBits + tagOffset - 1, tagOffset)
  }

  icn.tx.req.get.valid := false.B
  icn.tx.req.get.bits := DontCare

  private val wakeups = Wire(Vec(node.outstanding, Valid(UInt(raw.W))))

  private val reqPipe = Module(new Queue(icn.rx.req.get.bits.cloneType, entries = 1, pipe = true))
  reqPipe.io.enq <> icn.rx.req.get

  private val icnRspArb = Module(new ResetRRArbiter(icn.tx.resp.get.bits.cloneType, node.outstanding))
  icn.tx.resp.get <> icnRspArb.io.out

  private val snReqArb = Module(new ResetRRArbiter(sn.tx.req.get.bits.cloneType, node.outstanding))
  sn.tx.req.get <> snReqArb.io.out

  private val snDatArb = Module(new ResetRRArbiter(sn.tx.data.get.bits.cloneType, node.outstanding))
  sn.tx.data.get <> snDatArb.io.out

  private val cms = for(idx <- 0 until node.outstanding) yield {
    val cm = Module(new ChiSnBridgeCtrlMachine(node, node.outstanding, ioDataBits, compareTag))
    cm.suggestName(s"cm_$idx")
    cm.io.wakeupIns := wakeups.zipWithIndex.filterNot(_._2 == idx).map(_._1)
    wakeups(idx).valid := cm.io.wakeupOut.valid
    wakeups(idx).bits := cm.io.wakeupOut.bits
    cm.io.idx := idx.U
    icnRspArb.io.in(idx).valid := cm.icn.tx.resp.valid
    icnRspArb.io.in(idx).bits := cm.icn.tx.resp.bits.asTypeOf(icn.tx.resp.get.bits.cloneType)
    cm.icn.tx.resp.ready := icnRspArb.io.in(idx).ready
    snReqArb.io.in(idx) <> cm.sn.tx.req
    snDatArb.io.in(idx) <> cm.sn.tx.data
    cm
  }

  private val shouldBeWaited = cms.map(cm => cm.io.info.bits.isSnooped && cm.io.info.valid && !cm.io.wakeupOut.valid)
  private val cmAddrSeq = cms.map(cm => cm.io.info.bits.addr)
  private val req = icn.rx.req.get.bits.asTypeOf(new ReqFlit)
  private val reqTagMatchVec = VecInit(shouldBeWaited.zip(cmAddrSeq).map(elm => elm._1 && compareTag(elm._2, req.Addr)))
  private val reqTagMatchVecReg = RegEnable(reqTagMatchVec, reqPipe.io.enq.fire)

  private val busyEntries = cms.map(_.io.info.valid)
  private val enqCtrl = PickOneLow(busyEntries)

  reqPipe.io.deq.ready := enqCtrl.valid
  icn.rx.resp.get.ready := true.B
  icn.rx.data.get.ready := true.B
  sn.rx.resp.get.ready := true.B
  sn.rx.data.get.ready := true.B
  for((cm, idx) <- cms.zipWithIndex) {
    cm.icn.rx.req.valid := reqPipe.io.deq.valid && enqCtrl.bits(idx)
    cm.icn.rx.req.bits := reqPipe.io.deq.bits.asTypeOf(new ReqFlit)
    cm.io.waitNum := PopCount(reqTagMatchVecReg)
    cm.icn.rx.resp.get.valid := icn.rx.resp.get.valid && icn.rx.resp.get.bits.asTypeOf(new RespFlit).TxnID === idx.U
    cm.icn.rx.resp.get.bits := icn.rx.resp.get.bits.asTypeOf(new RespFlit)
    cm.icn.rx.data.valid := icn.rx.data.get.valid && icn.rx.data.get.bits.asTypeOf(new DataFlit).TxnID === idx.U
    cm.icn.rx.data.bits := icn.rx.data.get.bits.asTypeOf(new DataFlit)

    cm.sn.rx.resp.valid := sn.rx.resp.get.valid && sn.rx.resp.get.bits.asTypeOf(new RespFlit).TxnID === idx.U
    cm.sn.rx.resp.bits := sn.rx.resp.get.bits.asTypeOf(cm.sn.rx.resp.bits)
    cm.io.readDataFire := sn.rx.data.get.fire && sn.rx.data.get.bits.asTypeOf(new DataFlit).TxnID === idx.U
    cm.io.readDataLast := true.B
  }

  private val readDataPipe = Module(new Queue(gen = new DataFlit, entries = 1, pipe = true))
  private val snDataBits = sn.rx.data.get.bits.asTypeOf(new DataFlit)
  private val ctrlVec = VecInit(cms.map(_.io.info.bits))
  private val ctrlSel = ctrlVec(snDataBits.TxnID(log2Ceil(node.outstanding) - 1, 0))

  readDataPipe.io.enq.valid := sn.rx.data.get.valid
  sn.rx.data.get.ready := readDataPipe.io.enq.ready
  readDataPipe.io.enq.bits := sn.rx.data.get.bits.asTypeOf(new DataFlit)

  readDataPipe.io.enq.bits.Opcode := DatOpcode.CompData
  readDataPipe.io.enq.bits.DataID := 0.U
  readDataPipe.io.enq.bits.TxnID := ctrlSel.txnId
  readDataPipe.io.enq.bits.SrcID := 0.U
  readDataPipe.io.enq.bits.TgtID := ctrlSel.srcId
  readDataPipe.io.enq.bits.Resp := "b010".U

  icn.tx.data.get.valid := readDataPipe.io.deq.valid
  icn.tx.data.get.bits := readDataPipe.io.deq.bits.asTypeOf(icn.tx.data.get.bits)
  readDataPipe.io.deq.ready := icn.tx.data.get.ready
}
