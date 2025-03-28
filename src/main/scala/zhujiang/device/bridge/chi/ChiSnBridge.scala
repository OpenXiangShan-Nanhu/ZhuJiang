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
    aidBits = node.aidBits
  )

  val icn = IO(new DeviceIcnBundle(node))
  val sn = IO(new IcnBundle(snNode))
  val nodeId = IO(Input(UInt(niw.W)))

  private def compareTag(addr0: UInt, addr1: UInt): Bool = true.B

  icn.tx.req.get.valid := false.B
  icn.tx.req.get.bits := DontCare

  private val wakeups = Wire(Vec(node.outstanding, Valid(UInt(raw.W))))

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
  sn.rx.resp.get.ready := true.B
  for((cm, idx) <- cms.zipWithIndex) {
    cm.icn.rx.req.valid := icn.rx.req.get.valid && enqCtrl.bits(idx)
    cm.icn.rx.req.bits := icn.rx.req.get.bits.asTypeOf(new ReqFlit)
    cm.icn.rx.resp.get.valid := icn.rx.resp.get.valid && icn.rx.resp.get.bits.asTypeOf(new RespFlit).TxnID === idx.U
    cm.icn.rx.resp.get.bits := icn.rx.resp.get.bits.asTypeOf(new RespFlit)
    cm.icn.rx.data.valid := icn.rx.data.get.valid && icn.rx.data.get.bits.asTypeOf(new DataFlit).TxnID === idx.U
    cm.icn.rx.data.bits := icn.rx.data.get.bits.asTypeOf(new DataFlit)

    cm.sn.rx.resp.valid := sn.rx.resp.get.valid && sn.rx.resp.get.bits.asTypeOf(new RespFlit).TxnID === idx.U
    cm.sn.rx.resp.bits := sn.rx.resp.get.bits.asTypeOf(cm.sn.rx.resp.bits)
    cm.io.readDataFire := sn.rx.data.get.fire && sn.rx.data.get.bits.asTypeOf(new DataFlit).TxnID === idx.U
    cm.io.readDataLast := true.B
    cm.io.waitNum := waitNum
  }

  private val readDataPipe = Module(new Queue(gen = new DataFlit, entries = 1, pipe = true))
  private val snDataBits = sn.rx.data.get.bits.asTypeOf(new DataFlit)
  private val ctrlVec = VecInit(cms.map(_.io.info.bits))
  private val ctrlSel = ctrlVec(snDataBits.TxnID(log2Ceil(node.outstanding) - 1, 0))

  readDataPipe.io.enq.valid := sn.rx.data.get.valid
  sn.rx.data.get.ready := readDataPipe.io.enq.ready
  readDataPipe.io.enq.bits := sn.rx.data.get.bits.asTypeOf(new DataFlit)

  readDataPipe.io.enq.bits.Opcode := DatOpcode.CompData
  readDataPipe.io.enq.bits.DataID := ctrlSel.addr(5, 4)
  readDataPipe.io.enq.bits.TxnID := ctrlSel.txnId
  readDataPipe.io.enq.bits.SrcID := 0.U
  readDataPipe.io.enq.bits.DBID := snDataBits.TxnID
  readDataPipe.io.enq.bits.HomeNID := nodeId
  readDataPipe.io.enq.bits.TgtID := ctrlSel.srcId
  readDataPipe.io.enq.bits.Resp := "b010".U

  icn.tx.data.get.valid := readDataPipe.io.deq.valid
  icn.tx.data.get.bits := readDataPipe.io.deq.bits.asTypeOf(icn.tx.data.get.bits)
  readDataPipe.io.deq.ready := icn.tx.data.get.ready
}
