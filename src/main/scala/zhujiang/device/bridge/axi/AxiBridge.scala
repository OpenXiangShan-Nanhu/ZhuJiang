package zhujiang.device.bridge.axi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.PickOneLow
import zhujiang.ZJModule
import zhujiang.axi._
import zhujiang.chi.{DatOpcode, DataFlit, ReqFlit, RespFlit}

class AxiBridge(node: Node)(implicit p: Parameters) extends ZJModule {
  private val compareTagBits = 24
  private val tagOffset = 6
  require(node.nodeType == NodeType.S)
  private val axiParams = AxiParams(idBits = log2Ceil(node.outstanding), dataBits = dw)

  val icn = IO(new DeviceIcnBundle(node))
  val axi = IO(new AxiBundle(axiParams))

  private def compareTag(addr0: UInt, addr1: UInt): Bool = {
    addr0(compareTagBits + tagOffset - 1, tagOffset) === addr1(compareTagBits + tagOffset - 1, tagOffset)
  }

  private val wakeups = Wire(Vec(node.outstanding, Valid(UInt(raw.W))))

  private val icnRspArb = Module(new RRArbiter(icn.tx.resp.get.bits.cloneType, node.outstanding))
  icn.tx.resp.get <> icnRspArb.io.out

  private val awArb = Module(new RRArbiter(new AWFlit(axiParams), node.outstanding))
  axi.aw <> awArb.io.out

  private val arArb = Module(new RRArbiter(new ARFlit(axiParams), node.outstanding))
  axi.ar <> arArb.io.out

  private val dataBuffer = Module(new AxiDataBuffer(axiParams, node.outstanding, node.outstanding))
  private val dataBufferallocSelector = Module(new DataBufferAllocReqSelector(node.outstanding))
  dataBuffer.io.alloc <> dataBufferallocSelector.io.out
  dataBuffer.io.icn.valid := icn.rx.data.get.valid
  dataBuffer.io.icn.bits := icn.rx.data.get.bits.asTypeOf(dataBuffer.io.icn.bits)
  icn.rx.data.get.ready := dataBuffer.io.icn.ready
  axi.w <> dataBuffer.io.axi

  private val cms = for(idx <- 0 until node.outstanding) yield {
    val cm = Module(new AxiBridgeCtrlMachine(node, axiParams, node.outstanding, compareTag))
    cm.suggestName(s"cm_$idx")
    cm.io.wakeupIns := wakeups.zipWithIndex.filterNot(_._2 == idx).map(_._1)
    wakeups(idx).valid := cm.io.wakeupOut.valid
    wakeups(idx).bits := cm.io.wakeupOut.bits
    cm.io.idx := idx.U
    icnRspArb.io.in(idx).valid := cm.icn.tx.resp.valid
    icnRspArb.io.in(idx).bits := cm.icn.tx.resp.bits.asTypeOf(icn.tx.resp.get.bits.cloneType)
    cm.icn.tx.resp.ready := icnRspArb.io.in(idx).ready
    awArb.io.in(idx) <> cm.axi.aw
    arArb.io.in(idx) <> cm.axi.ar
    dataBufferallocSelector.io.in(idx) <> cm.dataBufferAlloc.req
    cm.dataBufferAlloc.resp := dataBufferallocSelector.io.out.fire && dataBufferallocSelector.io.out.bits.idxOH(idx)
    cm
  }

  private val wSeq = cms.map(_.axi.w)
  private val awQueue = Module(new Queue(UInt(node.outstanding.W), entries = node.outstanding))
  awQueue.io.enq.valid := awArb.io.out.fire
  awQueue.io.enq.bits := UIntToOH(awArb.io.chosen)
  when(awArb.io.out.fire) {
    assert(awQueue.io.enq.ready)
  }
  private val wSelValid = Mux1H(awQueue.io.deq.bits, wSeq.map(_.valid))
  awQueue.io.deq.ready := dataBuffer.io.fromCmDat.ready && wSelValid
  dataBuffer.io.fromCmDat.valid := awQueue.io.deq.valid && wSelValid
  dataBuffer.io.fromCmDat.bits.flit := Mux1H(awQueue.io.deq.bits, wSeq.map(_.bits))
  dataBuffer.io.fromCmDat.bits.idxOH := awQueue.io.deq.bits
  wSeq.zipWithIndex.foreach({ case (w, i) => w.ready := dataBuffer.io.fromCmDat.ready && awQueue.io.deq.valid && awQueue.io.deq.bits(i) })

  private val shouldBeWaited = cms.map(cm => cm.io.info.valid && !cm.io.wakeupOut.valid)
  private val cmAddrSeq = cms.map(cm => cm.io.info.bits.addr)
  private val req = icn.rx.req.get.bits.asTypeOf(new ReqFlit)
  private val reqTagMatchVec = shouldBeWaited.zip(cmAddrSeq).map(elm => elm._1 && compareTag(elm._2, req.Addr))
  private val waitNum = PopCount(reqTagMatchVec)

  private val busyEntries = cms.map(_.io.info.valid)
  private val enqCtrl = PickOneLow(busyEntries)

  icn.rx.req.get.ready := enqCtrl.valid
  icn.rx.data.get.ready := true.B
  axi.r.ready := true.B
  axi.b.ready := true.B

  for((cm, idx) <- cms.zipWithIndex) {
    cm.icn.rx.req.valid := icn.rx.req.get.valid && enqCtrl.bits(idx)
    cm.icn.rx.req.bits := icn.rx.req.get.bits.asTypeOf(new ReqFlit)
    cm.icn.rx.data.valid := dataBuffer.io.toCmDat.valid && dataBuffer.io.toCmDat.bits.TxnID === idx.U
    cm.icn.rx.data.bits := dataBuffer.io.toCmDat.bits

    cm.axi.b.valid := axi.b.valid && axi.b.bits.id === idx.U
    cm.axi.b.bits := axi.b.bits
    cm.io.readDataFire := axi.r.fire && axi.r.bits.id === idx.U
    cm.io.readDataLast := axi.r.bits.last
    cm.io.waitNum := waitNum
  }

  private val readDataPipe = Module(new Queue(gen = new DataFlit, entries = 1, pipe = true))
  private val ctrlVec = VecInit(cms.map(_.io.info.bits))
  private val ctrlSel = ctrlVec(axi.r.bits.id(log2Ceil(node.outstanding) - 1, 0))

  readDataPipe.io.enq.valid := axi.r.valid
  axi.r.ready := readDataPipe.io.enq.ready

  readDataPipe.io.enq.bits := DontCare
  readDataPipe.io.enq.bits.Data := axi.r.bits.data
  readDataPipe.io.enq.bits.Opcode := DatOpcode.CompData
  readDataPipe.io.enq.bits.DataID := ctrlSel.readCnt
  readDataPipe.io.enq.bits.TxnID := ctrlSel.returnTxnId.get
  readDataPipe.io.enq.bits.SrcID := 0.U
  readDataPipe.io.enq.bits.TgtID := ctrlSel.returnNid.get
  readDataPipe.io.enq.bits.HomeNID := ctrlSel.srcId
  readDataPipe.io.enq.bits.DBID := ctrlSel.txnId
  readDataPipe.io.enq.bits.Resp := "b010".U

  icn.tx.data.get.valid := readDataPipe.io.deq.valid
  icn.tx.data.get.bits := readDataPipe.io.deq.bits.asTypeOf(icn.tx.data.get.bits)
  readDataPipe.io.deq.ready := icn.tx.data.get.ready
}
