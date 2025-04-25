package zhujiang.device.home

import chisel3._
import chisel3.experimental.hierarchy.{instantiable, public}
import chisel3.util._
import dongjiang.DongJiang
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.DeviceIcnBundle
import xijiang.{Node, NodeType}
import xs.utils.ResetRRArbiter
import xs.utils.debug.{HAssert, HardwareAssertion, HardwareAssertionKey}
import xs.utils.mbist.{MbistInterface, MbistPipeline}
import xs.utils.sram.SramHelper
import zhujiang.chi.FlitHelper.connIcn
import zhujiang.chi.{ChiBuffer, DataFlit, HReqFlit, NodeIdBundle, ReqAddrBundle, RingFlit}
import zhujiang.{DftWires, ZJRawModule}

@instantiable
class HomeWrapper(nodes:Seq[Node], nrFriends:Int)(implicit p:Parameters) extends ZJRawModule with ImplicitClock with ImplicitReset {
  private val node = nodes.head
  @public
  val io = IO(new Bundle {
    val lans = MixedVec(nodes.map(new DeviceIcnBundle(_)))
    val friends = Input(Vec(nodes.size, Vec(nrFriends, UInt(niw.W))))
    val nids = Input(Vec(nodes.size, UInt(niw.W)))
    val ci = Input(UInt(ciIdBits.W))
    val bank = Input(UInt(nodes.head.bankBits.W))
    val dfx = Input(new DftWires)
  })
  @public val reset = IO(Input(AsyncReset()))
  @public val clock = IO(Input(Clock()))
  val implicitClock = clock
  val implicitReset = reset

  private val cg = Module(new xs.utils.ClockGate)
  private val ckCtrl = RegInit(true.B)
  private val ckenCnt = RegInit(zjParams.hnxCgThreshold.U(log2Ceil(zjParams.hnxCgThreshold + 1).W))
  private val hnx = Module(new DongJiang(node))
  private val lanPipes = Seq.tabulate(nodes.length, zjParams.hnxPipelineDepth + 1) { case(i, j) =>
    val pipe = Module(new ChiBuffer(nodes(i)))
    pipe.suggestName(s"lan_${i}_pipe_$j")
  }
  private val inbound = io.lans.map({ lan =>
    val rxvs = lan.rx.elements.values.map {
      case chn:DecoupledIO[Data] => chn.valid
      case _ => false.B
    }
    RegNext(Cat(rxvs.toSeq).orR)
  }).reduce(_ | _)

  if(zjParams.hnxPipelineDepth == 0) {
    cg.io.E := ckCtrl | inbound
  } else {
    cg.io.E := ckCtrl
  }
  cg.io.CK := clock
  cg.io.TE := io.dfx.func.cgen
  hnx.io.config.ci := io.ci
  hnx.io.config.bankId := io.bank
  hnx.clock := cg.io.Q
  hnx.io.flushCache.req.valid := false.B
  hnx.io.flushCache.req.bits := DontCare
  hnx.io.config.closeLLC := false.B

  when(inbound) {
    ckCtrl := true.B
  }.elsewhen(ckCtrl) {
    ckCtrl := Mux(ckenCnt.orR, true.B, RegNext(hnx.io.working))
  }
  when(inbound) {
    ckenCnt := zjParams.hnxCgThreshold.U
  }.elsewhen(ckenCnt.orR) {
    ckenCnt := ckenCnt - 1.U
  }

  private val hnxLans = for(i <- nodes.indices) yield {
    for(j <- 1 until lanPipes(i).size) {
      lanPipes(i)(j).io.dev <> lanPipes(i)(j - 1).io.icn
    }
    lanPipes(i).head.io.dev <> io.lans(i)
    lanPipes(i).last.io.icn
  }

  for(chn <- node.ejects.filterNot(_ == "DBG")) {
    val rxSeq = hnxLans.map(_.tx.getBundle(chn).get)
    if(rxSeq.size == 1) {
      hnx.io.lan.rx.getBundle(chn).get <> rxSeq.head
    } else {
      val arb = ResetRRArbiter(rxSeq.head.bits.cloneType, rxSeq.size)
      arb.io.in.zip(rxSeq).foreach({case(a, b) => a <> b})
      hnx.io.lan.rx.getBundle(chn).get <> arb.io.out
    }
  }

  private val mems = zjParams.island.filter(n => n.nodeType == NodeType.S)
  for(chn <- node.injects.filterNot(_ == "DBG")) {
    val txBdSeq = hnxLans.map(_.rx.getBundle(chn).get)
    val txBd = hnx.io.lan.tx.getBundle(chn).get

    val tgt = if(chn == "ERQ" && mems.nonEmpty) {
      val addr = txBd.bits.asTypeOf(new HReqFlit).Addr.asTypeOf(new ReqAddrBundle)
      val memSelOH = mems.map(m => m.addrCheck(addr, io.ci))
      val memIds = mems.map(_.nodeId.U(niw.W))
      when(txBd.valid) { HAssert(PopCount(memSelOH) === 1.U, cf"ERQ addr not match! @ 0x${addr.asUInt}%x") }
      Mux1H(memSelOH, memIds)
    } else {
      txBd.bits.asTypeOf(new RingFlit(txBd.bits.getWidth)).TgtID
    }

    val friendsHitVec = Wire(UInt(io.friends.size.W))
    friendsHitVec    := Cat(io.friends.map(fs => Cat(fs.map(_ === tgt.asTypeOf(new NodeIdBundle).router)).orR).reverse)
    dontTouch(friendsHitVec)

    val srcId = Mux1H(friendsHitVec, io.nids)

    val txd = if(chn == "ERQ" && mems.nonEmpty) {
      val ori = txBd.bits.asTypeOf(new HReqFlit)
      val res = WireInit(ori)
      val noDmt = ori.ReturnNID.get.andR
      res.ReturnNID.get := Mux(noDmt, srcId, ori.ReturnNID.get)
      res.TgtID := tgt
      res.asTypeOf(txBd.bits)
    } else if(chn == "DAT") {
      val ori = txBd.bits.asTypeOf(new DataFlit)
      val res = WireInit(ori)
      res.HomeNID := srcId
      res.TgtID := tgt
      res.asTypeOf(txBd.bits)
    } else {
      val ori = txBd.bits.asTypeOf(new RingFlit(txBd.bits.getWidth))
      val res = WireInit(ori)
      res.TgtID := tgt
      res.asTypeOf(txBd.bits)
    }

    for(i <- txBdSeq.indices) {
      txBdSeq(i).valid := txBd.valid & friendsHitVec(i)
      txBdSeq(i).bits := txd
    }
    txBd.ready := Mux1H(friendsHitVec, txBdSeq.map(_.ready))
    when(txBd.valid) {
      HAssert(PopCount(friendsHitVec) === 1.U, cf"$chn port friends not match!")
    }
  }

  hnx.io.lan.tx.debug.foreach(_ := DontCare)

  MbistInterface("NocHome", io.dfx.func, hasMbist)
  private val assertionNode = HardwareAssertion.placePipe(Int.MaxValue, moduleTop = true).map(_.head)
  HardwareAssertion.release(assertionNode, "hwa", "home")
  assertionNode.foreach(_.hassert.bus.get.ready := true.B)
  if(p(HardwareAssertionKey).enable) {
    val dbgTx = hnxLans.filter(_.node.hfpId == 0).head.rx.debug.get
    val dbgBd = WireInit(0.U.asTypeOf(Decoupled(new RingFlit(debugFlitBits))))
    if(assertionNode.isDefined) {
      dontTouch(assertionNode.get.hassert)
      dbgBd.bits.Payload := assertionNode.get.hassert.bus.get.bits
      dbgBd.valid := assertionNode.get.hassert.bus.get.valid
      assertionNode.get.hassert.bus.get.ready := dbgBd.ready
    }
    connIcn(dbgTx, dbgBd)
  }
}
