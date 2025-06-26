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
import xs.utils.dft.{BaseTestBundle, PowerDomainTestBundle}
import xs.utils.mbist.MbistInterface
import xs.utils.sram.{SramCtrlBundle, SramHelper}
import zhujiang.chi.FlitHelper.{connIcn, hwaConn}
import zhujiang.chi._
import zhujiang.utils.DoubleCounterClockGate
import zhujiang.{ZJDftWires, ZJRawModule}

@instantiable
class HomeWrapper(nodes: Seq[Node], nrFriends: Int)(implicit p: Parameters) extends ZJRawModule with ImplicitClock with ImplicitReset {
  private val node = nodes.head
  @public
  val io = IO(new Bundle {
    val lans = MixedVec(nodes.map(new DeviceIcnBundle(_)))
    val friends = Input(Vec(nodes.size, Vec(nrFriends, UInt(niw.W))))
    val nids = Input(Vec(nodes.size, UInt(niw.W)))
    val ci = Input(UInt(ciIdBits.W))
    val bank = Input(UInt(nodes.head.bankBits.W))
    val dfx = new ZJDftWires
    val ramctl = Input(new SramCtrlBundle)
  })
  @public val reset = IO(Input(AsyncReset()))
  @public val clock = IO(Input(Clock()))
  val implicitClock = clock
  val implicitReset = reset

  private val cg = Module(new DoubleCounterClockGate(testctl = true))
  private val hnx = Module(new DongJiang(node))
  private val lanPipes = Seq.tabulate(nodes.length, zjParams.hnxPipelineDepth + 1) { case (i, j) =>
    val pipe = Module(new ChiBuffer(nodes(i)))
    pipe.suggestName(s"lan_${i}_pipe_$j")
  }
  require(zjParams.hnxPipelineDepth < 12)
  private val hnxLans = for(i <- nodes.indices) yield {
    for(j <- 1 until lanPipes(i).size) {
      lanPipes(i)(j).io.dev <> lanPipes(i)(j - 1).io.icn
    }
    lanPipes(i).head.io.dev <> io.lans(i)
    lanPipes(i).last.io.icn
  }

  private val inbound = hnxLans.map({ lan =>
    val reqv = lan.tx.req.map(_.valid).getOrElse(false.B)
    val hprv = lan.tx.hpr.map(_.valid).getOrElse(false.B)
    reqv | hprv
  }).reduce(_ | _)
  cg.io.te := io.dfx.cgen
  cg.io.test_on.foreach(_ := io.dfx.llc.clk_on)
  cg.io.test_off.foreach(_ := io.dfx.llc.clk_off)
  cg.io.inbound := inbound
  cg.io.working := hnx.io.working
  hnx.io.config.ci := io.ci
  hnx.io.config.bankId := io.bank
  hnx.clock := cg.io.ock
  hnx.io.flushCache.req.valid := false.B
  hnx.io.flushCache.req.bits := DontCare
  hnx.io.config.closeLLC := false.B
  hnx.io.ramPwrCtl.deact := !cg.io.active
  hnx.io.ramPwrCtl.ret := false.B
  hnx.io.ramPwrCtl.stop := false.B

  for(chn <- nodes.head.ejects.filterNot(_ == "DBG").filterNot(n => n == "HPR" && !hasHprRing)) {
    val rxSeq = hnxLans.map(_.tx.bundleMap(chn))
    if(rxSeq.size == 1) {
      hnx.io.lan.rx.bundleMap(chn) <> rxSeq.head
    } else {
      val arb = ResetRRArbiter(rxSeq.head.bits.cloneType, rxSeq.size)
      arb.io.in.zip(rxSeq).foreach({ case (a, b) => a <> b })
      hnx.io.lan.rx.bundleMap(chn) <> arb.io.out
    }
  }
  private val erqTgt = Wire(UInt(niw.W))
  dontTouch(erqTgt)
  private val mems = zjParams.island.filter(n => n.nodeType == NodeType.S)
  for(chn <- nodes.head.injects.filterNot(_ == "DBG").filterNot(n => n == "HPR" && !hasHprRing)) {
    val txBdSeq = hnxLans.map(_.rx.bundleMap(chn))
    val txBd = hnx.io.lan.tx.bundleMap(chn)
    val tgt = if(chn == "ERQ" && mems.nonEmpty) {
      val addr = txBd.bits.asTypeOf(new HReqFlit).Addr.asTypeOf(new ReqAddrBundle)
      val memSelOH = mems.map(m => m.addrCheck(addr, io.ci))
      val memIds = mems.map(_.nodeId.U(niw.W))
      when(txBd.valid) {
        HAssert(PopCount(memSelOH) === 1.U, cf"ERQ addr not match! @ 0x${addr.asUInt}%x")
      }
      Mux1H(memSelOH, memIds)
    } else {
      txBd.bits.asTypeOf(new RingFlit(txBd.bits.getWidth)).TgtID
    }

    val friendsHitVec = Wire(UInt(io.friends.size.W))
    friendsHitVec := Cat(io.friends.map(fs => Cat(fs.map(_ === tgt.asTypeOf(new NodeIdBundle).router)).orR).reverse)
    dontTouch(friendsHitVec)

    val srcId = Mux1H(friendsHitVec, io.nids)

    val txd = if(chn == "ERQ" && mems.nonEmpty) {
      val ori = txBd.bits.asTypeOf(new HReqFlit)
      val res = WireInit(ori)
      val noDmt = ori.ReturnNID.get.andR
      res.ReturnNID.get := Mux(noDmt, srcId, ori.ReturnNID.get)
      res.TgtID := tgt
      erqTgt := tgt
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

  val mbistIntf = MbistInterface("NocHome", io.dfx, hasMbist)
  SramHelper.genSramCtrlBundleTop() := io.ramctl
  private val assertionNode = HardwareAssertion.placePipe(Int.MaxValue, moduleTop = true).map(_.head)
  HardwareAssertion.release(assertionNode, "hwa", "home")
  assertionNode.foreach(_.hassert.bus.get.ready := true.B)
  if(p(HardwareAssertionKey).enable) {
    val dbgTx = hnxLans.filter(_.node.hfpId == 0).head.rx.debug.get
    val dbgBd = WireInit(0.U.asTypeOf(Decoupled(new RingFlit(debugFlitBits))))
    if(assertionNode.isDefined) {
      dontTouch(assertionNode.get.hassert)
      hwaConn(dbgBd, assertionNode.get.hassert)
    }
    connIcn(dbgTx, dbgBd)
  }
}
