package zhujiang.device.home

import chisel3._
import chisel3.experimental.hierarchy.{instantiable, public}
import chisel3.util._
import dongjiang.DongJiang
import zhujiang.{DftWires, ZJModule, ZJRawModule}
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.ResetRRArbiter
import xs.utils.debug.{DomainInfo, HardwareAssertion}
import xs.utils.perf.PerfCounterUtils
import zhujiang.chi.{ChiBuffer, HReqFlit, NodeIdBundle, ReqAddrBundle, RingFlit}

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
  private val lanPipes = Seq.tabulate(nodes.length, zjParams.hnxPipelineDepth + 2) { case(i, j) =>
    val pipe = Module(new ChiBuffer(nodes(i)))
    pipe.suggestName(s"lan_${i}_pipe_$j")
  }
  private val inbound = lanPipes.map({ ps =>
    val out = ps.head.io.out
    Cat(out.node.ejects.map(chn => out.tx.getBundle(chn).get.valid)).orR
  }).reduce(_ | _)

  cg.io.CK := clock
  cg.io.E := ckCtrl
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
    ckCtrl := Mux(ckenCnt.orR, true.B, RegNext(hnx.io.working, false.B))
  }
  when(inbound) {
    ckenCnt := zjParams.hnxCgThreshold.U
  }.elsewhen(ckenCnt.orR) {
    ckenCnt := ckenCnt - 1.U
  }

  private val hnxLans = for(i <- nodes.indices) yield {
    for(j <- 1 until lanPipes(i).size) {
      lanPipes(i)(j).io.in <> lanPipes(i)(j - 1).io.out
    }
    lanPipes(i).head.io.in <> io.lans(i)
    lanPipes(i).last.io.out
  }

  for(chn <- node.ejects) {
    val rxSeq = hnxLans.map(_.tx.getBundle(chn).get)
    if(rxSeq.size == 1) {
      hnx.io.lan.rx.getBundle(chn).get <> rxSeq.head
    } else {
      val arb = ResetRRArbiter(rxSeq.head.bits.cloneType, rxSeq.size)
      arb.io.in.zip(rxSeq).foreach({case(a, b) => a <> b})
      hnx.io.lan.rx.getBundle(chn).get <> arb.io.out
    }
  }

  private val mems = zjParams.island.filter(n => n.nodeType == NodeType.S && n.attr == "mem")
  for(chn <- node.injects) {
    val txBdSeq = hnxLans.map(_.rx.getBundle(chn).get)
    val txBd = hnx.io.lan.tx.getBundle(chn).get

    val tgt = if(chn == "ERQ" && mems.nonEmpty) {
      val addr = txBd.bits.asTypeOf(new HReqFlit).Addr.asTypeOf(new ReqAddrBundle)
      val memSelOH = mems.map(m => addr.checkBank(m.bankBits, m.bankId.U, zjParams.memBankOff))
      val memIds = mems.map(_.nodeId.U(niw.W))
      Mux1H(memSelOH, memIds)
    } else {
      txBd.bits.asTypeOf(new RingFlit(txBd.bits.getWidth)).TgtID
    }

    val friendsHitVec = io.friends.map(fs => Cat(fs.map(_ === tgt)).orR)
    val srcId = Mux1H(friendsHitVec, io.nids)

    val txd = if(chn == "ERQ" && mems.nonEmpty) {
      val ori = txBd.bits.asTypeOf(new HReqFlit)
      val res = WireInit(ori)
      val noDmt = ori.ReturnNID.get.andR
      res.ReturnNID.get := Mux(noDmt, srcId, ori.ReturnNID.get)
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
      assert(PopCount(friendsHitVec) === 1.U)
    }
  }

  HardwareAssertion(true.B)
  private val assertionNode = HardwareAssertion.placePipe(Int.MaxValue, true)
  @public
  val assertionOut = IO(assertionNode.assertion.cloneType)
  @public
  val assertionInfo = DomainInfo(assertionNode.desc)
  assertionOut <> assertionNode.assertion

  PerfCounterUtils.genXmrHelper()
}
