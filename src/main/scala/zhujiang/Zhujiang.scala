package zhujiang

import chisel3._
import chisel3.experimental.hierarchy.{Definition, Instance}
import chisel3.util._
import chisel3.experimental.{ChiselAnnotation, annotate}
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType, Ring}
import dongjiang.pcu._
import dongjiang.dcu._
import zhujiang.chi.ChiBuffer
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}
import xs.utils.debug.HardwareAssertion
import xs.utils.sram.SramBroadcastBundle
import xs.utils.{DFTResetSignals, ResetGen}
import zhujiang.axi.AxiBundle
import zhujiang.device.async.{DeviceIcnAsyncBundle, IcnAsyncBundle, IcnSideAsyncModule}
import zhujiang.device.ddr.MemoryComplex
import zhujiang.device.reset.ResetDevice
import scala.math.pow

class DftWires extends Bundle {
  val reset = new DFTResetSignals
  val func = new SramBroadcastBundle
}

class Zhujiang(implicit p: Parameters) extends ZJModule {
  require(p(ZJParametersKey).tfsParams.isEmpty)

  print(
    s"""
       |ZhuJiang Message: {
       |  Support Protocol: CHI-G
       |  nodeIdBits: ${niw}
       |  requestAddrBits: ${raw}
       |  dataBits: ${dw}
       |  dataCheckBits: ${dcw}
       |  txnIdBits: 12
       |  dbIdBits: 16
       |}
       |""".stripMargin)

  private val localRing = Module(new Ring(true))
  val dft = IO(Input(new DftWires))
  localRing.dfx_reset := dft.reset
  localRing.clock := clock

  private def placeResetGen(name: String, icn: IcnBundle): AsyncReset = {
    val mst = Seq(NodeType.CC, NodeType.RI, NodeType.RF).map(_ == icn.node.nodeType).reduce(_ || _)
    val rstGen = Module(new ResetGen)
    rstGen.suggestName(name + "_rst_sync")
    rstGen.dft := dft.reset
    if(mst) rstGen.reset := icn.resetState.get(0).asAsyncReset
    else rstGen.reset := icn.resetState.get(1).asAsyncReset
    rstGen.o_reset
  }

  require(localRing.icnHis.get.count(_.node.attr == "ddr_cfg") == 1)
  require(localRing.icnSns.get.count(_.node.attr == "ddr_data") == 1)
  private val memCfgIcn = localRing.icnHis.get.filter(_.node.attr == "ddr_cfg").head
  private val memDatIcn = localRing.icnSns.get.filter(_.node.attr == "ddr_data").head
  private val memSubSys = Module(new MemoryComplex(memCfgIcn.node, memDatIcn.node))
  memSubSys.io.icn.cfg <> memCfgIcn
  memSubSys.io.icn.mem <> memDatIcn
  memSubSys.reset := placeResetGen(s"ddr", memCfgIcn)

  require(localRing.icnHis.get.count(_.node.defaultHni) == 1)
  require(localRing.icnRis.get.count(_.node.attr == "dma") == 1)
  require(localRing.icnHis.get.length == 2)

  private val socCfgIcn = localRing.icnHis.get.filter(_.node.defaultHni).head
  private val socDmaIcn = localRing.icnRis.get.filter(n => n.node.attr == "dma").head
  private val socCfgDev = Module(new IcnSideAsyncModule(socCfgIcn.node))
  private val socDmaDev = Module(new IcnSideAsyncModule(socDmaIcn.node))
  socCfgDev.io.icn <> socCfgIcn
  socDmaDev.io.icn <> socDmaIcn
  socCfgDev.reset := placeResetGen(s"soc", socCfgIcn)
  socDmaDev.reset := socCfgDev.reset

  private val resetDev = Module(new ResetDevice)
  resetDev.clock := clock
  resetDev.reset := reset
  socCfgIcn.resetInject.get := resetDev.io.resetInject
  resetDev.io.resetState := socCfgIcn.resetState.get

  require(localRing.icnCcs.get.nonEmpty)
  private val ccnIcnSeq = localRing.icnCcs.get
  private val ccnAysncDevSeq = if(p(ZJParametersKey).cpuAsync) Some(ccnIcnSeq.map(icn => Module(new IcnSideAsyncModule(icn.node)))) else None
  private val ccnSyncDevSeq = if(p(ZJParametersKey).cpuAsync) None else Some(ccnIcnSeq.map(icn => Module(new ChiBuffer(icn.node))))
  for(i <- ccnIcnSeq.indices) {
    val domainId = ccnIcnSeq(i).node.domainId
    ccnAysncDevSeq.foreach(devs => devs(i).io.icn <> ccnIcnSeq(i))
    ccnSyncDevSeq.foreach(devs => devs(i).io.in <> ccnIcnSeq(i))
    ccnAysncDevSeq.foreach(devs => devs(i).reset := placeResetGen(s"cc_$domainId", ccnIcnSeq(i)))
    ccnSyncDevSeq.foreach(devs => devs(i).reset := placeResetGen(s"cc_$domainId", ccnIcnSeq(i)))
  }

  require(localRing.icnHfs.get.nonEmpty)
  private val pcuIcnSeq = localRing.icnHfs.get
  private val pcuDef = Definition(new ProtocolCtrlUnit(pcuIcnSeq.head.node)) // TODO: There's a risk here
  private val pcuDevSeq = pcuIcnSeq.map(icn => Instance(pcuDef))
  private val nrPCU = localRing.icnHfs.get.length
  private val nrDCU = localRing.icnSns.get.filterNot(_.node.mainMemory).length
  for(i <- pcuIcnSeq.indices) {
    val bankId = pcuIcnSeq(i).node.bankId
    pcuDevSeq(i).io.hnfID := pcuIcnSeq(i).node.nodeId.U
    pcuDevSeq(i).io.pcuID := bankId.U
    pcuDevSeq(i).io.dcuNodeIDVec.zipWithIndex.foreach { case(n, j) => n := pcuIcnSeq(i).node.friends(j).nodeId.U }
    pcuDevSeq(i).io.toLocal <> pcuIcnSeq(i)
    pcuDevSeq(i).reset := placeResetGen(s"pcu_$bankId", pcuIcnSeq(i))
    pcuDevSeq(i).clock := clock
    pcuDevSeq(i).suggestName(s"pcu_$bankId")
  }

  require(!localRing.icnSns.get.forall(_.node.mainMemory))
  private val dcuIcnSeq = localRing.icnSns.get.filterNot(_.node.mainMemory).sortBy(_.node.dpId).groupBy(_.node.bankId).toSeq
  private val dcuDef = Definition(new DataCtrlUnit(dcuIcnSeq.head._2.map(_.node).sortBy(_.dpId))) // TODO: There's a risk here.
  private val dcuDevSeq = dcuIcnSeq.map(is => Instance(dcuDef))
  for(i <- dcuIcnSeq.indices) {
    val bankId = dcuIcnSeq(i)._1
    for(j <- dcuIcnSeq(i)._2.indices) dcuDevSeq(i).io.icns(j) <> dcuIcnSeq(i)._2(j)
    for(j <- dcuIcnSeq(i)._2.indices) {
      dcuDevSeq(i).io.friendsNodeIDVec(j).zipWithIndex.foreach {
        case (v, k) =>
          if (k < dcuIcnSeq(i)._2.map(_.node.friends.map(_.nodeId.U))(j).length) {
            v := dcuIcnSeq(i)._2.map(_.node.friends.map(_.nodeId.U))(j)(k)
          } else {
            v := (pow(2, niw).toInt - 1).U
          }
      }
    }
    dcuDevSeq(i).reset := placeResetGen(s"dcu_$bankId", dcuIcnSeq(i)._2.head)
    dcuDevSeq(i).clock := clock
    dcuDevSeq(i).suggestName(s"dcu_$bankId")
    HardwareAssertion.fromDomain(dcuDevSeq(i).assertionOut, dcuDevSeq(i).assertionInfo, level = 0, s"dcu $bankId")
    HardwareAssertion.placePipe(1)
  }
  private val assertionNode = HardwareAssertion.placePipe(Int.MaxValue)
  val io = IO(new Bundle {
    val ddr = new AxiBundle(memSubSys.io.ddr.params)
    val soc = new SocIcnBundle(socCfgDev.io.icn.node, socDmaDev.io.icn.node)
    val ccn = MixedVec(ccnIcnSeq.map(cc => new CcnIcnBundle(cc.node)))
    val chip = Input(UInt(nodeAidBits.W))
    val onReset = Output(Bool())
    val assertionOut = Output(assertionNode.assertion.cloneType)
  })
  HardwareAssertion.setTopNode(assertionNode)
  io.assertionOut := assertionNode.assertion
  io.onReset := resetDev.io.onReset
  io.ddr <> memSubSys.io.ddr
  io.soc.cfg <> socCfgDev.io.async
  io.soc.dma <> socDmaDev.io.async
  io.soc.reset := socCfgDev.reset
  localRing.io_chip := io.chip
  for(i <- io.ccn.indices) {
    io.ccn(i).async.foreach(_ <> ccnAysncDevSeq.get(i).io.async)
    io.ccn(i).sync.foreach(_ <> ccnSyncDevSeq.get(i).io.out)
    if(p(ZJParametersKey).cpuAsync) {
      io.ccn(i).reset := ccnAysncDevSeq.get(i).reset
    } else {
      io.ccn(i).reset := ccnSyncDevSeq.get(i).reset
    }
  }
}

class CcnIcnBundle(val node:Node)(implicit p:Parameters) extends Bundle {
  val async = if(p(ZJParametersKey).cpuAsync) Some(new IcnAsyncBundle(node)) else None
  val sync = if(p(ZJParametersKey).cpuAsync) None else Some(new IcnBundle(node))
  val reset = Output(AsyncReset())
  def <>(that: CcnDevBundle):Unit = {
    this.async.foreach(_ <> that.async.get)
    this.sync.foreach(_ <> that.sync.get)
    that.reset := this.reset
  }
}

class CcnDevBundle(val node:Node)(implicit p:Parameters) extends Bundle {
  val async = if(p(ZJParametersKey).cpuAsync) Some(new DeviceIcnAsyncBundle(node)) else None
  val sync = if(p(ZJParametersKey).cpuAsync) None else Some(new DeviceIcnBundle(node))
  val reset = Input(AsyncReset())
  def <>(that: CcnIcnBundle):Unit = {
    this.async.foreach(_ <> that.async.get)
    this.sync.foreach(_ <> that.sync.get)
    this.reset := that.reset
  }
}

class SocIcnBundle(cfgNode:Node, dmaNode:Node)(implicit p:Parameters) extends Bundle {
  val cfg = new IcnAsyncBundle(cfgNode)
  val dma = new IcnAsyncBundle(dmaNode)
  val reset = Output(AsyncReset())
  def <>(that: SocDevBundle):Unit = {
    this.cfg <> that.cfg
    this.dma <> that.dma
    that.reset := this.reset
  }
}

class SocDevBundle(cfgNode:Node, dmaNode:Node)(implicit p:Parameters) extends Bundle {
  val cfg = new DeviceIcnAsyncBundle(cfgNode)
  val dma = new DeviceIcnAsyncBundle(dmaNode)
  val reset = Input(AsyncReset())
  def <>(that: SocIcnBundle):Unit = {
    this.cfg <> that.cfg
    this.dma <> that.dma
    this.reset := that.reset
  }
}