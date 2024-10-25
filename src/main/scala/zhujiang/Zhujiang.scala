package zhujiang

import chisel3._
import chisel3.experimental.hierarchy.{Definition, Instance}
import chisel3.util._
import chisel3.experimental.{ChiselAnnotation, annotate}
import org.chipsalliance.cde.config.Parameters
import xijiang.{NodeType, Ring}
import dongjiang.pcu._
import dongjiang.dcu._
import dongjiang.ddrc._
import chisel3.util.{Decoupled, DecoupledIO}
import xijiang.c2c.C2cLinkPort
import zhujiang.chi.{DataFlit, ReqFlit, RespFlit}
import sifive.enterprise.firrtl.NestedPrefixModulesAnnotation
import xijiang.router.base.IcnBundle
import xs.utils.{DFTResetSignals, ResetGen}
import zhujiang.axi.AxiBundle
import zhujiang.device.async.{IcnAsyncBundle, IcnSideAsyncModule}
import zhujiang.device.cluster.interconnect.DftWires
import zhujiang.device.ddr.MemoryComplex
import zhujiang.device.reset.ResetDevice

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
  socCfgDev.reset := placeResetGen(s"cfg", socCfgIcn)
  socDmaDev.reset := placeResetGen(s"dma", socDmaIcn)

  private val resetDev = Module(new ResetDevice)
  resetDev.clock := clock
  resetDev.reset := reset
  socCfgIcn.resetInject.get := resetDev.io.resetInject
  resetDev.io.resetState := socCfgIcn.resetState.get

  require(localRing.icnCcs.get.nonEmpty)
  private val ccnIcnSeq = localRing.icnCcs.get
  private val ccnDevSeq = ccnIcnSeq.map(icn => Module(new IcnSideAsyncModule(icn.node)))
  for(i <- ccnIcnSeq.indices) {
    val domainId = ccnIcnSeq(i).node.domainId
    ccnDevSeq(i).io.icn <> ccnIcnSeq(i)
    ccnDevSeq(i).reset := placeResetGen(s"cc_$domainId", ccnIcnSeq(i))
    ccnDevSeq(i).suggestName(s"cluster_async_sink_$domainId")
  }

  require(localRing.icnHfs.get.nonEmpty)
  private val pcuIcnSeq = localRing.icnHfs.get
  private val pcuDef = Definition(new ProtocolCtrlUnit(pcuIcnSeq.head.node))
  private val pcuDevSeq = pcuIcnSeq.map(icn => Instance(pcuDef))
  for(i <- pcuIcnSeq.indices) {
    val bankId = pcuIcnSeq(i).node.bankId
    pcuDevSeq(i).io.hnfID := pcuIcnSeq(i).node.nodeId.U
    pcuDevSeq(i).io.toLocal <> pcuIcnSeq(i)
    pcuDevSeq(i).reset := placeResetGen(s"pcu_$bankId", pcuIcnSeq(i))
    pcuDevSeq(i).clock := clock
    pcuDevSeq(i).suggestName(s"pcu_$bankId")
  }

  require(!localRing.icnSns.get.forall(_.node.mainMemory))
  private val dcuIcnSeq = localRing.icnSns.get.filterNot(_.node.mainMemory).groupBy(_.node.bankId).toSeq
  private val dcuDef = Definition(new DataCtrlUnit(dcuIcnSeq.head._2.head.node, dcuIcnSeq.head._2.length))
  private val dcuDevSeq = dcuIcnSeq.map(is => Instance(dcuDef))
  for(i <- dcuIcnSeq.indices) {
    val bankId = dcuIcnSeq(i)._1
    for(j <- dcuIcnSeq(i)._2.indices) dcuDevSeq(i).io.sn(j) <> dcuIcnSeq(i)._2(j)
    dcuDevSeq(i).reset := placeResetGen(s"dcu_$bankId", dcuIcnSeq(i)._2.head)
    dcuDevSeq(i).clock := clock
    dcuDevSeq(i).suggestName(s"dcu_$bankId")
  }

  val io = IO(new Bundle {
    val ddr = new AxiBundle(memSubSys.io.ddr.params)
    val soc = new Bundle {
      val cfg = new IcnAsyncBundle(socCfgDev.io.icn.node)
      val dma = new IcnAsyncBundle(socDmaDev.io.icn.node)
    }
    val cluster = MixedVec(ccnDevSeq.map(cc => new IcnAsyncBundle(cc.io.icn.node)))
    val chip = Input(UInt(nodeAidBits.W))
    val onReset = Output(Bool())
  })
  io.onReset := resetDev.io.onReset
  io.cluster.suggestName("cluster_async")
  io.ddr <> memSubSys.io.ddr
  io.soc.cfg <> socCfgDev.io.async
  io.soc.dma <> socDmaDev.io.async
  for(i <- ccnDevSeq.indices) io.cluster(i) <> ccnDevSeq(i).io.async
  localRing.io_chip := io.chip
}
