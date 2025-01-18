package zhujiang

import chisel3._
import chisel3.experimental.hierarchy.{Definition, Instance}
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{NodeType, Ring}
import dongjiang.pcu._
import dongjiang.dcu._
import xijiang.router.base.IcnBundle
import xs.utils.debug.{DomainInfo, HardwareAssertion}
import xs.utils.sram.SramBroadcastBundle
import xs.utils.{DFTResetSignals, ResetGen}
import zhujiang.axi.{AxiBundle, ExtAxiBundle}
import zhujiang.device.bridge.axilite.AxiLiteBridge
import zhujiang.device.socket.{SocketIcnSide, SocketIcnSideBundle}
import zhujiang.device.ddr.MemoryComplex
import zhujiang.device.dma.Axi2Chi
import zhujiang.device.reset.ResetDevice

import scala.math.pow

class DftWires extends Bundle {
  val reset = new DFTResetSignals
  val func = new SramBroadcastBundle
}

class Zhujiang(isTop:Boolean = false)(implicit p: Parameters) extends ZJModule with NocIOHelper {
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
  private val dft = Wire(new DftWires)
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

  private def placeSocket(pfx:String, icn: IcnBundle, idx:Option[Int]): SocketIcnSide = {
    icn.resetInject.foreach(_ := DontCare)
    val pfxStr = s"${pfx}_${idx.map(_.toString).getOrElse("")}"
    val dev = Module(new SocketIcnSide(icn.node))
    dev.io.icn <> icn
    dev.reset := placeResetGen(pfxStr, icn)
    dev.suggestName(s"${pfxStr}_socket")
    dev
  }

  require(localRing.icnHis.get.count(_.node.attr == "ddr_cfg") == 1)
  require(localRing.icnSns.get.count(_.node.attr == "ddr_data") == 1)
  private val memCfgIcn = localRing.icnHis.get.filter(_.node.attr == "ddr_cfg").head
  private val memDatIcn = localRing.icnSns.get.filter(_.node.attr == "ddr_data").head
  private val memSubSys = Module(new MemoryComplex(memCfgIcn.node, memDatIcn.node))
  memSubSys.io.icn.cfg <> memCfgIcn
  memSubSys.io.icn.mem <> memDatIcn
  memSubSys.reset := placeResetGen(s"ddr", memCfgIcn)

  require(localRing.icnHis.get.count(_.node.attr == "main") == 1)
  require(localRing.icnRis.get.count(_.node.attr == "main") == 1)
  require(localRing.icnHis.get.length == 2)

  private val cfgIcnSeq = localRing.icnHis.get.filterNot(_.node.attr == "ddr_cfg")
  require(cfgIcnSeq.nonEmpty)
  require(cfgIcnSeq.count(_.node.defaultHni) == 1)
  private val cfgDevSeq = cfgIcnSeq.zipWithIndex.map({case(icn, idx) =>
    val default = icn.node.defaultHni
    val pfxStr = if(default) "cfg_default" else s"cfg_$idx"
    val cfg = Module(new AxiLiteBridge(icn.node, 64, 3))
    cfg.icn <> icn
    cfg.reset := placeResetGen(pfxStr, icn)
    cfg
  })

  private val dmaIcnSeq = localRing.icnRis.get
  require(dmaIcnSeq.nonEmpty)
  private val dmaDevSeq = dmaIcnSeq.zipWithIndex.map({case(icn, idx) =>
    val pfxStr = s"dma_$idx"
    val dma = Module(new Axi2Chi(icn.node))
    dma.icn <> icn
    dma.reset := placeResetGen(pfxStr, icn)
    dma.suggestName(pfxStr)
    dma
  })

  private val resetDev = Module(new ResetDevice)
  resetDev.clock := clock
  resetDev.reset := reset
  private val defaultCfg = cfgIcnSeq.filter(_.node.defaultHni).head
  defaultCfg.resetInject.get := resetDev.io.resetInject
  resetDev.io.resetState := defaultCfg.resetState.get

  require(localRing.icnCcs.get.nonEmpty)
  private val ccnIcnSeq = localRing.icnCcs.get
  private val ccnSocketSeq = ccnIcnSeq.map(icn => placeSocket("cc", icn, Some(icn.node.domainId)))

  require(localRing.icnHfs.get.nonEmpty)
  private val pcuIcnSeq = localRing.icnHfs.get
  private val pcuDef = Definition(new ProtocolCtrlUnit(pcuIcnSeq.head.node)) // TODO: There's a risk here
  private val pcuDevSeq = pcuIcnSeq.map(icn => Instance(pcuDef))
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
    val chip = Input(UInt(nodeAidBits.W))
    val onReset = Output(Bool())
    val assertionOut = assertionNode.assertion.cloneType
    val dft = Input(new DftWires)
  })
  dft := io.dft
  val ddrDrv = memSubSys.io.ddr
  val cfgDrv = cfgDevSeq.map(_.axi)
  val dmaDrv = dmaDevSeq.map(_.axi)
  val ccnDrv = ccnSocketSeq.map(_.io.socket)
  runIOAutomation()
  io.assertionOut <> assertionNode.assertion
  io.onReset := resetDev.io.onReset
  localRing.io_chip := io.chip
  dontTouch(io.assertionOut)
  HardwareAssertion.setTopNode(assertionNode)
  val assertionInfo = DomainInfo(assertionNode.desc)
  if(isTop) HardwareAssertion.release("hwa")
}

trait NocIOHelper {
  def p: Parameters
  def ddrDrv: AxiBundle
  def cfgDrv: Seq[AxiBundle]
  def dmaDrv: Seq[AxiBundle]
  def ccnDrv: Seq[SocketIcnSideBundle]

  lazy val ddrIO: ExtAxiBundle = IO(new ExtAxiBundle(ddrDrv.params))
  lazy val cfgIO: Seq[ExtAxiBundle] = cfgDrv.map(drv => IO(new ExtAxiBundle(drv.params)))
  lazy val dmaIO: Seq[ExtAxiBundle] = dmaDrv.map(drv => IO(Flipped(new ExtAxiBundle(drv.params))))
  lazy val ccnIO: Seq[SocketIcnSideBundle] = ccnDrv.map(drv => IO(new SocketIcnSideBundle(drv.node)(p)))

  def runIOAutomation():Unit = {
    ddrIO <> ddrDrv
    ddrIO.suggestName("m_axi_ddr")
    cfgIO.zip(cfgDrv).zipWithIndex.foreach({ case((a, b), i) =>
      if(b.params.attr != "") a.suggestName(s"m_axi_cfg_${b.params.attr}")
      else a.suggestName(s"m_axi_cfg_$i")
      a <> b
    })
    dmaIO.zip(dmaDrv).zipWithIndex.foreach({ case((a, b), i) =>
      if(b.params.attr != "") a.suggestName(s"s_dma_${b.params.attr}")
      else a.suggestName(s"s_axi_dma_$i")
      a <> b
    })
    ccnIO.zip(ccnDrv).foreach({ case (a, b) =>
      a.suggestName(s"ccn_0x${b.node.nodeId.toHexString}")
      a <> b
      dontTouch(a)
    })
  }
}