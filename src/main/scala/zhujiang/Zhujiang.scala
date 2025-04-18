package zhujiang

import chisel3._
import chisel3.experimental.hierarchy.{Definition, Instance}
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.IcnBundle
import xijiang.{NodeType, Ring}
import xs.utils.debug.HardwareAssertionKey
import xs.utils.mbist.{MbistInterface, MbistPipeline}
import xs.utils.sram.{SramBroadcastBundle, SramHelper}
import xs.utils.{DFTResetSignals, ResetGen}
import zhujiang.axi.{AxiBuffer, AxiBundle, ExtAxiBundle}
import zhujiang.device.bridge.axi.AxiBridge
import zhujiang.device.bridge.axilite.AxiLiteBridge
import zhujiang.device.dma.Axi2Chi
import zhujiang.device.home.HomeWrapper
import zhujiang.device.misc.MiscDevice
import zhujiang.device.socket.{SocketIcnSide, SocketIcnSideBundle}

class DftWires extends Bundle {
  val reset = new DFTResetSignals
  val func = new SramBroadcastBundle
}

class Zhujiang(implicit p: Parameters) extends ZJModule with NocIOHelper {
  require(p(ZJParametersKey).tfsParams.isEmpty)
  override val desiredName = zjParams.ciName
  print(
    s"""
       |${desiredName} Info: {
       |  Support Protocol: CHI-G
       |  nodeIdBits: ${niw}
       |  requestAddrBits: ${raw}
       |  dataBits: ${dw}
       |  dataCheckBits: ${dcw}
       |  txnIdBits: 12
       |  dbIdBits: 16
       |}
       |""".stripMargin)

  private val ring = Module(new Ring)
  private val dft = Wire(new DftWires)
  ring.dfx_reset := dft.reset
  ring.clock := clock

  private def placeResetGen(name: String, icn: IcnBundle): AsyncReset = {
    val mst = Seq(NodeType.CC, NodeType.RI, NodeType.RF).map(_ == icn.node.nodeType).reduce(_ || _)
    val rstGen = Module(new ResetGen)
    rstGen.suggestName(name + "_rst_sync")
    rstGen.dft := dft.reset
    if(mst) rstGen.reset := icn.resetState.get(0).asAsyncReset
    else rstGen.reset := icn.resetState.get(1).asAsyncReset
    rstGen.o_reset
  }

  private def placeSocket(pfx: String, icn: IcnBundle, idx: Option[Int]): SocketIcnSide = {
    icn.resetInject.foreach(_ := DontCare)
    val pfxStr = s"${pfx}_${idx.map(_.toString).getOrElse("")}"
    val dev = Module(new SocketIcnSide(icn.node))
    dev.io.dev <> icn
    dev.reset := placeResetGen(pfxStr, icn)
    dev.suggestName(icn.node.deviceName)
    dev
  }

  require(ring.icnHfs.get.nonEmpty)
  private val hfIcnSeq = ring.icnHfs.get.sortBy(_.node.hfpId).groupBy(_.node.bankId).toSeq
  private val nrHfFrnd = ring.icnHfs.get.map(_.node.friends.size).max
  private val hfDef = Definition(new HomeWrapper(hfIcnSeq.head._2.map(_.node), nrHfFrnd)) // TODO: There's a risk here.
  private val hfDevSeq = Seq.tabulate(hfIcnSeq.size)(_ => Instance(hfDef))
  for(i <- hfIcnSeq.indices) {
    val devName = hfIcnSeq(i)._2.head.node.deviceName
    val bankId = hfIcnSeq(i)._1
    val icnSeq = hfIcnSeq(i)._2
    for(j <- icnSeq.indices) {
      hfDevSeq(i).io.lans(j) <> icnSeq(j)
      hfDevSeq(i).io.nids(j) := icnSeq(j).node.nodeId.U
      for(k <- 0 until nrHfFrnd) {
        val frnds = icnSeq(j).node.friends.map(_.nodeId.U(niw.W))
        if(k < frnds.size) hfDevSeq(i).io.friends(j)(k) := frnds(k)
        else hfDevSeq(i).io.friends(j)(k) := frnds.last
      }
    }
    hfDevSeq(i).io.ci := ring.io_ci
    hfDevSeq(i).io.bank := bankId.U
    hfDevSeq(i).reset := placeResetGen(devName, hfIcnSeq(i)._2.head)
    hfDevSeq(i).clock := clock
    hfDevSeq(i).io.dfx := dft
    hfDevSeq(i).suggestName(devName)
  }

  private val memDatIcns = ring.icnSns.get
  private val memDevSeq = memDatIcns.map({ icn =>
    val nidStr = icn.node.nodeId.toHexString
    val attrStr = if(icn.node.attr == "") s"0x$nidStr" else s"${icn.node.attr}"
    val bridge = Module(new AxiBridge(icn.node.copy(attr = attrStr)))
    bridge.reset := placeResetGen(icn.node.deviceName, icn)
    bridge.icn <> icn
    bridge
  })
  memDevSeq.foreach(d => d.suggestName(d.icn.node.deviceName))
  private val memAxiPorts = memDevSeq.map(_.axi)

  private val cfgIcnSeq = ring.icnHis.get
  require(cfgIcnSeq.nonEmpty)
  require(cfgIcnSeq.count(_.node.defaultHni) == 1)
  private val cfgDevSeq = cfgIcnSeq.map({ icn =>
    val nidStr = icn.node.nodeId.toHexString
    val attrStr = if(icn.node.attr == "") s"0x$nidStr" else s"${icn.node.attr}"
    val cfg = Module(new AxiLiteBridge(icn.node.copy(attr = attrStr), 64, 3))
    cfg.icn <> icn
    cfg.reset := placeResetGen(icn.node.deviceName, icn)
    cfg.nodeId := icn.node.nodeId.U
    cfg
  })
  cfgDevSeq.foreach(d => d.suggestName(d.icn.node.deviceName))
  private val cfgAxiPorts = cfgDevSeq.map(_.axi)

  private val dmaIcnSeq = ring.icnRis.get
  require(dmaIcnSeq.nonEmpty)
  private val dmaDevSeq = dmaIcnSeq.zipWithIndex.map({ case (icn, idx) =>
    val nidStr = icn.node.nodeId.toHexString
    val attrStr = if(icn.node.attr == "") s"0x$nidStr" else s"${icn.node.attr}"
    val dma = Module(new Axi2Chi(icn.node.copy(attr = attrStr)))
    dma.icn <> icn
    dma.reset := placeResetGen(icn.node.deviceName, icn)
    dma
  })
  dmaDevSeq.foreach(d => d.suggestName(d.icn.node.deviceName))
  private val dmaAxiPorts = dmaDevSeq.map(_.axi)

  require(ring.icnCcs.get.nonEmpty)
  private val ccnIcnSeq = ring.icnCcs.get
  private val ccnSocketSeq = ccnIcnSeq.map(icn => placeSocket("cc", icn, Some(icn.node.domainId)))

  require(ring.icnMns.get.nonEmpty)
  private val mnIcn = ring.icnMns.get.head
  private val mnDev = Module(new MiscDevice(mnIcn.node))
  mnDev.io.icn <> mnIcn
  mnIcn.resetInject.get := mnDev.io.icn.resetInject.get
  mnDev.io.icn.resetState.get := mnIcn.resetState.get
  mnDev.clock := clock
  mnDev.reset := reset
  mnDev.suggestName(mnIcn.node.deviceName)

  val io = IO(new Bundle {
    val ci = Input(UInt(ciIdBits.W))
    val onReset = Output(Bool())
    val dft = Input(new DftWires)
    val resetBypass = Output(AsyncReset())
    val intr = Option.when(p(HardwareAssertionKey).enable)(Output(Bool()))
  })
  io.resetBypass := ResetGen(2, Some(io.dft.reset))
  dft := io.dft
  val ddrDrv = memAxiPorts
  val cfgDrv = cfgAxiPorts
  val dmaDrv = dmaAxiPorts
  val ccnDrv = ccnSocketSeq.map(_.io.socket)
  val hwaDrv = mnDev.io.axi
  runIOAutomation()
  io.onReset := mnDev.io.onReset
  ring.io_ci := io.ci
  io.intr.foreach(_ := mnDev.io.intr.get)

  private val mbistPl = MbistPipeline.PlaceMbistPipeline(Int.MaxValue, "MbistPipelineNocMisc", hasMbist)
  private val mbistIntfNocMisc = if (hasMbist) {
    val brc = SramHelper.genBroadCastBundleTop()
    brc := io.dft.func
    val params = mbistPl.get.nodeParams
    val intf = Some(Module(new MbistInterface(
      params = Seq(params),
      ids = Seq(mbistPl.get.childrenIds),
      name = s"MbistIntfNocMisc",
      pipelineNum = 1
    )))
    intf.get.toPipeline.head <> mbistPl.get.mbist
    mbistPl.get.registerCSV(intf.get.info, "MbistNocMisc")
    intf.get.mbist := DontCare
    dontTouch(intf.get.mbist)
    //TODO: add mbist controller connections here
    intf
  } else {
    None
  }
  ZhujiangGlobal.addRing(desiredName, this)
}

trait NocIOHelper {
  def p: Parameters
  def ddrDrv: Seq[AxiBundle]
  def cfgDrv: Seq[AxiBundle]
  def dmaDrv: Seq[AxiBundle]
  def ccnDrv: Seq[SocketIcnSideBundle]
  def hwaDrv: Option[AxiBundle]

  lazy val ddrIO: Seq[ExtAxiBundle] = ddrDrv.map(drv => IO(new ExtAxiBundle(drv.params)))
  lazy val cfgIO: Seq[ExtAxiBundle] = cfgDrv.map(drv => IO(new ExtAxiBundle(drv.params)))
  lazy val dmaIO: Seq[ExtAxiBundle] = dmaDrv.map(drv => IO(Flipped(new ExtAxiBundle(drv.params))))
  lazy val ccnIO: Seq[SocketIcnSideBundle] = ccnDrv.map(drv => IO(new SocketIcnSideBundle(drv.node)(p)))
  lazy val hwaIO: Option[ExtAxiBundle] = hwaDrv.map(drv => IO(Flipped(new ExtAxiBundle(drv.params))))

  def runIOAutomation(bufChain:Int = 0): Unit = {
    ddrIO.zip(ddrDrv).zipWithIndex.foreach({ case ((a, b), i) =>
      val portName = s"m_axi_mem_${b.params.attr}"
      a.suggestName(portName)
      a <> AxiBuffer.chain(b, bufChain, Some(portName))
      dontTouch(a)
      dontTouch(b)
    })
    cfgIO.zip(cfgDrv).zipWithIndex.foreach({ case ((a, b), i) =>
      val portName = s"m_axi_cfg_${b.params.attr}"
      a.suggestName(portName)
      a <> AxiBuffer.chain(b, bufChain, Some(portName))
      dontTouch(a)
      dontTouch(b)
    })
    dmaIO.zip(dmaDrv).zipWithIndex.foreach({ case ((a, b), i) =>
      val portName = s"s_axi_${b.params.attr}"
      a.suggestName(portName)
      b <> AxiBuffer.chain(a, bufChain, Some(portName))
      dontTouch(a)
      dontTouch(b)
    })
    ccnIO.zip(ccnDrv).foreach({ case (a, b) =>
      a.suggestName(s"ccn_0x${b.node.nodeId.toHexString}")
      a <> b
      dontTouch(a)
    })
    hwaIO.zip(hwaDrv).foreach({ case (a, b) =>
      val portName = s"s_axi_hwa"
      a.suggestName(portName)
      b <> AxiBuffer.chain(a, bufChain, Some(portName))
      dontTouch(a)
      dontTouch(b)
    })
  }
}