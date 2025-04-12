package zhujiang

import chisel3._
import chisel3.experimental.hierarchy.{Definition, Instance}
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.IcnBundle
import xijiang.{NodeType, Ring}
import xs.utils.debug.HardwareAssertionKey
import xs.utils.sram.SramBroadcastBundle
import xs.utils.{DFTResetSignals, ResetGen}
import zhujiang.axi.{AxiBoundaryBuffer, AxiBundle, ExtAxiBundle}
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

  private def placeSocket(pfx:String, icn: IcnBundle, idx:Option[Int]): SocketIcnSide = {
    icn.resetInject.foreach(_ := DontCare)
    val pfxStr = s"${pfx}_${idx.map(_.toString).getOrElse("")}"
    val dev = Module(new SocketIcnSide(icn.node))
    dev.io.dev <> icn
    dev.reset := placeResetGen(pfxStr, icn)
    dev.suggestName(s"${pfxStr}_socket")
    dev
  }

  private val memDatIcns = ring.icnSns.get
  private val (memDevSeq, memNameSeq) = memDatIcns.map({ icn =>
    val nidStr = icn.node.nodeId.toHexString
    val attrStr = if(icn.node.attr == "") s"0x$nidStr" else s"${icn.node.attr}"
    val bridge = Module(new AxiBridge(icn.node.copy(attr = attrStr)))
    val nameStr = s"chi_to_axi_0x$nidStr"
    bridge.reset := placeResetGen(nameStr, icn)
    bridge.icn <> icn
    (bridge, nameStr)
  }).unzip
  memDevSeq.zip(memNameSeq).foreach({case(a, b) => a.suggestName(b)})
  private val memAxiPorts = memDevSeq.map(d => {
    val buf = Module(new AxiBoundaryBuffer(d.axi.params))
    buf.reset := d.reset
    buf.io.mst <> d.axi
    buf.suggestName(s"m_axi_${d.axi.params.attr}_buf")
    buf.io.slv
  })

  private val cfgIcnSeq = ring.icnHis.get
  require(cfgIcnSeq.nonEmpty)
  require(cfgIcnSeq.count(_.node.defaultHni) == 1)
  private val (cfgDevSeq, cfgNameSeq) = cfgIcnSeq.map({ icn =>
    val nidStr = icn.node.nodeId.toHexString
    val attrStr = if(icn.node.attr == "") s"0x$nidStr" else s"${icn.node.attr}"
    val nameStr = s"chi_to_axi_lite_0x$nidStr"
    val cfg = Module(new AxiLiteBridge(icn.node.copy(attr = attrStr), 64, 3))
    cfg.icn <> icn
    cfg.reset := placeResetGen(nameStr, icn)
    cfg.nodeId := icn.node.nodeId.U
    (cfg, nameStr)
  }).unzip
  cfgDevSeq.zip(cfgNameSeq).foreach({case(a, b) => a.suggestName(b)})
  private val cfgAxiPorts = cfgDevSeq.map(d => {
    if(d.axi.params.attr.contains("main")) {
      d.axi
    } else {
    val buf = Module(new AxiBoundaryBuffer(d.axi.params))
    buf.reset := d.reset
    buf.io.mst <> d.axi
    buf.suggestName(s"m_axi_${d.axi.params.attr}_buf")
    buf.io.slv
      }
  })

  private val dmaIcnSeq = ring.icnRis.get
  require(dmaIcnSeq.nonEmpty)
  private val (dmaDevSeq, dmaNameSeq) = dmaIcnSeq.zipWithIndex.map({case(icn, idx) =>
    val nidStr = icn.node.nodeId.toHexString
    val attrStr = if(icn.node.attr == "") s"0x$nidStr" else s"${icn.node.attr}"
    val nameStr = s"axi_to_chi_0x$nidStr"
    val dma = Module(new Axi2Chi(icn.node.copy(attr = attrStr)))
    dma.icn <> icn
    dma.reset := placeResetGen(nameStr, icn)
    (dma, nameStr)
  }).unzip
  dmaDevSeq.zip(dmaNameSeq).foreach({case(a, b) => a.suggestName(b)})
  private val dmaAxiPorts = dmaDevSeq.map(d => {
    if(d.axi.params.attr.contains("main")) {
      d.axi
    } else {
      val buf = Module(new AxiBoundaryBuffer(d.axi.params))
      buf.reset := d.reset
      d.axi <> buf.io.slv
      buf.suggestName(s"s_axi_${d.axi.params.attr}_buf")
      buf.io.mst
    }
  })

  require(ring.icnCcs.get.nonEmpty)
  private val ccnIcnSeq = ring.icnCcs.get
  private val ccnSocketSeq = ccnIcnSeq.map(icn => placeSocket("cc", icn, Some(icn.node.domainId)))

  require(ring.icnHfs.get.nonEmpty)
  private val hfIcnSeq = ring.icnHfs.get.sortBy(_.node.hfpId).groupBy(_.node.bankId).toSeq
  private val nrHfFrnd = ring.icnHfs.get.map(_.node.friends.size).max
  private val hfDef = Definition(new HomeWrapper(hfIcnSeq.head._2.map(_.node), nrHfFrnd)) // TODO: There's a risk here.
  private val hfDevSeq = Seq.tabulate(hfIcnSeq.size)(_ => Instance(hfDef))
  for(i <- hfIcnSeq.indices) {
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
    hfDevSeq(i).reset := placeResetGen(s"hnf_$bankId", hfIcnSeq(i)._2.head)
    hfDevSeq(i).clock := clock
    hfDevSeq(i).io.dfx := dft
    hfDevSeq(i).suggestName(s"hnf_$bankId")
  }

  require(ring.icnMns.get.nonEmpty)
  private val mnIcn = ring.icnMns.get.head
  private val mnDev = Module(new MiscDevice(mnIcn.node))
  mnDev.io.icn <> mnIcn
  mnIcn.resetInject.get := mnDev.io.icn.resetInject.get
  mnDev.io.icn.resetState.get := mnIcn.resetState.get
  mnDev.clock := clock
  mnDev.reset := reset

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

  def runIOAutomation():Unit = {
    ddrIO.zip(ddrDrv).zipWithIndex.foreach({ case((a, b), i) =>
      a.suggestName(s"m_axi_mem_${b.params.attr}")
      a <> b
      dontTouch(a)
      dontTouch(b)
    })
    cfgIO.zip(cfgDrv).zipWithIndex.foreach({ case((a, b), i) =>
      a.suggestName(s"m_axi_cfg_${b.params.attr}")
      a <> b
      dontTouch(a)
      dontTouch(b)
    })
    dmaIO.zip(dmaDrv).zipWithIndex.foreach({ case((a, b), i) =>
      a.suggestName(s"s_axi_${b.params.attr}")
      a <> b
      dontTouch(a)
      dontTouch(b)
    })
    ccnIO.zip(ccnDrv).foreach({ case (a, b) =>
      a.suggestName(s"ccn_0x${b.node.nodeId.toHexString}")
      a <> b
      dontTouch(a)
    })
    hwaIO.zip(hwaDrv).foreach({ case (a, b) =>
      a.suggestName(s"s_axi_hwa")
      a <> b
      dontTouch(a)
      dontTouch(b)
    })
  }
}