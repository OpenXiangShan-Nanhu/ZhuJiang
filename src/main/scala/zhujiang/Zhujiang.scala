package zhujiang

import chisel3._
import chisel3.experimental.hierarchy.{Definition, Instance}
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.IcnBundle
import xijiang.{NodeType, Ring}
import xs.utils.debug.HardwareAssertionKey
import xs.utils.mbist.{MbistInterface, MbistPipeline}
import xs.utils.sram.{SramBroadcastBundle, SramCtrlBundle, SramHelper}
import xs.utils.{DFTResetSignals, ResetGen}
import zhujiang.axi.{AxiBuffer, AxiBundle, ExtAxiBundle}
import zhujiang.device.bridge.axi.AxiBridge
import zhujiang.device.bridge.axilite.AxiLiteBridge
import zhujiang.device.dma.Axi2Chi
import zhujiang.device.home.HomeWrapper
import zhujiang.device.io.IoWrapper
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
  private val ramctl = Wire(new SramCtrlBundle)
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

  private def placeSocket(icn: IcnBundle, idx: Option[Int]): SocketIcnSide = {
    icn.resetInject.foreach(_ := DontCare)
    val pfxStr = s"cc_${idx.map(_.toString).getOrElse("")}"
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
    ZhujiangGlobal.addHnf(this, devName, icnSeq.map(_.node.nodeId))
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
    hfDevSeq(i).io.ramctl := ramctl
    hfDevSeq(i).suggestName(devName)
  }

  private val ioIcns = ring.icnSns.getOrElse(Seq()) ++
    ring.icnHis.getOrElse(Seq()) ++
    ring.icnRis.getOrElse(Seq()) ++
    ring.icnRhs.getOrElse(Seq()) ++
    ring.icnMns.getOrElse(Seq())
  private val ioIcnGroups = ioIcns.groupBy(_.node.axiDevParams.get.wrapper).toSeq
  private val ioWrps = for((_, iog) <- ioIcnGroups) yield {
    val iow = Module(new IoWrapper(iog.map(_.node)))
    iow.io.dft := dft
    iow.io.ramctl := ramctl
    iow.reset := reset
    iow.icnSeq.zip(iog).foreach({case(a, b) => a <> b})
    iow
  }
  ioWrps.foreach(iow => iow.suggestName(s"iowrp_${iow.icnSeq.head.node.axiDevParams.get.wrapper.toLowerCase}"))

  private val ccnIcnSeq = ring.icnCcs.get
  private val ccnSocketSeq = ccnIcnSeq.map(icn => placeSocket(icn, Some(icn.node.domainId)))

  val io = IO(new Bundle {
    val ci = Input(UInt(ciIdBits.W))
    val onReset = Output(Bool())
    val dft = Input(new DftWires)
    val ramctl = Input(new SramCtrlBundle)
    val resetBypass = Output(AsyncReset())
    val intr = Option.when(p(HardwareAssertionKey).enable)(Output(Bool()))
  })
  io.resetBypass := ResetGen(2, Some(io.dft.reset))
  dft := io.dft
  ramctl := io.ramctl
  private val mnIow = ioWrps.filter(_.io.onReset.isDefined).head
  val ddrDrv = ioWrps.flatMap(_.memAxiPorts)
  val cfgDrv = ioWrps.flatMap(_.cfgAxiPorts)
  val dmaDrv = ioWrps.flatMap(_.dmaAxiPorts)
  val ccnDrv = ccnSocketSeq.map(_.io.socket)
  val hwaDrv = mnIow.hwaAxiPort
  runIOAutomation()
  io.onReset := mnIow.io.onReset.get
  ring.io_ci := io.ci
  io.intr.foreach(_ := mnIow.io.intr.get)
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

  private def attrStr(attr:String, idx:Int, size:Int):String = {
    if(attr != "") {
      s"_$attr"
    } else if(size == 1) {
      s""
    } else {
      s"_$idx"
    }
  }

  def runIOAutomation(): Unit = {
    ddrIO.zip(ddrDrv).zipWithIndex.foreach({ case ((a, b), i) =>
      val attr = attrStr(b.params.attr, i, ddrIO.size)
      a.suggestName(s"m_axi_mem$attr")
      a <> b
      dontTouch(a)
      dontTouch(b)
    })
    cfgIO.zip(cfgDrv).zipWithIndex.foreach({ case ((a, b), i) =>
      val attr = attrStr(b.params.attr, i, cfgIO.size)
      a.suggestName(s"m_axi_cfg$attr")
      a <> b
      dontTouch(a)
      dontTouch(b)
    })
    dmaIO.zip(dmaDrv).zipWithIndex.foreach({ case ((a, b), i) =>
      val attr = attrStr(b.params.attr, i, dmaIO.size)
      a.suggestName(s"s_axi$attr")
      b <> a
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
      b <> a
      dontTouch(a)
      dontTouch(b)
    })
  }
}