package zhujiang.device.io

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.ResetGen
import xs.utils.debug.HardwareAssertionKey
import xs.utils.dft.BaseTestBundle
import xs.utils.mbist.MbistInterface
import xs.utils.sram.{SramCtrlBundle, SramHelper}
import zhujiang.axi._
import zhujiang.device.AxiDeviceParams
import zhujiang.device.bridge.axi.AxiBridge
import zhujiang.device.bridge.axilite.AxiLiteBridge
import zhujiang.device.dma.Axi2Chi
import zhujiang.device.misc.MiscDevice
import zhujiang.utils.DoubleCounterClockGate
import zhujiang.{ZJModule, ZhujiangGlobal}

class IoWrapper(nodes: Seq[Node])(implicit p:Parameters) extends ZJModule {
  private val nts = nodes.map(_.nodeType)
  nts.foreach(nt => require(NodeType.testAxiDev(nt), s"illegal node type ${NodeType.strMap(nt)}"))
  override val desiredName = s"IoWrapper${nodes.head.axiDevParams.get.wrapper.capitalize}"
  val io = IO(new Bundle{
    val dft = Input(new BaseTestBundle)
    val ramctl = Input(new SramCtrlBundle)
    val onReset = Option.when(nodes.exists(_.nodeType == NodeType.M))(Output(Bool()))
    val intr = Option.when(nodes.exists(_.nodeType == NodeType.M) && p(HardwareAssertionKey).enable)(Output(Bool()))
  })
  val icnSeq = nodes.map(n => IO(new DeviceIcnBundle(n, true)))
  icnSeq.foreach(icn => icn.suggestName(s"dev_0x${icn.node.nodeId.toHexString}"))
  println(s"$desiredName:{")
  icnSeq.foreach(icn => println(s"  ${NodeType.strMap(icn.node.nodeType)}:${icn.node.deviceName}"))
  println("}")
  private val mbistReq = WireInit(false.B)

  private def placeResetSync(name: String, dev: DeviceIcnBundle): AsyncReset = {
    val mst = dev.node.nodeType == NodeType.RI || dev.node.nodeType == NodeType.RH
    val rstGen = Module(new ResetGen)
    rstGen.suggestName(name + "_rst_sync")
    rstGen.dft := io.dft.toResetDftBundle
    if(mst) rstGen.reset := dev.resetState.get(0).asAsyncReset
    else rstGen.reset := dev.resetState.get(1).asAsyncReset
    rstGen.o_reset
  }
  private def attrStrGen(params: AxiDeviceParams, size:Int, idx:Int):String = {
    val attr = params.attr
    val wrp = params.wrapper
    val attrStr = if(attr != "") {
      s"$attr"
    } else if (size == 1) {
      s"$wrp"
    } else {
      s"${wrp}_$idx"
    }
    attrStr
  }

  private val icnSns = icnSeq.filter(_.node.nodeType == NodeType.S)
  private val memDevSeq = for(icn <- icnSns) yield {
    val bridge = Module(new AxiBridge(icn.node))
    ZhujiangGlobal.addAxiDev(bridge, icn.node)
    bridge.reset := placeResetSync(icn.node.deviceName, icn)
    bridge.icn.rx <> icn.rx
    bridge.icn.tx <> icn.tx
    bridge
  }
  memDevSeq.foreach(d => d.suggestName(d.icn.node.deviceName))
  val (memAxiPorts, memAxiBufs) = memDevSeq.zipWithIndex.map({case(dev, idx) =>
    val attrStr = attrStrGen(dev.icn.node.axiDevParams.get, memDevSeq.size, idx)
    val buf = Module(new AxiBufferChain(dev.axi.params.copy(attr = attrStr), dev.icn.node.axiDevParams.get.buffers))
    val icg = Module(new DoubleCounterClockGate)
    val iop = IO(new AxiBundle(buf.io.out.params.copy(attr = attrStr)))
    icg.reset := dev.reset
    icg.io.te := io.dft.cgen
    icg.io.inbound := dev.icn.rx.req.map(_.valid).getOrElse(false.B) | mbistReq
    icg.io.working := dev.working
    dev.clock := icg.io.ock
    buf.reset := dev.reset
    buf.io.in <> dev.axi
    iop <> buf.io.out
    (iop, buf)
  }).unzip
  memAxiPorts.foreach(ap => ap.suggestName(s"m_axi_mem_${ap.params.attr}"))
  memAxiBufs.foreach(bc => bc.suggestName(s"m_mem_buf_chain_${bc.io.in.params.attr}"))

  private val icnHis = icnSeq.filter(_.node.nodeType == NodeType.HI)
  private val cfgDevSeq = for(icn <- icnHis) yield {
    val bridge = Module(new AxiLiteBridge(icn.node, 64, 3))
    bridge.reset := placeResetSync(icn.node.deviceName, icn)
    bridge.icn.rx <> icn.rx
    bridge.icn.tx <> icn.tx
    bridge.nodeId := icn.node.nodeId.U
    bridge
  }
  cfgDevSeq.foreach(d => d.suggestName(d.icn.node.deviceName))
  val (cfgAxiPorts, cfgAxiBufs) = cfgDevSeq.zipWithIndex.map({case(dev, idx) =>
    val attrStr = attrStrGen(dev.icn.node.axiDevParams.get, cfgDevSeq.size, idx)
    val buf = Module(new AxiBufferChain(dev.axi.params.copy(attr = attrStr), dev.icn.node.axiDevParams.get.buffers))
    val icg = Module(new DoubleCounterClockGate)
    val iop = IO(new AxiBundle(buf.io.out.params.copy(attr = attrStr)))
    icg.reset := dev.reset
    icg.io.te := io.dft.cgen
    icg.io.inbound := dev.icn.rx.req.map(_.valid).getOrElse(false.B) | mbistReq
    icg.io.working := dev.working
    dev.clock := icg.io.ock
    buf.reset := dev.reset
    buf.io.in <> dev.axi
    iop <> buf.io.out
    (iop, buf)
  }).unzip
  cfgAxiPorts.foreach(ap => ap.suggestName(s"m_axi_cfg_${ap.params.attr}"))
  cfgAxiBufs.foreach(bc => bc.suggestName(s"m_cfg_buf_chain_${bc.io.in.params.attr}"))

  private val dmaIcnSeq = icnSeq.filter(icn => icn.node.nodeType == NodeType.RI || icn.node.nodeType == NodeType.RH)
  private val dmaDevSeq = for(icn <- dmaIcnSeq ) yield {
    val bridge = Module(new Axi2Chi(icn.node))
    ZhujiangGlobal.addAxiDev(bridge, icn.node)
    bridge.icn.rx <> icn.rx
    bridge.icn.tx <> icn.tx
    bridge.reset := placeResetSync(icn.node.deviceName, icn)
    bridge
  }
  dmaDevSeq.foreach(d => d.suggestName(d.icn.node.deviceName))
  val (dmaAxiPorts, dmaAxiBufs) = dmaDevSeq.zipWithIndex.map({ case (dev, idx) =>
    val attrStr = attrStrGen(dev.icn.node.axiDevParams.get, dmaDevSeq.size, idx)
    val buf = Module(new AxiBufferChain(dev.axi.params.copy(attr = attrStr), dev.icn.node.axiDevParams.get.buffers))
    val icg = Module(new DoubleCounterClockGate)
    val iop = IO(Flipped(new AxiBundle(buf.io.out.params.copy(attr = attrStr))))
    icg.reset := dev.reset
    icg.io.te := io.dft.cgen
    icg.io.inbound := buf.io.out.ar.valid | buf.io.out.aw.valid | buf.io.out.w.valid | mbistReq
    icg.io.working := dev.working
    dev.clock := icg.io.ock
    buf.reset := dev.reset
    dev.axi <> buf.io.out
    buf.io.in <> iop
    (iop, buf)
  }).unzip
  dmaAxiPorts.foreach(ap => ap.suggestName(s"s_axi_${ap.params.attr}"))
  dmaAxiBufs.foreach(bc => bc.suggestName(s"s_buf_chain_${bc.io.in.params.attr}"))

  private val mnIcn = icnSeq.filter(_.node.nodeType == NodeType.M)
  require(mnIcn.size <= 1)
  private val mnDev = Option.when(mnIcn.nonEmpty)(Module(new MiscDevice(mnIcn.head.node)))
  mnDev.foreach(mn => {
    mn.io.icn <> mnIcn.head
    mn.clock := clock
    mn.reset := ResetGen(2, Some(io.dft.toResetDftBundle))
    mn.io.dft := io.dft.toResetDftBundle
    io.onReset.get := mn.io.onReset
    mn.suggestName(mnIcn.head.node.deviceName)
  })
  val hwaAxiPort = if(mnDev.isDefined && mnDev.get.io.axi.isDefined) {
    val icn = mnDev.get.io.icn
    val axi = mnDev.get.io.axi.get
    val io = IO(Flipped(new AxiBundle(axi.params)))
    val buf = Module(new AxiBufferChain(axi.params, icn.node.axiDevParams.get.buffers))
    buf.suggestName(s"hwa_buf_chain")
    axi <> buf.io.out
    buf.io.in <> io
    Some(io)
  } else {
    None
  }
  hwaAxiPort.foreach(_.suggestName(s"s_axi_hwa"))
  if(io.intr.isDefined) {
    io.intr.get := mnDev.get.io.intr.get
  }

  private val hasS = nts.contains(NodeType.S)
  private val hasRI = nts.contains(NodeType.RI)
  private val hasRH = nts.contains(NodeType.RH)
  private val hasM = nts.contains(NodeType.M) && p(HardwareAssertionKey).enable
  if(hasS || hasRI || hasRH || hasM) {
    val mbistIntf = MbistInterface(desiredName, io.dft, hasMbist)
    SramHelper.genSramCtrlBundleTop() := io.ramctl
    mbistReq := mbistIntf.map(_.toPipeline.head.req).getOrElse(false.B)
  }
}
