package xijiang

import chisel3._
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.IcnBundle
import xijiang.tfs._
import xs.utils.{DFTResetSignals, ResetGen}
import zhujiang.device.misc.ResetDevice
import zhujiang.{ZJModule, ZJParametersKey}

class Ring(implicit p: Parameters) extends ZJModule {
  private val tfs = p(ZJParametersKey).tfsParams.isDefined
  override val desiredName = zjParams.ciName
  val io_ci = IO(Input(UInt(ciIdBits.W)))
  val dfx_reset = IO(Input(new DFTResetSignals))
  val reset_state = if(tfs) Some(IO(Output(Bool()))) else None

  dontTouch(io_ci)
  println("Local Ring Node Info {")
  private val ring = p(ZJParametersKey).island
  private val routersAndNodes = ring.map(n => (n.genRouter(p), n))
  for(((r, _), i) <- routersAndNodes.zipWithIndex) {
    val left = if(i == 0) routersAndNodes.last else routersAndNodes(i - 1)
    val right = if(i == routersAndNodes.size - 1) routersAndNodes.head else routersAndNodes(i + 1)
    r.router.rings.head.rx := left._1.router.rings.head.tx
    r.router.rings.last.rx := right._1.router.rings.last.tx
    r.router.ci := io_ci
    r.router.reset.rx := left._1.router.reset.tx
    r.reset := withClockAndReset(clock, r.icn.resetState.get(1).asAsyncReset){ResetGen(dft = Some(dfx_reset))}
  }
  println("\n}\n")

  private val chiRoutersAndNodes = routersAndNodes.filterNot(_._2.nodeType == NodeType.P)
  private val icnsWithNodes = for((r, n) <- chiRoutersAndNodes) yield {
    if(tfs) {
      val m = Module(new TrafficGen(n))
      m.icn <> r.icn
      m.nodeId := r.router.nodeId
      m.suggestName(s"${n.routerStr}TrafficGen")
      if(n.nodeType == NodeType.M) {
        val resetDev = Module(new ResetDevice)
        resetDev.clock := clock
        resetDev.reset := reset
        r.icn.resetInject.get := resetDev.io.resetInject
        resetDev.io.resetState := r.icn.resetState.get
        reset_state.get := resetDev.io.onReset
      }
      (None, n)
    } else {
      val port = IO(new IcnBundle(n, true)(p))
      port.suggestName(n.icnStr)
      port <> r.icn
      (Some(port), n)
    }
  }

  val icnCcs = if(!tfs) Some(icnsWithNodes.filter(_._2.nodeType == NodeType.CC).map(_._1.get)) else None
  val icnRfs = if(!tfs) Some(icnsWithNodes.filter(_._2.nodeType == NodeType.RF).map(_._1.get)) else None
  val icnRis = if(!tfs) Some(icnsWithNodes.filter(_._2.nodeType == NodeType.RI).map(_._1.get)) else None
  val icnHfs = if(!tfs) Some(icnsWithNodes.filter(_._2.nodeType == NodeType.HF).map(_._1.get)) else None
  val icnHis = if(!tfs) Some(icnsWithNodes.filter(_._2.nodeType == NodeType.HI).map(_._1.get)) else None
  val icnSns = if(!tfs) Some(icnsWithNodes.filter(_._2.nodeType == NodeType.S).map(_._1.get)) else None
  val icnMns = if(!tfs) Some(icnsWithNodes.filter(_._2.nodeType == NodeType.M).map(_._1.get)) else None

  private val functionalRouters = routersAndNodes.filter(_._2.nodeType != NodeType.P).map(_._1)
  for(i <- functionalRouters.indices) {
    for(j <- (i + 1) until functionalRouters.size) {
      require(functionalRouters(i).node.nodeId != functionalRouters(j).node.nodeId)
    }
  }
}
