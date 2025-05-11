package xijiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.{BaseRouter, RouterHelper}
import zhujiang.ZJParametersKey
import zhujiang.chi._

package object router {
  class RnRouter(node: Node)(implicit p: Parameters) extends BaseRouter(node) {
    require(node.nodeType == NodeType.CC || node.nodeType == NodeType.RF || node.nodeType == NodeType.RI || node.nodeType == NodeType.RH)
    private val rxreq = if(node.nodeType == NodeType.RH) {
      icn.rx.hpr.get
    } else {
      icn.rx.req.get
    }
    private val injectFlit = rxreq.bits.asTypeOf(new RReqFlit)
    private val addr = injectFlit.Addr.asTypeOf(new ReqAddrBundle)
    private val memAttr = injectFlit.MemAttr.asTypeOf(new MemAttr)

    private val defaultHni = p(ZJParametersKey).island.filter(r => NodeType.HI == r.nodeType && r.defaultHni).head
    private val possibleCompleterTypes = Seq(NodeType.CC, NodeType.HI)
    private val possibleCompleters = p(ZJParametersKey).island.filter(r => possibleCompleterTypes.contains(r.nodeType) && !r.defaultHni) ++ node.friends // friends is HF Nodes
    private val completerSelOH = possibleCompleters.map(_.isReqCompleter(addr, router.ci, memAttr, zjParams.hnxBankOff))
    private val completerId = possibleCompleters.map(_.nodeId.U(niw.W))
    private val reqTarget = Mux(Cat(completerSelOH).orR, Mux1H(completerSelOH, completerId), defaultHni.nodeId.U)
    when(rxreq.valid) {
      assert(PopCount(completerSelOH) <= 1.U)
    }
    if(p(ZJParametersKey).tfsParams.isEmpty) {
      if(RouterHelper.testRingRx(node, "HPR")) ringInjectsMap("HPR").bits.tgt := reqTarget
      if(RouterHelper.testRingRx(node, "REQ")) ringInjectsMap("REQ").bits.tgt := reqTarget
    }
  }
}
