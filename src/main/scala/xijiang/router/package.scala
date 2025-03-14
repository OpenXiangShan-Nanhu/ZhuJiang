package xijiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.BaseRouter
import zhujiang.ZJParametersKey
import zhujiang.chi._

package object router {
  class RnRouter(node: Node)(implicit p: Parameters) extends BaseRouter(node) {
    require(node.nodeType == NodeType.CC || node.nodeType == NodeType.RF || node.nodeType == NodeType.RI)
    private val injectFlit = icn.rx.req.get.bits.asTypeOf(new RReqFlit)
    private val addr = injectFlit.Addr.asTypeOf(new ReqAddrBundle)
    private val memAttr = injectFlit.MemAttr.asTypeOf(new MemAttr)

    private val defaultHni = p(ZJParametersKey).island.filter(r => NodeType.HI == r.nodeType && r.defaultHni).head
    private val possibleCompleterTypes = Seq(NodeType.CC, NodeType.HI)
    private val possibleCompleters = p(ZJParametersKey).island.filter(r => possibleCompleterTypes.contains(r.nodeType) && !r.defaultHni) ++ node.friends // friends is HF Nodes
    private val completerSelOH = possibleCompleters.map(_.isReqCompleter(addr, router.ci, memAttr, zjParams.hnxBankOff))
    private val completerId = possibleCompleters.map(_.nodeId.U(niw.W))
    private val reqTarget = Mux(Cat(completerSelOH).orR, Mux1H(completerSelOH, completerId), defaultHni.nodeId.U)
    assert(PopCount(completerSelOH) <= 1.U)
    if(p(ZJParametersKey).tfsParams.isEmpty) injectsMap("REQ").bits.tgt := reqTarget
  }
}
