package xijiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.BaseRouter
import zhujiang.ZJParametersKey
import zhujiang.chi._

package object router {
  class RxReqRouter(node: Node)(implicit p: Parameters) extends BaseRouter(node) {
    require(node.nodeType == NodeType.CC || node.nodeType == NodeType.RF || node.nodeType == NodeType.RI)
    private val injectAddr = icn.rx.req.get.bits.asTypeOf(new ReqFlit).Addr.asTypeOf(new ReqAddrBundle)
    private val reqTarget = if(node.csnNode) {
      val csnReqTarget = Wire(new NodeIdBundle)
      csnReqTarget.net := true.B
      csnReqTarget.nid := DontCare
      csnReqTarget.aid := injectAddr.chip
      csnReqTarget.asUInt
    } else {
      val defaultHni = p(ZJParametersKey).localRing.filter(r => NodeType.HI == r.nodeType && r.defaultHni).head
      val possibleCompleterTypes = Seq(NodeType.CC, NodeType.HF, NodeType.HI)
      val possibleCompleters = p(ZJParametersKey).localRing.filter(r => possibleCompleterTypes.contains(r.nodeType) && !r.defaultHni)
      val completerSelOH = possibleCompleters.map(_.isReqCompleter(injectAddr, router.chip))
      val completerId = possibleCompleters.map(_.nodeId.U(niw.W))
      val localReqTarget = Mux(Cat(completerSelOH).orR, Mux1H(completerSelOH, completerId), defaultHni.nodeId.U)
      localReqTarget
    }
    if(p(ZJParametersKey).tfsParams.isEmpty) {
      injectsMap("REQ").bits.tgt := reqTarget
    }
  }

  class ChipToChipRouter(node: Node)(implicit p: Parameters) extends BaseRouter(node) {
    require(node.csnNode && node.nodeType == NodeType.C)
    private val csnHfs = p(ZJParametersKey).csnRing.filter(_.nodeType == NodeType.HF)
    private val csnRfs = p(ZJParametersKey).csnRing.filter(_.nodeType == NodeType.RF)
    if(p(ZJParametersKey).tfsParams.isEmpty) {
      val reqTgt = WireInit(icn.rx.req.get.bits.asTypeOf(new ReqFlit).TgtID.asTypeOf(new NodeIdBundle))
      if(csnHfs.length > 1) {
        val reqAddr = icn.rx.req.get.bits.asTypeOf(new ReqFlit).Addr.asTypeOf(new ReqAddrBundle)
        val hfSelOH = csnHfs.map(chf => reqAddr.checkBank(chf.bankBits, chf.bankId.U))
        val hfNids = csnHfs.map(_.nodeId.U.asTypeOf(new NodeIdBundle).nid)
        when(icn.rx.req.get.valid) {
          assert(PopCount(hfSelOH) === 1.U)
        }
        reqTgt.nid := Mux1H(hfSelOH, hfNids)
      } else {
        reqTgt.nid := 0.U
      }

      val snoopTgt = WireInit(icn.rx.snoop.get.bits.asTypeOf(new SnoopFlit).TgtID.asTypeOf(new NodeIdBundle))
      if(csnRfs.length > 1) {
        val snpAddr = icn.rx.snoop.get.bits.asTypeOf(new SnoopFlit).Addr.asTypeOf(new SnpAddrBundle)
        val rfSelOH = csnRfs.map(crf => snpAddr.checkBank(crf.bankBits, crf.bankId.U))
        val rfNids = csnRfs.map(_.nodeId.U.asTypeOf(new NodeIdBundle).nid)
        when(icn.rx.snoop.get.valid) {
          assert(PopCount(rfSelOH) === 1.U)
        }
        snoopTgt.nid := Mux1H(rfSelOH, rfNids)
      } else {
        snoopTgt.nid := 0.U
      }

      injectsMap("REQ").bits.tgt := reqTgt.asUInt
      injectsMap("SNP").bits.tgt := snoopTgt.asUInt
    }
  }
}
