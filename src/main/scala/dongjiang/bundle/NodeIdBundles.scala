package dongjiang.bundle

import chisel3._
import chisel3.util._
import dongjiang._
import org.chipsalliance.cde.config._
import xs.utils.debug.HardwareAssertion

object Net {
  val LOCAL = 0.U
  val CSN = 1.U
}

class NodeIdBundle(implicit p: Parameters) extends DJBundle {
  val net = UInt(nodeNetBits.W)
  val nid = UInt(nodeNidBits.W)
  val aid = UInt(nodeAidBits.W)
  def chip = aid

  def isLocal : Bool = net === Net.LOCAL
  def isCSN   : Bool = net === Net.CSN

  def isCc    : Bool = isLocal & fromXNode(nid, ccNodeIdSeq)
  def isRni   : Bool = isLocal & fromXNode(nid, rniNodeIdSeq)
  def isSn    : Bool = isLocal & fromXNode(nid, snNodeIdSeq)
  def isC2C   : Bool = isCSN   & fromXNode(nid, c2cNodeIdSeq)
  def isHnx   : Bool = isCSN   & fromXNode(nid, hnxNodeIdSeq)
  def isRn    : Bool = isCc    | isRni

  def getMetaId(enAst: Bool = true.B): UInt = {
    HardwareAssertion.withEn(isCc | isC2C, enAst)
    val metaId = WireInit(0.U(metaIdBits.W))
    when(isLocal) {
      ccNodeIdSeq.zipWithIndex.foreach {
        case (ccId, i) =>
          when(getNID(ccId.U) === nid) {
            metaId := i.U
          }
      }
    }.otherwise {
      metaId := chip
    }
    metaId
  }


  def setByMetaId(metaId: UInt, enAst: Bool = true.B): Unit = {
    require(metaId.getWidth == metaIdBits)
    // Net
    val _net = metaId >= nrCcNode.U
    // Local
    val _nid_local = UInt(nodeNidBits.W)
    ccNodeIdSeq.zipWithIndex.foreach {
      case(ccNodeId, i) =>
        when(i.U === metaId) {
          _nid_local := getNID(ccNodeId.U)
        }
    }
    val _aid_local = 1.U
    // CSN
    val chipId = metaId - nrCcNode.U
    val _aid_csn = chipId
    // Set value
    this.net := _net
    this.nid := Mux(_net === Net.LOCAL, _nid_local, DontCare) // nid will be remap in CSN ICN
    this.aid := Mux(_net === Net.LOCAL, _aid_local, _aid_csn)
  }


  def getLocalDirect(friendsVec: Seq[Seq[UInt]], enAst: Bool = true.B): UInt = {
    val directVec = Wire(Vec(friendsVec.length, Bool()))
    friendsVec.zip(directVec).foreach {
      case(f, d) =>
        d := f.map { case fid => getNID(fid) === nid }.reduce(_ | _)
        HardwareAssertion.withEn(PopCount(f.map(_ === nid)) <= 1.U, enAst)
    }
    HardwareAssertion.withEn(PopCount(directVec) === 1.U, enAst)
    PriorityEncoder(directVec)
  }
}
