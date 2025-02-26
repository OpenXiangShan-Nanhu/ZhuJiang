package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import xs.utils.debug.HardwareAssertion

trait HasNodeId extends DJBundle { this: Bundle =>
  val fromLAN   = Bool()
  val nodeId    = UInt(nodeIdBits.W)

  def fromBBN   = !fromLAN

  // LAN
  def lanNId    = nodeId(nodeIdBits-1, nodeIdBits-lanNBits)
  def lanAId    = nodeId(lanABits-1 , 0)
  // BBN
  def bbnIId    = nodeId(nodeIdBits - 1, nodeIdBits - bbnIBits)
  def bbnBId    = nodeId(bbnBBits - 1, 0)

  // LAN
  def isCc :Bool = fromLAN & fromLanXNode(nodeId, ccNodeIdSeq)
  def isRni:Bool = fromLAN & fromLanXNode(nodeId, rniNodeIdSeq)
  def isSn :Bool = fromLAN & fromLanXNode(nodeId, snNodeIdSeq)
  def isRn :Bool = isCc    | isRni

  // metaId
  def metaId: UInt = {
    val metaId = WireInit(0.U(metaIdBits.W))
    when(fromLAN) {
      ccNodeIdSeq.zipWithIndex.foreach {
        case (ccId, i) =>
          when(ccId.U >> lanABits === lanNId) {
            metaId := i.U
          }
      }
    }.otherwise {
      metaId := bbnIId
    }
    metaId
  }

  // setNodeId
  def setNodeId(metaId: UInt): Unit = {
    require(metaId.getWidth == metaIdBits)
    // fromLAN
    val _fromLAN = metaId < nrCcNode.U
    // LAN
    val _lan_nodeId = WireInit(0.U(nodeIdBits.W))
    ccNodeIdSeq.zipWithIndex.foreach {
      case(ccNodeId, i) =>
        when(i.U === metaId) {
          _lan_nodeId := ccNodeId.U | 1.U // RNF agentId always be 1
        }
    }
    // BBN
    val _bbn_nodeId = WireInit(0.U(nodeIdBits.W))
    _bbn_nodeId := (metaId - nrCcNode.U) << bbnBBits  // bbNBId will be remap in BBN Router
    // Set value
    this.fromLAN := _fromLAN
    this.nodeId  := Mux(_fromLAN, _lan_nodeId, _bbn_nodeId)
  }

  // getLanDirect
  def getLanDirect(friendsVec: Seq[Seq[UInt]], enAst: Bool = true.B): UInt = {
    val directVec = Wire(Vec(friendsVec.length, Bool()))
    friendsVec.zip(directVec).foreach {
      case(f, d) =>
        d := f.map { case fid => fid >> lanABits === lanNId }.reduce(_ | _)
        HardwareAssertion.withEn(PopCount(f.map(_ >> lanABits === lanNId)) <= 1.U, enAst)
    }
    HardwareAssertion.withEn(PopCount(directVec) === 1.U, enAst)
    PriorityEncoder(directVec)
  }
}

class NodeId(implicit p: Parameters) extends DJBundle with HasNodeId
