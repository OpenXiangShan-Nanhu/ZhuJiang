package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import xs.utils.debug.HardwareAssertion

trait HasNodeId { this: DJBundle =>
  val isLAN     = Bool()
  val nodeId    = UInt(nodeIdBits.W)

  def fromBBN   = !isLAN

  // LAN
  def lanNId    = nodeId(nodeIdBits-1, nodeIdBits-lanNBits)
  def lanAId    = nodeId(lanABits-1 , 0)
  // BBN
  def bbnCI     = nodeId(nodeIdBits - 1, nodeIdBits - bbnIBits)
  def bbnBId    = nodeId(bbnBBits - 1, 0)

  // LAN NID
  def fromCc :Bool = isLAN & fromLanXNode(nodeId, ccNodeIdSeq)
  def fromRni:Bool = isLAN & fromLanXNode(nodeId, rniNodeIdSeq)
  def fromSn :Bool = isLAN & fromLanXNode(nodeId, snNodeIdSeq)
  def fromRn :Bool = fromCc  | fromRni

  // LAN AID
  // AID: 0 -> CHI2TL/DMA; 1 -> L2; 2 -> TL2CHI
  def fromCcRnf:  Bool = fromCc  & lanAId === 1.U
  def fromCcRni:  Bool = fromCc  & lanAId === 2.U
  def fromRniDma: Bool = fromRni & lanAId === 0.U

  // metaId
  def metaId: UInt = {
    val metaId = WireInit(0.U(metaIdBits.W))
    when(isLAN) {
      ccNodeIdSeq.zipWithIndex.foreach {
        case (ccId, i) =>
          when(ccId.U >> lanABits === lanNId) {
            metaId := i.U
          }
      }
    }.otherwise {
      metaId := bbnCI
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
    this.isLAN := _fromLAN
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
