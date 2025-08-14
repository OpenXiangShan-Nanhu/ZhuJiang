package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import xs.utils.debug._

trait HasNodeId { this: DJBundle =>
  val fromLAN   = Bool()
  val nodeId    = UInt(nodeIdBits.W)

  def fromBBN   = !fromLAN

  // LAN
  def lanNId    = nodeId(nodeIdBits-1, nodeIdBits-lanNBits)
  def lanAId    = nodeId(lanABits-1 , 0)
  // BBN
  def bbnCI     = nodeId(nodeIdBits - 1, nodeIdBits - bbnIBits)
  def bbnBId    = nodeId(bbnBBits - 1, 0)

  // LAN NID
  def fromCc :Bool = fromLAN & fromLanXNode(nodeId, ccNodeIdSeq)
  def fromRni:Bool = fromLAN & fromLanXNode(nodeId, rniNodeIdSeq)
  def fromSn :Bool = fromLAN & fromLanXNode(nodeId, snNodeIdSeq)
  def fromRn :Bool = fromCc  | fromRni

  // LAN AID
  // AID: 1 -> CHI2TL/DMA; 1 -> L2; 1 -> TL2CHI
  def fromCcRnf:  Bool = fromCc  & lanAId === 1.U
  def fromCcRni:  Bool = fromCc  & lanAId === 1.U
  def fromRniDma: Bool = fromRni & lanAId === 1.U

  // metaIdOH
  def metaIdOH: UInt = {
    val idxOH = WireInit(0.U(nrSfMetas.W))
    when(fromLAN) {
      ccNodeIdSeq.zipWithIndex.foreach {
        case (ccId, i) =>
          when(ccId.U >> lanABits === lanNId) {
            idxOH := UIntToOH(i.U)
          }
      }
    }.otherwise {
      idxOH := UIntToOH(bbnCI + nrCcNode.U)
    }
    idxOH
  }

  // setNodeId
  def setSnpNodeId(metaId: UInt): Unit = {
    if(nrSfMetas > 1) {
      require(metaId.getWidth == metaIdBits, s"${metaId.getWidth} =/= $metaIdBits")
      // fromLAN
      val _fromLAN = metaId < nrCcNode.U
      // LAN
      val _lan_nodeId = WireInit(0.U(nodeIdBits.W))
      ccNodeIdSeq.zipWithIndex.foreach {
        case (ccNodeId, i) =>
          when(i.U === metaId) {
            _lan_nodeId := ccNodeId.U | 1.U // RNF agentId always be 1
          }
      }
      // BBN
      val _bbn_nodeId = WireInit(0.U(nodeIdBits.W))
      _bbn_nodeId := (metaId - nrCcNode.U) << bbnBBits // bbNBId will be remap in BBN Router // TODO: Consider the missing self
      // Set value
      this.fromLAN := _fromLAN
      this.nodeId := Mux(_fromLAN, _lan_nodeId, _bbn_nodeId)
    } else {
      this.fromLAN := true.B
      this.nodeId := ccNodeIdSeq.head.U | 1.U
    }
  }
}

class NodeId(implicit p: Parameters) extends DJBundle with HasNodeId
