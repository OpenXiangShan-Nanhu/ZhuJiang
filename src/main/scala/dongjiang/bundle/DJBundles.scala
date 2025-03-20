package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import zhujiang.chi._

/*
 * NoC Type
 */
object NocType {
  val LAN = 1.U
  val BBN = 0.U

  def setRx(filt: ReqFlit,   t: Int) = filt.TgtID := t.U << filt.TgtID.getWidth
  def setRx(filt: SnoopFlit, t: Int) = filt.TgtID := t.U
  def setRx(filt: RespFlit,  t: Int) = filt.TgtID := t.U
  def setRx(filt: DataFlit,  t: Int) = filt.TgtID := t.U

  def rxIs(filt: ReqFlit,    t: Int) = filt.TgtID === t.U
  def rxIs(filt: SnoopFlit,  t: Int) = filt.TgtID === t.U
  def rxIs(filt: RespFlit,   t: Int) = filt.TgtID === t.U
  def rxIs(filt: DataFlit,   t: Int) = filt.TgtID === t.U

  def setTx(filt: ReqFlit,   t: Int) = filt.SrcID := t.U
  def setTx(filt: SnoopFlit, t: Int) = filt.SrcID := t.U
  def setTx(filt: RespFlit,  t: Int) = filt.SrcID := t.U
  def setTx(filt: DataFlit,  t: Int) = filt.SrcID := t.U

  def txIs(filt: ReqFlit,    t: Int) = filt.SrcID === t.U
  def txIs(filt: SnoopFlit,  t: Int) = filt.SrcID === t.U
  def txIs(filt: RespFlit,   t: Int) = filt.SrcID === t.U
  def txIs(filt: DataFlit,   t: Int) = filt.SrcID === t.U
}


/*
 * Addr
 */
trait HasAddr extends DJBundle { this: DJBundle =>
  val addr      = UInt(addrBits.W)

  def ci        = getCI(addr)
  def useAddr   = getUseAddr(addr)
  def bankId    = getBankId(addr)
  def offset    = getOffset(addr)
  def dirBank   = getDirBank(addr)
  def llcTag    = getLlcTag(addr)
  def llcSet    = getLlcSet(addr)
  def sfTag     = getSfTag(addr)
  def sfSet     = getSfSet(addr)
  def posTag    = getPosTag(addr)
  def posSet    = getPosSet(addr)

  def toLAN(selfCI: UInt) = ci === selfCI
  def toBBN(selfCI: UInt) = ci =/= selfCI

  def catByX(bank: UInt, tag: UInt, tagBits: Int, set: UInt, setBits: Int, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = {
    require(bank.getWidth    == bankBits,    s"bankBits:    ${bank.getWidth} =/= ${bankBits}")
    require(tag.getWidth     == tagBits,     s"tagBits:     ${tag.getWidth} =/= ${tagBits}")
    require(set.getWidth     == setBits,     s"setBits:     ${set.getWidth} =/= ${setBits}")
    require(dirBank.getWidth == dirBankBits, s"dirBankBits: ${dirBank.getWidth} =/= ${dirBankBits}")
    require(offset.getWidth  == offsetBits,  s"offsetBits:  ${offset.getWidth} =/= ${offsetBits}")
    val useAddr_ = Cat(tag, set, dirBank)
    val addr_ = Cat(useAddr_(useAddrBits-1, bankId_lo-offsetBits), bank, useAddr_(bankId_lo-offsetBits-1, 0), offset)
    require(addr_.getWidth   == addrBits)
    this.addr := addr_
  }
  def catPoS(bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, posTagBits, set, posSetBits, dirBank, offset)
  def catLLC(bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, llcTagBits, set, llcSetBits, dirBank, offset)
  def catSF (bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, sfTagBits,  set, sfSetBits,  dirBank, offset)

  def addrType: String = "llc"
  def cat(bank: UInt, tag: UInt, set: UInt, dirBank: UInt): Unit = if(addrType == "llc") catLLC(bank, tag, set, dirBank) else catSF(bank, tag, set, dirBank)
  def tag: UInt = if(addrType == "llc") llcTag else sfTag
  def set: UInt = if(addrType == "llc") llcSet else sfSet

}

class Addr(dirType: String = "llc")(implicit p: Parameters) extends DJBundle with HasAddr {
  override def addrType: String = dirType
}

/*
 * PoS
 */
class PosIndex(implicit p: Parameters) extends DJBundle {
  val set = UInt(posSetBits.W)
  val way = UInt(posWayBits.W)

  def idxMatch   (i: Int, j: Int) = set === i.U & way === j.U
  def getLLCTxnID(dirBank: Int)   = Cat(dirBank.U, set, way)
  def getLLCTxnID(dirBank: UInt)  = Cat(dirBank, set, way)
}

trait HasPosIndex extends DJBundle { this: DJBundle =>
  val pos = new PosIndex()
}

trait HasLLCTxnID extends DJBundle { this: DJBundle =>
  val pos       = new PosIndex()
  val dirBank   = UInt(dirBankBits.W)
  def llcTxnID  = pos.getLLCTxnID(dirBank)
}

/*
 * ChiTask
 */
class ChiTask(implicit p: Parameters) extends DJBundle with HasNodeId with HasChiChannel
  with HasChiOp with HasChiOrderAndExpCompAck with HasChiSnpField with HasChiSize  {
  // REQ
  val txnID       = UInt(ChiTxnIdBits.W)
  val memAttr     = new MemAttr()
  // SNP
  val fwdNID      = UInt(nodeIdBits.W)
  val fwdTxnID    = UInt(ChiFwdTxnIdBits.W)
  val retToSrc    = Bool()
}