package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import zhujiang.chi._
import dongjiang.frontend.decode._

/*
 * NoC Type
 */
object NocType {
  val LAN = 1.U
  val BBN = 0.U

  def setRx(filt: Flit, t: Int) : Unit = filt.tgt := t.U
  def setRx(filt: Flit, t: UInt): Unit = filt.tgt := t
  def rxIs (filt: Flit, t: Int) : Bool = filt.tgt === t.U

  def setTx(filt: Flit, t: Int) : Unit = filt.src := t.U
  def setTx(filt: Flit, t: UInt): Unit = filt.src := t
  def txIs (filt: Flit, t: Int) : Bool = filt.src === t.U
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

  def toLAN(selfCI: UInt)  = ci === selfCI
  def toBBN(selfCI: UInt)  = ci =/= selfCI
  def getNoC(selfCI: UInt) = Mux(toLAN(selfCI), NocType.LAN, NocType.BBN)

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
  val pos     = new PosIndex()
  val dirBank = UInt(dirBankBits.W)
  def get     = pos.getLLCTxnID(dirBank)
}

class LLCTxnID(implicit p: Parameters) extends DJBundle with HasLLCTxnID

/*
 * ChiTask
 */
trait HasChiTask { this: DJBundle with HasNodeId with HasChiChannel with HasChiOp
  with HasChiOrderAndExpCompAck with HasChiSnpField with HasChiSize =>
  // REQ
  val txnID     = UInt(ChiTxnIdBits.W)
  val memAttr   = new MemAttr()
  // SNP
  val fwdNID    = UInt(nodeIdBits.W)
  val fwdTxnID  = UInt(ChiFwdTxnIdBits.W)
  val retToSrc  = Bool()
}

class ChiTask(implicit p: Parameters) extends DJBundle with HasNodeId with HasChiChannel
  with HasChiOp with HasChiOrderAndExpCompAck with HasChiSnpField with HasChiSize with HasChiTask

trait HasFastChiInst { this: DJBundle with HasNodeId with HasChiChannel with HasChiOp
  with HasChiOrderAndExpCompAck with HasChiSnpField with HasChiSize with HasChiTask with HasAddr =>
  def getInst(ci: UInt): ChiInst = {
    val inst = Wire(new ChiInst)
    inst.channel    := channel
    inst.fromLAN    := fromLAN
    inst.toLAN      := toLAN(ci)
    inst.opcode     := opcode
    inst.expCompAck := expCompAck
    inst
  }
}