package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._


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

  def catByX(bank: UInt, tag: UInt, tagBits: Int, set: UInt, setBits: Int, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = {
    require(bank.getWidth    == bankBits)
    require(tag.getWidth     == tagBits)
    require(set.getWidth     == setBits)
    require(dirBank.getWidth == dirBankBits)
    require(offset.getWidth  == offsetBits)
    val useAddr_ = Cat(tag, set, dirBank)
    val addr_ = Cat(useAddr_(useAddrBits-1, bankId_lo-offsetBits), bank, useAddr_(bankId_lo-offsetBits-1, 0), offset)
    require(addr_.getWidth   == addrBits)
    this.addr := addr_
  }
  def catPoS(bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, posTagBits, set, posSetBits, dirBank, offset)
  def catLLC(bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, llcTagBits, set, llcSetBits, dirBank, offset)
  def catSF (bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, sfTagBits,  set, sfSetBits,  dirBank, offset)
}

class Addr(implicit p: Parameters) extends DJBundle with HasAddr

class PosIndex(implicit p: Parameters) extends DJBundle {
  val set = UInt(posSetBits.W)
  val way = UInt(posWayBits.W)

  def idxMatch(i: Int, j: Int)  = set === i.U & way === j.U
  def getLLCTxnID(dirBank: Int) = Cat(dirBank.U, set, way)
}

trait HasPosIndex extends DJBundle { this: DJBundle =>
  val pos = new PosIndex()
}

trait HasLLCTxnID extends DJBundle { this: DJBundle =>
  val llcTxnID = UInt(log2Ceil(djparam.nrPoS).W)
}

trait HasDCID extends DJBundle { this: DJBundle =>
  val dcid = UInt(dcIdBits.W)
}

class DCID(implicit p: Parameters) extends DJBundle with HasDCID