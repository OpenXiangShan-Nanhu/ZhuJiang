package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._

class AddrBundle(implicit p: Parameters) extends DJBundle {
  val addr = UInt(addrBits.W)

//  // base
//  def cacheable = addr(cacheable_hi, cacheable_lo)
//  def ccxChipId = addr(ccxChipId_hi, ccxChipId_lo)
//  def useAddr   = Cat(addr(useAddr_hi, bankId_hi+1), addr(bankId_lo-1, useAddr_lo))
//  def bankId    = addr(bankId_hi,    bankId_lo)
//  def offset    = addr(offset_hi,    offset_lo)
//  // llc
//  def dirBank   = useAddr(dirBank_ua_hi, dirBank_ua_lo)
//  def llcTag    = useAddr(llcTag_ua_hi,  llcTag_ua_lo)
//  def llcSet    = useAddr(llcSet_ua_hi,  llcSet_ua_lo)
//  // sf
//  def sfTag     = useAddr(sfTag_ua_hi, sfTag_ua_lo)
//  def sfSet     = useAddr(sfSet_ua_hi, sfSet_ua_lo)
//  // pos
//  def posTag    = useAddr(posTag_ua_hi, posTag_ua_lo)
//  def posSet    = useAddr(posSet_ua_hi, posSet_ua_lo)
}
