package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._


trait HasAddr extends DJBundle { this: Bundle =>
  val addr      = UInt(addrBits.W)

  def islandId  = getIslandId(addr)
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
}