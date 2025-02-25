package dongjiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import scala.math.{log, max, min}
import zhujiang.{HasZJParams, ZJParametersKey}
import xijiang.NodeType


case class DJParam(
                  // -------------------------- Size and DCT ---------------------------- //
                  addressBits:        Int = 48,
                  llcSizeInKiB:       Int = 16 * 1024,
                  sfSizeInKiB:        Int = 8 * 1024,
                  openDCT:            Boolean = true,
                  // ------------------------ Frontend -------------------------- //
                  nrTaskBuf:          Int = 64,
                  nrPoS:              Int = 128, // The number of outstanding
                  // ------------------------ Memblock ----------------------- //
                  dataBufSizeInByte:  Int = 32 * 32,
                  nrDataCM:           Int = 64,
                  // Data SRAM
                  nrDSBank:           Int = 4,
                  dataSetup:          Int = 3,
                  dataLatency:        Int = 3,
                  dataExtraHold:      Boolean = false,
                  // ------------------------ Directory  ----------------------- //
                  // Replace Policy is PLRU
                  llcWays:            Int = 16, // self llc ways
                  sfWays:             Int = 16, // snoop filter ways
                  // Dir SRAM
                  nrDirBank:          Int = 2,
                  dirSetup:           Int = 2,
                  dirLatency:         Int = 2,
                  dirExtraHold:       Boolean = false,
                ) {
  lazy val hasLLC    = llcSizeInKiB != 0
  lazy val cacheLine = 64 // Bytes
  lazy val beatByte  = 32
  lazy val nrBeat    = 2
  // Last level Cache
  lazy val llcSets   = llcSizeInKiB * 1024 / cacheLine / llcWays
  // Snoop filter
  lazy val sfSets    = sfSizeInKiB * 1024 / cacheLine / sfWays
  // PoS Table
  lazy val posWays   = if(hasLLC) min(llcWays, sfWays) else sfWays
  lazy val posSets   = nrPoS / posWays
  // Memblock
  lazy val nrDataBuf = dataBufSizeInByte / beatByte
  // Backend
  lazy val nrReplaceCM  = nrPoS / 2
  lazy val nrRetryBuf   = nrPoS / 4
  lazy val nrSnoopCM    = nrPoS / 4
  lazy val nrReadCM     = nrPoS / 4
  lazy val nrDatalessCM = nrPoS / 4

  require(llcSizeInKiB >= 0)
  require(sfSizeInKiB > 0)
  require(nrTaskBuf > 0)
  require(nrPoS > 0)
  require(isPow2(dataBufSizeInByte))
  require(nrDataCM >= nrDataBuf)
  require(isPow2(nrDSBank))
  require(isPow2(nrDirBank))
  require(isPow2(llcWays))
  require(isPow2(sfWays))
  require(nrBeat == 1 | nrBeat == 2 | nrBeat == 4)
}


trait HasParseZJParam extends HasZJParams {
  // Get Local Nodes
  lazy val localCcNodes     = zjParams.localRing.filter(_.nodeType == NodeType.CC)
  lazy val localRniNodes    = zjParams.localRing.filter(_.nodeType == NodeType.RI)
  lazy val localHnfNodes    = zjParams.localRing.filter(_.nodeType == NodeType.HF)
  lazy val localSnNodes     = zjParams.localRing.filter(_.nodeType == NodeType.S)
  lazy val ccNodeIdSeq      = localCcNodes.map(_.nodeId)
  lazy val rniNodeIdSeq     = localRniNodes.map(_.nodeId)
  lazy val snNodeIdSeq      = localSnNodes.map(_.nodeId)
  require(localCcNodes.nonEmpty)
  require(localHnfNodes.nonEmpty)
  require(localSnNodes.nonEmpty)

  // Get CSN Nodes
  lazy val csnC2CNodes      = zjParams.csnRing.filter(_.nodeType == NodeType.C2C)
  lazy val csnHnxNodes      = zjParams.csnRing.filter(_.nodeType == NodeType.HX)
  lazy val c2cNodeIdSeq     = csnC2CNodes.map(_.nodeId)
  lazy val hnxNodeIdSeq     = csnHnxNodes.map(_.nodeId)
  lazy val hasCSN           = zjParams.csnRing.nonEmpty
  lazy val ccxChipBits      = 3
  if(hasCSN) {
    require(csnC2CNodes.nonEmpty)
    require(csnHnxNodes.length == localHnfNodes.length)
  }

  // Node ID
  lazy val nrFriendsNodeMax = localHnfNodes.map(_.friends.length).max
  lazy val nodeIdBits       = zjParams.nodeIdBits

  // Bank and Node Number
  lazy val nrHnfPort        = localHnfNodes.map(_.hfpId).distinct.length
  lazy val nrBank           = localHnfNodes.length / nrHnfPort
  lazy val nrCcNode         = localCcNodes.length
  lazy val nrChip           = 8 // TODO: parameterize
  lazy val metaIdBits       = log2Ceil(nrCcNode + nrChip)

  // ICN Number Per Bank
  lazy val nrLocalIcn       = nrHnfPort
  lazy val nrCsnIcn         = if(hasCSN) 1 else 0
  lazy val nrIcn            = nrLocalIcn + nrCsnIcn

  /*
   * Check from X node
   */
  def fromXNode(nodeId: UInt, nodeIdSeq: Seq[Int]): Bool = {
    require(nodeId.getWidth == nodeNidBits)
    val fromX = WireInit(false.B)
    fromX := nodeIdSeq.map(_.asUInt >> nodeAidBits === nodeId).reduce(_ | _)
    fromX
  }

  /*
   * Get nid from NodeId
   */
  def getNID(nodeId: UInt): UInt = {
    require(nodeId.getWidth == nodeIdBits)
    nodeId(nodeIdBits-nodeNetBits-1, nodeAidBits)
  }
}


trait HasDJParam extends HasParseZJParam {
  val p: Parameters
  val djparam = p(ZJParametersKey).djParams

  // CHI field width
  lazy val chiOpcodeBits      = 7
  lazy val chiTxnIdBits       = 12
  lazy val chiDBIDBits        = 12
  lazy val chiFwdTxnIdBits    = 12
  lazy val chiReturnTxnIdBits = 12
  lazy val chiSizeBits        = 3

  // Data Parameters
  lazy val dataBits         = djparam.cacheLine * 8
  lazy val beatBits         = djparam.beatByte * 8
  lazy val maskBits         = djparam.beatByte
  lazy val chiFullSize      = log2Ceil(djparam.cacheLine) // 6
  lazy val chiHalfSize      = log2Ceil(djparam.beatByte)  // 5
  require(maskBits == zjParams.beBits)


  // Addr Parameters
  // [fullAddr] = [cacheable] + [ccxChipId] + [useAddr1] + [bankId] + [useAddr0] + [offset]
  // [useAddr]  = [cacheable] + [ccxChipId] + [useAddr1] + [useAddr0]
  //            = [llcTag]    + [llcSet]    + [dirBank]
  //            = [sfTag]     + [sfSet]     + [dirBank]
  //            = [posTag]    + [posSet]    + [dirBank]
  //            = [unUse]     + [dsBank]
  // full
  lazy val addrBits         = djparam.addressBits
  lazy val cacheableBits    = 1
  lazy val ccxChipIdBits    = 3
  lazy val bankBits         = log2Ceil(nrBank)
  lazy val offsetBits       = log2Ceil(djparam.cacheLine)
  lazy val dirBankBits      = log2Ceil(djparam.nrDirBank)
  lazy val useAddrBits      = addrBits - bankBits - offsetBits
  // llc per dirBank
  lazy val llcSets          = djparam.llcSets / djparam.nrDirBank
  lazy val llcSetBits       = log2Ceil(llcSets)
  lazy val llcTagBits       = addrBits - llcSetBits - dirBankBits - offsetBits
  // sf per dirBank
  lazy val sfSets           = djparam.sfSets / djparam.nrDirBank
  lazy val sfSetBits        = log2Ceil(sfSets)
  lazy val sfTagBits        = addrBits - sfSetBits - dirBankBits - offsetBits
  // pos per dirBank
  lazy val posSets          = djparam.posSets / djparam.nrDirBank
  lazy val posSetBits       = log2Ceil(posSets)
  lazy val posTagBits       = addrBits - posSetBits - dirBankBits - offsetBits
  // require [ccxChipId] > [bankId] > [offset]
  require(bankOff + bankBits - 1 < addrBits - (cacheableBits + ccxChipBits))
  require(bankOff > offsetBits)
  // cacheable
  lazy val cacheable_hi     = addrBits - 1
  lazy val cacheable_lo     = addrBits - cacheableBits
  // ccxChipId
  lazy val ccxChipId_hi     = cacheable_lo - 1
  lazy val ccxChipId_lo     = cacheable_lo - ccxChipBits
  // useAddr
  lazy val useAddr_hi       = addrBits - 1
  lazy val useAddr_lo       = offsetBits
  // bankId
  lazy val bankId_hi        = bankOff + bankBits - 1
  lazy val bankId_lo        = bankOff
  // offset
  lazy val offset_hi        = offsetBits - 1
  lazy val offset_lo        = 0
  // dirBank
  lazy val dirBank_ua_hi    = useAddr_lo + dirBankBits - 1
  lazy val dirBank_ua_lo    = useAddr_lo
  // llcTag(per dirBank)
  lazy val llcTag_ua_hi     = useAddrBits - 1
  lazy val llcTag_ua_lo     = useAddrBits - llcTagBits
  // llcSet(per dirBank)
  lazy val llcSet_ua_hi     = llcTag_ua_lo - 1
  lazy val llcSet_ua_lo     = dirBankBits
  // sfTag(per dirBank)
  lazy val sfTag_ua_hi      = useAddrBits - 1
  lazy val sfTag_ua_lo      = useAddrBits - sfTagBits
  // sfSet(per dirBank)
  lazy val sfSet_ua_hi      = sfTag_ua_lo - 1
  lazy val sfSet_ua_lo      = dirBankBits
  // posTag(per dirBank)
  lazy val posTag_ua_hi     = useAddrBits - 1
  lazy val posTag_ua_lo     = useAddrBits - posTagBits
  // posSet(per dirBank)
  lazy val posSet_ua_hi     = posTag_ua_lo - 1
  lazy val posSet_ua_lo     = dirBankBits

  // base
  def cacheable(addr: UInt) = addr(cacheable_hi, cacheable_lo)
  def ccxChipId(addr: UInt) = addr(ccxChipId_hi, ccxChipId_lo)
  def useAddr  (addr: UInt) = Cat(addr(useAddr_hi, bankId_hi + 1), addr(bankId_lo - 1, useAddr_lo))
  def bankId   (addr: UInt) = addr(bankId_hi, bankId_lo)
  def offset   (addr: UInt) = addr(offset_hi, offset_lo)
  // llc
  def dirBank  (addr: UInt) = useAddr(addr)(dirBank_ua_hi, dirBank_ua_lo)
  def llcTag   (addr: UInt) = useAddr(addr)(llcTag_ua_hi, llcTag_ua_lo)
  def llcSet   (addr: UInt) = useAddr(addr)(llcSet_ua_hi, llcSet_ua_lo)
  // sf
  def sfTag    (addr: UInt) = useAddr(addr)(sfTag_ua_hi, sfTag_ua_lo)
  def sfSet    (addr: UInt) = useAddr(addr)(sfSet_ua_hi, sfSet_ua_lo)
  // pos
  def posTag   (addr: UInt) = useAddr(addr)(posTag_ua_hi, posTag_ua_lo)
  def posSet   (addr: UInt) = useAddr(addr)(posSet_ua_hi, posSet_ua_lo)


  // Frontend(Per dirBank) Parameters
  lazy val nrTaskBuf        = djparam.nrTaskBuf / djparam.nrDirBank

  // Memblock Parameters
  lazy val dbIdBits         = log2Ceil(djparam.nrDataBuf)
  lazy val dcIdBits         = log2Ceil(djparam.nrDataCM)
  require(dbIdBits <= dcIdBits)
  require(dcIdBits <= chiTxnIdBits)

  // Replacement(PLRU) Parameters
  lazy val sReplWayBits     = djparam.llcWays - 1
  lazy val sfReplWayBits    = djparam.sfWays - 1


  // TIMEOUT CHECK CNT VALUE
  lazy val TIMEOUT_TASKBUF  = 20000 + 10000 // MSHR
  lazy val TIMEOUT_POS      = 10000 + 10000
  lazy val TIMEOUT_RETRYBUF =  5000 + 10000
  lazy val TIMEOUT_SNP      =         10000
  lazy val TIMEOUT_READ     =         10000
  lazy val TIMEOUT_COMMIT   =  5000 + 10000
  lazy val TIMEOUT_REPLACE  =         10000
  lazy val TIMEOUT_LOCK     = 10000 + 10000
  lazy val TIMEOUT_DATACM   = 10000 + 10000
}


abstract class DJModule(implicit val p: Parameters) extends Module with HasDJParam

abstract class DJBundle(implicit val p: Parameters) extends Bundle with HasDJParam

abstract class DJRawModule(implicit val p: Parameters) extends RawModule with HasDJParam
