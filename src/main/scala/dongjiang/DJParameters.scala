package dongjiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import scala.math.{log, max, min}
import zhujiang.{HasZJParams, ZJParametersKey}
import xijiang.NodeType
import xs.utils.debug._


case class DJParam(
                  // -------------------------- Size and DCT ---------------------------- //
                  addressBits:        Int = 48,
                  llcSizeInKiB:       Int = 16 * 1024,
                  sfSizeInKiB:        Int = 8 * 1024,
                  openDCT:            Boolean = true,
                  // ------------------------ Frontend -------------------------- //
                  nrReqTaskBuf:       Int = 64,
                  nrSnpTaskBuf:       Int = 32,
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
  lazy val CacheLine = 64 // Bytes
  lazy val BeatByte  = 32
  lazy val nrBeat    = CacheLine / BeatByte
  // Last level Cache
  lazy val llcSets   = llcSizeInKiB * 1024 / CacheLine / llcWays
  // Snoop filter
  lazy val sfSets    = sfSizeInKiB * 1024 / CacheLine / sfWays
  // PoS Table
  lazy val posWays   = if(hasLLC) min(llcWays, sfWays) else sfWays
  lazy val posSets   = nrPoS / posWays
  // Memblock
  lazy val nrDataBuf = dataBufSizeInByte / BeatByte
  // Backend
  lazy val nrReplaceCM  = nrPoS / 2
  lazy val nrRetryBuf   = nrPoS / 4
  lazy val nrSnoopCM    = nrPoS / 4
  lazy val nrReadCM     = nrPoS / 4
  lazy val nrDatalessCM = nrPoS / 4

  require(llcSizeInKiB >= 0)
  require(sfSizeInKiB > 0)
  require(nrReqTaskBuf > 0)
  require(nrSnpTaskBuf >= 0)
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
  // Get LAN Nodes
  lazy val lanCcNodes       = zjParams.localRing.filter(_.nodeType == NodeType.CC)
  lazy val lanRniNodes      = zjParams.localRing.filter(_.nodeType == NodeType.RI)
  lazy val lanHnfNodes      = zjParams.localRing.filter(_.nodeType == NodeType.HF)
  lazy val lanSnNodes       = zjParams.localRing.filter(_.nodeType == NodeType.S)
  lazy val ccNodeIdSeq      = lanCcNodes.map(_.nodeId)
  lazy val rniNodeIdSeq     = lanRniNodes.map(_.nodeId)
  lazy val snNodeIdSeq      = lanSnNodes.map(_.nodeId)
  require(lanCcNodes.nonEmpty)
  require(lanHnfNodes.nonEmpty)
  require(lanSnNodes.nonEmpty)

  // Get BBN Nodes
  lazy val bbnC2CNodes      = zjParams.csnRing.filter(_.nodeType == NodeType.C2C)
  lazy val bbnHnxNodes      = zjParams.csnRing.filter(_.nodeType == NodeType.HX)
  lazy val hasHnx           = zjParams.csnRing.nonEmpty
  if(hasHnx) {
    require(bbnC2CNodes.nonEmpty)
    require(bbnHnxNodes.length == lanHnfNodes.length)
  }

  // Node ID
  lazy val nrFriendsNodeMax = lanHnfNodes.map(_.friends.length).max
  lazy val nodeIdBits       = zjParams.nodeIdBits

  // Bank and Node Number
  lazy val nrHnfPort        = lanHnfNodes.map(_.hfpId).distinct.length
  lazy val nrBank           = lanHnfNodes.length / nrHnfPort
  lazy val nrCcNode         = lanCcNodes.length
  lazy val nrCI             = 16 // TODO: parameterize
  lazy val metaIdBits       = log2Ceil(nrCcNode + nrCI)

  // ICN Number Per Bank
  lazy val nrLanIcn       = nrHnfPort
  lazy val nrBbnIcn         = if(hasHnx) 1 else 0
  lazy val nrIcn            = nrLanIcn + nrBbnIcn

  // temp node id bits
  val lanNBits = 5
  val lanABits = 3
  val bbnIBits = 4
  val bbnBBits = 4

  /*
   * Check from X node
   */
  def fromLanXNode(nodeId: UInt, nodeIdSeq: Seq[Int]): Bool = {
    require(nodeId.getWidth == nodeIdBits)
    val fromXSeq = nodeIdSeq.map(_.asUInt >> lanABits === nodeId >> lanABits)
    HardwareAssertion(PopCount(fromXSeq) <= 1.U)
    fromXSeq.reduce(_ | _)
  }
}


trait HasDJParam extends HasParseZJParam {
  val p: Parameters
  val djparam = p(ZJParametersKey).djParams

  // CHI field width
  lazy val ChiOpcodeBits      = 7
  lazy val ChiTxnIdBits       = 12
  lazy val ChiDBIDBits        = 12
  lazy val ChiFwdTxnIdBits    = 12
  lazy val ChiReturnTxnIdBits = 12
  lazy val ChiSizeBits        = 3

  // Data Parameters
  lazy val DataBits         = djparam.CacheLine * 8
  lazy val BeatBits         = djparam.BeatByte * 8
  lazy val MaskBits         = djparam.BeatByte
  lazy val ChiFullSize      = log2Ceil(djparam.CacheLine) // 6
  lazy val ChiHalfSize      = log2Ceil(djparam.BeatByte)  // 5
  require(MaskBits == zjParams.beBits)


  // Addr Parameters
  // [fullAddr] = [useAddr1] + [bankId] + [useAddr0] + [offset]
  // [useAddr]  = [useAddr1] + [useAddr0]
  //            = [llcTag]   + [llcSet] + [dirBank]
  //            = [sfTag]    + [sfSet]  + [dirBank]
  //            = [posTag]   + [posSet] + [dirBank]
  //            = [ci]       + [unUse]
  //            = [unUse]    + [dsBank]
  // full
  lazy val addrBits         = djparam.addressBits
  lazy val ciBits           = 4
  lazy val bankBits         = log2Ceil(nrBank)
  lazy val offsetBits       = log2Ceil(djparam.CacheLine)
  lazy val dirBankBits      = log2Ceil(djparam.nrDirBank)
  lazy val dsBankBits       = log2Ceil(djparam.nrDSBank)
  lazy val useAddrBits      = addrBits - bankBits - offsetBits
  // llc per dirBank
  lazy val llcSets          = djparam.llcSets / djparam.nrDirBank
  lazy val llcSetBits       = log2Ceil(llcSets)
  lazy val llcTagBits       = addrBits - bankBits - llcSetBits - dirBankBits - offsetBits
  // sf per dirBank
  lazy val sfSets           = djparam.sfSets / djparam.nrDirBank
  lazy val sfSetBits        = log2Ceil(sfSets)
  lazy val sfTagBits        = addrBits - bankBits - sfSetBits - dirBankBits - offsetBits
  // pos per dirBank
  lazy val posSets          = djparam.posSets / djparam.nrDirBank
  lazy val posSetBits       = log2Ceil(posSets)
  lazy val posTagBits       = addrBits - bankBits - posSetBits - dirBankBits - offsetBits
  // require [ci_lo] > [bankId_hi] > [bankId_lo] > [offset_hi]
  require(addrBits - ciBits > bankOff + bankBits - 1)
  require(bankOff > offsetBits)
  // useAddr
  lazy val useAddr_hi       = addrBits - 1
  lazy val useAddr_lo       = offsetBits
  // bankId
  lazy val bankId_hi        = bankOff + bankBits - 1
  lazy val bankId_lo        = bankOff
  // offset
  lazy val offset_hi        = offsetBits - 1
  lazy val offset_lo        = 0
  // islandId
  lazy val ci_ua_hi         = useAddrBits - 1
  lazy val ci_ua_lo         = useAddrBits - ciBits
  // dirBank
  lazy val dirBank_ua_hi    = dirBankBits - 1
  lazy val dirBank_ua_lo    = 0
  // llcTag(per dirBank)
  lazy val llcTag_ua_hi     = useAddrBits - 1
  lazy val llcTag_ua_lo     = useAddrBits - llcTagBits
  // llcSet(per dirBank)
  lazy val llcSet_ua_hi     = dirBankBits + llcSetBits - 1
  lazy val llcSet_ua_lo     = dirBankBits
  // sfTag(per dirBank)
  lazy val sfTag_ua_hi      = useAddrBits - 1
  lazy val sfTag_ua_lo      = useAddrBits - sfTagBits
  // sfSet(per dirBank)
  lazy val sfSet_ua_hi      = dirBankBits + sfSetBits - 1
  lazy val sfSet_ua_lo      = dirBankBits
  // posTag(per dirBank)
  lazy val posTag_ua_hi     = useAddrBits - 1
  lazy val posTag_ua_lo     = useAddrBits - posTagBits
  // posSet(per dirBank)
  lazy val posSet_ua_hi     = dirBankBits + posSetBits - 1
  lazy val posSet_ua_lo     = dirBankBits
  // dsBank
  lazy val dsBank_ua_hi     = dsBankBits - 1
  lazy val dsBank_ua_lo     = 0

  // base
  def getCI       (a: UInt) = a(ci_ua_hi, ci_ua_lo)
  def getUseAddr  (a: UInt) = Cat(a(useAddr_hi, bankId_hi + 1), a(bankId_lo - 1, useAddr_lo))
  def getBankId   (a: UInt) = a(bankId_hi, bankId_lo)
  def getOffset   (a: UInt) = a(offset_hi, offset_lo)
  // llc
  def getDirBank  (a: UInt) = getUseAddr(a)(dirBank_ua_hi, dirBank_ua_lo)
  def getLlcTag   (a: UInt) = getUseAddr(a)(llcTag_ua_hi, llcTag_ua_lo)
  def getLlcSet   (a: UInt) = getUseAddr(a)(llcSet_ua_hi, llcSet_ua_lo)
  // sf
  def getSfTag    (a: UInt) = getUseAddr(a)(sfTag_ua_hi, sfTag_ua_lo)
  def getSfSet    (a: UInt) = getUseAddr(a)(sfSet_ua_hi, sfSet_ua_lo)
  // pos
  def getPosTag   (a: UInt) = getUseAddr(a)(posTag_ua_hi, posTag_ua_lo)
  def getPosSet   (a: UInt) = getUseAddr(a)(posSet_ua_hi, posSet_ua_lo)
  // ds
  def getDSBank   (a: UInt) = getUseAddr(a)(dsBank_ua_hi, dsBank_ua_lo)


  // Frontend(Per dirBank) Parameters
  lazy val nrReqTaskBuf     = djparam.nrReqTaskBuf / djparam.nrDirBank
  lazy val nrSnpTaskBuf     = djparam.nrReqTaskBuf / djparam.nrDirBank
  lazy val posWays          = djparam.posWays
  lazy val posWayBits       = log2Ceil(posWays)

  // Memblock Parameters
  lazy val dbIdBits         = log2Ceil(djparam.nrDataBuf)
  lazy val dcIdBits         = log2Ceil(djparam.nrDataCM)
  require(dbIdBits <= dcIdBits)
  require(dcIdBits <= ChiTxnIdBits)

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
