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
                  llcSizeInB:         Int = 2 * 1024 * 1024,
                  sfSizeInB:          Int = 2 * 1024 * 1024,
                  openDCT:            Boolean = true,
                  // ------------------------ Frontend -------------------------- //
                  nrReqTaskBuf:       Int = 16,
                  nrSnpTaskBuf:       Int = 8,
                  nrPoS:              Int = 64, // The number of outstanding
                  // ------------------------ Memblock ----------------------- //
                  dataBufSizeInByte:  Int = 64 * 32,
                  nrDataCM:           Int = 32,
                  // Data SRAM
                  nrDSBank:           Int = 4,
                  dataSetup:          Int = 2,
                  dataExtraHold:      Boolean = false,
                  dataLatency:        Int = 2,
                  // ------------------------ Directory  ----------------------- //
                  // Replace Policy is PLRU
                  llcWays:            Int = 16, // self llc ways
                  sfWays:             Int = 16, // snoop filter ways
                  // Dir SRAM
                  nrDirBank:          Int = 2,
                  dirSetup:           Int = 1,
                  dirExtraHold:       Boolean = false,
                  dirLatency:         Int = 2,
                ) {
  lazy val hasLLC    = llcSizeInB != 0
  lazy val CacheLine = 64 // Bytes
  lazy val BeatByte  = 32
  lazy val nrBeat    = CacheLine / BeatByte
  // Last level Cache
  lazy val llcSets   = llcSizeInB / CacheLine / llcWays
  // Snoop filter
  lazy val sfSets    = sfSizeInB / CacheLine / sfWays
  // PoS Table
  lazy val posWays   = if(hasLLC) min(llcWays, sfWays) else sfWays
  lazy val posSets   = nrPoS / posWays
  // Data
  lazy val nrDataBuf = dataBufSizeInByte / BeatByte
  lazy val dsSets    = llcSets * llcWays

  require(llcSizeInB  >= CacheLine * llcWays, s"illegal llc size: ${llcSizeInB}B")
  require(sfSizeInB >= CacheLine * llcWays, s"illegal llc size: ${sfSizeInB}B")
  require(posSets / nrDirBank > 1, s"illegal pos size: ${nrPoS}")
  require(llcSets >= nrDirBank)
  require(sfSets >= nrDirBank)
  require(dsSets >= nrDSBank)
  require(nrReqTaskBuf > 0)
  require(nrSnpTaskBuf >= 0)
  require(isPow2(dataBufSizeInByte))
  require(isPow2(nrDSBank))
  require(isPow2(nrDirBank))
  require(isPow2(llcWays))
  require(isPow2(sfWays))
}


trait HasParseZJParam extends HasZJParams {
  // Get LAN Nodes
  lazy val lanCcNodes       = zjParams.island.filter(_.nodeType == NodeType.CC)
  lazy val lanRniNodes      = zjParams.island.filter(_.nodeType == NodeType.RI)
  lazy val lanHnfNodes      = zjParams.island.filter(_.nodeType == NodeType.HF)
  lazy val lanSnNodes       = zjParams.island.filter(_.nodeType == NodeType.S)
  lazy val ccNodeIdSeq      = lanCcNodes.map(_.nodeId)
  lazy val rniNodeIdSeq     = lanRniNodes.map(_.nodeId)
  lazy val snNodeIdSeq      = lanSnNodes.map(_.nodeId)
  require(lanCcNodes.nonEmpty)
  require(lanHnfNodes.nonEmpty)
  require(lanSnNodes.nonEmpty)

  // Get BBN Nodes
  lazy val hasBBN           = zjParams.r2rPos.nonEmpty

  // Node ID
  lazy val nodeIdBits       = zjParams.nodeIdBits

  // Bank and Node Number
  lazy val nrHnfPort        = lanHnfNodes.map(_.hfpId).distinct.length
  lazy val nrBank           = lanHnfNodes.length / nrHnfPort
  lazy val nrCcNode         = lanCcNodes.length
  lazy val nrCI             = 16 // TODO: parameterize
  lazy val nrSfMetas        = nrCcNode + nrCI
  lazy val metaIdBits       = log2Ceil(nrSfMetas)

  // ICN Number Per Bank
  lazy val nrLanIcn         = 1
  lazy val nrBbnIcn         = if(hasBBN) 1 else 0
  lazy val nrIcn            = nrLanIcn + nrBbnIcn
  lazy val LAN              = 0
  lazy val BBN              = 1

  // temp node id bits
  lazy val lanNBits = nodeNidBits
  lazy val lanABits = nodeAidBits
  lazy val bbnIBits = ciIdBits
  lazy val bbnBBits = niw - ciIdBits

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

  // Addr Parameters
  // [fullAddr] = [useAddr1] + [bankId] + [useAddr0] + [offset]
  // [useAddr]  = [useAddr1] + [useAddr0]
  //            = [llcTag]   + [llcSet] + [dirBank]
  //            = [sfTag]    + [sfSet]  + [dirBank]
  //            = [posTag]   + [posSet] + [dirBank]
  //            = [ci]       + [unUse]
  // [ds]       = [llcSet]   + [llcWay] + [dirBank]
  //            = [dsIndex]    + [dsBank]
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
  require(addrBits - ciBits > hnxBankOff + bankBits - 1)
  require(hnxBankOff > offsetBits)
  // useAddr
  lazy val useAddr_hi       = addrBits - 1
  lazy val useAddr_lo       = offsetBits
  // bankId
  lazy val bankId_hi        = hnxBankOff + bankBits - 1
  lazy val bankId_lo        = hnxBankOff
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
  // dsIndex
  lazy val dsIdx_ds_hi      = llcSetBits + llcWayBits + dirBankBits - 1
  lazy val dsIdx_ds_lo      = dsBankBits
  // dsBank
  lazy val dsBank_ds_hi     = dsBankBits - 1
  lazy val dsBank_ds_lo     = 0

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
  def getDS       (a: UInt, way: UInt) = {
    require(way.getWidth == llcWayBits)
    val temp = Cat(getLlcSet(a), way, getDirBank(a))
    val idx  = temp(dsIdx_ds_hi,  dsIdx_ds_lo)
    val bank = temp(dsBank_ds_hi, dsBank_ds_lo)
    (idx, bank)
  }

  // Data Parameters
  lazy val DataBits         = djparam.CacheLine * 8
  lazy val BeatBits         = djparam.BeatByte * 8
  lazy val MaskBits         = djparam.BeatByte
  lazy val ChiFullSize      = log2Ceil(djparam.CacheLine) // 6
  lazy val ChiHalfSize      = log2Ceil(djparam.BeatByte) // 5
  require(MaskBits == zjParams.beBits)


  // Frontend(Per dirBank) and Directory Parameters
  lazy val nrReqTaskBuf     = djparam.nrReqTaskBuf / djparam.nrDirBank
  lazy val nrSnpTaskBuf     = djparam.nrReqTaskBuf / djparam.nrDirBank
  lazy val nrPoS            = djparam.nrPoS / djparam.nrDirBank
  lazy val posWays          = djparam.posWays
  lazy val posWayBits       = log2Ceil(posWays)
  lazy val dirMuticycle     = djparam.dirLatency.max(if(djparam.dirExtraHold) djparam.dirSetup + 1 else djparam.dirSetup)
  lazy val readDirLatency   = djparam.dirSetup + djparam.dirLatency
  lazy val llcWayBits       = log2Ceil(djparam.llcWays)
  // [S1(PoS/Block)] + [S2(ReadDir)] + [S3(Decode)] + Reserve for snp
  lazy val nrIssueBuf       = 4
  lazy val issueBufBits     = log2Ceil(nrIssueBuf)


  // DataBlock Parameters
  // dc/db
  lazy val dbIdBits         = log2Ceil(djparam.nrDataBuf)
  lazy val dcIdBits         = log2Ceil(djparam.nrDataCM)
  //ds
  lazy val dsBank           = djparam.nrDSBank
  lazy val nrDsSet          = djparam.dsSets / dsBank
  lazy val dsIdxBits        = log2Ceil(nrDsSet)
  lazy val dsMuticycle      = djparam.dataLatency.max(if(djparam.dataExtraHold) djparam.dataSetup + 1 else djparam.dataSetup)
  lazy val readDsLatency    = djparam.dataSetup + djparam.dataLatency

  // Replacement(PLRU) Parameters
  lazy val sReplWayBits     = djparam.llcWays - 1
  lazy val sfReplWayBits    = djparam.sfWays - 1

  // Backend Parameters
  lazy val nrReplaceCM      = djparam.nrPoS / 2
  lazy val nrTaskCM         = 4
  lazy val nrSnoopCM        = djparam.nrPoS / 4
  lazy val nrReadCM         = djparam.nrPoS / 2
  lazy val nrDatalessCM     = djparam.nrPoS / 4
  lazy val nrWriOrAtmCM     = djparam.nrPoS / 4
  lazy val fastRespQSzie    = djparam.nrPoS / 8


  // TIMEOUT CHECK CNT VALUE
  lazy val TIMEOUT_TASKBUF  = 30000
  lazy val TIMEOUT_POS      = 20000
  lazy val TIMEOUT_COMMIT   = 18000
  lazy val TIMEOUT_LOCK     = 18000
  lazy val TIMEOUT_REPLACE  = 15000
  lazy val TIMEOUT_DATACM   = 10000
  lazy val TIMEOUT_ISSUE    = 10000
  lazy val TIMEOUT_SNP      = 10000
  lazy val TIMEOUT_READ     = 10000
  lazy val TIMEOUT_WOA      = 10000
}


abstract class DJModule(implicit val p: Parameters) extends Module with HasDJParam

abstract class DJBundle(implicit val p: Parameters) extends Bundle with HasDJParam

abstract class DJRawModule(implicit val p: Parameters) extends RawModule with HasDJParam
