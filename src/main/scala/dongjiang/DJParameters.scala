package dongjiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import scala.math.{log, max, min}
import zhujiang.{HasZJParams, ZJParametersKey}
import xijiang.NodeType
import dongjiang.frontend.decode.Decode._
import xs.utils.debug._


case class DJParam(
                  // -------------------------- Size and DCT ---------------------------- //
                  addressBits:        Int = 48,
                  llcSizeInB:         Int = 2 * 1024 * 1024,
                  sfSizeInB:          Int = 2 * 1024 * 1024,
                  openDCT:            Boolean = true,
                  // ------------------------ Frontend -------------------------- //
                  nrReqTaskBuf:       Int = 8,
                  nrHprTaskBuf:       Int = 4,
                  nrSnpTaskBuf:       Int = 4,  // unuse in BBN is false
                  nrPoS:              Int = 32, // The number of outstanding
                  nrReadCM:           Int = 0,  // 0: nrReadCM is calculate by nrPoS, otherwise use user define value
                  // ------------------------ Memblock ----------------------- //
                  dataBufSizeInByte:  Int = 64 * 32,
                  // Data SRAM
                  nrDSBank:           Int = 4,
                  dataRamSetup:       Int = 2,
                  dataRamExtraHold:   Boolean = false,
                  dataRamLatency:     Int = 2,
                  // ------------------------ Directory  ----------------------- //
                  // Replace Policy is PLRU
                  llcWays:            Int = 16, // self llc ways
                  sfWays:             Int = 16, // snoop filter ways
                  // Dir SRAM
                  nrDirBank:          Int = 2,
                  dirRamSetup:        Int = 1,
                  dirRamExtraHold:    Boolean = false,
                  dirRamLatency:      Int = 2,
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
  lazy val nrCommit  = nrPoS - (posSets * 2)
  // Data
  lazy val nrDataBuf = dataBufSizeInByte / BeatByte
  lazy val dsSets    = llcSets * llcWays

  require(llcSizeInB >= CacheLine * llcWays, s"illegal llc size: ${llcSizeInB}B")
  require(sfSizeInB  >= CacheLine * llcWays, s"illegal llc size: ${sfSizeInB}B")
  require(posSets <= llcSets,   s"illegal pos size: posSets($posSets) = nrPoS($nrPoS) / posWays($posWays) <= llcSets = $llcSets")
  require(posSets <= sfSets,    s"illegal pos size: posSets($posSets) = nrPoS($nrPoS) / posWays($posWays) <= sfSets = $sfSets")
  require(llcSets >  nrDirBank, s"illegal llc sets: llcSets($llcSets) = llcSizeInB($llcSizeInB) / CacheLine($CacheLine) / llcWays($llcWays) > nrDirBank = $nrDirBank")
  require(sfSets  >  nrDirBank, s"illegal sf sets : sfSets($sfSets)   = sfSizeInB($sfSizeInB)   / CacheLine($CacheLine) / llcWays($llcWays) > nrDirBank = $nrDirBank")
  require(dsSets  >  nrDSBank,  s"illegal ds sets : dsSets($dsSets)   = llcSets($llcSets) * llcWays($llcWays) > nrDirBank = $nrDirBank")
  require(posWays >= 4)
  require(isPow2(dataBufSizeInByte))
  require(isPow2(nrDSBank))
  require(isPow2(nrDirBank))
  require(isPow2(llcSets))
  require(isPow2(sfSets))
  require(isPow2(llcWays))
  require(isPow2(sfWays))
}


trait HasParseZJParam extends HasZJParams {
  // Has HPR
  lazy val hasHPR           = zjParams.hasHprRing

  // Get LAN Nodes
  lazy val lanCcNodes       = zjParams.island.filter(_.nodeType == NodeType.CC)
  lazy val lanRniNodes      = zjParams.island.filter(n => n.nodeType == NodeType.RI || n.nodeType == NodeType.RH)
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
  lazy val nrCI             = 1 << ciIdBits
  lazy val nrSfMetas        = if(hasBBN) nrCcNode + (nrCI - 1) else nrCcNode
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
    val fromXSeq = Cat(nodeIdSeq.map(_.asUInt >> lanABits === nodeId >> lanABits))
    HardwareAssertion(PopCount(fromXSeq) <= 1.U)
    fromXSeq.orR
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
  require(hnxBankOff > offsetBits - 1)
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
  lazy val ci_hi            = addrBits - 1
  lazy val ci_lo            = addrBits - ciBits
  // dirBank
  lazy val dirBank_ua_hi    = dirBankBits - 1
  lazy val dirBank_ua_lo    = 0
  // llcTag(per dirBank)
  lazy val llcTag_ua_hi     = useAddrBits - 1
  lazy val llcTag_ua_lo     = useAddrBits - llcTagBits
  // llcSet(per dirBank)
  lazy val llcSet_ua_hi     = dirBankBits + llcSetBits - 1
  lazy val llcSet_ua_lo     = dirBankBits
  require(llcSet_ua_hi >= llcSet_ua_lo, s"$llcSet_ua_hi = $dirBankBits + $llcSetBits - 1 >= $dirBankBits")
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
  def getCI       (a: UInt) = a(ci_hi, ci_lo)
  def getUseAddr  (a: UInt) = if(hnxBankOff == offsetBits) a(useAddr_hi, bankId_hi + 1) else Cat(a(useAddr_hi, bankId_hi + 1), a(bankId_lo - 1, useAddr_lo))
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

  // HnTxnID
  lazy val hnTxnIDBits      = dirBankBits + posSetBits + posWayBits
  // dirBank
  lazy val dirBank_hn_hi    = hnTxnIDBits - 1
  lazy val dirBank_hn_lo    = hnTxnIDBits - dirBankBits
  // posSet
  lazy val posSet_hn_hi     = posWayBits + posSetBits - 1
  lazy val posSet_hn_lo     = posWayBits
  // posWay
  lazy val posWay_hn_hi     = posWayBits - 1
  lazy val posWay_hn_lo     = 0

  // Data Parameters
  lazy val DataBits         = djparam.CacheLine * 8
  lazy val BeatBits         = djparam.BeatByte * 8
  lazy val MaskBits         = djparam.BeatByte
  lazy val ChiFullSize      = log2Ceil(djparam.CacheLine) // 6
  lazy val beatSize         = log2Ceil(djparam.BeatByte)  // 5
  require(MaskBits == zjParams.beBits)

  // Frontend(Per dirBank) and Directory Parameters
  lazy val nrHprTaskBuf     = (djparam.nrHprTaskBuf / djparam.nrDirBank).max(2)
  lazy val nrReqTaskBuf     = (djparam.nrReqTaskBuf / djparam.nrDirBank).max(2)
  lazy val nrSnpTaskBuf     = if(hasBBN) (djparam.nrReqTaskBuf / djparam.nrDirBank).max(2) else 0
  lazy val nrPoS            = djparam.nrPoS / djparam.nrDirBank
  lazy val nrCommit         = djparam.nrCommit / djparam.nrDirBank
  lazy val posWays          = djparam.posWays
  lazy val posWayBits       = log2Ceil(posWays)
  lazy val dirMuticycle     = djparam.dirRamLatency.max(if(djparam.dirRamExtraHold) djparam.dirRamSetup + 1 else djparam.dirRamSetup)
  lazy val readDirLatency   = (if(djparam.dirRamSetup > 1 || djparam.dirRamExtraHold) djparam.dirRamSetup + 1 else djparam.dirRamSetup) + djparam.dirRamLatency + 1
  lazy val llcWayBits       = log2Ceil(djparam.llcWays)

  // DataBlock Parameters
  // dc/db
  lazy val nrDataCM         = (djparam.nrDataBuf / 2).max(2)
  lazy val dbIdBits         = log2Ceil(djparam.nrDataBuf)
  lazy val dcIdBits         = log2Ceil(nrDataCM)
  //ds
  lazy val dsBank           = djparam.nrDSBank
  lazy val nrDsSet          = djparam.dsSets / dsBank
  lazy val dsIdxBits        = log2Ceil(nrDsSet)
  lazy val dsMuticycle      = djparam.dataRamLatency.max(if(djparam.dataRamExtraHold) djparam.dataRamSetup + 1 else djparam.dataRamSetup)
  lazy val readDsLatency    = (if(djparam.dataRamSetup > 1) djparam.dataRamSetup + 1 else djparam.dataRamSetup) + djparam.dataRamLatency

  // Backend Parameters
  lazy val nrReplaceCM      = (djparam.nrPoS / 2).max(2)
  lazy val nrTaskCM         = 3 // SnoopCM + ReadCM + WriteCM // TODO: + DatalessCM
  lazy val nrSnoopCM        = (djparam.nrPoS / 4).max(2)
  lazy val nrReadCM         = if (djparam.nrReadCM > 0) djparam.nrReadCM else (djparam.nrPoS / 2).max(2)
  lazy val nrDatalessCM     = (djparam.nrPoS / 4).max(2)
  lazy val nrWriteCM        = (djparam.nrPoS / 4).max(2)
  lazy val nrReceiveCM      = (djparam.nrPoS / 4).max(2)

  // TIMEOUT CHECK CNT VALUE
  lazy val TIMEOUT_TASKBUF  = 30000 * 4
  lazy val TIMEOUT_DATACM   = 30000 * 4
  lazy val TIMEOUT_POS      = 20000 * 4
  lazy val TIMEOUT_LOCK     = 20000 * 4
  lazy val TIMEOUT_COMMIT   = 18000 * 4
  lazy val TIMEOUT_REPLACE  = 15000 * 4
  lazy val TIMEOUT_SNP      = 10000 * 4
  lazy val TIMEOUT_READ     = 10000 * 4
  lazy val TIMEOUT_WRITE    = 10000 * 4
}


abstract class DJModule(implicit val p: Parameters) extends Module with HasDJParam {
  override def resetType: Module.ResetType.Type = Module.ResetType.Asynchronous
  def isTopModule = false
  /*
   * Print message
   */
  if(isTopModule) {
    // full addr
    val sUseAddr1 = if (hnxBankOff != 6) s"[useAddr1(${useAddr_hi}:${bankId_hi + 1})] + " else s"[useAddr(${useAddr_hi}:${bankId_hi + 1})] + "
    val sUseAddr0 = if (hnxBankOff != 6) s"[useAddr0(${bankId_lo - 1}:${useAddr_lo})] + " else s""
    val sBankId = if (nrBank != 0) s"[bankId(${bankId_hi}:${bankId_lo})] + " else s""
    val sFullAddr = s"[fullAddr(${djparam.addressBits - 1}:0)] = " + sUseAddr1 + sBankId + sUseAddr0 + s"[offset(${offset_hi}:${offset_lo})]"
    // dir bank id
    val sDirBankId = if (djparam.nrDirBank > 1) s"+ [dirBank(${dirBank_ua_hi}:${dirBank_ua_lo})]" else s""
    // pos set
    val sPosSet = if (posSets > 1) s"+ [posSet(${posSet_hn_hi}:${posSet_hn_lo})]" else s""
    //
    print(
      s"""
         |DongJiang Info: {
         |  Support Protocol: CHI-G
         |  lanPortNum: ${nrLanIcn}
         |  bbnPortNum: ${nrBbnIcn}
         |  llcSize: ${djparam.llcSizeInB} B = ${djparam.llcSizeInB / 1024} KiB
         |  sfSize: ${djparam.sfSizeInB} B = ${djparam.sfSizeInB / 1024} KiB
         |  llcWays: ${djparam.llcWays}
         |  sfWays: ${djparam.sfWays}
         |  sfMetas: ${nrSfMetas}
         |  openDCT: ${djparam.openDCT}
         |  nrReqBuf: ${nrReqTaskBuf * djparam.nrDirBank}
         |  nrHprBuf: ${nrHprTaskBuf * djparam.nrDirBank}
         |  nrSnpBuf: ${nrSnpTaskBuf * djparam.nrDirBank}
         |  nrPoS: ${djparam.nrPoS} = dirBank[${djparam.nrDirBank}] x posSets[${posSets}] x posWays[${posWays}]
         |  dataBufSize: ${djparam.dataBufSizeInByte} B = 32 B x ${djparam.dataBufSizeInByte / 32} Enrties
         |  dataSetup: ${djparam.dataRamSetup}
         |  dataLatency: ${djparam.dataRamSetup}
         |  dataExtraHold: ${djparam.dataRamExtraHold}
         |  dirSetup: ${djparam.dirRamSetup}
         |  dirLatency: ${djparam.dirRamLatency}
         |  dirExtraHold: ${djparam.dirRamExtraHold}
         |  ccNodeIdSeq: $ccNodeIdSeq
         |  rniNodeIdSeq: $rniNodeIdSeq
         |  snNodeIdSeq: $snNodeIdSeq
         |  address slice:
         |    $sFullAddr
         |                     = [ci(${ci_hi}:${ci_lo})]
         |    [useAddr(${useAddrBits - 1}:0)]  = [llcTag(${llcTag_ua_hi}:${llcTag_ua_lo})] + [llcSet(${llcSet_ua_hi}:${llcSet_ua_lo})] $sDirBankId
         |                     = [sfTag (${sfTag_ua_hi}:${sfTag_ua_lo})] + [sfSet (${sfSet_ua_hi}:${sfSet_ua_lo})] $sDirBankId
         |                     = [posTag(${posTag_ua_hi}:${posTag_ua_lo})] $sPosSet $sDirBankId
         |  hnTxnID slice:
         |    [hnTxnID(${hnTxnIDBits - 1}:0)]   = [dirBank(${dirBank_hn_hi}:${dirBank_hn_lo})] $sPosSet + [posWay(${posWay_hn_hi}:${posWay_hn_lo})]
         |  decodeTableSize: ${l_ci * l_si * l_ti} = ChiInst($l_ci) x StateInst($l_si) x TaskInst($l_ti) x SecTaskInst($l_sti)
         |}
         |""".stripMargin)
  }
}

abstract class DJBundle(implicit val p: Parameters) extends Bundle with HasDJParam

abstract class DJRawModule(implicit val p: Parameters) extends RawModule with HasDJParam
