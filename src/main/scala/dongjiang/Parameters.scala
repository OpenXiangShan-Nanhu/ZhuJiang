package dongjiang

import dongjiang.chi._
import dongjiang.pcu.IncoID
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xijiang.NodeType
import zhujiang.{HasZJParams, ZJParametersKey}
import scala.math.{max, min}
import xs.utils.debug.{DomainInfo, HardwareAssertion}


// Node Interface Params, used for generation
case class InterfaceParam
(
  // BASE
  name:     String, // RNSLAVE / RNMASTER / SNMASTER
  intfID:   Int,
  isRn:     Boolean,
  isSlave:  Boolean,
  nrEntry:  Int = 16,
  nrEvictEntry: Int = 4 // Use In RN
) {
  lazy val entryIdBits  = log2Ceil(nrEntry)
  lazy val isSn         = !isRn
  lazy val isMaster     = !isSlave
  lazy val hasReq2exu   = isRn
  lazy val hasDBRCReq   = isMaster
}


case class DJParam(
                  // -------------------------- Base Mes ---------------------- //
                  addressBits:        Int = 48,
                  // ------------------------- Interface Mes -------------------- //
                  localRnSlaveIntf:   InterfaceParam = InterfaceParam( name = "RnSalve_LOCAL",  intfID = IncoID.LOCALSLV, isRn = true,   isSlave = true,   nrEntry = 32, nrEvictEntry = 8),
                  localSnMasterIntf:  InterfaceParam = InterfaceParam( name = "SnMaster_LOCAL", intfID = IncoID.LOCALMST, isRn = false,  isSlave = false,  nrEntry = 16),
                  csnRnSlaveIntf:     Option[InterfaceParam] = None,
                  csnRnMasterIntf:    Option[InterfaceParam] = None,
                  openDCT:            Boolean = true,
                  openDMT:            Boolean = true,
                  // ------------------------ DCU Base Mes Per Bank ------------------ //
                  nrDSBank:           Int = 4,
                  nrDCUWBuf:          Int = 8,
                  nrDCURBuf:          Int = 4,
                  nrDCURespQ:         Int = 4,
                  dcuSetup:           Int = 3,
                  dcuLatency:         Int = 3,
                  dcuExtraHold:       Boolean = false,
                  // --------------------------- Data Buffer Base Mes ------------------- //
                  nrDatBuf:           Int = 32, // The number of Data Buffer entries Per PCU
                  nrDatBufRsvd:       Int = 4, // The number of Data Buffer entries reserved
                  nrAPU:              Int = 8, // The number of Atomic Process entries Per PCU
                  // ------------------------ EXU Base Mes Per Bank ------------------ //
                  nrPipeTaskQueue:    Int = 4,
                  nrExuReqQueue:      Int = 4,
                  nrExuRespQueue:     Int = 4,
                  // MSHR
                  nrMSHRSets:         Int = 4,
                  // ------------------------ Directory Mes Per Bank ------------------ //
                  // self dir & ds mes, dont care when hasLLC is false
                  selfWays:           Int = 16,
                  selfSets:           Int = 4096, // The number of Self Directory sets Per EXU
                  selfReplacementPolicy: String = "plru",
                  // snoop filter dir mes
                  sfDirWays:          Int = 16,
                  sfDirSets:          Int = 2048, // The number of Snoop Filter Directory Sets Per EXU
                  sfReplacementPolicy: String = "plru",
                  // DIR SRAM
                  nrDirBank:          Int = 4, // The number of Self / Snoop Filter Directory Bank Per EXU
                  dirSetup:           Int = 2,
                  dirLatency:         Int = 2,
                  dirExtraHold:       Boolean = false,
                ) {
  val nrMSHRWays = min(selfWays, sfDirWays)
  require(nrAPU <= nrDatBuf)
  require(nrDatBufRsvd <= nrDatBuf)
  require(min(selfSets, sfDirSets) >= nrMSHRSets)
  require(isPow2(nrDirBank))
  require(isPow2(nrMSHRSets))
  require(nrDatBuf <= nrDatBuf)
  require(nrDCURespQ >= 0)
  require(nrPipeTaskQueue > 0)
  require(nrExuReqQueue > 0)
  require(nrExuRespQueue > 0)
  require(selfReplacementPolicy == "random" || selfReplacementPolicy == "plru")
  require(sfReplacementPolicy == "random" || sfReplacementPolicy == "plru")
}


trait HasParseZJParam extends HasZJParams {
  lazy val cacheableBits    = 1

  // Get Nodes
  lazy val localCcNodes     = zjParams.localRing.filter(_.nodeType == NodeType.CC)
  lazy val localRniNodes    = zjParams.localRing.filter(_.nodeType == NodeType.RI)
  lazy val localHnfNodes    = zjParams.localRing.filter(_.nodeType == NodeType.HF)
  lazy val localDcuNodes    = zjParams.localRing.filter(_.nodeType == NodeType.S).filter(!_.mainMemory)
  lazy val localDDRCNode    = zjParams.localRing.filter(_.mainMemory).head
  require(localCcNodes.nonEmpty)
  require(localHnfNodes.nonEmpty)
  require(zjParams.localRing.filter(_.mainMemory).length == 1)

  // TODO: Support CSN
  lazy val hasCSN           = false
  lazy val ccxChipBits      = 3

  // CHI Signals Width
  lazy val fullNodeIdBits   = zjParams.nodeIdBits
  lazy val useNodeIdBits    = nodeNidBits
  lazy val opcodeBits       = 7

  // DCU Freiends Node ID Max Length
  lazy val nrFriendsNodeMax = localDcuNodes.map(_.friends.length).max

  // Local Base Node Mes
  // [bank] = [dcuBank] + [pcuBank]
  lazy val nrBank           = localDcuNodes.map(_.bankId).max + 1
  lazy val nrHnf            = localHnfNodes.length
  lazy val nrBankPerPCU     = nrBank / nrHnf
  lazy val pcuBankBits      = log2Ceil(nrHnf)
  lazy val dcuBankBits      = log2Ceil(nrBankPerPCU)
  lazy val fullBankBits     = pcuBankBits + dcuBankBits
  lazy val nrCcNode         = localCcNodes.length
  lazy val ccNodeIdBits     = log2Ceil(nrCcNode)
  lazy val metaIdBits       = ccNodeIdBits
  require(nrBank >= localHnfNodes.length)
  require(useNodeIdBits > metaIdBits)
  require(log2Ceil(nrBank) == fullBankBits)
  require(localHnfNodes.map(_.bankId).max + 1 == nrHnf)

  lazy val ccNodeIdSeq      = localCcNodes.map(_.nodeId)
  lazy val rniNodeIdSeq     = localRniNodes.map(_.nodeId)
  lazy val dcuNodeIdSeq     = localDcuNodes.map(_.nodeId)
  lazy val ddrcNodeId       = localDDRCNode.nodeId

  /*
   * Check From X Node
   */
  def fromXNode(x: UInt, nodeIdSeq: Seq[Int]): Bool = {
    require(x.getWidth == fullNodeIdBits | x.getWidth == useNodeIdBits)
    val fromX = WireInit(false.B)
    if(x.getWidth == fullNodeIdBits) {
      fromX := nodeIdSeq.map(_.asUInt >> nodeAidBits === x >> nodeAidBits).reduce(_ | _)
    } else {
      fromX := nodeIdSeq.map(_.asUInt >> nodeAidBits === x).reduce(_ | _)
    }
    fromX
  }
  def fromCcNode  (x: UInt): Bool = fromXNode(x, ccNodeIdSeq)
  def fromRniNode (x: UInt): Bool = fromXNode(x, rniNodeIdSeq)
  def fromRnNode  (x: UInt): Bool = fromCcNode(x) | fromRniNode(x)
  def fromDcuNode (x: UInt): Bool = fromXNode(x, dcuNodeIdSeq)
  def fromDDRCNode(x: UInt): Bool = fromXNode(x, Seq(ddrcNodeId))
  def fromSnNode  (x: UInt): Bool = fromDcuNode(x) | fromDDRCNode(x)


  /*
   * Get Node ID or Meta ID
   */
  def getUseNodeID(x: UInt) = {
    require(x.getWidth == fullNodeIdBits)
    x(nodeNidBits - 1, nodeAidBits)
  }

  def getFullNodeID(x: UInt) = {
    require(x.getWidth == useNodeIdBits)
    val aID = WireInit(0.U(nodeAidBits.W))
    when(fromCcNode(x) | fromRniNode(x)) { aID := 1.U }
    Cat(0.U(nodeNetBits.W), x, aID)
  }

  def getFriendDcuIDByDcuBankID(x: UInt, friendIdSeq: Seq[UInt]): UInt = {
    // Check dcuBankID when use it: assert(x < nrBankPerPCU.U)
    val nodeID = WireInit(0.U(fullNodeIdBits.W))
    friendIdSeq.zipWithIndex.foreach { case(id, i) => when(x === i.U) { nodeID := id } }
    nodeID
  }

  def getMetaIDByNodeID(x: UInt): UInt = {
    // Check nodeID whe use it: assert(fromCcNode(x))
    require(x.getWidth == useNodeIdBits | x.getWidth == fullNodeIdBits)
    val useNodeId = if(x.getWidth == useNodeIdBits) x else getUseNodeID(x)
    val metaId    = WireInit(0.U(metaIdBits.W))
    ccNodeIdSeq.zipWithIndex.foreach { case (id, i) => when(useNodeId === getUseNodeID(id.U.asTypeOf(UInt(fullNodeIdBits.W)))) { metaId := i.U } }
    metaId
  }

  def getNodeIDByMetaID(x: UInt) = {
    // Check metaID when use it: assert(x < nrCcNode.U)
    val nodeID = WireInit(0.U(fullNodeIdBits.W))
    ccNodeIdSeq.zipWithIndex.foreach { case (id, i) => when(x === i.U) { nodeID := id.U } }
    nodeID
  }

  def getDCUDirectByTgtID(x: UInt, friendsVec: Seq[Seq[UInt]]) = {
    require(x.getWidth == fullNodeIdBits)
    val directVec = Wire(Vec(friendsVec.length, Bool()))
    friendsVec.zipWithIndex.foreach {
      case(f, i) =>
        directVec(i) := f.map { case id => id === x | (id + 1.U) === x }.reduce(_ | _)
        HardwareAssertion(PopCount(f.map(_ === x)) <= 1.U)
    }
    directVec
  }
}


trait HasDJParam extends HasParseZJParam {
  val p: Parameters
  val djparam = p(ZJParametersKey).djParams

  lazy val chiTxnIdBits = 12

  // Data Mes Parameters
  lazy val blockBytes       = 64 // cache line bytes
  lazy val beatBytes        = 32
  lazy val nrBeat           = 2
  lazy val dataBits         = blockBytes * 8
  lazy val beatBits         = beatBytes * 8
  lazy val maskBits         = beatBytes
  lazy val chiFullSize      = log2Ceil(blockBytes)
  lazy val chiHalfSize      = log2Ceil(beatBytes)
  lazy val chiSizeWidth     = log2Ceil(chiFullSize)


  // Base Mes Parameters
  // [fullAddr] = [cacheable] + [ccxChipID] + [useAddr1] + [bankID] + [useAddr0] + [offset]
  // [useAddr]  = [useAddr1] + [useAddr0]
  lazy val offsetBits       = log2Ceil(blockBytes)
  lazy val fullAddrBits     = djparam.addressBits
  lazy val useAddrBits      = fullAddrBits - cacheableBits - ccxChipBits - fullBankBits  - offsetBits// need to check input pcu addr unuse bits is 0 expect bankBits
  lazy val dirBankBits      = log2Ceil(djparam.nrDirBank)
  require(isPow2(nrBeat))
  require(bankOff + fullBankBits - 1 < fullAddrBits - (cacheableBits + ccxChipBits))
  require(bankOff > offsetBits)


  // Base Interface Mes
  require(djparam.csnRnSlaveIntf.nonEmpty  | !hasCSN)
  require(djparam.csnRnMasterIntf.nonEmpty | !hasCSN)
  lazy val intfMesSeq       = if(hasCSN) Seq(djparam.localRnSlaveIntf, djparam.localSnMasterIntf, djparam.csnRnSlaveIntf.get, djparam.csnRnMasterIntf.get)
                              else       Seq(djparam.localRnSlaveIntf, djparam.localSnMasterIntf)
  lazy val nrIntf           = intfMesSeq.length
  lazy val nrIntfBits       = log2Ceil(nrIntf)
  lazy val nrIntfEntryMax   = intfMesSeq.map(_.nrEntry).max
  lazy val intfEntryIdBits  = log2Ceil(nrIntfEntryMax)
  require(intfEntryIdBits <= chiTxnIdBits)

  // Base DCU Mes
  lazy val nrPerDCUEntry    = djparam.selfSets * djparam.selfWays
  lazy val nrDSEntry        = nrPerDCUEntry / djparam.nrDSBank
  // [dcuIndex] = [secBeat] + [sSet] + [dirBank] + [sWay] = [secBeat] + [dsIndex] + [dsBank]
  // Bank will be transfer to EREQ TgtID
  lazy val dcuIndexBits     = log2Ceil(nrPerDCUEntry) + 1
  lazy val dsIndexBits      = log2Ceil(nrDSEntry)
  lazy val dsBankBits       = log2Ceil(djparam.nrDSBank)
  require(dcuIndexBits == (dsIndexBits + dsBankBits + 1))


  // DataBuffer entry Id Bits
  lazy val dbIdBits         = log2Ceil(djparam.nrDatBuf)
  lazy val apuIdBits        = log2Ceil(djparam.nrAPU)
  lazy val nrDBWithoutAPUs  = djparam.nrDatBuf - djparam.nrAPU
  require(dbIdBits <= chiTxnIdBits)

  // SELF DIR Parameters: [useAddr] = [sTag] + [sSet] + [dirBank]
  lazy val sWayBits         = log2Ceil(djparam.selfWays)
  lazy val sSetBits         = log2Ceil(djparam.selfSets /djparam.nrDirBank)
  lazy val sTagBits         = useAddrBits - sSetBits - dirBankBits
  require(sSetBits + dirBankBits + sWayBits == dcuIndexBits - 1)

  // SF DIR Parameters: [useAddr] = [sfTag] + [sfSet] + [dirBank]
  lazy val sfWayBits        = log2Ceil(djparam.sfDirWays)
  lazy val sfSetBits        = log2Ceil(djparam.sfDirSets / djparam.nrDirBank)
  lazy val sfTagBits        = useAddrBits - sfSetBits - dirBankBits

  // DIR SET MAX
  lazy val nrMinDirSet      = min(djparam.selfSets, djparam.sfDirSets)
  lazy val nrMaxDirSet      = max(djparam.selfSets, djparam.sfDirSets)
  lazy val minDirSetBits    = log2Ceil(nrMinDirSet)
  lazy val maxDirSetBits    = log2Ceil(nrMaxDirSet)

  // MSHR TABLE Parameters: [useAddr] = [mshrTag] + [mshrSet]
  lazy val mshrWayBits      = log2Ceil(djparam.nrMSHRWays)
  lazy val mshrSetBits      = log2Ceil(djparam.nrMSHRSets)
  lazy val mshrTagBits      = useAddrBits - mshrSetBits

  // replacement Parameters
  lazy val sReplWayBits     = if(djparam.selfReplacementPolicy != "random") djparam.selfWays - 1 else 0
  lazy val sfReplWayBits    = if(djparam.sfReplacementPolicy != "random") djparam.sfDirWays - 1 else 0
  require(djparam.selfReplacementPolicy == "random" | djparam.selfReplacementPolicy == "plru", "It should modify sReplWayBits when use replacement except of random or plru")
  require(djparam.sfReplacementPolicy == "random" | djparam.sfReplacementPolicy == "plru", "It should modify cReplWayBits when use replacement except of random or plru")


  // TIMEOUT CHECK CNT VALUE
  lazy val TIMEOUT_MSHR       = 20000 + 10000 // MSHR
  lazy val TIMEOUT_DB         = 10000 + 10000 // DataBuffer
  lazy val TIMEOUT_RSINTF_SNP =  2000 + 10000 // Rn Slave Intf Snp
  lazy val TIMEOUT_RSINTF_REQ = 20000 + 10000 // Rn Slave Intf Req
  lazy val TIMEOUT_RSINTF_RSP =  8000         // Rn Slave Intf Req
  lazy val TIMEOUT_SMINTF     =  2000 + 10000 // Sn Master Intf
  lazy val TIMEOUT_RMINTF     =         10000 // Rn Master Intf
  lazy val TIMEOUT_PIPEEXU    =  3000         // Pipe Execute
  lazy val TIMEOUT_DCU_R      = 10000         // DCU Read Buffer
  lazy val TIMEOUT_DCU_W      = 10000         // DCU Write Buffer

  def parseFullAddr(x: UInt): (UInt, UInt, UInt, UInt, UInt, UInt) = {
    require(x.getWidth == fullAddrBits)
    val offset    = x
    val pcuBank   = x       >> bankOff
    val dcuBank   = pcuBank >> pcuBankBits
    val ccxChipID = x       >> fullAddrBits - (ccxChipBits + cacheableBits)
    val cacheable = x       >> fullAddrBits - cacheableBits
    val useAddr   = Cat(x(fullAddrBits - (ccxChipBits + cacheableBits) - 1, bankOff + fullBankBits), x(bankOff - 1, offsetBits)); require(useAddr.getWidth == useAddrBits)
    // return: [1:cacheable] [2:ccxChipID] [3:dcuBank] [4:pcuBank] [5:offset] [6:useAddr]
    (cacheable(cacheableBits - 1, 0), ccxChipID(ccxChipBits - 1, 0), dcuBank(dcuBankBits - 1, 0), pcuBank(pcuBankBits - 1, 0), offset(offsetBits - 1, 0), useAddr)
  }

  def getFullAddr(x: UInt, dcuBankID: UInt, pcuBankID: UInt, offset: UInt): UInt = {
    require(x.getWidth == useAddrBits)
    require(dcuBankID.getWidth == dcuBankBits)
    require(pcuBankID.getWidth == pcuBankBits)
    val addr0     = x(bankOff - offsetBits - 1, 0)
    val addr1     = x(useAddrBits - 1, bankOff - offsetBits)
    val ccxChipID = 0.U(cacheableBits.W)
    val cacheable = 0.U(ccxChipBits.W)
    val fullAddr  = Cat(cacheable, ccxChipID, addr1, dcuBankID, pcuBankID, addr0, offset)
    require(fullAddr.getWidth == fullAddrBits)
    fullAddr
  }

  def parseSelfAddr(x: UInt): (UInt, UInt, UInt) = {
    require(x.getWidth == useAddrBits)
    val dirBank = x
    val sSet    = dirBank   >> dirBankBits
    val sTag    = sSet      >> sSetBits
    // return: [1:sTag] [2:sSet] [3:dirBank]
    (sTag(sTagBits - 1, 0), sSet(sSetBits - 1, 0), dirBank(dirBankBits - 1, 0))
  }

  def parseSFAddr(x: UInt): (UInt, UInt, UInt) = {
    require(x.getWidth == useAddrBits)
    val dirBank = x
    val sfSet   = dirBank   >> dirBankBits
    val sfTag   = sfSet     >> sfSetBits
    // return: [1:sfTag] [2:sfSet] [3:dirBank]
    (sfTag(sfTagBits - 1, 0), sfSet(sfSetBits - 1, 0), dirBank(dirBankBits - 1, 0))
  }

  def parseMSHRAddr(x: UInt): (UInt, UInt) = {
    require(x.getWidth == useAddrBits)
    val mshrSet   = x
    val mshrTag   = x       >> mshrSetBits
    // return: [1:mshrTag] [2:mshrSet]
    (mshrTag(mshrTagBits - 1, 0), mshrSet(mshrSetBits - 1, 0))
  }

  def getDirBank(x: UInt): UInt = {
    require(x.getWidth == useAddrBits)
    x(dirBankBits - 1, 0)
  }

  def getDCUAddress(secBeat: Bool, sSet: UInt, dirBank: UInt, sWay:UInt): UInt = {
    require(sSet.getWidth == sSetBits)
    require(dirBank.getWidth == dirBankBits)
    require(sWay.getWidth == sWayBits)
    val dcuAddr = Cat(secBeat.asUInt, sSet, dirBank, sWay)
    require(dcuAddr.getWidth == dcuIndexBits, s"${dcuAddr.getWidth} = ${sSet.getWidth} + ${dirBank.getWidth} + ${sWay.getWidth} =/= $dcuIndexBits")
    dcuAddr
  }

  def parseDCUAddr(x: UInt): (UInt, UInt, UInt) = {
    require(x.getWidth == fullAddrBits)
    val dsBank  = x
    val dsIndex = dsBank    >> dsBankBits
    val beatOff = dsIndex   >> dsIndexBits
    // return: [1: beatOff] [2:dsIndex] [3:dsBank]
    (beatOff(0) ,dsIndex(dsIndexBits - 1, 0), dsBank(dsBankBits - 1, 0))
  }

  def toDataID(x: UInt): UInt = {
    require(nrBeat == 1 | nrBeat == 2 | nrBeat == 4)
    if (nrBeat == 1) { "b00".U }
    else if (nrBeat == 2) { Mux(x === 0.U, "b00".U, "b10".U) }
    else if (nrBeat == 4) { x }
    else { 0.U }
  }

  def toBeatNum(x: UInt): UInt = {
    if (nrBeat == 1) { assert(x === "b00".U); 0.U }
    else if (nrBeat == 2) { assert(x === "b00".U | x === "b10".U); Mux(x === "b00".U, 0.U, 1.U) }
    else if (nrBeat == 4) { x }
    else { 0.U }
  }
}


abstract class DJModule(implicit val p: Parameters) extends Module with HasDJParam

abstract class DJBundle(implicit val p: Parameters) extends Bundle with HasDJParam

abstract class DJRawModule(implicit val p: Parameters) extends RawModule with HasDJParam
