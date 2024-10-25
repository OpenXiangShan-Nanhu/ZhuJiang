package dongjiang

import dongjiang.chi._
import dongjiang.pcu.ChipType
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xijiang.NodeType
import zhujiang.{HasZJParams, ZJParametersKey}

import scala.math.{max, min}


// Node Interface Params, used for generation
case class InterfaceParam
(
  // BASE
  name: String, // RNSLAVE / RNMASTER / SNMASTER
  isRn: Boolean,
  isSlave: Boolean,
  chipId: Option[Int] = None, // Use In MASTER
  nrEntry: Int = 16,
  nrEvictEntry: Int = 4 // Use In RN
) {
  lazy val entryIdBits = log2Ceil(nrEntry)
  lazy val isSn = !isRn
  lazy val isMaster = !isSlave
  lazy val hasReq2Slice = true
  lazy val hasDBRCReq = isMaster
  if(isMaster) require(chipId.nonEmpty) else require(chipId.isEmpty)
}


case class DJParam(
                  // -------------------------- Base Mes ---------------------- //
                  blockBytes: Int = 64,
                  beatBytes: Int = 32,
                  addressBits: Int = 48,
                  hasLLC: Boolean = true, // TODO
                  // ------------------------- Interface Mes -------------------- //
                  localRnSlaveIntf:   InterfaceParam =              InterfaceParam( name = "RnSalve_LOCAL",  isRn = true,   isSlave = true, nrEntry = 32, nrEvictEntry = 8),
                  localSnMasterIntf:  InterfaceParam =              InterfaceParam( name = "SnMaster_LOCAL", isRn = false,  isSlave = false, chipId = Some(0), nrEntry = 32),
                  csnRnSlaveIntf:     Option[InterfaceParam] = None, // Some(InterfaceParam( name = "RnSalve_CSN",    isRn = true,   isSlave = true)),
                  csnRnMasterIntf:    Option[InterfaceParam] = None, // Some(InterfaceParam( name = "RnMaster_CSN",   isRn = true,   isSlave = false, chipId = Some(1) )),
                  chiTxnidBits: Int = 12, // TODO
                  chiDBIDBits: Int = 16, // TODO
                  // --------------------------- Base Mes ------------------- //
                  // number of bank or buffer
                  nrBank: Int = 4, // TODO
                  nrDatBuf: Int = 32,
                  // ------------------------ DCU Base Mes Per Bank ------------------ //
                  nrDSBank: Int = 4,
                  nrDCUWBuf: Int = 16,
                  nrDCURBuf: Int = 16,
                  nrDCURespQ: Int = 4,
                  dcuMulticycle: Int = 2,
                  dcuHoldMcp: Boolean = true,
                  // ------------------------ CPU Base Mes Per Bank ------------------ //
                  nrMpTaskQueue: Int = 4,
                  nrMpReqQueue: Int = 4,
                  nrMpRespQueue: Int = 4,
                  // MSHR
                  nrMSHRSets: Int = 4,
                  // ------------------------ Directory Mes Per Bank ------------------ //
                  // self dir & ds mes, dont care when hasLLC is false
                  selfWays: Int = 16,
                  selfSets: Int = 4096,
                  selfReplacementPolicy: String = "plru",
                  // snoop filter dir mes
                  sfDirWays: Int = 16,
                  sfDirSets: Int = 2048,
                  sfReplacementPolicy: String = "plru",
                  // DIR SRAM
                  nrDirBank: Int = 4,
                  dirMulticycle: Int = 2,
                  dirHoldMcp: Boolean = true,
                ) {
  val nrMSHRWays = min(selfWays, sfDirWays) // TODO
  require(min(selfSets, sfDirSets) >= nrMSHRSets)
  require(isPow2(nrDirBank))
  require(isPow2(nrMSHRSets))
  require(nrMpTaskQueue > 0)
  require(nrMpReqQueue > 0)
  require(nrMpRespQueue > 0)
  require(nrBank == 1 | nrBank == 2 | nrBank == 4)
  require(selfReplacementPolicy == "random" || selfReplacementPolicy == "plru")
  require(sfReplacementPolicy == "random" || sfReplacementPolicy == "plru")
  require(log2Ceil(nrDCUWBuf) <= chiDBIDBits)
  require(nrDCURespQ >= 2)
}


trait HasParseZJParam extends HasZJParams {
  lazy val localRnfNode    = zjParams.localRing.filter(_.nodeType == NodeType.CC)
  lazy val localHnfNode    = zjParams.localRing.filter(_.nodeType == NodeType.HF)
  lazy val localSnNode     = zjParams.localRing.filter(_.nodeType == NodeType.S)
  lazy val hasCSN          = zjParams.csnRing.nonEmpty
  lazy val csnHnfNodeOpt   = if(hasCSN) Option(zjParams.csnRing.filter(_.nodeType == NodeType.HF).last) else None
  lazy val csnRnfNodeOpt   = if(hasCSN) Option(zjParams.csnRing.filter(_.nodeType == NodeType.RF).last) else None

  require(zjParams.localRing.count(_.nodeType == NodeType.HF) >= 1)
  require(localSnNode.last.mainMemory)
  localHnfNode.foreach { case h => require(h.splitFlit) }
  if(hasCSN) {
    require(csnHnfNodeOpt.get.splitFlit)
    require(csnRnfNodeOpt.get.splitFlit)
  }

  // CHI Node ID Width
  lazy val chiNodeIdBits   = zjParams.nodeIdBits

  // Local Base Node Mes
  lazy val nrBankPerDJ     = p(ZJParametersKey).djParams.nrBank / localHnfNode.length
  lazy val nrRnfNode       = zjParams.localRing.count(_.nodeType == NodeType.CC)
  lazy val rnfNodeIdBits   = log2Ceil(nrRnfNode)
  lazy val rnNodeIdSeq     = localRnfNode.map(_.nodeId)
  lazy val snNodeIdSeq     = localSnNode.map(_.nodeId)
  lazy val ddrcNodeId      = localSnNode.filter(_.mainMemory).map(_.nodeId).last
  lazy val hnfNodeIdSeq    = localHnfNode.map(_.nodeId)

  // CSN Base Node Mes
  lazy val csnHnfNodeId    = 0 // TODO

  def fromSnNode(x: UInt) = snNodeIdSeq.map(_.asUInt === x).reduce(_ | _)

  def getSnNodeIDByBankId(x: UInt, i: Int): UInt = {
    val nodeID = WireInit(0xfff.U)
    snNodeIdSeq.zipWithIndex.foreach {
      case(id, i) =>
        when(x === i.U) {
          nodeID := id.U
        }
    }
    assert(nodeID =/= 0xfff.U, "getSnNodeIDByBankId ERROR BankId[0x%x] Index[0x%x]", x, i.U)
    nodeID
  }

  def getMetaIdByNodeID(x: UInt): UInt = {
    val metaId = WireInit(0.U((rnfNodeIdBits + 1).W))
    rnNodeIdSeq.zipWithIndex.foreach {
      case (id, i) =>
        when(x === (id.U + 1.U)) {
          metaId := i.U
        }
    }
    // TODO:
    // assert(metaId =/= (nrRnfNode + 1).U, "getMetaIdByNodeID ERROR NodeID[0x%x]", x)
    metaId(rnfNodeIdBits - 1, 0)
  }

  def getNodeIDByMetaId(x: UInt, i: Int) = {
    val nodeID = WireInit(0xfff.U)
    rnNodeIdSeq.zipWithIndex.foreach {
      case (id, i) =>
        when(x === i.U) {
          nodeID := id.U + 1.U
        }
    }
    assert(nodeID =/= 0xfff.U, "getNodeIDByMetaId ERROR MetaId[0x%x] Index[0x%x]", x, i.U)
    nodeID(10, 0)
  }
}


trait HasDJParam extends HasParseZJParam {
  val p: Parameters
  val djparam = p(ZJParametersKey).djParams //TODO: use lazy val in all parameters

  // Base Mes Parameters
  lazy val nrBeat          = djparam.blockBytes / djparam.beatBytes
  lazy val beatNumBits     = log2Ceil(nrBeat)
  lazy val addressBits     = djparam.addressBits
  lazy val dataBits        = djparam.blockBytes * 8
  lazy val beatBits        = djparam.beatBytes * 8
  lazy val localChipId     = 0 // TODO
  lazy val csnChipId       = 1 // TODO
  require(isPow2(nrBeat))
  require(nrBankPerDJ * localHnfNode.length == djparam.nrBank)

  // Base Interface Mes
  lazy val hasCSNIntf      = djparam.csnRnSlaveIntf.nonEmpty & djparam.csnRnMasterIntf.nonEmpty
  lazy val interfaceMes    = if(hasCSNIntf) Seq(djparam.localRnSlaveIntf, djparam.localSnMasterIntf, djparam.csnRnSlaveIntf.get, djparam.csnRnMasterIntf.get)
                             else           Seq(djparam.localRnSlaveIntf, djparam.localSnMasterIntf)
  lazy val nrIntf          = interfaceMes.length
  lazy val nrSlvIntf       = interfaceMes.count(_.isSlave)
  lazy val nrMasIntf       = interfaceMes.count(_.isMaster)
  lazy val nrIntfBits      = log2Ceil(nrIntf)
  lazy val nrIntfEntryMax  = interfaceMes.map(_.nrEntry).max
  lazy val intfEntryIdBits = log2Ceil(nrIntfEntryMax)

  // Base DCU Mes
  lazy val dcuWBufIdBits   = log2Ceil(djparam.nrDCUWBuf)
  lazy val dcuRBufIdBits   = log2Ceil(djparam.nrDCURBuf)
  lazy val nrPerDCUEntry   = djparam.selfSets * djparam.selfWays
  lazy val nrDSEntry       = nrPerDCUEntry / djparam.nrDSBank
  // DCU Index = [sSet] + [dirBank] + [sWay] = [dsIndex] + [dsBank]
  // Bank -> EREQ TgtID
  lazy val dcuIndexBits    = log2Ceil(nrPerDCUEntry)
  lazy val dsIndexBits     = log2Ceil(nrDSEntry)
  lazy val dsBankBits      = log2Ceil(djparam.nrDSBank)
  require(dcuIndexBits == (dsIndexBits + dsBankBits))

  // Base Fake DDRC Mes
  lazy val nrDDRCBank      = dataBits / 64
  lazy val nrDDRCRBuf      = 8
  lazy val nrDDRCWBuf      = 8
  lazy val nrDDRCRespQ     = 8
  lazy val ddrcWBufIdBits  = log2Ceil(nrDDRCWBuf)

  // Slice Id Bits Parameters
  lazy val dbIdBits        = log2Ceil(djparam.nrDatBuf)

  // DIR BASE Parameters
  lazy val bankBits        = log2Ceil(djparam.nrBank)
  lazy val dirBankBits     = log2Ceil(djparam.nrDirBank)
  lazy val offsetBits      = log2Ceil(djparam.blockBytes)

  // SELF DIR Parameters: [sTag] + [sSet] + [dirBank] + [bank] + [offset]
  lazy val sWayBits        = log2Ceil(djparam.selfWays)
  lazy val sSetBits        = log2Ceil(djparam.selfSets /djparam.nrDirBank)
  lazy val sTagBits        = djparam.addressBits - sSetBits - dirBankBits - bankBits - offsetBits
  require(sSetBits + dirBankBits + sWayBits == dcuIndexBits)

  // SF DIR Parameters: [sfTag] + [sfSet] + [dirBank] + [bank] + [offset]
  lazy val sfWayBits       = log2Ceil(djparam.sfDirWays)
  lazy val sfSetBits       = log2Ceil(djparam.sfDirSets / djparam.nrDirBank)
  lazy val sfTagBits       = djparam.addressBits - sfSetBits - dirBankBits - bankBits - offsetBits

  // DIR SET MAX
  lazy val maxDirSetBits   = max(sSetBits, sfSetBits)

  // MSHR TABLE Parameters: [mshrTag] + [mshrSet] + [bank] + [offset]
  lazy val mshrWayBits     = log2Ceil(djparam.nrMSHRWays)
  lazy val mshrSetBits     = log2Ceil(djparam.nrMSHRSets)
  lazy val mshrTagBits     = djparam.addressBits - mshrSetBits - bankBits - offsetBits

  // replacement Parameters
  lazy val sReplWayBits    = if(djparam.selfReplacementPolicy != "random") djparam.selfWays - 1 else 0
  lazy val sfReplWayBits   = if(djparam.sfReplacementPolicy != "random") djparam.sfDirWays - 1 else 0
  require(djparam.selfReplacementPolicy == "random" | djparam.selfReplacementPolicy == "plru", "It should modify sReplWayBits when use replacement except of random or plru")
  require(djparam.sfReplacementPolicy == "random" | djparam.sfReplacementPolicy == "plru", "It should modify cReplWayBits when use replacement except of random or plru")

  // Node address id map check
  // TODO

  // TIMEOUT CHECK CNT VALUE
  lazy val TIMEOUT_DB      = 10000 + 10000 // DataBuffer
  lazy val TIMEOUT_MSHR    = 8000  + 10000 // BlockTable
  lazy val TIMEOUT_RSINTF  = 5000  + 10000 // Rn Slave Intf
  lazy val TIMEOUT_SMINTF  = 5000  + 10000 // Sn Master Intf
  lazy val TIMEOUT_RMINTF  = 5000  + 10000 // Rn Master Intf
  lazy val TIMEOUT_MSLOCK  = 3000  + 10000 // MSHR Lock
  lazy val TIMEOUT_EXU     = 3000  + 10000 // Pipe Execute

  // some requirements for CHI width
  require(intfEntryIdBits <= djparam.chiTxnidBits)
  require(dbIdBits <= djparam.chiDBIDBits)

  def getChipTypeByAddr(x: UInt): UInt = {
    val chipType = Wire(UInt(ChipType.width.W))
    when(x(46, 44) === localChipId.U) {
      chipType := ChipType.Local
    }.otherwise {
      chipType := ChipType.CSN
    }
    chipType
  }

  def parseAddress(x: UInt, modBankBits: Int = 1, setBits: Int = 1, tagBits: Int = 1): (UInt, UInt, UInt, UInt, UInt) = {
      val offset  = x
      val bank    = offset    >> offsetBits // TODO
      val modBank = bank      >> bankBits
      val set     = modBank   >> modBankBits
      val tag     = set       >> setBits
      // return: [1:tag] [2:set] [3:modBank] [4:bank] [5:offset]
      require(x.getWidth == addressBits)
      (tag(tagBits - 1, 0), set(setBits - 1, 0), modBank(modBankBits - 1, 0), bank(bankBits - 1, 0), offset(offsetBits - 1, 0))
  }

  def parseMSHRAddress(x: UInt): (UInt, UInt, UInt) = {
      val (tag, set, modBank, bank, offset) = parseAddress(x, modBankBits = 0, setBits = mshrSetBits, tagBits = mshrTagBits)
      require(sSetBits + dirBankBits >= mshrSetBits)
      // return: [1:mshrTag] [2:mshrSet] [3:bank]
      (tag, set, bank)
  }

  def parseDCUAddress(x: UInt): (UInt, UInt) = {
    val dsBank  = x
    val dsIndex = dsBank    >> dsBankBits
    // return: [1:dsIndex] [2:dsBank]
    (dsIndex(dsIndexBits - 1, 0), dsBank(dsBankBits - 1, 0))
  }


  def getDCUAddress(addr:UInt, sWay:UInt): UInt = {
      require(addr.getWidth == addressBits)
      require(sWay.getWidth == sWayBits)
      val (sTag, sSet, dirbank, bank, offset) = parseAddress(addr, dirBankBits, sSetBits, sTagBits)
      val dcuAddr = Cat(sSet, dirbank, sWay)
      require(dcuAddr.getWidth == dcuIndexBits, s"${dcuAddr.getWidth} = ${sSet.getWidth} + ${dirbank.getWidth} + ${sWay.getWidth} =/= $dcuIndexBits")
      dcuAddr
  }

  def getDirBank(x: UInt): UInt = parseAddress(x, dirBankBits, 0, 0)._3

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
