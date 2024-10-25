package dongjiang.pcu

import dongjiang._
import dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import scala.collection.immutable.ListMap
import scala.math.{max, min}

object ChipType {
    val width         = 1
    val Local         = "b0".U
    val CSN           = "b1".U
}

// ---------------------------------------------------------------- Xbar Id Bundle ----------------------------------------------------------------------------- //
// Interconnect ID
object IncoID {
    val width       = 2
    val LOCALSLV    = 0
    val LOCALMAS    = 1 // TODO: MAS -> MST
    val CSNSLV      = 2
    val CSNMAS      = 3 // TODO: MAS -> MST
}

class IDBundle(implicit p: Parameters) extends DJBundle {
    val IncoId      = UInt(max(bankBits, nrIntfBits).W)

    def intfId      = IncoId
    def bankId      = IncoId

    def LOCALSLV    = IncoId === IncoID.LOCALSLV.U
    def LOCALMAS    = IncoId === IncoID.LOCALMAS.U
    def CSNSLV      = IncoId === IncoID.CSNSLV.U
    def CSNMAS      = IncoId === IncoID.CSNMAS.U
}

trait HasFromIncoID extends DJBundle { this: Bundle => val from = new IDBundle() }

trait HasToIncoID extends DJBundle { this: Bundle => val to = new IDBundle() }

trait HasIncoID extends DJBundle with HasFromIncoID with HasToIncoID

trait HasDBID extends DJBundle { this: Bundle => val dbid = UInt(dbIdBits.W) }

trait HasAddr extends DJBundle {this: Bundle =>
    val addr        = UInt(addressBits.W);
    def bank        = parseAddress(addr)._4
    def mTag        = parseMSHRAddress(addr)._1
    def mSet        = parseMSHRAddress(addr)._2
    def mBank       = parseMSHRAddress(addr)._3
    def dirBank     = getDirBank(addr)
    def addrNoOff   = addr(addressBits - 1, offsetBits)
}

trait HasMSHRSet extends DJBundle { this: Bundle => val mshrSet = UInt(mshrSetBits.W) }

class MSHRSetBundle(implicit p: Parameters) extends DJBundle with HasMSHRSet

trait HasMSHRWay extends DJBundle { this: Bundle => val mshrWay = UInt(mshrWayBits.W) }

trait HasMHSRIndex extends DJBundle with HasMSHRSet with HasMSHRWay { def mshrMatch(set: UInt, way: UInt): Bool = mshrSet === set & mshrWay === way }

object PipeID { val width = 1; val RESP = "b0".U; val REQ = "b1".U }

trait HasPipeID extends Bundle { this: Bundle => val pipeId = UInt(PipeID.width.W); def toReqPipe = pipeId === PipeID.REQ; def toRespPipe = pipeId === PipeID.RESP }

trait HasPCUID extends DJBundle { this: Bundle => val pcuId = UInt(intfEntryIdBits.W) }

// ---------------------------------------------------------------- CHI Base Mes Bundle ----------------------------------------------------------------------------- //
// TODO: Check Unuse singnals
trait HasBaseCHIMesBundle extends DJBundle with HasCHIChannel { this: Bundle =>
    // CHI ID
    def useTgt          : Boolean = true
    def useSrc          : Boolean = true
    def useTxn          : Boolean = true
    val tgtIDOpt        = if(useTgt) Some(UInt(chiNodeIdBits.W)) else None
    val srcIDOpt        = if(useSrc) Some(UInt(chiNodeIdBits.W)) else None
    val txnIDOpt        = if(useTxn) Some(UInt(djparam.chiTxnidBits.W)) else None
    require(chiNodeIdBits >= nrRnfNode)
    def tgtID           = tgtIDOpt.get
    def snpMetaVec      = tgtID(nrRnfNode - 1 ,0)
    def srcID           = srcIDOpt.get
    def txnID           = txnIDOpt.get
    // Snp Mes(Use In Snp)
    def useSnp          : Boolean = true
    val doNotGoToSDOpt  = if(useSnp) Some(Bool()) else None
    val retToSrcOpt     = if(useSnp) Some(Bool()) else None
    def doNotGoToSD     = doNotGoToSDOpt.get
    def retToSrc        = retToSrcOpt.get
    // resp Mes(Use In Resp)
    def useResp         : Boolean = true
    val respOpt         = if(useResp) Some(UInt(ChiResp.width.W)) else None
    val fwdStateOpt     = if(useResp) Some(UInt(ChiResp.width.W)) else None
    def resp            = respOpt.get
    def fwdState        = fwdStateOpt.get
    // DBIDResp DBID(Use In Mas)
    def useChiDBID      : Boolean = true
    val chiDBIDOpt      = if(useChiDBID) Some(UInt(djparam.chiDBIDBits.W)) else None
    def chiDBID         = chiDBIDOpt.get
    // CHI Mes(Common)
    val opcode          = UInt(6.W)
    val expCompAck      = Bool()

}

// ---------------------------------------------------------------- RNSLV CHI Mes Bundle ----------------------------------------------------------------------------- //
class RNSLVCHIMesBundle(implicit p: Parameters) extends DJBundle with HasBaseCHIMesBundle {
    override def useChiDBID: Boolean = false
}

// ---------------------------------------------------------------- MSHR CHI Mes Bundle ----------------------------------------------------------------------------- //
class SNMASCHIMesBundle(implicit p: Parameters) extends DJBundle with HasBaseCHIMesBundle {
    override def useSrc : Boolean = false
    override def useTxn : Boolean = false
    override def useSnp : Boolean = false
}

// ---------------------------------------------------------------- MSHR CHI Mes Bundle ----------------------------------------------------------------------------- //
class MSHRCHIMesBundle(implicit p: Parameters) extends DJBundle with HasBaseCHIMesBundle with HasIncoID {
    override def useTgt : Boolean = false
    override def useResp: Boolean = false
    override def useChiDBID: Boolean = false
}

// ---------------------------------------------------------------- Req To Slice Bundle ----------------------------------------------------------------------------- //
class Req2SliceBundle(implicit p: Parameters) extends DJBundle with HasBaseCHIMesBundle with HasPCUID with HasIncoID with HasAddr with HasDBID {
    override def useTgt : Boolean = false
    override def useResp: Boolean = false
    override def useChiDBID: Boolean = false
}

// -------------------------------------------------------------- Req Ack To Node Bundle ---------------------------------------------------------------------------- //
class ReqAck2NodeBundle(implicit p: Parameters) extends DJBundle with HasToIncoID with HasPCUID { val retry = Bool(); def receive = !retry }

// ---------------------------------------------------------------- Resp To Node Bundle ----------------------------------------------------------------------------- //
class Resp2NodeBundle(implicit p: Parameters) extends DJBundle with HasBaseCHIMesBundle with HasAddr with HasIncoID with HasDBID {
    override def useSnp : Boolean = false
    override def useChiDBID: Boolean = false
    val needReadDB      = Bool()
}


// ---------------------------------------------------------------- Req To Node Bundle ----------------------------------------------------------------------------- //
class Req2NodeBundle(implicit p: Parameters) extends DJBundle with HasBaseCHIMesBundle with HasIncoID with HasAddr with HasMSHRWay with HasDBID {
    override def useChiDBID: Boolean = false
    val selfWay         = UInt(sWayBits.W)
}


// ---------------------------------------------------------------- Resp To Slice Bundle ----------------------------------------------------------------------------- //
class Resp2SliceBundle(implicit p: Parameters) extends DJBundle with HasIncoID with HasDBID with HasMHSRIndex {
    val isSnpResp       = Bool()
    val isReqResp       = Bool()
    val isWriResp       = Bool()
    val hasData         = Bool()
    // Indicate Snoopee final state
    val resp            = UInt(ChiResp.width.W)
    // Indicate Requster final state in DCT
    val fwdState        = Valid(UInt(ChiResp.width.W))

    def isResp          = isSnpResp | isReqResp | isWriResp
    def isUpdate        = !isResp
}


// ---------------------------------------------------------------- DataBuffer Base Bundle ----------------------------------------------------------------------------- //
trait HasDBRCOp extends DJBundle { this: Bundle =>
    val isRead = Bool()
    val isClean = Bool()
}
// Base Data Bundle
trait HasDBData extends DJBundle { this: Bundle =>
    val data = UInt(beatBits.W)
    val dataID = UInt(2.W)
    def beatNum: UInt = {
        if (nrBeat == 1) { 0.U }
        else if (nrBeat == 2) { Mux(dataID === 0.U, 0.U, 1.U) }
        else { dataID }
    }
    def isLast: Bool = beatNum === (nrBeat - 1).U
}

// DataBuffer Read/Clean Req
class DBRCReq    (implicit p: Parameters)   extends DJBundle with HasDBRCOp with HasDBID with HasToIncoID
class DBWReq     (implicit p: Parameters)   extends DJBundle                             with HasFromIncoID with HasPCUID
class DBWResp    (implicit p: Parameters)   extends DJBundle                with HasDBID with HasToIncoID   with HasPCUID
class NodeFDBData(implicit p: Parameters)   extends DJBundle with HasDBData with HasDBID with HasToIncoID
class NodeTDBData(implicit p: Parameters)   extends DJBundle with HasDBData with HasDBID

class DBBundle(hasDBRCReq: Boolean = false)(implicit p: Parameters) extends DJBundle {
    val dbRCReqOpt  = if(hasDBRCReq) Some(Decoupled(new DBRCReq)) else None
    val wReq        = Decoupled(new DBWReq)
    val wResp       = Flipped(Decoupled(new DBWResp))
    val dataFDB     = Flipped(Decoupled(new NodeFDBData))
    val dataTDB     = Decoupled(new NodeTDBData)

    def dbRCReq     = dbRCReqOpt.get
}




