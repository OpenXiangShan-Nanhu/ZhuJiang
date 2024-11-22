package dongjiang.pcu

import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.SnpOpcode._
import dongjiang._
import dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import scala.collection.immutable.ListMap
import scala.math.{max, min}

// ---------------------------------------------------------------- Xbar Id Bundle ----------------------------------------------------------------------------- //
// Interconnect ID
object IncoID {
  val width       = 1
  val LOCALSLV    = 0
  val LOCALMST    = 1
  def max = LOCALMST
}

trait HasFromIncoID extends DJBundle { this: Bundle => val from = UInt(max(IncoID.width, dcuBankBits).W) }

trait HasToIncoID extends DJBundle { this: Bundle => val to = UInt(max(IncoID.width, dcuBankBits).W) }

trait HasIncoID extends DJBundle with HasFromIncoID with HasToIncoID

trait HasDBID extends DJBundle { this: Bundle => val dbID = UInt(dbIdBits.W); def apuEID = ((djparam.nrDatBuf - 1).U - dbID)(apuIdBits - 1, 0) }

trait HasUseAddr extends DJBundle {this: Bundle =>
  val useAddr     = UInt(useAddrBits.W)
  def mTag        = parseMSHRAddr(useAddr)._1
  def mSet        = parseMSHRAddr(useAddr)._2
  def sTag        = parseSelfAddr(useAddr)._1
  def sSet        = parseSelfAddr(useAddr)._2
  def sfTag       = parseSFAddr(useAddr)._1
  def sfSet       = parseSFAddr(useAddr)._2
  def dirBank     = parseSFAddr(useAddr)._3
  def minDirSet   = useAddr(minDirSetBits - 1, 0)
  def fullAddr(d: UInt, p:UInt, sec: Bool = false.B) = getFullAddr(useAddr, d, p, sec)
  def snpAddr (d: UInt, p:UInt) = fullAddr(d, p)(fullAddrBits - 1, 3)
}

trait HasMSHRSet extends DJBundle { this: Bundle => val mshrSet = UInt(mshrSetBits.W) }

class MSHRSetBundle(implicit p: Parameters) extends DJBundle with HasMSHRSet

trait HasMSHRWay extends DJBundle { this: Bundle => val mshrWay = UInt(mshrWayBits.W) }

trait HasMHSRIndex extends DJBundle with HasMSHRSet with HasMSHRWay { def mshrMatch(set: Int, way: Int): Bool = mshrSet === set.U & mshrWay === way.U }

object PipeID { val width = 1; val RESP = "b0".U; val REQ = "b1".U }

trait HasPipeID extends Bundle { this: Bundle => val pipeID = UInt(PipeID.width.W); def toReqPipe = pipeID === PipeID.REQ; def toRespPipe = pipeID === PipeID.RESP }

trait HasIntfEntryID extends DJBundle { this: Bundle => val entryID = UInt(intfEntryIdBits.W) }

trait HasDcuID extends DJBundle { this: Bundle => val dcuID = UInt(dcuBankBits.W) } // DCU Bank ID

// ---------------------------------------------------------------- CHI Base Bundle ----------------------------------------------------------------------------- //
class ChiIndexBundle(implicit p: Parameters) extends DJBundle {
  val nodeID          = UInt(useNodeIdBits.W)
  val txnID           = UInt(chiTxnIdBits.W)
  val beatOH          = UInt(2.W)
  def snpCcMetaVec    = nodeID(nrCcNode - 1 ,0)
  def fullSize        = beatOH === "b11".U
  def fstBeat         = beatOH === "b01".U
  def secBeat         = beatOH === "b10".U
}

class ChiMesBundle(implicit p: Parameters) extends DJBundle with HasCHIChannel {
  // Snp Mes(Use In Snp)
  val doNotGoToSD     = Bool()
  val retToSrc        = Bool()
  val fwdState        = UInt(ChiResp.width.W)
  // REQ Mes(Use In Req)
  val expCompAck      = Bool()
  // Common
  val opcode          = UInt(7.W)
  val resp            = UInt(ChiResp.width.W)
}

// ---------------------------------------------------------------- PCU Base Bundle ----------------------------------------------------------------------------- //
// Dont use mshrSet when Bundle HasUseAddr
class PcuIndexBundle(implicit p: Parameters) extends DJBundle with HasMHSRIndex with HasDBID with HasIntfEntryID

// ---------------------------------------------------------------- Req To EXU Bundle ----------------------------------------------------------------------------- //
class Req2ExuBundle(implicit p: Parameters) extends DJBundle with HasIncoID {
  val chiIndex      = new ChiIndexBundle()
  val chiMes        = new ChiMesBundle()
  val pcuIndex      = new PcuIndexBundle()
  val pcuMes        = new DJBundle with HasUseAddr
}

// -------------------------------------------------------------- Req Ack To Intf Bundle ---------------------------------------------------------------------------- //
class ReqAck2IntfBundle(implicit p: Parameters) extends DJBundle with HasToIncoID with HasIntfEntryID {
  val retry         = Bool()
  def receive       = !retry
}

// ---------------------------------------------------------------- Resp To Intf Bundle ----------------------------------------------------------------------------- //
class Resp2IntfBundle(implicit p: Parameters) extends DJBundle with HasIncoID {
  val chiIndex      = new ChiIndexBundle()
  val chiMes        = new ChiMesBundle()
  val pcuIndex      = new PcuIndexBundle()
  val pcuMes        = new DJBundle with HasUseAddr
}


// ---------------------------------------------------------------- Req To Intf Bundle ----------------------------------------------------------------------------- //
class Req2IntfBundle(implicit p: Parameters) extends DJBundle with HasIncoID {
  val chiIndex      = new ChiIndexBundle()
  val chiMes        = new ChiMesBundle()
  val pcuIndex      = new PcuIndexBundle()
  val pcuMes        = new DJBundle with HasUseAddr {
    // only use in local sn master interface
    val doDMT       = Bool()
    val selfWay     = UInt(sWayBits.W)
    val toDCU       = Bool()
    // only use in local rn slave interface
    val hasPcuDBID  = Bool() // already get DBID in Write
  }

  def addrWithDcuID = Cat(pcuMes.useAddr, from)
}


// ---------------------------------------------------------------- Resp To Exu Bundle ----------------------------------------------------------------------------- //
class Resp2ExuBundle(implicit p: Parameters) extends DJBundle with HasIncoID {
  val chiIndex      = new ChiIndexBundle()
  val chiMes        = new ChiMesBundle()
  val pcuIndex      = new PcuIndexBundle()
  val pcuMes        = new DJBundle with HasUseAddr {
    val isSnpResp   = Bool()
    val isReqResp   = Bool()
    val isWriResp   = Bool()
    val isCompAck   = Bool()
    val hasData     = Bool()
    val fwdSVald    = Bool()
    def isResp      = isSnpResp | isReqResp | isWriResp | isCompAck
    def isUpdate    = !isResp
  }
}


// ---------------------------------------------------------------- DataBuffer Base Bundle ----------------------------------------------------------------------------- //
trait HasDBRCOp extends DJBundle { this: Bundle =>
  val isRead        = Bool()
  val isClean       = Bool()
}
// Base Data Bundle
trait HasDBData extends DJBundle { this: Bundle =>
  val data          = UInt(beatBits.W)
  val dataID        = UInt(2.W)
  def beatNum: UInt = toBeatNum(dataID)
  def isLast:  Bool = beatNum === (nrBeat - 1).U
}
trait HasMask extends DJBundle { this: Bundle =>
  val mask          = UInt(maskBits.W)
}
// DataBuffer Read/Clean Req
class DBRCReq     (implicit p: Parameters)   extends DJBundle with HasDBRCOp with HasDBID with HasToIncoID                       { val rBeatOH = UInt(2.W); val exuAtomic = Bool() }
class GetDBID     (implicit p: Parameters)   extends DJBundle                             with HasFromIncoID with HasIntfEntryID { val atomicVal = Bool();  val atomicOp = UInt(AtomicOp.width.W); val swapFst = Bool(); }
class DBIDResp    (implicit p: Parameters)   extends DJBundle                with HasDBID with HasToIncoID   with HasIntfEntryID { val retry = Bool();      def receive = !retry }
class NodeFDBData (implicit p: Parameters)   extends DJBundle with HasDBData with HasDBID with HasToIncoID   with HasMask
class NodeTDBData (implicit p: Parameters)   extends DJBundle with HasDBData with HasDBID                    with HasMask        { val atomicVal = Bool() }

class DBBundle(hasDBRCReq: Boolean = false)(implicit p: Parameters) extends DJBundle {
  val dbRCReqOpt  = if(hasDBRCReq) Some(Decoupled(new DBRCReq)) else None
  val getDBID     = Decoupled(new GetDBID)
  val dbidResp    = Flipped(Decoupled(new DBIDResp))
  val dataFDB     = Flipped(Decoupled(new NodeFDBData))
  val dataTDB     = Decoupled(new NodeTDBData)

  def dbRCReq     = dbRCReqOpt.get
}




