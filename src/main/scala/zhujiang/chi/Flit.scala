package zhujiang.chi

import chisel3._
import chisel3.util.{Cat, DecoupledIO}
import org.chipsalliance.cde.config.Parameters
import zhujiang.{ZJBundle, ZJParametersKey}

class Flit(implicit p: Parameters) extends Bundle {
  lazy val zjParams = p(ZJParametersKey)
  lazy val M = zjParams.M
  lazy val PB = zjParams.PB
  lazy val E = zjParams.E
  lazy val R = zjParams.R
  lazy val S = zjParams.S
  lazy val Y = zjParams.Y
  lazy val raw = zjParams.requestAddrBits
  lazy val saw = zjParams.snoopAddrBits
  lazy val niw = zjParams.nodeIdBits
  lazy val dw = zjParams.dataBits
  lazy val bew = zjParams.beBits
  def src:UInt = elements("SrcID").asUInt
  def tgt:UInt = elements("TgtID").asUInt
  def qos:UInt = elements("QoS").asUInt
}

class ReqFlit(dmt:Boolean = false)(implicit p: Parameters) extends Flit {
  val RSVDC = UInt(Y.W)
  val SecID1 = UInt(S.W)
  val MECID = UInt(E.max(R).W)
  val PBHA = UInt(PB.W)
  val MPAM = UInt(M.W)
  //  val TraceTag = UInt(1.W)
  //  val TagOp = UInt(2.W)
  val ExpCompAck = Bool()
  val Excl = Bool()
  //  val PGroupID = UInt(8.W)
  val SnpAttr = Bool()
  val MemAttr = UInt(4.W)
  //  val PCrdType = UInt(4.W)
  val Order = UInt(2.W)
  //  val AllowRetry = Bool()
  //  val LikelyShared = Bool()
  //  val NSE = Bool()
  //  val NS = Bool()
  val Addr = UInt(raw.W)
  val Size = UInt(3.W)
  val Opcode = UInt(7.W)
  val ReturnTxnID = if(dmt) Some(UInt(12.W)) else None
  //  val StashNIDValid = Bool()
  val ReturnNID = if(dmt) Some(UInt(niw.W)) else None
  val TxnID = UInt(12.W)
  val SrcID = UInt(niw.W)
  val TgtID = UInt(niw.W)
  val QoS = UInt(4.W)

  def StreamID = MECID
  def SnoopMe = Excl
  //  def CAH = Excl
  //  def StashGroupID = PGroupID
  //  def TagGroupID = PGroupID
  //  def LPID = PGroupID
  def DoDWT = SnpAttr
  //  def Endian = StashNIDValid
  //  def Deep = StashNIDValid
  //  def PrefetchTgtHint = StashNIDValid
  def StashNID = ReturnNID.get
  def DataTarget = ReturnNID.get
  def fullSize = Size === 6.U
}
class RReqFlit(implicit p: Parameters) extends ReqFlit(false)
class HReqFlit(implicit p: Parameters) extends ReqFlit(true)

class RespFlit(implicit p: Parameters) extends Flit {
//  val TraceTag = UInt(1.W)
//  val TagOp = UInt(2.W)
//  val PCrdType = UInt(4.W)
  val DBID = UInt(12.W)
  val CBusy = UInt(3.W)
  val FwdState = UInt(3.W)
  val Resp = UInt(3.W)
  val RespErr = UInt(2.W)
  val Opcode = UInt(5.W)
  val TxnID = UInt(12.W)
  val SrcID = UInt(niw.W)
  val TgtID = UInt(niw.W)
  val QoS = UInt(4.W)

  def PGroupID = DBID
  def StashGroupID = DBID
  def TagGroupID = DBID
  def DataPull = FwdState
}

class SnoopFlit(implicit p: Parameters) extends Flit {
  val MECID = UInt(E.W)
  val MPAM = UInt(M.W)
//  val TraceTag = UInt(1.W)
  val RetToSrc = Bool()
  val DoNotGoToSD = Bool()
//  val NSE = Bool()
//  val NS = Bool()
  val Addr = UInt(saw.W)
  val Opcode = UInt(5.W)
  val FwdTxnID = UInt(12.W)
  val FwdNID = UInt(niw.W)
  val TxnID = UInt(12.W)
  val SrcID = UInt(niw.W)
  val TgtID = UInt(niw.W)
  val QoS = UInt(4.W)

  def VMIDExt = FwdTxnID
  def PBHA = FwdNID
}

class DataFlit(implicit p: Parameters) extends Flit {
//  val Poison = UInt(pw.W)
//  val DataCheck = UInt(dcw.W)
  val Data = UInt(dw.W)
  val BE = UInt(bew.W)
  val RSVDC = UInt(Y.W)
//  val Replicate = Bool()
//  val NumDat = UInt(2.W)
//  val CAH = Bool()
//  val TraceTag = UInt(1.W)
//  val TU = UInt((dw / 128).W)
//  val Tag = UInt((dw / 32).W)
//  val TagOp = UInt(2.W)
  val DataID = UInt(2.W)
//  val CCID = UInt(2.W)
  val DBID = UInt(16.W)
  val CBusy = UInt(3.W)
//  val DataPull = Bool()
  val DataSource = UInt(8.W)
  val Resp = UInt(3.W)
  val RespErr = UInt(2.W)
  val Opcode = UInt(4.W)
  val HomeNID = UInt(niw.W)
  val TxnID = UInt(12.W)
  val SrcID = UInt(niw.W)
  val TgtID = UInt(niw.W)
  val QoS = UInt(4.W)

  def MECID = DBID
  def FwdState = DataSource
}

object FlitType {
  val encodings = Seq(
    "REQ" -> 0,
    "RSP" -> 1,
    "DAT" -> 2,
    "SNP" -> 3,
    "ERQ" -> 4,
    "DBG" -> 6,
    "HPR" -> 7
  ).toMap
  val req = encodings("REQ")
  val rsp = encodings("RSP")
  val dat = encodings("DAT")
  val snp = encodings("SNP")
  val erq = encodings("ERQ")
  val dbg = encodings("DBG")
  val hpr = encodings("HPR")
  val cppDefines = encodings.map(e => s"#define ${e._1} ${e._2}\n").reduce(_ + _)
}

class RingFlit(width:Int)(implicit p: Parameters) extends ZJBundle {
  val Payload = UInt((width - niw - niw - 16).W)
  val TxnID = UInt(12.W)
  val SrcID = UInt(niw.W)
  val TgtID = UInt(niw.W)
  val QoS = UInt(4.W)

  def qos:UInt = QoS
  def tgt:UInt = TgtID
  def src:UInt = SrcID
  def txn:UInt = TxnID
  def did:UInt = this.asTypeOf(new DataFlit).DataID
}

class ReqAddrBundle(implicit p: Parameters) extends ZJBundle {
  val ci = UInt(ciIdBits.W)
  val tag = UInt((raw - ciIdBits - 6).W)
  val offset = UInt(6.W)
  def checkBank(width: Int, bankId: UInt, bankOffset:Int): Bool = {
    if(width == 0) true.B
    else tag(bankOffset + width - 7, bankOffset - 6) === bankId
  }
  def devAddr: UInt = Cat(tag, offset)
  require(this.getWidth == raw)
}

class DeviceReqAddrBundle(implicit p: Parameters) extends ZJBundle {
  private val cpuSpaceBits = zjParams.cpuSpaceBits
  val ci = UInt(ciIdBits.W)
  val tag = UInt((raw - ciIdBits - cpuIdBits - cpuSpaceBits).W)
  val core = UInt(cpuIdBits.W)
  val dev = UInt(cpuSpaceBits.W)
  require(this.getWidth == raw)
}

class SnpAddrBundle(implicit p: Parameters) extends ZJBundle {
  val ci = UInt(ciIdBits.W)
  val tag = UInt((raw - ciIdBits - 3).W)
  val offset = UInt(3.W)
  def checkBank(width: Int, bankId: UInt, bankOffset:Int): Bool = {
    if(width == 0) true.B
    else tag(bankOffset + width - 7, bankOffset - 6) === bankId
  }
  require(this.getWidth == raw)
}

class NodeIdBundle(implicit p: Parameters) extends ZJBundle {
  val nid = UInt(nodeNidBits.W)
  val aid = UInt(nodeAidBits.W)
  def router: UInt = Cat(nid, 0.U(aid.getWidth.W))
}

object FlitHelper {
  def connIcn(sink:DecoupledIO[Data], src:DecoupledIO[Data], checkWidth:Boolean = true):Unit = {
    if(checkWidth) require(sink.getWidth == src.getWidth, s"left flit width: ${sink.getWidth}, right flit width: ${src.getWidth}")
    sink.valid := src.valid
    src.ready := sink.ready
    sink.bits := src.bits.asTypeOf(sink.bits)
  }
}