package zhujiang.chi

import chisel3._
import chisel3.util.Cat
import org.chipsalliance.cde.config.Parameters
import zhujiang.{ZJBundle, ZJParametersKey}

class Flit(implicit p: Parameters) extends Bundle {
  def src = elements("SrcID").asInstanceOf[UInt]
  def tgt = elements("TgtID").asInstanceOf[UInt]
  def txn = elements("TxnID").asInstanceOf[UInt]
  def did = elements("DataID").asInstanceOf[UInt]
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
//  val QoS = UInt(4.W)

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
//  val QoS = UInt(4.W)

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
//  val QoS = UInt(4.W)

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
//  val QoS = UInt(4.W)

  def MECID = DBID
  def FwdState = DataSource
}

object ChannelEncodings {
  def REQ: Int = 0
  def RSP: Int = 1
  def DAT: Int = 2
  def SNP: Int = 3
  def ERQ: Int = 4

  val encodingsMap = Map[String, Int](
    "REQ" -> REQ,
    "RSP" -> RSP,
    "DAT" -> DAT,
    "SNP" -> SNP,
    "ERQ" -> ERQ
  )
}

class Credit[T <: Flit](gen: T) extends Bundle {
  val flitpend = Output(Bool())
  val flit = Output(gen)
  val flitv = Output(Bool())
  val lcrdv = Input(Bool())
}

object Credit {
  def apply[T <: Flit](gen: T): Credit[T] = {
    new Credit(gen)
  }
}

class ReqAddrBundle(implicit p: Parameters) extends ZJBundle {
  val mmio = Bool()
  val chip = UInt(nodeAidBits.W)
  val tag = UInt((raw - 1 - nodeAidBits - 6).W)
  val offset = UInt(6.W)
  def checkBank(width: Int, bankId: UInt): Bool = {
    if(width == 0) true.B
    else tag(bankOff + width - 7, bankOff - 6) === bankId
  }
  def devAddr: UInt = Cat(tag, offset)
}

class SnpAddrBundle(implicit p: Parameters) extends ZJBundle {
  val mmio = Bool()
  val chip = UInt(nodeAidBits.W)
  val tag = UInt((raw - 1 - nodeAidBits - 3).W)
  val offset = UInt(3.W)
  def checkBank(width: Int, bankId: UInt): Bool = {
    if(width == 0) true.B
    else tag(bankOff + width - 7, bankOff - 6) === bankId
  }
}

class NodeIdBundle(implicit p: Parameters) extends ZJBundle {
  val net = Bool()
  val nid = UInt(nodeNidBits.W)
  val aid = UInt(nodeAidBits.W)
  def chip: UInt = aid
  def router: UInt = Cat(net, nid, 0.U(aid.getWidth.W))
}