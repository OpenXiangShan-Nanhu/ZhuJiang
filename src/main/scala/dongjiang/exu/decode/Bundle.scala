package dongjiang.pcu.exu.decode

import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.SnpOpcode._
import zhujiang.chi._
import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import dongjiang.chi.CHIChannel._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xs.utils.ParallelLookUp
import dongjiang.chi.ChiState._
import math.max

object RespType {
  val width         = 4
  val NotResp       = "b0000".U
  val Snp           = "b0001".U
  val SnpFwd        = "b0010".U
  val Read          = "b0100".U // Read
  val CB            = "b1000".U // Copy Back

  def Snp_RD        = Snp | Read
  def SnpFwd_RD     = SnpFwd | Read

  def isSnpX(x: UInt) = x === Snp | x === SnpFwd
}

class InstBundle extends Bundle {
//  def ChipTypeWidth = 1
  def ChiChnlWidth  = CHIChannel.width
  def ChiStateWidth = ChiState.width
  def RespTypeWidth = RespType.width
  def ChiRespWidth  = ChiResp.width

//  val chipType    = UInt(ChipTypeWidth.W)
  val channel     = UInt(ChiChnlWidth.W)
  val opcode      = UInt(ReqOpcode.width.W)
  val srcState    = UInt(ChiStateWidth.W)
  val othState    = UInt(ChiStateWidth.W)
  val hnState     = UInt(ChiStateWidth.W)
  val respType    = UInt(RespTypeWidth.W)
  val respHasData = Bool()
  val slvResp     = UInt(ChiRespWidth.W)
  val fwdState    = UInt(ChiRespWidth.W)
  val mstResp     = UInt(ChiRespWidth.W) // Read Down
}


trait HasOperationsBundle extends Bundle {
  // Commit(Resp to Rn Node)
  val commit      = Bool()

  // Send Snoop to Rn Node
  val snoop       = Bool()

  // Send Read to Sn Node
  val readDown    = Bool()
  val writeDown   = Bool()

  // Send Flush to DCU
  val flush       = Bool()

  // Read DataBuffer to Send Data to Resp Node
  val rDB2Src     = Bool() // Read DataBuffer to Req Src
  val cleanDB     = Bool() // Clean DataBuffer

  // Read(Send Data to Resp Node) or Write DataStorage
  val readDCU     = Bool()
  val writeDCU    = Bool()

  // Write New State to Directory
  val wSDir       = Bool()
  val wSFDir      = Bool()
}

class OperationsBundle extends Bundle with HasOperationsBundle

object SnpTgt {
  val width         = 2
  val NONE          = "b00".U
  val ALL           = "b01".U
  val ONE           = "b10".U
  val OTH           = "b11".U
}

class DecodeBundle extends Bundle with HasOperationsBundle {
  def CHIChnlWidth  = CHIChannel.width
  def ChiRespWidth  = ChiResp.width
  def ChiStateWidth = ChiState.width

  // Commit(Resp to Rn Node)
  val respChnl    = UInt(CHIChnlWidth.W)
  val respOp      = UInt(max(DatOpcode.width, RspOpcode.width).W)
  val resp        = UInt(ChiRespWidth.W)
  val fwdState    = UInt(ChiRespWidth.W)

  // Send Snoop to Slave Node
  val snpOp       = UInt(SnpOpcode.width.W)
  val retToSrc    = Bool() // only one snp will be set reqToSec when it need to snp more than 1 node
//  val doNotGoToSD = Bool() // The default is true
  val snpTgt      = UInt(SnpTgt.width.W)

  // Send Read or Write to Master Node
  val rdOp        = UInt(ReqOpcode.width.W)
  val wdOp        = UInt(ReqOpcode.width.W)

  // Write New State to Directory
  val hnState     = UInt(ChiStateWidth.W)
  val srcState    = UInt(ChiStateWidth.W)
  val othState    = UInt(ChiStateWidth.W)

  // No need to do anything
  val nothingTODO = Bool()

  def needWaitSlv = snoop | (commit & respOp === CompDBIDResp)
  def needWaitMst = readDown | writeDown | readDCU | writeDCU

  def decode(inst: InstBundle, table: Seq[(UInt, UInt)]): DecodeBundle = {
    this := ParallelLookUp(
      inst.asUInt,
      table
    ).asTypeOf(new DecodeBundle)
//    this := Mux1H(table.map(_._1 === inst.asUInt), table.map(_._2)).asTypeOf(new DecodeBundle(chiRespWidth, chiStateWidth))
    this
  }
}


object Inst {
  val HasData = true.B

//  def FromLocal           : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.chipType := ChipType.Local;  temp.asUInt }
//  def FromCSN             : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.chipType := ChipType.CSN;    temp.asUInt }
  def Chnl      (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.channel := x;                temp.asUInt }
  def Op        (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.opcode := x;                 temp.asUInt }
  def SrcIs     (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.srcState := x;               temp.asUInt }
  def OthIs     (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.othState := x;               temp.asUInt }
  def HnIs      (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.hnState := x;                temp.asUInt }
  def RespIs    (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.respType := x;               temp.asUInt }
  def RespData  (x: Bool) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.respHasData := x;            temp.asUInt }
  def RnRespIs  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.slvResp := x;                temp.asUInt }
  def FwdStateIs(x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.fwdState := x;               temp.asUInt }
  def SnRespIs  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.mstResp := x;                 temp.asUInt }

  def LocalReqInst (op: UInt, src: UInt, oth: UInt, hn: UInt, data: Bool = false.B):                          UInt = Chnl(CHIChannel.REQ) | Op(op) | SrcIs(src) | OthIs(oth) | HnIs(hn) | RespData(data)
  def LocalSnpInst (op: UInt, src: UInt, oth: UInt, hn: UInt):                                                UInt = Chnl(CHIChannel.SNP) | Op(op) | SrcIs(src) | OthIs(oth) | HnIs(hn)
  def LocalRespInst(chnl: UInt, op: UInt, src: UInt, oth: UInt, hn: UInt, respType: UInt,
                    data: Bool = false.B, rn: UInt = ChiResp.I, fwd: UInt = ChiResp.I, sn: UInt = ChiResp.I): UInt = Chnl(chnl) | Op(op) | SrcIs(src) | OthIs(oth) | HnIs(hn) | RespIs(respType) | RespData(data) |
                                                                                                                     RnRespIs(rn) | FwdStateIs(fwd) | SnRespIs(sn)
}



object Code {
  // Operations
  def Commit           : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.commit := true.B;       temp.asUInt }
  def SnpAll           : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.snoop := true.B; temp.snpTgt := SnpTgt.ALL; temp.asUInt }
  def SnpOne           : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.snoop := true.B; temp.snpTgt := SnpTgt.ONE; temp.asUInt }
  def SnpOth           : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.snoop := true.B; temp.snpTgt := SnpTgt.OTH; temp.asUInt }
  def ReadDown         : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.readDown := true.B;     temp.asUInt }
  def WriteDown        : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.writeDown := true.B;    temp.asUInt }
  def Flush            : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.flush := true.B;        temp.asUInt }
  def RDB2Src          : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.rDB2Src := true.B;      temp.asUInt }
  def CleanDB          : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.cleanDB := true.B;      temp.asUInt }
  def ReadDCU          : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.readDCU := true.B;      temp.asUInt }
  def WriteDCU         : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.writeDCU := true.B;     temp.asUInt }
  def WSDir            : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.wSDir := true.B;        temp.asUInt }
  def WSFDir           : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.wSFDir := true.B;       temp.asUInt }

  // other
  def RespChnl(x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.respChnl := x;          temp.asUInt }
  def RespOp  (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.respOp := x;            temp.asUInt }
  def Resp    (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.resp := x;              temp.asUInt }
  def FwdState(x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.fwdState := x;          temp.asUInt }
  def SnpOp   (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.snpOp := x;             temp.asUInt }
  def RetToSrc         : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.retToSrc := true.B;     temp.asUInt }
  def ReadOp  (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.rdOp := x;              temp.asUInt }
  def WriOp   (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.wdOp := x;              temp.asUInt }
  def HnState (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.hnState := x;           temp.asUInt }
  def SrcState(x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.srcState := x;          temp.asUInt }
  def OthState(x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.othState := x;          temp.asUInt }
  def NothingTODO      : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.nothingTODO := true.B;  temp.asUInt }
  def ERROE            : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle()));                              temp.asUInt }
}

