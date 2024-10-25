package dongjiang.pcu.exu.decode

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import dongjiang.chi.CHIChannel._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xs.utils.ParallelLookUp
import dongjiang.chi.CHIOp.REQ._
import dongjiang.chi.CHIOp.RSP._
import dongjiang.chi.CHIOp.DAT._
import dongjiang.chi.CHIOp.SNP._
import dongjiang.chi.ChiState._


object RespType {
  val width         = 3
  val NotResp       = "b000".U
  val Snp           = "b001".U
  val SnpFwd        = "b010".U
  val RD            = "b100".U // Read Down

  def Snp_RD        = Snp | RD
  def SnpFwd_RD     = SnpFwd | RD
}

class InstBundle extends Bundle {
  def ChipTypeWidth = 1
  def ChiChnlWidth  = CHIChannel.width
  def ChiStateWidth = 3
  def RespTypeWidth = RespType.width
  def ChiRespWidth  = 3

  val chipType    = UInt(ChipTypeWidth.W)
  val channel     = UInt(ChiChnlWidth.W)
  val opcode      = UInt(6.W)
  val srcState    = UInt(ChiStateWidth.W)
  val othState    = UInt(ChiStateWidth.W)
  val hnState     = UInt(ChiStateWidth.W)
  val respType    = UInt(RespTypeWidth.W)
  val respHasData = Bool()
  val snpResp     = UInt(ChiRespWidth.W)
  val fwdState    = UInt(ChiRespWidth.W)
  val rdResp      = UInt(ChiRespWidth.W) // Read Down
}


trait HasOperationsBundle extends Bundle {
  // Commit(Resp to Rn Node)
  val commit      = Bool()

  // Send Snoop to Rn Node
  val snoop       = Bool()

  // Send Read to Sn Node
  val readDown    = Bool()
  val writeDown   = Bool()

  // Read DataBuffer to Send Data to Resp Node
  val rDB2Src     = Bool() // Read DataBuffer to Req Src
  val cleanDB     = Bool() // Clean DataBuffer

  // Read(Send Data to Resp Node) or Write DataStorage
  val readDCU     = Bool()
  val writeDCU    = Bool()

  // Write New State to Directory
  val wSDir       = Bool()
  val wSFDir      = Bool()

  def reqToSlv    = snoop
  def reqToMas    = readDown | writeDown | readDCU | writeDCU
}

class OperationsBundle extends Bundle with HasOperationsBundle

class DecodeBundle extends Bundle with HasOperationsBundle {
  def CHIChnlWidth  = CHIChannel.width
  def ChiRespWidth  = 3
  def ChiStateWidth = 3

  // Commit(Resp to Rn Node)
  val respChnl    = UInt(CHIChnlWidth.W)
  val respOp      = UInt(5.W)
  val resp        = UInt(ChiRespWidth.W)
  val fwdState    = UInt(ChiRespWidth.W)

  // Send Snoop to Slave Node
  val snpOp       = UInt(5.W)
  val retToSrc    = Bool()
//  val doNotGoToSD = Bool() // The default is true

  // Send Read or Write to Master Node
  val rdOp        = UInt(6.W)
  val wdOp        = UInt(6.W)

  // Write New State to Directory
  val hnState     = UInt(ChiStateWidth.W)
  val srcState    = UInt(ChiStateWidth.W)
  val othState    = UInt(ChiStateWidth.W)

  // No need to do anything
  val nothingTODO = Bool()

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

  def FromLocal           : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.chipType := ChipType.Local;  temp.asUInt }
  def FromCSN             : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.chipType := ChipType.CSN;    temp.asUInt }
  def Chnl      (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.channel := x;                temp.asUInt }
  def Op        (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.opcode := x;                 temp.asUInt }
  def SrcIs     (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.srcState := x;               temp.asUInt }
  def OthIs     (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.othState := x;               temp.asUInt }
  def HnIs      (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.hnState := x;                temp.asUInt }
  def RespIs    (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.respType := x;               temp.asUInt }
  def RespData  (x: Bool) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.respHasData := x;            temp.asUInt }
  def SnpRespIs (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.snpResp := x;                temp.asUInt }
  def FwdStateIs(x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.fwdState := x;               temp.asUInt }
  def RDRespIs  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new InstBundle())); temp.rdResp := x;                 temp.asUInt }

  def LocalReqInst (op: UInt, src: UInt, oth: UInt, hn: UInt, data: Bool = false.B): UInt = FromLocal | Chnl(CHIChannel.REQ) | Op(op) | SrcIs(src) | OthIs(oth) | HnIs(hn) | RespData(data)
  def LocalRespInst(chnl: UInt, op: UInt, src: UInt, oth: UInt, hn: UInt, respType: UInt, data: Bool = false.B, snp: UInt = ChiResp.I, fwd: UInt = ChiResp.I, rd: UInt = ChiResp.I): UInt = FromLocal | Chnl(chnl) | Op(op) | SrcIs(src) | OthIs(oth) | HnIs(hn) | RespIs(respType) | RespData(data) | SnpRespIs(snp) | FwdStateIs(fwd) | RDRespIs(rd)
}



object Code {
  // Operations
  def Commit           : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.commit := true.B;       temp.asUInt }
  def Snoop            : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.snoop := true.B;        temp.asUInt }
  def ReadDown         : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.readDown := true.B;     temp.asUInt }
  def WriteDown        : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.writeDown := true.B;    temp.asUInt }
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
  def retToSrc         : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.retToSrc := true.B;     temp.asUInt }
  def ReadOp  (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.rdOp := x;              temp.asUInt }
  def WriOp   (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.wdOp := x;              temp.asUInt }
  def HnState (x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.hnState := x;           temp.asUInt }
  def SrcState(x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.srcState := x;          temp.asUInt }
  def OthState(x: UInt): UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.othState := x;          temp.asUInt }
  def NothingTODO      : UInt = { val temp = WireInit(0.U.asTypeOf(new DecodeBundle())); temp.nothingTODO := true.B;  temp.asUInt }
}

