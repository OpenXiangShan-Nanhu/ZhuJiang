package dongjiang.frontend.decode

import chisel3._
import chisel3.experimental.SourceInfo
import chisel3.util._
import dongjiang.{DJBundle, bundle}
import org.chipsalliance.cde.config.Parameters
import zhujiang.chi._
import dongjiang.bundle._
import xs.utils.ParallelLookUp
import dongjiang.frontend.decode.DecodeCHI._
import math.max
import dongjiang.bundle.ChiChannel._
import xs.utils.debug._
import dongjiang.data._

/*
 * (UInt, Seq[(UInt, )])
 *   ChiInst -> [StateInst -> (TaskCode, [TaskInst -> SecTaskCode, [SecTaskInst -> CommitCode]])]
 *      ^            ^                       ^                           ^
 * RxReq/RxSnp    DirResp             ReqResp/SnpResp              ReqResp/SnpResp
 *
 * Table: Seq[(ChiInst, Seq[(StateInst, Code)])]
 *
 * Code: (TaskCode, Seq[(TaskInst, SecCode)])
 *
 * SecCode: (TaskCode, Seq[(TaskInst, CommitCode)])
 *
 */

/*
 * Task Inst Description:
 * 1. Channel     : 0 -> REQ; 1 -> DAT; 2 -> RSP; 3 -> SNP; Task must be REQ/SNP
 * 2. fromLAN     : true -> Task from LAN(CC/RNI) NoC; false -> Task from BBN(HNX) NoC
 * 3. toLAN       : true -> Task to LAN(SN) NoC; false -> Task to BBN(HNX) NoC
 * 4. opcode      : REQ/SNP Opcode
 * 5. expCompAck  : REQ.ExpCompAck
 * 6. allocate    : REQ.MemAttr.allocate
 * 7. ewa         : REQ.MemAttr.ewa
 * 8. order       : REQ.Order
 *
 * Note0: TaskInst comes from the CHI Bundle, and the CHI Bundle is uniformly assigned in ToChiTask.
 * Note1: Please refer to https://saluyk0ac00.feishu.cn/wiki/OHENwBqbeil9VIkkRQacWTAnnFd?from=from_copylink for the concepts of LAN and BBN.
 */
class ChiInst extends Bundle  {
  // REQ: LAN -> LAN; LAN -> BBN; BBN -> LAN;
  // SNP: BBN -> LAN
  val valid       = Bool()
  val channel     = UInt(ChiChannel.width.W)
  val fromLAN     = Bool() // true -> LAN; false -> BBN
  val toLAN       = Bool() // true -> LAN; false -> BBN
  val opcode      = UInt(ReqOpcode.width.max(SnpOpcode.width).W)
  val expCompAck  = Bool()
  val allocate    = Bool() // Only use in write, ignore it in others
  val ewa         = Bool() // Only use in write, ignore it in others
  val order       = UInt(Order.width.W) // TODO: Looks like there's no need to check the order in ChiInst.

  override def toPrintable: Printable = {
    var pa = cf""
    elements.toSeq.reverse.foreach { case (name, value) =>
      pa = pa + cf"$name -> $value "
    }
    pa
  }
}

class StateInst extends Bundle {
  val valid       = Bool()
  val srcHit      = Bool()
  val othHit      = Bool()
  val llcState    = UInt(ChiState.width.W)

  override def toPrintable: Printable = {
    var pa = cf""
    elements.toSeq.reverse.foreach { case (name, value) =>
      pa = pa + cf"$name -> $value "
    }
    pa
  }
}

class TaskInst extends Bundle {
  // from TaskCM
  val valid       = Bool()
  val fwdValid    = Bool()
  val channel     = UInt(ChiChannel.width.W)
  val opcode      = UInt(RspOpcode.width.max(DatOpcode.width).W)
  val resp        = UInt(ChiResp.width.W)
  val fwdResp     = UInt(ChiResp.width.W)

  // from ReceiveCM
  val wrRespIsDat = Bool() // Has been got write resp(XCBWrData)
  val cbRespIsAck = Bool() // CopyBack Resp is CompAck
  val cbResp      = UInt(ChiResp.width.W)

  override def toPrintable: Printable = {
    var pa = cf""
    elements.toSeq.reverse.foreach { case (name, value) =>
      pa = pa + cf"$name -> $value "
    }
    pa
  }
}

trait HasPackTaskInst { this: Bundle => val taskInst = new TaskInst() }

object CMID {
  lazy val SNP  = 0
  lazy val WRI  = 1
  lazy val READ = 2
  lazy val DL   = 3
}

trait HasOperations { this: Bundle =>
  val snoop       = Bool() // -> Return Valid   TaskInst
  val read        = Bool() // -> Return Valid   TaskInst
  val dataless    = Bool() // -> Return Valid   TaskInst
  val wriOrAtm    = Bool() // -> Return Invalid TaskInst // Write or Atomic // TODO: del Atomic
  def opsValid    = snoop | read | dataless | wriOrAtm
  def opsInvalid  = !opsValid
  def cmid: UInt  = {
    PriorityMux(Seq(
      snoop    -> CMID.SNP.U,
      read     -> CMID.READ.U,
      dataless -> CMID.DL.U,
      wriOrAtm -> CMID.WRI.U,
    ))
  }
}

class Operations extends Bundle with HasOperations

trait HasPackOperations { this: Bundle => val ops = new Operations() }

object SnpTgt {
  val width       = 2
  val NONE        = "b00".U
  val ALL         = "b01".U
  val ONE         = "b10".U // Select first other
  val OTH         = "b11".U
}

trait HasSnpTgt { this: Bundle => val snpTgt = UInt(SnpTgt.width.W) }

trait HasTaskCode { this: Bundle with HasOperations with HasPackDataOp =>
  // Common
  val opcode      = UInt(ReqOpcode.width.max(SnpOpcode.width).W)

  // Wait ReceiveCM Done
  val waitRecDone = Bool() // -> Return Valid   TaskInst

  // Req
  val expCompAck  = Bool()
  val doDMT       = Bool()

  // Snoop
  val retToSrc    = Bool()
  val snpTgt      = UInt(SnpTgt.width.W)
  def snpAll      = snpTgt(0).asBool
  def snpOne      = snpTgt(1).asBool
  def snpOth      = snpTgt(2).asBool

  def valid: Bool = opsValid | waitRecDone
}

class TaskCode extends Bundle with HasOperations with HasPackDataOp with HasTaskCode

trait HasPackTaskCode { this: Bundle => val taskCode = new TaskCode() }

trait HasWriDirCode { this: Bundle =>
  // Write Directory
  val wriSRC      = Bool()
  val wriSNP      = Bool()
  val wriLLC      = Bool()
  val srcValid    = Bool()
  val snpValid    = Bool()
  val llcState    = UInt(ChiState.width.W)

  def wriSF       = wriSRC | wriSNP
  def wriDir      = wriSF  | wriLLC
}

trait HasCommitCode { this: Bundle with HasWriDirCode with HasPackDataOp =>
  val waitSecDone = Bool() // Need wait second task done

  val sendResp    = Bool()
  val sendfwdResp = Bool() // CompData to RN in DCT
  val channel     = UInt(ChiChannel.width.W)
  val commitOp    = UInt(RspOpcode.width.max(DatOpcode.width).W)
  val resp        = UInt(ChiResp.width.W)
  val fwdResp     = UInt(ChiResp.width.W)

  // def
  def valid       = sendResp | wriDir | dataOp.valid
  def invalid     = !valid
}

class CommitCode extends Bundle with HasWriDirCode with HasPackDataOp with HasCommitCode

trait HasPackCmtCode { this: Bundle => val commit = new CommitCode() }

object DecodeCHI {
  val width = ChiResp.width

  val I  = "b000".U(width.W)
  val SC = "b001".U(width.W)
  val UC = "b010".U(width.W)
  val UD = "b011".U(width.W)

  val I_PD  = "b100".U(width.W)
  val SC_PD = "b101".U(width.W)
  val UC_PD = "b110".U(width.W)
  val UD_PD = "b110".U(width.W)
  val SD_PD = "b111".U(width.W)

  def toResp(x: UInt): UInt = {
    val result = WireInit(0.U(ChiResp.width.W))
    when(x === UD) {
      result := ChiResp.SD
    }.otherwise {
      result := x
    }
    result
  }

  def toState(x: UInt): UInt = {
    val result = WireInit(0.U(ChiState.width.W))
    switch(x) {
      is(I)  { result := ChiState.I  }
      is(SC) { result := ChiState.SC }
      is(UC) { result := ChiState.UC }
      is(UD) { result := ChiState.UD }
    }
    assert(!(x & ChiResp.PassDirty).orR)
    result
  }
}

object Inst {
  // Chi Inst
  def chiValid          : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.valid         := true.B;  temp.asUInt }
  def isReq             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.channel       := REQ;     temp.asUInt | chiValid }
  def isSnp             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.channel       := SNP;     temp.asUInt | chiValid }
  def fromLAN           : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.fromLAN       := true.B;  temp.asUInt }
  def fromBBN           : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.fromLAN       := false.B; temp.asUInt }
  def toLAN             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.toLAN         := true.B;  temp.asUInt }
  def toBBN             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.toLAN         := false.B; temp.asUInt }
  def reqIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.opcode        := x;       require(x.getWidth == ReqOpcode.width); temp.asUInt | isReq }
  def snpIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.opcode        := x;       require(x.getWidth == RspOpcode.width); temp.asUInt | isSnp }
  def expCompAck        : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.expCompAck    := true.B;  temp.asUInt }
  def allocate          : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.allocate      := true.B;  temp.asUInt }
  def ewa               : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.ewa           := true.B;  temp.asUInt }
  def noOrder           : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.order         := Order.None;              temp.asUInt }
  def isEO              : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.order         := Order.EndpointOrder;     temp.asUInt }
  def isOWO             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.order         := Order.OWO;               temp.asUInt | expCompAck }

  // State Inst
  def stateValid        : UInt = { val temp = WireInit(0.U.asTypeOf(new StateInst())); temp.valid       := true.B;      temp.asUInt }
  def srcHit            : UInt = { val temp = WireInit(0.U.asTypeOf(new StateInst())); temp.srcHit      := true.B;      temp.asUInt | stateValid }
  def srcMiss           : UInt = { val temp = WireInit(0.U.asTypeOf(new StateInst())); temp.srcHit      := false.B;     temp.asUInt | stateValid }
  def othHit            : UInt = { val temp = WireInit(0.U.asTypeOf(new StateInst())); temp.othHit      := true.B;      temp.asUInt | stateValid }
  def othMiss           : UInt = { val temp = WireInit(0.U.asTypeOf(new StateInst())); temp.othHit      := false.B;     temp.asUInt | stateValid }
  def sfHit             : UInt = srcHit  | othHit
  def sfMiss            : UInt = srcMiss | othMiss
  def llcIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new StateInst())); temp.llcState    := toState(x);  require(x.getWidth == DecodeCHI.width); temp.asUInt | stateValid }

  // Task Inst
  def taskValid         : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.valid        := true.B;    temp.asUInt }
  def fwdValid          : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.fwdValid     := true.B;    temp.asUInt }
  def isRsp             : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.channel      := RSP;       temp.asUInt | taskValid }
  def isDat             : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.channel      := DAT;       temp.asUInt | taskValid }
  def rspIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.opcode       := x;         require(x.getWidth == RspOpcode.width); temp.asUInt | isRsp }
  def datIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.opcode       := x;         require(x.getWidth == DatOpcode.width); temp.asUInt | isDat }
  def respIs  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.resp         := toResp(x); require(x.getWidth == DecodeCHI.width); temp.asUInt }
  def fwdIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.fwdResp      := toResp(x); require(x.getWidth == DecodeCHI.width); temp.asUInt | fwdValid }
  def xCBWrDataValid    : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.wrRespIsDat  := true.B;    temp.asUInt | taskValid }
  def cbRespIsCompAck   : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.cbRespIsAck  := true.B;    temp.asUInt | taskValid }
  def cbRespIs(x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.cbResp       := toResp(x); require(x.getWidth == DecodeCHI.width); temp.asUInt | xCBWrDataValid  }
  def hasGotNCBWrData   : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.cbResp       := toResp(I); temp.asUInt | taskValid | xCBWrDataValid }
  def emptyResp         : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst()));                                 temp.asUInt | taskValid }
  def noResp            : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst()));                                 temp.asUInt }
}



object Code {
  // Task Code Operations
  def snpAll  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.snoop    := true.B; temp.snpTgt := SnpTgt.ALL; require(x.getWidth == SnpOpcode.width); temp.asUInt }
  def snpOne  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.snoop    := true.B; temp.snpTgt := SnpTgt.ONE; require(x.getWidth == SnpOpcode.width); temp.asUInt }
  def snpOth  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.snoop    := true.B; temp.snpTgt := SnpTgt.OTH; require(x.getWidth == SnpOpcode.width); temp.asUInt }
  def read    (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.read     := true.B; require(x.getWidth == ReqOpcode.width); temp.asUInt }
  def dataless(x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.dataless := true.B; require(x.getWidth == ReqOpcode.width); temp.asUInt }
  def wriOrAtm(x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.wriOrAtm := true.B; require(x.getWidth == ReqOpcode.width); temp.asUInt }

  // Task Code DataOp
  // note0: cant set reqs expect of WriOrAtm
  // note1: cant set clean when commit will use DataBuffer
  def tdop(x: String*): UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); x.foreach(name => temp.dataOp.elements(name) := true.B); assert(!temp.dataOp.repl); temp.asUInt }

  // Task Code Other
  def waitRecDone       : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.waitRecDone  := true.B; temp.asUInt }
  def taskECA           : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.expCompAck   := true.B; temp.asUInt }
  def doDMT             : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.doDMT        := true.B; temp.asUInt }
  def retToSrc          : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.retToSrc     := true.B; temp.asUInt }
  def noTask            : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode()));                              temp.asUInt }

  // Commit Code Need Wait Second Task Done
  def waitSecDone       : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.waitSecDone  := true.B;   temp.asUInt }

  // Commit Code
  def sendResp          : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.sendResp     := true.B;    temp.asUInt }
  def sendFwdResp       : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.sendfwdResp  := true.B;    temp.asUInt }
  def cmtRsp  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.channel      := RSP;       temp.commitOp := x; require(x.getWidth == RspOpcode.width); temp.asUInt | sendResp }
  def cmtDat  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.channel      := DAT;       temp.commitOp := x; require(x.getWidth == DatOpcode.width); temp.asUInt | sendResp }
  def resp    (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.resp         := toResp(x); require(x.getWidth == DecodeCHI.width); temp.asUInt }
  def fwdResp (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.fwdResp      := toResp(x); require(x.getWidth == DecodeCHI.width); temp.asUInt | sendFwdResp }

  // Commit Code Write SF/LLC
  def wriSRC  (x: Boolean) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.srcValid  := x.asBool;    temp.wriSRC := true.B;   temp.asUInt }
  def wriSNP  (x: Boolean) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.snpValid  := x.asBool;    temp.wriSNP := true.B;   temp.asUInt }
  def wriLLC  (x: UInt)    : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.llcState  := toState(x);  temp.wriLLC := true.B;  require(x.getWidth == DecodeCHI.width); temp.asUInt }

  // Commit Code DataOp
  // If need write llc(valid) and not hit in llc, the save and clean operation will be handed over to ReplaceCM for execution.
  def cdop(x: String*): UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); x.foreach(name => temp.dataOp.elements(name) := true.B); assert(!temp.dataOp.repl); temp.asUInt }

  // CommitCode NoCMT or ERROR
  def noCmt : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.asUInt }

  // Use In Decode Table
  def first(commitCode: UInt): (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]) = (noTask, Seq(Inst.noResp -> (noTask, Seq(Inst.noResp -> commitCode))))

  def first(taskCode: UInt, commitCode: UInt): (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]) = (taskCode, Seq(Inst.noResp -> (noTask, Seq(Inst.noResp -> commitCode))))

  def first(taskCode: UInt, taskInst: UInt, commitCode: UInt): (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]) = (taskCode, Seq(taskInst -> (noTask, Seq(Inst.noResp -> commitCode))))

  def second(commitCode: UInt): (UInt, Seq[(UInt, UInt)]) = (noTask, Seq(Inst.noResp -> commitCode))

  def second(taskCode: UInt, commitCode: UInt): (UInt, Seq[(UInt, UInt)]) = (taskCode, Seq(Inst.noResp -> commitCode))

  def second(taskCode: UInt, taskInst: UInt, commitCode: UInt): (UInt, Seq[(UInt, UInt)]) = (taskCode, Seq(taskInst -> commitCode))
}


object Decode {
  type DecodeType = (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))])

  private val table = Read_LAN_DCT_DMT.table ++ Dataless_LAN.table ++ Write_LAN.table

  // ChiInst length
  val l_ci = table.length
  // StateInst length
  val l_si = table.map(_._2.length).max
  // TaskInst length
  val l_ti = table.map(_._2.map(_._2._2.length).max).max
  // SecTaskInst length
  val l_sti = table.map(_._2.map(_._2._2.map(_._2._2.length).max).max).max

  private val chiInst    = new ChiInst().getWidth
  private val stateInst  = new StateInst().getWidth
  private val taskCode   = new TaskCode().getWidth
  private val taskInst   = new TaskInst().getWidth
  private val commitCode = new CommitCode().getWidth

  def decode(chi: ChiInst, state: StateInst, task: TaskInst, secTask: TaskInst)(implicit p: Parameters, s: SourceInfo) = {

    val chiInstVec      = WireInit(VecInit(Seq.fill(l_ci) { 0.U(chiInst.W) }))
    val stateInstVec2   = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { 0.U(stateInst.W) }) }))
    val taskCodeVec2    = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { 0.U(taskCode.W)  }) }))
    val taskInstVec3    = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { VecInit(Seq.fill(l_ti) { 0.U(taskInst.W) }) }) }))
    val secCodeVec3     = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { VecInit(Seq.fill(l_ti) { 0.U(taskCode.W) }) }) }))
    val secInstVec4     = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { VecInit(Seq.fill(l_ti) { VecInit(Seq.fill(l_sti) { 0.U(taskInst.W)   }) }) }) }))
    val commitCodeVec4  = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { VecInit(Seq.fill(l_ti) { VecInit(Seq.fill(l_sti) { 0.U(commitCode.W) }) }) }) }))

    val table = Read_LAN_DCT_DMT.table ++ Dataless_LAN.table ++ Write_LAN.table

    table.zipWithIndex.foreach {
      case(t0, i) =>
        // require
        require(t0._1.getWidth == chiInst, s"($i) Width [${t0._1.getWidth}] =/= ChiInst Width [$chiInst]")
        // connect
        chiInstVec(i) := t0._1
        t0._2.zipWithIndex.foreach {
          case(t1, j) =>
            // require
            require(t1._1.getWidth    == stateInst, s"($i, $j) Width [${t1._1.getWidth}] =/= StateInst Width [$stateInst]")
            require(t1._2._1.getWidth == taskCode,  s"($i, $j) Width [${t1._2._1.getWidth}] =/= TaskCode Width [$taskCode]")
            // connect
            stateInstVec2(i)(j) := t1._1
            taskCodeVec2(i)(j)  := t1._2._1
            t1._2._2.zipWithIndex.foreach {
              case(t2, k) =>
                // require
                require(t2._1.getWidth    == taskInst, s"($i, $j, $k) Width [${t2._1.getWidth}] =/= TaskInst Width [$taskInst]")
                require(t2._2._1.getWidth == taskCode, s"($i, $j, $k) Width [${t2._2._1.getWidth}] =/= SecTaskCode Width [$taskCode]")
                // connect
                taskInstVec3(i)(j)(k) := t2._1
                secCodeVec3(i)(j)(k)  := t2._2._1
                t2._2._2.zipWithIndex.foreach {
                  case(t3, l) =>
                    // require
                    require(t3._1.getWidth == taskInst,   s"($i, $j, $k, $l) Width [${t3._1.getWidth}] =/= SecTaskInst Width [$taskInst]")
                    require(t3._2.getWidth == commitCode, s"($i, $j, $k, $l) Width [${t3._2.getWidth}] =/= CommitCode Width [$commitCode]")
                    // connect
                    secInstVec4(i)(j)(k)(l)     := t3._1
                    commitCodeVec4(i)(j)(k)(l)  := t3._2
                }
            }
        }
    }

    // First Input ChiInst
    val stateInstVec_0    = ParallelLookUp(chi.asUInt, chiInstVec.zip(stateInstVec2))
    val taskCodeVec_0     = ParallelLookUp(chi.asUInt, chiInstVec.zip(taskCodeVec2))

    val taskInstVec2_0    = ParallelLookUp(chi.asUInt, chiInstVec.zip(taskInstVec3))
    val secCodeVec2_0     = ParallelLookUp(chi.asUInt, chiInstVec.zip(secCodeVec3))

    val secInstVec3_0     = ParallelLookUp(chi.asUInt, chiInstVec.zip(secInstVec4))
    val cmtCodeVec3_0     = ParallelLookUp(chi.asUInt, chiInstVec.zip(commitCodeVec4))
    val cmtCodeVec_0      = cmtCodeVec3_0.map(_.head.head)

    // Second Input StateInst
    val taskCode_1        = ParallelLookUp(state.asUInt, stateInstVec_0.zip(taskCodeVec_0))

    val taskInstVec_1     = ParallelLookUp(state.asUInt, stateInstVec_0.zip(taskInstVec2_0))
    val secCodeVec_1      = ParallelLookUp(state.asUInt, stateInstVec_0.zip(secCodeVec2_0))

    val secInstVec2_1     = ParallelLookUp(state.asUInt, stateInstVec_0.zip(secInstVec3_0))
    val cmtCodeVec2_1     = ParallelLookUp(state.asUInt, stateInstVec_0.zip(cmtCodeVec3_0))

    // Third Input TaskInst
    val secCode_2         = ParallelLookUp(task.asUInt, taskInstVec_1.zip(secCodeVec_1))

    val secInstVec_2      = ParallelLookUp(task.asUInt, taskInstVec_1.zip(secInstVec2_1))
    val cmtCodeVec_2      = ParallelLookUp(task.asUInt, taskInstVec_1.zip(cmtCodeVec2_1))

    // Fourth Input SecTaskInst
    val cmtCode_3         = ParallelLookUp(secTask.asUInt, secInstVec_2.zip(cmtCodeVec_2))


    // Result
    val taskRes = taskCode_1.asTypeOf(new TaskCode())
    val secRes  = secCode_2.asTypeOf(new TaskCode())
    val cmtRes  = cmtCode_3.asTypeOf(new CommitCode())

    // HardwareAssertion valid
    HardwareAssertion.withEn(chi.valid & state.valid & task.valid, secTask.valid, cf"When SecTaskInst is valid, ChiInst[${chi.valid}] StateInst[${state.valid}] TaskInst[${task.valid}] should be valid")
    HardwareAssertion.withEn(chi.valid & state.valid,  task.valid, cf"When TaskInst is valid,  ChiInst[${chi.valid}] StateInst[${state.valid}] should be valid")
    HardwareAssertion.withEn(chi.valid,  state.valid, cf"When StateInst is valid, ChiInst[${chi.valid}] should be valid")
    // HardwareAssertion result
    // chiInstVecCf
    var chiInstVecCf = cf""
    chiInstVec.zipWithIndex.foreach { case(inst, i) =>
      chiInstVecCf = chiInstVecCf + cf"[$i] -> [${inst.asTypeOf(new ChiInst)}]\n"
    }
    // stateInstVecCf
    var stateInstVecCf = cf""
    stateInstVec_0.zipWithIndex.foreach { case (inst, i) =>
      stateInstVecCf = stateInstVecCf + cf"[$i] -> [${inst.asTypeOf(new StateInst)}]\n"
    }
    // taskInstVecCf
    var taskInstVecCf = cf""
    taskInstVec_1.zipWithIndex.foreach { case (inst, i) =>
      taskInstVecCf = taskInstVecCf + cf"[$i] -> [${inst.asTypeOf(new TaskInst)}]\n"
    }
    // secInstVecCf
    var secInstVecCf = cf""
    secInstVec_2.zipWithIndex.foreach { case (inst, i) =>
      secInstVecCf = secInstVecCf + cf"[$i] -> [${inst.asTypeOf(new TaskInst)}]\n"
    }
    HardwareAssertion.withEn(PopCount(Cat(chiInstVec.map(_.asUInt === chi.asUInt))) === 1.U,       chi.valid,
      cf"\n\nDECODE [ChiInst] ERROR\n\nChiInst:\n$chi\n\nAll Legal ChiInsts:\n" + chiInstVecCf + cf"\n")
    HardwareAssertion.withEn(PopCount(Cat(stateInstVec_0.map(_.asUInt === state.asUInt))) === 1.U, state.valid,
      cf"\n\nDECODE [StateInst] ERROR\n\nChiInst:\n$chi\nStateInst Invalid:\n$state\n\nAll Legal StateInsts:\n" + stateInstVecCf + cf"\n")
    HardwareAssertion.withEn(PopCount(Cat(taskInstVec_1.map(_.asUInt === task.asUInt))) === 1.U,   task.valid,
      cf"\n\nDECODE [TaskInst] ERROR\n\nChiInst:\n$chi\nStateInst:\n$state\nTaskInst:\n$task\n\nAll Legal TaskInsts:\n" + taskInstVecCf + cf"\n")
    HardwareAssertion.withEn(PopCount(Cat(secInstVec_2.map(_.asUInt === secTask.asUInt))) === 1.U, secTask.valid,
      cf"\n\nDECODE [SecTaskInst] ERROR\n\nChiInst:\n$chi\nStateInst:\n$state\nTaskInst:\n$task\nSecond TaskInst:\n$secTask\n\nAll Legal SecTaskInsts:\n" + secInstVecCf + cf"\n")

    // Return
    ((stateInstVec_0.map(_.asTypeOf(new StateInst)), taskCodeVec_0.map(_.asTypeOf(new TaskCode)), cmtCodeVec_0.map(_.asTypeOf(new CommitCode))), (taskRes, secRes, cmtRes))
  }

  def decode(chi: ChiInst)(implicit p: Parameters, s: SourceInfo):(Seq[StateInst], Seq[TaskCode], Seq[CommitCode]) = decode(chi, 0.U.asTypeOf(new StateInst), 0.U.asTypeOf(new TaskInst), 0.U.asTypeOf(new TaskInst))._1
}