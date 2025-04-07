package dongjiang.frontend.decode

import chisel3._
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

class ChiInst extends Bundle {
  // REQ: LAN -> LAN; LAN -> BBN; BBN -> LAN;
  // SNP: BBN -> LAN
  val channel     = UInt(ChiChannel.width.W)
  val fromLAN     = Bool() // true -> LAN; false -> BBN
  val toLAN       = Bool() // true -> LAN; false -> BBN
  val opcode      = UInt(ReqOpcode.width.max(SnpOpcode.width).W)
  val expCompAck  = Bool()
}

class StateInst extends Bundle {
  val valid       = Bool()
  val srcHit      = Bool()
  val othHit      = Bool()
  val llcState    = UInt(ChiState.width.W)
}

class TaskInst extends Bundle {
  val valid       = Bool()
  val fwdValid    = Bool()
  val channel     = UInt(ChiChannel.width.W)
  val opcode      = UInt(RspOpcode.width.max(DatOpcode.width).W)
  val resp        = UInt(ChiResp.width.W)
  val fwdResp     = UInt(ChiResp.width.W)
}

trait HasPackTaskInst { this: Bundle => val inst = new TaskInst() }

object CMID {
  lazy val SNP  = 0
  lazy val READ = 1
  lazy val DL   = 2
  lazy val WOA  = 3
}

trait HasOperations { this: Bundle =>
  val snoop       = Bool()
  val read        = Bool()
  val dataless    = Bool()
  val wriOrAtm    = Bool() // Write or Atomic
  def valid       = snoop | read | dataless | wriOrAtm
  def invalid     = !valid
  def cmid: UInt  = {
    PriorityMux(Seq(
      snoop    -> CMID.SNP.U,
      read     -> CMID.READ.U,
      dataless -> CMID.DL.U,
      wriOrAtm -> CMID.WOA.U,
    ))
  }
}

class Operations extends Bundle with HasOperations

trait HasPackOperations { this: Bundle => val ops = new Operations() }

object SnpTgt {
  val width       = 3
  val NONE        = "b000".U
  val ALL         = "b001".U
  val ONE         = "b010".U // Select first other
  val OTH         = "b100".U
}

trait HasSnpTgt { this: Bundle => val snpTgt = UInt(SnpTgt.width.W) }

trait HasTaskCode { this: Bundle with HasOperations =>
  val flag        = Bool()

  // Common
  val opcode      = UInt(ReqOpcode.width.max(SnpOpcode.width).W)
  val needDB      = Bool()
  val canNest     = Bool()

  // Req
  val expCompAck  = Bool()
  def doDMT       = read & !needDB

  // Snoop
  val retToSrc    = Bool()
  val snpTgt      = UInt(SnpTgt.width.W)
  def snpAll      = snpTgt(0).asBool
  def snpOne      = snpTgt(1).asBool
  def snpOth      = snpTgt(2).asBool

  // TODO: MemAttr
}

class TaskCode extends Bundle with HasOperations with HasTaskCode

trait HasPackTaskCode { this: Bundle => val code = new TaskCode() }

trait HasWriDirCode { this: Bundle =>
  // Write Directory
  val wriSF       = UInt(SnpTgt.width.W)
  val wriLLC      = Bool()
  val srcValid    = Bool()
  val snpValid    = Bool()
  val llcState    = UInt(ChiState.width.W)

  def wriDir      = wriSF | wriLLC
}

class WriDirCode  extends Bundle with HasWriDirCode

trait HasPackWriDirCode { this: Bundle => val code = new WriDirCode() }

trait HasCommitCode { this: Bundle with HasWriDirCode =>
  val flag        = Bool()

  // Need wait second task done
  val waitSecDone = Bool()

  // Commit
  val commit      = Bool()
  val fwdCommit   = Bool()
  val channel     = UInt(ChiChannel.width.W)
  val commitOp    = UInt(RspOpcode.width.max(DatOpcode.width).W)
  val resp        = UInt(ChiResp.width.W)
  val fwdResp     = UInt(ChiResp.width.W)

  // Not Need Commit
  val noCmt       = Bool()

  // def
  def valid       = commit | noCmt | wriDir
  def invalid     = !valid
}

class CommitCode extends Bundle with HasWriDirCode with HasCommitCode

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
  def isReq             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.channel       := REQ;     temp.asUInt }
  def isSnp             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.channel       := SNP;     temp.asUInt }
  def fromLAN           : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.fromLAN       := true.B;  temp.asUInt }
  def fromBBN           : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.fromLAN       := false.B; temp.asUInt }
  def toLAN             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.toLAN         := true.B;  temp.asUInt }
  def toBBN             : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.toLAN         := false.B; temp.asUInt }
  def reqIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.opcode        := x;       require(x.getWidth == ReqOpcode.width); temp.asUInt | isReq }
  def snpIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.opcode        := x;       require(x.getWidth == RspOpcode.width); temp.asUInt | isSnp }
  def expCompAck        : UInt = { val temp = WireInit(0.U.asTypeOf(new ChiInst())); temp.expCompAck    := true.B;  temp.asUInt }

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
  def isRsp             : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.channel      := RSP;       temp.asUInt | taskValid}
  def isDat             : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.channel      := DAT;       temp.asUInt | taskValid}
  def rspIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.opcode       := x;         require(x.getWidth == RspOpcode.width); temp.asUInt | isRsp }
  def datIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.opcode       := x;         require(x.getWidth == DatOpcode.width); temp.asUInt | isDat }
  def respIs  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.resp         := toResp(x); require(x.getWidth == DecodeCHI.width);   temp.asUInt }
  def fwdIs   (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst())); temp.fwdResp      := toResp(x); require(x.getWidth == DecodeCHI.width);   temp.asUInt | fwdValid}
  def noResp            : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskInst()));                                 temp.asUInt }
}



object Code {
  // Flag
  def taskFlag : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode()));   temp.flag := true.B; temp.asUInt }
  def cmtFlag  : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.flag := true.B; temp.asUInt }

  // Task Code Operations
  def snpAll  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.snoop    := true.B; temp.snpTgt := SnpTgt.ALL; require(x.getWidth == SnpOpcode.width); temp.asUInt | taskFlag }
  def snpOne  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.snoop    := true.B; temp.snpTgt := SnpTgt.ONE; require(x.getWidth == SnpOpcode.width); temp.asUInt | taskFlag }
  def snpOth  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.snoop    := true.B; temp.snpTgt := SnpTgt.OTH; require(x.getWidth == SnpOpcode.width); temp.asUInt | taskFlag }
  def read    (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.read     := true.B; require(x.getWidth == ReqOpcode.width); temp.asUInt | taskFlag }
  def dataless(x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.dataless := true.B; require(x.getWidth == ReqOpcode.width); temp.asUInt | taskFlag }
  def wriOrAtm(x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.opcode := x; temp.wriOrAtm := true.B; require(x.getWidth == ReqOpcode.width); temp.asUInt | taskFlag }

  // Task Code Other
  def needDB            : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.needDB      := true.B; temp.asUInt }
  def canNest           : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.canNest     := true.B; temp.asUInt }
  def taskECA           : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.expCompAck  := true.B; temp.asUInt }
  def retToSrc          : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode())); temp.retToSrc    := true.B; temp.asUInt }
  def noTask            : UInt = { val temp = WireInit(0.U.asTypeOf(new TaskCode()));                             temp.asUInt | taskFlag }



  // Commit Code Need Wait Second Task Done
  def waitSecDone       : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.waitSecDone := true.B;   temp.asUInt }

  // Commit Code Commit
  def commit            : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.commit     := true.B;    temp.asUInt | cmtFlag }
  def fwdCommit         : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.fwdCommit  := true.B;    temp.asUInt }
  def cmtRsp  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.channel    := RSP;       temp.commitOp := x; require(x.getWidth == RspOpcode.width); temp.asUInt | commit }
  def cmtDat  (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.channel    := DAT;       temp.commitOp := x; require(x.getWidth == DatOpcode.width); temp.asUInt | commit }
  def resp    (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.resp       := toResp(x); require(x.getWidth == DecodeCHI.width); temp.asUInt }
  def fwdResp (x: UInt) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.fwdCommit  := toResp(x); require(x.getWidth == DecodeCHI.width); temp.asUInt | fwdCommit }

  // CommitCode Write SF/LLC
  def wriSRC  (x: Boolean) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.srcValid   := x.asBool;    temp.wriSF := true.B;   temp.asUInt | cmtFlag }
  def wriSNP  (x: Boolean) : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.snpValid   := x.asBool;    temp.wriSF := true.B;   temp.asUInt | cmtFlag }
  def wriLLC  (x: UInt)    : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.llcState   := toState(x);  temp.wriLLC := true.B;  require(x.getWidth == DecodeCHI.width); temp.asUInt | cmtFlag }


  // CommitCode NoCMT or ERROR
  def noCmt             : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode())); temp.noCmt := true.B; temp.asUInt | cmtFlag }
  def error             : UInt = { val temp = WireInit(0.U.asTypeOf(new CommitCode()));                       temp.asUInt }


  // Use In Decode Table
  def first(commitCode: UInt): (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]) = (noTask, Seq(Inst.noResp -> (noTask, Seq(Inst.noResp -> commitCode))))

  def first(taskCode: UInt, commitCode: UInt): (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]) = (taskCode, Seq(Inst.noResp -> (noTask, Seq(Inst.noResp -> commitCode))))

  def first(taskCode: UInt, taskInst: UInt, commitCode: UInt): (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]) = (taskCode, Seq(taskInst -> (noTask, Seq(Inst.noResp -> commitCode))))

  def second(commitCode: UInt): (UInt, Seq[(UInt, UInt)]) = (noTask, Seq(Inst.noResp -> commitCode))

  def second(taskCode: UInt, commitCode: UInt): (UInt, Seq[(UInt, UInt)]) = (taskCode, Seq(Inst.noResp -> commitCode))
}


object Decode {
  def decode(chi: ChiInst = 0.U.asTypeOf(new ChiInst), state: StateInst = 0.U.asTypeOf(new StateInst), task: TaskInst = 0.U.asTypeOf(new TaskInst), secTask: TaskInst = 0.U.asTypeOf(new TaskInst)) = {
    val table = Read_LAN_DCT_DMT.table ++ Dataless_LAN.table

    // ChiInst length
    val l_ci  = table.length
    // StateInst length
    val l_si  = table.map(_._2.length).max
    // TaskInst length
    val l_ti  = table.map(_._2.map(_._2._2.length).max).max
    // SecTaskInst length
    val l_sti = table.map(_._2.map(_._2._2.map(_._2._2.length).max).max).max

    val chiInstVec      = WireInit(VecInit(Seq.fill(l_ci) { 0.U.asTypeOf(new ChiInst) }))
    val stateInstVec2   = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { 0.U.asTypeOf(new StateInst)  }) }))
    val taskCodeVec2    = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { 0.U.asTypeOf(new TaskCode)   }) }))
    val taskInstVec3    = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { VecInit(Seq.fill(l_ti) { 0.U.asTypeOf(new TaskInst)   }) }) }))
    val secCodeVec3     = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { VecInit(Seq.fill(l_ti) { 0.U.asTypeOf(new TaskCode)   }) }) }))
    val secInstVec4     = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { VecInit(Seq.fill(l_ti) { VecInit(Seq.fill(l_sti) { 0.U.asTypeOf(new TaskInst)   }) }) }) }))
    val commitCodeVec4  = WireInit(VecInit(Seq.fill(l_ci) { VecInit(Seq.fill(l_si) { VecInit(Seq.fill(l_ti) { VecInit(Seq.fill(l_sti) { 0.U.asTypeOf(new CommitCode) }) }) }) }))

    table.zipWithIndex.foreach {
      case(t0, i) =>
        // require
        require(t0._1.getWidth == new ChiInst().getWidth, s"($i) Width [${t0._1.getWidth}] =/= ChiInst Width [${new ChiInst().getWidth}]")
        // connect
        chiInstVec(i) := t0._1.asTypeOf(new ChiInst)
        t0._2.zipWithIndex.foreach {
          case(t1, j) =>
            // require
            require(t1._1.getWidth    == new StateInst().getWidth, s"($i, $j) Width [${t1._1.getWidth}] =/= StateInst Width [${new StateInst().getWidth}]")
            require(t1._2._1.getWidth == new TaskCode().getWidth,  s"($i, $j) Width [${t1._2._1.getWidth}] =/= TaskCode Width [${new TaskCode().getWidth}]")
            // connect
            stateInstVec2(i)(j) := t1._1.asTypeOf(new StateInst)
            taskCodeVec2(i)(j)  := t1._2._1.asTypeOf(new TaskCode)
            t1._2._2.zipWithIndex.foreach {
              case(t2, k) =>
                // require
                require(t2._1.getWidth    == new TaskInst().getWidth, s"($i, $j, $k) Width [${t2._1.getWidth}] =/= TaskInst Width [${new TaskInst().getWidth}]")
                require(t2._2._1.getWidth == new TaskCode().getWidth, s"($i, $j, $k) Width [${t2._2._1.getWidth}] =/= SecTaskCode Width [${new TaskCode().getWidth}]")
                // connect
                taskInstVec3(i)(j)(k) := t2._1.asTypeOf(new TaskInst)
                secCodeVec3(i)(j)(k)  := t2._2._1.asTypeOf(new TaskCode)
                t2._2._2.zipWithIndex.foreach {
                  case(t3, l) =>
                    // require
                    require(t3._1.getWidth == new TaskInst().getWidth,   s"($i, $j, $k, $l) Width [${t3._1.getWidth}] =/= SecTaskInst Width [${new TaskInst().getWidth}]")
                    require(t3._2.getWidth == new CommitCode().getWidth, s"($i, $j, $k, $l) Width [${t3._2.getWidth}] =/= CommitCode Width [${new CommitCode().getWidth}]")
                    // connect
                    secInstVec4(i)(j)(k)(l)     := t3._1.asTypeOf(new TaskInst)
                    commitCodeVec4(i)(j)(k)(l)  := t3._2.asTypeOf(new CommitCode)
                }
            }
        }
    }

    // First Input ChiInst
    val stateInstVec_0    = ParallelLookUp(chi.asUInt,    chiInstVec.map(_.asUInt).zip(stateInstVec2))
    val taskCodeVec_0     = ParallelLookUp(chi.asUInt,    chiInstVec.map(_.asUInt).zip(taskCodeVec2))

    val taskInstVec2_0    = ParallelLookUp(chi.asUInt,    chiInstVec.map(_.asUInt).zip(taskInstVec3))
    val secCodeVec2_0     = ParallelLookUp(chi.asUInt,    chiInstVec.map(_.asUInt).zip(secCodeVec3))

    val secInstVec3_0     = ParallelLookUp(chi.asUInt,    chiInstVec.map(_.asUInt).zip(secInstVec4))
    val cmtCodeVec3_0     = ParallelLookUp(chi.asUInt,    chiInstVec.map(_.asUInt).zip(commitCodeVec4))
    val cmtCodeVec_0      = cmtCodeVec3_0.map(_.head.head)

    // Second Input StateInst
    val taskCode_1        = ParallelLookUp(state.asUInt,  stateInstVec_0.map(_.asUInt).zip(taskCodeVec_0))

    val taskInstVec_1     = ParallelLookUp(state.asUInt,  stateInstVec_0.map(_.asUInt).zip(taskInstVec2_0))
    val secCodeVec_1      = ParallelLookUp(state.asUInt,  stateInstVec_0.map(_.asUInt).zip(secCodeVec2_0))

    val secInstVec2_1     = ParallelLookUp(state.asUInt,  stateInstVec_0.map(_.asUInt).zip(secInstVec3_0))
    val cmtCodeVec2_1     = ParallelLookUp(state.asUInt,  stateInstVec_0.map(_.asUInt).zip(cmtCodeVec3_0))

    // Third Input TaskInst
    val secCode_2         = ParallelLookUp(task.asUInt,   taskInstVec_1.map(_.asUInt).zip(secCodeVec_1))

    val secInstVec_2      = ParallelLookUp(task.asUInt,   taskInstVec_1.map(_.asUInt).zip(secInstVec2_1))
    val cmtCodeVec_2      = ParallelLookUp(task.asUInt,   taskInstVec_1.map(_.asUInt).zip(cmtCodeVec2_1))

    // Fourth Input SecTaskInst
    val cmtCode_3         = ParallelLookUp(secTask.asUInt, secInstVec_2.map(_.asUInt).zip(cmtCodeVec_2))

    ((stateInstVec_0, taskCodeVec_0, cmtCodeVec_0), (taskInstVec_1, secCodeVec_1), (secInstVec_2, cmtCodeVec_2), (taskCode_1, secCode_2, cmtCode_3), (l_ci, l_si ,l_ti, l_sti))
  }

  def decode(inst: StateInst, instVec: Vec[StateInst], codeVec: Vec[TaskCode]): TaskCode = ParallelLookUp(inst.asUInt, instVec.map(_.asUInt).zip(codeVec))

  def decode(inst: StateInst, instVec: Vec[StateInst], cmtVec: Vec[CommitCode]): CommitCode = ParallelLookUp(inst.asUInt, instVec.map(_.asUInt).zip(cmtVec))

  def decode(inst: TaskInst, instVec: Vec[TaskInst], cmtVec: Vec[CommitCode]): CommitCode = ParallelLookUp(inst.asUInt, instVec.map(_.asUInt).zip(cmtVec))

  def l_ci: Int = decode()._5._1

  def l_si: Int = decode()._5._2

  def l_ti: Int = decode()._5._3

  def l_sti: Int = decode()._5._4
}