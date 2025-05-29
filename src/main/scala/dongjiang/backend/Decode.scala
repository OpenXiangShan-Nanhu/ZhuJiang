  package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, HasPackDirMsg}
import dongjiang.data._
import zhujiang.chi.ReqOpcode._
import dongjiang.frontend._
import dongjiang.frontend.decode._
import dongjiang.backend.CmtState._
import chisel3.experimental.BundleLiterals._
import dongjiang.frontend.decode.Decode.{w_ci, w_si, w_sti, w_ti}

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- Get Decode Result ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class GetDecRes(implicit val p: Parameters) extends Module {
  val io = IO(new Bundle {
    val list        = Flipped(Valid(MixedVec(UInt(log2Ceil(Decode.l_ci).W), UInt(log2Ceil(Decode.l_si).W), UInt(log2Ceil(Decode.l_ti).W), UInt(log2Ceil(Decode.l_sti).W))))
    val taskCode    = Output(new TaskCode)
    val secTaskCode = Output(new TaskCode)
    val commitCode  = Output(new CommitCode)
  })
  dontTouch(io)

  val l = io.list.bits
  val ((chiInstVec, stateInstVec2, taskInstVec3, secInstVec4), (taskCodeVec2, secCodeVec3, commitCodeVec4)) = Decode.parse

  io.taskCode     := taskCodeVec2(l(0))(l(1)).asTypeOf(new TaskCode)
  io.secTaskCode  := secCodeVec3(l(0))(l(1))(l(2)).asTypeOf(new TaskCode)
  io.commitCode   := commitCodeVec4(l(0))(l(1))(l(2))(l(3)).asTypeOf(new CommitCode)

  HAssert.withEn(chiInstVec(l(0)).asTypeOf(new ChiInst).valid,                      io.list.valid, "CHI Instruction access out of bounds")
  HAssert.withEn(stateInstVec2(l(0))(l(1)).asTypeOf(new StateInst).valid,           io.list.valid, "State Instruction access out of bounds")
  HAssert.withEn(taskInstVec3(l(0))(l(1))(l(2)).asTypeOf(new TaskInst).valid,       io.list.valid, "Task Instruction access out of bounds")
  HAssert.withEn(secInstVec4(l(0))(l(1))(l(2))(l(3)).asTypeOf(new TaskInst).valid,  io.list.valid, "Second Task Instruction access out of bounds")
}


// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- Decode ---------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class Decode(times: String)(implicit p: Parameters) extends DJModule {
  override val desiredName = s"Decode${times.toUpperCase}"
  val io = IO(new Bundle {
    // Decode List
    val listIn      = Input(MixedVec(UInt(log2Ceil(Decode.l_ci).W), UInt(log2Ceil(Decode.l_si).W), UInt(log2Ceil(Decode.l_ti).W), UInt(log2Ceil(Decode.l_sti).W)))
    val listOut     = Output(MixedVec(UInt(log2Ceil(Decode.l_ci).W), UInt(log2Ceil(Decode.l_si).W), UInt(log2Ceil(Decode.l_ti).W), UInt(log2Ceil(Decode.l_sti).W)))
    // Task Inst
    val inst0       = Input(new TaskInst)
    val inst1       = Input(new TaskInst)
    // Task/Commit Code
    val taskCode    = Output(new TaskCode)
    val cmtCode     = Output(new CommitCode)
    // HnTxnID
    val hnTxnIdIn   = Input(UInt(hnTxnIDBits.W))
    val hnTxnIdOut  = Output(UInt(hnTxnIDBits.W))
    // Ouput
    val valid       = Output(Bool())
  })
  dontTouch(io)
  require(times == "Third" | times == "Fourth")

  /*
   * Merge
   */
  val mergeVal    = io.inst0.valid & io.inst1.valid
  val mergeInst   = WireInit((io.inst0.asUInt | io.inst1.asUInt).asTypeOf(new TaskInst))
  mergeInst.valid := mergeVal

  /*
   * Decode
   */
  val decList     = if(times == "Third") Decode.thirdDec(io.listIn, mergeInst, io.hnTxnIdIn) else Decode.fourthDec(io.listIn, mergeInst, io.hnTxnIdIn)

  /*
   * GetResult
   */
  val getRes      = Module(new GetDecRes())
  val taskCode    = if(times == "Third") getRes.io.secTaskCode else 0.U.asTypeOf(new TaskCode)
  val cmtCode     = getRes.io.commitCode
  getRes.io.list.valid  := mergeVal
  getRes.io.list.bits   := decList

  /*
   * Output
   */
  io.valid      := RegNext(mergeVal)
  io.listOut    := RegEnable(decList,       mergeVal)
  io.taskCode   := RegEnable(taskCode,      mergeVal)
  io.cmtCode    := RegEnable(cmtCode,       mergeVal)
  io.hnTxnIdOut := RegEnable(io.hnTxnIdIn,  mergeVal)
}