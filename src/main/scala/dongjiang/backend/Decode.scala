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
    // Message In
    val decMesIn      = Flipped(Valid(new DJBundle with HasPackTaskInst with HasDecList with HasHnTxnID))
    // Result Out
    val hnTxnIdOut    = Valid(UInt(hnTxnIDBits.W))
    val decListOut    = Output(MixedVec(UInt(log2Ceil(Decode.l_ci).W), UInt(log2Ceil(Decode.l_si).W), UInt(log2Ceil(Decode.l_ti).W), UInt(log2Ceil(Decode.l_sti).W)))
    val taskCodeOut   = Output(new TaskCode)
    val cmtCodeOut    = Output(new CommitCode)
  })
  dontTouch(io)
  require(times == "Third" | times == "Fourth")

  /*
   * Input
   */
  val decValReg   = RegNext(io.decMesIn.valid)
  val decMesReg   = RegEnable(io.decMesIn.bits, io.decMesIn.valid)

  /*
   * Decode
   */
  val taskInst    = WireInit(decMesReg.taskInst)
  taskInst.valid  := decValReg & decMesReg.taskInst.valid
  HAssert.withEn(decMesReg.taskInst.valid, decValReg)
  // decode
  val decList     = if(times == "Third") {
    Decode.thirdDec(decMesReg.decList, taskInst, decMesReg.hnTxnID)
  } else {
    Decode.fourthDec(decMesReg.decList, taskInst, decMesReg.hnTxnID)
  }


  /*
   * GetResult
   */
  val getRes            = Module(new GetDecRes())
  val taskCode          = if(times == "Third") getRes.io.secTaskCode else 0.U.asTypeOf(new TaskCode)
  val cmtCode           = getRes.io.commitCode
  getRes.io.list.valid  := decValReg
  getRes.io.list.bits   := decList

  /*
   * Output
   */
  io.hnTxnIdOut.valid   := RegNext(decValReg)
  io.hnTxnIdOut.bits    := RegEnable(decMesReg.hnTxnID, decValReg)
  io.decListOut         := RegEnable(decList,           decValReg)
  io.taskCodeOut        := RegEnable(taskCode,          decValReg)
  io.cmtCodeOut         := RegEnable(cmtCode,           decValReg)
}