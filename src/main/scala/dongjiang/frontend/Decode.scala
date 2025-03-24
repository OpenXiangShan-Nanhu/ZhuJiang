package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.directory._
import xs.utils.debug._

import dongjiang.frontend.decode._

class Decode(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config      = new DJConfigIO()
    val task_s2     = Flipped(Valid(new DJBundle {
      val chi       = new ChiTask with HasAddr with HasFastChiInst
      val pos       = new PosIndex()
    }))
    val respDir_s3  = Flipped(Valid(new DJBundle {
      val dir       = new DirMsg with HasFastStateInst
      val alrDeqDB  = Bool()
    }))
    val task_s3     = Valid(new DJBundle {
      val chi       = new ChiTask with HasAddr
      val pos       = new PosIndex()
      val dir       = new DirMsg()
      val alrDeqDB  = Bool()
      val code      = new TaskCode()
      val commit    = new CommitCode()
    })
    val canNest_s3  = Valid(new PosIndex())
  })

  dontTouch(io)

  /*
   * Reg and Wire declaration
   */
  val chiInst_s2          = Wire(new ChiInst())
  val stateInstVecReg_s3  = RegInit(VecInit(Seq.fill(Decode.l_si) { 0.U.asTypeOf(new StateInst)  }))
  val taskCodeVecReg_s3   = RegInit(VecInit(Seq.fill(Decode.l_si) { 0.U.asTypeOf(new TaskCode)   }))
  val commitVecReg_s3     = RegInit(VecInit(Seq.fill(Decode.l_si) { 0.U.asTypeOf(new CommitCode) }))
  val validReg_s3         = RegNext(io.task_s2.valid)
  val taskReg_s3          = RegEnable(io.task_s2.bits, 0.U.asTypeOf(io.task_s2.bits), io.task_s2.valid)
  val stateInst_s3        = Wire(new StateInst())

  /*
   * [S2]: Pre-Decode
   */
  chiInst_s2          := io.task_s2.bits.chi.getInst(io.config.ci)
  stateInstVecReg_s3  := Decode.decode(chiInst_s2)._1._1
  taskCodeVecReg_s3   := Decode.decode(chiInst_s2)._1._2
  commitVecReg_s3     := Decode.decode(chiInst_s2)._1._3

  /*
   * [S3]: Decode
   */
  val metaId_s3   = taskReg_s3.chi.metaId
  val dirValid_s3 = io.respDir_s3.valid
  stateInst_s3    := Mux(dirValid_s3, io.respDir_s3.bits.dir.getInst(metaId_s3), 0.U.asTypeOf(stateInst_s3))
  HardwareAssertion.withEn(io.respDir_s3.valid, validReg_s3 & taskReg_s3.chi.memAttr.cacheable)

  /*
   * [S3]: Decode
   */
  val code_s3 = Decode.decode(stateInst_s3, stateInstVecReg_s3, taskCodeVecReg_s3)
  val cmt_s3  = Decode.decode(stateInst_s3, stateInstVecReg_s3, commitVecReg_s3)

  /*
   * [S3]: Output S3
   */
  // task_s3
  io.task_s3.valid          := validReg_s3
  io.task_s3.bits.chi       := taskReg_s3.chi
  io.task_s3.bits.pos       := taskReg_s3.pos
  io.task_s3.bits.dir       := Mux(dirValid_s3, io.respDir_s3.bits.dir, 0.U.asTypeOf(io.respDir_s3.bits.dir))
  io.task_s3.bits.alrDeqDB  := io.respDir_s3.bits.alrDeqDB
  io.task_s3.bits.code      := code_s3
  io.task_s3.bits.commit    := Mux(code_s3.valid, 0.U.asTypeOf(new CommitCode), cmt_s3)
  // canNest_s3
  io.canNest_s3.valid       := validReg_s3 & code_s3.canNest
  io.canNest_s3.bits        := taskReg_s3.pos


  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(Int.MaxValue-2)
}