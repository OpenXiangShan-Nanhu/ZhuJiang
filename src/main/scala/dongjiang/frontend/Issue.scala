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

class Issue(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config      = new DJConfigIO()
    // In
    val task_s3     = Flipped(Valid(new DJBundle {
      val chi       = new ChiTask with HasAddr
      val pos       = new PosIndex()
      val dir       = new DirMsg()
      val alrDeqDB  = Bool()
      val code      = new TaskCode()
      val commit    = new CommitCode()
    }))
    // Out
    val commit_s3   = Valid(new DJBundle {
      val chi       = new ChiTask
      val pos       = new PosIndex()
      val dir       = new DirMsg()
      val alrDeqDB  = Bool()
      val hasOps    = Bool()
      val commit    = new CommitCode()
    })
    val cmAlloc_s4  = new DJBundle {
      val recOps    = Input(new Operations())
      val ops       = Output(new Operations())
      val task      = new DJBundle {
        val chi     = new ChiTask with HasAddr
        val pos     = new PosIndex()
        val ops     = new Operations()
        val alrDeqDB= Bool()
        val snpVec  = Vec(nrSfMetas, Bool())
      }
    }
    val alrUseBuf   = Output(UInt((issueBufBits+1).W))
  })
  dontTouch(io)

  /*
   * Module and Reg declaration
   */
  val allVec_s3         = Wire(Vec(nrSfMetas, Bool()))
  val othVec_s3         = Wire(Vec(nrSfMetas, Bool()))
  val oneVec_s3         = Wire(Vec(nrSfMetas, Bool()))
  val cmTask_s3         = Wire(chiselTypeOf(io.cmAlloc_s4.task))
  val cmTaskBufReg_s4   = Reg(Vec(nrIssueBuf, chiselTypeOf(io.cmAlloc_s4.task)))
  val taskOpsVecReg_s4  = RegInit(VecInit(Seq.fill(nrIssueBuf) { 0.U.asTypeOf(new Operations()) }))
  val taskNidVecReg_s4  = RegInit(VecInit(Seq.fill(nrIssueBuf) { 0.U(issueBufBits.W) }))


  /*
   * [S3] Receive task and trans to commit
   */
  io.commit_s3.valid          := io.task_s3.valid
  io.commit_s3.bits.chi       := io.task_s3.bits.chi
  io.commit_s3.bits.pos       := io.task_s3.bits.pos
  io.commit_s3.bits.dir       := io.task_s3.bits.dir
  io.commit_s3.bits.alrDeqDB  := io.task_s3.bits.alrDeqDB
  io.commit_s3.bits.hasOps    := io.task_s3.bits.code.valid
  io.commit_s3.bits.commit    := io.task_s3.bits.commit


  /*
   * [S3] Receive task and trans to cmTask
   */
  val task_s3               = io.task_s3.bits
  cmTask_s3.pos             := task_s3.pos
  cmTask_s3.ops             := task_s3.code
  cmTask_s3.alrDeqDB        := task_s3.alrDeqDB
  cmTask_s3.chi             := task_s3.chi
  // set by decode
  cmTask_s3.chi.channel     := Mux(task_s3.code.snoop, ChiChannel.SNP, ChiChannel.REQ)
  cmTask_s3.chi.opcode      := task_s3.code.opcode
  cmTask_s3.chi.expCompAck  := task_s3.code.expCompAck
  cmTask_s3.chi.retToSrc    := task_s3.code.retToSrc
  // snp tgt vec
  val metaId_s3 = task_s3.chi.metaId
  allVec_s3 := task_s3.dir.sf.allVec
  othVec_s3 := task_s3.dir.sf.othVec(metaId_s3)
  oneVec_s3 := PriorityEncoderOH(othVec_s3)
  cmTask_s3.snpVec := PriorityMux(Seq(
    task_s3.code.snpAll -> allVec_s3,
    task_s3.code.snpOth -> othVec_s3,
    task_s3.code.snpOne -> oneVec_s3,
    true.B              -> 0.U.asTypeOf(cmTask_s3.snpVec)
  ))


  /*
   * [S4] Save cmTask
   */
  val freeVec_s4  = taskOpsVecReg_s4.map(!_.asUInt.orR)
  val freeId_s4   = PriorityEncoder(freeVec_s4)
  val deqOpHit_s4 = io.cmAlloc_s4.recOps.asUInt === cmTask_s3.ops.asUInt
  val nid_s4      = PopCount(taskOpsVecReg_s4.map(ops => ops.asUInt === cmTask_s3.ops.asUInt))
  HardwareAssertion.withEn(freeVec_s4.reduce(_ | _), io.task_s3.valid)
  HardwareAssertion.withEn(nid_s4 > 0.U, deqOpHit_s4)

  when(io.task_s3.valid) {
    taskOpsVecReg_s4(freeId_s4) := cmTask_s3.ops
    cmTaskBufReg_s4(freeId_s4)  := cmTask_s3
    taskNidVecReg_s4(freeId_s4) := nid_s4 - deqOpHit_s4
  }

  /*
   * Select Buffer to Output
   */
  val canRec_s4       = io.cmAlloc_s4.recOps.asUInt.orR
  val selVec_s4       = taskOpsVecReg_s4.zip(taskNidVecReg_s4).map { case(ops, id) => ops.asUInt === io.cmAlloc_s4.recOps.asUInt & id === 0.U }
  val selId_s4        = PriorityEncoder(selVec_s4)
  io.cmAlloc_s4.ops   := taskOpsVecReg_s4.map(_.asUInt).reduce(_ | _).asTypeOf(new Operations())
  io.cmAlloc_s4.task  := cmTaskBufReg_s4(selId_s4)
  HardwareAssertion.withEn(selVec_s4.reduce(_ | _), canRec_s4)

  when(canRec_s4) {
    taskOpsVecReg_s4(selId_s4)  := 0.U.asTypeOf(new Operations())
  }

  /*
   * Output Already Use Buffer Number
   */
  val useVec_s4 = taskOpsVecReg_s4.map(_.asUInt.orR)
  io.alrUseBuf  := PopCount(useVec_s4)

  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(Int.MaxValue-2)
}