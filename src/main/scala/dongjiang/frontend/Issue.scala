package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.data.HasAlrDB
import dongjiang.directory._
import xs.utils.debug._
import dongjiang.frontend.decode._
import dongjiang.backend._

class Issue(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config        = new DJConfigIO()
    // In
    val task_s3       = Flipped(Valid(new PackChi with HasAddr with HasPackPosIndex with HasPackDirMsg with HasAlrDB with HasPackTaskCode with HasPackCmtCode))
    // Out
    val cmtAlloc_s3   = Valid(new CommitTask())
    val cmAllocVec_s4 = Vec(nrTaskCM, Decoupled(new CMTask()))
    val alrUseBuf     = Output(UInt((issueBufBits+1).W))
  })
  dontTouch(io)

  /*
   * Module and Reg declaration
   */
  // s3
  val allVec_s3         = Wire(Vec(nrSfMetas, Bool()))
  val othVec_s3         = Wire(Vec(nrSfMetas, Bool()))
  val oneVec_s3         = Wire(Vec(nrSfMetas, Bool()))
  val cmTask_s3         = Wire(chiselTypeOf(io.cmAllocVec_s4.head.bits))
  // s4
  val taskOpsVecReg_s4  = RegInit(VecInit(Seq.fill(nrIssueBuf) { 0.U.asTypeOf(Valid(new Operations())) }))
  val taskNidVecReg_s4  = RegInit(VecInit(Seq.fill(nrIssueBuf) { 0.U(issueBufBits.W) }))
  val cmTaskBufReg_s4   = Reg(Vec(nrIssueBuf, chiselTypeOf(io.cmAllocVec_s4.head.bits)))


  /*
   * [S3] Receive task and trans to commit
   */
  io.cmtAlloc_s3.valid          := io.task_s3.valid
  io.cmtAlloc_s3.bits.chi       := io.task_s3.bits.chi
  io.cmtAlloc_s3.bits.pos       := io.task_s3.bits.pos
  io.cmtAlloc_s3.bits.dir       := io.task_s3.bits.dir
  io.cmtAlloc_s3.bits.alrDB     := io.task_s3.bits.alrDB
  io.cmtAlloc_s3.bits.commit    := io.task_s3.bits.commit


  /*
   * [S3] Receive task and trans to cmTask
   */
  val task_s3               = io.task_s3.bits
  cmTask_s3.addr            := task_s3.addr
  cmTask_s3.llcTxnID.pos    := task_s3.pos
  cmTask_s3.llcTxnID.dirBank:= dirBank.U
  cmTask_s3.chi             := task_s3.chi
  cmTask_s3.alrDB           := task_s3.alrDB
  // set by decode
  cmTask_s3.chi.channel     := Mux(task_s3.code.snoop, ChiChannel.SNP, ChiChannel.REQ)
  cmTask_s3.chi.opcode      := task_s3.code.opcode
  cmTask_s3.chi.expCompAck  := task_s3.code.expCompAck
  cmTask_s3.chi.retToSrc    := task_s3.code.retToSrc
  cmTask_s3.needDB          := task_s3.code.needDB
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
  val freeVec_s3  = taskOpsVecReg_s4.map(!_.asUInt.orR)
  val freeId_s3   = PriorityEncoder(freeVec_s3)
  val deqOpHit_s3 = io.cmAllocVec_s4.zipWithIndex.map { case(alloc, i) => alloc.fire & task_s3.code.cmid === i.U }.reduce(_ | _)
  val nid_s3      = PopCount(taskOpsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === task_s3.code.cmid))
  HardwareAssertion.withEn(freeVec_s3.reduce(_ | _), io.task_s3.valid)
  HardwareAssertion.withEn(nid_s3 > 0.U, deqOpHit_s3)

  when(io.task_s3.valid) {
    taskOpsVecReg_s4(freeId_s3).bits  := task_s3.code
    taskNidVecReg_s4(freeId_s3)       := nid_s3 - deqOpHit_s3
    cmTaskBufReg_s4(freeId_s3)        := cmTask_s3
  }

  /*
   * Select Buffer to Output
   */
  // vec
  val snpVec_s4   = taskOpsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === CMID.SNP.U)
  val readVec_s4  = taskOpsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === CMID.READ.U)
  val dlVec_s4    = taskOpsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === CMID.DL.U)
  val woaVec_s4   = taskOpsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === CMID.WOA.U)
  val zeroVec_s4  = taskNidVecReg_s4.map(_ === 0.U)
  // id
  val snpId_s4  = PriorityEncoder(snpVec_s4.zip(zeroVec_s4).map (a => a._1 & a._2))
  val readId_s4 = PriorityEncoder(readVec_s4.zip(zeroVec_s4).map(a => a._1 & a._2))
  val dlId_s4   = PriorityEncoder(dlVec_s4.zip(zeroVec_s4).map  (a => a._1 & a._2))
  val woaId_s4  = PriorityEncoder(woaVec_s4.zip(zeroVec_s4).map (a => a._1 & a._2))
  // valid
  io.cmAllocVec_s4(CMID.SNP).valid  := snpVec_s4.reduce(_ | _)
  io.cmAllocVec_s4(CMID.READ).valid := readVec_s4.reduce(_ | _)
  io.cmAllocVec_s4(CMID.DL).valid   := dlVec_s4.reduce(_ | _)
  io.cmAllocVec_s4(CMID.WOA).valid  := woaVec_s4.reduce(_ | _)
  // bits
  io.cmAllocVec_s4(CMID.SNP).bits   := cmTaskBufReg_s4(snpId_s4)
  io.cmAllocVec_s4(CMID.READ).bits  := cmTaskBufReg_s4(snpId_s4)
  io.cmAllocVec_s4(CMID.DL).bits    := cmTaskBufReg_s4(snpId_s4)
  io.cmAllocVec_s4(CMID.WOA).bits   := cmTaskBufReg_s4(snpId_s4)
  // invalid
  taskOpsVecReg_s4.zipWithIndex.foreach {
    case(ops, i) =>
      val toBeValid   = io.task_s3.valid & freeId_s3 === i.U
      val toBeInvalid = io.cmAllocVec_s4(ops.bits.cmid).ready & taskNidVecReg_s4(i) === 0.U
      ops.valid := PriorityMux(Seq(
        toBeValid   -> true.B,
        toBeInvalid -> false.B,
        true.B      -> ops.valid
      ))
      HardwareAssertion.withEn(!ops.valid, toBeValid)
      HardwareAssertion.withEn(ops.valid, toBeInvalid)
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