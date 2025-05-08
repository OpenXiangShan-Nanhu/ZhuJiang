package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.data._
import dongjiang.directory._
import xs.utils.debug._
import dongjiang.frontend.decode._
import dongjiang.backend._
import chisel3.experimental.BundleLiterals._

class Issue(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config        = new DJConfigIO()
    // In
    val cmTask_s3     = Flipped(Valid(new CMTask with HasPackOperations))
    // Out
    val cmTaskVec     = Vec(nrTaskCM, Decoupled(new CMTask))
    val alrUseBuf     = Output(UInt((issueBufBits+1).W))
  })
  dontTouch(io)

  /*
   * Reg declaration
   */
  val nidVecReg_s4  = RegInit(VecInit(Seq.fill(nrIssueBuf) { 0.U(issueBufBits.W) }))
  val opsVecReg_s4  = RegInit(VecInit(Seq.fill(nrIssueBuf) { Valid(new Operations()).Lit(_.valid -> false.B) }))
  val taskBufReg_s4 = Reg(Vec(nrIssueBuf, new CMTask))


  /*
   * [S3] Save cmTask
   */
  val freeVec_s3  = Cat(opsVecReg_s4.map(!_.valid).reverse)
  val freeId_s3   = PriorityEncoder(freeVec_s3)
  val deqOpHit_s3 = Cat(io.cmTaskVec.zipWithIndex.map { case(alloc, i) => alloc.fire & io.cmTask_s3.bits.ops.cmid === i.U }).orR
  val nid_s3      = PopCount(Cat(opsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === io.cmTask_s3.bits.ops.cmid)))
  HardwareAssertion.withEn(freeVec_s3.orR, io.cmTask_s3.valid)
  HardwareAssertion.withEn(nid_s3 > 0.U, deqOpHit_s3)

  when(io.cmTask_s3.valid) {
    nidVecReg_s4(freeId_s3)       := nid_s3 - deqOpHit_s3
    opsVecReg_s4(freeId_s3).bits  := io.cmTask_s3.bits.ops
    taskBufReg_s4(freeId_s3)      := io.cmTask_s3.bits
  }

  /*
   * Select Buffer to Output
   */
  // vec
  val zeroVec_s4  = nidVecReg_s4.map(_ === 0.U)
  val snpVec_s4   = opsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === CMID.SNP.U)
  val readVec_s4  = opsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === CMID.READ.U)
  val dlVec_s4    = opsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === CMID.DL.U)
  val woaVec_s4   = opsVecReg_s4.map(ops => ops.valid & ops.bits.cmid === CMID.WRI.U)
  // id
  val snpId_s4  = PriorityEncoder(snpVec_s4.zip(zeroVec_s4).map (a => a._1 & a._2))
  val readId_s4 = PriorityEncoder(readVec_s4.zip(zeroVec_s4).map(a => a._1 & a._2))
  val dlId_s4   = PriorityEncoder(dlVec_s4.zip(zeroVec_s4).map  (a => a._1 & a._2))
  val woaId_s4  = PriorityEncoder(woaVec_s4.zip(zeroVec_s4).map (a => a._1 & a._2))
  // valid
  io.cmTaskVec(CMID.SNP).valid  := snpVec_s4.reduce(_ | _)
  io.cmTaskVec(CMID.READ).valid := readVec_s4.reduce(_ | _)
  io.cmTaskVec(CMID.DL).valid   := dlVec_s4.reduce(_ | _)
  io.cmTaskVec(CMID.WRI).valid  := woaVec_s4.reduce(_ | _)
  // bits
  io.cmTaskVec(CMID.SNP).bits   := taskBufReg_s4(snpId_s4)
  io.cmTaskVec(CMID.READ).bits  := taskBufReg_s4(readId_s4)
  io.cmTaskVec(CMID.DL).bits    := taskBufReg_s4(dlId_s4)
  io.cmTaskVec(CMID.WRI).bits   := taskBufReg_s4(woaId_s4)
  // invalid
  opsVecReg_s4.zipWithIndex.foreach {
    case(ops, i) =>
      val toBeValid   = io.cmTask_s3.valid & io.cmTask_s3.bits.ops.opsValid & freeId_s3 === i.U
      val toBeInvalid = io.cmTaskVec(ops.bits.cmid).ready & nidVecReg_s4(i) === 0.U
      ops.valid := PriorityMux(Seq(
        toBeValid   -> true.B,
        toBeInvalid -> false.B,
        true.B      -> ops.valid
      ))
      HardwareAssertion.withEn(ops.bits.opsValid, ops.valid)
      HardwareAssertion.withEn(!ops.valid, toBeValid)
      HardwareAssertion.checkTimeout(!ops.valid, TIMEOUT_ISSUE, cf"TIMEOUT: Issue Index[${i.U}]")
  }

  /*
   * Output Already Use Buffer Number
   */
  val useVec_s4 = opsVecReg_s4.map(_.valid)
  io.alrUseBuf  := PopCount(useVec_s4)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}