package dongjiang.frontend

import chisel3._
import chisel3.util._
import chisel3.experimental.BundleLiterals._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang._
import dongjiang.backend.{CMTask, CommitTask}
import dongjiang.backend.RespComp
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.directory._
import xs.utils.debug._
import dongjiang.frontend.decode._
import dongjiang.data._
import xs.utils.ParallelLookUp

class Decode(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config      = new DJConfigIO()
    // From Block
    val task_s2     = Flipped(Valid(new PackChi with HasAddr with HasPackHnIdx with HasAlready)) // TODO: no need addr
    // From Directory
    val respDir_s3  = Flipped(Valid(new PackDirMsg))
    // To Backend
    val cmtTask_s3  = Valid(new CommitTask with HasHnTxnID)
    val cmTask_s3   = Valid(new CMTask with HasPackOperations)
    val respComp_s3 = Valid(new RespComp)
    // To DataBlock
    val fastData_s3 = Decoupled(new DataTask)
  })
  dontTouch(io)

  /*
   * Reg and Wire declaration
   */
  val chiInst_s2          = Wire(new ChiInst())
  val validReg_s3         = RegNext(io.task_s2.valid)
  val taskReg_s3          = RegEnable(io.task_s2.bits, io.task_s2.valid)
  val stateInst_s3        = Wire(new StateInst())

  /*
   * [S2]: Pre-Decode
   */
  chiInst_s2              := io.task_s2.bits.chi.getChiInst(io.task_s2.valid)
  val result_s2           = Decode.decode(chiInst_s2)
  val stateInstVecReg_s3  = RegEnable(VecInit(result_s2._1), io.task_s2.valid)
  val taskCodeVecReg_s3   = RegEnable(VecInit(result_s2._2), io.task_s2.valid)
  val commitVecReg_s3     = RegEnable(VecInit(result_s2._3), io.task_s2.valid)
  dontTouch(chiInst_s2)

  /*
   * [S3]: Decode
   */
  val dirValid_s3 = io.respDir_s3.valid
  stateInst_s3    := Mux(dirValid_s3, io.respDir_s3.bits.dir.getStateInst(taskReg_s3.chi.metaIdOH, validReg_s3), (new StateInst).Lit(_.valid -> true.B))
  dontTouch(stateInst_s3)
  HardwareAssertion.withEn(io.respDir_s3.valid, validReg_s3 & taskReg_s3.chi.memAttr.cacheable)

  /*
   * [S3]: Decode
   */
  val taskCode_s3 = ParallelLookUp(stateInst_s3.asUInt, stateInstVecReg_s3.map(_.asUInt).zip(taskCodeVecReg_s3))
  val cmtCode_s3  = ParallelLookUp(stateInst_s3.asUInt, stateInstVecReg_s3.map(_.asUInt).zip(commitVecReg_s3))
  dontTouch(taskCode_s3)
  dontTouch(cmtCode_s3)
  var stateInstVecCf = cf""
  stateInstVecReg_s3.zipWithIndex.foreach { case (inst, i) => stateInstVecCf = stateInstVecCf + cf"[$i] -> [$inst]\n" }
  HardwareAssertion.withEn(PopCount(Cat(stateInstVecReg_s3.map(_.asUInt === stateInst_s3.asUInt))) === 1.U, validReg_s3,
    cf"\n\nDECODE [StateInst] ERROR\n\nChiInst:\n${taskReg_s3.chi.getChiInst()}\nStateInst Invalid:\n$stateInst_s3\n\nAll Legal StateInsts:\n" + stateInstVecCf + cf"\n")
  HardwareAssertion.withEn(taskCode_s3.valid | cmtCode_s3.valid, validReg_s3)

  /*
   * [S3]: Output Commit Task
   */
  io.cmtTask_s3.valid             := validReg_s3
  io.cmtTask_s3.bits.hnTxnID      := taskReg_s3.hnIdx.getTxnID
  io.cmtTask_s3.bits.chi          := taskReg_s3.chi
  io.cmtTask_s3.bits.chi.dataVec  := Mux(taskCode_s3.snoop, DataVec.Full, taskReg_s3.chi.dataVec)
  io.cmtTask_s3.bits.dir          := Mux(dirValid_s3, io.respDir_s3.bits.dir, 0.U.asTypeOf(io.respDir_s3.bits.dir))
  io.cmtTask_s3.bits.alr.reqs     := io.fastData_s3.fire & io.fastData_s3.bits.dataOp.reqs | taskReg_s3.alr.reqs
  io.cmtTask_s3.bits.alr.sData    := io.fastData_s3.fire & io.fastData_s3.bits.dataOp.send
  io.cmtTask_s3.bits.taskCode     := taskCode_s3
  io.cmtTask_s3.bits.taskInst     := DontCare
  io.cmtTask_s3.bits.commit       := Mux(taskCode_s3.valid, 0.U.asTypeOf(new CommitCode), cmtCode_s3)
  io.cmtTask_s3.bits.snpTgt       := taskCode_s3.snpTgt
  io.cmtTask_s3.bits.ds.set(taskReg_s3.addr, io.respDir_s3.bits.dir.llc.way)
  HardwareAssertion.withEn(taskCode_s3.valid, validReg_s3 & cmtCode_s3.invalid)

  /*
   * [S3]: Send CM Task to Issue
   */
  io.cmTask_s3.valid                := validReg_s3 & taskCode_s3.opsValid
  // chi
  io.cmTask_s3.bits.chi             := taskReg_s3.chi
  // set by taskCode_s3
  io.cmTask_s3.bits.chi.dataVec     := Mux(taskCode_s3.snoop, DataVec.Full, taskReg_s3.chi.dataVec)
  io.cmTask_s3.bits.chi.opcode      := taskCode_s3.opcode
  io.cmTask_s3.bits.chi.expCompAck  := taskCode_s3.expCompAck
  io.cmTask_s3.bits.chi.retToSrc    := taskCode_s3.retToSrc
  io.cmTask_s3.bits.ops             := taskCode_s3
  io.cmTask_s3.bits.dataOp          := taskCode_s3.dataOp
  io.cmTask_s3.bits.snpVec          := io.respDir_s3.bits.dir.getSnpVec(taskCode_s3.snpTgt, taskReg_s3.chi.metaIdOH)
  io.cmTask_s3.bits.doDMT           := taskCode_s3.doDMT
  // other
  io.cmTask_s3.bits.hnTxnID         := taskReg_s3.hnIdx.getTxnID
  io.cmTask_s3.bits.alr             := taskReg_s3.alr // Be sure not to trigger taskCM when you need to send fastData
  io.cmTask_s3.bits.fromRepl        := false.B
  io.cmTask_s3.bits.cbResp          := io.respDir_s3.bits.dir.llc.metaVec.head.cbResp
  io.cmTask_s3.bits.ds.set(taskReg_s3.addr, io.respDir_s3.bits.dir.llc.way)

  /*
   * [S3]: Send RespComp to ReceiveCM for WriteEvictOrEvict
   */
  io.respComp_s3.valid              := validReg_s3 & taskReg_s3.chi.reqIs(WriteEvictOrEvict)
  io.respComp_s3.bits.hnTxnID       := taskReg_s3.hnIdx.getTxnID
  io.respComp_s3.bits.comp          := stateInst_s3.othHit | stateInst_s3.llcState =/= ChiState.I

  // fastData
  val respCompData_s3 = cmtCode_s3.sendResp & cmtCode_s3.channel === ChiChannel.DAT & cmtCode_s3.commitOp === CompData // TODO: SnpRespData
  val cleanUnuseDB_s3 = taskReg_s3.chi.isHalfSize & !taskCode_s3.snoop
  HAssert.withEn(!(respCompData_s3 & cleanUnuseDB_s3), validReg_s3)
  HAssert.withEn(io.respDir_s3.bits.dir.llc.hit, validReg_s3 & respCompData_s3)
  // valid and base bits
  io.fastData_s3.valid                := validReg_s3 & !taskCode_s3.valid & (respCompData_s3 | cleanUnuseDB_s3)
  io.fastData_s3.bits                 := DontCare
  io.fastData_s3.bits.hnTxnID         := taskReg_s3.hnIdx.getTxnID
  // respCompData_s3
  when(respCompData_s3) {
    io.fastData_s3.bits.txDat.DBID    := taskReg_s3.hnIdx.getTxnID
    io.fastData_s3.bits.txDat.Resp    := cmtCode_s3.resp
    io.fastData_s3.bits.txDat.Opcode  := CompData
    io.fastData_s3.bits.txDat.HomeNID := DontCare // remap in SAM
    io.fastData_s3.bits.txDat.TxnID   := taskReg_s3.chi.txnID
    io.fastData_s3.bits.txDat.SrcID   := taskReg_s3.chi.getNoC(io.config.ci)
    io.fastData_s3.bits.txDat.TgtID   := taskReg_s3.chi.nodeId
    // other bits
    io.fastData_s3.bits.dataOp.reqs   := true.B
    io.fastData_s3.bits.dataOp.read   := true.B
    io.fastData_s3.bits.dataOp.send   := true.B
    io.fastData_s3.bits.dataVec       := taskReg_s3.chi.dataVec
    io.fastData_s3.bits.ds.set(taskReg_s3.addr, io.respDir_s3.bits.dir.llc.way)
    HardwareAssertion.withEn(cmtCode_s3.dataOp.reqs & cmtCode_s3.dataOp.read & cmtCode_s3.dataOp.send & cmtCode_s3.dataOp.clean, io.fastData_s3.valid)
  }.otherwise {
    io.fastData_s3.bits.dataVec       := VecInit(taskReg_s3.chi.dataVec.map(!_))
    io.fastData_s3.bits.dataOp.clean  := cleanUnuseDB_s3
    HardwareAssertion.withEn(io.fastData_s3.ready, io.fastData_s3.valid)
  }

  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(1)
}