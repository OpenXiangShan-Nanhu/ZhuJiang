package dongjiang.frontend

import chisel3._
import chisel3.util._
import chisel3.experimental.BundleLiterals._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang._
import dongjiang.backend.{CMTask, CommitTask, RecRespType}
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
    val config          = new DJConfigIO()
    // From Block
    val task_s2         = Flipped(Valid(new PackChi with HasAddr with HasPackHnIdx with HasAlready)) // TODO: no need addr
    // From Directory
    val respDir_s3      = Flipped(Valid(new DirMsg))
    // To Backend
    val cmtTask_s3      = Valid(new CommitTask with HasHnTxnID)
    val recRespType_s3  = Valid(new RecRespType)
    // To DataBlock
    val fastData_s3     = Decoupled(new DataTask)
  })
  dontTouch(io)

  /*
   * Module Reg and Wire declaration
   */
  val decInit             = WireInit(0.U.asTypeOf(new PackDecList()))
  val getDecRes           = GetDecRes()
  val validReg_s3         = RegNext(io.task_s2.valid)
  val taskReg_s3          = RegEnable(io.task_s2.bits, io.task_s2.valid)
  val stateInst_s3        = Wire(new StateInst())
  val respCompData_s3     = Wire(Bool())
  val cleanUnuseDB_s3     = Wire(Bool())

  /*
   * Get ChiInst and StateInst
   */
  val chiInst_s2      = WireInit(io.task_s2.bits.chi.getChiInst)
  val respDir_s3      = Mux(io.respDir_s3.valid, io.respDir_s3.bits, 0.U.asTypeOf(new DirMsg()))
  stateInst_s3        := Mux(io.respDir_s3.valid, io.respDir_s3.bits.getStateInst(taskReg_s3.chi.metaIdOH), (new StateInst).Lit(_.valid -> true.B))
  // valid
  chiInst_s2.valid    := io.task_s2.valid
  stateInst_s3.valid  := validReg_s3
  // dontTouch
  dontTouch(chiInst_s2)
  dontTouch(stateInst_s3)
  // HAssert
  HAssert.withEn( io.respDir_s3.valid, validReg_s3 &  taskReg_s3.chi.memAttr.cacheable)
  HAssert.withEn(!io.respDir_s3.valid, validReg_s3 & !taskReg_s3.chi.memAttr.cacheable)

  /*
   * Get Decode Result
   */
  val decList_s2          = decInit.firstDec(chiInst_s2)
  val decList_s3          = RegNext(decList_s2.asTypeOf(new PackDecList)).secondDec(stateInst_s3)
  val taskCode_s3         = getDecRes.io.taskCode
  val cmtCode_s3          = getDecRes.io.commitCode
  getDecRes.io.list.valid := validReg_s3
  getDecRes.io.list.bits  := decList_s3
  dontTouch(decList_s2)
  dontTouch(decList_s3)

  /*
   * Output Commit Task
   */
  io.cmtTask_s3.valid                 := validReg_s3
  io.cmtTask_s3.bits.hnTxnID          := taskReg_s3.hnIdx.getTxnID
  io.cmtTask_s3.bits.chi              := taskReg_s3.chi
  io.cmtTask_s3.bits.chi.dataVec      := Mux(taskCode_s3.snoop, DataVec.Full, taskReg_s3.chi.dataVec)
  io.cmtTask_s3.bits.dir              := respDir_s3
  io.cmtTask_s3.bits.alr.reqs         := io.fastData_s3.fire & io.fastData_s3.bits.dataOp.reqs | taskReg_s3.alr.reqs
  io.cmtTask_s3.bits.alr.sData        := io.fastData_s3.fire & respCompData_s3
  io.cmtTask_s3.bits.alr.cleanDB      := io.fastData_s3.fire & cleanUnuseDB_s3
  io.cmtTask_s3.bits.decList          := decList_s3
  io.cmtTask_s3.bits.ds.set(taskReg_s3.addr, respDir_s3.llc.way)
  HardwareAssertion.withEn(taskCode_s3.valid | cmtCode_s3.valid, validReg_s3)

  /*
   * Send RespType to ReceiveCM for WriteEvictOrEvict
   */
  io.recRespType_s3.valid              := validReg_s3 & taskReg_s3.chi.reqIs(WriteEvictOrEvict)
  io.recRespType_s3.bits.hnTxnID       := taskReg_s3.hnIdx.getTxnID
  io.recRespType_s3.bits.dbid          := stateInst_s3.srcHit & !stateInst_s3.othHit & stateInst_s3.llcState === ChiState.I

  // fastData
  respCompData_s3                     := cmtCode_s3.sendResp & cmtCode_s3.channel === ChiChannel.DAT & cmtCode_s3.commitOp === CompData // TODO: SnpRespData
  cleanUnuseDB_s3                     := taskReg_s3.alr.reqs & taskReg_s3.chi.isHalfSize & !taskCode_s3.snoop
  HAssert.withEn(!(respCompData_s3 & cleanUnuseDB_s3), validReg_s3)
  HAssert.withEn(respDir_s3.llc.hit, validReg_s3 & respCompData_s3)
  // valid and base bits
  io.fastData_s3.valid                := validReg_s3 & ((!taskCode_s3.valid & respCompData_s3) | cleanUnuseDB_s3)
  io.fastData_s3.bits                 := DontCare
  io.fastData_s3.bits.hnTxnID         := taskReg_s3.hnIdx.getTxnID
  // respCompData_s3
  when(respCompData_s3) {
    io.fastData_s3.bits.txDat.DBID    := taskReg_s3.hnIdx.getTxnID
    io.fastData_s3.bits.txDat.Resp    := cmtCode_s3.resp
    io.fastData_s3.bits.txDat.Opcode  := CompData
    io.fastData_s3.bits.txDat.HomeNID := DontCare // remap in SAM
    io.fastData_s3.bits.txDat.TxnID   := taskReg_s3.chi.txnID
    io.fastData_s3.bits.txDat.SrcID   := taskReg_s3.chi.getNoC
    io.fastData_s3.bits.txDat.TgtID   := taskReg_s3.chi.nodeId
    // other bits
    io.fastData_s3.bits.dataOp        := 0.U.asTypeOf(new DataOp)
    io.fastData_s3.bits.dataOp.reqs   := true.B
    io.fastData_s3.bits.dataOp.read   := true.B
    io.fastData_s3.bits.dataOp.send   := true.B
    io.fastData_s3.bits.dataVec       := taskReg_s3.chi.dataVec
    io.fastData_s3.bits.ds.set(taskReg_s3.addr, respDir_s3.llc.way)
    HardwareAssertion.withEn(cmtCode_s3.dataOp.reqs & cmtCode_s3.dataOp.read & cmtCode_s3.dataOp.send & cmtCode_s3.dataOp.clean, io.fastData_s3.valid)
  // cleanUnuseDB_s3
  }.otherwise {
    io.fastData_s3.bits.dataVec       := VecInit(taskReg_s3.chi.dataVec.map(!_))
    io.fastData_s3.bits.dataOp        := 0.U.asTypeOf(new DataOp)
    io.fastData_s3.bits.dataOp.clean  := true.B
    HardwareAssertion.withEn(io.fastData_s3.ready, io.fastData_s3.valid)
  }

  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(1)
}