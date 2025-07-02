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
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.directory._
import xs.utils.debug._
import dongjiang.frontend.decode._
import dongjiang.data._
import xs.utils.ParallelLookUp
import dongjiang.backend.GetDecRes

class Decode(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config          = new DJConfigIO()
    // From Block
    val task_s2         = Flipped(Valid(new PackChi with HasAddr with HasPackHnIdx with HasAlready with HasQoS)) // TODO: no need addr
    // From Directory
    val respDir_s3      = Flipped(Valid(new DirMsg))
    // To Backend
    val cmtTask_s3      = Valid(new CommitTask with HasHnTxnID)
    // To DataBlock
    val reqDB_s3        = Decoupled(new HnTxnID with HasDataVec)
    val fastData_s3     = Decoupled(new DataTask)
    val cleanDB_s3      = Valid(new HnTxnID with HasDataVec)

  })
  dontTouch(io)

  /*
   * Module Reg and Wire declaration
   */
  val getDecRes         = Module(new GetDecRes)
  val validReg_s3       = RegNext(io.task_s2.valid)
  val taskReg_s3        = RegEnable(io.task_s2.bits, io.task_s2.valid)
  val stateInst_s3      = Wire(new StateInst())

  /*
   * Get ChiInst and StateInst
   */
  val chiInst_s2        =  WireInit(io.task_s2.bits.chi.getChiInst)
  val respDir_s3        =  Mux(io.respDir_s3.valid, io.respDir_s3.bits, 0.U.asTypeOf(new DirMsg()))
  stateInst_s3          := Mux(io.respDir_s3.valid, io.respDir_s3.bits.getStateInst(taskReg_s3.chi.metaIdOH), (new StateInst).Lit(_.valid -> true.B))
  // valid
  chiInst_s2.valid      := io.task_s2.valid
  stateInst_s3.valid    := validReg_s3
  // dontTouch
  dontTouch(chiInst_s2)
  dontTouch(stateInst_s3)
  // HAssert
  HAssert.withEn( validReg_s3 & taskReg_s3.chi.memAttr.cacheable, io.respDir_s3.valid)
  HAssert.withEn( io.respDir_s3.valid, validReg_s3 &  taskReg_s3.chi.memAttr.cacheable)
  HAssert.withEn(!io.respDir_s3.valid, validReg_s3 & !taskReg_s3.chi.memAttr.cacheable)

  /*
   * Get Decode Result
   */
  val decList_s2          = Decode.firstDec(chiInst_s2)
  val decList_s3          = Decode.secondDec(RegEnable(decList_s2, io.task_s2.valid), stateInst_s3)
  val taskCode_s3         = getDecRes.io.taskCode
  val cmtCode_s3          = getDecRes.io.commitCode
  getDecRes.io.list.valid := validReg_s3
  getDecRes.io.list.bits  := decList_s3
  dontTouch(decList_s2)
  dontTouch(decList_s3)

  /*
   * Output Commit Task
   */
  io.cmtTask_s3.valid               := validReg_s3
  io.cmtTask_s3.bits.hnTxnID        := taskReg_s3.hnIdx.getTxnID
  io.cmtTask_s3.bits.qos            := taskReg_s3.qos
  io.cmtTask_s3.bits.chi            := taskReg_s3.chi
  io.cmtTask_s3.bits.chi.dataVec    := taskReg_s3.chi.dataVec
  io.cmtTask_s3.bits.dir            := respDir_s3
  io.cmtTask_s3.bits.alr.reqDB      := io.reqDB_s3.fire | taskReg_s3.alr.reqDB
  io.cmtTask_s3.bits.alr.sData      := io.fastData_s3.fire
  io.cmtTask_s3.bits.alr.sDBID      := taskReg_s3.alr.reqDB
  io.cmtTask_s3.bits.decList        := decList_s3
  io.cmtTask_s3.bits.task           := taskCode_s3
  io.cmtTask_s3.bits.cmt            := Mux(taskCode_s3.isValid, 0.U.asTypeOf(new CommitCode), cmtCode_s3)
  io.cmtTask_s3.bits.ds.set(taskReg_s3.addr, respDir_s3.llc.way)
  HardwareAssertion.withEn(taskCode_s3.isValid | cmtCode_s3.isValid, validReg_s3)
  HardwareAssertion.withEn(respDir_s3.sf.metaIsVal, validReg_s3 & taskCode_s3.snoop)

  /*
   * Request DataBuffer & Fast send CompData to CHI
   */
  val respCompData_s3               = validReg_s3 & !taskCode_s3.isValid & cmtCode_s3.sendResp & cmtCode_s3.channel === ChiChannel.DAT & cmtCode_s3.opcode === CompData // TODO: SnpRespData
  // reqDB
  io.reqDB_s3.valid                 := respCompData_s3
  io.reqDB_s3.bits.hnTxnID          := taskReg_s3.hnIdx.getTxnID
  io.reqDB_s3.bits.dataVec          := taskReg_s3.chi.dataVec
  HAssert.withEn(!taskReg_s3.alr.reqDB, io.reqDB_s3.valid)
  // CompData
  io.fastData_s3.valid              := respCompData_s3 & io.reqDB_s3.ready
  io.fastData_s3.bits               := DontCare
  io.fastData_s3.bits.hnTxnID       := taskReg_s3.hnIdx.getTxnID
  // CompData.txDat
  io.fastData_s3.bits.txDat.DBID    := taskReg_s3.hnIdx.getTxnID
  io.fastData_s3.bits.txDat.Resp    := cmtCode_s3.resp
  io.fastData_s3.bits.txDat.Opcode  := CompData
  io.fastData_s3.bits.txDat.HomeNID := DontCare // remap in SAM
  io.fastData_s3.bits.txDat.TxnID   := taskReg_s3.chi.txnID
  io.fastData_s3.bits.txDat.SrcID   := taskReg_s3.chi.getNoC
  io.fastData_s3.bits.txDat.TgtID   := taskReg_s3.chi.nodeId
  // CompData.other bits
  io.fastData_s3.bits.dataOp        := 0.U.asTypeOf(new DataOp)
  io.fastData_s3.bits.dataOp.read   := true.B
  io.fastData_s3.bits.dataOp.send   := true.B
  io.fastData_s3.bits.dataVec       := taskReg_s3.chi.dataVec
  io.fastData_s3.bits.ds.set(taskReg_s3.addr, respDir_s3.llc.way)
  HardwareAssertion.withEn(cmtCode_s3.dataOp.isValid, io.fastData_s3.valid)


  /*
   * Clean unues DataBuffer
   */
  val cleanUnuseDB_s3               = validReg_s3 & taskReg_s3.alr.reqDB & !taskReg_s3.chi.isFullSize & !(respDir_s3.sf.hit | respDir_s3.llc.hit)
  io.cleanDB_s3.valid               := cleanUnuseDB_s3
  io.cleanDB_s3.bits.hnTxnID        := taskReg_s3.hnIdx.getTxnID
  io.cleanDB_s3.bits.dataVec        := VecInit(taskReg_s3.chi.dataVec.map(!_))
  // HAssert
  HAssert(!(respCompData_s3 & cleanUnuseDB_s3))
  HAssert.withEn(io.cleanDB_s3.bits.isHalfSize, io.cleanDB_s3.valid)

  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(1)
}