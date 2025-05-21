package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.{CMTask, FastResp}
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.SnpOpcode._
import dongjiang.data._

class Block(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config        = new DJConfigIO()
    // Task
    val chiTask_s0    = Flipped(Valid(new PackChi with HasAddr))
    val task_s1       = Valid(new PackChi with HasAddr with HasPackHnIdx with HasAlready)
    // Read Directory
    val readDir_s1    = Decoupled(new Addr with HasPackHnIdx)
    // Message from PoS
    val posBlock_s1   = Input(Bool())
    val hnIdx_s1      = Input(new HnIndex())
    // Return to TaskBuf
    val retry_s1      = Output(Bool())
    // Send Req To Data
    val reqDB_s1      = Decoupled(new HnTxnID with HasDataVec)
    // Resp to RN: ReadReceipt, DBIDResp, CompDBIDResp
    val fastResp_s1   = Decoupled(new FastResp())
  })
  dontTouch(io)

  /*
   * Reg and Wire declaration
   */
  val validReg_s1         = RegInit(false.B)
  val taskReg_s1          = Reg(new PackChi with HasAddr)
  val sReceiptReg_s1      = RegInit(false.B)  // need send ReadReceipt in s1
  val sDBIDReg_s1         = RegInit(false.B)  // need send XDBIDResp in s1
  val reqIsWEOEReg_s1     = RegInit(false.B)  // Req is WriteEvictOrEvict
  val shouldResp_s1       = Wire(Bool())      // shouled send fast resp to Backend
  val blockByDB_s1        = Wire(Bool())      // block by no DataBuffer
  val block_s1            = Wire(new Bundle {
    val pos               = Bool()
    val dir               = Bool()
    val resp              = Bool()
    def all = pos | dir | resp
  })
  dontTouch(block_s1)

  /*
   * Receive Task
   */
  validReg_s1     := io.chiTask_s0.valid
  when(io.chiTask_s0.valid) {
    taskReg_s1    := io.chiTask_s0.bits
  }

  /*
   * Block logic
   */
  block_s1.pos      := io.posBlock_s1
  block_s1.dir      := taskReg_s1.chi.memAttr.cacheable & !io.readDir_s1.ready
  block_s1.resp     := blockByDB_s1 | (shouldResp_s1 & !io.fastResp_s1.ready)
  io.retry_s1       := validReg_s1 & block_s1.all

  /*
   * Task Out
   */
  io.task_s1.valid            := validReg_s1 & !block_s1.all
  io.task_s1.bits.chi         := taskReg_s1.chi
  io.task_s1.bits.addr        := taskReg_s1.addr
  io.task_s1.bits.hnIdx       := io.hnIdx_s1
  io.task_s1.bits.alr.reqs    := io.reqDB_s1.fire
  io.task_s1.bits.alr.sData   := false.B
  io.task_s1.bits.alr.cleanDB := false.B

  /*
   * Read Directory
   */
  io.readDir_s1.valid         := validReg_s1 & taskReg_s1.chi.memAttr.cacheable & !(block_s1.pos | block_s1.resp)
  io.readDir_s1.bits.addr     := taskReg_s1.addr
  io.readDir_s1.bits.hnIdx    := io.hnIdx_s1

  /*
   * Resp to Node
   */
  // needRespReg_s1 is associated with dbid2Rn of Commit.
  sReceiptReg_s1                    := io.chiTask_s0.bits.chi.isRead  & io.chiTask_s0.bits.chi.isEO
  sDBIDReg_s1                       := io.chiTask_s0.bits.chi.needSendDBID
  reqIsWEOEReg_s1                   := io.chiTask_s0.bits.chi.reqIs(WriteEvictOrEvict)
  shouldResp_s1                     := sReceiptReg_s1 | reqIsWEOEReg_s1 | (sDBIDReg_s1 & io.reqDB_s1.ready)
  blockByDB_s1                      := sDBIDReg_s1 & !io.reqDB_s1.ready
  // Send Req To Data
  io.reqDB_s1.valid                 := validReg_s1 & sDBIDReg_s1 & io.fastResp_s1.ready & !(block_s1.pos | block_s1.dir)
  io.reqDB_s1.bits.dataVec          := DataVec.Full
  io.reqDB_s1.bits.hnTxnID          := io.hnIdx_s1.getTxnID
  // Send Fast Resp To CHI
  io.fastResp_s1.valid              := validReg_s1 & shouldResp_s1 & !(block_s1.pos | block_s1.dir)
  io.fastResp_s1.bits.dataVec       := taskReg_s1.chi.dataVec
  io.fastResp_s1.bits.rsp           := DontCare
  io.fastResp_s1.bits.rsp.SrcID     := taskReg_s1.chi.getNoC
  io.fastResp_s1.bits.rsp.TgtID     := taskReg_s1.chi.nodeId
  io.fastResp_s1.bits.rsp.TxnID     := taskReg_s1.chi.txnID
  io.fastResp_s1.bits.rsp.DBID      := io.hnIdx_s1.getTxnID
  io.fastResp_s1.bits.rsp.RespErr   := RespErr.NormalOkay
  io.fastResp_s1.bits.rsp.Opcode    := PriorityMux(Seq(
    sReceiptReg_s1                              -> ReadReceipt,
    reqIsWEOEReg_s1                             -> Comp,
    (sDBIDReg_s1 & taskReg_s1.chi.isOWO)        -> DBIDResp,
    (sDBIDReg_s1 & taskReg_s1.chi.memAttr.ewa)  -> CompDBIDResp
  ))
  HardwareAssertion.withEn(taskReg_s1.chi.isOWO | taskReg_s1.chi.memAttr.ewa, validReg_s1 && taskReg_s1.chi.isWrite)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}