package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import zhujiang.chi.RspOpcode._

class Block(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Task
    val task_s0       = Flipped(Valid(new ChiTask()))
    val task_s1       = Valid(new ChiTask with HasPosIndex with HasDCID)
    // Read Directory
    val readDir_s1    = Decoupled(new Addr with HasDCID)
    // Req to DataBuffer
    val reqDB_s1      = Decoupled(new DJBundle with HasLLCTxnID {
      val double      = Bool()
    })
    val respDB_s1     = Input(new DCID())
    // Message from PoS
    val posRetry_s1   = Input(Bool())
    val posIdx_s1     = Input(new PosIndex())
    // Return to TaskBuf
    val retry_s1      = Output(Bool())
    // Resp to Node(RN/SN)
    val resp2Node_s1  = Decoupled(new RespFlit())
    // Block Message(The number of resources already used)
    val useNum        = Input(UInt(retryBits.W))
  })
  dontTouch(io)

  HardwareAssertion(!io.task_s0.valid)

  /*
   * REG and Wire declaration
   */
  val valid_s1        = RegInit(false.B)
  val taskReg_s1      = Reg(new ChiTask())
  val needDBReg_s1    = RegInit(false.B)
  val needRespReg_s1  = RegInit(false.B)
  val needRsvdReg_s1  = RegInit(false.B)
  val block_s1        = Wire(new Bundle {
    val rsvd          = Bool()
    val pos           = Bool()
    val dir           = Bool()
    val db            = Bool()
    val resp          = Bool()
    def all = rsvd | pos | dir | db | resp
  })
  dontTouch(block_s1)

  /*
   * Receive Task
   */
  valid_s1        := io.task_s0.valid
  taskReg_s1      := io.task_s0.bits
  needDBReg_s1    := io.task_s0.bits.reqNeedData | io.task_s0.bits.snpNeedData
  needRespReg_s1  := io.task_s0.bits.isWrite | io.task_s0.bits.isEO

  /*
   * Block logic
   */
  // Reserve an extra entry for the task to be sent to LAN
  val useNum      = io.useNum + (valid_s1 & io.task_s0.valid)
  needRsvdReg_s1  := useNum > (nrRetryBuf.U - 1.U - io.task_s0.bits.fromBBN)
  // block
  block_s1.rsvd   := needRsvdReg_s1
  block_s1.pos    := io.posRetry_s1
  block_s1.dir    := !io.readDir_s1.ready
  block_s1.db     := needDBReg_s1 & !io.reqDB_s1.ready
  block_s1.resp   := needRespReg_s1 & !io.resp2Node_s1.ready
  io.retry_s1     := valid_s1 & block_s1.all

  /*
   * Send Req to DataBuffer
   */

  io.reqDB_s1.valid         := valid_s1 & needDBReg_s1 & !(block_s1.rsvd | block_s1.pos | block_s1.dir | block_s1.resp)
  io.reqDB_s1.bits.llcTxnID := io.posIdx_s1.getLLCTxnID(dirBank)
  io.reqDB_s1.bits.double   := taskReg_s1.isFullSize

  /*
   * Task Out
   */
  io.task_s1.valid      := valid_s1 & !block_s1.all
  io.task_s1.bits       := taskReg_s1.asUInt.asTypeOf(io.task_s1.bits)
  io.task_s1.bits.pos   := io.posIdx_s1
  io.task_s1.bits.dcid  := io.respDB_s1.dcid

  /*
   * Read Directory
   */
  io.readDir_s1.valid     := valid_s1 & !(block_s1.rsvd | block_s1.pos | block_s1.db | block_s1.resp)
  io.readDir_s1.bits.addr := taskReg_s1.addr
  io.readDir_s1.bits.dcid := io.respDB_s1.dcid

  /*
   * Resp to Node
   */
  io.resp2Node_s1.valid        := valid_s1 & needRespReg_s1 & (block_s1.rsvd | block_s1.pos | block_s1.dir | block_s1.db)
  io.resp2Node_s1.bits         := DontCare
  io.resp2Node_s1.bits.TgtID   := taskReg_s1.nodeId
  io.resp2Node_s1.bits.TxnID   := taskReg_s1.txnID
  io.resp2Node_s1.bits.Opcode  := Mux(taskReg_s1.isEO, ReadReceipt, Mux(taskReg_s1.isOWO, DBIDResp, CompDBIDResp))
  io.resp2Node_s1.bits.RespErr := RespErr.NormalOkay
  io.resp2Node_s1.bits.DBID    := io.posIdx_s1.getLLCTxnID(dirBank)


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}