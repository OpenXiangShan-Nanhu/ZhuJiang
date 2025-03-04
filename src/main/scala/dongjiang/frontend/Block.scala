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
    val taskIn    = Flipped(Valid(new ChiTask()))
    val taskOut   = Valid(new ChiTask with HasPosIndex with HasDCID)
    // Read Directory
    val readDir   = Decoupled(new Addr with HasDCID)
    // Req to DataBuffer
    val reqDB     = Decoupled(new DJBundle with HasLLCTxnID {
      val double  = Bool()
    })
    val respDB    = Input(new DCID())
    // Message from PoS
    val posRetry  = Input(Bool())
    val posIdx    = Input(new PosIndex())
    // Block Message(The number of resources already used)
    val useNum    = Input(UInt(retryBits.W))
    // Return to TaskBuf
    val retryOut  = Output(Bool())
    // Resp to Node(RN/SN)
    val resp2Node = Decoupled(new RespFlit())
  })
  dontTouch(io)

  HardwareAssertion(!io.taskIn.valid)

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
  })
  dontTouch(block_s1)

  /*
   * Receive Task
   */
  valid_s1        := io.taskIn.valid
  taskReg_s1      := io.taskIn.bits
  needDBReg_s1    := io.taskIn.bits.reqNeedData | io.taskIn.bits.snpNeedData
  needRespReg_s1  := io.taskIn.bits.isWrite | io.taskIn.bits.isEO

  /*
   * Block logic
   */
  // Reserve an extra entry for the task to be sent to LAN
  val useNum      = io.useNum + (valid_s1 & io.taskIn.valid)
  needRsvdReg_s1  := useNum > (nrRetryBuf.U - 1.U - io.taskIn.bits.fromBBN)
  // block
  block_s1.rsvd   := needRsvdReg_s1
  block_s1.pos    := io.posRetry
  block_s1.dir    := !io.readDir.ready
  block_s1.db     := needDBReg_s1 & !io.reqDB.ready
  block_s1.resp   := needRespReg_s1 & !io.resp2Node.ready
  val block       = block_s1.rsvd | block_s1.pos | block_s1.dir | block_s1.db | block_s1.resp
  io.retryOut     := valid_s1 & block

  /*
   * Send Req to DataBuffer
   */

  io.reqDB.valid          := valid_s1 & needDBReg_s1 & !(block_s1.rsvd | block_s1.pos | block_s1.dir | block_s1.resp)
  io.reqDB.bits.llcTxnID  := io.posIdx.getLLCTxnID(dirBank)
  io.reqDB.bits.double    := taskReg_s1.isFullSize

  /*
   * Task Out
   */
  io.taskOut.valid      := valid_s1 & !block
  io.taskOut.bits       := taskReg_s1.asUInt.asTypeOf(io.taskOut.bits)
  io.taskOut.bits.pos   := io.posIdx
  io.taskOut.bits.dcid  := io.respDB.dcid

  /*
   * Read Directory
   */
  io.readDir.valid      := valid_s1 & !(block_s1.rsvd | block_s1.pos | block_s1.db | block_s1.resp)
  io.readDir.bits.addr  := taskReg_s1.addr
  io.readDir.bits.dcid  := io.respDB.dcid

  /*
   * Resp to Node
   */
  io.resp2Node.valid        := valid_s1 & needRespReg_s1 & (block_s1.rsvd | block_s1.pos | block_s1.dir | block_s1.db)
  io.resp2Node.bits         := DontCare
  io.resp2Node.bits.TgtID   := taskReg_s1.nodeId
  io.resp2Node.bits.TxnID   := taskReg_s1.txnID
  io.resp2Node.bits.Opcode  := Mux(taskReg_s1.isEO, ReadReceipt, Mux(taskReg_s1.isOWO, DBIDResp, CompDBIDResp))
  io.resp2Node.bits.RespErr := RespErr.NormalOkay
  io.resp2Node.bits.DBID    := io.posIdx.getLLCTxnID(dirBank)


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}