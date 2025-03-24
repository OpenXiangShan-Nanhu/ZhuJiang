package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.SnpOpcode._

class Block(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Task
    val chiTask_s0    = Flipped(Valid(new ChiTask with HasAddr))
    val task_s1       = Valid(new DJBundle {
      val chi         = new ChiTask with HasAddr
      val pos         = new PosIndex()
    })
    // Read Directory
    val readDir_s1    = Decoupled(new DJBundle with HasAddr with HasPosIndex {
      val early       = Bool() // Early access to data
    })
    // Message from PoS
    val posRetry_s1   = Input(Bool())
    val posIdx_s1     = Input(new PosIndex())
    // Return to TaskBuf
    val retry_s1      = Output(Bool())
    // Resp to Node(RN/SN): ReadReceipt, DBIDResp, CompDBIDResp
    val fastResp_s1   = Decoupled(new RespFlit())
    // Block Message(The number of resources already used)
    val alrUseBuf     = Input(UInt((issueBufBits+1).W))
  })
  dontTouch(io)

  /*
   * Reg and Wire declaration
   */
  val validReg_s1     = RegInit(false.B)
  val taskReg_s1      = Reg(new ChiTask with HasAddr)
  val needRespReg_s1  = RegInit(false.B)
  val needRsvdReg_s1  = RegInit(false.B)
  val block_s1        = Wire(new Bundle {
    val rsvd          = Bool()
    val pos           = Bool()
    val dir           = Bool()
    val resp          = Bool()
    def all = rsvd | pos | dir | resp
  })
  dontTouch(block_s1)

  /*
   * Receive Task
   */
  validReg_s1     := io.chiTask_s0.valid
  taskReg_s1      := io.chiTask_s0.bits

  /*
   * Block logic
   */
  // willUseBufNum = S1 + S2 + IssueBuf already use number
  val alrUseBuf_s0  = (validReg_s1 & !block_s1.all) + io.alrUseBuf; dontTouch(alrUseBuf_s0)
  val freeBufNum_s0 = nrIssueBuf.U - alrUseBuf_s0; dontTouch(freeBufNum_s0)
  HardwareAssertion(alrUseBuf_s0 <= nrIssueBuf.U)
  // Reserve an extra entry for the snp task
  needRsvdReg_s1    := Mux(io.chiTask_s0.bits.isSnp, freeBufNum_s0 === 0.U, freeBufNum_s0 <= 1.U)
  // block
  block_s1.rsvd     := needRsvdReg_s1
  block_s1.pos      := io.posRetry_s1
  block_s1.dir      := !io.readDir_s1.ready & taskReg_s1.memAttr.cacheable
  block_s1.resp     := needRespReg_s1 & !io.fastResp_s1.ready
  io.retry_s1       := validReg_s1 & block_s1.all

  /*
   * Task Out
   */
  io.task_s1.valid      := validReg_s1 & !block_s1.all
  io.task_s1.bits.chi   := taskReg_s1
  io.task_s1.bits.pos   := io.posIdx_s1

  /*
   * Read Directory
   */
  io.readDir_s1.valid       := validReg_s1 & taskReg_s1.memAttr.cacheable & !(block_s1.rsvd | block_s1.pos | block_s1.resp)
  io.readDir_s1.bits.addr   := taskReg_s1.addr
  io.readDir_s1.bits.pos    := io.posIdx_s1
  io.readDir_s1.bits.early  := io.task_s1.bits.chi.isRead | io.task_s1.bits.chi.isSnp & io.task_s1.bits.chi.snpIs(SnpMakeInvalid)

  /*
   * Resp to Node
   */
  // needRespReg_s1 is associated with dbid2Rn of Commit.
  needRespReg_s1              := io.chiTask_s0.bits.isAtomic | (io.chiTask_s0.bits.isWrite & !io.chiTask_s0.bits.reqIs(WriteEvictOrEvict)) | (io.chiTask_s0.bits.isRead & io.chiTask_s0.bits.isEO)
  io.fastResp_s1.valid        := validReg_s1 & needRespReg_s1 & (block_s1.rsvd | block_s1.pos | block_s1.dir)
  io.fastResp_s1.bits         := DontCare
  io.fastResp_s1.bits.TgtID   := taskReg_s1.nodeId
  io.fastResp_s1.bits.TxnID   := taskReg_s1.txnID
  io.fastResp_s1.bits.Opcode  := PriorityMux(Seq(
    (taskReg_s1.isRead  & taskReg_s1.isEO)      -> ReadReceipt,
    (taskReg_s1.isWrite & !taskReg_s1.noOrder)  -> DBIDResp,
    (taskReg_s1.isWrite)                        -> CompDBIDResp,
  ))
  io.fastResp_s1.bits.RespErr := RespErr.NormalOkay
  io.fastResp_s1.bits.DBID    := io.posIdx_s1.getLLCTxnID(dirBank)


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}