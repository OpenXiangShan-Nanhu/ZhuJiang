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
import dongjiang.data._

class Block(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config        = new DJConfigIO()
    // Task
    val chiTask_s0    = Flipped(Valid(new Chi with HasAddr))
    val task_s1       = Valid(new PackChi with HasAddr with HasPackPosIndex with HasAlready)
    // Read Directory
    val readDir_s1    = Decoupled(new Addr with HasPackPosIndex)
    // Message from PoS
    val posBlock_s1   = Input(Bool())
    val posIdx_s1     = Input(new PosIndex())
    // Return to TaskBuf
    val retry_s1      = Output(Bool())
    // Send Req To Data
    val reqDB_s1      = Decoupled(new PackLLCTxnID with HasDataVec)
    // Resp to Node(RN/SN): ReadReceipt, DBIDResp, CompDBIDResp
    val fastResp_s1   = Decoupled(new RespFlit())
    // Block Message(The number of resources already used)
    val alrUseBuf     = Input(UInt((issueBufBits+1).W))
  })
  dontTouch(io)

  /*
   * Reg and Wire declaration
   */
  val validReg_s1         = RegInit(false.B)
  val taskReg_s1          = Reg(new Chi with HasAddr)
  val needRespReadReg_s1  = RegInit(false.B)
  val needRespDBIDReg_s1  = RegInit(false.B)
  val needRsvdReg_s1      = RegInit(false.B)
  val block_s1            = Wire(new Bundle {
    val rsvd              = Bool()
    val pos               = Bool()
    val dir               = Bool()
    val resp              = Bool()
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
  block_s1.pos      := io.posBlock_s1
  block_s1.dir      := !io.readDir_s1.ready & taskReg_s1.memAttr.cacheable
  block_s1.resp     := !io.fastResp_s1.ready & needRespReadReg_s1
  io.retry_s1       := validReg_s1 & block_s1.all

  /*
   * Task Out
   */
  io.task_s1.valid      := validReg_s1 & !block_s1.all
  io.task_s1.bits.chi   := taskReg_s1
  io.task_s1.bits.addr  := taskReg_s1.addr
  io.task_s1.bits.pos   := io.posIdx_s1
  io.task_s1.bits.alr.reqs  := io.reqDB_s1.fire
  io.task_s1.bits.alr.sDBID := io.fastResp_s1.fire & taskReg_s1.isWrite
  io.task_s1.bits.alr.sData := false.B
  HardwareAssertion.withEn(io.task_s1.bits.alr.reqs, validReg_s1 & io.task_s1.bits.alr.sDBID)

  /*
   * Read Directory
   */
  io.readDir_s1.valid       := validReg_s1 & taskReg_s1.memAttr.cacheable & !(block_s1.rsvd | block_s1.pos | block_s1.resp)
  io.readDir_s1.bits.addr   := taskReg_s1.addr
  io.readDir_s1.bits.pos    := io.posIdx_s1

  /*
   * Resp to Node
   */
  // needRespReg_s1 is associated with dbid2Rn of Commit.
  needRespReadReg_s1          := io.chiTask_s0.bits.isRead & io.chiTask_s0.bits.isEO
  needRespDBIDReg_s1          := io.chiTask_s0.bits.needSendDBID()
  // Send Req To Data
  io.reqDB_s1.valid           := validReg_s1 & needRespDBIDReg_s1 & !block_s1.all
  io.reqDB_s1.bits.dataVec    := taskReg_s1.dataVec
  io.reqDB_s1.bits.llcTxnID.pos     := io.posIdx_s1
  io.reqDB_s1.bits.llcTxnID.dirBank := dirBank.U
  // Send Fast Resp To CHI
  io.fastResp_s1.valid        := validReg_s1 & (needRespReadReg_s1 | (needRespDBIDReg_s1 & io.reqDB_s1.ready)) & !(block_s1.rsvd | block_s1.pos | block_s1.dir)
  io.fastResp_s1.bits         := DontCare
  io.fastResp_s1.bits.SrcID   := taskReg_s1.getNoC(io.config.ci)
  io.fastResp_s1.bits.TgtID   := taskReg_s1.nodeId
  io.fastResp_s1.bits.TxnID   := taskReg_s1.txnID
  io.fastResp_s1.bits.Opcode  := PriorityMux(Seq(
    (taskReg_s1.isRead  & taskReg_s1.isEO)        -> ReadReceipt,
    (taskReg_s1.isWrite & taskReg_s1.isOWO)       -> DBIDResp,
    (taskReg_s1.isWrite & taskReg_s1.memAttr.ewa) -> CompDBIDResp
  ))
  HardwareAssertion.withEn(taskReg_s1.isOWO | taskReg_s1.memAttr.ewa, validReg_s1 && taskReg_s1.isWrite)
  io.fastResp_s1.bits.RespErr := RespErr.NormalOkay
  io.fastResp_s1.bits.DBID    := io.posIdx_s1.getLLCTxnID(dirBank)


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}