package dongjiang.backend.receive

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, HasPackDirMsg}
import dongjiang.frontend._
import dongjiang.frontend.decode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang.backend._
import dongjiang.backend.receive.State._
import dongjiang.data._

object State {
  val width   = 5
  val FREE    = "b00000".U // 0x0
  val REQDB   = "b00001".U // 0x1
  val SEND    = "b00010".U // 0x2
  val WAIT0   = "b00100".U // 0x4
  val WAIT1   = "b01000".U // 0x8
  val RESP    = "b10000".U // 0x10
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> ReqDB --> Send --> Wait0 --> Wait1 --> RespCmt --> Free
  val state   = UInt(State.width.W)

  // TODO: There might be situations where the data has arrived, but the task gets blocked.
  def isValid = state.orR
  def isFree  = !isValid
  def isReqDB = state(0)
  def isSend  = state(1)
  def isWait0 = state(2)
  def isWait1 = state(3)
  def isResp  = state(4)
}


class ReceiveCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config        = new DJConfigIO()
    // Commit Task In
    val alloc         = Flipped(Decoupled(new CMTask))
    // ReqDB
    val reqDB         = Decoupled(new PackLLCTxnID with HasDataVec)
    // CHI
    val txRsp         = Decoupled(new RespFlit())
    val rxRsp         = Flipped(Valid(new RespFlit()))
    val rxDat         = Flipped(Valid(new DataFlit()))
    // Resp To Commit
    val respCmt       = Decoupled(new RespToCmt)
  })

  /*
   * Reg and Wire declaration
   */
  val cmRegVec = RegInit(VecInit(Seq.fill(nrReadCM) { 0.U.asTypeOf(new CMState) }))
  val msgRegVec = Reg(Vec(nrReadCM, new PackCMTask with HasPackTaskInst))

  /*
   * [Free] Receive Task IO
   */
  val cmVec_rec   = cmRegVec.map(_.isFree) // Free Vec
  val cmId_rec    = PriorityEncoder(cmVec_rec)
  io.alloc.ready  := cmVec_rec.reduce(_ | _)

  /*
   * ReqDB
   */
  val cmVec_reqDB         = cmRegVec.map(_.isReqDB)
  val cmId_reqDB          = StepRREncoder(cmVec_reqDB, io.reqDB.fire)
  io.reqDB.valid          := cmVec_reqDB.reduce(_ | _)
  io.reqDB.bits.llcTxnID  := msgRegVec(cmId_reqDB).task.llcTxnID
  io.reqDB.bits.dataVec   := msgRegVec(cmId_reqDB).task.chi.dataVec

  /*
   * [Send]
   */
  val cmVec_sRsp  = cmRegVec.map(_.isSend)
  val cmId_sRsp   = StepRREncoder(cmVec_sRsp, io.txRsp.fire)
  val task_sRsp   = msgRegVec(cmId_sRsp).task
  // valid
  io.txRsp.valid  := cmVec_sRsp.reduce(_ | _)
  // bits
  io.txRsp.bits         := DontCare
  io.txRsp.bits.SrcID   := task_sRsp.chi.getNoC(io.config.ci)
  io.txRsp.bits.TgtID   := task_sRsp.chi.nodeId
  io.txRsp.bits.TxnID   := task_sRsp.chi.txnID
  io.txRsp.bits.Opcode  := task_sRsp.chi.opcode
  io.txRsp.bits.RespErr := RespErr.NormalOkay
  io.txRsp.bits.DBID    := task_sRsp.llcTxnID.get


  /*
   * Resp
   */
  val cmVec_resp    = cmRegVec.map(_.isResp)
  val cmId_resp     = StepRREncoder(cmVec_resp, io.respCmt.fire)
  val task_resp     = msgRegVec(cmId_resp)
  // valid
  io.respCmt.valid  := cmVec_resp.reduce(_ | _)
  // bits
  io.respCmt.bits.llcTxnID      := task_resp.task.llcTxnID
  io.respCmt.bits.alr           := task_resp.task.alr
  io.respCmt.bits.inst.valid    := true.B
  io.respCmt.bits.inst.fwdValid := false.B
  io.respCmt.bits.inst.channel  := task_resp.inst.channel
  io.respCmt.bits.inst.opcode   := task_resp.inst.opcode
  io.respCmt.bits.inst.resp     := task_resp.inst.resp
  io.respCmt.bits.inst.fwdResp  := ChiResp.I


  /*
   * Modify Ctrl Machine Table
   */
  cmRegVec.zip(msgRegVec).zipWithIndex.foreach {
    case ((cm, msg), i) =>
      val allocHit    = io.alloc.fire   & cmId_rec  === i.U
      val reqDBHit    = io.reqDB.fire   & i.U === cmId_reqDB
      val sRspHit     = io.txRsp.fire   & cmId_sRsp === i.U
      val waitRspHit  = io.rxRsp.fire   & io.rxRsp.bits.TxnID === msg.task.llcTxnID.get &
                        io.rxRsp.bits.Opcode === CompAck & cm.isValid
      val waitDatHit  = io.rxDat.fire   & io.rxDat.bits.TxnID === msg.task.llcTxnID.get &
                        (io.rxDat.bits.Opcode === NonCopyBackWriteData | io.rxDat.bits.Opcode === CopyBackWriteData) & cm.isValid
      val respHit     = io.respCmt.fire & cmId_resp === i.U

      // Message
      when(allocHit) {
        msg.task := io.alloc.bits
        msg.inst := 0.U.asTypeOf(new TaskInst)
      // store already reqs
      }.elsewhen(reqDBHit) {
        msg.task.alr.reqs := true.B
        HardwareAssertion(!msg.task.alr.reqs)
      // Store Inst
      }.elsewhen(waitRspHit | waitDatHit) {
        msg.inst.channel := Mux(waitRspHit, ChiChannel.RSP,       ChiChannel.DAT)
        msg.inst.opcode  := Mux(waitRspHit, io.rxRsp.bits.Opcode, io.rxDat.bits.Opcode)
        msg.inst.resp    := Mux(waitRspHit, io.rxRsp.bits.Resp,   io.rxDat.bits.Resp)
      }

      // ctrl machine state
      cm.state := PriorityMux(Seq(
        allocHit    -> Mux(io.alloc.bits.alr.sDBID, WAIT0, Mux(io.alloc.bits.needReqDB, REQDB, SEND)),
        reqDBHit    -> SEND,
        sRspHit     -> WAIT0,
        waitRspHit  -> RESP,
        waitDatHit  -> Mux(msg.task.chi.isFullSize, Mux(cm.isWait1, RESP, WAIT1), RESP),
        respHit     -> FREE,
        true.B      -> cm.state
      ))

      // HardwareAssertion
      HardwareAssertion.withEn(cm.isFree,   allocHit)
      HardwareAssertion.withEn(cm.isReqDB,  reqDBHit)
      HardwareAssertion.withEn(cm.isSend,   sRspHit)
      HardwareAssertion.withEn(cm.isWait0,  waitRspHit)
      HardwareAssertion.withEn(cm.isWait0 | cm.isWait1, waitDatHit)
      HardwareAssertion.withEn(cm.isResp,   respHit)
      HardwareAssertion.withEn(msg.task.chi.rspIs(Comp), waitRspHit)
      HardwareAssertion.withEn(msg.task.chi.rspIs(CompDBIDResp, DBIDResp), waitDatHit)
      HardwareAssertion.checkTimeout(cm.isFree, TIMEOUT_REC, cf"TIMEOUT: Receive Index[${i}]")
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}