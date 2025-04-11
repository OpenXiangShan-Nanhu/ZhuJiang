package dongjiang.backend.read

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg}
import dongjiang.frontend._
import dongjiang.frontend.decode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang.backend._
import dongjiang.backend.read.State._

object State {
  val width       = 6
  val FREE        = "b000000".U
  val REQDB       = "b000001".U
  val SENDREQ     = "b000010".U
  val WAITDATA0   = "b000100".U
  val WAITDATA1   = "b001000".U
  val RESPCMT     = "b010000".U
  val SENDACK     = "b100000".U
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> ReqDB --> SendReq --> WaitData0 --> WaitData1 -->  RespCmt--> SendCompAck --> Free
  val state         = UInt(State.width.W)

  def isValid       = state.orR
  def isInvalid     = !isValid
  def isReqDB       = state(0)
  def isSendReq     = state(1)
  def isWaitData0   = state(2)
  def isWaitData1   = state(3)
  def isWaitData    = state(2) | state(3)
  def isRespCmt     = state(4)
  def isSendCompAck = state(5)
}

class ReadCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config        = new DJConfigIO()
    // Get Full Addr In PoS
    val getAddr       = new GetAddr()
    // Commit Task In
    val alloc         = Flipped(Decoupled(new CMTask))
    // CHI
    val txReq         = Decoupled(new ReqFlit(true))
    val txRsp         = Decoupled(new RespFlit())
    val rxDat         = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Resp To Commit
    val respCmt       = Decoupled(new RespToCmt)
    // Req To Data
    val reqDB         = Decoupled(new PackLLCTxnID with HasDataVec)
  })

  /*
   * Reg and Wire declaration
   */
  val cmRegVec    = RegInit(VecInit(Seq.fill(nrReadCM) { 0.U.asTypeOf(new CMState) }))
  val msgRegVec   = Reg(Vec(nrReadCM, new PackCMTask with HasPackTaskInst))

  /*
   * Receive Task IO
   */
  val cmVec_rec     = cmRegVec.map(_.isInvalid) // Free Vec
  val cmId_rec      = PriorityEncoder(cmVec_rec)
  io.alloc.ready    := cmVec_rec.reduce(_ | _)

  /*
   * ReqDB
   */
  val cmVec_reqDB       = cmRegVec.map(_.isReqDB)
  val cmId_reqDB        = StepRREncoder(cmVec_reqDB, io.reqDB.fire)
  io.reqDB.valid        := cmVec_reqDB.reduce(_ | _)
  io.reqDB.bits.llcTxnID:= msgRegVec(cmId_reqDB).task.llcTxnID
  io.reqDB.bits.dataVec := msgRegVec(cmId_reqDB).task.chi.dataVec

  /*
   * SendReq
   */
  val cmVec_sReq  = cmRegVec.map(_.isSendReq)
  val cmId_sReq   = StepRREncoder(cmVec_sReq, io.txReq.fire)
  val task_sReq   = msgRegVec(cmId_sReq).task
  // valid
  io.txReq.valid  := cmVec_sReq.reduce(_ | _)
  // get addr
  io.getAddr.llcTxnID           := task_sReq.llcTxnID
  // bits
  io.txReq.bits                 := DontCare
  io.txReq.bits.ExpCompAck      := task_sReq.chi.expCompAck
  io.txReq.bits.MemAttr         := task_sReq.chi.memAttr.asUInt
  io.txReq.bits.Order           := Order.None
  io.txReq.bits.Addr            := io.getAddr.result.addr
  io.txReq.bits.Size            := task_sReq.chi.getSize
  io.txReq.bits.Opcode          := task_sReq.chi.opcode
  io.txReq.bits.ReturnTxnID.get := Mux(task_sReq.doDMT, task_sReq.chi.txnID,  task_sReq.llcTxnID.get)
  io.txReq.bits.ReturnNID.get   := Mux(task_sReq.doDMT, task_sReq.chi.nodeId, Fill(io.txReq.bits.ReturnNID.get.getWidth, 1.U)) // If set RetrunNID max value, it will be remap in SAM
  io.txReq.bits.TxnID           := task_sReq.llcTxnID.get
  io.txReq.bits.SrcID           := task_sReq.chi.getNoC(io.config.ci)

  /*
   * Send CompAck
   */
  val cmVec_sAck  = cmRegVec.map(_.isSendCompAck)
  val cmId_sAck   = StepRREncoder(cmVec_sAck, io.txRsp.fire)
  val task_sAck   = msgRegVec(cmId_sAck).task
  // valid
  io.txRsp.valid  := cmVec_sAck.reduce(_ | _)
  // bits
  io.txRsp.bits         := DontCare
  io.txRsp.bits.Opcode  := CompAck
  io.txRsp.bits.TxnID   := task_sAck.chi.txnID
  io.txRsp.bits.SrcID   := task_sAck.chi.getNoC(io.config.ci)
  io.txRsp.bits.TgtID   := task_sAck.chi.nodeId


  /*
   * Send Resp To Commit
   */
  val cmVec_resp  = cmRegVec.map(_.isRespCmt)
  val cmId_resp   = StepRREncoder(cmVec_resp, io.respCmt.fire)
  val msg_resp    = msgRegVec(cmId_resp)
  // valid
  io.respCmt.valid := cmVec_resp.reduce(_ | _)
  // bits
  io.respCmt.bits.llcTxnID  := msg_resp.task.llcTxnID
  io.respCmt.bits.alr       := msg_resp.task.alr
  when(msg_resp.task.doDMT) {
    io.respCmt.bits.inst          := 0.U.asTypeOf(new TaskInst)
  }.otherwise {
    io.respCmt.bits.inst.valid    := true.B
    io.respCmt.bits.inst.fwdValid := false.B
    io.respCmt.bits.inst.channel  := ChiChannel.DAT
    io.respCmt.bits.inst.opcode   := msg_resp.inst.opcode
    io.respCmt.bits.inst.resp     := msg_resp.inst.resp
    io.respCmt.bits.inst.fwdResp  := ChiResp.I
  }

  /*
   * Modify Ctrl Machine Table
   */
  cmRegVec.zip(msgRegVec).zipWithIndex.foreach {
    case ((cm, msg), i) =>
      val allocHit    = io.alloc.fire   & i.U === cmId_rec
      val reqDBHit    = io.reqDB.fire   & i.U === cmId_reqDB
      val sendReqHit  = io.txReq.fire   & i.U === cmId_sReq
      val recDataHit  = io.rxDat.fire   & msg.task.llcTxnID.get === io.rxDat.bits.TxnID & io.rxDat.bits.Opcode === CompData
      val sendAckHit  = io.txRsp.fire   & i.U === cmId_sAck
      val respCmtHit  = io.respCmt.fire & i.U === cmId_resp

      // Store Msg From Frontend
      when(allocHit) {
        msg.task := io.alloc.bits
        msg.inst := 0.U.asTypeOf(new TaskInst)
      // store already reqs
      }.elsewhen(reqDBHit) {
        msg.task.alr.reqs := true.B
      // Store Message
      }.elsewhen(recDataHit) {
        // chi
        msg.task.chi.nodeId := io.rxDat.bits.HomeNID
        msg.task.chi.txnID  := io.rxDat.bits.DBID
        // inst
        msg.inst.resp       := io.rxDat.bits.DBID
      }

      // Get Next State
      cm.state := PriorityMux(Seq(
        allocHit    -> Mux(io.alloc.bits.needReqDB, REQDB, SENDREQ),
        reqDBHit    -> SENDREQ,
        sendReqHit  -> Mux(!msg.task.doDMT, WAITDATA0, RESPCMT),
        recDataHit  -> Mux(msg.task.chi.isHalfSize, RESPCMT, WAITDATA1),
        respCmtHit  -> Mux(msg.task.chi.expCompAck, SENDACK, FREE),
        sendAckHit  -> FREE,
        true.B      -> cm.state,
      ))

      // HardwareAssertion
      HardwareAssertion.withEn(cm.isInvalid,      allocHit)
      HardwareAssertion.withEn(cm.isReqDB,        reqDBHit)
      HardwareAssertion.withEn(cm.isSendReq,      sendReqHit)
      HardwareAssertion.withEn(cm.isWaitData,     recDataHit)
      HardwareAssertion.withEn(cm.isSendCompAck,  sendAckHit)
      HardwareAssertion.withEn(cm.isRespCmt,      respCmtHit)
      HardwareAssertion.checkTimeout(cm.isInvalid, TIMEOUT_READ, cf"TIMEOUT: Read Index[${i}]")
  }


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}