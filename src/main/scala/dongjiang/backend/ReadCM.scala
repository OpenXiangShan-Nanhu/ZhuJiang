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
import dongjiang.backend._
import dongjiang.backend.read.State._

object State {
  val width       = 5
  val Free        = "b00000".U
  val ReqDB       = "b00001".U
  val SendReq     = "b00010".U
  val WaitData0   = "b00100".U
  val WaitData1   = "b01000".U
  val SendCompAck = "b10000".U
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> ReqDB --> SendReq --> WaitData0 --> WaitData1 --> SendCompAck --> Free
  //                                  ^                           ^
  // Internal:                     CantNest                    RespCmt
  val valid     = Bool()
  val state     = UInt(State.width.W)
  val cantNest  = Bool()
  val respCmt   = Bool()

  def isReqDB       = state(0)
  def isSendReq     = state(1)
  def isWaitData0   = state(2)
  def isWaitData1   = state(3)
  def isWaitData    = state(2) | state(3)
  def isSendCompAck = state(4)
}

class ReadCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Commit Task In
    val alloc       = Flipped(Decoupled(new CMTask))
    // CHI
    val txReq       = Decoupled(new ReqFlit(true))
    val txRsp       = Decoupled(new RespFlit())
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Update PoS Message
    val updPosNest  = Decoupled(new PackLLCTxnID with HasNest)
    // Resp To Commit
    val respCmt     = Decoupled(new RespToCmt)
    // Req To Data
    val reqDB       = Decoupled(new PackLLCTxnID with HasChiSize)
  })
  HardwareAssertion(!io.alloc.valid)

  /*
   * Reg and Wire declaration
   */
  val cmRegVec    = RegInit(VecInit(Seq.fill(nrReadCM) { 0.U.asTypeOf(new CMState) }))
  val msgRegVec   = Reg(Vec(nrReadCM, new CMTask with HasPackTaskInst))

  /*
   * [REC] Receive Task IO
   */
  val cmVec_rec     = cmRegVec.map(!_.valid) // Free Vec
  val cmId_rec      = PriorityEncoder(cmVec_rec)

  /*
   * ReqDB
   */
  val cmVec_reqDB       = cmRegVec.map(_.isReqDB)
  val cmId_reqDB        = StepRREncoder(cmVec_reqDB, io.reqDB.fire)
  io.reqDB.valid        := cmVec_reqDB.reduce(_ | _)
  io.reqDB.bits.llcTxnID:= msgRegVec(cmId_reqDB).llcTxnID

  /*
   * SendReq
   */
  val cmVec_sReq  = cmRegVec.map(_.isSendReq)
  val cmId_sReq   = StepRREncoder(cmVec_reqDB, io.txReq.fire)
  val msg_sReq    = msgRegVec(cmId_sReq)
  // valid
  io.txReq.valid  := cmVec_sReq.reduce(_ | _)
  // bits
  io.txReq.bits.ExpCompAck      := msg_sReq.chi.expCompAck
  io.txReq.bits.MemAttr         := msg_sReq.chi.memAttr.asUInt
  io.txReq.bits.Order           := Order.None
  io.txReq.bits.Addr            := msg_sReq.chi.order
  io.txReq.bits.Size            := msg_sReq.chi.size
  io.txReq.bits.Opcode          := msg_sReq.chi.opcode
  io.txReq.bits.ReturnTxnID.get := Mux(msg_sReq.doDMT, msg_sReq.chi.txnID,  msg_sReq.llcTxnID.get)
  io.txReq.bits.ReturnNID.get   := Mux(msg_sReq.doDMT, msg_sReq.chi.nodeId, Fill(io.txReq.bits.ReturnNID.get.getWidth, 1.U)) // If set RetrunNID max value, it will be remap in SAM
  io.txReq.bits.TxnID           := msg_sReq.llcTxnID.get
  io.txReq.bits.SrcID           := DontCare // remap in SAM
  // set tgt noc type
  NocType.setTx(io.txReq.bits, msg_sReq.chi.getNoC(io.config.ci))

  /*
   * Send CompAck
   */
  val cmVec_sAck  = cmRegVec.map(_.isSendCompAck)
  val cmId_sAck   = StepRREncoder(cmVec_sAck, io.txRsp.fire)
  val msg_sAck    = msgRegVec(cmId_sAck)
  // valid
  io.txRsp.valid  := cmVec_sAck.reduce(_ | _)
  // bits
  io.txRsp.bits         := DontCare
  io.txRsp.bits.Opcode  := CompAck
  io.txRsp.bits.TxnID   := msg_sAck.chi.txnID
  io.txRsp.bits.SrcID   := DontCare
  io.txRsp.bits.TgtID   := msg_sAck.chi.nodeId
  NocType.setTx(io.txRsp.bits, msg_sAck.chi.getNoC(io.config.ci))


  /*
   * Update PoS Message
   */
  val cmVec_nest  = cmRegVec.map(_.cantNest)
  val cmId_nest   = StepRREncoder(cmVec_nest, io.updPosNest.fire)
  val msg_nest    = msgRegVec(cmId_nest)
  // valid
  io.updPosNest.valid := cmVec_nest.reduce(_ | _)
  // bits
  io.updPosNest.bits.llcTxnID := msg_nest.llcTxnID
  io.updPosNest.bits.canNest  := false.B

  /*
   * Update PoS Message
   */
  val cmVec_resp  = cmRegVec.map(_.respCmt)
  val cmId_resp   = StepRREncoder(cmVec_resp, io.respCmt.fire)
  val msg_resp    = msgRegVec(cmId_resp)
  // valid
  io.respCmt.valid := cmVec_resp.reduce(_ | _)
  // bits
  io.respCmt.bits.llcTxnID      := msg_resp.llcTxnID
  io.respCmt.bits.inst.valid    := true.B
  io.respCmt.bits.inst.fwdValid := false.B
  io.respCmt.bits.inst.channel  := ChiChannel.DAT
  io.respCmt.bits.inst.opcode   := msg_resp.inst.opcode
  io.respCmt.bits.inst.resp     := msg_resp.inst.resp
  io.respCmt.bits.inst.fwdResp  := ChiResp.I

  /*
   * Modify Ctrl Machine Table
   */
  cmRegVec.zip(msgRegVec).zipWithIndex.foreach {
    case ((cm, msg), i) =>
      val allocHit    = io.alloc.fire      & i.U === cmId_rec
      val reqDBHit    = io.reqDB.fire      & i.U === cmId_reqDB
      val sendReqHit  = io.txReq.fire      & i.U === cmId_sReq
      val recDataHit  = io.rxDat.fire      & msg.llcTxnID.get === io.rxDat.bits.TxnID
      val sendAckHit  = io.txRsp.fire      & i.U === cmId_sAck
      val updNestHit  = io.updPosNest.fire & i.U === cmId_nest
      val respCmtHit  = io.respCmt.fire    & i.U === cmId_resp

      // Store Msg From Frontend
      when(allocHit) {
        msg      := io.alloc.bits
        msg.inst := DontCare
      // Store AlrReqDB
      }.elsewhen(reqDBHit) {
        cm.cantNest    := true.B
        msg.alrDB.reqs := true.B
      // Store Message
      }.elsewhen(recDataHit) {
        // chi
        msg.chi.nodeId := io.rxDat.bits.HomeNID
        msg.chi.txnID  := io.rxDat.bits.DBID
        // inst
        msg.inst.resp  := io.rxDat.bits.DBID
      }

      // Get Next State
      val nxtState = PriorityMux(Seq(
        allocHit    -> Mux(io.alloc.bits.needDB & !io.alloc.bits.alrDB.reqs, ReqDB, SendReq),
        reqDBHit    -> SendReq,
        sendReqHit  -> Mux(!msg.doDMT, WaitData0, Free),
        recDataHit  -> Mux(msg.chi.isNotFullSize, Mux(msg.chi.expCompAck, SendCompAck, Free), Mux(cm.isWaitData0, WaitData0, WaitData1)),
        sendAckHit  -> Free,
        true.B      -> cm.state,
      ))

      // Trans State and flag
      cm.valid    := Mux(allocHit, true.B, !(cm.state === Free & !cm.cantNest & !cm.respCmt))
      cm.state    := nxtState
      cm.cantNest := Mux(updNestHit, false.B, cm.state === SendReq & nxtState =/= SendReq) // SendReq -> Other
      cm.respCmt  := Mux(respCmtHit, false.B, (cm.state === WaitData0 | cm.state === WaitData1) & (nxtState === SendCompAck | nxtState === Free)) // WaitData -> SendCompAck/Free

      // HardwareAssertion
      HardwareAssertion.withEn(cm.state === Free, allocHit)
      HardwareAssertion.withEn(cm.isReqDB,        reqDBHit)
      HardwareAssertion.withEn(cm.isSendReq,      sendReqHit)
      HardwareAssertion.withEn(cm.isWaitData,     recDataHit)
      HardwareAssertion.withEn(cm.isSendCompAck,  sendAckHit)
      HardwareAssertion.withEn(cm.cantNest,       updNestHit)
      HardwareAssertion.withEn(cm.respCmt,        respCmtHit)
      HardwareAssertion.placePipe(Int.MaxValue-3)
  }


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}