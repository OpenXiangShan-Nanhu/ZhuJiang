package dongjiang.backend.snoop

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
import dongjiang.bundle.ChiChannel._
import dongjiang.backend._
import dongjiang.backend.snoop.State._
import dongjiang.data.DataTask


object State {
  val width       = 5
  val FREE        = "b0000".U
  val REQDB       = "b0001".U
  val SENDSNP     = "b0010".U
  val WAITRESP    = "b0100".U
  val RESP        = "b1000".U
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> ReqDB --> SendSnp --> WaitResp0 --> WaitResp1 --> Resp --> Free
  val state       = UInt(State.width.W)
  val alrSendVec  = Vec(nrSfMetas, Bool())
  val getRespVec  = Vec(nrSfMetas, Bool())
  val getDataVec  = Vec(2, Bool())

  def isValid     = state.orR
  def isFree      = !isValid
  def isReqDB     = state(0)
  def isSendSnp   = state(1)
  def isWaitResp  = state(2)
  def isResp      = state(3)
}


class SnoopCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Commit Task In
    val alloc       = Flipped(Decoupled(new CMTask))
    // CHI
    val txSnp       = Decoupled(new SnoopFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Resp To Commit
    val respCmt     = Decoupled(new RespToCmt)
    // Req To Data
    val reqDB       = Decoupled(new PackLLCTxnID with HasDataVec)
    // Resp To Replace
    val respRepl    = Decoupled(new PackLLCTxnID() with HasChiChannel with HasChiResp)
  })


  /*
   * Reg and Wire declaration
   */
  val cmRegVec    = RegInit(VecInit(Seq.fill(nrReadCM) { 0.U.asTypeOf(new CMState) }))
  val msgRegVec   = Reg(Vec(nrReadCM, new PackCMTask with HasPackTaskInst))
  val nodeId_sSnp = Wire(new NodeId())
  val rspNodeId   = WireInit(0.U.asTypeOf(new NodeId()))
  val datNodeId   =  WireInit(0.U.asTypeOf(new NodeId()))

  /*
   * Get metaId
   */
  rspNodeId.fromLAN := NocType.rxIs(io.rxRsp.bits, LAN)
  datNodeId.fromLAN := NocType.rxIs(io.rxDat.bits, LAN)
  rspNodeId.nodeId  := io.rxRsp.bits.SrcID
  datNodeId.nodeId  := io.rxDat.bits.SrcID
  val rspFire       = io.rxRsp.fire & (io.rxRsp.bits.Opcode === SnpResp | io.rxRsp.bits.Opcode === SnpRespFwded)
  val datFire       = io.rxDat.fire & (io.rxRsp.bits.Opcode === SnpRespData | io.rxRsp.bits.Opcode === SnpRespDataFwded)
  val rspMetaId     = OHToUInt(rspNodeId.metaIdOH)
  val datMetaId     = OHToUInt(datNodeId.metaIdOH)
  HardwareAssertion.withEn(rspNodeId.metaIdOH.orR, rspFire)
  HardwareAssertion.withEn(datNodeId.metaIdOH.orR, datFire)

  /*
   * Receive Task IO
   */
  val cmVec_rec   = cmRegVec.map(!_.isValid) // Free Vec
  val cmId_rec    = PriorityEncoder(cmVec_rec)
  io.alloc.ready  := cmVec_rec.reduce(_ | _)
  HardwareAssertion.withEn(io.alloc.bits.snpVec.asUInt.orR, io.alloc.valid)

  /*
   * Req DB
   */
  val cmVec_reqDB = cmRegVec.map(_.isReqDB)
  val cmId_reqDB  = StepRREncoder(cmVec_reqDB, io.reqDB.fire)
  io.reqDB.valid  := cmVec_reqDB.reduce(_ | _)
  io.reqDB.bits.llcTxnID  := msgRegVec(cmId_reqDB).task.llcTxnID
  io.reqDB.bits.dataVec   := msgRegVec(cmId_reqDB).task.chi.dataVec

  /*
   * Send Snp
   */
  val cmVec_sSnp  = cmRegVec.map(_.isSendSnp)
  val cmId_sSnp   = StepRREncoder(cmVec_sSnp, io.txSnp.fire)
  val task_sSnp   = msgRegVec(cmId_sSnp).task
  val cm_sSnp     = cmRegVec(cmId_sSnp)
  val metaId_sSnp = OHToUInt(task_sSnp.snpVec.asUInt ^ cm_sSnp.alrSendVec.asUInt)
  val snpIsFst    = cm_sSnp.alrSendVec.asUInt === 0.U
  nodeId_sSnp.setSnpNodeId(metaId_sSnp)
  // valid
  io.txSnp.valid  := cmVec_sSnp.reduce(_ | _)
  // bits
  io.txSnp.bits             := DontCare
  io.txSnp.bits.RetToSrc    := Mux(snpIsFst, task_sSnp.chi.retToSrc, false.B)
  io.txSnp.bits.DoNotGoToSD := false.B
  io.txSnp.bits.Addr        := DontCare // remap in chi xbar
  io.txSnp.bits.Opcode      := Mux(snpIsFst, task_sSnp.chi.opcode, task_sSnp.chi.getNoFwdSnpOp)
  io.txSnp.bits.FwdTxnID    := task_sSnp.chi.txnID
  io.txSnp.bits.FwdNID      := task_sSnp.chi.nodeId
  io.txSnp.bits.TxnID       := task_sSnp.llcTxnID.get
  io.txSnp.bits.SrcID       := Mux(nodeId_sSnp.fromLAN, LAN.U, BBN.U)
  io.txSnp.bits.TgtID       := nodeId_sSnp.nodeId
  

  /*
   * Send Resp To Commit
   */
  val cmVec_resp  = cmRegVec.map(_.isResp)
  val cmId_resp   = StepRREncoder(cmVec_resp, io.respCmt.fire)
  val msg_resp    = msgRegVec(cmId_resp)
  // valid
  io.respCmt.valid  := cmVec_resp.reduce(_ | _) & !msg_resp.task.fromRepl
  io.respRepl.valid := cmVec_resp.reduce(_ | _) & msg_resp.task.fromRepl
  // bits respCmt
  io.respCmt.bits.llcTxnID    := msg_resp.task.llcTxnID
  io.respCmt.bits.inst        := msg_resp.inst
  io.respCmt.bits.inst.valid  := true.B
  io.respCmt.bits.alr         := msg_resp.task.alr
  // bits respCmt
  io.respRepl.bits.channel    := msg_resp.inst.channel
  io.respRepl.bits.llcTxnID   := msg_resp.task.llcTxnID
  io.respRepl.bits.resp       := msg_resp.inst.resp


  /*
   * Modify Ctrl Machine Table
   */
  cmRegVec.zip(msgRegVec).zipWithIndex.foreach {
    case ((cm, msg), i) =>
      val allocHit    = io.alloc.fire   & i.U === cmId_rec
      val reqDBHit    = io.reqDB.fire   & i.U === cmId_reqDB
      val sendSnpHit  = io.txSnp.fire   & i.U === cmId_sSnp
      val recRespHit  = rspFire         & msg.task.llcTxnID.get === io.rxRsp.bits.TxnID
      val recDataHit  = datFire         & msg.task.llcTxnID.get === io.rxDat.bits.TxnID
      val respCmtHit  = io.respCmt.fire & i.U === cmId_resp

      // Store Msg From Frontend
      val rspIsFwd = io.rxRsp.bits.Opcode === SnpRespFwded & recRespHit
      val datIsFwd = io.rxDat.bits.Opcode === SnpRespData & recDataHit
      when(allocHit) {
        msg.task := io.alloc.bits
        msg.inst := 0.U.asTypeOf(new TaskInst)
      // Store AlrReqDB
      }.elsewhen(reqDBHit) {
        msg.task.alr.reqs := true.B
      // Store Message
      }.elsewhen(recDataHit | recRespHit) {
        // inst
        msg.inst.valid    := true.B
        msg.inst.fwdValid := msg.inst.fwdValid | rspIsFwd | datIsFwd
        msg.inst.fwdResp  := PriorityMux(Seq(
          rspIsFwd -> io.rxRsp.bits.FwdState,
          datIsFwd -> io.rxDat.bits.FwdState,
          true.B   -> msg.inst.fwdResp,
        ))
        msg.inst.channel := PriorityMux(Seq(
          recDataHit -> DAT,
          recRespHit -> Mux(msg.inst.channel === DAT, DAT, RSP),
          true.B     -> msg.inst.channel,
        ))
        msg.inst.opcode := PriorityMux(Seq(
          recDataHit -> io.rxDat.bits.Opcode,
          recRespHit -> Mux(msg.inst.channel === DAT, msg.inst.opcode, io.rxRsp.bits.Opcode),
          true.B     -> msg.inst.opcode,
        ))
        msg.inst.resp := PriorityMux(Seq(
          recDataHit -> io.rxDat.bits.Resp,
          recRespHit -> Mux(msg.inst.channel === DAT, msg.inst.resp, io.rxRsp.bits.Resp),
          true.B     -> msg.inst.resp,
        ))
      // Release
      }.elsewhen(respCmtHit) {
        msg := 0.U.asTypeOf(msg)
      }

      // Update
      when(respCmtHit) {
        cm.alrSendVec := 0.U.asTypeOf(cm.alrSendVec)
        cm.getRespVec := 0.U.asTypeOf(cm.getRespVec)
        cm.getDataVec := 0.U.asTypeOf(cm.getDataVec)
      }.otherwise {
        // alrSendVec
        cm.alrSendVec(metaId_sSnp) := cm.alrSendVec(metaId_sSnp) & !sendSnpHit
        // getDataVec
        val beatId = Mux(io.rxDat.bits.DataID === "b10".U, 1.U, 0.U)
        cm.getDataVec(beatId) := cm.getDataVec(beatId) & !recDataHit
        // getRespVec
        cm.getRespVec.zipWithIndex.foreach { case(get, i) =>
          val rspHit = recRespHit & rspMetaId === i.U
          val datHit = recDataHit & datMetaId === i.U
          get := get | rspHit | datHit
          HardwareAssertion(PopCount(Seq(rspHit, datHit)) <= 1.U)
          HardwareAssertion.withEn(!rspHit, get)
        }
      }


      // Get Next State
      val alrSnpAll = PopCount(msg.task.snpVec.asUInt ^ cm.alrSendVec.asUInt) === 1.U
      val alrGetAll = PopCount(msg.task.snpVec.asUInt ^ cm.getRespVec.asUInt) === 0.U & (cm.getDataVec.asUInt === msg.task.chi.dataVec.asUInt | !msg.task.chi.retToSrc) // TODO
      cm.state := PriorityMux(Seq(
        allocHit   -> Mux(io.alloc.bits.needReqDB, REQDB, SENDSNP),
        reqDBHit   -> SENDSNP,
        sendSnpHit -> Mux(alrSnpAll, WAITRESP, SENDSNP),
        recRespHit -> Mux(alrGetAll, RESP, WAITRESP),
        recDataHit -> Mux(alrGetAll, RESP, WAITRESP),
        respCmtHit -> FREE,
        true.B     -> cm.state,
      ))

      HardwareAssertion.withEn(cm.isFree,       allocHit)
      HardwareAssertion.withEn(cm.isReqDB,      reqDBHit)
      HardwareAssertion.withEn(cm.isSendSnp,    sendSnpHit)
      HardwareAssertion.withEn(cm.isWaitResp,   recRespHit)
      HardwareAssertion.withEn(cm.isWaitResp,   recDataHit)
      HardwareAssertion.withEn(cm.isResp,       respCmtHit)
      HardwareAssertion.checkTimeout(cm.isFree, TIMEOUT_SNP, cf"TIMEOUT: Snoop Index[${i}]")
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}