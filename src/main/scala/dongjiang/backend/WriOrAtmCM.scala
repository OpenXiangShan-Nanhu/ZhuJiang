package dongjiang.backend.wrioratm

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
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import dongjiang.backend._
import dongjiang.backend.wrioratm.State._
import dongjiang.data._

object State {
  val width       = 7
  val FREE        = "b0000000".U
  val SENDREQ     = "b0000001".U
  val CANNEST     = "b0000010".U
  val WAITDBID    = "b0000100".U
  val DATATASK    = "b0001000".U
  val WAITDATA    = "b0010000".U
  val WAITCOMP    = "b0100000".U
  val RESP        = "b1000000".U
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> SendReq  --> CanNest --> WaitDBID --> DataTask --> WaitData/WaitComp --> RespCmt/RespRepl --> Free
  val state       = UInt(State.width.W)

  def isValid     = state.orR
  def isFree      = !isValid
  def isSendReq   = state(0)
  def isCanNest   = state(1)
  def isWaitDBID  = state(2)
  def isDataTask  = state(3)
  def isWaitData  = state(4)
  def isWaitComp  = state(5)
  def isResp      = state(6)
}


class WriOrAtmCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config        = new DJConfigIO()
    // Commit Task In
    val alloc         = Flipped(Decoupled(new CMTask))
    // CHI
    val txReq         = Decoupled(new ReqFlit(true))
    val rxRsp         = Flipped(Valid(new RespFlit()))
    // Update PoS Message
    val updPosNestVec = Vec(djparam.nrDirBank, Decoupled(new PackPosIndex with HasNest))
    // Resp To Commit
    val respCmt       = Decoupled(new RespToCmt)
    // Resp To Replace
    val respRepl      = Decoupled(new PackLLCTxnID() with HasChiChannel with HasChiResp)
    // DataTask
    val dataTask      = Decoupled(new DataTask)
    val dataResp      = Flipped(Valid(new PackLLCTxnID()))
  })

  /*
   * Reg and Wire declaration
   */
  val cmRegVec  = RegInit(VecInit(Seq.fill(nrReadCM) { 0.U.asTypeOf(new CMState) }))
  val msgRegVec = Reg(Vec(nrReadCM, new CMTask()))

  /*
   * [Free] Receive Task IO
   */
  val cmVec_rec   = cmRegVec.map(_.isFree) // Free Vec
  val cmId_rec    = PriorityEncoder(cmVec_rec)
  io.alloc.ready  := cmVec_rec.reduce(_ | _)
  HardwareAssertion.withEn(!io.alloc.bits.dataOp.reqs, io.alloc.valid & io.alloc.bits.alr.reqs)

  /*
   * [SendReq]
   */
  val cmVec_sReq  = cmRegVec.map(_.isSendReq)
  val cmId_sReq   = StepRREncoder(cmVec_sReq, io.txReq.fire)
  val task_sReq   = msgRegVec(cmId_sReq)
  // valid
  io.txReq.valid  := cmVec_sReq.reduce(_ | _)
  // TODO: toBBN WriteBackFull / WriteEvictOrEvict
  val replOp            = WriteNoSnpFull
  // bits
  io.txReq.bits         := DontCare
  io.txReq.bits.MemAttr := task_sReq.chi.memAttr.asUInt
  io.txReq.bits.Order   := Order.None
  io.txReq.bits.Addr    := DontCare // remap in chi xbar
  io.txReq.bits.Size    := task_sReq.chi.getSize
  io.txReq.bits.Opcode  := Mux(task_sReq.fromRepl, replOp, task_sReq.chi.opcode)
  io.txReq.bits.TxnID   := task_sReq.llcTxnID.get
  io.txReq.bits.SrcID   := task_sReq.chi.getNoC(io.config.ci)


  /*
   * [CanNest] Update PoS Message
   */
  val fire_nest   = io.updPosNestVec.map(_.fire).reduce(_ | _)
  val cmVec_nest  = cmRegVec.map(_.isCanNest)
  val cmId_nest   = StepRREncoder(cmVec_nest, fire_nest)
  val task_nest   = msgRegVec(cmId_nest)
  // valid
  io.updPosNestVec.zipWithIndex.foreach { case (upd, i) => upd.valid := cmVec_nest.reduce(_ | _) & task_nest.llcTxnID.dirBank === i.U }
  // bits
  io.updPosNestVec.foreach(_.bits.pos := task_nest.llcTxnID.pos)
  io.updPosNestVec.foreach(_.bits.canNest := false.B)

  /*
   * [DataTask]
   */
  val cmVec_sDat  = cmRegVec.map(_.isDataTask)
  val cmId_sDat   = StepRREncoder(cmVec_sDat, io.dataTask.fire)
  val task_sDat   = msgRegVec(cmId_sDat)
  // valid
  io.dataTask.valid := cmVec_sDat.reduce(_ | _)
  // bits
  io.dataTask.bits                := DontCare
  io.dataTask.bits.dataOp         := task_sDat.dataOp
  io.dataTask.bits.llcTxnID       := task_sDat.llcTxnID
  io.dataTask.bits.ds             := task_sDat.ds
  io.dataTask.bits.dataVec        := task_sDat.chi.dataVec
  io.dataTask.bits.txDat.Resp     := task_sDat.cbResp
  io.dataTask.bits.txDat.Opcode   := task_sDat.chi.opcode
  io.dataTask.bits.txDat.TxnID    := task_sDat.chi.txnID
  io.dataTask.bits.txDat.SrcID    := task_sDat.chi.getNoC(io.config.ci)
  io.dataTask.bits.txDat.TgtID    := task_sDat.chi.nodeId


  /*
   * Resp
   */
  val cmVec_resp    = cmRegVec.map(_.isResp)
  val cmId_resp     = StepRREncoder(cmVec_resp, io.respCmt.fire)
  val task_resp     = msgRegVec(cmId_resp)
  // valid
  io.respCmt.valid  := cmVec_resp.reduce(_ | _) & !task_resp.fromRepl
  io.respRepl.valid := cmVec_resp.reduce(_ | _) & task_resp.fromRepl
  // bits respCmt
  io.respCmt.bits               := DontCare
  io.respCmt.bits.llcTxnID      := task_resp.llcTxnID
  io.respCmt.bits.alr           := task_resp.alr
  // bits respRepl
  io.respRepl.bits.llcTxnID     := task_resp.llcTxnID
  io.respRepl.bits.channel      := DontCare
  io.respRepl.bits.resp         := ChiState.I


  /*
   * Modify Ctrl Machine Table
   */
  cmRegVec.zip(msgRegVec).zipWithIndex.foreach {
    case ((cm, msg), i) =>
      val allocHit  = io.alloc.fire    & cmId_rec  === i.U
      val sReqHit   = io.txReq.fire    & cmId_sReq === i.U
      val cNestHit  = fire_nest        & cmId_nest === i.U
      val waitHit   = io.rxRsp.fire    & io.rxRsp.bits.TxnID === msg.llcTxnID.get &
                      (io.rxRsp.bits.Opcode === CompDBIDResp | io.rxRsp.bits.Opcode === DBIDResp) & cm.isValid
      val taskHit   = io.dataTask.fire & cmId_sDat === i.U
      val dataHit   = io.dataResp.fire & io.dataResp.bits.llcTxnID.get === msg.llcTxnID.get & msg.chi.memAttr.ewa & cm.isValid
      val compHit   = io.rxRsp.fire    & io.rxRsp.bits.TxnID === msg.llcTxnID.get & io.rxRsp.bits.Opcode === Comp & cm.isValid
      val respHit   = (io.respCmt.fire | io.respRepl.fire) & cmId_resp === i.U

      // message
      when(allocHit) {
        msg := io.alloc.bits
      }.elsewhen(waitHit) {
        msg.chi.txnID  := io.rxRsp.bits.DBID
        msg.chi.nodeId := io.rxRsp.bits.SrcID
      }.elsewhen(taskHit) {
        msg := 0.U.asTypeOf(msg)
      }

      // ctrl machine state
      cm.state := PriorityMux(Seq(
        allocHit -> SENDREQ,
        sReqHit  -> Mux(msg.chi.toLAN, WAITDBID, CANNEST), // TODO: has risk
        cNestHit -> WAITDBID,
        waitHit  -> DATATASK,
        taskHit  -> Mux(msg.chi.memAttr.ewa, WAITDATA, WAITCOMP),
        dataHit  -> RESP,
        compHit  -> RESP,
        respHit  -> FREE,
        true.B   -> cm.state
      ))

      // HardwareAssertion
      HardwareAssertion.withEn(cm.isFree,     allocHit)
      HardwareAssertion.withEn(cm.isSendReq,  sReqHit)
      HardwareAssertion.withEn(cm.isCanNest,  cNestHit)
      HardwareAssertion.withEn(cm.isWaitDBID, waitHit)
      HardwareAssertion.withEn(cm.isDataTask, taskHit)
      HardwareAssertion.withEn(cm.isWaitData, dataHit)
      HardwareAssertion.withEn(cm.isWaitComp, compHit)
      HardwareAssertion.withEn(cm.isResp,     respHit)
      HardwareAssertion.withEn(io.rxRsp.bits.Opcode === CompDBIDResp, waitHit & msg.chi.memAttr.ewa)
      HardwareAssertion.withEn(io.rxRsp.bits.Opcode === DBIDResp, waitHit & !msg.chi.memAttr.ewa)
      HardwareAssertion.checkTimeout(cm.isFree, TIMEOUT_WOA, cf"TIMEOUT: Write or Atomic Index[${i}]")
  }


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}