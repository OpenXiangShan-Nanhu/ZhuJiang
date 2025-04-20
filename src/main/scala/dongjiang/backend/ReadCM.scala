package dongjiang.backend.read

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang.{backend, _}
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
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object State {
  val width       = 7
  val FREE        = "b0000001".U
  val REQDB       = "b0000010".U
  val SENDREQ     = "b0000100".U
  val WAITDATA0   = "b0001000".U
  val WAITDATA1   = "b0010000".U
  val RESPCMT     = "b0100000".U
  val SENDACK     = "b1000000".U
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> ReqDB --> SendReq --> WaitData0 --> WaitData1 -->  RespCmt--> SendCompAck --> Free
  val state         = UInt(State.width.W)

  def isFree        = state(0)
  def isValid       = !isFree
  def isReqDB       = state(1)
  def isSendReq     = state(2)
  def isWaitData0   = state(3)
  def isWaitData1   = state(4)
  def isWaitData    = isWaitData0 | isWaitData1
  def isRespCmt     = state(5)
  def isSendCompAck = state(6)
}

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- Ctrl Machine Entry ---------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class ReadEntry(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config        = new DJConfigIO()
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
    // For Debug
    val state         = UInt(State.width.W)
  })

  /*
   * Reg and Wire declaration
   */
  val cmReg = RegInit((new CMState with HasPackCMTask with HasPackTaskInst).Lit(_.state -> FREE))
  io.state  := cmReg.state

  /*
   * Receive Task IO
   */
  io.alloc.ready := cmReg.isFree

  /*
   * ReqDB
   */
  io.reqDB.valid          := cmReg.isReqDB
  io.reqDB.bits.llcTxnID  := cmReg.task.llcTxnID
  io.reqDB.bits.dataVec   := cmReg.task.chi.dataVec

  /*
   * SendReq
   */
  // valid
  io.txReq.valid  := cmReg.isSendReq
  // bits
  io.txReq.bits                 := DontCare
  io.txReq.bits.ExpCompAck      := cmReg.task.chi.expCompAck
  io.txReq.bits.MemAttr         := cmReg.task.chi.memAttr.asUInt
  io.txReq.bits.Order           := Order.None
  io.txReq.bits.Addr            := DontCare // set in chi xbar
  io.txReq.bits.Size            := cmReg.task.chi.getSize
  io.txReq.bits.Opcode          := cmReg.task.chi.opcode
  io.txReq.bits.ReturnTxnID.get := Mux(cmReg.task.doDMT, cmReg.task.chi.txnID,  cmReg.task.llcTxnID.get)
  io.txReq.bits.ReturnNID.get   := Mux(cmReg.task.doDMT, cmReg.task.chi.nodeId, Fill(io.txReq.bits.ReturnNID.get.getWidth, 1.U)) // If set RetrunNID max value, it will be remap in SAM
  io.txReq.bits.TxnID           := cmReg.task.llcTxnID.get
  io.txReq.bits.SrcID           := cmReg.task.chi.getNoC(io.config.ci)

  /*
   * Send CompAck
   */
  // valid
  io.txRsp.valid  := cmReg.isSendCompAck
  // bits
  io.txRsp.bits         := DontCare
  io.txRsp.bits.Opcode  := CompAck
  io.txRsp.bits.TxnID   := cmReg.task.chi.txnID
  io.txRsp.bits.SrcID   := cmReg.task.chi.getNoC(io.config.ci)
  io.txRsp.bits.TgtID   := cmReg.task.chi.nodeId


  /*
   * Send Resp To Commit
   */
  // valid
  io.respCmt.valid := cmReg.isRespCmt
  // bits
  io.respCmt.bits.llcTxnID  := cmReg.task.llcTxnID
  io.respCmt.bits.alr       := cmReg.task.alr
  when(cmReg.task.doDMT) {
    io.respCmt.bits.inst          := 0.U.asTypeOf(new TaskInst)
  }.otherwise {
    io.respCmt.bits.inst.valid    := true.B
    io.respCmt.bits.inst.fwdValid := false.B
    io.respCmt.bits.inst.channel  := ChiChannel.DAT
    io.respCmt.bits.inst.opcode   := CompData
    io.respCmt.bits.inst.resp     := cmReg.inst.resp
    io.respCmt.bits.inst.fwdResp  := ChiResp.I
  }

  /*
   * Modify Ctrl Machine
   */
  // Hit Message
  val allocHit    = io.alloc.fire
  val reqDBHit    = io.reqDB.fire
  val sendReqHit  = io.txReq.fire
  val recDataHit  = io.rxDat.fire & cmReg.task.llcTxnID.get === io.rxDat.bits.TxnID & io.rxDat.bits.Opcode === CompData & cm.isValid
  val sendAckHit  = io.txRsp.fire
  val respCmtHit  = io.respCmt.fire

  // Store Msg From Frontend
  when(allocHit) {
    cmReg.task := io.alloc.bits
    cmReg.inst := 0.U.asTypeOf(new TaskInst)
  // store already reqs
  }.elsewhen(reqDBHit) {
    cmReg.task.alr.reqs := true.B
  // Store Message
  }.elsewhen(recDataHit) {
    // chi
    cmReg.task.chi.nodeId := io.rxDat.bits.HomeNID
    cmReg.task.chi.txnID  := io.rxDat.bits.DBID
    // inst
    cmReg.inst.resp       := io.rxDat.bits.DBID
  }

  // Get Next State
  cmReg.state := PriorityMux(Seq(
    allocHit    -> Mux(io.alloc.bits.needReqDB, REQDB, SENDREQ),
    reqDBHit    -> SENDREQ,
    sendReqHit  -> Mux(!cmReg.task.doDMT, WAITDATA0, RESPCMT),
    recDataHit  -> Mux(cmReg.task.chi.isHalfSize, RESPCMT, WAITDATA1),
    respCmtHit  -> Mux(cmReg.task.chi.expCompAck, SENDACK, FREE),
    sendAckHit  -> FREE,
    true.B      -> cmReg.state,
  ))

  // HardwareAssertion
  HardwareAssertion.withEn(cmReg.isFree,          allocHit)
  HardwareAssertion.withEn(cmReg.isReqDB,         reqDBHit)
  HardwareAssertion.withEn(cmReg.isSendReq,       sendReqHit)
  HardwareAssertion.withEn(cmReg.isWaitData,      recDataHit)
  HardwareAssertion.withEn(cmReg.isSendCompAck,   sendAckHit)
  HardwareAssertion.withEn(cmReg.isRespCmt,       respCmtHit)
  HardwareAssertion(PopCount(Seq(allocHit, reqDBHit, sendReqHit, recDataHit, sendAckHit, respCmtHit)) <= 1.U)
  HardwareAssertion.checkTimeout(cmReg.isFree, TIMEOUT_READ, cf"TIMEOUT: Read State[${cmReg.state}]")
}


// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Ctrl Machine ------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class ReadCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config        = new DJConfigIO()
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
   * Module declaration
   */
  val entries = Seq.fill(nrReadCM) { Module(new ReadEntry()) }

  /*
   * For debug
   */
  val stateVec = Wire(Vec(nrReadCM, UInt(State.width.W)))
  stateVec.zip(entries.map(_.io.state)).foreach { case(a, b) => a := b }
  dontTouch(stateVec)

  /*
   * Connect CM <- IO
   */
  entries.foreach(_.io.config := io.config)
  Alloc(entries.map(_.io.alloc), io.alloc)
  entries.foreach(_.io.rxDat := io.rxDat)

  /*
   * Connect IO <- CM
   */
  io.txReq    <> fastRRArb(entries.map(_.io.txReq)) // TODO: split to LAN and BBN
  io.txRsp    <> fastRRArb(entries.map(_.io.txRsp))
  io.respCmt  <> fastRRArb(entries.map(_.io.respCmt))
  io.reqDB    <> fastRRArb(entries.map(_.io.reqDB))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}