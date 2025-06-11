package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.queue.FastQueue
import xs.utils.debug._
import dongjiang.frontend._
import dongjiang.frontend.decode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang.backend._
import dongjiang.backend.READSTATE._
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object READSTATE {
  val width       = 3
  val FREE        = 0x0.U
  val CANNEST     = 0x1.U
  val SENDREQ     = 0x2.U
  val WAITDATA0   = 0x3.U
  val WAITDATA1   = 0x4.U
  val CANTNEST    = 0x5.U
  val SENDACK     = 0x6.U
  val RESPCMT     = 0x7.U
}

class ReadState(implicit p: Parameters) extends DJBundle {
  // REQ To LAN:
  // CHI: Free --> SendReq --> WaitData0 --> WaitData1 --> RespCmt --> Free
  // REQ To BBN:
  // CHI: Free --> CanNest --> SendReq --> WaitData0 --> WaitData1 --> CantNest --> SendCompAck --> RespCmt --> Free
  val state         = UInt(READSTATE.width.W)

  def isFree        = state === FREE
  def isValid       = !isFree
  def isCanNest     = state === CANNEST
  def isSendReq     = state === SENDREQ
  def isWaitData0   = state === WAITDATA0
  def isWaitData1   = state === WAITDATA1
  def isWaitData    = isWaitData0 | isWaitData1
  def isCantNest    = state === CANTNEST
  def isUpdNest     = isCanNest | isCantNest
  def isSendCompAck = state === SENDACK
  def isRespCmt     = state === RESPCMT
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
    // Task
    val alloc         = Flipped(Decoupled(new CMTask))
    val resp          = Decoupled(new CMResp)
    // CHI
    val txReq         = Decoupled(new ReqFlit(true))
    val txRsp         = if(hasBBN) Some(Decoupled(new RespFlit())) else None
    val rxDat         = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Update PoS
    val updPosNest    = Decoupled(new PosCanNest)
    // For Debug
    val dbg           = Valid(new ReadState with HasHnTxnID)
  })

  /*
   * Reg and Wire declaration
   */
  val reg   = RegInit((new ReadState with HasPackCMTask with HasPackTaskInst).Lit(_.state -> FREE))
  val next  = WireInit(reg)

  /*
   * Output for debug
   */
  io.dbg.valid        := reg.isValid
  io.dbg.bits.state   := reg.state
  io.dbg.bits.hnTxnID := reg.task.hnTxnID

  /*
   * Receive Task
   */
  io.alloc.ready := reg.isFree
  HAssert.withEn(!io.alloc.bits.doDMT, io.alloc.valid & io.alloc.bits.chi.toBBN)

  /*
   * SendReq
   */
  // valid
  io.txReq.valid                := reg.isSendReq
  // bits
  io.txReq.bits                 := DontCare
  io.txReq.bits.ExpCompAck      := reg.task.chi.expCompAck
  io.txReq.bits.MemAttr         := reg.task.chi.memAttr.asUInt
  io.txReq.bits.Order           := Order.None
  io.txReq.bits.Addr            := Cat(0.U((addrBits - offsetBits).W), reg.task.chi.getOffset)
  io.txReq.bits.Size            := reg.task.chi.getSize
  io.txReq.bits.Opcode          := reg.task.chi.opcode
  io.txReq.bits.ReturnTxnID.get := Mux(reg.task.doDMT, reg.task.chi.txnID,  reg.task.hnTxnID)
  io.txReq.bits.ReturnNID.get   := Mux(reg.task.doDMT, reg.task.chi.nodeId, Fill(io.txReq.bits.ReturnNID.get.getWidth, 1.U)) // If set RetrunNID max value, it will be remap in SAM
  io.txReq.bits.TxnID           := reg.task.hnTxnID
  io.txReq.bits.SrcID           := reg.task.chi.getNoC

  /*
   * Send CompAck
   */
  if(hasBBN) {
    // valid
    io.txRsp.get.valid        := reg.isSendCompAck
    // bits
    io.txRsp.get.bits         := DontCare
    io.txRsp.get.bits.Opcode  := CompAck
    io.txRsp.get.bits.TxnID   := reg.task.chi.txnID
    io.txRsp.get.bits.SrcID   := reg.task.chi.getNoC
    io.txRsp.get.bits.TgtID   := reg.task.chi.nodeId
  }

  /*
   * Send Resp To Commit
   */
  // valid
  io.resp.valid         := reg.isRespCmt
  // bits
  io.resp.bits.hnTxnID  := reg.task.hnTxnID
  io.resp.bits.fromRec  := false.B
  io.resp.bits.toRepl   := false.B
  when(!reg.task.doDMT) {
    io.resp.bits.taskInst           := 0.U.asTypeOf(new TaskInst)
    io.resp.bits.taskInst.valid     := true.B
    io.resp.bits.taskInst.fwdValid  := false.B
    io.resp.bits.taskInst.channel   := ChiChannel.DAT
    io.resp.bits.taskInst.opcode    := CompData
    io.resp.bits.taskInst.resp      := reg.taskInst.resp
    io.resp.bits.taskInst.fwdResp   := ChiResp.I
  }.otherwise {
    io.resp.bits.taskInst           := 0.U.asTypeOf(new TaskInst)
    io.resp.bits.taskInst.valid     := true.B
  }

  /*
   * Update PoS CanNest flag
   */
  io.updPosNest.valid       := reg.isUpdNest
  io.updPosNest.bits.hnIdx  := reg.task.getHnIdx
  io.updPosNest.bits.nest   := reg.isCanNest

  /*
   * Modify Ctrl Machine
   */
  // Store Msg From Frontend
  val recDataHit  = io.rxDat.fire & reg.task.hnTxnID === io.rxDat.bits.TxnID & io.rxDat.bits.Opcode === CompData
  when(io.alloc.fire) {
    next.task             := io.alloc.bits
    next.taskInst         := 0.U.asTypeOf(new TaskInst)
  // Store Message
  }.elsewhen(recDataHit) {
    // chi
    next.task.chi.nodeId  := io.rxDat.bits.HomeNID
    next.task.chi.txnID   := io.rxDat.bits.DBID
    // inst
    next.taskInst.resp    := io.rxDat.bits.Resp
  }

  // Get Next State
  switch(reg.state) {
    is(FREE) {
      when(io.alloc.fire)       { next.state := Mux(io.alloc.bits.chi.toBBN, CANNEST, SENDREQ) }
    }
    is(CANNEST) {
      when(io.updPosNest.fire)  { next.state := SENDREQ }
    }
    is(SENDREQ) {
      when(io.txReq.fire)       { next.state := Mux(reg.task.doDMT, RESPCMT, WAITDATA0) }
    }
    is(WAITDATA0) {
      when(recDataHit)          { next.state := Mux(reg.task.chi.isHalfSize, Mux(reg.task.chi.toBBN, CANTNEST, RESPCMT), WAITDATA1) }
    }
    is(WAITDATA1) {
      when(recDataHit)          { next.state := Mux(reg.task.chi.toBBN, CANTNEST, RESPCMT) }
    }
    is(CANTNEST) {
      when(io.updPosNest.fire)  { next.state := Mux(reg.task.chi.expCompAck, SENDACK, FREE) }
    }
    is(SENDACK) {
      when(io.txRsp.map(_.fire).getOrElse(false.B)) { next.state := RESPCMT }
    }
    is(RESPCMT) {
      when(io.resp.fire)        { next.state := FREE }
    }
  }

  /*
   * HAssert
   */
  HAssert.withEn(reg.isFree,        io.alloc.fire)
  HAssert.withEn(reg.isUpdNest,     reg.isValid & io.updPosNest.fire)
  HAssert.withEn(reg.isSendReq,     reg.isValid & io.txReq.fire)
  HAssert.withEn(reg.isWaitData,    reg.isValid & recDataHit)
  HAssert.withEn(reg.isRespCmt,     reg.isValid & io.resp.fire)
  HAssert.withEn(reg.isSendCompAck, reg.isValid & io.txRsp.map(_.fire).getOrElse(false.B))

  /*
   * Set new task
   */
  val set = io.alloc.fire | reg.isValid; dontTouch(set)
  when(set) { reg := next }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(reg.isFree, TIMEOUT_READ, cf"TIMEOUT: Read State[${reg.state}]")
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
    // Task
    val alloc         = Flipped(Decoupled(new CMTask))
    val resp          = Decoupled(new CMResp)
    // CHI
    val txReq         = Decoupled(new ReqFlit(true))
    val txRsp         = if(hasBBN) Some(Decoupled(new RespFlit())) else None
    val rxDat         = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Update PoS
    val updPosNest    = Decoupled(new PosCanNest)
  })

  /*
   * Module declaration
   */
  val entries = Seq.fill(nrReadCM) { Module(new ReadEntry()) } // TODO: reserve for toLAN
  val dbgVec  = VecInit(entries.map(_.io.dbg))
  dontTouch(dbgVec)

  /*
   * Connect CM <- IO
   */
  entries.foreach(_.io.config := io.config)
  Alloc(entries.map(_.io.alloc), FastQueue(io.alloc))
  entries.foreach(_.io.rxDat := io.rxDat)

  /*
   * Connect IO <- CM
   */
  io.txReq        <> fastRRArb(entries.map(_.io.txReq)) // TODO: split to LAN and BBN
  io.resp         <> fastRRArb(entries.map(_.io.resp))
  io.updPosNest   <> fastRRArb(entries.map(_.io.updPosNest))
  if(hasBBN) {
    io.txRsp.get  <> fastRRArb(entries.map(_.io.txRsp.get))
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}