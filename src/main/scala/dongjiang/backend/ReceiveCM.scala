package dongjiang.backend

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
import dongjiang.backend.RECSTATE._
import dongjiang.data._
import xs.utils.queue.FastQueue
import chisel3.experimental.BundleLiterals._
import dongjiang.backend.READSTATE.{CANNEST, FREE, REQDB, SENDREQ}

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object RECSTATE {
  val width     = 3
  val FREE      = 0x0.U
  val WAITRES   = 0x1.U // Wait Directory result
  val REQDB     = 0x2.U
  val SENDRSP   = 0x3.U
  val WAITDATA0 = 0x4.U
  val WAITDATA1 = 0x5.U
  val WAITACK   = 0x6.U
  val RESPCMT   = 0x7.U
}

trait HasRecState { this: Bundle =>
  // No WriteEvictOrEvict:
  // CHI:         Free --> SendRsp --> WaitData0 --> WaitData1 --> RespCmt --> Free
  // WriteEvictOrEvict:
  // GetData:     Free --> WaitRes --> ReqDB --> SendRsp --> WaitData0 --> WaitData1 --> RespCmt --> Free
  // NotGetData:  Free --> WaitRes --> SendRsp --> WaitAck--> RespCmt --> Free
  val state       = UInt(RECSTATE.width.W)

  def isFree      = state === FREE
  def isValid     = !isFree
  def isWaitRes   = state === WAITRES
  def isReqDB     = state === REQDB
  def isSendRsp   = state === SENDRSP
  def isWaitData0 = state === WAITDATA0
  def isWaitData1 = state === WAITDATA1
  def isWaitData  = isWaitData0 | isWaitData1
  def isWaitAck   = state === WAITACK
  def isRespCmt   = state === RESPCMT
}

class FastResp(implicit p: Parameters) extends DJBundle with HasDataVec { val rsp = new RespFlit() }

class RespComp(implicit p: Parameters) extends DJBundle with HasHnTxnID { val comp = Bool() }

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- Ctrl Machine Entry ---------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class ReceiveEntry(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Task
    val alloc       = Flipped(Decoupled(new FastResp))
    val respCompVec = Vec(djparam.nrDirBank, Flipped(Valid(new RespComp))) // broadcast signal
    val resp        = Decoupled(new CMResp)
    // ReqDB
    val reqDB       = Decoupled(new HnTxnID with HasDataVec)
    // CHI
    val txRsp       = Decoupled(new RespFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit()))
    // For Debug
    val dbg         = Valid(new HnTxnID with HasRecState)
  })

  /*
   * Reg and Wire declaration
   */
  val reg   = RegInit(new DJBundle with HasDataVec with HasRecState with HasPackTaskInst {
    val rsp = new RespFlit
  }.Lit(_.state -> FREE))
  val next  = WireInit(reg)

  /*
   * Output for debug
   */
  io.dbg.valid        := reg.isValid
  io.dbg.bits.state   := reg.state
  io.dbg.bits.hnTxnID := reg.rsp.DBID

  /*
   * Receive Task
   */
  io.alloc.ready  := reg.isFree

  /*
   * ReqDB
   */
  io.reqDB.valid        := reg.isReqDB
  io.reqDB.bits.hnTxnID := reg.rsp.DBID
  io.reqDB.bits.dataVec := DataVec.Full
  HAssert.withEn(reg.isFullSize, io.reqDB.valid)

  /*
   * Send Rsp to CHI
   */
  io.txRsp.valid  := reg.isSendRsp
  io.txRsp.bits   := reg.rsp

  /*
   * Send Resp To Commit
   */
  // valid
  io.resp.valid           := reg.isRespCmt
  // bits
  io.resp.bits.hnTxnID    := reg.rsp.DBID
  io.resp.bits.alr.reqs   := reg.rsp.Opcode =/= Comp
  io.resp.bits.alr.sData  := DontCare
  io.resp.bits.fromRec    := true.B
  io.resp.bits.toRepl     := false.B
  io.resp.bits.taskInst   := reg.taskInst
  HAssert.withEn(io.resp.bits.taskInst.wrRespIsDat ^ io.resp.bits.taskInst.cbRespIsAck, io.resp.valid)


  /*
   * Modify Ctrl Machine Table
   */
  val dirResVec = VecInit(io.respCompVec.map(r => r.fire & r.bits.hnTxnID === reg.rsp.DBID))
  val dirResId  = PriorityEncoder(dirResVec)
  val dirResHit = dirResVec.asUInt.orR
  HAssert(PopCount(dirResVec) <= 1.U)
  val rspHit    = io.rxRsp.fire    & io.rxRsp.bits.TxnID === reg.rsp.DBID & io.rxRsp.bits.Opcode === CompAck & reg.rsp.Opcode === Comp
  val datHit    = io.rxDat.fire    & io.rxDat.bits.TxnID === reg.rsp.DBID & (io.rxDat.bits.Opcode === CopyBackWriteData | io.rxDat.bits.Opcode === NonCopyBackWriteData)
  // Store Msg From Frontend
  when(io.alloc.fire) {
    next.rsp              := io.alloc.bits.rsp
    next.dataVec          := io.alloc.bits.dataVec
  // Modify to send CompDBIDResp
  }.elsewhen(dirResHit & !io.respCompVec(dirResId).bits.comp) {
    next.rsp.Opcode       := CompDBIDResp
  // Store Inst
  }.elsewhen(rspHit | datHit) {
    next.taskInst.valid       := true.B
    next.taskInst.wrRespIsDat := datHit
    next.taskInst.cbRespIsAck := rspHit
    next.taskInst.cbResp      := Mux(datHit, io.rxDat.bits.Resp, ChiResp.I)
  }

  // Get Next State
  switch(reg.state) {
    is(FREE) {
      when(io.alloc.fire) { next.state := Mux(io.alloc.bits.rsp.Opcode === Comp, WAITRES, SENDRSP) }
    }
    is(WAITRES) {
      when(dirResHit)     { next.state := Mux(io.respCompVec(dirResId).bits.comp, SENDRSP, REQDB) }
    }
    is(REQDB) {
      when(io.reqDB.fire) { next.state := SENDRSP }
    }
    is(SENDRSP) {
      when(io.txRsp.fire) { next.state := Mux(reg.rsp.Opcode === Comp, WAITACK, WAITDATA0) }
    }
    is(WAITDATA0) {
      when(datHit)        { next.state := Mux(reg.isHalfSize, RESPCMT, WAITDATA1) }
    }
    is(WAITDATA1) {
      when(datHit)        { next.state := RESPCMT }
    }
    is(WAITACK) {
      when(rspHit)        { next.state := RESPCMT }
    }
    is(RESPCMT) {
      when(io.resp.fire)  { next.state := FREE }
    }
  }

  /*
   * HAssert
   */
  HAssert.withEn(reg.isFree,      io.alloc.fire)
  HAssert.withEn(reg.isWaitRes,   reg.isValid & dirResHit)
  HAssert.withEn(reg.isReqDB,     reg.isValid & io.reqDB.fire)
  HAssert.withEn(reg.isSendRsp,   reg.isValid & io.txRsp.fire)
  HAssert.withEn(reg.isWaitData,  reg.isValid & datHit)
  HAssert.withEn(reg.isWaitAck,   reg.isValid & rspHit)
  HAssert.withEn(reg.isRespCmt,   reg.isValid & io.resp.fire)

  /*
   * Set new task
   */
  val set = io.alloc.fire | reg.isValid; dontTouch(set)
  when(set) { reg := next }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(reg.isFree, TIMEOUT_REC, cf"TIMEOUT: Receive State[${reg.state}]")
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Ctrl Machine ------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class ReceiveCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Task
    val alloc       = Flipped(Decoupled(new FastResp))
    val respCompVec = Vec(djparam.nrDirBank, Flipped(Valid(new RespComp))) // broadcast signal
    val resp        = Decoupled(new CMResp)
    // ReqDB
    val reqDB       = Decoupled(new HnTxnID with HasDataVec)
    // CHI
    val txRsp       = Decoupled(new RespFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit()))
  })

  /*
   * Module and Reg declaration
   */
  val entries = Seq.fill(nrReceiveCM) { Module(new ReceiveEntry()) }
  val dbgVec  = VecInit(entries.map(_.io.dbg))
  dontTouch(dbgVec)
  
  /*
   * Connect CM <- IO
   */
  entries.foreach(_.io.config := io.config)
  Alloc(entries.map(_.io.alloc), io.alloc)
  entries.foreach(_.io.respCompVec := io.respCompVec)
  entries.foreach(_.io.rxRsp := io.rxRsp)
  entries.foreach(_.io.rxDat := io.rxDat)

  /*
   * Connect IO <- CM
   */
  io.txRsp  <> fastRRArb(entries.map(_.io.txRsp))
  io.resp   <> fastRRArb(entries.map(_.io.resp))
  io.reqDB  <> fastRRArb(entries.map(_.io.reqDB))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}