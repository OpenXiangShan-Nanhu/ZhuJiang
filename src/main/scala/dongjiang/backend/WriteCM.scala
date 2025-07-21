package dongjiang.backend

import chisel3._
import chisel3.experimental.BundleLiterals._
import chisel3.util._
import dongjiang._
import dongjiang.backend.WriteState._
import dongjiang.bundle._
import dongjiang.data._
import dongjiang.frontend._
import dongjiang.utils._
import org.chipsalliance.cde.config._
import xs.utils.debug._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object WriteState {
  val width       = 3
  val FREE        = 0x0.U
  val CANNEST     = 0x1.U
  val SENDREQ     = 0x2.U
  val WAITDBID    = 0x3.U
  val DATATASK    = 0x4.U
  val WAITDATA    = 0x5.U
  val CANTNEST    = 0x6.U
  val RESPCMT     = 0x7.U
}

class WriMes(implicit p: Parameters) extends DJBundle {
  // TODO: WriteEvictOrEvict to BBN
  // REQ To LAN:
  // CHI: Free --> SendReq --> WaitDBID --> DataTask --> WaitData --> RespCmt --> Free
  // REQ To BBN:
  // CHI: Free --> CanNest --> SendReq --> WaitDBID --> DataTask --> WaitData --> CantNest --> RespCmt --> Free
  val state       = UInt(WriteState.width.W)
  val alrGetComp  = Bool() // already get comp from CHI

  def isFree      = state === FREE
  def isValid     = !isFree
  def isCanNest   = state === CANNEST
  def isSendReq   = state === SENDREQ
  def isWaitDBID  = state === WAITDBID
  def isDataTask  = state === DATATASK
  def isWaitData  = state === WAITDATA
  def isCantNest  = state === CANTNEST
  def isUpdNest   = isCanNest | isCantNest
  def isRespCmt   = state === RESPCMT & alrGetComp
}

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- Ctrl Machine Entry ---------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class WriteEntry(implicit p: Parameters) extends DJModule {
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
    val rxRsp         = Flipped(Valid(new RespFlit()))
    // DataTask
    val dataTask      = Decoupled(new DataTask)
    val dataResp      = Flipped(Valid(new HnTxnID()))
    // Update PoS
    val updPosNest    = if(hasBBN) Some(Decoupled(new PosCanNest)) else None
    // For Debug
    val dbg           = Valid(new ReadState with HasHnTxnID)
  })

  /*
   * Reg and Wire declaration
   */
  val reg   = RegInit((new WriMes with HasPackCMTask with HasRespErr).Lit(_.state -> FREE, _.alrGetComp -> false.B))
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
  io.alloc.ready  := reg.isFree

  /*
   * SendReq
   */
  // valid
  io.txReq.valid        := reg.isSendReq
  // bits
  io.txReq.bits         := DontCare
  io.txReq.bits.MemAttr := reg.task.chi.memAttr.asUInt
  io.txReq.bits.Order   := Order.None
  io.txReq.bits.Addr    := DontCare
  io.txReq.bits.Size    := reg.task.chi.size
  io.txReq.bits.Opcode  := reg.task.chi.opcode
  io.txReq.bits.TxnID   := reg.task.hnTxnID
  io.txReq.bits.SrcID   := reg.task.chi.getNoC
  io.txReq.bits.QoS     := reg.task.qos


  /*
   * Update PoS Message
   */
  if (hasBBN) {
    io.updPosNest.get.valid       := reg.isUpdNest
    io.updPosNest.get.bits.hnIdx  := reg.task.getHnIdx
    io.updPosNest.get.bits.nest   := reg.isCanNest
    io.updPosNest.get.bits.qos    := reg.task.qos
  }

  /*
   * Send DataTask to DataBlock
   */
  // valid
  io.dataTask.valid               := reg.isDataTask
  // bits
  io.dataTask.bits                := DontCare
  io.dataTask.bits.dataOp         := reg.task.dataOp
  io.dataTask.bits.hnTxnID        := reg.task.hnTxnID
  io.dataTask.bits.ds             := reg.task.ds
  io.dataTask.bits.dataVec        := reg.task.chi.dataVec
  io.dataTask.bits.txDat.Resp     := reg.task.cbResp
  io.dataTask.bits.txDat.Opcode   := Mux(reg.task.chi.isImmediateWrite, NonCopyBackWriteData, CopyBackWriteData)
  io.dataTask.bits.txDat.TxnID    := reg.task.chi.txnID
  io.dataTask.bits.txDat.SrcID    := reg.task.chi.getNoC
  io.dataTask.bits.txDat.TgtID    := reg.task.chi.nodeId
  io.dataTask.bits.qos            := reg.task.qos

  /*
   * Send Resp To Commit
   */
  // valid
  io.resp.valid               := reg.isRespCmt
  // bits respCmt
  io.resp.bits                := DontCare
  io.resp.bits.hnTxnID        := reg.task.hnTxnID
  io.resp.bits.toRepl         := reg.task.fromRepl
  io.resp.bits.taskInst.valid := true.B
  io.resp.bits.qos            := reg.task.qos
  io.resp.bits.respErr        := reg.respErr

  /*
   * Modify Ctrl Machine Table
   */
  val dbidHit = reg.isValid & io.rxRsp.fire & io.rxRsp.bits.TxnID === reg.task.hnTxnID & (io.rxRsp.bits.Opcode === CompDBIDResp | io.rxRsp.bits.Opcode === DBIDResp)
  val compHit = reg.isValid & io.rxRsp.fire & io.rxRsp.bits.TxnID === reg.task.hnTxnID & (io.rxRsp.bits.Opcode === CompDBIDResp | io.rxRsp.bits.Opcode === Comp)
  // Store Msg From Frontend or CHI
  when(io.alloc.fire) {
    next.task             := io.alloc.bits
    next.alrGetComp       := false.B
    next.respErr          := RespErr.NormalOkay
  }.elsewhen(dbidHit) {
    next.task.chi.txnID   := io.rxRsp.bits.DBID
    next.task.chi.nodeId  := io.rxRsp.bits.SrcID
    next.alrGetComp       := compHit | reg.alrGetComp
    when(!reg.isERR) {
      next.respErr        := io.rxRsp.bits.RespErr
    }
    HAssert.withEn(!reg.alrGetComp, compHit)
  }.elsewhen(compHit) {
    next.alrGetComp       := compHit
    next.respErr          := io.rxRsp.bits.RespErr
    when(!reg.isERR) {
      next.respErr        := io.rxRsp.bits.RespErr
    }
    HAssert(!reg.alrGetComp)
  }
  HAssert.withEn(!reg.alrGetComp, compHit)

  // Get Next State
  val updNestFire = io.updPosNest.map(_.fire).getOrElse(false.B)
  val dataRespHit = io.dataResp.fire  & io.dataResp.bits.hnTxnID === reg.task.hnTxnID
  switch(reg.state) {
    is(FREE) {
      when(io.alloc.fire)     { next.state := Mux(io.alloc.bits.chi.toBBN, CANNEST, SENDREQ) }
    }
    is(CANNEST) {
      when(updNestFire)       { next.state := SENDREQ }
    }
    is(SENDREQ) {
      when(io.txReq.fire)     { next.state := WAITDBID }
    }
    is(WAITDBID) {
      when(dbidHit)           { next.state := DATATASK }
    }
    is(DATATASK) {
      when(io.dataTask.fire)  { next.state := WAITDATA }
    }
    is(WAITDATA) {
      when(dataRespHit)       { next.state := Mux(reg.task.chi.toBBN, CANTNEST, RESPCMT) }
    }
    is(CANTNEST) {
      when(updNestFire)       { next.state := RESPCMT }
    }
    is(RESPCMT) {
      when(io.resp.fire)      { next.state := FREE }
    }
  }

  /*
   * HAssert
   */
  HAssert.withEn(reg.isFree,      io.alloc.fire)
  HAssert.withEn(reg.isUpdNest,   reg.isValid & updNestFire)
  HAssert.withEn(reg.isWaitDBID,  reg.isValid & dbidHit)
  HAssert.withEn(reg.isDataTask,  reg.isValid & io.dataTask.fire)
  HAssert.withEn(reg.isWaitData,  reg.isValid & dataRespHit)
  HAssert.withEn(reg.isRespCmt,   reg.isValid & io.resp.fire)

  /*
   * Set new task
   */
  val set = io.alloc.fire | reg.isValid; dontTouch(set)
  when(set) { reg := next }

  // HardwareAssertion
  HardwareAssertion.checkTimeout(reg.isFree, TIMEOUT_WRITE, cf"TIMEOUT: Write State[${reg.state}]")
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Ctrl Machine ------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class WriteCM(implicit p: Parameters) extends DJModule {
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
    val rxRsp         = Flipped(Valid(new RespFlit()))
    // DataTask
    val dataTask      = Decoupled(new DataTask)
    val dataResp      = Flipped(Valid(new HnTxnID()))
    // Update PoS
    val updPosNest    = if(hasBBN) Some(Decoupled(new PosCanNest)) else None
  })

  /*
   * Module declaration
   */
  val entries = Seq.fill(nrWriteCM) { Module(new WriteEntry()) } // TODO: reserve for toLAN
  val dbgVec  = VecInit(entries.map(_.io.dbg))
  dontTouch(dbgVec)

  /*
   * Connect CM <- IO
   */
  Alloc(entries.map(_.io.alloc), io.alloc)
  entries.foreach(_.io.config   := io.config)
  entries.foreach(_.io.rxRsp    := io.rxRsp)
  entries.foreach(_.io.dataResp := io.dataResp)

  /*
   * Connect IO <- CM
   */
  io.txReq      <> fastQosRRArb(entries.map(_.io.txReq)) // TODO: split to LAN and BBN
  io.resp       <> fastQosRRArb(entries.map(_.io.resp))
  io.dataTask   <> fastQosRRArb(entries.map(_.io.dataTask))
  if(hasBBN) {
    io.updPosNest.get <> fastQosRRArb(entries.map(_.io.updPosNest.get))
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}