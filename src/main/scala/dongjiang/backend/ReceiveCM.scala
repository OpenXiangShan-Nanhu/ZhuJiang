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
import xs.utils.queue.FastQueue
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object State {
  val width   = 6
  val FREE    = "b000001".U // 0x1
  val REQDB   = "b000010".U // 0x2
  val SEND    = "b000100".U // 0x4
  val WAIT0   = "b001000".U // 0x8
  val WAIT1   = "b010000".U // 0x10
  val RESP    = "b100000".U // 0x20
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> ReqDB --> Send --> Wait0 --> Wait1 --> RespCmt --> Free
  val state   = UInt(State.width.W)

  def isFree  = state(0)
  def isValid  = !isFree
  def isReqDB = state(1)
  def isSend  = state(2)
  def isWait0 = state(3)
  def isWait1 = state(4)
  def isResp  = state(5)
}

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- Ctrl Machine Entry ---------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class ReceiveEntry(implicit p: Parameters) extends DJModule {
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
    // For Debug
    val state         = UInt(State.width.W)
  })

  /*
   * Reg and Wire declaration
   */
  val cmReg     = RegInit((new CMState with HasPackCMTask with HasPackTaskInst).Lit(_.state -> FREE))
  val tempDatQ  = Module(new FastQueue(new PackLLCTxnID with HasChiResp { val opcode = UInt(ChiOpcodeBits.W) }, djparam.nrPoS*2 + 1, false))
  io.state      := cmReg.state

  /*
   * [Free] Receive Task
   */
  io.alloc.ready  := cmReg.isFree

  /*
   * ReqDB
   */
  io.reqDB.valid          := cmReg.isReqDB
  io.reqDB.bits.llcTxnID  := cmReg.task.llcTxnID
  io.reqDB.bits.dataVec   := cmReg.task.chi.dataVec

  /*
   * [Send]
   */
  // valid
  io.txRsp.valid        := cmReg.isSend
  // bits
  io.txRsp.bits         := DontCare
  io.txRsp.bits.SrcID   := cmReg.task.chi.getNoC(io.config.ci)
  io.txRsp.bits.TgtID   := cmReg.task.chi.nodeId
  io.txRsp.bits.TxnID   := cmReg.task.chi.txnID
  io.txRsp.bits.Opcode  := cmReg.task.chi.opcode
  io.txRsp.bits.RespErr := RespErr.NormalOkay
  io.txRsp.bits.DBID    := cmReg.task.llcTxnID.get


  /*
   * Resp
   */
  // valid
  io.respCmt.valid              := cmReg.isResp
  // bits
  io.respCmt.bits.llcTxnID      := cmReg.task.llcTxnID
  io.respCmt.bits.alr           := cmReg.task.alr
  io.respCmt.bits.inst.valid    := true.B
  io.respCmt.bits.inst.fwdValid := false.B
  io.respCmt.bits.inst.channel  := cmReg.inst.channel
  io.respCmt.bits.inst.opcode   := cmReg.inst.opcode
  io.respCmt.bits.inst.resp     := cmReg.inst.resp
  io.respCmt.bits.inst.fwdResp  := ChiResp.I


  /*
   * Modify Ctrl Machine Table
   */
  // Hit Message
  val allocHit    = io.alloc.fire
  val reqDBHit    = io.reqDB.fire
  val sRspHit     = io.txRsp.fire
  val waitRspHit  = io.rxRsp.fire & io.rxRsp.bits.TxnID === cmReg.task.llcTxnID.get & io.rxRsp.bits.Opcode === CompAck & cmReg.isValid
  val waitDatHit  = tempDatQ.io.deq.valid & tempDatQ.io.deq.bits.llcTxnID.get === cmReg.task.llcTxnID.get & cmReg.isValid
  val respHit     = io.respCmt.fire

  // Message
  when(allocHit) {
    cmReg.task := io.alloc.bits
    cmReg.inst := 0.U.asTypeOf(new TaskInst)
  // store already reqs
  }.elsewhen(reqDBHit) {
    cmReg.task.alr.reqs := true.B
    HardwareAssertion(!cmReg.task.alr.reqs)
  // Store Inst
  }.elsewhen(waitRspHit | waitDatHit) {
    cmReg.inst.channel := Mux(waitRspHit, ChiChannel.RSP,       ChiChannel.DAT)
    cmReg.inst.opcode  := Mux(waitRspHit, io.rxRsp.bits.Opcode, tempDatQ.io.deq.bits.opcode)
    cmReg.inst.resp    := Mux(waitRspHit, io.rxRsp.bits.Resp,   tempDatQ.io.deq.bits.resp)
  }

  // ctrl machine state
  cmReg.state := PriorityMux(Seq(
    allocHit    -> Mux(io.alloc.bits.alr.sDBID, WAIT0, Mux(io.alloc.bits.needReqDB, REQDB, SEND)),
    reqDBHit    -> SEND,
    sRspHit     -> WAIT0,
    waitRspHit  -> RESP,
    waitDatHit  -> Mux(cmReg.task.chi.isFullSize, Mux(cmReg.isWait1, RESP, WAIT1), RESP),
    respHit     -> FREE,
    true.B      -> cmReg.state
  ))

  // HardwareAssertion
  HardwareAssertion.withEn(cmReg.isFree,   allocHit)
  HardwareAssertion.withEn(cmReg.isReqDB,  reqDBHit)
  HardwareAssertion.withEn(cmReg.isSend,   sRspHit)
  HardwareAssertion.withEn(cmReg.isWait0,  waitRspHit)
  HardwareAssertion.withEn(cmReg.isWait0 | cmReg.isWait1, waitDatHit)
  HardwareAssertion.withEn(cmReg.isResp,   respHit)
  HardwareAssertion.withEn(cmReg.task.chi.rspIs(Comp), waitRspHit)
  HardwareAssertion.withEn(cmReg.task.chi.rspIs(CompDBIDResp, DBIDResp), waitDatHit)
  HardwareAssertion.checkTimeout(cmReg.isFree, TIMEOUT_REC, cf"TIMEOUT: Receive State[0x${cmReg.state}%x]")

  /*
   * Temporary Data Response
   */
  val datIsWrData = io.rxDat.fire & (io.rxDat.bits.Opcode === NonCopyBackWriteData | io.rxDat.bits.Opcode === CopyBackWriteData)
  val deqNoHit    = tempDatQ.io.deq.valid & !tempDatQ.io.deq.ready
  // enq
  tempDatQ.io.enq.valid           := datIsWrData | deqNoHit
  when(datIsWrData) {
    tempDatQ.io.enq.bits.llcTxnID := io.rxDat.bits.TxnID.asTypeOf(new LLCTxnID())
    tempDatQ.io.enq.bits.opcode   := io.rxDat.bits.Opcode
    tempDatQ.io.enq.bits.resp     := io.rxDat.bits.Resp
    HardwareAssertion(io.rxDat.bits.TxnID >> new LLCTxnID().getWidth === 0.U)
  }.otherwise {
    tempDatQ.io.enq.bits          := tempDatQ.io.deq.bits
  }
  // deq
  tempDatQ.io.deq.ready := Mux(datIsWrData, waitDatHit, true.B)
  // HardwareAssertion
  HardwareAssertion.withEn(tempDatQ.io.enq.ready, tempDatQ.io.enq.valid)
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
   * Module declaration
   */
  val entries = Seq.fill(nrReceiveCM) { Module(new ReceiveEntry()) }

  /*
   * For debug
   */
  val stateVec = Wire(Vec(nrReceiveCM, UInt(State.width.W)))
  stateVec.zip(entries.map(_.io.state)).foreach { case(a, b) => a := b }
  dontTouch(stateVec)

  /*
   * Connect CM <- IO
   */
  entries.foreach(_.io.config := io.config)
  Alloc(entries.map(_.io.alloc), io.alloc)
  entries.foreach(_.io.rxRsp := io.rxRsp)
  entries.foreach(_.io.rxDat := io.rxDat)

  /*
   * Connect IO <- CM
   */
  io.txRsp    <> fastRRArb(entries.map(_.io.txRsp))
  io.respCmt  <> fastRRArb(entries.map(_.io.respCmt))
  io.reqDB    <> fastRRArb(entries.map(_.io.reqDB))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}