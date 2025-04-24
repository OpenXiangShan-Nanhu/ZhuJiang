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
import zhujiang.chi.DatOpcode._
import dongjiang.backend._
import dongjiang.backend.wrioratm.State._
import dongjiang.data._
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object State {
  val width       = 8
  val FREE        = "b00000001".U // 0x1
  val SENDREQ     = "b00000010".U // 0x2
  val CANNEST     = "b00000100".U // 0x4
  val WAITDBID    = "b00001000".U // 0x8
  val DATATASK    = "b00010000".U // 0x10
  val WAITDATA    = "b00100000".U // 0x20
  val WAITCOMP    = "b01000000".U // 0x40
  val RESP        = "b10000000".U // 0x80
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> SendReq  --> CanNest --> WaitDBID --> DataTask --> WaitData/WaitComp --> RespCmt/RespRepl --> Free
  val state       = UInt(State.width.W)

  def isFree      = state(0)
  def isValid     = !isFree
  def isSendReq   = state(1)
  def isCanNest   = state(2)
  def isWaitDBID  = state(3)
  def isDataTask  = state(4)
  def isWaitData  = state(5)
  def isWaitComp  = state(6)
  def isResp      = state(7)
}


class WriOrAtmEntry(implicit p: Parameters) extends DJModule {
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
    // For Debug
    val state         = UInt(State.width.W)
  })

  /*
   * Reg and Wire declaration
   */
  val cmReg = RegInit((new CMState with HasPackCMTask).Lit(_.state -> FREE))
  io.state  := cmReg.state


  /*
   * [Free] Receive Task IO
   */
  io.alloc.ready  := cmReg.isFree
  HardwareAssertion.withEn(!io.alloc.bits.dataOp.reqs, io.alloc.valid & io.alloc.bits.alr.reqs)

  /*
   * [SendReq]
   */
  // valid
  io.txReq.valid        := cmReg.isSendReq

  // bits
  io.txReq.bits         := DontCare
  io.txReq.bits.MemAttr := cmReg.task.chi.memAttr.asUInt
  io.txReq.bits.Order   := Order.None
  io.txReq.bits.Addr    := DontCare // remap in chi xbar
  io.txReq.bits.Size    := cmReg.task.chi.getSize
  io.txReq.bits.Opcode  := cmReg.task.chi.opcode
  io.txReq.bits.TxnID   := cmReg.task.llcTxnID.get
  io.txReq.bits.SrcID   := cmReg.task.chi.getNoC(io.config.ci)


  /*
   * [CanNest] Update PoS Message
   */
  val fire_nest   = io.updPosNestVec.map(_.fire).reduce(_ | _)
  // valid
  io.updPosNestVec.zipWithIndex.foreach { case (upd, i) => upd.valid := cmReg.isCanNest & cmReg.task.llcTxnID.dirBank === i.U }
  // bits
  io.updPosNestVec.foreach(_.bits.pos := cmReg.task.llcTxnID.pos)
  io.updPosNestVec.foreach(_.bits.canNest := true.B)

  /*
   * [DataTask]
   */
  // valid
  io.dataTask.valid               := cmReg.isDataTask
  // bits
  io.dataTask.bits                := DontCare
  io.dataTask.bits.dataOp         := cmReg.task.dataOp
  io.dataTask.bits.dataOp.reqs    := cmReg.task.needReqDB
  io.dataTask.bits.llcTxnID       := cmReg.task.llcTxnID
  io.dataTask.bits.ds             := cmReg.task.ds
  io.dataTask.bits.dataVec        := cmReg.task.chi.dataVec
  io.dataTask.bits.txDat.Resp     := cmReg.task.cbResp
  io.dataTask.bits.txDat.Opcode   := Mux(cmReg.task.chi.isImmediateWrite, NonCopyBackWriteData, CopyBackWriteData)
  io.dataTask.bits.txDat.TxnID    := cmReg.task.chi.txnID
  io.dataTask.bits.txDat.SrcID    := cmReg.task.chi.getNoC(io.config.ci)
  io.dataTask.bits.txDat.TgtID    := cmReg.task.chi.nodeId


  /*
   * Resp
   */
  // valid
  io.respCmt.valid  := cmReg.isResp & !cmReg.task.fromRepl
  io.respRepl.valid := cmReg.isResp &  cmReg.task.fromRepl
  // bits respCmt
  io.respCmt.bits               := DontCare
  io.respCmt.bits.llcTxnID      := cmReg.task.llcTxnID
  io.respCmt.bits.alr           := cmReg.task.alr
  // bits respRepl
  io.respRepl.bits.llcTxnID     := cmReg.task.llcTxnID
  io.respRepl.bits.channel      := ChiChannel.RSP
  io.respRepl.bits.resp         := ChiState.I


  /*
   * Modify Ctrl Machine Table
   */
  // Hit Message
  val allocHit  = io.alloc.fire
  val sReqHit   = io.txReq.fire
  val cNestHit  = fire_nest
  val waitHit   = io.rxRsp.fire    & io.rxRsp.bits.TxnID === cmReg.task.llcTxnID.get &
                  (io.rxRsp.bits.Opcode === CompDBIDResp | io.rxRsp.bits.Opcode === DBIDResp) & cmReg.isValid
  val taskHit   = io.dataTask.fire
  val dataHit   = io.dataResp.fire & io.dataResp.bits.llcTxnID.get === cmReg.task.llcTxnID.get & cmReg.isValid
  val compHit   = io.rxRsp.fire    & io.rxRsp.bits.TxnID === cmReg.task.llcTxnID.get & io.rxRsp.bits.Opcode === Comp & cmReg.isValid
  val respHit   = io.respCmt.fire | io.respRepl.fire

  // task
  when(allocHit) {
    // TODO: toBBN WriteBackFull / WriteEvictOrEvict
    val replOp = WriteNoSnpFull
    cmReg.task := io.alloc.bits
    cmReg.task.chi.opcode := Mux(io.alloc.bits.fromRepl, replOp, io.alloc.bits.chi.opcode)
  }.elsewhen(waitHit) {
    cmReg.task.chi.txnID  := io.rxRsp.bits.DBID
    cmReg.task.chi.nodeId := io.rxRsp.bits.SrcID
  }.elsewhen(respHit) {
    cmReg.task := 0.U.asTypeOf(cmReg.task)
  }

  // ctrl machine state
  cmReg.state := PriorityMux(Seq(
    allocHit -> SENDREQ,
    sReqHit  -> Mux(cmReg.task.chi.toLAN, WAITDBID, CANNEST), // TODO: has risk
    cNestHit -> WAITDBID,
    waitHit  -> DATATASK,
    taskHit  -> Mux(cmReg.task.chi.memAttr.ewa, WAITDATA, WAITCOMP),
    dataHit  -> RESP,
    compHit  -> RESP,
    respHit  -> FREE,
    true.B   -> cmReg.state
  ))

  // HardwareAssertion
  HardwareAssertion.withEn(cmReg.isFree,     allocHit)
  HardwareAssertion.withEn(cmReg.isSendReq,  sReqHit)
  HardwareAssertion.withEn(cmReg.isCanNest,  cNestHit)
  HardwareAssertion.withEn(cmReg.isWaitDBID, waitHit)
  HardwareAssertion.withEn(cmReg.isDataTask, taskHit)
  HardwareAssertion.withEn(cmReg.isWaitData, dataHit)
  HardwareAssertion.withEn(cmReg.isWaitComp, compHit)
  HardwareAssertion.withEn(cmReg.isResp,     respHit)
  HardwareAssertion.withEn(io.rxRsp.bits.Opcode === CompDBIDResp, waitHit & cmReg.task.chi.memAttr.ewa)
  HardwareAssertion.withEn(io.rxRsp.bits.Opcode === DBIDResp, waitHit & !cmReg.task.chi.memAttr.ewa)
  HardwareAssertion.checkTimeout(cmReg.isFree, TIMEOUT_WOA, cf"TIMEOUT: Write or Atomic State[${cmReg.state}]")
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Ctrl Machine ------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
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
   * Module declaration
   */
  val entries = Seq.fill(nrWriOrAtmCM) { Module(new WriOrAtmEntry()) }

  /*
   * For debug
   */
  val stateVec = Wire(Vec(nrWriOrAtmCM, UInt(State.width.W)))
  stateVec.zip(entries.map(_.io.state)).foreach { case(a, b) => a := b }
  dontTouch(stateVec)

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
  io.txReq    <> fastRRArb(entries.map(_.io.txReq)) // TODO: split to LAN and BBN
  io.updPosNestVec.zipWithIndex.foreach { case(upd, i) => upd <> fastRRArb(entries.map(_.io.updPosNestVec(i))) } // TODO: split to LAN and BBN
  io.respCmt  <> fastRRArb(entries.map(_.io.respCmt))
  io.respRepl <> fastRRArb(entries.map(_.io.respRepl))
  io.dataTask <> fastRRArb(entries.map(_.io.dataTask))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}