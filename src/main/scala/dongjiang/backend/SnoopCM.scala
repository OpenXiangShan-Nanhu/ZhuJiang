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
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object State {
  val width       = 5
  val FREE        = "b00001".U // 0x1
  val REQDB       = "b00010".U // 0x2
  val SENDSNP     = "b00100".U // 0x4
  val WAITRESP    = "b01000".U // 0x8
  val RESP        = "b10000".U // 0x10
}

class CMState(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> ReqDB --> SendSnp --> WaitResp0 --> WaitResp1 --> Resp --> Free
  val state       = UInt(State.width.W)
  val alrSendVec  = Vec(nrSfMetas, Bool())
  val getRespVec  = Vec(nrSfMetas, Bool())
  val getDataVec  = Vec(2, Bool())
  def getDataOne  = getDataVec.reduce(_ ^ _)
  def getDataAll  = getDataVec.reduce(_ & _)

  def isFree      = state(0)
  def isValid     = !isFree
  def isReqDB     = state(1)
  def isSendSnp   = state(2)
  def isWaitResp  = state(3)
  def isResp      = state(4)
}


class SnoopEntry(implicit p: Parameters) extends DJModule {
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
    // For Debug
    val state       = UInt(State.width.W)
  })


  /*
   * Reg and Wire declaration
   */
  val cmReg       = RegInit((new CMState with HasPackCMTask with HasPackTaskInst).Lit(_.state -> FREE))
  val nodeId_sSnp = Wire(new NodeId())
  val rspNodeId   = WireInit(0.U.asTypeOf(new NodeId()))
  val datNodeId   =  WireInit(0.U.asTypeOf(new NodeId()))
  io.state        := cmReg.state

  /*
   * Get metaId
   */
  rspNodeId.fromLAN := NocType.rxIs(io.rxRsp.bits, LAN)
  datNodeId.fromLAN := NocType.rxIs(io.rxDat.bits, LAN)
  rspNodeId.nodeId  := io.rxRsp.bits.SrcID
  datNodeId.nodeId  := io.rxDat.bits.SrcID
  val rspFire       = io.rxRsp.fire & (io.rxRsp.bits.Opcode === SnpResp     | io.rxRsp.bits.Opcode === SnpRespFwded)
  val datFire       = io.rxDat.fire & (io.rxDat.bits.Opcode === SnpRespData | io.rxDat.bits.Opcode === SnpRespDataFwded)
  val rspMetaId     = OHToUInt(rspNodeId.metaIdOH)
  val datMetaId     = OHToUInt(datNodeId.metaIdOH)
  HardwareAssertion.withEn(rspNodeId.metaIdOH.orR, rspFire)
  HardwareAssertion.withEn(datNodeId.metaIdOH.orR, datFire)

  /*
   * Receive Task IO
   */
  io.alloc.ready  := cmReg.isFree
  HardwareAssertion.withEn(io.alloc.bits.snpVec.asUInt.orR, io.alloc.valid)

  /*
   * Req DB
   */
  io.reqDB.valid  := cmReg.isReqDB
  io.reqDB.bits.llcTxnID  := cmReg.task.llcTxnID
  io.reqDB.bits.dataVec   := cmReg.task.chi.dataVec

  /*
   * Send Snp
   */
  val metaId_sSnp = OHToUInt(cmReg.task.snpVec.asUInt ^ cmReg.alrSendVec.asUInt)
  val snpIsFst    = cmReg.alrSendVec.asUInt === 0.U
  nodeId_sSnp.setSnpNodeId(metaId_sSnp)
  // valid
  io.txSnp.valid            := cmReg.isSendSnp
  // bits
  io.txSnp.bits             := DontCare
  io.txSnp.bits.RetToSrc    := Mux(snpIsFst, cmReg.task.chi.retToSrc, false.B)
  io.txSnp.bits.DoNotGoToSD := true.B
  io.txSnp.bits.Addr        := DontCare // remap in chi xbar
  io.txSnp.bits.Opcode      := Mux(snpIsFst, cmReg.task.chi.opcode, cmReg.task.chi.getNoFwdSnpOp)
  io.txSnp.bits.FwdTxnID    := cmReg.task.chi.txnID
  io.txSnp.bits.FwdNID      := cmReg.task.chi.nodeId
  io.txSnp.bits.TxnID       := cmReg.task.llcTxnID.get
  io.txSnp.bits.SrcID       := Mux(nodeId_sSnp.fromLAN, LAN.U, BBN.U)
  io.txSnp.bits.TgtID       := nodeId_sSnp.nodeId
  

  /*
   * Send Resp To Commit
   */
  // valid
  io.respCmt.valid  := cmReg.isResp & !cmReg.task.fromRepl
  io.respRepl.valid := cmReg.isResp &  cmReg.task.fromRepl
  // bits respCmt
  io.respCmt.bits.llcTxnID    := cmReg.task.llcTxnID
  io.respCmt.bits.inst        := cmReg.inst
  io.respCmt.bits.inst.valid  := true.B
  io.respCmt.bits.alr         := cmReg.task.alr
  // bits respCmt
  io.respRepl.bits.channel    := cmReg.inst.channel
  io.respRepl.bits.llcTxnID   := cmReg.task.llcTxnID
  io.respRepl.bits.resp       := cmReg.inst.resp


  /*
   * Modify Ctrl Machine Table
   */
  // Hit Message
  val allocHit    = io.alloc.fire
  val reqDBHit    = io.reqDB.fire
  val sendSnpHit  = io.txSnp.fire
  val recRespHit  = rspFire & cmReg.task.llcTxnID.get === io.rxRsp.bits.TxnID & cmReg.isValid
  val recDataHit  = datFire & cmReg.task.llcTxnID.get === io.rxDat.bits.TxnID & cmReg.isValid
  val respCmtHit  = io.respCmt.fire | io.respRepl.fire

  // Store Msg From Frontend
  val rspIsFwd = io.rxRsp.bits.Opcode === SnpRespFwded & recRespHit
  val datIsFwd = io.rxDat.bits.Opcode === SnpRespData  & recDataHit
  when(allocHit) {
    cmReg      := 0.U.asTypeOf(cmReg)
    cmReg.task := io.alloc.bits
  // Store AlrReqDB
  }.elsewhen(reqDBHit) {
    cmReg.task.alr.reqs := true.B
  // Store Message
  }.elsewhen(recDataHit | recRespHit) {
    // inst
    cmReg.inst.valid    := true.B
    cmReg.inst.fwdValid := cmReg.inst.fwdValid | rspIsFwd | datIsFwd
    cmReg.inst.fwdResp  := PriorityMux(Seq(
      rspIsFwd -> io.rxRsp.bits.FwdState,
      datIsFwd -> io.rxDat.bits.FwdState,
      true.B   -> cmReg.inst.fwdResp,
    ))
    cmReg.inst.channel := PriorityMux(Seq(
      recDataHit -> DAT,
      recRespHit -> Mux(cmReg.inst.channel === DAT, DAT, RSP),
      true.B     -> cmReg.inst.channel,
    ))
    cmReg.inst.opcode := PriorityMux(Seq(
      recDataHit -> io.rxDat.bits.Opcode,
      recRespHit -> Mux(cmReg.inst.channel === DAT, cmReg.inst.opcode, io.rxRsp.bits.Opcode),
      true.B     -> cmReg.inst.opcode,
    ))
    cmReg.inst.resp := PriorityMux(Seq(
      recDataHit -> io.rxDat.bits.Resp,
      recRespHit -> Mux(cmReg.inst.channel === DAT, cmReg.inst.resp, io.rxRsp.bits.Resp),
      true.B     -> cmReg.inst.resp,
    ))
  // Release
  }.elsewhen(respCmtHit) {
    cmReg := 0.U.asTypeOf(cmReg)
  }

  // Release
  when(respCmtHit) {
    cmReg.alrSendVec := 0.U.asTypeOf(cmReg.alrSendVec)
    cmReg.getRespVec := 0.U.asTypeOf(cmReg.getRespVec)
    cmReg.getDataVec := 0.U.asTypeOf(cmReg.getDataVec)
  // Update
  }.otherwise {
    // alrSendVec
    cmReg.alrSendVec(metaId_sSnp) := cmReg.alrSendVec(metaId_sSnp) | sendSnpHit
    HardwareAssertion.withEn(!cmReg.alrSendVec(metaId_sSnp), sendSnpHit)
    // getDataVec
    val beatId = Mux(io.rxDat.bits.DataID === "b10".U, 1.U, 0.U)
    cmReg.getDataVec(beatId) := cmReg.getDataVec(beatId) | recDataHit
    HardwareAssertion.withEn(!cmReg.getDataVec(beatId), recDataHit)
    // getRespVec
    cmReg.getRespVec.zipWithIndex.foreach { case(get, i) =>
      val rspHit = recRespHit & rspMetaId === i.U
      val datHit = recDataHit & datMetaId === i.U
      get := get | rspHit | datHit
      HardwareAssertion(PopCount(Seq(rspHit, datHit)) <= 1.U)
      HardwareAssertion.withEn(!rspHit, get)
    }
  }


  // Check send or wait
  val alrSnpAll     = PopCount(cmReg.task.snpVec.asUInt ^ cmReg.alrSendVec.asUInt) === 1.U & sendSnpHit
  val alrGetRspAll  = PopCount(cmReg.task.snpVec.asUInt ^ cmReg.getRespVec.asUInt) === 0.U
  val alrGetDatAll  = !cmReg.getDataOne
  val alrGetAll     = alrGetRspAll & alrGetDatAll & cmReg.isWaitResp

  // Get Next State
  cmReg.state   := PriorityMux(Seq(
    allocHit    -> Mux(io.alloc.bits.needReqDB, REQDB, SENDSNP),
    reqDBHit    -> SENDSNP,
    alrSnpAll   -> WAITRESP,
    alrGetAll   -> RESP,
    respCmtHit  -> FREE,
    true.B      -> cmReg.state,
  ))

  // HardwareAssertion
  HardwareAssertion.withEn(cmReg.isFree,    allocHit)
  HardwareAssertion.withEn(cmReg.isReqDB,   reqDBHit)
  HardwareAssertion.withEn(cmReg.isSendSnp, sendSnpHit)
  HardwareAssertion.withEn(cmReg.isSendSnp | cmReg.isWaitResp, recRespHit)
  HardwareAssertion.withEn(cmReg.isSendSnp | cmReg.isWaitResp, recDataHit)
  HardwareAssertion.withEn(cmReg.isResp,    respCmtHit)
  HardwareAssertion.checkTimeout(cmReg.isFree, TIMEOUT_SNP, cf"TIMEOUT: Snoop State[${cmReg.state}]")
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Ctrl Machine ------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
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
   * Module declaration
   */
  val entries = Seq.fill(nrSnoopCM) { Module(new SnoopEntry()) }

  /*
   * For debug
   */
  val stateVec = Wire(Vec(nrSnoopCM, UInt(State.width.W)))
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
  io.txSnp    <> fastRRArb(entries.map(_.io.txSnp)) // TODO: split to LAN and BBN
  io.respCmt  <> fastRRArb(entries.map(_.io.respCmt))
  io.respRepl <> fastRRArb(entries.map(_.io.respRepl))
  io.reqDB    <> fastRRArb(entries.map(_.io.reqDB))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}