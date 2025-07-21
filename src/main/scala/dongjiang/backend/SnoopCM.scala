package dongjiang.backend

import chisel3._
import chisel3.experimental.BundleLiterals._
import chisel3.util._
import dongjiang._
import dongjiang.backend.SnpState._
import dongjiang.bundle.ChiChannel._
import dongjiang.bundle._
import dongjiang.frontend.decode._
import dongjiang.utils._
import org.chipsalliance.cde.config._
import xs.utils.debug._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.SnpOpcode._
import zhujiang.chi._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object SnpState {
  val width       = 3
  val FREE        = 0x0.U
  val PRESNP      = 0x1.U // Pre-Snoop using for the SnpUniqueFwd snoop is only permitted if the cacheline is cached at a single RN-F
  val SENDSNP     = 0x2.U
  val WAITRESP    = 0x3.U
  val RESPCMT     = 0x4.U
}

class SnpMes(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> PreSnp --> SendSnp --> WaitResp --> RespCmt --> Free
  val state       = UInt(SnpState.width.W)
  val alrSendVec  = Vec(nrSfMetas, Bool())
  val getRespVec  = Vec(nrSfMetas, Bool())
  val getDataVec  = Vec(djparam.nrBeat, Bool())
  def getDataOne  = getDataVec.reduce(_ ^ _)
  def getDataAll  = getDataVec.reduce(_ & _)

  def isFree      = state === FREE
  def isValid     = !isFree
  def isPreSnp    = state === PRESNP
  def isSendSnp   = state === SENDSNP
  def isWaitResp  = state === WAITRESP
  def isResp      = state === RESPCMT
}

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- Ctrl Machine Entry ---------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class SnoopEntry(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Task
    val alloc       = Flipped(Decoupled(new CMTask))
    val resp        = Decoupled(new CMResp)
    // CHI
    val txSnp       = Decoupled(new SnoopFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // For Debug
    val dbg         = Valid(new ReadState with HasHnTxnID) // TODO: dont use ReadState
  })


  /*
   * Reg and Wire declaration
   */
  val reg         = RegInit((new SnpMes with HasPackCMTask with HasPackTaskInst with HasRespErr).Lit(_.state -> FREE))
  val next        = WireInit(reg)
  val snpNodeId   = Wire(new NodeId())
  val rspNodeId   = WireInit(0.U.asTypeOf(new NodeId()))
  val datNodeId   =  WireInit(0.U.asTypeOf(new NodeId()))

  /*
   * Output for debug
   */
  io.dbg.valid        := reg.isValid
  io.dbg.bits.state   := reg.state
  io.dbg.bits.hnTxnID := reg.task.hnTxnID

  /*
   * Get metaId
   */
  rspNodeId.fromLAN := NocType.rxIs(io.rxRsp.bits, LAN)
  datNodeId.fromLAN := NocType.rxIs(io.rxDat.bits, LAN)
  rspNodeId.nodeId  := io.rxRsp.bits.SrcID
  datNodeId.nodeId  := io.rxDat.bits.SrcID
  val rspHit        = reg.isValid & io.rxRsp.fire & reg.task.hnTxnID === io.rxRsp.bits.TxnID & (io.rxRsp.bits.Opcode === SnpResp     | io.rxRsp.bits.Opcode === SnpRespFwded)
  val datHit        = reg.isValid & io.rxDat.fire & reg.task.hnTxnID === io.rxDat.bits.TxnID & (io.rxDat.bits.Opcode === SnpRespData | io.rxDat.bits.Opcode === SnpRespDataFwded)
  val rspMetaId     = OHToUInt(rspNodeId.metaIdOH)
  val datMetaId     = OHToUInt(datNodeId.metaIdOH)
  HardwareAssertion.withEn(rspNodeId.metaIdOH.orR, rspHit)
  HardwareAssertion.withEn(datNodeId.metaIdOH.orR, datHit)

  /*
   * Receive Task
   */
  io.alloc.ready    := reg.isFree
  HardwareAssertion.withEn(io.alloc.bits.snpVec.asUInt.orR, io.alloc.valid)

  /*
   * Send Snoop
   */
  val snpMetaId   = PriorityEncoder(reg.task.snpVec.asUInt ^ reg.alrSendVec.asUInt)
  val snpIsLast   = PopCount(reg.task.snpVec.asUInt ^ reg.alrSendVec.asUInt) === 1.U
  snpNodeId.setSnpNodeId(snpMetaId)
  // valid
  io.txSnp.valid            := reg.isPreSnp | reg.isSendSnp
  // bits
  io.txSnp.bits             := DontCare
  io.txSnp.bits.RetToSrc    := snpIsLast & reg.task.chi.retToSrc
  io.txSnp.bits.DoNotGoToSD := true.B
  io.txSnp.bits.Addr        := DontCare
  io.txSnp.bits.Opcode      := Mux(reg.isPreSnp, SnpMakeInvalid, reg.task.chi.opcode)
  io.txSnp.bits.FwdTxnID    := reg.task.chi.txnID
  io.txSnp.bits.FwdNID      := reg.task.chi.nodeId
  io.txSnp.bits.TxnID       := reg.task.hnTxnID
  io.txSnp.bits.SrcID       := Mux(snpNodeId.fromLAN, LAN.U, BBN.U)
  io.txSnp.bits.TgtID       := snpNodeId.nodeId
  io.txSnp.bits.QoS         := reg.task.qos
  HAssert.withEn(reg.task.chi.snpIs(SnpUniqueFwd), reg.isValid & reg.task.chi.isSnpFwd & PopCount(reg.task.snpVec.asUInt) > 1.U)
  HAssert.withEn(io.txSnp.bits.RetToSrc === false.B, 
                (io.txSnp.bits.Opcode === SnpCleanShared | io.txSnp.bits.Opcode === SnpCleanInvalid | io.txSnp.bits.Opcode === SnpMakeInvalid |
                 io.txSnp.bits.Opcode === SnpOnceFwd     | io.txSnp.bits.Opcode === SnpUniqueFwd)   & io.txSnp.valid)
  

  /*
   * Send Resp To Commit
   */
  // valid
  io.resp.valid             := reg.isResp
  // bits
  io.resp.bits.hnTxnID      := reg.task.hnTxnID
  io.resp.bits.taskInst     := reg.taskInst
  io.resp.bits.toRepl       := reg.task.fromRepl
  io.resp.bits.qos          := reg.task.qos
  io.resp.bits.respErr      := reg.respErr

  /*
   * Modify Ctrl Machine Table
   */
  val rspIsFwd              = io.rxRsp.bits.Opcode === SnpRespFwded     & rspHit
  val datIsFwd              = io.rxDat.bits.Opcode === SnpRespDataFwded & datHit
  // Store message from Frontend
  when(io.alloc.fire) {
    next.task               := io.alloc.bits
    next.taskInst           := 0.U.asTypeOf(new TaskInst)
    next.respErr            := RespErr.NormalOkay
  // Store message from CHI
  }.elsewhen(rspHit | datHit) {
    // inst valid
    next.taskInst.valid     := true.B
    // inst fwd
    next.taskInst.fwdValid  := reg.taskInst.fwdValid | rspIsFwd | datIsFwd
    next.taskInst.fwdResp   := PriorityMux(Seq(
      rspIsFwd -> io.rxRsp.bits.FwdState,
      datIsFwd -> io.rxDat.bits.FwdState,
      true.B   -> reg.taskInst.fwdResp,
    ))
    // inst channel
    next.taskInst.channel   := PriorityMux(Seq(
      datHit -> DAT,
      rspHit -> Mux(reg.taskInst.channel === DAT, DAT, RSP),
      true.B -> reg.taskInst.channel,
    ))
    // inst opcode
    next.taskInst.opcode    := PriorityMux(Seq(
      datHit -> io.rxDat.bits.Opcode,
      rspHit -> Mux(reg.taskInst.channel === DAT | reg.taskInst.fwdValid, reg.taskInst.opcode, io.rxRsp.bits.Opcode),
      true.B -> reg.taskInst.opcode,
    ))
    // inst resp
    next.taskInst.resp      := PriorityMux(Seq(
      datHit -> io.rxDat.bits.Resp,
      rspHit -> Mux(reg.taskInst.channel === DAT | reg.taskInst.resp >= io.rxRsp.bits.Resp, reg.taskInst.resp, io.rxRsp.bits.Resp),
      true.B -> reg.taskInst.resp,
    ))
    HAssert.withEn(next.taskInst.resp =/= ChiResp.SD & next.taskInst.resp =/= ChiResp.SD_PD, rspHit | datHit)
    // resp error
    when(!reg.isERR) {
      next.respErr          := Mux(datHit, io.rxDat.bits.RespErr, io.rxRsp.bits.RespErr)
    }
  }

  // Release or alloc
  when(io.alloc.fire){
    next.alrSendVec := 0.U.asTypeOf(reg.alrSendVec)
    next.getRespVec := 0.U.asTypeOf(reg.getRespVec)
    next.getDataVec := 0.U.asTypeOf(reg.getDataVec)
  // Update
  }.otherwise {
    // alrSendVec
    next.alrSendVec(snpMetaId) := reg.alrSendVec(snpMetaId) | io.txSnp.fire
    HardwareAssertion.withEn(!reg.alrSendVec(snpMetaId), io.txSnp.fire)
    // getDataVec
    val beatId = io.rxDat.valid & Mux(io.rxDat.bits.DataID === "b10".U, 1.U, 0.U)
    next.getDataVec(beatId) := reg.getDataVec(beatId) | datHit
    HardwareAssertion.withEn(!reg.getDataVec(beatId), datHit)
    // getRespVec
    next.getRespVec.zipWithIndex.foreach { case(get, i) =>
      val getRspHit = rspHit & rspMetaId === i.U
      val getDatHit = datHit & datMetaId === i.U
      get := reg.getRespVec(i) | getRspHit | getDatHit
      HardwareAssertion(PopCount(Seq(getRspHit, getDatHit)) <= 1.U)
      HardwareAssertion.withEn(!getRspHit, reg.getRespVec(i))
    }
  }


  // Check send or wait
  val snpLastNext   = PopCount(reg.task.snpVec.asUInt ^ reg.alrSendVec.asUInt) === 2.U & io.txSnp.fire
  val alrSnpAll     = PopCount(reg.task.snpVec.asUInt ^ reg.alrSendVec.asUInt) === 1.U & io.txSnp.fire
  val alrGetRspAll  = PopCount(reg.task.snpVec.asUInt ^ reg.getRespVec.asUInt) === 0.U
  val alrGetDatAll  = !reg.getDataOne
  val alrGetAll     = alrGetRspAll & alrGetDatAll

  // Get Next State
  val snpUnqiueFwdMoreThan1 = io.alloc.bits.chi.snpIs(SnpUniqueFwd) & PopCount(io.alloc.bits.snpVec) > 1.U
  switch(reg.state) {
    is(FREE) {
      when(io.alloc.fire) { next.state := Mux(snpUnqiueFwdMoreThan1, PRESNP, SENDSNP) }
    }
    is(PRESNP) {
      when(snpLastNext)   { next.state := SENDSNP }
    }
    is(SENDSNP) {
      when(alrSnpAll)     { next.state := WAITRESP }
    }
    is(WAITRESP) {
      when(alrGetAll)     { next.state := RESPCMT }
    }
    is(RESPCMT) {
      when(io.resp.fire)  { next.state := FREE }
    }
  }

  /*
   * HAssert
   */
  HAssert.withEn(reg.isFree, io.alloc.fire)
  HAssert.withEn(reg.isResp, reg.isValid & io.resp.fire)
  HAssert.withEn(reg.isPreSnp | reg.isSendSnp, reg.isValid & io.txSnp.fire)
  HAssert.withEn(reg.isPreSnp | reg.isSendSnp | reg.isWaitResp, rspHit | datHit)

  /*
   * Set new task
   */
  val set = io.alloc.fire | reg.isValid; dontTouch(set)
  when(set) { reg := next }

  // HardwareAssertion
  HardwareAssertion.checkTimeout(reg.isFree, TIMEOUT_SNP, cf"TIMEOUT: Snoop State[${reg.state}]")
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
    // Task
    val alloc       = Flipped(Decoupled(new CMTask))
    val resp        = Decoupled(new CMResp)
    // CHI
    val txSnp       = Decoupled(new SnoopFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
  })

  /*
   * Module declaration
   */
  val entries = Seq.fill(nrSnoopCM) { Module(new SnoopEntry()) } // TODO: reserve for toLAN
  val dbgVec  = VecInit(entries.map(_.io.dbg))
  dontTouch(dbgVec)

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
  io.txSnp    <> fastQosRRArb(entries.map(_.io.txSnp)) // TODO: split to LAN and BBN
  io.resp     <> fastQosRRArb(entries.map(_.io.resp))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}