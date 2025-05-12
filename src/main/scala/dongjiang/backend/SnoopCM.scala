package dongjiang.backend

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
import dongjiang.backend.SNPSTARE._
import dongjiang.data.DataTask
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object SNPSTARE {
  val width       = 5
  val FREE        = "b00001".U // 0x1
  val REQDB       = "b00010".U // 0x2
  val SENDSNP     = "b00100".U // 0x4
  val WAITRESP    = "b01000".U // 0x8
  val RESPCMT     = "b10000".U // 0x10
}

class SnpMes(implicit p: Parameters) extends DJBundle {
  // CHI: Free --> ReqDB --> SendSnp --> WaitResp --> RespCmt --> Free
  val state       = UInt(SNPSTARE.width.W)
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
    // Task
    val alloc       = Flipped(Decoupled(new CMTask))
    val resp        = Decoupled(new CMResp)
    // CHI
    val txSnp       = Decoupled(new SnoopFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Req To Data
    val reqDB       = Decoupled(new HnTxnID with HasDataVec)
    // For Debug
    val dbg         = Valid(new ReadState with HasHnTxnID)
  })


  /*
   * Reg and Wire declaration
   */
  val reg         = RegInit((new SnpMes with HasPackCMTask with HasPackTaskInst).Lit(_.state -> FREE))
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
  io.alloc.ready  := reg.isFree
  HardwareAssertion.withEn(io.alloc.bits.snpVec.asUInt.orR, io.alloc.valid)

  /*
   * ReqDB
   */
  io.reqDB.valid        := reg.isReqDB
  io.reqDB.bits.hnTxnID := reg.task.hnTxnID
  io.reqDB.bits.dataVec := reg.task.chi.dataVec
  HAssert.withEn(io.reqDB.bits.dataVec === DataVec.Full, io.reqDB.valid )

  /*
   * Send Snoop
   */
  val snpMetaId   = PriorityEncoder(reg.task.snpVec.asUInt ^ reg.alrSendVec.asUInt)
  val snpIsFst    = reg.alrSendVec.asUInt === 0.U
  snpNodeId.setSnpNodeId(snpMetaId)
  // valid
  io.txSnp.valid            := reg.isSendSnp
  // bits
  io.txSnp.bits             := DontCare
  io.txSnp.bits.RetToSrc    := Mux(snpIsFst, reg.task.chi.retToSrc, false.B)
  io.txSnp.bits.DoNotGoToSD := true.B
  io.txSnp.bits.Addr        := DontCare // remap in chi xbar
  io.txSnp.bits.Opcode      := Mux(snpIsFst, reg.task.chi.opcode, reg.task.chi.getNoFwdSnpOp)
  io.txSnp.bits.FwdTxnID    := reg.task.chi.txnID
  io.txSnp.bits.FwdNID      := reg.task.chi.nodeId
  io.txSnp.bits.TxnID       := reg.task.hnTxnID
  io.txSnp.bits.SrcID       := Mux(snpNodeId.fromLAN, LAN.U, BBN.U)
  io.txSnp.bits.TgtID       := snpNodeId.nodeId
  

  /*
   * Send Resp To Commit
   */
  // valid
  io.resp.valid         := reg.isResp
  // bits
  io.resp.bits.hnTxnID  := reg.task.hnTxnID
  io.resp.bits.taskInst := reg.taskInst
  io.resp.bits.alr      := reg.task.alr
  io.resp.bits.fromRec  := false.B
  io.resp.bits.toRepl   := reg.task.fromRepl

  /*
   * Modify Ctrl Machine Table
   */
  val rspIsFwd              = io.rxRsp.bits.Opcode === SnpRespFwded     & rspHit
  val datIsFwd              = io.rxDat.bits.Opcode === SnpRespDataFwded & datHit
  // Store message from Frontend
  when(io.alloc.fire) {
    next.task               := io.alloc.bits
    next.taskInst           := 0.U.asTypeOf(new TaskInst)
  // Store already ReqDB
  }.elsewhen(io.reqDB.fire) {
    next.task.alr.reqs      := true.B
    HAssert(!reg.task.alr.reqs)
  // Store message from CHI
  }.elsewhen(rspHit | datHit) {
    // inst
    next.taskInst.valid     := true.B
    next.taskInst.fwdValid  := reg.taskInst.fwdValid | rspIsFwd | datIsFwd
    next.taskInst.fwdResp   := PriorityMux(Seq(
      rspIsFwd -> io.rxRsp.bits.FwdState,
      datIsFwd -> io.rxDat.bits.FwdState,
      true.B   -> reg.taskInst.fwdResp,
    ))
    next.taskInst.channel   := PriorityMux(Seq(
      datHit -> DAT,
      rspHit -> Mux(reg.taskInst.channel === DAT, DAT, RSP),
      true.B -> reg.taskInst.channel,
    ))
    next.taskInst.opcode    := PriorityMux(Seq(
      datHit -> io.rxDat.bits.Opcode,
      rspHit -> Mux(reg.taskInst.channel === DAT, reg.taskInst.opcode, io.rxRsp.bits.Opcode),
      true.B -> reg.taskInst.opcode,
    ))
    next.taskInst.resp      := PriorityMux(Seq(
      datHit -> io.rxDat.bits.Resp,
      rspHit -> Mux(reg.taskInst.channel === DAT, reg.taskInst.resp, io.rxRsp.bits.Resp),
      true.B -> reg.taskInst.resp,
    ))
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
    val beatId = Mux(io.rxDat.bits.DataID === "b10".U, 1.U, 0.U)
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
  val alrSnpAll     = PopCount(reg.task.snpVec.asUInt ^ reg.alrSendVec.asUInt) === 1.U & io.txSnp.fire
  val alrGetRspAll  = PopCount(reg.task.snpVec.asUInt ^ reg.getRespVec.asUInt) === 0.U
  val alrGetDatAll  = !reg.getDataOne
  val alrGetAll     = alrGetRspAll & alrGetDatAll

  // Get Next State
  switch(reg.state) {
    is(FREE) {
      when(io.alloc.fire) { next.state := Mux(io.alloc.bits.needReqDB, REQDB, SENDSNP) }
    }
    is(REQDB) {
      when(io.reqDB.fire) { next.state := SENDSNP }
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
  HAssert.withEn(reg.isFree,    io.alloc.fire)
  HAssert.withEn(reg.isReqDB,   reg.isValid & io.reqDB.fire)
  HAssert.withEn(reg.isSendSnp, reg.isValid & io.txSnp.fire)
  HAssert.withEn(reg.isResp,    reg.isValid & io.resp.fire)
  HAssert.withEn(reg.isSendSnp | reg.isWaitResp, rspHit | datHit)

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
    // Req To Data
    val reqDB       = Decoupled(new HnTxnID with HasDataVec)
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
  io.txSnp    <> fastRRArb(entries.map(_.io.txSnp)) // TODO: split to LAN and BBN
  io.resp     <> fastRRArb(entries.map(_.io.resp))
  io.reqDB    <> fastRRArb(entries.map(_.io.reqDB))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}