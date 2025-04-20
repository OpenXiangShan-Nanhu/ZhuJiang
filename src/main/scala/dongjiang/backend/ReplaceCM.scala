package dongjiang.backend.replace

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle.{HasAddr, HasPackLLCTxnID, PackChi, _}
import dongjiang.data.HasAlready
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, HasPackDirMsg}
import dongjiang.frontend.decode.{HasPackCmtCode, _}
import dongjiang.data._
import zhujiang.chi.ReqOpcode._
import dongjiang.frontend._
import dongjiang.backend._
import dongjiang.backend.replace.State._

object State {
  val width   = 6
  val FREE    = "b000000".U
  val WRIDIR  = "b000001".U
  val WAITDIR = "b000010".U
  val REPLSF  = "b000100".U
  val REPLLLC = "b001000".U
  val WAITCM  = "b010000".U
  val RESP    = "b100000".U
}

class CMState(implicit p: Parameters) extends DJBundle {
  // FREE --> WRIDIR --> WAITDIR --> REPL(SF) --> WAITCM --> WRITE --> WAITDIR --> RESP(LLC) --> WAITCM --> RESP
  //                  ^           ^                                 ^           ^
  //               lockSet    unLockSet                          lockSet    unLockSet
  val state     = UInt(State.width.W)

  def isValid   = state.orR
  def isFree    = !isValid
  def isWriDir  = state(0)
  def isWaitDir = state(1)
  def isReplSF  = state(2)
  def isReplLLC = state(3)
  def isWaitCM  = state(4)
  def isResp    = state(5)
}



class ReplaceCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config          = new DJConfigIO()
    // Commit Task In
    val alloc           = Flipped(Decoupled(new ReplTask))
    val resp            = Valid(new LLCTxnID())
    // Send Task To CM
    val cmAllocVec      = Vec(2, Decoupled(new CMTask)) // SNP and WOA
    // Update PoS Message
    val updPosTag       = Valid(new LLCTxnID with HasAddr)
    val lockPosSet      = Valid(new LLCTxnID)
    val unlockPosSet    = Valid(new LLCTxnID)
    // Write Directory
    val writeDir        = new DJBundle {
      val llc           = Decoupled(new DirEntry("llc") with HasPackPosIndex)
      val sf            = Decoupled(new DirEntry("sf")  with HasPackPosIndex)
    }
    // Write Directory Resp
    val respDir         = new DJBundle {
      val llc           = Flipped(Valid(new DirEntry("llc")))
      val sf            = Flipped(Valid(new DirEntry("sf")))
    }
    // Resp From TaskCM
    val respRepl        = Flipped(Valid(new PackLLCTxnID with HasChiChannel with HasChiResp)) // Channel and Resp only use in SnpResp

  })

  /*
   * Reg and Wire declaration
   */
  val cmRegVec  = RegInit(VecInit(Seq.fill(nrReplaceCM) { 0.U.asTypeOf(new CMState()) }))
  val msgRegVec = Reg(Vec(nrReplaceCM, new ReplTask()))
  val replDsVec = Reg(Vec(nrReplaceCM, new DsIdx()))
  val waitPipe  = Module(new Pipe(UInt(log2Ceil(nrReplaceCM).W), readDirLatency))


  /*
   * [REC] Receive Task IO
   */
  val cmVec_rec   = cmRegVec.map(_.isFree) // Free Vec
  val cmId_rec    = PriorityEncoder(cmVec_rec)
  io.alloc.ready  := cmVec_rec.reduce(_ | _)
  HardwareAssertion.withEn(io.alloc.bits.alr.reqs, io.alloc.valid & io.alloc.bits.replLLC)

  /*
   * [WRITE]
   */
  val cmVec_wri = cmRegVec.map(_.isWriDir)
  val cmId_wri  = RREncoder(cmVec_wri)
  val msg_wri   = msgRegVec(cmId_wri)
  // llc
  io.writeDir.llc.valid         := cmVec_wri.reduce(_ | _) & msg_wri.wriLLC & (!msg_wri.wriSF | io.writeDir.sf.ready)
  io.writeDir.llc.bits.addr     := msg_wri.llcTxnID.getAddr // remap in DongJiang
  io.writeDir.llc.bits.wayOH    := msg_wri.dir.llc.wayOH
  io.writeDir.llc.bits.hit      := msg_wri.dir.llc.hit
  io.writeDir.llc.bits.metaVec  := msg_wri.dir.llc.metaVec
  io.writeDir.llc.bits.pos      := msg_wri.llcTxnID.pos
  // sf
  io.writeDir.sf.valid          := cmVec_wri.reduce(_ | _) & msg_wri.wriSF & (!msg_wri.wriLLC | io.writeDir.llc.ready)
  io.writeDir.sf.bits.addr      := msg_wri.llcTxnID.getAddr // remap in DongJiang
  io.writeDir.sf.bits.wayOH     := msg_wri.dir.sf.wayOH
  io.writeDir.sf.bits.hit       := msg_wri.dir.sf.hit
  io.writeDir.sf.bits.metaVec   := msg_wri.dir.sf.metaVec
  io.writeDir.sf.bits.pos       := msg_wri.llcTxnID.pos
  // waitPipe
  waitPipe.io.enq.valid := (io.writeDir.llc.fire | io.writeDir.sf.fire) & msg_wri.replDIR
  waitPipe.io.enq.bits  := cmId_wri

  /*
   * Update PoS Tag and Lock
   */
  val cm_get      = waitPipe.io.deq.valid
  val cmId_get    = waitPipe.io.deq.bits
  val msg_get     = msgRegVec(cmId_get)
  val addr_get    = Mux(io.respDir.sf.valid, io.respDir.sf.bits.addr, io.respDir.llc.bits.addr)
  val needReplSF  = io.respDir.sf.valid & io.respDir.sf.bits.metaVec.map(_.isValid).reduce(_ | _)
  val needReplLLC = io.respDir.llc.valid & io.respDir.llc.bits.metaVec.head.isValid
  HardwareAssertion.withEn(io.respDir.sf.valid ^ io.respDir.llc.valid, cm_get)
  HardwareAssertion.withEn(cm_get, io.respDir.sf.valid | io.respDir.llc.valid)
  // updPosTag when resp need repl
  io.updPosTag.valid        := cm_get & (needReplSF | needReplLLC)
  io.updPosTag.bits.dirBank := msg_get.llcTxnID.dirBank
  io.updPosTag.bits.pos     := msg_get.llcTxnID.pos
  io.updPosTag.bits.addr    := addr_get
  // save repl ds
  when(needReplLLC) {
    replDsVec(cmId_get).bank  := getDS(io.respDir.llc.bits.addr, io.respDir.llc.bits.way)._1
    replDsVec(cmId_get).idx   := getDS(io.respDir.llc.bits.addr, io.respDir.llc.bits.way)._2
  }
  // lock when write
  io.lockPosSet.valid       := waitPipe.io.enq.fire
  io.lockPosSet.bits        := msg_wri.llcTxnID
  // unlock when get resp
  io.unlockPosSet.valid     := waitPipe.io.deq.fire
  io.unlockPosSet.bits      := msg_get.llcTxnID


  /*
   * [REPLSF]
   */
  val cmVec_rpSF  = cmRegVec.map(_.isReplSF)
  val cmId_rpSF   = RREncoder(cmVec_rpSF)
  val msg_rpSF    = msgRegVec(cmId_rpSF)
  // SNP
  io.cmAllocVec(CMID.SNP).valid             := cmVec_rpSF.reduce(_ | _)
  io.cmAllocVec(CMID.SNP).bits              := DontCare
  io.cmAllocVec(CMID.SNP).bits.chi.nodeId   := DontCare
  io.cmAllocVec(CMID.SNP).bits.chi.channel  := ChiChannel.SNP
  io.cmAllocVec(CMID.SNP).bits.chi.opcode   := SnpOpcode.SnpUnique
  io.cmAllocVec(CMID.SNP).bits.chi.dataVec  := Seq(true.B, true.B)
  io.cmAllocVec(CMID.SNP).bits.chi.retToSrc := true.B
  io.cmAllocVec(CMID.SNP).bits.chi.toLAN    := DontCare
  io.cmAllocVec(CMID.SNP).bits.llcTxnID     := msg_rpSF.llcTxnID
  io.cmAllocVec(CMID.SNP).bits.alr          := msg_rpSF.alr
  io.cmAllocVec(CMID.SNP).bits.snpVec       := msg_rpSF.dir.sf.metaVec.map(_.state.asBool)
  io.cmAllocVec(CMID.SNP).bits.fromRepl     := true.B
  io.cmAllocVec(CMID.SNP).bits.alr          := msg_rpSF.alr
  io.cmAllocVec(CMID.SNP).bits.dataOp.reqs  := true.B

  /*
   * [REPLLLC]
   */
  val cmVec_rpLLC = cmRegVec.map(_.isReplLLC)
  val cmId_rpLLC  = RREncoder(cmVec_rpLLC)
  val msg_rpLLC   = msgRegVec(cmId_rpLLC)
  // REQ
  io.cmAllocVec(CMID.WOA).valid             := cmVec_rpLLC.reduce(_ | _)
  io.cmAllocVec(CMID.WOA).bits              := DontCare
  io.cmAllocVec(CMID.WOA).bits.chi.nodeId   := DontCare
  io.cmAllocVec(CMID.WOA).bits.chi.channel  := ChiChannel.REQ
  io.cmAllocVec(CMID.WOA).bits.chi.opcode   := DontCare // will remap in WOA
  io.cmAllocVec(CMID.WOA).bits.chi.dataVec  := Seq(true.B, true.B)
  io.cmAllocVec(CMID.WOA).bits.chi.memAttr.allocate   := false.B
  io.cmAllocVec(CMID.WOA).bits.chi.memAttr.device     := false.B
  io.cmAllocVec(CMID.WOA).bits.chi.memAttr.cacheable  := true.B
  io.cmAllocVec(CMID.WOA).bits.chi.memAttr.ewa        := true.B
  io.cmAllocVec(CMID.WOA).bits.chi.toLAN    := DontCare
  io.cmAllocVec(CMID.WOA).bits.fromRepl     := true.B
  io.cmAllocVec(CMID.WOA).bits.ds           := replDsVec(cmId_rpLLC)
  io.cmAllocVec(CMID.WOA).bits.cbResp       := msg_rpLLC.dir.llc.metaVec.head.cbResp
  io.cmAllocVec(CMID.WOA).bits.alr          := msg_rpLLC.alr
  io.cmAllocVec(CMID.WOA).bits.dataOp.reqs  := false.B
  io.cmAllocVec(CMID.WOA).bits.dataOp.repl  := true.B
  io.cmAllocVec(CMID.WOA).bits.dataOp.read  := true.B
  HardwareAssertion.withEn(msg_rpLLC.alr.reqs, cmVec_rpLLC.reduce(_ | _))

  /*
   * [RESP]
   */
  val cmVec_resp        = cmRegVec.map(_.isResp)
  val cmId_resp         = RREncoder(cmVec_resp)
  val msg_resp          = msgRegVec(cmId_resp)
  io.resp.valid         := cmVec_resp.reduce(_ | _)
  io.resp.bits.dirBank  := msg_resp.llcTxnID.dirBank
  io.resp.bits.pos      := msg_resp.llcTxnID.pos


  /*
   * Modify Ctrl Machine Table
   */
  cmRegVec.zip(msgRegVec).zipWithIndex.foreach {
    case ((cm, msg), i) =>
      val allocHit    = io.alloc.fire & cmId_rec === i.U
      val writeHit    = (io.writeDir.llc.fire | io.writeDir.sf.fire) & cmId_wri === i.U
      val waitHit     = waitPipe.io.deq.valid & waitPipe.io.deq.bits === i.U
      val replSFHit   = io.cmAllocVec(CMID.SNP).fire & cmId_rpSF === i.U
      val replLLCHit  = io.cmAllocVec(CMID.WOA).fire & cmId_rpLLC === i.U
      val cmRespHit   = io.respRepl.fire & io.respRepl.bits.llcTxnID.get === msg.llcTxnID.get
      val respHit     = io.resp.fire & cmId_resp === i.U

      // Message
      val needSecRepl = cmRespHit & !io.respRepl.bits.isValid
      HardwareAssertion.withEn(msg.replSF, needSecRepl)
      HardwareAssertion.withEn(io.respRepl.bits.isDat, needSecRepl)
      when(allocHit) {
        msg := io.alloc.bits
      }.elsewhen(needSecRepl) {
        msg.wriLLC        := true.B
        msg.dir.llc.wayOH := DontCare
        msg.dir.llc.hit   := false.B
        msg.dir.llc.metaVec.head.state := Mux(io.respRepl.bits.passDirty, ChiState.UD, ChiState.SC)
      }.elsewhen(respHit) {
        msg := 0.U.asTypeOf(msg)
      }

      // Ctrl Machine State
      cm.state := PriorityMux(Seq(
        allocHit    -> WRIDIR,
        writeHit    -> Mux(msg.replDIR, WAITDIR, RESP),
        waitHit     -> Mux(needReplSF, REPLSF, Mux(needReplLLC, REPLLLC, RESP)),
        replSFHit   -> WAITCM,
        replLLCHit  -> WAITCM,
        cmRespHit   -> Mux(needSecRepl, REPLLLC, RESP),
        respHit     -> FREE,
        true.B      -> cm.state
      ))


      HardwareAssertion.withEn(cm.isFree,     allocHit, cf"Replace Index[${i}]")
      HardwareAssertion.withEn(cm.isWriDir,   writeHit, cf"Replace Index[${i}]")
      HardwareAssertion.withEn(cm.isWaitDir,  waitHit, cf"Replace Index[${i}]")
      HardwareAssertion.withEn(cm.isReplSF,   replSFHit, cf"Replace Index[${i}]")
      HardwareAssertion.withEn(cm.isReplLLC,  replLLCHit, cf"Replace Index[${i}]")
      HardwareAssertion.withEn(cm.isWaitCM,   cmRespHit, cf"Replace Index[${i}]")
      HardwareAssertion.withEn(cm.isResp,     respHit, cf"Replace Index[${i}]")
      HardwareAssertion(!(replSFHit & replLLCHit), cf"Replace Index[${i}]")
      HardwareAssertion.checkTimeout(cm.isFree, TIMEOUT_REPLACE, cf"TIMEOUT: Replace Index[${i}]")
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}