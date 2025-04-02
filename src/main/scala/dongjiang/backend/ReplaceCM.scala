package dongjiang.backend.replace

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle.{HasAddr, HasPackLLCTxnID, PackChi, _}
import dongjiang.data.HasAlrDB
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, HasPackDirMsg}
import dongjiang.frontend.decode.{HasPackCmtCode, _}
import dongjiang.data._
import zhujiang.chi.ReqOpcode._
import dongjiang.frontend._
import dongjiang.backend._
import dongjiang.backend.replace.State._

object State {
  val width   = 5
  val FREE    = "b00000".U
  val WRITE   = "b00001".U
  val WAITDIR = "b00010".U
  val REPL    = "b00100".U
  val WAITCM  = "b01000".U
  val RESP    = "b10000".U
}

// TODO
class CMState(implicit p: Parameters) extends DJBundle {
  // FREE --> WRITE --> WAITDIR --> REPLSF/RESPLLC --> WAITCM --> WRITE --> WAITDIR --> RESPLLC --> WAITCM --> RESP
  //            ^                         ^                         ^                      ^
  //         lockSet                  unLockSet                  lockSet                unLockSet
  val state     = UInt(State.width.W)
  val waitSF    = Bool()
  val waitLLC   = Bool()
  val replSF    = Bool()
  val replLLC   = Bool()
  def needWait  = waitSF | waitLLC
  def needRepl  = replSF | replLLC

  def isValid   = state.orR
  def isFree    = !isValid
  def isWrite   = state(0)
  def isWaitDir = state(1)
  def isRepl    = state(2)
  def isWaitCM  = state(3)
  def isResp    = state(4)
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
    val cmAllocVec      = Vec(nrTaskCM, Decoupled(new CMTask))
    // Update PoS Message
    val updPosTag       = Valid(new LLCTxnID with HasAddr)
    val lockPosSet      = Valid(new LLCTxnID with HasLockSet)
    // Write Directory
    val writeDir        = new DJBundle {
      val llc           = Decoupled(new DirEntry("llc") with HasPackPosIndex)
      val sf            = Decoupled(new DirEntry("sf") with HasPackPosIndex)
    }
    // Write Directory Resp
    val respDir         = new DJBundle {
      val llc           = Flipped(Valid(new DirEntry("llc")))
      val sf            = Flipped(Valid(new DirEntry("sf")))
    }
    // Resp From TaskCM
    val respRepl        = Flipped(Valid(new PackLLCTxnID with HasChiChannel with HasChiResp))

  })

  io.cmAllocVec(CMID.READ) <> DontCare
  io.cmAllocVec(CMID.DL)   <> DontCare


  /*
   * Reg and Wire declaration
   */
  val cmRegVec  = RegInit(VecInit(Seq.fill(nrReplaceCM) { 0.U.asTypeOf(new CMState()) }))
  val msgRegVec = Reg(Vec(nrReplaceCM, chiselTypeOf(io.alloc.bits)))
  val waitPipe  = Module(new Pipe(UInt(log2Ceil(nrReplaceCM).W), readDirLatency))


  /*
   * [REC] Receive Task IO
   */
  val cmVec_rec   = cmRegVec.map(_.isFree) // Free Vec
  val cmId_rec    = PriorityEncoder(cmVec_rec)
  io.alloc.ready  := cmVec_rec.reduce(_ | _)

  /*
   * [WRITE]
   */
  val cmVec_wri = cmRegVec.map(_.isWrite)
  val cmId_wri  = RREncoder(cmVec_wri)
  val msg_wri   = msgRegVec(cmId_wri)
  // llc
  io.writeDir.llc.valid         := msg_wri.wriLLC
  io.writeDir.llc.bits.addr     := msg_wri.addr
  io.writeDir.llc.bits.wayOH    := msg_wri.dir.llc.wayOH
  io.writeDir.llc.bits.hit      := msg_wri.dir.llc.hit
  io.writeDir.llc.bits.metaVec  := msg_wri.dir.llc.metaVec
  io.writeDir.llc.bits.pos      := msg_wri.llcTxnID.pos
  // sf
  io.writeDir.sf.valid          := msg_wri.wriSF
  io.writeDir.sf.bits.addr      := msg_wri.addr
  io.writeDir.sf.bits.wayOH     := msg_wri.dir.sf.wayOH
  io.writeDir.sf.bits.hit       := msg_wri.dir.sf.hit
  io.writeDir.sf.bits.metaVec   := msg_wri.dir.sf.metaVec
  io.writeDir.sf.bits.pos       := msg_wri.llcTxnID.pos
  // waitPipe
  waitPipe.io.enq.valid := io.writeDir.llc.fire | io.writeDir.sf.fire
  waitPipe.io.enq.bits  := cmId_wri
  HardwareAssertion.withEn(io.respDir.sf.valid | io.respDir.llc.valid, waitPipe.io.deq.valid)


  /*
   * [REPL]
   */
  val cmVec_repl  = cmRegVec.map(_.isRepl)
  val cmId_repl   = RREncoder(cmVec_repl)
  val msg_repl    = msgRegVec(cmId_repl)
  // SNP
  io.cmAllocVec(CMID.SNP).valid             := cmRegVec(cmId_repl).replSF & !(io.writeDir.llc.fire | io.writeDir.sf.fire)
  io.cmAllocVec(CMID.SNP).bits              := DontCare
  io.cmAllocVec(CMID.SNP).bits.chi.nodeId   := DontCare
  io.cmAllocVec(CMID.SNP).bits.chi.channel  := ChiChannel.SNP
  io.cmAllocVec(CMID.SNP).bits.chi.opcode   := SnpOpcode.SnpUnique
  io.cmAllocVec(CMID.SNP).bits.chi.dataVec  := Seq(true.B, true.B)
  io.cmAllocVec(CMID.SNP).bits.chi.retToSrc := true.B
  io.cmAllocVec(CMID.SNP).bits.chi.toLAN    := DontCare
  io.cmAllocVec(CMID.SNP).bits.addr         := msg_repl.addr
  io.cmAllocVec(CMID.SNP).bits.llcTxnID     := msg_repl.llcTxnID
  io.cmAllocVec(CMID.SNP).bits.needDB       := true.B
  io.cmAllocVec(CMID.SNP).bits.alrDB        := msg_repl.alrDB
  io.cmAllocVec(CMID.SNP).bits.snpVec       := msg_repl.dir.sf.metaVec.map(_.state.asBool)
  io.cmAllocVec(CMID.SNP).bits.fromRepl     := true.B
  io.cmAllocVec(CMID.SNP).bits.ds.bank      := DontCare
  io.cmAllocVec(CMID.SNP).bits.ds.idx       := DontCare
  io.cmAllocVec(CMID.SNP).bits.cbResp       := DontCare
  // REQ
  io.cmAllocVec(CMID.WOA).valid             := cmRegVec(cmId_repl).replLLC & !(io.writeDir.llc.fire | io.writeDir.sf.fire)
  io.cmAllocVec(CMID.WOA).bits              := DontCare
  io.cmAllocVec(CMID.WOA).bits.chi.nodeId   := DontCare
  io.cmAllocVec(CMID.WOA).bits.chi.channel  := ChiChannel.REQ
  io.cmAllocVec(CMID.WOA).bits.chi.opcode   := Mux(msg_repl.Addr.isToLAN(io.config.ci), ReqOpcode.WriteNoSnpFull, ReqOpcode.WriteEvictOrEvict)
  io.cmAllocVec(CMID.WOA).bits.chi.dataVec  := Seq(true.B, true.B)
  io.cmAllocVec(CMID.WOA).bits.chi.memAttr.allocate   := false.B
  io.cmAllocVec(CMID.WOA).bits.chi.memAttr.device     := false.B
  io.cmAllocVec(CMID.WOA).bits.chi.memAttr.cacheable  := true.B
  io.cmAllocVec(CMID.WOA).bits.chi.memAttr.ewa        := true.B
  io.cmAllocVec(CMID.WOA).bits.chi.toLAN    := msg_repl.Addr.isToLAN(io.config.ci)
  io.cmAllocVec(CMID.WOA).bits.fromRepl     := true.B
  io.cmAllocVec(CMID.WOA).bits.ds.bank      := getDS(msg_repl.addr, msg_repl.dir.llc.way)._1
  io.cmAllocVec(CMID.WOA).bits.ds.idx       := getDS(msg_repl.addr, msg_repl.dir.llc.way)._2
  io.cmAllocVec(CMID.SNP).bits.cbResp       := msg_repl.dir.llc.metaVec.head.state

  /*
   * Update PoS Tag
   */
  io.updPosTag.valid        := io.cmAllocVec(CMID.SNP).fire | io.cmAllocVec(CMID.WOA).fire
  io.updPosTag.bits.dirBank := msg_repl.llcTxnID.dirBank
  io.updPosTag.bits.pos     := msg_repl.llcTxnID.pos
  io.updPosTag.bits.addr    := msg_repl.addr


  /*
   * Update PoS Lock
   */
  // TODO
  io.lockPosSet.valid           := io.writeDir.llc.fire | io.writeDir.sf.fire | cmRegVec(cmId_repl).replSF | cmRegVec(cmId_repl).replLLC
  io.lockPosSet.bits.dirBank    := msg_wri.llcTxnID.dirBank
  io.lockPosSet.bits.pos        := msg_wri.llcTxnID.pos
  io.lockPosSet.bits.lock       := io.writeDir.llc.fire | io.writeDir.sf.fire


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
  val hwaVec2 = WireInit(VecInit(Seq.fill(nrReplaceCM) { VecInit(Seq.fill(8) { true.B }) }))
  cmRegVec.zip(msgRegVec).zipWithIndex.foreach {
    case ((cm, msg), i) =>
      val allocHit    = io.alloc.fire & cmId_rec === i.U
      val writeHit    = (io.writeDir.llc.fire | io.writeDir.sf.fire) & cmId_wri === i.U
      val waitHit     = waitPipe.io.deq.valid & waitPipe.io.deq.bits === i.U
      val replSFHit   = io.cmAllocVec(CMID.SNP).fire & cmId_repl === i.U
      val replLLCHit  = io.cmAllocVec(CMID.WOA).fire & cmId_repl === i.U
      val cmRespHit   = io.respRepl.fire & io.respRepl.bits.llcTxnID.get === msg.llcTxnID.get
      val respHit     = io.resp.fire & cmId_resp === i.U

      // Message
      val wriAll      = WireInit(false.B)
      val needSecRepl = cmRespHit & io.respRepl.bits.state =/= ChiState.I & io.respRepl.bits.isSnp
      when(allocHit) {
        msg := io.alloc.bits
      }.elsewhen(writeHit) {
        val wriSF_nxt  = msg.wriSF  & !io.writeDir.sf.fire
        val wriLLC_nxt = msg.wriLLC & !io.writeDir.llc.fire
        msg.wriSF     := wriSF_nxt
        msg.wriLLC    := wriLLC_nxt
        wriAll        := !wriSF_nxt & !wriLLC_nxt
      }.elsewhen(waitHit) {
        when(io.respDir.sf.valid)  { msg.dir.sf  := io.respDir.sf.bits  }
        when(io.respDir.llc.valid) { msg.dir.llc := io.respDir.llc.bits }
        msg.addr := Mux(io.respDir.sf.valid, io.respDir.sf.bits.hit, io.respDir.llc.bits.addr)
      }.elsewhen(needSecRepl) {
        msg.wriLLC        := true.B
        msg.dir.llc.wayOH := DontCare
        msg.dir.llc.hit   := false.B
        msg.dir.llc.metaVec.head.state := io.respRepl.bits.state
      }.elsewhen(respHit) {
        msg := 0.U.asTypeOf(msg)
      }

      // Ctrl Machine Wait
      val waitAll     = WireInit(false.B)

      when(allocHit) {
        cm.waitSF   := io.alloc.bits.wriSF  & !io.alloc.bits.dir.llc.hit
        cm.waitLLC  := io.alloc.bits.wriLLC & !io.alloc.bits.dir.llc.hit
      }.elsewhen(waitHit) {
        // wait
        val waitSFnxt   = cm.waitSF  & !io.respDir.sf.valid
        val waitLLCnxt  = cm.waitLLC & !io.respDir.llc.valid
        cm.waitSF       := waitSFnxt
        cm.waitLLC      := waitLLCnxt
        waitAll         := !waitSFnxt & !waitLLCnxt
      }.elsewhen(needSecRepl) {
        cm.waitLLC      := true.B
      }

      // Ctrl Machine Repl
      val replAll = WireInit(false.B)
      when(waitHit) {
        when(io.respDir.sf.valid)  { cm.replSF  := io.respDir.sf.bits.hit  }
        when(io.respDir.llc.valid) { cm.replLLC := io.respDir.llc.bits.hit }
      }.elsewhen(replSFHit | replLLCHit) {
        val replSF_nxt  = cm.replSF  & !replSFHit
        val replLLC_nxt = cm.replLLC & !replLLCHit
        cm.replSF       := replSF_nxt
        cm.replLLC      := replLLC_nxt
        replAll         := !replSF_nxt & !replLLC_nxt
      }.elsewhen(needSecRepl) {
        cm.replLLC      := true.B
      }

      // Ctrl Machine State
      cm.state := PriorityMux(Seq(
        allocHit  -> WRITE,
        wriAll    -> Mux(cm.needWait, WAITDIR, RESP),
        waitAll   -> REPL,
        replAll   -> Mux(cm.needRepl, WAITCM, RESP),
        cmRespHit -> Mux(needSecRepl, REPL, RESP),
        respHit   -> FREE,
        true.B    -> cm.state
      ))

      when(allocHit) { hwaVec2(i)(0) := cm.asUInt === 0.U & msg.asUInt === 0.U }
      when(waitHit)  { hwaVec2(i)(1) := !(cm.replSF & cm.replLLC)  }
      hwaVec2(i)(2) := !(replSFHit & replLLCHit)
  }

  /*
   * HardwareAssertion placePipe
   */
  hwaVec2.transpose.zipWithIndex.foreach {
    case (vec, i) =>
      val idx = PriorityEncoder(vec)
      HardwareAssertion(vec.reduce(_ | _), cf"Index[$idx] : Type[$i]")
  }
  HardwareAssertion.placePipe(Int.MaxValue-2)
}