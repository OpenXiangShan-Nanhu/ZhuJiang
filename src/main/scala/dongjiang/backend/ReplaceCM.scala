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
import dongjiang.frontend.decode.{HasPackCmtCode, _}
import dongjiang.data._
import zhujiang.chi.ReqOpcode._
import dongjiang.frontend._
import dongjiang.backend._
import dongjiang.backend.REPLSTATE._
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ------------------------------------------ Replace State -------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object REPLSTATE {
  val width     = 4
  val FREE      = 0x0.U
  val REQPOS    = 0x1.U
  val WRIDIR    = 0x2.U
  val WAITDIR   = 0x3.U
  val REPLHT    = 0x4.U // Replace HnTxnID in DataBlock
  val REPLLLC   = 0x5.U // Send CM Task to SnoopCM
  val REPLSF    = 0x6.U // Send CM Task to WriteCM
  val WAITREPL  = 0x7.U // Wait replace done
  val DATATASK  = 0x8.U // Send save and clean to DataBlock
  val WAITRESP  = 0x9.U // Wait DataBlock Resp
  val CLEANPOS  = 0xA.U
}

trait HasReplMes { this: DJBundle =>
  // Replace LLC Directory:
  // State: Free -> ReqPoS -> WriDir -> WaitDir ---> ReplHnTxnID -> ReplLLC -> WaitRepl -> CleanPoS -> Free
  //                            ^                |                                            ^
  //                            |                |-(replWayIsInvalid)-> DataTask -> WaitResp -|
  //                            |
  //                            -------------(SnpRespWithData)------------
  //                                                                     |
  // Replace SnoopFilter:                                                |
  // State: Free -> ReqPoS -> WriDir -> WaitDir ---> ReplSF -> WaitRepl ---> CleanPoS -> Free
  //                                             |                              ^
  //                                             |------(replWayIsInvalid)------|
  // No need replace:
  // State: Free -> WriDir -> Free
  //
  // Note: no need ReplHnTxnID when it has been replace sf
  // Note: DataTask include save and clean
  val state         = UInt(REPLSTATE.width.W)
  // Replace task message
  val repl          = new DJBundle with HasHnTxnID with HasDsIdx {
    val toLan       = Bool()
  }
  // Flag
  val sResp         = Bool() // Should  send response to Commit
  val alrSResp      = Bool() // Already send response to Commit
  val alrReplSF     = Bool() // Already send task to SnoopCM to replace sf

  def isFree        = state === FREE & !sResp
  def isValid       = !isFree
  def isReqPoS      = state === REQPOS
  def isWriDir      = state === WRIDIR
  def isWaitDir     = state === WAITDIR
  def isReplHnTxnID = state === REPLHT
  def isReplLLC     = state === REPLLLC
  def isReplSF      = state === REPLSF
  def isWaitRepl    = state === WAITREPL
  def isDataTask    = state === DATATASK
  def isWaitResp    = state === WAITRESP
  def isCleanPoS    = state === CLEANPOS
}


// ----------------------------------------------------------------------------------------------------- //
// ------------------------------------------ Replace Entry -------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class ReplaceEntry(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config          = new DJConfigIO()
    // Commit Task In
    val alloc           = Flipped(Decoupled(new ReplTask))
    val resp            = Decoupled(new HnTxnID)
    // Send Task To CM
    val cmTaskVec       = Vec(2, Decoupled(new CMTask)) // Snoop and Write
    val cmResp          = Flipped(Valid(new CMResp)) // broadcast signal
    // From/To PoS
    val reqPoS          = new ReqPoS()
    val updPosTag       = Valid(new Addr with HasPackHnIdx)
    val cleanPoS        = Decoupled(new PosClean)
    // Write Directory
    val writeDir        = Decoupled(new DJBundle {
      val llc           = Valid(new DirEntry("llc") with HasPackHnIdx)
      val sf            = Valid(new DirEntry("sf")  with HasPackHnIdx)
    })
    // Write Directory Resp(broadcast signal)
    val respDir         = new DJBundle {
      val llc           = Flipped(Valid(new DirEntry("llc") with HasHnTxnID))
      val sf            = Flipped(Valid(new DirEntry("sf")  with HasHnTxnID))
    }
    // Send Task To Data
    val replHnTxnID     = Decoupled(new ReplHnTxnID)
    val dataTask        = Decoupled(new DataTask)
    val dataResp        = Flipped(Valid(new HnTxnID)) // broadcast signal
    // for debug
    val dbg             = new Bundle {
      val source        = Valid(new PackHnIdx with HasHnTxnID)
      val replace       = Valid(new PackHnIdx with HasHnTxnID)
    }
  })
  dontTouch(io.dbg)

  /*
   * Reg and Wire declaration
   */
  val taskReg   = RegInit((new ReplTask with HasReplMes).Lit(_.state -> FREE, _.sResp -> false.B, _.alrReplSF -> false.B))
  val taskAlloc = WireInit(0.U.asTypeOf(taskReg))
  val taskNext  = WireInit(taskReg)

  /*
   * Ouput debug message
   */
  // source
  io.dbg.source.valid         := taskReg.isValid
  io.dbg.source.bits.hnTxnID  := taskReg.hnTxnID
  io.dbg.source.bits.hnIdx    := taskReg.getHnIdx
  // replace
  io.dbg.replace.valid        := taskReg.state > REQPOS
  io.dbg.replace.bits.hnTxnID := taskReg.repl.hnTxnID
  io.dbg.replace.bits.hnIdx   := taskReg.repl.getHnIdx

  /*
   * Receive Replace Task
   */
  io.alloc.ready            := taskReg.isFree
  when(io.alloc.fire) {
    taskAlloc.hnTxnID       := io.alloc.bits.hnTxnID
    taskAlloc.dir           := io.alloc.bits.dir
    taskAlloc.wriSF         := io.alloc.bits.wriSF
    taskAlloc.wriLLC        := io.alloc.bits.wriLLC
    taskAlloc.repl.hnTxnID  := io.alloc.bits.hnTxnID // reset repl.hnTxnID = alloc.hnTxnID
  }

  /*
   * Request PoS
   */
  io.reqPoS.req.valid         := taskReg.isReqPoS
  io.reqPoS.req.bits.dirBank  := taskReg.dirBank
  io.reqPoS.req.bits.pos.set  := taskReg.posSet
  io.reqPoS.req.bits.pos.way  := DontCare
  io.reqPoS.req.bits.channel  := Mux(taskReg.replSF, ChiChannel.SNP, ChiChannel.REQ)
  when(io.reqPoS.req.fire) {
    taskNext.repl.hnTxnID     := io.reqPoS.resp.hnTxnID
  }
  HAssert.withEn(taskReg.replDIR, io.reqPoS.req.valid)

  /*
   * Write Directory
   */
  io.writeDir.valid                 := taskReg.isWriDir
  // llc
  io.writeDir.bits.llc.valid        := taskReg.wriLLC
  io.writeDir.bits.llc.bits.addr    := Mux(taskReg.alrReplSF, taskReg.repl.hnTxnID, taskReg.hnTxnID) // remap in DongJiang
  io.writeDir.bits.llc.bits.wayOH   := taskReg.dir.llc.wayOH
  io.writeDir.bits.llc.bits.hit     := taskReg.dir.llc.hit
  io.writeDir.bits.llc.bits.metaVec := taskReg.dir.llc.metaVec
  io.writeDir.bits.llc.bits.hnIdx   := taskReg.repl.getHnIdx
  // sf
  io.writeDir.bits.sf.valid         := taskReg.wriSF
  io.writeDir.bits.sf.bits.addr     := Mux(taskReg.alrReplSF, taskReg.repl.hnTxnID, taskReg.hnTxnID)  // remap in DongJiang
  io.writeDir.bits.sf.bits.wayOH    := taskReg.dir.sf.wayOH
  io.writeDir.bits.sf.bits.hit      := taskReg.dir.sf.hit
  io.writeDir.bits.sf.bits.metaVec  := taskReg.dir.sf.metaVec
  io.writeDir.bits.sf.bits.hnIdx    := taskReg.repl.getHnIdx
  // HAssert
  HAssert.withEn(io.writeDir.bits.llc.bits.hit, io.writeDir.bits.llc.valid & io.writeDir.bits.llc.bits.meta.isInvalid)
  HAssert.withEn(io.writeDir.bits.sf.bits.hit,  io.writeDir.bits.sf.valid  & Cat(io.writeDir.bits.sf.bits.allVec) === 0.U)

  /*
   * Update PoS Tag and Save Message of Addr
   */
  val sfRespHit   = taskReg.isValid & io.respDir.sf.valid   & io.respDir.sf.bits.hnTxnID  === taskReg.repl.hnTxnID
  val llcRespHit  = taskReg.isValid & io.respDir.llc.valid  & io.respDir.llc.bits.hnTxnID === taskReg.repl.hnTxnID
  val dirRespHit  = sfRespHit | llcRespHit
  val needReplSF  = io.respDir.sf.valid   & io.respDir.sf.bits.metaVec.map(_.isValid).reduce(_ | _)
  val needReplLLC = io.respDir.llc.valid  & io.respDir.llc.bits.metaVec.head.isValid
  val respAddr    = Mux(io.respDir.sf.valid, io.respDir.sf.bits.addr, io.respDir.llc.bits.addr)
  HardwareAssertion(!(io.respDir.sf.valid & io.respDir.llc.valid))
  // Update PosTag when resp need repl
  io.updPosTag.valid        := dirRespHit & !taskReg.alrReplSF // Even when needReplSF or needReplLLC is not needed, updTag must still be sent to unlock the PoS Lock.
  io.updPosTag.bits.addr    := respAddr
  io.updPosTag.bits.hnIdx   := taskReg.repl.getHnIdx
  // Save toLan and dsIdx in replace message
  when(llcRespHit) {
    taskNext.repl.toLan     := io.respDir.llc.bits.Addr.isToLAN(io.config.ci)
    taskNext.repl.ds.set(io.respDir.llc.bits.addr, io.respDir.llc.bits.way)
  }
  // save already replace sf
  when(sfRespHit) {
    taskNext.alrReplSF := true.B
    HAssert(!taskReg.alrReplSF)
  }.elsewhen(taskNext.isFree) {
    taskNext.alrReplSF := false.B
  }


  /*
   * Send Replace Req to SnoopCM
   */
  io.cmTaskVec(CMID.SNP).valid              := taskReg.isReplSF
  io.cmTaskVec(CMID.SNP).bits               := DontCare
  io.cmTaskVec(CMID.SNP).bits.chi.nodeId    := DontCare
  io.cmTaskVec(CMID.SNP).bits.chi.channel   := ChiChannel.SNP
  io.cmTaskVec(CMID.SNP).bits.chi.opcode    := SnpOpcode.SnpUnique
  io.cmTaskVec(CMID.SNP).bits.chi.dataVec   := Seq(true.B, true.B)
  io.cmTaskVec(CMID.SNP).bits.chi.retToSrc  := true.B
  io.cmTaskVec(CMID.SNP).bits.chi.toLAN     := DontCare
  io.cmTaskVec(CMID.SNP).bits.hnTxnID       := taskReg.repl.hnTxnID
  io.cmTaskVec(CMID.SNP).bits.alr.reqs      := false.B
  io.cmTaskVec(CMID.SNP).bits.snpVec        := taskReg.dir.sf.metaVec.map(_.state.asBool)
  io.cmTaskVec(CMID.SNP).bits.fromRepl      := true.B
  io.cmTaskVec(CMID.SNP).bits.dataOp.reqs   := true.B

  /*
   * Send Replace Req to WriteCN
   */
  io.cmTaskVec(CMID.WRI).valid                      := taskReg.isReplLLC
  io.cmTaskVec(CMID.WRI).bits                       := DontCare
  io.cmTaskVec(CMID.WRI).bits.chi.nodeId            := DontCare
  io.cmTaskVec(CMID.WRI).bits.chi.channel           := ChiChannel.REQ
  io.cmTaskVec(CMID.WRI).bits.chi.opcode            := Mux(taskReg.repl.toLan, WriteNoSnpFull, Mux(taskReg.dir.llc.meta.isDirty, WriteBackFull, WriteEvictOrEvict))
  io.cmTaskVec(CMID.WRI).bits.chi.dataVec           := Seq(true.B, true.B)
  io.cmTaskVec(CMID.WRI).bits.chi.memAttr.allocate  := false.B
  io.cmTaskVec(CMID.WRI).bits.chi.memAttr.device    := false.B
  io.cmTaskVec(CMID.WRI).bits.chi.memAttr.cacheable := true.B
  io.cmTaskVec(CMID.WRI).bits.chi.memAttr.ewa       := true.B
  io.cmTaskVec(CMID.WRI).bits.chi.toLAN             := taskReg.repl.toLan
  io.cmTaskVec(CMID.WRI).bits.hnTxnID               := taskReg.repl.hnTxnID
  io.cmTaskVec(CMID.WRI).bits.fromRepl              := true.B
  io.cmTaskVec(CMID.WRI).bits.ds                    := taskReg.repl.ds
  io.cmTaskVec(CMID.WRI).bits.cbResp                := Mux(taskReg.repl.toLan, ChiResp.I, taskReg.dir.llc.meta.cbResp)
  io.cmTaskVec(CMID.WRI).bits.alr.reqs              := true.B
  io.cmTaskVec(CMID.WRI).bits.dataOp.repl           := true.B

  /*
   * Replace HnTxnID in DataBlock
   */
  io.replHnTxnID.valid          := taskReg.isReplHnTxnID
  io.replHnTxnID.bits.before    := taskReg.hnTxnID
  io.replHnTxnID.bits.next      := taskReg.repl.hnTxnID

  /*
   * Send Task to DataBlock
   */
  io.dataTask.valid             := taskReg.isDataTask
  io.dataTask.bits              := DontCare
  io.dataTask.bits.hnTxnID      := Mux(taskReg.alrReplSF, taskReg.repl.hnTxnID, taskReg.hnTxnID)
  io.dataTask.bits.dataOp.save  := true.B
  io.dataTask.bits.dataOp.clean := true.B
  io.dataTask.bits.dataVec      := DataVec.Full

  /*
   * Clean PoS
   */
  io.cleanPoS.valid         := taskReg.isCleanPoS
  io.cleanPoS.bits.hnIdx    := taskReg.repl.getHnIdx
  io.cleanPoS.bits.channel  := Mux(taskReg.alrReplSF, ChiChannel.SNP, ChiChannel.REQ)

  /*
   * Send Resp to Commit
   */
  io.resp.valid         := taskReg.sResp
  io.resp.bits.hnTxnID  := taskReg.hnTxnID

  /*
   * Get Next State
   */
  val cmRespHit   = taskReg.isValid & io.cmResp.valid & io.cmResp.bits.hnTxnID === taskReg.repl.hnTxnID
  val dataRespHit = taskReg.isValid & io.dataResp.valid & io.dataResp.bits.hnTxnID === Mux(taskReg.alrReplSF, taskReg.repl.hnTxnID, taskReg.hnTxnID)
  val cmRespData  = io.cmResp.bits.taskInst.valid & io.cmResp.bits.taskInst.channel === ChiChannel.DAT
  switch(taskReg.state) {
    // Free
    is(FREE) {
      when(io.alloc.fire)               { taskNext.state := Mux(io.alloc.bits.replDIR, REQPOS, WRIDIR) }
    }
    // Request PoS
    is(REQPOS) {
      when(io.reqPoS.req.fire)          { taskNext.state := WRIDIR }
    }
    // Write Directory
    is(WRIDIR) {
      when(io.writeDir.fire)            { taskNext.state := Mux(taskReg.replDIR, WAITDIR, FREE) }
    }
    // Wait Directory write response
    is(WAITDIR) {
      when(dirRespHit)                  { taskNext.state := Mux(sfRespHit, Mux(needReplSF, REPLSF, CLEANPOS), Mux(needReplLLC, Mux(taskReg.alrReplSF, REPLLLC, REPLHT), DATATASK)) }
    }
    // Replace HnTxnID in DataBlock
    is(REPLHT) {
      when(io.replHnTxnID.fire)         { taskNext.state := REPLLLC }
    }
    // Send replace task to SnoopCM
    is(REPLSF) {
      when(io.cmTaskVec(CMID.SNP).fire) { taskNext.state := WAITREPL }
    }
    // Send replace task to WriteCM
    is(REPLLLC) {
      when(io.cmTaskVec(CMID.WRI).fire) { taskNext.state := WAITREPL }
    }
    // Wait TaskCM response
    is(WAITREPL) {
      when(cmRespHit)                   { taskNext.state := Mux(cmRespData, WRIDIR, CLEANPOS) }
    }
    // Send DataTask to DataBlock
    is(DATATASK) {
      when(io.dataTask.fire)            { taskNext.state := WAITRESP }
    }
    // Wait DataBlock response
    is(WAITRESP) {
      when(dataRespHit)                 { taskNext.state := CLEANPOS }
    }
    // Clean PoS
    is(CLEANPOS) {
      when(io.cleanPoS.fire)            { taskNext.state := FREE }
    }
  }

  /*
   * HAssert
   */
  HAssert.withEn(taskReg.isFree,        io.alloc.fire)
  HAssert.withEn(taskReg.isReqPoS,      taskReg.isValid & io.reqPoS.req.fire)
  HAssert.withEn(taskReg.isWriDir,      taskReg.isValid & io.writeDir.fire)
  HAssert.withEn(taskReg.isWaitDir,     taskReg.isValid & dirRespHit)
  HAssert.withEn(taskReg.isReplHnTxnID, taskReg.isValid & io.replHnTxnID.fire)
  HAssert.withEn(taskReg.isReplSF,      taskReg.isValid & io.cmTaskVec(CMID.SNP).fire)
  HAssert.withEn(taskReg.isReplLLC,     taskReg.isValid & io.cmTaskVec(CMID.WRI).fire)
  HAssert.withEn(taskReg.isWaitRepl,    taskReg.isValid & cmRespHit)
  HAssert.withEn(taskReg.isDataTask,    taskReg.isValid & io.dataTask.fire)
  HAssert.withEn(taskReg.isCleanPoS,    taskReg.isValid & io.cleanPoS.fire)
  HAssert.withEn(taskReg.isWaitRepl | taskReg.isWaitResp, taskReg.isValid & dataRespHit)

  /*
   * Get next sResp
   * ---------------------
   * | wSF | wLLC | Resp |
   * |-------------------|
   * |  I  |  I   |  T   |
   * |  V  |  I   |  T   |
   * |  I  |  V   |  F   |
   * |  V  |  V   |  X   |
   * ---------------------
   */
  when(io.resp.fire) {
    taskNext.sResp    := false.B
    taskNext.alrSResp := taskReg.state =/= FREE
    // HAssert
    HAssert(taskReg.sResp)
    HAssert(!taskReg.alrSResp)
  // Send resp no need wait replace task done
  }.elsewhen(io.writeDir.fire) {
    val wriSFVal      = io.writeDir.bits.sf.valid  & io.writeDir.bits.sf.bits.metaIsVal
    val wriLLCVal     = io.writeDir.bits.llc.valid & io.writeDir.bits.llc.bits.metaIsVal
    taskNext.sResp    := PriorityMux(Seq(
      taskReg.alrSResp          -> false.B,
      (!wriSFVal & !wriLLCVal)  -> true.B,
      ( wriSFVal & !wriLLCVal)  -> true.B,
      (!wriSFVal &  wriLLCVal)  -> false.B
    ))
    // HAssert
    HAssert(!(wriSFVal & wriLLCVal))
    HAssert.withEn(!taskReg.sResp, taskNext.sResp)
  // Replace task done
  }.elsewhen(io.cleanPoS.fire) {
    taskNext.sResp    := !taskReg.alrSResp
    taskNext.alrSResp := false.B
    // HAssert
    HAssert(!taskReg.sResp)
  }

  /*
   * Get new write directory message
   */
  val cmRespDirty     = io.cmResp.bits.taskInst.resp.asTypeOf(new ChiResp).passDirty
  when(sfRespHit) {
    taskNext.wriSF    := false.B
    taskNext.wriLLC   := false.B
    taskNext.dir.sf   := io.respDir.sf.bits
    taskNext.dir.llc  := DontCare
    HAssert(taskReg.replSF)
  }.elsewhen(llcRespHit) {
    taskNext.wriSF    := false.B
    taskNext.wriLLC   := false.B
    taskNext.dir.sf   := DontCare
    taskNext.dir.llc  := io.respDir.llc.bits
    HAssert(taskReg.replLLC)
  }.elsewhen(cmRespHit & cmRespData) {
    taskNext.wriSF    := false.B
    taskNext.wriLLC   := true.B
    taskNext.dir.sf   := DontCare
    taskNext.dir.llc.wayOH      := DontCare
    taskNext.dir.llc.hit        := false.B
    taskNext.dir.llc.meta.state := Mux(cmRespDirty, ChiState.UD, ChiState.SC)
  }

  /*
   * Set new task
   */
  val set = io.alloc.fire | taskReg.isValid; dontTouch(set)
  when(set) {
    taskReg       := Mux(io.alloc.fire, taskAlloc, taskNext)
    taskReg.state := taskNext.state
  }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(taskReg.isFree, TIMEOUT_REPLACE, cf"TIMEOUT: Replace State[${taskReg.state}]")
}

// ----------------------------------------------------------------------------------------------------- //
// --------------------------------------------- ReplaceCM --------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class ReplaceCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config          = new DJConfigIO()
    // Commit Task In
    val task            = Flipped(Decoupled(new ReplTask))
    val resp            = Valid(new HnTxnID)
    // Send Task To CM
    val cmTaskVec       = Vec(2, Decoupled(new CMTask)) // Snoop and Write
    val cmResp          = Flipped(Valid(new CMResp))
    // From/To PoS
    val reqPosVec       = Vec(djparam.nrDirBank, new ReqPoS())
    val updPosTag       = Valid(new Addr with HasPackHnIdx)
    val cleanPoS        = Decoupled(new PosClean)
    // Write Directory
    val writeDir        = Decoupled(new DJBundle {
      val llc           = Valid(new DirEntry("llc") with HasPackHnIdx)
      val sf            = Valid(new DirEntry("sf")  with HasPackHnIdx)
    })
    // Write Directory Resp(broadcast signal)
    val respDir         = new DJBundle {
      val llc           = Flipped(Valid(new DirEntry("llc") with HasHnTxnID))
      val sf            = Flipped(Valid(new DirEntry("sf")  with HasHnTxnID))
    }
    // Send Task To Data
    val replHnTxnID     = Valid(new ReplHnTxnID)
    val dataTask        = Decoupled(new DataTask)
    val dataResp        = Flipped(Valid(new HnTxnID)) // broadcast signal
  })

  /*
   * Module and Wire declaration
   */
  val entries  = Seq.fill(nrReplaceCM) { Module(new ReplaceEntry()) }
  val dbgVec   = WireInit(VecInit(entries.map(_.io.dbg)))
  val selIdVec = Wire(Vec(djparam.nrDirBank, UInt(log2Ceil(nrReplaceCM).W)))
  dontTouch(dbgVec)
  dontTouch(selIdVec)

  /*
   * Receive Replace Task from Commit
   */
  Alloc(entries.map(_.io.alloc), io.task)

  /*
   * Request PoS
   */
  io.reqPosVec.map(_.req).zipWithIndex.foreach { case(req, i) =>
    val hitVec  = VecInit(entries.map{ e => e.io.reqPoS.req.valid & e.io.reqPoS.req.bits.dirBank === i.U })
    selIdVec(i) := StepRREncoder(hitVec, req.fire)
    req.valid   := hitVec.asUInt.orR
    req.bits    := VecInit(entries.map(_.io.reqPoS.req.bits))(selIdVec(i))
  }
  entries.zipWithIndex.foreach { case(e, i) =>
    val dirBank           = e.io.reqPoS.req.bits.dirBank
    e.io.reqPoS.req.ready := io.reqPosVec(dirBank).req.ready & selIdVec(dirBank) === i.U
    e.io.reqPoS.resp      := io.reqPosVec(dirBank).resp
  }

  /*
   * Connect Replace <- IO
   */
  entries.foreach { e =>
    e.io.config   := io.config
    e.io.cmResp   := io.cmResp
    e.io.respDir  := io.respDir
    e.io.dataResp := io.dataResp
  }


  /*
   * Connect IO <- Replace
   */
  io.resp         := fastRRArb.validOut(entries.map(_.io.resp))
  io.replHnTxnID  := fastRRArb.validOut(entries.map(_.io.replHnTxnID))
  io.updPosTag    := fastRRArb(entries.map(_.io.updPosTag))
  io.cleanPoS     <> fastRRArb(entries.map(_.io.cleanPoS))
  io.dataTask     <> fastRRArb(entries.map(_.io.dataTask))
  io.writeDir     <> fastRRArb(entries.map(_.io.writeDir))
  // Send cmTask
  io.cmTaskVec.zip(entries.map(_.io.cmTaskVec).transpose).foreach { case(a, b) => a <> fastRRArb(b) }

  /*
   * HAssert placePipe
   */
  HAssert.placePipe(1)
}