package dongjiang.backend

import chisel3._
import chisel3.experimental.BundleLiterals._
import chisel3.util._
import dongjiang._
import dongjiang.backend.ReplaceState._
import dongjiang.bundle._
import dongjiang.data._
import dongjiang.directory.DirEntry
import dongjiang.frontend._
import dongjiang.frontend.decode._
import dongjiang.utils._
import org.chipsalliance.cde.config._
import xs.utils.debug._
import zhujiang.chi.ReqOpcode._
import zhujiang.chi._

// ----------------------------------------------------------------------------------------------------- //
// ------------------------------------------ Replace State -------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object ReplaceState {
  val width     = 4
  val FREE      = 0x0.U
  val REQPOS    = 0x1.U
  val WRIDIR    = 0x2.U
  val WAITDIR   = 0x3.U
  val UPDATEID  = 0x4.U // Update HnTxnID in DataBlock
  val RESPCMT   = 0x5.U // Send Resp to Commit
  val REQDB     = 0x6.U // Request DataBuffer
  val WRITE     = 0x7.U // Send CM Task to WriteCM
  val SNOOP     = 0x8.U // Send CM Task to SnoopCM
  val WAITRWRI  = 0x9.U // Wait replace LLC done
  val WAITRSNP  = 0xA.U // Wait replace SF done
  val COPYID    = 0xB.U // Copy repl.hnTxnID to task.hnTxnID in ReplaceCM
  val SAVEDATA  = 0xC.U // Send save and clean to DataBlock
  val WAITRESP  = 0xD.U // Wait DataBlock Resp
  val CLEANPOST = 0xE.U // Clean PoS(task.hnTxnID)
  val CLEANPOSR = 0xF.U // Clean PoS(repl.hnTxnID)
}

trait HasReplMes { this: DJBundle =>
  // 0. Replace LLC Directory:
  //  State: Free -> ReqPoS -> WriDir -> WaitDir ---> UpdHnTxnID -> ReplLLC -> WaitWri -> (RespCmt/CleanPosT) -> CleanPosR -> Free
  //                    ^                         |                                               ^
  //                    |                         |--(replWayIsInvalid)--> SaveData --> WaitResp -|
  //                    |
  //                    ---------- CopyHnTxnID <-----(SnpRespWithData)-------------------------
  //                                                                                          |
  // 1. Replace SnoopFilter:                                                                  |
  //  State: Free -> ReqPoS -> WriDir -> WaitDir -> RespCmt ---> ReqDB --> ReplSF -> WaitSnp ---> CleanPosR -> Free
  //                                                         |                                        ^
  //                                                         |------------(replWayIsInvalid)----------|
  // 2. No need replace:
  //  State: Free -> WriDir -> RespCmt -> Free
  //
  // Note: only do CleanPosT when alrReplSF in replace LLC directory
  // Note: dont do RespCmt when alrReplSF in replace LLC directory

  val state         = UInt(ReplaceState.width.W)
  val ds            = new DsIdx()
  // Replace task message
  val repl          = new DJBundle with HasHnTxnID { val toLan = Bool() }
  // Flag
  val needSnp       = Bool() // Need send task to SnoopCM to replace sf
  val alrReplSF     = Bool() // Already replace SF

  def isFree        = state === FREE
  def isValid       = !isFree
  def isReqPoS      = state === REQPOS
  def isWriDir      = state === WRIDIR
  def isWaitDir     = state === WAITDIR
  def isUpdHnTxnID  = state === UPDATEID
  def isRespCmt     = state === RESPCMT
  def isReqDB       = state === REQDB
  def isWrite       = state === WRITE
  def isSnoop       = state === SNOOP
  def isWaitWrite   = state === WAITRWRI
  def isWaitSnp     = state === WAITRSNP
  def isWaitRepl    = isWaitWrite | isWaitSnp
  def isCopyID      = state === COPYID
  def isSaveData    = state === SAVEDATA
  def isWaitResp    = state === WAITRESP
  def isCleanPosT   = state === CLEANPOST
  def isCleanPosR   = state === CLEANPOSR
  def isCleanPoS    = isCleanPosT | isCleanPosR
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
    val updPosTag       = Valid(new Addr with HasAddrValid with HasPackHnIdx)
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
    val reqDB           = Decoupled(new HnTxnID with HasDataVec with HasQoS)
    val updHnTxnID      = Decoupled(new UpdHnTxnID)
    val dataTask        = Decoupled(new DataTask)
    val dataResp        = Flipped(Valid(new HnTxnID)) // broadcast signal
    // for debug
    val dbg             = Valid(new Bundle {
      val task          = new PackHnIdx with HasHnTxnID
      val repl          = new PackHnIdx with HasHnTxnID
    })
  })
  dontTouch(io.dbg)

  /*
   * Reg and Wire declaration
   */
  val reg   = RegInit((new ReplTask with HasReplMes).Lit(_.state -> FREE, _.alrReplSF -> false.B))
  val alloc = WireInit(0.U.asTypeOf(reg))
  val next  = WireInit(reg)

  /*
   * Ouput debug message
   */
  io.dbg.valid              := reg.isValid
  // task HnTxnID
  io.dbg.bits.task.hnTxnID  := reg.hnTxnID
  io.dbg.bits.task.hnIdx    := reg.getHnIdx
  // repl HnTxnID
  io.dbg.bits.repl.hnTxnID  := reg.repl.hnTxnID
  io.dbg.bits.repl.hnIdx    := reg.repl.getHnIdx

  /*
   * Receive Replace Task
   */
  io.alloc.ready        := reg.isFree
  when(io.alloc.fire) {
    alloc.dir           := io.alloc.bits.dir
    alloc.wriSF         := io.alloc.bits.wriSF
    alloc.wriLLC        := io.alloc.bits.wriLLC
    alloc.hnTxnID       := io.alloc.bits.hnTxnID
    alloc.repl.hnTxnID  := io.alloc.bits.hnTxnID // reset repl.hnTxnID = alloc.hnTxnID
    alloc.qos           := io.alloc.bits.qos
  }

  /*
   * Request PoS
   */
  io.reqPoS.req.valid         := reg.isReqPoS
  io.reqPoS.req.bits.dirBank  := reg.dirBank
  io.reqPoS.req.bits.pos.set  := reg.posSet
  io.reqPoS.req.bits.pos.way  := DontCare
  io.reqPoS.req.bits.channel  := Mux(reg.isReplSF, ChiChannel.SNP, ChiChannel.REQ)
  io.reqPoS.req.bits.qos      := reg.qos
  HAssert.withEn(reg.isReplDIR, io.reqPoS.req.valid)

  /*
   * Set HnTxnID:
   *
   *  from alloc            -> 0
   *  from PoS first time   -> 1
   *  from PoS second time  -> 2
   *  Note: alrReplSF indicate it from PoS second time
   *
   *  ---------------------------------------------------
   *  |    State     | alloc | ReqPoS | CopyID | ReqPoS |
   *  ---------------------------------------------------
   *  | task.hnTxnID |   0   |    0   |    1   |    1   |
   *  | repl.hnTxnID |   0   |    1   |    1   |    2   |
   *  ---------------------------------------------------
   *
   *  task.hnTxnID: Replacer, stores the address of the directory to be written to
   *  repl.hnTxnID: Replaced, stores the address that needs to be replaced
   *
   */
  // CopyID
  when(reg.isCopyID) {
    next.hnTxnID      := reg.repl.hnTxnID
  // ReqPoS
  }.elsewhen(io.reqPoS.req.fire) {
    next.repl.hnTxnID := io.reqPoS.resp.hnTxnID
  }

  /*
   * Write Directory
   */
  io.writeDir.valid                 := reg.isWriDir
  // llc
  io.writeDir.bits.llc.valid        := reg.wriLLC
  io.writeDir.bits.llc.bits.addr    := reg.hnTxnID // remap in DongJiang
  io.writeDir.bits.llc.bits.wayOH   := reg.dir.llc.wayOH
  io.writeDir.bits.llc.bits.hit     := reg.dir.llc.hit
  io.writeDir.bits.llc.bits.metaVec := reg.dir.llc.metaVec
  io.writeDir.bits.llc.bits.hnIdx   := reg.getHnIdx
  // sf
  io.writeDir.bits.sf.valid         := reg.wriSF
  io.writeDir.bits.sf.bits.addr     := reg.hnTxnID  // remap in DongJiang
  io.writeDir.bits.sf.bits.wayOH    := reg.dir.sf.wayOH
  io.writeDir.bits.sf.bits.hit      := reg.dir.sf.hit
  io.writeDir.bits.sf.bits.metaVec  := reg.dir.sf.metaVec
  io.writeDir.bits.sf.bits.hnIdx    := reg.getHnIdx
  // HAssert
  // when write dir meta is invalid, it must be hit
  HAssert.withEn(io.writeDir.bits.llc.bits.hit, io.writeDir.valid & io.writeDir.bits.llc.valid & io.writeDir.bits.llc.bits.meta.isInvalid)
  HAssert.withEn(io.writeDir.bits.sf.bits.hit,  io.writeDir.valid & io.writeDir.bits.sf.valid  & io.writeDir.bits.sf.bits.metaIsInv)
  // when write dir no hit, meta must be valid
  HAssert.withEn(io.writeDir.bits.llc.bits.meta.isValid,  io.writeDir.valid & io.writeDir.bits.llc.valid & !io.writeDir.bits.llc.bits.hit)
  HAssert.withEn(io.writeDir.bits.sf.bits.metaIsVal,      io.writeDir.valid & io.writeDir.bits.sf.valid  & !io.writeDir.bits.sf.bits.hit)

  /*
   * Update PoS Tag and Save Message of Addr
   */
  val sfRespHit   = reg.isWaitDir & io.respDir.sf.valid   & io.respDir.sf.bits.hnTxnID  === reg.hnTxnID
  val llcRespHit  = reg.isWaitDir & io.respDir.llc.valid  & io.respDir.llc.bits.hnTxnID === reg.hnTxnID
  val dirRespHit  = sfRespHit | llcRespHit
  val needReplSF  = io.respDir.sf.valid   & io.respDir.sf.bits.metaVec.map(_.isValid).reduce(_ | _)
  val needReplLLC = io.respDir.llc.valid  & io.respDir.llc.bits.metaVec.head.isValid
  val respAddr    = Mux(io.respDir.sf.valid, io.respDir.sf.bits.addr, io.respDir.llc.bits.addr)
  HAssert(!(io.respDir.sf.valid & io.respDir.llc.valid))
  HAssert.withEn(reg.isReplSF,  sfRespHit)
  HAssert.withEn(reg.isReplLLC, llcRespHit)
  // Update PosTag when resp need repl
  io.updPosTag.valid        := dirRespHit // Even when needReplSF or needReplLLC is false, updTag must still be sent to unlock the PoS Lock.
  io.updPosTag.bits.addrVal := Mux(io.respDir.sf.valid, io.respDir.sf.bits.metaIsVal, io.respDir.llc.bits.metaIsVal)
  io.updPosTag.bits.addr    := respAddr
  io.updPosTag.bits.hnIdx   := reg.repl.getHnIdx
  // Save toLan and dsIdx in replace message
  when(llcRespHit) {
    next.repl.toLan         := io.respDir.llc.bits.Addr.isToLAN(io.config.ci)
    next.ds.set(io.respDir.llc.bits.addr, io.respDir.llc.bits.way)
  }
  // save flag
  when(sfRespHit) {
    next.needSnp    := needReplSF
    next.alrReplSF  := true.B
    HAssert(!reg.alrReplSF)
  }.elsewhen(next.isFree) {
    next.needSnp    := false.B
    next.alrReplSF  := false.B
  }

  /*
   * Send Replace Req to SnoopCM
   */
  io.cmTaskVec(CMID.SNP).valid              := reg.isSnoop
  io.cmTaskVec(CMID.SNP).bits               := DontCare
  io.cmTaskVec(CMID.SNP).bits.chi.nodeId    := DontCare
  io.cmTaskVec(CMID.SNP).bits.chi.channel   := ChiChannel.SNP
  io.cmTaskVec(CMID.SNP).bits.chi.opcode    := SnpOpcode.SnpUnique
  io.cmTaskVec(CMID.SNP).bits.chi.dataVec   := Seq(true.B, true.B)
  io.cmTaskVec(CMID.SNP).bits.chi.retToSrc  := true.B
  io.cmTaskVec(CMID.SNP).bits.chi.toLAN     := DontCare
  io.cmTaskVec(CMID.SNP).bits.hnTxnID       := reg.repl.hnTxnID
  io.cmTaskVec(CMID.SNP).bits.snpVec        := reg.dir.sf.metaVec.map(_.state.asBool)
  io.cmTaskVec(CMID.SNP).bits.fromRepl      := true.B

  /*
   * Send Replace Req to WriteCN
   */
  io.cmTaskVec(CMID.WRI).valid                      := reg.isWrite
  io.cmTaskVec(CMID.WRI).bits                       := DontCare
  io.cmTaskVec(CMID.WRI).bits.chi.nodeId            := DontCare
  io.cmTaskVec(CMID.WRI).bits.chi.channel           := ChiChannel.REQ
  io.cmTaskVec(CMID.WRI).bits.chi.opcode            := Mux(reg.repl.toLan, WriteNoSnpFull, Mux(reg.dir.llc.meta.isDirty, WriteBackFull, WriteEvictOrEvict))
  io.cmTaskVec(CMID.WRI).bits.chi.dataVec           := Seq(true.B, true.B)
  io.cmTaskVec(CMID.WRI).bits.chi.memAttr.allocate  := false.B
  io.cmTaskVec(CMID.WRI).bits.chi.memAttr.device    := false.B
  io.cmTaskVec(CMID.WRI).bits.chi.memAttr.cacheable := true.B
  io.cmTaskVec(CMID.WRI).bits.chi.memAttr.ewa       := true.B
  io.cmTaskVec(CMID.WRI).bits.chi.toLAN             := reg.repl.toLan
  io.cmTaskVec(CMID.WRI).bits.hnTxnID               := reg.repl.hnTxnID
  io.cmTaskVec(CMID.WRI).bits.fromRepl              := true.B
  io.cmTaskVec(CMID.WRI).bits.ds                    := reg.ds
  io.cmTaskVec(CMID.WRI).bits.cbResp                := Mux(reg.repl.toLan, ChiResp.I, reg.dir.llc.meta.cbResp)
  io.cmTaskVec(CMID.WRI).bits.dataOp.repl           := true.B

  io.cmTaskVec.foreach(_.bits.qos := reg.qos)

  /*
   * Request DataBuffer
   */
  io.reqDB.valid                := reg.isReqDB
  io.reqDB.bits.hnTxnID         := reg.repl.hnTxnID
  io.reqDB.bits.dataVec         := DataVec.Full
  io.reqDB.bits.qos             := reg.qos

  /*
   * Replace HnTxnID in DataBlock
   */
  io.updHnTxnID.valid           := reg.isUpdHnTxnID
  io.updHnTxnID.bits.before     := reg.hnTxnID
  io.updHnTxnID.bits.next       := reg.repl.hnTxnID

  /*
   * Send Task to DataBlock
   */
  io.dataTask.valid             := reg.isSaveData
  io.dataTask.bits              := DontCare
  io.dataTask.bits.hnTxnID      := reg.hnTxnID
  io.dataTask.bits.dataOp.save  := true.B
  io.dataTask.bits.dataVec      := DataVec.Full
  io.dataTask.bits.ds           := reg.ds
  io.dataTask.bits.qos          := reg.qos

  /*
   * Clean PoS
   */
  io.cleanPoS.valid             := reg.isCleanPoS
  io.cleanPoS.bits.hnIdx        := Mux(reg.isCleanPosT, reg.getHnIdx, reg.repl.getHnIdx)
  io.cleanPoS.bits.channel      := Mux(reg.isCleanPosT, ChiChannel.SNP, Mux(reg.isReplSF, ChiChannel.SNP, ChiChannel.REQ))
  io.cleanPoS.bits.qos          := reg.qos

  /*
   * Send Resp to Commit
   */
  io.resp.valid                 := reg.isRespCmt
  io.resp.bits.hnTxnID          := reg.hnTxnID

  /*
   * Get Next State
   */
  val cmRespData  = io.cmResp.bits.taskInst.valid   & io.cmResp.bits.taskInst.channel === ChiChannel.DAT
  val cmRespHit   = reg.isValid & io.cmResp.valid   & io.cmResp.bits.hnTxnID === reg.repl.hnTxnID
  val dataRespHit = reg.isValid & io.dataResp.valid & io.dataResp.bits.hnTxnID === reg.hnTxnID
  switch(reg.state) {
    // Free
    is(FREE) {
      when(io.alloc.fire)               { next.state := Mux(io.alloc.bits.isReplDIR, REQPOS, WRIDIR) }
    }
    // Request PoS
    is(REQPOS) {
      when(io.reqPoS.req.fire)          { next.state := WRIDIR }
    }
    // Write Directory
    is(WRIDIR) {
      when(io.writeDir.fire)            { next.state := Mux(reg.isReplDIR, WAITDIR, RESPCMT) }
    }
    // Wait Directory write response
    is(WAITDIR) {
      when(dirRespHit)                  { next.state := Mux(sfRespHit, RESPCMT, Mux(needReplLLC, UPDATEID, SAVEDATA)) }
    }
    // Update HnTxnID in DataBlock
    is(UPDATEID) {
      when(io.updHnTxnID.fire)          { next.state := WRITE }
    }
    // Send resp to Commit
    is(RESPCMT) {
      when(io.resp.fire)                { next.state := Mux(reg.isReplSF, Mux(reg.needSnp, REQDB, CLEANPOSR), Mux(reg.isReplLLC, CLEANPOSR, FREE)) }
    }
    // Request DataBuffer
    is(REQDB) {
      when(io.reqDB.fire)               { next.state := SNOOP }
    }
    // Send replace task to WriteCM
    is(WRITE) {
      when(io.cmTaskVec(CMID.WRI).fire) { next.state := WAITRWRI }
    }
    // Send replace task to SnoopCM
    is(SNOOP) {
      when(io.cmTaskVec(CMID.SNP).fire) { next.state := WAITRSNP }
    }
    // Wait WriteCM response
    is(WAITRWRI) {
      when(cmRespHit)                   { next.state := Mux(reg.alrReplSF, CLEANPOST, RESPCMT) }
    }
    // Wait SnoopCM response
    is(WAITRSNP) {
      when(cmRespHit)                   { next.state := Mux(cmRespData, COPYID, CLEANPOSR) }
    }
    // Copy repl.hnTxnID to task.hnTxnID in ReplaceCM
    is(COPYID) {
      when(true.B)                      { next.state := REQPOS }
    }
    // Send DataTask to DataBlock
    is(SAVEDATA) {
      when(io.dataTask.fire)            { next.state := WAITRESP }
    }
    // Wait DataBlock response
    is(WAITRESP) {
      when(dataRespHit)                 { next.state := Mux(reg.alrReplSF, CLEANPOST, RESPCMT) }
    }
    // Clean PoS
    is(CLEANPOST) {
      when(io.cleanPoS.fire)            { next.state := CLEANPOSR }
    }
    // Clean PoS
    is(CLEANPOSR) {
      when(io.cleanPoS.fire)            { next.state := FREE }
    }
  }

  /*
   * HAssert
   */
  HAssert.withEn(!(reg.isReplSF & reg.isReplLLC), reg.isValid)
  HAssert.withEn(reg.isFree,        io.alloc.fire)
  HAssert.withEn(reg.isReqPoS,      reg.isValid & io.reqPoS.req.fire)
  HAssert.withEn(reg.isWriDir,      reg.isValid & io.writeDir.fire)
  HAssert.withEn(reg.isWaitDir,     reg.isValid & dirRespHit)
  HAssert.withEn(reg.isUpdHnTxnID,  reg.isValid & io.updHnTxnID.fire)
  HAssert.withEn(reg.isReqDB,       reg.isValid & io.reqDB.fire)
  HAssert.withEn(reg.isReplSF,      reg.isValid & io.cmTaskVec(CMID.SNP).fire)
  HAssert.withEn(reg.isReplLLC,     reg.isValid & io.cmTaskVec(CMID.WRI).fire)
  HAssert.withEn(reg.isWaitRepl,    reg.isValid & cmRespHit)
  HAssert.withEn(reg.isSaveData,    reg.isValid & io.dataTask.fire)
  HAssert.withEn(reg.isCleanPoS,    reg.isValid & io.cleanPoS.fire)
  // TODO: HAssert.withEn(reg.isWaitWrite | reg.isWaitResp, reg.isValid & dataRespHit)


  /*
   * Get new write directory message
   */
  val cmRespDirty               = io.cmResp.bits.taskInst.resp.asTypeOf(new ChiResp).passDirty
  when(cmRespHit & cmRespData) {
    next.wriSF              := false.B
    next.wriLLC             := true.B
    next.dir.llc.hit        := false.B
    next.dir.llc.meta.state := Mux(cmRespDirty, ChiState.UD, ChiState.SC)
    HAssert(reg.isReplSF)
  }

  /*
   * Save snoop filter response to determine the snoop target
   */
  when(sfRespHit) {
    next.dir.sf             := io.respDir.sf.bits
  }

  /*
   * Set new task
   */
  val set = io.alloc.fire | reg.isValid; dontTouch(set)
  when(set) {
    reg       := Mux(io.alloc.fire, alloc, next)
    reg.state := next.state
  }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(reg.isFree, TIMEOUT_REPLACE, cf"TIMEOUT: Replace State[${reg.state}]")
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
    val updPosTag       = Valid(new Addr with HasAddrValid with HasPackHnIdx)
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
    val reqDB           = Decoupled(new HnTxnID with HasDataVec)
    val updHnTxnID      = Valid(new UpdHnTxnID)
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
    selIdVec(i) := StepRREncoder(hitVec, true.B) // TODO: has risk of StepRREncoder enable
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
  fastQosRRArb(entries.map(_.io.reqDB), io.reqDB)
  io.resp         := fastRRArb.validOut(entries.map(_.io.resp))
  io.updHnTxnID   := fastRRArb.validOut(entries.map(_.io.updHnTxnID))
  io.updPosTag    := fastArb(entries.map(_.io.updPosTag))
  io.cleanPoS     <> fastQosRRArb(entries.map(_.io.cleanPoS))
  io.dataTask     <> fastQosRRArb(entries.map(_.io.dataTask))
  io.writeDir     <> fastRRArb(entries.map(_.io.writeDir)) // TODO: use fastQosRRArb
  HAssert(PopCount(entries.map(_.io.updPosTag.valid)) <= 1.U)
  // Send cmTask
  io.cmTaskVec.zip(entries.map(_.io.cmTaskVec).transpose).foreach { case(a, b) => a <> fastQosRRArb(b) }


  /*
   * HAssert placePipe
   */
  HAssert.placePipe(1)
}