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
  val width     = 5
  val FREE      = 0x0.U
  val REQPOS    = 0x1.U
  val WRIDIR    = 0x2.U
  val WAITDIR   = 0x3.U
  val CUTID     = 0x4.U   // Cut HnTxnID in DataBlock
  val REPLLLC   = 0x5.U   // Send CM Task to SnoopCM
  val REPLSF    = 0x6.U   // Send CM Task to WriteCM
  val WAITRWRI  = 0x7.U   // Wait replace LLC done
  val WAITRSNP  = 0x8.U   // Wait replace SF done
  val COPYID    = 0x9.U   // Copy repl.hnTxnID to task.hnTxnID in ReplaceCM
  val SAVEDATA  = 0xA.U   // Send save and clean to DataBlock
  val CLEANDATA = 0xB.U   // Send clean to DataBlock
  val WAITRESP0 = 0xC.U   // Wait DataBlock Resp
  val WAITRESP1 = 0xD.U   // Wait DataBlock Resp
  val CLEANPOS0 = 0xE.U   // Clean PoS(task.hnTxnID)
  val CLEANPOS1 = 0xF.U   // Clean PoS(repl.hnTxnID)
  val CLEANPOS2 = 0x10.U  // Clean PoS(repl.hnTxnID)
}

trait HasReplMes { this: DJBundle =>
  // 0. Replace LLC Directory:
  // State: Free -> ReqPoS -> WriDir -> WaitDir ---> CutHnTxnID -> ReplLLC -> WaitWri -> (CleanPoS0) -> CleanPoS1 -> Free
  //                   ^                         |                                             ^
  //                   |                         |-(replWayIsInvalid)-> SaveData -> WaitResp0 -|
  //                   |
  //                   ------- CopyHnTxnID <-------(SnpRespWithData)-----
  //                                                                    |
  // 1. Replace SnoopFilter:                                            |
  // State: Free -> ReqPoS -> WriDir -> WaitDir ---> ReplSF -> WaitSnp ---> CleanData -> WaitResp1 -> CleanPoS2 -> Free
  //                                             |                                                        ^
  //                                             |--------------------(replWayIsInvalid)------------------|
  // 2. No need replace:
  // State: Free -> WriDir -> Free
  //
  // Node: only do CleanPoS0 when alrReplSF

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
  def isCutHnTxnID  = state === CUTID
  def isReplLLC     = state === REPLLLC
  def isReplSF      = state === REPLSF
  def isWaitWrite   = state === WAITRWRI
  def isWaitSnp     = state === WAITRSNP
  def isWaitRepl    = isWaitWrite | isWaitSnp
  def isCopyID      = state === COPYID
  def isSaveData    = state === SAVEDATA
  def isCleanData   = state === CLEANDATA
  def isDataTask    = isSaveData | isCleanData
  def isWaitResp0   = state === WAITRESP0
  def isWaitResp1   = state === WAITRESP1
  def isWaitResp    = isWaitResp0 | isWaitResp1
  def isCleanPoS0   = state === CLEANPOS0
  def isCleanPoS1   = state === CLEANPOS1
  def isCleanPoS2   = state === CLEANPOS2
  def isCleanPoS    = isCleanPoS0 | isCleanPoS1 | isCleanPoS2
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
    val cutHnTxnID      = Decoupled(new CutHnTxnID)
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
  val taskReg   = RegInit((new ReplTask with HasReplMes).Lit(_.state -> FREE, _.sResp -> false.B, _.alrReplSF -> false.B))
  val taskAlloc = WireInit(0.U.asTypeOf(taskReg))
  val taskNext  = WireInit(taskReg)

  /*
   * Ouput debug message
   */
  io.dbg.valid              := taskReg.isValid
  // task HnTxnID
  io.dbg.bits.task.hnTxnID  := taskReg.hnTxnID
  io.dbg.bits.task.hnIdx    := taskReg.getHnIdx
  // repl HnTxnID
  io.dbg.bits.repl.hnTxnID  := taskReg.repl.hnTxnID
  io.dbg.bits.repl.hnIdx    := taskReg.repl.getHnIdx

  /*
   * Receive Replace Task
   */
  io.alloc.ready            := taskReg.isFree
  when(io.alloc.fire) {
    taskAlloc.dir           := io.alloc.bits.dir
    taskAlloc.wriSF         := io.alloc.bits.wriSF
    taskAlloc.wriLLC        := io.alloc.bits.wriLLC
    taskAlloc.hnTxnID       := io.alloc.bits.hnTxnID
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

  HAssert.withEn(taskReg.replDIR, io.reqPoS.req.valid)

  /*
   * Set HnTxnID:
   *
   *  from alloc            -> 0
   *  from PoS first time   -> 1
   *  from PoS second time  -> 2
   *  Node: alrReplSF indicate it from PoS second time
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
  when(taskReg.isCopyID & taskReg.alrSResp) {
    taskNext.hnTxnID      := taskReg.repl.hnTxnID
  // ReqPoS
  }.elsewhen(io.reqPoS.req.fire) {
    taskNext.repl.hnTxnID := io.reqPoS.resp.hnTxnID
  }

  /*
   * Write Directory
   */
  io.writeDir.valid                 := taskReg.isWriDir
  // llc
  io.writeDir.bits.llc.valid        := taskReg.wriLLC
  io.writeDir.bits.llc.bits.addr    := taskReg.hnTxnID // remap in DongJiang
  io.writeDir.bits.llc.bits.wayOH   := taskReg.dir.llc.wayOH
  io.writeDir.bits.llc.bits.hit     := taskReg.dir.llc.hit
  io.writeDir.bits.llc.bits.metaVec := taskReg.dir.llc.metaVec
  io.writeDir.bits.llc.bits.hnIdx   := taskReg.repl.getHnIdx
  // sf
  io.writeDir.bits.sf.valid         := taskReg.wriSF
  io.writeDir.bits.sf.bits.addr     := taskReg.hnTxnID  // remap in DongJiang
  io.writeDir.bits.sf.bits.wayOH    := taskReg.dir.sf.wayOH
  io.writeDir.bits.sf.bits.hit      := taskReg.dir.sf.hit
  io.writeDir.bits.sf.bits.metaVec  := taskReg.dir.sf.metaVec
  io.writeDir.bits.sf.bits.hnIdx    := taskReg.repl.getHnIdx
  // HAssert
  HAssert.withEn(io.writeDir.bits.llc.bits.hit, io.writeDir.valid & io.writeDir.bits.llc.valid & io.writeDir.bits.llc.bits.meta.isInvalid)
  HAssert.withEn(io.writeDir.bits.sf.bits.hit,  io.writeDir.valid & io.writeDir.bits.sf.valid  & Cat(io.writeDir.bits.sf.bits.allVec) === 0.U)

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
  io.updPosTag.valid        := dirRespHit // Even when needReplSF or needReplLLC is false, updTag must still be sent to unlock the PoS Lock.
  io.updPosTag.bits.addrVal := Mux(io.respDir.sf.valid, io.respDir.sf.bits.metaIsVal, io.respDir.llc.bits.metaIsVal)
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
  io.cutHnTxnID.valid           := taskReg.isCutHnTxnID
  io.cutHnTxnID.bits.before     := taskReg.hnTxnID
  io.cutHnTxnID.bits.next       := taskReg.repl.hnTxnID

  /*
   * Send Task to DataBlock
   */
  io.dataTask.valid             := taskReg.isDataTask
  io.dataTask.bits              := DontCare
  io.dataTask.bits.hnTxnID      := Mux(taskReg.isSaveData, taskReg.hnTxnID, taskReg.repl.hnTxnID)
  io.dataTask.bits.dataOp.save  := taskReg.isSaveData
  io.dataTask.bits.dataOp.clean := true.B
  io.dataTask.bits.dataVec      := DataVec.Full
  io.dataTask.bits.ds           := taskReg.repl.ds

  /*
   * Clean PoS
   */
  io.cleanPoS.valid         := taskReg.isCleanPoS
  io.cleanPoS.bits.hnIdx    := Mux(taskReg.isCleanPoS0, taskReg.getHnIdx, taskReg.repl.getHnIdx)
  io.cleanPoS.bits.channel  := Mux(taskReg.isCleanPoS1, ChiChannel.REQ,   ChiChannel.SNP)

  /*
   * Send Resp to Commit
   */
  io.resp.valid         := taskReg.sResp
  io.resp.bits.hnTxnID  := taskReg.hnTxnID
  HAssert.withEn(!taskReg.alrSResp, io.resp.valid )

  /*
   * Get Next State
   */
  val cmRespData  = io.cmResp.bits.taskInst.valid       & io.cmResp.bits.taskInst.channel === ChiChannel.DAT
  val cmRespHit   = taskReg.isValid & io.cmResp.valid   & io.cmResp.bits.hnTxnID === taskReg.repl.hnTxnID
  val dataRespHit = taskReg.isValid & io.dataResp.valid & io.dataResp.bits.hnTxnID === Mux(taskReg.isWaitResp0, taskReg.hnTxnID, taskReg.repl.hnTxnID)
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
      when(dirRespHit)                  { taskNext.state := Mux(sfRespHit, Mux(needReplSF, REPLSF, CLEANPOS2), Mux(needReplLLC, CUTID, SAVEDATA)) }
    }
    // Cut HnTxnID in DataBlock
    is(CUTID) {
      when(io.cutHnTxnID.fire)          { taskNext.state := REPLLLC }
    }
    // Send replace task to WriteCM
    is(REPLLLC) {
      when(io.cmTaskVec(CMID.WRI).fire) { taskNext.state := WAITRWRI }
    }
    // Send replace task to SnoopCM
    is(REPLSF) {
      when(io.cmTaskVec(CMID.SNP).fire) { taskNext.state := WAITRSNP }
    }
    // Wait WriteCM response
    is(WAITRWRI) {
      when(cmRespHit)                   { taskNext.state := Mux(taskReg.alrReplSF, CLEANPOS0, CLEANPOS1) }
    }
    // Wait SnoopCM response
    is(WAITRSNP) {
      when(cmRespHit)                   { taskNext.state := Mux(cmRespData, COPYID, CLEANDATA) }
    }
    // Copy repl.hnTxnID to task.hnTxnID in ReplaceCM
    is(COPYID) {
      when(taskReg.alrSResp)            { taskNext.state := REQPOS }
    }
    // Send DataTask to DataBlock
    is(SAVEDATA) {
      when(io.dataTask.fire)            { taskNext.state := WAITRESP0 }
    }
    // Send DataTask to DataBlock
    is(CLEANDATA) {
      when(io.dataTask.fire)            { taskNext.state := WAITRESP1 }
    }
    // Wait DataBlock response 0
    is(WAITRESP0) {
      when(dataRespHit)                 { taskNext.state := Mux(taskReg.alrReplSF, CLEANPOS0, CLEANPOS1) }
    }
    // Wait DataBlock response 1
    is(WAITRESP1) {
      when(dataRespHit)                 { taskNext.state := CLEANPOS2 }
    }
    // Clean PoS 0
    is(CLEANPOS0) {
      when(io.cleanPoS.fire)            { taskNext.state := CLEANPOS1 }
    }
    // Clean PoS 1
    is(CLEANPOS1) {
      when(io.cleanPoS.fire)            { taskNext.state := FREE }
    }
    // Clean PoS 2
    is(CLEANPOS2) {
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
  HAssert.withEn(taskReg.isCutHnTxnID,  taskReg.isValid & io.cutHnTxnID.fire)
  HAssert.withEn(taskReg.isReplSF,      taskReg.isValid & io.cmTaskVec(CMID.SNP).fire)
  HAssert.withEn(taskReg.isReplLLC,     taskReg.isValid & io.cmTaskVec(CMID.WRI).fire)
  HAssert.withEn(taskReg.isWaitRepl,    taskReg.isValid & cmRespHit)
  HAssert.withEn(taskReg.isDataTask,    taskReg.isValid & io.dataTask.fire)
  HAssert.withEn(taskReg.isCleanPoS,    taskReg.isValid & io.cleanPoS.fire)
  HAssert.withEn(taskReg.isWaitRepl | taskReg.isWaitResp, taskReg.isValid & dataRespHit)

  /*
  * Get next sResp
  * ----------------------
  * | wSF | wLLC | sResp |
  * |--------------------|
  * |  I  |  I   |   T   |
  * |  V  |  I   |   T   |
  * |  I  |  V   |   F   |
  * |  V  |  V   |   X   |
  * ----------------------
  * Node:
  *  The above table applies only to cases where substitutions may be required
  *  X: This does not happen with the Exclusive policy
  */
  when(io.resp.fire) {
    taskNext.sResp    := false.B
    taskNext.alrSResp := true.B
    // HAssert
    HAssert(taskReg.sResp)
    HAssert(!taskReg.alrSResp)
  // Send resp no need wait replace task done
  }.elsewhen(io.writeDir.fire) {
    val wriSFVal      = io.writeDir.bits.sf.valid  & io.writeDir.bits.sf.bits.metaIsVal
    val wriLLCVal     = io.writeDir.bits.llc.valid & io.writeDir.bits.llc.bits.metaIsVal
    taskNext.sResp    := PriorityMux(Seq(
      (taskNext.state === FREE) -> true.B,  // No need replace
      taskReg.alrSResp          -> false.B, // already send resp
      (!wriSFVal & !wriLLCVal)  -> true.B,
      ( wriSFVal & !wriLLCVal)  -> true.B,
      (!wriSFVal &  wriLLCVal)  -> false.B
    ))
    // HAssert
    HAssert(!(wriSFVal & wriLLCVal))
    HAssert.withEn(!taskReg.sResp,    taskNext.sResp)
    HAssert.withEn(!taskReg.alrSResp, taskNext.sResp)
  // Replace task done
  }.elsewhen(io.cleanPoS.fire) {
    taskNext.sResp    := !taskReg.alrSResp
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
    val cutHnTxnID      = Valid(new CutHnTxnID)
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
  io.resp         := fastRRArb.validOut(entries.map(_.io.resp))
  io.cutHnTxnID   := fastRRArb.validOut(entries.map(_.io.cutHnTxnID))
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