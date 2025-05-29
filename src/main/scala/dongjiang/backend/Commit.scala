package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, HasPackDirMsg}
import dongjiang.data._
import zhujiang.chi.ReqOpcode._
import dongjiang.frontend.decode._
import dongjiang.frontend.PosClean
import dongjiang.backend.CmtState._
import chisel3.experimental.BundleLiterals._
import dongjiang.frontend.decode.Decode.{w_ci, w_si, w_sti, w_ti}

// ----------------------------------------------------------------------------------------------------- //
// ------------------------------------------- Commit Flag --------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object CmtState {
  val width     = 3
  val FREE      = 0.U
  val FSTTASK   = 1.U // Doing first task (Snoop, Read, Dataless, Write, Receive)
  val SECTASK   = 2.U // Doing secone task(Snoop, Read, Dataless, Write)
  val COMMIT    = 3.U // Doing commit
  val CLEAN     = 4.U
}

class Flag(implicit p: Parameters) extends DJBundle {
  val intl = new DJBundle {
    // internal send
    val s = new DJBundle {
      val reqDB     = Bool() // Send Request DataBuffer to DataBlock
      val cmTask    = Bool() // Send Task To TaskCM
      val dataTask  = Bool() // Send Data Task(read, send, save)
      val wriDir    = Bool() // Send Write Task To Replace   // Note: Need wait data done
      // TODO: val dataFwd = Bool() // Send CompData in SnpFwd
    }
    // internal wait
    val w = new DJBundle {
      val recResp   = Bool() // Wait ReceiveCM(Write) Done
      val cmResp    = Bool() // Wait TaskCM Resp
      val replResp  = Bool() // Wait Replace Done
      val dataResp  = Bool() // Wait Data Task(read, send, save) Done
    }
  }
  // chi send/wait
  val chi = new DJBundle {
    val s_resp      = Bool() // Send Resp(Comp, SnpResp) To RN/HN
    val w_ack       = Bool() // Wait CompAck From RN
  }
}

class State(implicit p: Parameters) extends DJBundle {
  // Free -> Set -> FstTask -> SecTask -> Commit -> Clean -> Free
  val value     = UInt(CmtState.width.W)
  def isFree    = value === FREE
  def isFstTask = value === FSTTASK
  def isSecTask = value === SECTASK
  def isCommit  = value === COMMIT
  def isClean   = value === CLEAN
}


// ----------------------------------------------------------------------------------------------------- //
// ------------------------------------------- Commit Entry -------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class CommitEntry(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new DJBundle {
    val config      = new DJConfigIO
    val hnTxnID     = Input(UInt(hnTxnIDBits.W))
    val hnIdx       = Input(new HnIndex)
    // Commit Task In
    val alloc       = Flipped(Valid(new CommitTask with HasHnTxnID)) // broadcast signal
    // Decode List
    val trdInst     = new TaskInst
    val fthInst     = new TaskInst
    val decListOut  = Output(MixedVec(UInt(w_ci.W), UInt(w_si.W), UInt(w_ti.W), UInt(w_sti.W)))
    val decListIn   = Flipped(Valid(MixedVec(UInt(w_ci.W), UInt(w_si.W), UInt(w_ti.W), UInt(w_sti.W))))
    // Decode Result
    val taskCode    = Input(new TaskCode)
    val cmtCode     = Input(new CommitCode)
    // CHI
    val txRsp       = Decoupled(new RespFlit())
    val rxRsp       = Flipped(Valid(new RespFlit())) // Receive CompAck
    val rxDat       = Flipped(Valid(new DataFlit())) // Receive NCBWrDataCompAck
    // Send Task To TaskCM
    val cmTaskVec   = Vec(nrTaskCM, Decoupled(new CMTask))
    // Resp From TaskCM or ReceiveCM
    val cmResp      = Flipped(Valid(new CMResp))  // broadcast signal
    // Send Task To Replace
    val replTask    = Decoupled(new ReplTask)
    val replResp    = Flipped(Valid(new HnTxnID)) // broadcast signal
    // Send Task To Data
    val reqDB       = Decoupled(new HnTxnID with HasDataVec)
    val dataTask    = Decoupled(new DataTask)
    val dataResp    = Flipped(Valid(new HnTxnID)) // broadcast signal
    // Update PoS Message
    val cleanPoS    = Decoupled(new PosClean)
    // State
    val state       = Output(new State)
  })
  HAssert.withEn(io.alloc.bits.dirBank === io.hnIdx.dirBank, io.alloc.valid)

  /*
   * Reg and Wire declaration
   */
  // task
  val taskReg     = Reg(new CommitTask)
  val taskNext    = WireInit(taskReg)
  val taskAlloc   = Wire(new CommitTask)
  taskAlloc       := io.alloc.bits
  // flag
  val flagReg     = Reg(new Flag)
  val flagNext    = WireInit(flagReg)
  // state
  val stateReg    = RegInit(new State().Lit(_.value -> FREE))
  val stateNext   = WireInit(stateReg)
  val valid       = WireInit(!stateReg.isFree)
  io.state        := stateReg
  // inst
  val instReg     = RegInit(new TaskInst().Lit(_.valid -> false.B))
  val instNext    = WireInit(instReg)
  // other
  val cmTask      = WireInit(0.U.asTypeOf(new CMTask))
  val rspAckHit   = valid & io.rxRsp.valid & io.rxRsp.bits.TxnID === io.hnTxnID & io.rxRsp.bits.Opcode === CompAck
  val datAckHit   = valid & io.rxDat.valid & io.rxDat.bits.TxnID === io.hnTxnID & io.rxDat.bits.Opcode === NCBWrDataCompAck
  val compAckHit  = rspAckHit | datAckHit

  /*
   * Request DataBuffer
   */
  io.reqDB.valid        := valid & flagReg.intl.s.reqDB
  io.reqDB.bits.hnTxnID := io.hnTxnID
  io.reqDB.bits.dataVec := Mux(flagReg.intl.s.cmTask & taskReg.task.snoop, DataVec.Full, taskReg.chi.dataVec)

  /*
   * Send DataTask
   */
  // valid & init
  io.dataTask.valid               := valid & flagReg.intl.s.dataTask & taskReg.alr.reqDB
  io.dataTask.bits                := DontCare
  // dataOp
  io.dataTask.bits.dataOp         := taskReg.cmt.dataOp
  io.dataTask.bits.dataOp.save    := taskReg.cmt.dataOp.save & !flagReg.intl.s.wriDir
  HAssert.withEn(PopCount(io.dataTask.bits.dataOp.asUInt) =/= 0.U, io.dataTask.valid)
  // txDat
  // TODO: SnpRespData
  io.dataTask.bits.txDat.DBID     := io.hnTxnID
  io.dataTask.bits.txDat.Resp     := taskReg.cmt.resp
  io.dataTask.bits.txDat.Opcode   := taskReg.cmt.opcode
  io.dataTask.bits.txDat.HomeNID  := DontCare // remap in SAM
  io.dataTask.bits.txDat.TxnID    := taskReg.chi.txnID
  io.dataTask.bits.txDat.SrcID    := taskReg.chi.getNoC
  io.dataTask.bits.txDat.TgtID    := taskReg.chi.nodeId
  // other bits
  io.dataTask.bits.ds             := taskReg.ds
  io.dataTask.bits.hnTxnID        := io.hnTxnID
  io.dataTask.bits.dataVec        := Mux(taskReg.cmt.dataOp.save & taskReg.alr.getSnpData, DataVec.Full, taskReg.chi.dataVec)

  /*
   * Send write directory task to ReplaceCM
   */
  // valid and hnTxnID
  io.replTask.valid               := valid & flagReg.intl.s.wriDir & !flagReg.intl.w.dataResp
  io.replTask.bits.hnTxnID        := io.hnTxnID
  // write sf bits
  io.replTask.bits.wriSF          := taskReg.cmt.isWriSF
  io.replTask.bits.dir.sf.hit     := taskReg.dir.sf.hit
  io.replTask.bits.dir.sf.wayOH   := taskReg.dir.sf.wayOH
  io.replTask.bits.dir.sf.metaVec.map(_.state).zipWithIndex.foreach {
    case(s, i) =>
      val metaIdOH    = taskReg.chi.metaIdOH
      val srcVec      = VecInit(metaIdOH.asBools)
      val snpVec      = taskReg.dir.getSnpVec(taskReg.task.snpTgt, metaIdOH)
      s := PriorityMux(Seq(
        (srcVec(i) & taskReg.cmt.wriSRC) -> taskReg.cmt.srcValid, // modify source state
        (snpVec(i) & taskReg.cmt.wriSNP) -> taskReg.cmt.snpValid, // modify snoopee state
        true.B                           -> (taskReg.dir.sf.hit & taskReg.dir.sf.metaVec(i).state)
      ))
  }
  // write llc bits
  io.replTask.bits.wriLLC         := taskReg.cmt.wriLLC
  io.replTask.bits.dir.llc.hit    := taskReg.dir.llc.hit
  io.replTask.bits.dir.llc.wayOH  := taskReg.dir.llc.wayOH
  io.replTask.bits.dir.llc.metaVec.head.state := taskReg.cmt.llcState

  /*
   * Send second CMTask to TaskCM
   */
  // chi
  cmTask.chi                      := taskReg.chi
  cmTask.chi.channel              := Mux(taskReg.task.snoop, ChiChannel.SNP, ChiChannel.REQ)
  cmTask.chi.dataVec              := Mux(taskReg.task.snoop | taskReg.alr.getSnpData, DataVec.Full, taskReg.chi.dataVec)
  cmTask.chi.opcode               := taskReg.task.opcode
  cmTask.chi.expCompAck           := taskReg.task.expCompAck
  cmTask.chi.retToSrc             := taskReg.task.retToSrc
  // other
  cmTask.hnTxnID                  := io.hnTxnID
  cmTask.dataOp                   := taskReg.task.dataOp
  cmTask.ds                       := taskReg.ds
  cmTask.snpVec                   := taskReg.dir.getSnpVec(taskReg.task.snpTgt, taskReg.chi.metaIdOH)
  cmTask.fromRepl                 := false.B
  cmTask.cbResp                   := taskReg.dir.llc.metaVec.head.cbResp
  cmTask.doDMT                    := taskReg.task.doDMT
  // alloc
  io.cmTaskVec(CMID.SNP).valid    := valid & flagReg.intl.s.cmTask & taskReg.task.snoop
  io.cmTaskVec(CMID.READ).valid   := valid & flagReg.intl.s.cmTask & taskReg.task.read
  io.cmTaskVec(CMID.DL).valid     := valid & flagReg.intl.s.cmTask & taskReg.task.dataless
  io.cmTaskVec(CMID.WRI).valid    := valid & flagReg.intl.s.cmTask & taskReg.task.write
  io.cmTaskVec.foreach(_.bits     := cmTask)
  HAssert(PopCount(io.cmTaskVec.map(_.fire)) <= 1.U)

  /*
   * Send CHI.RSP
   */
  // valid
  io.txRsp.valid            := valid & flagReg.chi.s_resp
  // bits
  io.txRsp.bits             := DontCare
  io.txRsp.bits.DBID        := io.hnTxnID
  io.txRsp.bits.FwdState    := taskReg.cmt.fwdResp
  io.txRsp.bits.Resp        := taskReg.cmt.resp
  io.txRsp.bits.Opcode      := taskReg.cmt.opcode
  io.txRsp.bits.TxnID       := taskReg.chi.txnID
  io.txRsp.bits.SrcID       := taskReg.chi.getNoC
  io.txRsp.bits.TgtID       := taskReg.chi.nodeId


  /*
   * Clean PoS
   */
  // valid
  io.cleanPoS.valid         := valid & stateReg.isClean
  // bits
  io.cleanPoS.bits.hnIdx    := io.hnIdx
  io.cleanPoS.bits.channel  := taskReg.chi.channel

  /*
   * Decode when get resp
   *
   * Node:
   *  1st decode after get task from CHI
   *  2nd decode after Decode completes, execute result in backend
   *  3rd decode after Commit gets TaskCM & ReceiveCM response
   *  4th decode after Commit gets TaskCM response again
   */
  // Output
  io.trdInst          := instReg
  io.trdInst.valid    := stateReg.isFstTask & (flagReg.intl.w.recResp ^ flagReg.intl.w.cmResp)
  io.fthInst          := 0.U.asTypeOf(new TaskInst)
  io.fthInst.valid    := stateReg.isSecTask
  io.decListOut       := taskReg.decList
  // Input
  when(io.decListIn.valid) {
    taskNext.decList  := io.decListIn.bits
    taskNext.task     := io.taskCode
    taskNext.cmt      := Mux(stateReg.isFstTask & io.cmtCode.waitSecDone, 0.U.asTypeOf(new CommitCode), io.cmtCode)
    // HAssert state
    HAssert(instReg.valid)
    HAssert(stateReg.isFstTask | stateReg.isSecTask)
    // HAssert decList
    HAssert(taskReg.decList(0) === io.decListIn.bits(0))
    HAssert(taskReg.decList(1) === io.decListIn.bits(1))
    HAssert.withEn(taskReg.decList(2) === io.decListIn.bits(2), !stateReg.isFstTask)
    HAssert.withEn(taskReg.decList(3) === io.decListIn.bits(3), !stateReg.isSecTask)
    // HAssert code
    HAssert(taskReg.task.isValid)
    HAssert(!taskReg.cmt.isValid)
    HAssert.withEn(!io.taskCode.snoop, stateReg.isSecTask)
    HAssert.withEn(!io.taskCode.waitRecDone, stateReg.isSecTask)
  }

  /*
   * Get next flag
   */
  val allocHit  = io.alloc.fire & io.alloc.bits.hnTxnID === io.hnTxnID
  val cmRespHit = valid & io.cmResp.fire & io.cmResp.bits.hnTxnID === io.hnTxnID
  // Alloc or Decode done
  when(allocHit | io.decListIn.valid) {
    val task                  = Mux(io.decListIn.valid, taskNext.task,      io.alloc.bits.task)
    val cmt                   = Mux(io.decListIn.valid, taskNext.cmt,       io.alloc.bits.cmt)
    val alrReqDB              = Mux(io.decListIn.valid, taskReg.alr.reqDB,  io.alloc.bits.alr.reqDB)
    val alrSendData           = Mux(io.decListIn.valid, taskReg.alr.sData,  io.alloc.bits.alr.sData)
    val needWaitAck           = Mux(io.decListIn.valid, flagReg.chi.w_ack,  io.alloc.bits.chi.expCompAck & !io.alloc.bits.chi.reqIs(WriteEvictOrEvict))
     // task flag internal send
    flagNext.intl.s.reqDB     := (task.needDB | cmt.dataOp.isValid) & !alrReqDB
    flagNext.intl.s.cmTask    := task.opsIsValid
    flagNext.intl.s.dataTask  := Mux(cmt.dataOp.onlySave, !cmt.isWriDir, cmt.dataOp.isValid) & !alrSendData
    flagNext.intl.s.wriDir    := cmt.isWriDir
    // task flag internal wait
    flagNext.intl.w.recResp   := task.waitRecDone
    flagNext.intl.w.cmResp    := flagNext.intl.s.cmTask
    flagNext.intl.w.replResp  := flagNext.intl.s.wriDir
    flagNext.intl.w.dataResp  := flagNext.intl.s.dataTask | alrSendData
    // task flag chi send
    flagNext.chi.s_resp       := cmt.sendResp & cmt.channel === ChiChannel.RSP
    // task flag chi wait
    flagNext.chi.w_ack        := needWaitAck & !compAckHit
    // HAssert
    HAssert(allocHit ^ io.decListIn.valid)
    HAssert.withEn(!(task.isValid & cmt.isValid), allocHit)
    HAssert.withEn(flagReg.intl.asUInt === 0.U & !flagReg.chi.s_resp, io.decListIn.valid)
  // Running
  }.otherwise {
    // task flag internal send
    when(io.reqDB.fire)                     { flagNext.intl.s.reqDB     := false.B; HAssert(flagReg.intl.s.reqDB) }
    when(Cat(io.cmTaskVec.map(_.fire)).orR) { flagNext.intl.s.cmTask    := false.B; HAssert(flagReg.intl.s.cmTask   & flagReg.intl.w.cmResp)   }
    when(io.dataTask.fire)                  { flagNext.intl.s.dataTask  := false.B; HAssert(flagReg.intl.s.dataTask & flagReg.intl.w.dataResp) }
    when(io.replTask.fire)                  { flagNext.intl.s.wriDir    := false.B; HAssert(flagReg.intl.s.wriDir   & flagReg.intl.w.replResp) }
    // task flag internal wait
    when(cmRespHit &  io.cmResp.bits.fromRec)                         { flagNext.intl.w.recResp := false.B;  HAssert(flagReg.intl.w.recResp & valid)        }
    when(cmRespHit & !io.cmResp.bits.fromRec)                         { flagNext.intl.w.cmResp  := false.B;  HAssert.withEn(flagReg.intl.w.cmResp, valid)   }
    when(io.replResp.fire & io.replResp.bits.hnTxnID === io.hnTxnID)  { flagNext.intl.w.replResp := false.B; HAssert(flagReg.intl.w.replResp & valid)       }
    when(io.dataResp.fire & io.dataResp.bits.hnTxnID === io.hnTxnID)  { flagNext.intl.w.dataResp := false.B; HAssert.withEn(flagReg.intl.w.dataResp | flagReg.intl.w.replResp | flagReg.intl.w.cmResp, valid) }
    // task flag chi send
    when(io.txRsp.fire) { flagNext.chi.s_resp := false.B; HAssert(flagReg.chi.s_resp) }
    when(compAckHit)    { flagNext.chi.w_ack  := false.B; HAssert.withEn(flagReg.chi.w_ack | taskReg.chi.reqIs(WriteEvictOrEvict) & valid, rspAckHit)  }
  }

  // state
  val allFlagDone       = flagReg.intl.asUInt === 0.U & flagReg.chi.asUInt === 0.U
  switch(stateReg.value) {
    is(FREE) {
      when(allocHit)            { stateNext.value := Mux(io.alloc.bits.task.isValid, FSTTASK, COMMIT) }
    }
    is(FSTTASK) {
      when(io.decListIn.valid)  { stateNext.value := Mux(io.taskCode.isValid & io.cmtCode.waitSecDone, SECTASK, COMMIT) }
    }
    is(SECTASK) {
      when(io.decListIn.valid)  { stateNext.value := COMMIT }
    }
    is(COMMIT) {
      when(allFlagDone)       { stateNext.value := CLEAN }
    }
    is(CLEAN) {
      when(io.cleanPoS.fire)  { stateNext.value := FREE }
    }
  }
  HAssert.withEn(stateReg.isFree, allocHit)
  HAssert.withEn(stateReg.isFstTask | stateReg.isSecTask, io.decListIn.valid)
  HAssert.withEn(stateReg.isClean, io.cleanPoS.valid)


  /*
   * Get new task inst
   */
  when(allocHit) {
    instNext := 0.U.asTypeOf(new TaskInst)
  }.elsewhen(cmRespHit) {
    instNext := (instReg.asUInt | io.cmResp.bits.taskInst.asUInt).asTypeOf(new TaskInst)
    HAssert(io.cmResp.bits.taskInst.valid)
  }

  /*
   * Update already flag
   */
  // reqDB
  when(io.reqDB.fire) {
    taskNext.alr.reqDB := true.B
    HAssert(!taskReg.alr.reqDB)
  }
  // getSnpData
  when(cmRespHit) {
    val opcode  = io.cmResp.bits.taskInst.opcode
    val channel = io.cmResp.bits.taskInst.channel
    taskNext.alr.getSnpData := (opcode === SnpRespData | opcode === SnpRespDataFwded) & channel === ChiChannel.DAT
    HAssert.withEn(!taskReg.alr.getSnpData, taskNext.alr.getSnpData)
  }

  /*
   * Set new reg
   */
  val set = allocHit | valid; dontTouch(set)
  when(set) {
    taskReg   := Mux(allocHit, taskAlloc, taskNext)
    flagReg   := flagNext
    instReg   := instNext
    stateReg  := stateNext
  }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(!valid, TIMEOUT_COMMIT, cf"\n\nTIMEOUT[${io.hnTxnID}]: " +
    cf"\nInternal Send: ${flagReg.intl.s}\nInternal Wait: ${flagReg.intl.w}\n" +
    cf"CHI: ${flagReg.chi}\n${taskReg.chi.getChiInst}\n${taskReg.dir.getStateInst(taskReg.chi.metaIdOH)}\n\n")
}

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- Commit ---------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class Commit(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new DJBundle {
    val config      = new DJConfigIO
    // Commit Task In
    val cmtTaskVec  = Vec(djparam.nrDirBank, Flipped(Valid(new CommitTask with HasHnTxnID))) // broadcast signal
    // CHI
    val txRsp       = Decoupled(new RespFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit()))
    // Send Task To TaskCM
    val cmTaskVec   = Vec(nrTaskCM, Decoupled(new CMTask))
    // Resp From TaskCM or ReceiveCM
    val cmResp      = Flipped(Valid(new CMResp))  // broadcast signal
    // Send Task To Replace
    val reqDB       = Decoupled(new HnTxnID with HasDataVec)
    val replTask    = Decoupled(new ReplTask)
    val replResp    = Flipped(Valid(new HnTxnID)) // broadcast signal
    // Send Task To Data
    val dataTask    = Decoupled(new DataTask)
    val dataResp    = Flipped(Valid(new HnTxnID)) // broadcast signal
    // Update PoS Message
    val cleanPoS    = Decoupled(new PosClean)
  })

  /*
   * Module and Wire declaration
   */
  val entries = Seq.fill(djparam.nrPoS) { Module(new CommitEntry()) }
  val trdDec  = Module(new Decode("Third"))
  val fthDec  = Module(new Decode("Fourth"))

  /*
   * SuggestName of entries
   */
  def splitInt(value: Int, widths: Seq[Int]): Seq[Int] = {
    require(widths.forall(_ >= 0), "Widths must be non-negative")
    require(widths.sum <= 32, "Total width must ≤ 32")

    val totalWidth = widths.sum
    val maskedValue = if (totalWidth == 0) 0 else value & ((1 << totalWidth) - 1) // 处理全0情况

    widths.foldLeft((List.empty[Int], 0)) { case ((segments, shift), width) =>
      if (width == 0) {
        (segments :+ 0, shift) // 位宽为0，直接返回0
      } else {
        val segment = (maskedValue >> shift) & ((1 << width) - 1)
        (segments :+ segment, shift + width)
      }
    }._1
  }
  entries.zipWithIndex.foreach { case(e, i) =>
    val idx = splitInt(i, Seq(posWayBits, posSetBits, dirBankBits))
    e.suggestName(s"entries_${i}_bank${idx(2)}_set${idx(1)}_way${idx(0)}")
  }

  /*
   * Receive Commit Task from Frontend
   */
  entries.map(_.io.alloc).grouped(nrPoS).zip(io.cmtTaskVec).foreach { case(alloc, task) => alloc.foreach(_ := task) }

  /*
   * Decode Input
   */
  //
  val decIdIn           = io.cmResp.bits.hnTxnID
  // listIn
  trdDec.io.listIn      := VecInit(entries.map(_.io.decListOut))(decIdIn)
  fthDec.io.listIn      := VecInit(entries.map(_.io.decListOut))(decIdIn)
  // inst0
  trdDec.io.inst0       := io.cmResp.bits.taskInst
  fthDec.io.inst0       := io.cmResp.bits.taskInst
  trdDec.io.inst0.valid := io.cmResp.valid & io.cmResp.bits.taskInst.valid
  fthDec.io.inst0.valid := io.cmResp.valid & io.cmResp.bits.taskInst.valid
  // inst1
  trdDec.io.inst1       := VecInit(entries.map(_.io.trdInst))(decIdIn)
  fthDec.io.inst1       := VecInit(entries.map(_.io.fthInst))(decIdIn)
  // hnTxnIdIn
  trdDec.io.hnTxnIdIn   := decIdIn
  fthDec.io.hnTxnIdIn   := decIdIn

  /*
   * Decode Output
   */
  val decList   = Mux(trdDec.io.valid, trdDec.io.listOut,     fthDec.io.listOut)
  val taskCode  = Mux(trdDec.io.valid, trdDec.io.taskCode,    fthDec.io.taskCode)
  val cmtCode   = Mux(trdDec.io.valid, trdDec.io.cmtCode,     fthDec.io.cmtCode)
  val decIdOut  = Mux(trdDec.io.valid, trdDec.io.hnTxnIdOut,  fthDec.io.hnTxnIdOut)
  entries.zipWithIndex.foreach { case (e, i) =>
    e.io.decListIn.valid  := (trdDec.io.valid | fthDec.io.valid) & decIdOut === i.U
    e.io.decListIn.bits   := decList
    e.io.taskCode         := taskCode
    e.io.cmtCode          := cmtCode
  }
  HAssert(!(trdDec.io.valid & fthDec.io.valid))

  /*
   * Connect Commit <- IO
   */
  entries.zipWithIndex.foreach { case(e, i) =>
    e.io.config   := io.config
    e.io.hnTxnID  := i.U
    e.io.hnIdx    := i.U.asTypeOf(new HnTxnID).getHnIdx
    e.io.rxRsp    := io.rxRsp
    e.io.rxDat    := io.rxDat
    e.io.cmResp   := io.cmResp
    e.io.replResp := io.replResp
    e.io.dataResp := io.dataResp
  }

  /*
   * Connect IO <- Commit
   */
  io.reqDB    <> fastRRArb(entries.map(_.io.reqDB))
  io.txRsp    <> fastRRArb(entries.map(_.io.txRsp))
  io.replTask <> fastRRArb(entries.map(_.io.replTask))
  io.dataTask <> fastRRArb(entries.map(_.io.dataTask))
  io.cleanPoS <> fastRRArb(entries.map(_.io.cleanPoS))
  io.cmTaskVec.zip(entries.map(_.io.cmTaskVec).transpose).foreach { case(a, b) => a <> fastRRArb(b) }

  /*
   * HAssert placePipe
   */
  HAssert.placePipe(1)
}
