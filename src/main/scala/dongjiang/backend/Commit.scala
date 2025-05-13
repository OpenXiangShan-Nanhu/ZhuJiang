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
import dongjiang.frontend._
import dongjiang.frontend.decode._
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ------------------------------------------- Commit Flag --------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class Flag(implicit p: Parameters) extends DJBundle {
  val intl = new DJBundle {
    // internal send
    val s = new DJBundle {
      val data0     = Bool() // Send Data Task(reqs, read, send)
      val data1     = Bool() // Send Data Task(save, clean)  // Note: Need wait repl done
      val wriDir    = Bool() // Send Write Task To Replace   // Note: Need wait data0 done
      val secTask   = Bool() // Send Task To TaskCM
      // TODO: val dataFwd = Bool() // Send CompData in SnpFwd
    }
    // internal wait
    val w = new DJBundle {
      val receive   = Bool() // Wait ReceiveCM(Write) Done
      val cmResp    = Bool() // Wait TaskCM Resp
      val secResp   = Bool() // Wait Second TaskCM Resp
      val repl      = Bool() // Wait Replace Done
      val data0     = Bool() // Wait Data Task(read, send, send) Done
      val data1     = Bool() // Wait Data Task(save, clean) Done
    }
  }
  // chi send/wait
  val chi = new DJBundle {
    val s_resp      = Bool() // Send Resp(Comp, SnpResp) To RN/HN
    val w_ack       = Bool() // Wait CompAck From RN
  }
  // other
  val clean         = Bool() // Clean PoS Tag and Dir Lock
  val valid         = Bool() // Valid
}

trait HasFlag { this: DJBundle => val flag = new Flag }


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
    // CHI
    val txRsp       = Decoupled(new RespFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    // Send Task To TaskCM
    val cmTaskVec   = Vec(nrTaskCM, Decoupled(new CMTask))
    // Resp From TaskCM or ReceiveCM
    val cmResp      = Flipped(Valid(new CMResp))  // broadcast signal
    // Send Task To Replace
    val replTask    = Decoupled(new ReplTask)
    val replResp    = Flipped(Valid(new HnTxnID)) // broadcast signal
    // Send Task To Data
    val dataTask    = Decoupled(new DataTask)
    val dataResp    = Flipped(Valid(new HnTxnID)) // broadcast signal
    // Update PoS Message
    val cleanPoS    = Decoupled(new PosClean)
    // State
    val flag        = Output(new Flag)
  })
  HAssert.withEn(io.alloc.bits.dirBank === io.hnIdx.dirBank, io.alloc.valid)

  /*
   * Reg and Wire declaration
   */
  val taskReg     = RegInit((new CommitTask with HasFlag).Lit(_.flag.valid -> false.B))
  val valid       = WireInit(taskReg.flag.valid)
  val flagNext    = WireInit(taskReg.flag)
  val taskNext    = WireInit(taskReg)
  val taskAlloc   = Wire(new CommitTask with HasFlag)
  val flagDec     = Wire(new Flag) // Get new flag when decode done
  val secTaskInst = WireInit(0.U.asTypeOf(new TaskInst))
  val secCMTask   = WireInit(0.U.asTypeOf(new CMTask))
  val compAckHit  = io.rxRsp.valid & io.rxRsp.bits.TxnID === io.hnTxnID & io.rxRsp.bits.Opcode === CompAck
  io.flag         := taskReg.flag

  /*
   * Receive Commit Task from Frontend
   */
  val alloc                     = io.alloc.bits
  // task base message
  taskAlloc.chi                 := alloc.chi
  taskAlloc.dir                 := alloc.dir
  taskAlloc.alr                 := alloc.alr
  taskAlloc.taskCode            := DontCare
  taskAlloc.taskInst            := DontCare
  taskAlloc.commit              := alloc.commit
  taskAlloc.snpTgt              := alloc.snpTgt
  taskAlloc.ds                  := alloc.ds
  // task flag internal send
  taskAlloc.flag.intl.s.data0   := alloc.commit.dataOp.data0 & !taskAlloc.alr.sData
  taskAlloc.flag.intl.s.data1   := alloc.commit.dataOp.data1 & !(alloc.commit.wriLLC & !alloc.dir.llc.hit) // not replace llc
  taskAlloc.flag.intl.s.wriDir  := alloc.commit.wriDir
  taskAlloc.flag.intl.s.secTask := false.B
  // task flag internal wait
  taskAlloc.flag.intl.w.receive := alloc.taskCode.waitRecDone
  taskAlloc.flag.intl.w.cmResp  := alloc.taskCode.opsValid
  taskAlloc.flag.intl.w.secResp := false.B
  taskAlloc.flag.intl.w.repl    := alloc.commit.wriDir
  taskAlloc.flag.intl.w.data0   := alloc.commit.dataOp.data0
  taskAlloc.flag.intl.w.data1   := alloc.alr.cleanDB | (alloc.commit.dataOp.data1 & !(alloc.commit.wriLLC & !alloc.dir.llc.hit)) // not replace llc
  // task flag chi send
  taskAlloc.flag.chi.s_resp     := alloc.commit.sendResp & alloc.commit.channel === ChiChannel.RSP
  // task flag chi wait
  taskAlloc.flag.chi.w_ack      := alloc.chi.expCompAck & !alloc.chi.reqIs(WriteEvictOrEvict)
  // task flag clean and valid
  taskAlloc.flag.clean          := false.B
  taskAlloc.flag.valid          := true.B
  // check decode result from Frontend Decode
  when(io.alloc.valid) {
    HardwareAssertion.withEn(alloc.commit.asUInt === 0.U, !alloc.commit.valid)
    HardwareAssertion.withEn(alloc.commit.sendResp, io.alloc.bits.alr.sData)
    HardwareAssertion.withEn(alloc.commit.channel === ChiChannel.DAT, alloc.commit.dataOp.send)
  }


  /*
   * 1st decode after Decode completes, execute result in backend
   * 2nd decode after Commit gets TaskCM & ReceiveCM response, save to TaskReg
   * 3rd decode after Commit gets TaskCM response again, save to TaskReg
   */
  // judge decode is valid
  val waitSecDec  = (taskReg.flag.intl.w.cmResp | taskReg.flag.intl.w.receive) & valid
  val secDecReady = !waitSecDec
  val secDecValid = RegNext(waitSecDec) & secDecReady & valid
  val trdDecValid = io.cmResp.fire & io.cmResp.bits.hnTxnID === io.hnTxnID & taskReg.flag.intl.w.secResp & !taskReg.commit.valid & valid
  val decValid    = secDecValid | trdDecValid
  HAssert.withEn(!io.cmResp.bits.fromRec, trdDecValid)

  // Get second taskInst
  when(trdDecValid) {
    secTaskInst   := io.cmResp.bits.taskInst
  }
  // Get other inst
  val chiInst     = taskReg.chi.getChiInst(valid)
  val stateInst   = taskReg.dir.getStateInst(taskReg.chi.metaIdOH, valid)
  val taskInst    = WireInit(taskReg.taskInst)
  taskInst.valid  := taskReg.taskInst.valid & !(taskReg.flag.intl.w.cmResp | taskReg.flag.intl.w.receive) & valid

  // Decode result
  val decRes      = Decode.decode(chiInst, stateInst, taskInst, secTaskInst)._2
  val secTaskCode = WireInit(decRes._2)
  val cmtCode     = WireInit(decRes._3)
  dontTouch(secTaskCode)
  dontTouch(cmtCode)

  // get 2nd deocode result
  when(secDecValid) {
    taskNext.taskCode     := secTaskCode
    when(!cmtCode.waitSecDone) {
      taskNext.commit     := cmtCode
    }
    // HAssert
    HAssert(!taskReg.taskCode.valid)
    HAssert(!taskReg.commit.valid)
    HAssert.withEn(taskNext.taskCode.valid, cmtCode.waitSecDone)
    HAssert.withEn(cmtCode.asUInt === 0.U | cmtCode.waitSecDone, !cmtCode.valid)
  }.elsewhen(trdDecValid) {
    taskNext.taskCode     := 0.U.asTypeOf(new TaskCode)
    taskNext.commit       := cmtCode
    // HAssert
    HAssert(taskReg.taskCode.valid)
    HAssert(!taskReg.commit.valid)
    HAssert.withEn(cmtCode.asUInt === 0.U | cmtCode.waitSecDone, !cmtCode.valid)
  }

  /*
   * Get new flag when decode done
   */
  // task flag internal send
  flagDec.intl.s.data0    := taskNext.commit.dataOp.data0
  flagDec.intl.s.data1    := taskNext.commit.dataOp.data1 & !(taskNext.commit.wriLLC & !taskReg.dir.llc.hit) // not replace llc
  flagDec.intl.s.wriDir   := taskNext.commit.wriDir
  flagDec.intl.s.secTask  := taskNext.taskCode.opsValid
  // task flag internal wait
  flagDec.intl.w.receive  := false.B
  flagDec.intl.w.cmResp   := false.B
  flagDec.intl.w.secResp  := taskNext.taskCode.opsValid
  flagDec.intl.w.repl     := taskNext.commit.wriDir
  flagDec.intl.w.data0    := taskNext.commit.dataOp.data0
  flagDec.intl.w.data1    := taskNext.commit.dataOp.data1 & !(taskNext.commit.wriLLC & !taskReg.dir.llc.hit) // not replace llc
  // task flag chi send
  flagDec.chi.s_resp      := taskNext.commit.sendResp & taskNext.commit.channel === ChiChannel.RSP
  // task flag chi wait
  flagDec.chi.w_ack       := taskReg.flag.chi.w_ack & !compAckHit
  // task flag clean and valid
  flagDec.clean           := false.B
  flagDec.valid           := true.B

  /*
   * Send DataTask
   * Priority: data > data0 > data1
   */
  // data task valid
  val dtVal0  = valid & taskReg.flag.valid & taskReg.flag.intl.s.data0
  val dtVal1  = valid & taskReg.flag.valid & taskReg.flag.intl.s.data1 & !taskReg.flag.intl.w.repl
  val dtVal   = dtVal0 | dtVal1
  // dontTouch
  dontTouch(dtVal0)
  dontTouch(dtVal1)
  dontTouch(dtVal)

  // io.dataTask
  io.dataTask.valid               := dtVal0 | dtVal1
  io.dataTask.bits                := DontCare
  // dataOp
  when(dtVal) {
    io.dataTask.bits.dataOp       := taskReg.commit.dataOp
  }.elsewhen(dtVal0) {
    io.dataTask.bits.dataOp       := 0.U.asTypeOf(new DataOp)
    io.dataTask.bits.dataOp.reqs  := taskReg.commit.dataOp.reqs
    io.dataTask.bits.dataOp.read  := taskReg.commit.dataOp.read
    io.dataTask.bits.dataOp.send  := taskReg.commit.dataOp.send
  }.otherwise {
    io.dataTask.bits.dataOp       := 0.U.asTypeOf(new DataOp)
    io.dataTask.bits.dataOp.save  := taskReg.commit.dataOp.save
    io.dataTask.bits.dataOp.clean := taskReg.commit.dataOp.clean
  }
  when(io.dataTask.valid) {
    HardwareAssertion(PopCount(io.dataTask.bits.dataOp.asUInt) =/= 0.U)
    HardwareAssertion.withEn(!io.dataTask.bits.dataOp.reqs, taskReg.alr.reqs)
  }
  // txDat
  // TODO: SnpRespData
  io.dataTask.bits.txDat.DBID     := io.hnTxnID
  io.dataTask.bits.txDat.Resp     := taskReg.commit.resp
  io.dataTask.bits.txDat.Opcode   := taskReg.commit.commitOp
  io.dataTask.bits.txDat.HomeNID  := DontCare // remap in SAM
  io.dataTask.bits.txDat.TxnID    := taskReg.chi.txnID
  io.dataTask.bits.txDat.SrcID    := taskReg.chi.getNoC(io.config.ci)
  io.dataTask.bits.txDat.TgtID    := taskReg.chi.nodeId
  // other bits
  io.dataTask.bits.ds             := taskReg.ds
  io.dataTask.bits.hnTxnID        := io.hnTxnID
  io.dataTask.bits.dataVec        := taskReg.chi.dataVec

  /*
   * Send write directory task to ReplaceCM
   */
  // valid and hnTxnID
  io.replTask.valid               := valid & taskReg.flag.intl.s.wriDir & !taskReg.flag.intl.w.data0
  io.replTask.bits.hnTxnID        := io.hnTxnID
  // write sf bits
  io.replTask.bits.wriSF          := taskReg.commit.wriSF
  io.replTask.bits.dir.sf.hit     := taskReg.dir.sf.hit
  io.replTask.bits.dir.sf.wayOH   := taskReg.dir.sf.wayOH
  io.replTask.bits.dir.sf.metaVec.map(_.state).zipWithIndex.foreach {
    case(s, i) =>
      val metaIdOH = taskReg.chi.metaIdOH
      val snpVec   = taskReg.dir.getSnpVec(taskReg.snpTgt, metaIdOH)
      s := PriorityMux(Seq(
        metaIdOH(i.U) -> taskReg.commit.srcValid,
        snpVec(i)     -> taskReg.commit.snpValid,
        true.B        -> taskReg.dir.sf.metaVec(i).state
      ))
  }
  // write llc bits
  io.replTask.bits.wriLLC         := taskReg.commit.wriLLC
  io.replTask.bits.dir.llc.hit    := taskReg.dir.llc.hit
  io.replTask.bits.dir.llc.wayOH  := taskReg.dir.llc.wayOH
  io.replTask.bits.dir.llc.metaVec.head.state := taskReg.commit.llcState

  /*
   * Send second CMTask to TaskCM
   */
  // chi
  secCMTask.chi                   := taskReg.chi
  secCMTask.chi.channel           := Mux(taskReg.taskCode.snoop, ChiChannel.SNP, ChiChannel.REQ)
  secCMTask.chi.opcode            := taskReg.taskCode.opcode
  secCMTask.chi.expCompAck        := taskReg.taskCode.expCompAck
  secCMTask.chi.retToSrc          := taskReg.taskCode.retToSrc
  // other
  secCMTask.hnTxnID               := io.hnTxnID
  secCMTask.alr                   := taskReg.alr
  secCMTask.dataOp                := taskReg.taskCode.dataOp
  secCMTask.ds                    := taskReg.ds
  secCMTask.snpVec                := taskReg.dir.getSnpVec(taskReg.taskCode.snpTgt, taskReg.chi.metaIdOH)
  secCMTask.fromRepl              := false.B
  secCMTask.cbResp                := taskReg.dir.llc.metaVec.head.cbResp
  secCMTask.doDMT                 := taskReg.taskCode.doDMT
  // alloc
  io.cmTaskVec(CMID.SNP).valid    := valid & taskReg.flag.intl.s.secTask & taskReg.taskCode.snoop
  io.cmTaskVec(CMID.READ).valid   := valid & taskReg.flag.intl.s.secTask & taskReg.taskCode.read
  io.cmTaskVec(CMID.DL).valid     := valid & taskReg.flag.intl.s.secTask & taskReg.taskCode.dataless
  io.cmTaskVec(CMID.WRI).valid    := valid & taskReg.flag.intl.s.secTask & taskReg.taskCode.wriOrAtm
  io.cmTaskVec.foreach(_.bits     := secCMTask)
  HardwareAssertion(PopCount(io.cmTaskVec.map(_.fire)) <= 1.U)


  /*
   * Send CHI.RSP
   */
  // valid
  io.txRsp.valid            := valid & taskReg.flag.chi.s_resp
  // bits
  io.txRsp.bits             := DontCare
  io.txRsp.bits.DBID        := io.hnTxnID
  io.txRsp.bits.FwdState    := taskReg.commit.fwdResp
  io.txRsp.bits.Resp        := taskReg.commit.resp
  io.txRsp.bits.Opcode      := taskReg.commit.commitOp
  io.txRsp.bits.TxnID       := taskReg.chi.txnID
  io.txRsp.bits.SrcID       := taskReg.chi.getNoC(io.config.ci)
  io.txRsp.bits.TgtID       := taskReg.chi.nodeId


  /*
   * Clean PoS
   */
  // valid
  io.cleanPoS.valid         := valid & taskReg.flag.clean
  // bits
  io.cleanPoS.bits.hnIdx    := io.hnIdx
  io.cleanPoS.bits.channel  := taskReg.chi.channel

  /*
   * Get next flag
   */
  when(io.alloc.fire & io.alloc.bits.hnTxnID === io.hnTxnID) {
    flagNext := taskAlloc.flag
  }.elsewhen(decValid) {
    flagNext := flagDec
  }.otherwise {
    // task flag internal send
    when(io.dataTask.fire & dtVal0)         { flagNext.intl.s.data0   := false.B; HAssert(taskReg.flag.intl.s.data0)   }
    when(io.dataTask.fire & dtVal1)         { flagNext.intl.s.data1   := false.B; HAssert(taskReg.flag.intl.s.data1)   }
    when(io.replTask.fire)                  { flagNext.intl.s.wriDir  := false.B; HAssert(taskReg.flag.intl.s.wriDir)  }
    when(Cat(io.cmTaskVec.map(_.fire)).orR) { flagNext.intl.s.secTask := false.B; HAssert(taskReg.flag.intl.s.secTask) }
    // task flag internal wait
    when(io.cmResp.fire   & io.cmResp.bits.hnTxnID   === io.hnTxnID &  io.cmResp.bits.fromRec) { flagNext.intl.w.receive := false.B; HAssert(taskReg.flag.intl.w.receive) }
    when(io.cmResp.fire   & io.cmResp.bits.hnTxnID   === io.hnTxnID & !io.cmResp.bits.fromRec) { flagNext.intl.w.cmResp  := false.B; HAssert.withEn(taskReg.flag.intl.w.cmResp | taskReg.flag.intl.w.secResp, valid) }
    when(io.cmResp.fire   & io.cmResp.bits.hnTxnID   === io.hnTxnID & !io.cmResp.bits.fromRec) { flagNext.intl.w.secResp := false.B; HAssert.withEn(taskReg.flag.intl.w.cmResp | taskReg.flag.intl.w.secResp, valid) }
    when(io.replResp.fire & io.replResp.bits.hnTxnID === io.hnTxnID) { flagNext.intl.w.repl    := false.B; HAssert(taskReg.flag.intl.w.repl) }
    when(io.dataResp.fire & io.dataResp.bits.hnTxnID === io.hnTxnID) { flagNext.intl.w.data0   := taskReg.flag.intl.s.data0 }
    when(io.dataResp.fire & io.dataResp.bits.hnTxnID === io.hnTxnID) { flagNext.intl.w.data1   := taskReg.flag.intl.s.data1 }
    // when task is (1)invalid or (2)wait data or (3)wait cm resp or (4) wait replace resp can get data resp
    HAssert.withEn(!taskReg.flag.valid | taskReg.flag.intl.w.data0 | taskReg.flag.intl.w.data1 | taskReg.flag.intl.w.cmResp | taskReg.flag.intl.w.secResp | taskReg.flag.intl.w.repl, io.dataResp.fire & io.dataResp.bits.hnTxnID === io.hnTxnID)
    // task flag chi send
    when(io.txRsp.fire) { flagNext.chi.s_resp := false.B; HAssert(taskReg.flag.chi.s_resp) }
    when(compAckHit)    { flagNext.chi.w_ack  := false.B; HAssert(taskReg.flag.chi.w_ack | taskReg.chi.reqIs(WriteEvictOrEvict))  }
    // task flag clean and valid
    when(io.alloc.fire & io.alloc.bits.hnTxnID === io.hnTxnID) {
      flagNext.valid  := true.B
      flagNext.clean  := false.B
      HAssert(!taskReg.flag.valid)
    }.elsewhen(io.cleanPoS.fire) {
      flagNext.valid  := false.B
      flagNext.clean  := false.B
      HAssert(taskReg.flag.valid)
      HAssert(taskReg.flag.clean)
    }.elsewhen(taskReg.flag.valid) {
      flagNext.clean  := !taskReg.flag.intl.asUInt.orR & !taskReg.flag.chi.asUInt.orR // all flag is false
      HAssert(taskReg.flag.valid)
    }
  }

  /*
   * Get new task
   */
  when(io.alloc.fire & io.alloc.bits.hnTxnID === io.hnTxnID) {
    taskNext := taskAlloc
  }.elsewhen(io.cmResp.fire & io.cmResp.bits.hnTxnID === io.hnTxnID) {
    taskNext.taskInst := (taskReg.taskInst.asUInt | io.cmResp.bits.taskInst.asUInt).asTypeOf(new TaskInst)
    HAssert.withEn(io.cmResp.valid, io.cmResp.bits.fromRec)
  }
  taskNext.flag := flagNext

  /*
   * Set new task
   */
  val set = io.alloc.fire & io.alloc.bits.hnTxnID === io.hnTxnID | valid; dontTouch(set)
  when(set) {
    taskReg := taskNext
  }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(!valid, TIMEOUT_COMMIT, cf"\n\nTIMEOUT: " +
    cf"\nInternal Send: ${taskReg.flag.intl.s}\nInternal Wait: ${taskReg.flag.intl.w}\n" +
    cf"CHI: ${taskReg.flag.chi}\n${taskReg.chi.getChiInst()}\n${taskReg.dir.getStateInst(taskReg.chi.metaIdOH)}\n\n")
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
    // Send Task To TaskCM
    val cmTaskVec   = Vec(nrTaskCM, Decoupled(new CMTask))
    // Resp From TaskCM or ReceiveCM
    val cmResp      = Flipped(Valid(new CMResp))  // broadcast signal
    // Send Task To Replace
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
  val dbgVec  = WireInit(VecInit(entries.map(_.io.flag)))
  dontTouch(dbgVec)

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
   * Connect Commit <- IO
   */
  entries.zipWithIndex.foreach { case(e, i) =>
    e.io.config   := io.config
    e.io.hnTxnID  := i.U
    e.io.hnIdx    := i.U.asTypeOf(new HnTxnID).getHnIdx
    e.io.rxRsp    := io.rxRsp
    e.io.cmResp   := io.cmResp
    e.io.replResp := io.replResp
    e.io.dataResp := io.dataResp
  }

  /*
   * Connect IO <- Commit
   */
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