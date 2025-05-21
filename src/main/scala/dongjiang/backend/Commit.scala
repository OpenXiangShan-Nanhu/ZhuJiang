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
import dongjiang.backend.CmtState._
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ------------------------------------------- Commit Flag --------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object CmtState {
  val width     = 4
  val FREE      = 0x0.U
  val WAITDATA  = 0x1.U
  val SETFLAG0  = 0x2.U
  val WAIT0     = 0x3.U
  val TRDDEC    = 0x4.U
  val SETFLAG1  = 0x5.U
  val WAIT1     = 0x6.U
  val FTHDEC    = 0x7.U
  val SETFLAG2  = 0x8.U
  val COMMIT    = 0x9.U
  val CLEAN     = 0xA.U
}

class Flag(implicit p: Parameters) extends DJBundle {
  val intl = new DJBundle {
    // internal send
    val s = new DJBundle {
      val task      = Bool() // Send Task To TaskCM for the first time
      val secTask   = Bool() // Send Task To TaskCM for the second time
      val data0     = Bool() // Send Data Task(reqs, read, send)
      val data1     = Bool() // Send Data Task(save, clean)  // Note: Need wait repl done
      val wriDir    = Bool() // Send Write Task To Replace   // Note: Need wait data0 done
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
  // Free -> WaitData -> SetFlag0 -> Wait0 -> ThirdDecode -> SetFlag1 -> Wait1 -> FourthDec -> SetFlag2 -> Commit -> Clean -> Free
  //                  ^                    ^              ^                    ^            ^                     ^        ^
  //               dataResp             cmResp        decMes.fire            cmResp     decMes.fire         allFlagDone  cleanPoS.fire
  val state         = UInt(CmtState.width.W)
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
    // Decode
    val decMes      = Decoupled(new PackDecList with HasPackTaskInst { val isThird = Bool() } )
    val decRes      = Input(new PackDecList)
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
  // task
  val taskReg     = Reg(new CommitTask)
  val taskNext    = WireInit(taskReg)
  val decListNext = WireInit(taskReg.decList)
  // flag
  val flagReg     = RegInit(new Flag().Lit(_.state -> FREE))
  val flagNext    = WireInit(flagReg)
  val flagDec     = WireInit(flagReg)
  val valid       = WireInit(flagReg.state =/= FREE)
  // inst
  val instReg     = RegInit(new TaskInst().Lit(_.valid -> false.B))
  val instNext    = WireInit(instReg)
  // other
  val cmTask      = WireInit(0.U.asTypeOf(new CMTask))
  val rspAckHit   = valid & io.rxRsp.valid & io.rxRsp.bits.TxnID === io.hnTxnID & io.rxRsp.bits.Opcode === CompAck
  val datAckHit   = valid & io.rxDat.valid & io.rxDat.bits.TxnID === io.hnTxnID & io.rxDat.bits.Opcode === NCBWrDataCompAck
  val compAckHit  = rspAckHit | datAckHit
  io.flag         := flagReg

  /*
   * Get Decode Result
   */
  val getDecRes           = Module(new GetDecRes())
  val fstCode             = getDecRes.io.taskCode
  val secCode             = getDecRes.io.secTaskCode
  val cmtCode             = getDecRes.io.commitCode
  getDecRes.io.list.valid := valid
  getDecRes.io.list.bits  := taskReg.decList
  dontTouch(fstCode)
  dontTouch(secCode)
  dontTouch(cmtCode)
  
  /*
   * Send DataTask
   * Priority: data > data0 > data1
   */
  // data task valid
  val doingData0 = !flagReg.intl.s.data0 & flagReg.intl.w.data0
  val dtVal0     = valid  & flagReg.intl.s.data0
  val dtVal1     = valid  & flagReg.intl.s.data1 & !flagReg.intl.w.repl & !doingData0
  val dtVal      = dtVal0 & dtVal1
  // dontTouch
  dontTouch(dtVal0)
  dontTouch(dtVal1)
  dontTouch(dtVal)

  // io.dataTask
  io.dataTask.valid               := dtVal0 | dtVal1
  io.dataTask.bits                := DontCare
  // dataOp
  when(dtVal) {
    io.dataTask.bits.dataOp       := cmtCode.dataOp
  }.elsewhen(dtVal0) {
    io.dataTask.bits.dataOp       := 0.U.asTypeOf(new DataOp)
    io.dataTask.bits.dataOp.reqs  := cmtCode.dataOp.reqs
    io.dataTask.bits.dataOp.read  := cmtCode.dataOp.read
    io.dataTask.bits.dataOp.send  := cmtCode.dataOp.send
  }.otherwise {
    io.dataTask.bits.dataOp       := 0.U.asTypeOf(new DataOp)
    io.dataTask.bits.dataOp.save  := cmtCode.dataOp.save
    io.dataTask.bits.dataOp.clean := cmtCode.dataOp.clean
  }
  when(io.dataTask.valid) {
    HardwareAssertion(PopCount(io.dataTask.bits.dataOp.asUInt) =/= 0.U)
    HardwareAssertion.withEn(!io.dataTask.bits.dataOp.reqs, taskReg.alr.reqs)
  }
  // txDat
  // TODO: SnpRespData
  io.dataTask.bits.txDat.DBID     := io.hnTxnID
  io.dataTask.bits.txDat.Resp     := cmtCode.resp
  io.dataTask.bits.txDat.Opcode   := cmtCode.commitOp
  io.dataTask.bits.txDat.HomeNID  := DontCare // remap in SAM
  io.dataTask.bits.txDat.TxnID    := taskReg.chi.txnID
  io.dataTask.bits.txDat.SrcID    := taskReg.chi.getNoC
  io.dataTask.bits.txDat.TgtID    := taskReg.chi.nodeId
  // other bits
  io.dataTask.bits.ds             := taskReg.ds
  io.dataTask.bits.hnTxnID        := io.hnTxnID
  io.dataTask.bits.dataVec        := taskReg.chi.dataVec

  /*
   * Send write directory task to ReplaceCM
   */
  // valid and hnTxnID
  io.replTask.valid               := valid & flagReg.intl.s.wriDir & !flagReg.intl.w.data0
  io.replTask.bits.hnTxnID        := io.hnTxnID
  // write sf bits
  val snpTgt                      = Mux(secCode.snoop, secCode.snpTgt, fstCode.snpTgt)
  io.replTask.bits.wriSF          := cmtCode.wriSF
  io.replTask.bits.dir.sf.hit     := taskReg.dir.sf.hit
  io.replTask.bits.dir.sf.wayOH   := taskReg.dir.sf.wayOH
  io.replTask.bits.dir.sf.metaVec.map(_.state).zipWithIndex.foreach {
    case(s, i) =>
      val metaIdOH    = taskReg.chi.metaIdOH
      val srcVec      = VecInit(metaIdOH.asBools)
      val snpVec      = taskReg.dir.getSnpVec(snpTgt, metaIdOH)
      s := PriorityMux(Seq(
        (srcVec(i) & cmtCode.wriSRC) -> cmtCode.srcValid, // modify source state
        (snpVec(i) & cmtCode.wriSNP) -> cmtCode.snpValid, // modify snoopee state
        true.B                       -> (taskReg.dir.sf.hit & taskReg.dir.sf.metaVec(i).state)
      ))
  }
  // write llc bits
  io.replTask.bits.wriLLC         := cmtCode.wriLLC
  io.replTask.bits.dir.llc.hit    := taskReg.dir.llc.hit
  io.replTask.bits.dir.llc.wayOH  := taskReg.dir.llc.wayOH
  io.replTask.bits.dir.llc.metaVec.head.state := cmtCode.llcState

  /*
   * Send second CMTask to TaskCM
   */
  val taskCode = Mux(secCode.valid, secCode, fstCode)
  // chi
  cmTask.chi                      := taskReg.chi
  cmTask.chi.channel              := Mux(taskCode.snoop, ChiChannel.SNP, ChiChannel.REQ)
  cmTask.chi.dataVec              := Mux(taskCode.snoop, DataVec.Full,   taskReg.chi.dataVec)
  cmTask.chi.opcode               := taskCode.opcode
  cmTask.chi.expCompAck           := taskCode.expCompAck
  cmTask.chi.retToSrc             := taskCode.retToSrc
  // other
  cmTask.hnTxnID                  := io.hnTxnID
  cmTask.alr.reqs                 := taskReg.alr.reqs
  cmTask.alr.sData                := DontCare
  cmTask.alr.cleanDB              := DontCare
  cmTask.dataOp                   := taskCode.dataOp
  cmTask.ds                       := taskReg.ds
  cmTask.snpVec                   := taskReg.dir.getSnpVec(taskCode.snpTgt, taskReg.chi.metaIdOH)
  cmTask.fromRepl                 := false.B
  cmTask.cbResp                   := taskReg.dir.llc.metaVec.head.cbResp
  cmTask.doDMT                    := taskCode.doDMT
  // alloc
  io.cmTaskVec(CMID.SNP).valid    := valid & (flagReg.intl.s.task | flagReg.intl.s.secTask) & taskCode.snoop
  io.cmTaskVec(CMID.READ).valid   := valid & (flagReg.intl.s.task | flagReg.intl.s.secTask) & taskCode.read
  io.cmTaskVec(CMID.DL).valid     := valid & (flagReg.intl.s.task | flagReg.intl.s.secTask) & taskCode.dataless
  io.cmTaskVec(CMID.WRI).valid    := valid & (flagReg.intl.s.task | flagReg.intl.s.secTask) & taskCode.wriOrAtm
  io.cmTaskVec.foreach(_.bits     := cmTask)
  HAssert(PopCount(io.cmTaskVec.map(_.fire)) <= 1.U)
  HAssert.withEn(!(flagReg.intl.s.task & flagReg.intl.s.secTask), valid)


  /*
   * Send CHI.RSP
   */
  // valid
  io.txRsp.valid            := valid & flagReg.chi.s_resp
  // bits
  io.txRsp.bits             := DontCare
  io.txRsp.bits.DBID        := io.hnTxnID
  io.txRsp.bits.FwdState    := cmtCode.fwdResp
  io.txRsp.bits.Resp        := cmtCode.resp
  io.txRsp.bits.Opcode      := cmtCode.commitOp
  io.txRsp.bits.TxnID       := taskReg.chi.txnID
  io.txRsp.bits.SrcID       := taskReg.chi.getNoC
  io.txRsp.bits.TgtID       := taskReg.chi.nodeId


  /*
   * Clean PoS
   */
  // valid
  io.cleanPoS.valid         := valid & flagReg.state === CLEAN
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
  when(io.decMes.fire) {
    decListNext                 := io.decRes.decList
  }
  io.decMes.valid               := flagReg.state === TRDDEC | flagReg.state === FTHDEC
  io.decMes.bits.isThird        := flagReg.state === TRDDEC
  io.decMes.bits.decList        := taskReg.decList
  io.decMes.bits.taskInst       := instReg
  io.decMes.bits.taskInst.valid := io.decMes.valid
  // HAssert
  HAssert.withEn(instReg.valid, io.decMes.valid)
  HAssert.withEn(taskReg.decList(0) === io.decRes.decList(0), io.decMes.fire)
  HAssert.withEn(taskReg.decList(1) === io.decRes.decList(1), io.decMes.fire)
  HAssert.withEn(taskReg.decList(2) === io.decRes.decList(2), io.decMes.fire & flagReg.state =/= TRDDEC)
  HAssert.withEn(taskReg.decList(3) === io.decRes.decList(3), io.decMes.fire & flagReg.state =/= FTHDEC)

  /*
   * Get new flag when decode done
   */
  // parse decode result
  val noTask      = 0.U.asTypeOf(new TaskCode)
  val noCmt       = 0.U.asTypeOf(new CommitCode)
  val taskCodeDec = Mux(flagReg.state === SETFLAG0, fstCode, noTask)
  val secCodeDec  = Mux(flagReg.state === SETFLAG1, secCode, noTask)
  val cmtCodeDec  = PriorityMux(Seq(
    (flagReg.state === SETFLAG0 &  fstCode.valid)        -> noCmt,
    (flagReg.state === SETFLAG0 & !fstCode.valid)        -> cmtCode,
    (flagReg.state === SETFLAG1 &  cmtCode.waitSecDone)  -> noCmt,
    (flagReg.state === SETFLAG1 & !cmtCode.waitSecDone)  -> cmtCode,
    (flagReg.state === SETFLAG2)                         -> cmtCode
  ))
  // task flag internal send
  flagDec.intl.s.task     := taskCodeDec.opsValid
  flagDec.intl.s.secTask  := secCodeDec.opsValid
  flagDec.intl.s.data0    := cmtCodeDec.dataOp.data0 & !taskReg.alr.sData
  flagDec.intl.s.data1    := cmtCodeDec.dataOp.data1 & !(cmtCodeDec.wriLLC & !taskReg.dir.llc.hit) // not replace llc
  flagDec.intl.s.wriDir   := cmtCodeDec.wriDir
  // task flag internal wait
  val recHit              =  valid & io.cmResp.fire & io.cmResp.bits.hnTxnID === io.hnTxnID & io.cmResp.bits.fromRec
  flagDec.intl.w.receive  := flagReg.intl.w.receive & !recHit
  flagDec.intl.w.cmResp   := flagDec.intl.s.task
  flagDec.intl.w.secResp  := flagDec.intl.s.secTask
  flagDec.intl.w.repl     := flagDec.intl.s.wriDir
  flagDec.intl.w.data0    := flagDec.intl.s.data0
  flagDec.intl.w.data1    := flagDec.intl.s.data1
  // task flag chi send
  flagDec.chi.s_resp      := cmtCodeDec.sendResp & cmtCodeDec.channel === ChiChannel.RSP
  // task flag chi wait
  flagDec.chi.w_ack       := flagReg.chi.w_ack & !compAckHit
  // HAssert
  HAssert.withEn(flagReg.intl.w.receive, recHit)
  HAssert.withEn(taskReg.chi.isWrite, valid & taskCodeDec.waitRecDone)
  HAssert.withEn(!(io.alloc.bits.alr.cleanDB & io.alloc.bits.alr.sData), io.alloc.valid)
  HAssert.withEn(io.alloc.bits.alr.reqs, io.alloc.valid & (io.alloc.bits.alr.cleanDB | io.alloc.bits.alr.sData))
  HAssert.withEn(fstCode.waitRecDone, valid & flagReg.state === SETFLAG0 & taskReg.alr.cleanDB)
  HAssert.withEn(cmtCodeDec.dataOp.send, valid & cmtCodeDec.sendResp & cmtCodeDec.channel =/= ChiChannel.RSP)

  /*
   * Get next flag
   */
  val allocHit = io.alloc.fire & io.alloc.bits.hnTxnID === io.hnTxnID
  // flag
  when(allocHit) {
    flagNext                := 0.U.asTypeOf(new Flag) // clear
    flagNext.intl.w.receive := io.alloc.bits.chi.isWrite
    flagNext.intl.w.data0   := io.alloc.bits.alr.sData
    flagNext.intl.w.data1   := io.alloc.bits.alr.cleanDB
    flagNext.chi.w_ack      := io.alloc.bits.chi.expCompAck & !io.alloc.bits.chi.reqIs(WriteEvictOrEvict)
  }.elsewhen(flagReg.state === SETFLAG0 | flagReg.state === SETFLAG1 | flagReg.state === SETFLAG2) {
    flagNext                := flagDec
  }.otherwise {
    // task flag internal send
    when(Cat(io.cmTaskVec.map(_.fire)).orR) { flagNext.intl.s.task    := false.B; HAssert.withEn(flagReg.intl.s.task,    flagReg.intl.w.cmResp)  }
    when(Cat(io.cmTaskVec.map(_.fire)).orR) { flagNext.intl.s.secTask := false.B; HAssert.withEn(flagReg.intl.s.secTask, flagReg.intl.w.secResp) }
    when(io.dataTask.fire & dtVal0)         { flagNext.intl.s.data0   := false.B; HAssert(flagReg.intl.s.data0)   }
    when(io.dataTask.fire & dtVal1)         { flagNext.intl.s.data1   := false.B; HAssert(flagReg.intl.s.data1)   }
    when(io.replTask.fire)                  { flagNext.intl.s.wriDir  := false.B; HAssert(flagReg.intl.s.wriDir)  }
    // task flag internal wait
    when(io.cmResp.fire   & io.cmResp.bits.hnTxnID   === io.hnTxnID &  io.cmResp.bits.fromRec) { flagNext.intl.w.receive := false.B; HAssert(flagReg.intl.w.receive) }
    when(io.cmResp.fire   & io.cmResp.bits.hnTxnID   === io.hnTxnID & !io.cmResp.bits.fromRec) { flagNext.intl.w.cmResp  := false.B; HAssert.withEn(flagReg.intl.w.cmResp | flagReg.intl.w.secResp, valid) }
    when(io.cmResp.fire   & io.cmResp.bits.hnTxnID   === io.hnTxnID & !io.cmResp.bits.fromRec) { flagNext.intl.w.secResp := false.B; HAssert.withEn(flagReg.intl.w.cmResp | flagReg.intl.w.secResp, valid) }
    when(io.replResp.fire & io.replResp.bits.hnTxnID === io.hnTxnID) { flagNext.intl.w.repl    := false.B; HAssert(flagReg.intl.w.repl) }
    when(io.dataResp.fire & io.dataResp.bits.hnTxnID === io.hnTxnID) { flagNext.intl.w.data0   := flagReg.intl.s.data0 }
    when(io.dataResp.fire & io.dataResp.bits.hnTxnID === io.hnTxnID) { flagNext.intl.w.data1   := flagReg.intl.s.data1 }
    // when task is (1)invalid or (2)wait data or (3)wait cm resp or (4) wait replace resp can get data resp
    HAssert.withEn(!valid | flagReg.intl.w.data0 | flagReg.intl.w.data1 | flagReg.intl.w.cmResp | flagReg.intl.w.secResp | flagReg.intl.w.repl, io.dataResp.fire & io.dataResp.bits.hnTxnID === io.hnTxnID)
    // task flag chi send
    when(io.txRsp.fire) { flagNext.chi.s_resp := false.B; HAssert(flagReg.chi.s_resp) }
    when(compAckHit)    { flagNext.chi.w_ack  := false.B; HAssert.withEn(flagReg.chi.w_ack | taskReg.chi.reqIs(WriteEvictOrEvict), rspAckHit)  }
  }

  // state
  val getAllDataResp    = !flagReg.intl.w.data0 & !flagReg.intl.w.data1
  val willGetAllCMResp  = PopCount(Seq(flagReg.intl.w.cmResp, flagReg.intl.w.secResp, flagReg.intl.w.receive)) === 1.U & io.cmResp.fire & io.cmResp.bits.hnTxnID === io.hnTxnID
  val getAllCMResp      = (!flagReg.intl.w.cmResp & !flagReg.intl.w.secResp & !flagReg.intl.w.receive) | willGetAllCMResp
  val allFlagDone       = flagReg.intl.asUInt === 0.U & flagReg.chi.asUInt === 0.U
  switch(flagReg.state) {
    is(FREE) {
      when(allocHit)          { flagNext.state := WAITDATA }
    }
    is(WAITDATA) {
      when(getAllDataResp)    { flagNext.state := SETFLAG0 }
    }
    is(SETFLAG0) {
      when(true.B)            { flagNext.state := Mux(cmtCodeDec.valid , COMMIT, WAIT0) }
    }
    is(WAIT0) {
      when(getAllCMResp)      { flagNext.state := TRDDEC }
    }
    is(TRDDEC) {
      when(io.decMes.fire)    { flagNext.state := SETFLAG1 }
    }
    is(SETFLAG1) {
      when(true.B)            { flagNext.state := Mux(cmtCodeDec.valid , COMMIT, WAIT1) }
    }
    is(WAIT1) {
      when(getAllCMResp)      { flagNext.state := FTHDEC }
    }
    is(FTHDEC) {
      when(io.decMes.fire)    { flagNext.state := SETFLAG2 }
    }
    is(SETFLAG2) {
      when(true.B)            { flagNext.state := COMMIT }
    }
    is(COMMIT) {
      when(allFlagDone)       { flagNext.state := CLEAN }
    }
    is(CLEAN) {
      when(io.cleanPoS.fire)  { flagNext.state := FREE }
    }
  }
  HAssert.withEn(flagReg.state === FREE, allocHit)

  /*
   * Get new task
   */
  when(allocHit) {
    taskNext          := io.alloc.bits
  }.otherwise {
    taskNext          := taskReg
    taskNext.decList  := decListNext
  }

  /*
   * Get new task inst
   */
  when(allocHit) {
    instNext := 0.U.asTypeOf(new TaskInst)
  }.elsewhen(io.cmResp.fire & io.cmResp.bits.hnTxnID === io.hnTxnID) {
    when(flagReg.state <= WAIT0) {
      instNext := (instReg.asUInt | io.cmResp.bits.taskInst.asUInt).asTypeOf(new TaskInst)
    }.elsewhen(flagReg.state === WAIT1) {
      instNext := io.cmResp.bits.taskInst
    }
    HAssert(io.cmResp.bits.taskInst.valid)
  }

  /*
   * Set new reg
   */
  val set = allocHit | valid; dontTouch(set)
  when(set) {
    taskReg := taskNext
    flagReg := flagNext
    instReg := instNext
  }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(!valid, TIMEOUT_COMMIT, cf"\n\nTIMEOUT: " +
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
   * Decode
   */
  val decListVec        = VecInit(entries.map(_.io.decMes.bits.decList))
  val decInstVec        = VecInit(entries.map(_.io.decMes.bits.taskInst))
  // Decode ID
  val trdDecId          = PriorityEncoder(entries.map(e => e.io.decMes.valid &  e.io.decMes.bits.isThird))
  val fthDecId          = PriorityEncoder(entries.map(e => e.io.decMes.valid & !e.io.decMes.bits.isThird))
  // Decode
  val trdDecocde        = Wire(new PackDecList)
  val fthDecocde        = Wire(new PackDecList)
  trdDecocde.decList    := decListVec(trdDecId)
  fthDecocde.decList    := decListVec(fthDecId)
  // Result
  val trdRes            = trdDecocde.thirdDec (decInstVec(trdDecId), trdDecId)
  val fthRes            = fthDecocde.fourthDec(decInstVec(fthDecId), fthDecId)
  entries.zipWithIndex.foreach { case(e, i) =>
    val isThird         = e.io.decMes.bits.isThird
    e.io.decRes.decList := Mux(isThird, trdRes, fthRes)
    e.io.decMes.ready   := Mux(isThird, trdDecId === i.U, fthDecId === i.U)
  }
  dontTouch(trdDecId)
  dontTouch(fthDecId)
  dontTouch(trdDecocde)
  dontTouch(fthDecocde)
  dontTouch(trdRes)
  dontTouch(fthRes)


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
