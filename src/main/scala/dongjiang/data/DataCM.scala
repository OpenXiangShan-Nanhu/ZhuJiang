package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.UpdHnTxnID
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import xs.utils.queue.FastQueue
import dongjiang.data.CTRLSTATE._
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------- Ctrl Machine State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object CTRLSTATE {
  val width   = 3
  val FREE    = 0x0.U // -> ALLOC
  val ALLOC   = 0x1.U
  val REPL    = 0x2.U // Read DS and DB at the same time to replacegg
  val READ    = 0x3.U // Read DS and save in DB
  val SEND    = 0x4.U // Read DB and send to CHI
  val SAVE    = 0x5.U // Read DB adn send to DS
  val RESP    = 0x6.U // Send resp to Backend  // -> ALLOC
  val CLEAN   = 0x7.U // Clean DB mask and DBIDPool
}

trait HasCtrlMes { this: DJBundle =>
  // Without repl flag:
  //    Free  -> Alloc -> Read -> Send -> Save -> Resp --> Alloc
  // With repl flag:
  //    Free  -> Alloc -> Replace -> Send -> Resp -> Alloc
  // Clean:
  //    Alloc -> CLEAN -> Alloc/Free

  val state       = UInt(CTRLSTATE.width.W)
  val critical    = Bool()
  // send
  val sReadVec    = UInt(djparam.nrBeat.W)
  val sSendVec    = UInt(djparam.nrBeat.W)
  val sSaveVec    = UInt(djparam.nrBeat.W)
  // wait
  val wReadVec    = UInt(djparam.nrBeat.W) // Wait read DS data to DB
  val wSendVec    = UInt(djparam.nrBeat.W) // Wait read DB data to CHI
  val wSaveVec    = UInt(djparam.nrBeat.W) // Wait read DB data to CHI

  def readBeat    = PriorityEncoder(sReadVec)
  def saveBeat    = PriorityEncoder(sSaveVec)
  def sendBeat    = PriorityEncoder(sSendVec)

  def isReadAll   = wReadVec.asUInt === 0.U
  def isSendAll   = wSendVec.asUInt === 0.U
  def isSaveAll   = wSaveVec.asUInt === 0.U

  def isFree      = state === FREE
  def isValid     = !isFree
  def isAlloc     = state === ALLOC
  def isRepl      = state === REPL & sReadVec.asUInt =/= 0.U & sSaveVec.asUInt =/= 0.U
  def isRead      = state === READ & sReadVec.asUInt =/= 0.U
  def isSend      = state === SEND & sSendVec.asUInt =/= 0.U
  def isSave      = state === SAVE & sSaveVec.asUInt =/= 0.U
  def isResp      = state === RESP
  def isClean     = state === CLEAN
}

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- Ctrl Machine Entry ---------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class DataCtrlEntry(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val dcid        = Input(UInt(dcIdBits.W))
    // From/To Frontend/Backend
    val updHnTxnID  = Flipped(Valid(new UpdHnTxnID))
    val alloc       = Flipped(Decoupled(new HnTxnID with HasDataVec with HasDBIDVec))
    val task        = Flipped(Valid(new DataTask)) // broadcast signal
    val resp        = Decoupled(new HnTxnID)
    val clean       = Flipped(Valid(new HnTxnID with HasDataVec)) // broadcast signal
    // To DS/DB
    val readForRepl = Output(Bool())
    val readToDB    = Decoupled(new ReadDS)
    val readToDS    = Decoupled(new ReadDB)
    val readToCHI   = Decoupled(new ReadDB)
    // Other
    val release     = Decoupled(new DBIDVec with HasDataVec)
    val dsWriDB     = Flipped(Valid(new DCID with HasBeatNum)) // broadcast signal
    val txDatFire   = Flipped(Valid(new DCID with HasBeatNum)) // broadcast signal
    val dbWriDS     = Flipped(Valid(new DCID with HasBeatNum)) // broadcast signal
    val txDatBits   = Output(new DataFlit)
    // State
    val state       = Valid(new HnTxnID with HasDataVec with HasDBIDVec)
  })

  /*
   * Reg and Wire declaration
   */
  val reg  = RegInit((new PackDataTask with HasCtrlMes with HasDataVec with HasDBIDVec).Lit(_.state -> FREE))
  val next = WireInit(reg)
  require(djparam.nrBeat == 2)
  HAssert.withEn(reg.dbidVec(0) =/= reg.dbidVec(1), reg.isValid & reg.isFullSize)

  /*
    * Connect io
   */
  io.txDatBits          := reg.task.txDat
  io.state.valid        := reg.isValid
  io.state.bits.hnTxnID := reg.task.hnTxnID
  io.state.bits.dataVec := reg.dataVec
  io.state.bits.dbidVec := reg.dbidVec

  /*
   * Receive Req
   */
  io.alloc.ready := reg.isFree

  /*
   * Read DS / DB
   */
  io.readForRepl              := reg.isRepl
  // [0] from DS to DB/CHI
  io.readToDB.valid           := reg.isRepl | reg.isRead
  io.readToDB.bits.ds         := reg.task.ds
  io.readToDB.bits.dcid       := io.dcid
  io.readToDB.bits.dbid       := reg.dbidVec(reg.readBeat)
  io.readToDB.bits.beatNum    := reg.readBeat
  io.readToDB.bits.critical   := reg.critical
  io.readToDB.bits.qos        := reg.task.qos
  io.readToDB.bits.toCHI      := (reg.task.dataOp.repl | reg.task.dataOp.send) & !reg.task.dataOp.merge
  HAssert.withEn(!reg.isReadAll, io.readToDB.valid)

  // [1] from DB to CHI
  io.readToCHI.valid          := reg.isSend
  io.readToCHI.bits.ds        := DontCare
  io.readToCHI.bits.dcid      := io.dcid
  io.readToCHI.bits.dbid      := reg.dbidVec(reg.sendBeat)
  io.readToCHI.bits.beatNum   := reg.sendBeat
  io.readToCHI.bits.critical  := reg.critical
  io.readToCHI.bits.repl      := false.B
  io.readToCHI.bits.qos       := reg.task.qos
  HAssert.withEn(!reg.isSendAll, io.readToCHI.valid)

  // [2] from DB to DS
  io.readToDS.valid           := reg.isRepl | reg.isSave
  io.readToDS.bits.ds         := reg.task.ds
  io.readToDS.bits.dcid       := io.dcid
  io.readToDS.bits.dbid       := reg.dbidVec(reg.saveBeat)
  io.readToDS.bits.beatNum    := reg.saveBeat
  io.readToDS.bits.critical   := reg.critical
  io.readToDS.bits.repl       := reg.isRepl
  io.readToDS.bits.qos        := reg.task.qos
  HAssert.withEn(!reg.isSaveAll, io.readToDS.valid)

  // Assert for replace
  HAssert.withEn(!(io.readToDB.valid ^ io.readToDS.valid), reg.isRepl)
  HAssert.withEn(!(io.readToDB.fire  ^ io.readToDS.fire),  reg.isRepl)
  HAssert.withEn(!(reg.sReadVec ^ reg.sSaveVec).orR,       reg.isRepl)

  /*
   * Send Release to DBIDCtrl and DataBuffer
   */
  io.release.valid            := reg.isClean
  io.release.bits.dataVec     := (reg.dataVec.asUInt & reg.task.dataVec.asUInt).asBools
  io.release.bits.dbidVec     := reg.dbidVec
  HAssert.withEn(!io.release.bits.isZero, io.release.valid)

  /*
   * Send response to Bankend
   */
  io.resp.valid               := reg.isResp
  io.resp.bits.hnTxnID        := reg.task.hnTxnID
  HAssert.withEn(reg.isReadAll & reg.isSendAll & reg.isSaveAll, io.resp.valid)

  /*
   * Modify Ctrl Machine
   */
  // Get next task
  val taskHit = reg.isValid & io.task.valid & io.task.bits.hnTxnID === reg.task.hnTxnID
  when(taskHit) {
    next.task     := io.task.bits
    HAssert(reg.isAlloc)
    HAssert(io.task.bits.dataOp.isValid)
    reg.dataVec.zip(io.task.bits.dataVec).map { case(v, t) => HAssert.withEn(v, t) } // Check that the data location of the operation is not out of bounds
  }

  // Get next dataVec and dbidVec
  val cleanHit = reg.isValid & io.clean.valid & io.clean.bits.hnTxnID === reg.task.hnTxnID
  when(io.alloc.fire) {
    next.dataVec      := io.alloc.bits.dataVec
    next.dbidVec      := io.alloc.bits.dbidVec
  }.elsewhen(cleanHit) {
    next.task.dataVec := io.clean.bits.dataVec
    HAssert(!io.clean.bits.isZero)
    HAssert(reg.isAlloc)
    HAssert(!taskHit)
  }.elsewhen(io.release.fire) {
    next.dataVec      := VecInit((reg.dataVec.asUInt & ~io.release.bits.dataVec.asUInt).asBools)
  }

  // Set next sXVec and wXVec
  def setNextXXV(name: String, nsxv: UInt, nwxv: UInt, sxv: UInt, wxv: UInt, set: Bool, sFire: Bool, sBeat: UInt, wFire: Bool, wBeat: UInt) = {
    // reset
    when(io.alloc.fire) {
      nsxv := 0.U
      nwxv := 0.U
    // set
    }.elsewhen(taskHit & set) {
      nsxv := io.task.bits.dataVec.asUInt
      nwxv := io.task.bits.dataVec.asUInt
      HAssert(sxv === 0.U, name)
      HAssert(wxv === 0.U, name)
    // modify
    }.otherwise {
      // sXVec
      nsxv := PriorityMux(Seq(
        (sFire & wFire) -> (sxv & ~(UIntToOH(sBeat) | UIntToOH(wBeat))),
         sFire          -> (sxv & ~ UIntToOH(sBeat)),
         wFire          -> (sxv & ~ UIntToOH(wBeat)),
         true.B         ->  sxv
      ))
      HAssert.withEn(sxv(sBeat), sFire, name)
      HAssert.withEn(sBeat =/= wBeat, sFire & wFire, name)
      // wXVec
      when(wFire) {
        nwxv := wxv & ~ UIntToOH(wBeat)
        HAssert(wxv(wBeat), name)
      }
    }
  }

  // Read
  val dsWriDBHit = reg.isValid & io.dsWriDB.valid & io.dsWriDB.bits.dcid === io.dcid
  setNextXXV("read", next.sReadVec, next.wReadVec, reg.sReadVec, reg.wReadVec, io.task.bits.dataOp.readToDB,
    io.readToDB.fire, io.readToDB.bits.beatNum, dsWriDBHit, io.dsWriDB.bits.beatNum)

  // Send
  val txDatHit = reg.isValid & io.txDatFire.valid & io.txDatFire.bits.dcid === io.dcid
  setNextXXV("send", next.sSendVec, next.wSendVec, reg.sSendVec, reg.wSendVec, io.task.bits.dataOp.readToCHI,
    io.readToCHI.fire, io.readToCHI.bits.beatNum, txDatHit, io.txDatFire.bits.beatNum)

  // Save
  val dbWriDSHit = reg.isValid & io.dbWriDS.valid & io.dbWriDS.bits.dcid === io.dcid
  setNextXXV("save", next.sSaveVec, next.wSaveVec, reg.sSaveVec, reg.wSaveVec, io.task.bits.dataOp.readToDS,
    io.readToDS.fire, io.readToDS.bits.beatNum, dbWriDSHit, io.dbWriDS.bits.beatNum)

  // Get next critical
  when(io.alloc.fire | taskHit) {
    next.critical := false.B
    HAssert.withEn(!reg.critical, taskHit)
  }.otherwise {
    next.critical := PriorityMux(Seq(
      io.readToDB.fire  -> (PopCount(reg.sReadVec) > 1.U),
      io.readToCHI.fire -> (PopCount(reg.sSendVec) > 1.U),
      io.readToDS.fire  -> (PopCount(reg.sSaveVec) > 1.U),
      true.B            -> reg.critical
    ))
  }

  // Get next hnTxnID
  val updHnTxnIDHit   = reg.isValid & io.updHnTxnID.valid & io.updHnTxnID.bits.before === reg.task.hnTxnID
  when(io.alloc.fire) {
    next.task.hnTxnID := io.alloc.bits.hnTxnID
  }.elsewhen(updHnTxnIDHit) {
    next.task.hnTxnID := io.updHnTxnID.bits.next
    HAssert(reg.isAlloc)
  }

  // Get next state
  switch(reg.state) {
    is(FREE) {
      when(io.alloc.fire) {
        next.state := ALLOC
      }
    }
    is(ALLOC) {
      when(taskHit) {
        next.state := PriorityMux(Seq(
          io.task.bits.dataOp.repl -> REPL,
          io.task.bits.dataOp.read -> READ,
          io.task.bits.dataOp.send -> SEND,
          io.task.bits.dataOp.save -> SAVE,
        ))
      }.elsewhen(cleanHit) {
        next.state := CLEAN
      }
    }
    is(REPL) {
      when(next.isReadAll & next.isSaveAll) {
        next.state := Mux(next.isSendAll, RESP, SEND)
      }
    }
    is(READ) {
      when(next.isReadAll) {
        next.state := PriorityMux(Seq(
          (reg.task.dataOp.send & !next.isSendAll) -> SEND,
          (reg.task.dataOp.save & !next.isSaveAll) -> SAVE,
           true.B                                  -> RESP
        ))
      }
    }
    is(SEND) {
      when(next.isSendAll) {
        next.state := Mux(reg.task.dataOp.save & !next.isSaveAll, SAVE, RESP)
      }
    }
    is(SAVE) {
      when(next.isSaveAll) {
        next.state := RESP
      }
    }
    is(RESP) {
      when(io.resp.fire) {
        next.state := ALLOC
      }
    }
    is(CLEAN) {
      when(io.release.fire) {
        next.state := Mux(next.isZero, FREE, ALLOC)
      }
    }
  }
  HAssert.withEn(reg.isResp  | reg.isClean,   next.state < reg.state)
  HAssert.withEn(next.isFree | next.isAlloc,  next.state < reg.state)
  HAssert.withEn(reg.isFree  | reg.isClean,   next.isFree)
  HAssert.withEn(reg.isFree  | reg.isAlloc | reg.isResp | reg.isClean,  next.isAlloc)

  /*
   * Set new task
   */
  val set = io.alloc.fire | reg.isValid; dontTouch(set)
  when(set) { reg := next }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(reg.isFree | updHnTxnIDHit, TIMEOUT_DATACM, cf"TIMEOUT: DataCM State[${reg.state}]")
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Ctrl Machine ------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class DataCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // From Backend
    val updHnTxnID    = Flipped(Valid(new UpdHnTxnID))
    val reqDBIn       = Flipped(Decoupled(new HnTxnID with HasDataVec))
    val task          = Flipped(Valid(new DataTask)) // broadcast signal
    val resp          = Valid(new HnTxnID)
    val clean         = Flipped(Valid(new HnTxnID with HasDataVec)) // broadcast signal
    // To DS/DB
    val readToDB      = Decoupled(new ReadDS)
    val readToDS      = Decoupled(new ReadDB)
    val readToCHI     = Decoupled(new ReadDB)
    // To DBID Pool
    val reqDBOut      = Decoupled(Vec(djparam.nrBeat, Bool()))
    val dbidResp      = Vec(djparam.nrBeat, Flipped(UInt(dbIdBits.W)))
    // From/To CHI
    val getChiDat     = new DJBundle {
      val valid       = Input(Bool())
      val dcid        = Input(UInt(dcIdBits.W))
      val bits        = Output(new DataFlit)
    }
    val getDBID       = new DJBundle {
      val valid       = Input(Bool())
      val TxnID       = Input(UInt(ChiTxnIdBits.W))
      val DataID      = Input(UInt(2.W))
      val dbid        = Output(UInt(dbIdBits.W))
    }
    // Other
    val release       = Valid(new DBIDVec with HasDataVec)
    val dsWriDB       = Flipped(Valid(new DCID with HasBeatNum)) // broadcast signal
    val txDatFire     = Flipped(Valid(new DCID with HasBeatNum)) // broadcast signal
    val dbWriDS       = Flipped(Valid(new DCID with HasBeatNum)) // broadcast signal
  })
  // HAssert is request DataBuffer, size can be zero
  HAssert.withEn(!io.reqDBIn.bits.isZero,   io.reqDBIn.valid)
  HAssert.withEn(!io.task.bits.isZero,      io.task.valid)

  /*
   * Module declaration
   */
  val entries     = Seq.fill(nrDataCM) { Module(new DataCtrlEntry()) }
  val taskFireReg = RegNext(io.task.fire)
  val taskReg     = RegEnable(io.task.bits, io.task.fire)
  val dbgVec      = VecInit(entries.map(_.io.state))
  dontTouch(dbgVec)
  dbgVec.zipWithIndex.foreach { case(src, i) =>
    dbgVec.zipWithIndex.foreach { case (sink, j) =>
      HAssert.withEn(!(src.bits.hnTxnID === sink.bits.hnTxnID), src.valid & sink.valid & i.U =/= j.U)
      HAssert.withEn(!(src.bits.dbidVec(0) === sink.bits.dbidVec(0) & src.bits.dataVec(0) & sink.bits.dataVec(0)), src.valid & sink.valid & i.U =/= j.U)
      HAssert.withEn(!(src.bits.dbidVec(1) === sink.bits.dbidVec(1) & src.bits.dataVec(1) & sink.bits.dataVec(1)), src.valid & sink.valid & i.U =/= j.U)
    }
  }

  /*
   * Receive and send ReqDB
   */
  val freeDCVec             = VecInit(entries.map(_.io.alloc.ready))
  val hasFreeDC             = freeDCVec.asUInt.orR
  val freeDCID              = WireInit(PriorityEncoder(freeDCVec)); dontTouch(freeDCID)

  // reqDBOut
  io.reqDBOut.valid         := io.reqDBIn.valid & hasFreeDC
  io.reqDBOut.bits          := io.reqDBIn.bits.dataVec
  // ready
  io.reqDBIn.ready          :=  io.reqDBOut.ready & hasFreeDC

  /*
   * Send alloc and task to entries
   */
  entries.zipWithIndex.foreach { case (e, i) =>
    e.io.dcid               := i.U
    // Alloc
    e.io.alloc.valid        := io.reqDBIn.fire & freeDCID === i.U
    e.io.alloc.bits.hnTxnID := io.reqDBIn.bits.hnTxnID
    e.io.alloc.bits.dataVec := io.reqDBIn.bits.dataVec
    e.io.alloc.bits.dbidVec := io.dbidResp
    HAssert.withEn(io.reqDBOut.fire, e.io.alloc.fire, cf"DCID[${i.U}]")
    // Task
    e.io.task.valid         := taskFireReg
    e.io.task.bits          := taskReg
  }
  HAssert.withEn(io.reqDBOut.fire & PopCount(entries.map(_.io.alloc.fire)) === 1.U, io.reqDBIn.fire)
  HAssert.withEn(PopCount(entries.map(e => e.io.state.valid & e.io.state.bits.hnTxnID === taskReg.hnTxnID)) === 1.U, taskFireReg)

  /*
   * Connect CM <- IO
   */
  entries.foreach(_.io.updHnTxnID := io.updHnTxnID)
  entries.foreach(_.io.clean      := io.clean)
  entries.foreach(_.io.dsWriDB    := io.dsWriDB)
  entries.foreach(_.io.txDatFire  := io.txDatFire)
  entries.foreach(_.io.dbWriDS    := io.dbWriDS)
  HAssert.withEn(entries.map(e => e.io.state.valid & e.io.state.bits.hnTxnID === io.updHnTxnID.bits.before).reduce(_ | _), io.updHnTxnID.valid)

  /*
   * Connect IO <- CM
   */
  io.resp           := fastRRArb.validOut(entries.map(_.io.resp))
  io.release        := fastRRArb.validOut(entries.map(_.io.release))
  io.getChiDat.bits := VecInit(entries.map(_.io.txDatBits))(io.getChiDat.dcid)
  HAssert.withEn(dbgVec(io.getChiDat.dcid).valid, io.getChiDat.valid)
  // Get DBID by TxnID and DBID from CHI RxDat
  val getDBIDHitVec = VecInit(entries.map(e => e.io.state.valid & io.getDBID.TxnID === e.io.state.bits.hnTxnID))
  val getDCID       = PriorityEncoder(getDBIDHitVec)
  val getBeatNum    = Mux(io.getDBID.DataID === "b10".U, 1.U, 0.U)
  io.getDBID.dbid   := VecInit(entries.map(_.io.state.bits.dbidVec))(getDCID)(getBeatNum)
  HAssert.withEn(PopCount(getDBIDHitVec) === 1.U, io.getDBID.valid)
  HAssert.withEn(VecInit(entries.map(_.io.state.bits.dataVec))(getDCID)(getBeatNum), io.getDBID.valid)


  /*
   * Connect ReadToX
   */
  def connectReadToX[T <: Bundle with HasCritical](inVec: Vec[DecoupledIO[T]], out: DecoupledIO[T]): Unit = {
    val criticalVec   = VecInit(inVec.map(e => e.valid & e.bits.critical))
    val hasCritical   = criticalVec.asUInt.orR
    val criticalId    = PriorityEncoder(criticalVec)
    when(hasCritical) {
      inVec.foreach(_.ready := false.B) // init
      out <> inVec(criticalId)
    }.otherwise {
      out <> fastQosRRArb(inVec)
    }
    HAssert(PopCount(criticalVec) <= 1.U)
  }
  // Get replace vector
  val replVec     = VecInit(entries.map(e => e.io.readForRepl))
  val replCVec    = VecInit(entries.map(e => e.io.readForRepl & e.io.readToDS.bits.critical)) // replace critical vector
  val hasRepl     = replVec.asUInt.orR
  val hasReplC    = replCVec.asUInt.orR
  val replDCID    = WireInit(Mux(hasReplC, PriorityEncoder(replCVec), PriorityEncoder(replVec))); dontTouch(replDCID)
  val readToDBVec = VecInit(entries.map(_.io.readToDB))
  val readToDSVec = VecInit(entries.map(_.io.readToDS))
  // Has replace
  when(hasRepl) {
    readToDBVec.foreach(_.ready := false.B)   // init
    readToDSVec.foreach(_.ready := false.B)   // init
    io.readToDB.valid           := io.readToDS.ready                      // Read to DB valid
    io.readToDS.valid           := io.readToDB.ready                      // Read to DS valid
    io.readToDB.bits            := readToDBVec(replDCID).bits             // Read to DB bits
    io.readToDS.bits            := readToDSVec(replDCID).bits             // Read to DS bits
    readToDBVec(replDCID).ready := io.readToDS.ready & io.readToDB.ready  // Read to DB ready
    readToDSVec(replDCID).ready := io.readToDS.ready & io.readToDB.ready  // Read to DS ready
    HAssert(readToDBVec(replDCID).valid)
    HAssert(readToDSVec(replDCID).valid)
  // No Replace
  }.otherwise {
    connectReadToX(readToDBVec, io.readToDB)  // Read to DB
    connectReadToX(readToDSVec, io.readToDS)  // Read to DS
  }
  // Read to CHI
  connectReadToX(VecInit(entries.map(_.io.readToCHI)), io.readToCHI)
  // HAssert
  HAssert(!(io.readToDB.fire  ^ PopCount(entries.map(_.io.readToDB.fire))  === 1.U))
  HAssert(!(io.readToDS.fire  ^ PopCount(entries.map(_.io.readToDS.fire))  === 1.U))
  HAssert(!(io.readToCHI.fire ^ PopCount(entries.map(_.io.readToCHI.fire)) === 1.U))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}




















