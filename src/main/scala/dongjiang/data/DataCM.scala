package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.CutHnTxnID
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
  val width   = 4
  val FREE    = 0x0.U // -> ALLOC
  val ALLOC   = 0x1.U // *
  val REPL    = 0x2.U // * // Read DS and DB at the same time to replace
  val READ    = 0x3.U // * // Read DS and save in DB
  val WAIT    = 0x4.U // * // Wait data from DS to DB
  val SEND    = 0x5.U // * // Read DB and send to CHI
  val SAVE    = 0x6.U // * // Read DB adn send to DS
  val CLEAN   = 0x7.U // * // Clean DB mask and DBIDPool
  val RESP    = 0x8.U // -> ALLOC/FREE // Send resp to Backend
  val TOFREE  = 0x9.U // -> FREE       // Can be free when sending is all false

  // Note: with * must use dataOp.getNextStateS
}

trait HasCtrlMes { this: DJBundle =>
  // Without repl flag:
  //    Free -> Alloc -> Read -> Wait -> Send -> Save -> Clean -> Resp --> ToFree -> Free
  // With repl flag:
  //    Free -> Alloc -> Replace -> Wait -> Send -> Clean -> Resp -> ToFree -> Free
  // Without clean:
  //    Free -> Alloc -> Read -> Wait -> Send -> Save -> Resp -> Alloc

  val state     = UInt(CTRLSTATE.width.W)
  val opBeat    = UInt(log2Ceil(djparam.nrBeat).W)    // The beat block being operate (read/send/save/send)
  val wRBeat    = UInt(log2Ceil(djparam.nrBeat+1).W)  // Wait read DS data to DB
  val wSBeat    = UInt(log2Ceil(djparam.nrBeat+1).W)  // Wait read DB data to CHI

  def isCritical= opBeat === (djparam.nrBeat-1).U
  def waiting   = wRBeat.asUInt =/= 0.U
  def waitall   = wRBeat.asUInt === 0.U
  def sending   = wSBeat.asUInt =/= 0.U
  def sendall   = wSBeat.asUInt === 0.U

  def isFree    = state === FREE
  def isValid   = !isFree
  def isAlloc   = state === ALLOC
  def isRepl    = state === REPL
  def isRead    = state === READ
  def isWait    = state === WAIT
  def isSend    = state === SEND
  def isSave    = state === SAVE
  def isClean   = state === CLEAN
  def isResp    = state === RESP
  def isToFree  = state === TOFREE
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
    // From/To Backend
    val cutHnTxnID  = Flipped(Valid(new CutHnTxnID))
    val alloc       = Flipped(Decoupled(new HnTxnID with HasDataVec))
    val task        = Flipped(Valid(new DataTask)) // broadcast signal
    val resp        = Decoupled(new HnTxnID)
    // To DS/DB
    val readForRepl = Output(Bool())
    val readToDB    = Decoupled(new ReadDS)
    val readToDS    = Decoupled(new ReadDB)
    val readToCHI   = Decoupled(new ReadDB)
    // To DBID Pool
    val releaseDB   = Decoupled(new HnTxnID with HasDataVec)
    // Other
    val dsWriDB     = Flipped(Valid(new DCID)) // broadcast signal
    val txDatFire   = Flipped(Valid(new DCID)) // broadcast signal
    val txDatBits   = Output(new DataFlit)
    // State
    val state       = Valid(new HnTxnID)
  })

  /*
   * Reg and Wire declaration
   */
  require(djparam.nrBeat == 2)
  // set reg.dataVec when alloc
  // set reg.task.dataVec when io.data hit
  val reg = RegInit(new PackDataTask with HasCtrlMes with HasDataVec {
    def getBeatNum  = Mux(opBeat === 0.U, PriorityEncoder(task.dataVec), 1.U)
    def opAll       = Mux(task.isFullSize, opBeat === 1.U, opBeat === 0.U)  // The beat block is operate all (read/send/save/send)
  }.Lit(_.state -> FREE))
  val next = WireInit(reg)

  /*
    * Connect io
   */
  io.txDatBits          := reg.task.txDat
  io.txDatBits.DataID   := PriorityMux(Seq( // TODO: parameterization
    reg.task.isFullSize -> Mux(reg.wSBeat === 2.U, "b00".U, "b10".U),
    reg.task.dataVec(1) -> "b10".U,
    reg.task.dataVec(0) -> "b00".U
  ))
  io.state.valid        := reg.isValid
  io.state.bits.hnTxnID := reg.task.hnTxnID

  /*
   * Receive Req
   */
  io.alloc.ready := reg.isFree

  /*
   * Read DS / DB
   */
  io.readForRepl              := reg.isRepl
  // read DS to DB
  io.readToDB.valid           := reg.isRepl | reg.isRead
  io.readToDB.bits.ds         := reg.task.ds
  io.readToDB.bits.dcid       := io.dcid
  io.readToDB.bits.dbid       := DontCare // remap in DataCM
  io.readToDB.bits.beatNum    := reg.getBeatNum
  io.readToDB.bits.critical   := reg.isCritical
  // to DB to DS
  io.readToDS.valid           := reg.isRepl | reg.isSave
  io.readToDS.bits.ds         := reg.task.ds
  io.readToDS.bits.dcid       := io.dcid
  io.readToDS.bits.dbid       := DontCare // remap in DataCM
  io.readToDS.bits.beatNum    := reg.getBeatNum
  io.readToDS.bits.critical   := reg.isCritical
  io.readToDS.bits.repl       := reg.isRepl
  // to DB to CHI
  io.readToCHI.valid          := reg.isSend
  io.readToCHI.bits.ds        := DontCare
  io.readToCHI.bits.dcid      := io.dcid
  io.readToCHI.bits.dbid      := DontCare // remap in DataCM
  io.readToCHI.bits.beatNum   := reg.getBeatNum
  io.readToCHI.bits.critical  := reg.isCritical
  io.readToCHI.bits.repl      := false.B
  HAssert.withEn(!(io.readToDB.fire ^ io.readToDS.fire), reg.isRepl)

  /*
   * Clean DBIDPool
   */
  io.releaseDB.valid          := reg.isClean
  io.releaseDB.bits.hnTxnID   := reg.task.hnTxnID
  io.releaseDB.bits.dataVec   := reg.task.dataVec

  /*
   * Send response to Bankend
   */
  io.resp.valid               := reg.isResp & reg.sendall
  io.resp.bits.hnTxnID        := reg.task.hnTxnID

  /*
   * Modify Ctrl Machine
   */
  // Get next task
  val taskHit = reg.isValid & io.task.valid & io.task.bits.hnTxnID === reg.task.hnTxnID
  when(io.alloc.fire) {
    next.task     := 0.U.asTypeOf(new DataTask)
    HAssert.withEn(io.task.bits.hnTxnID =/= io.alloc.bits.hnTxnID, io.task.valid)
  }.elsewhen(taskHit) {
    next.task     := io.task.bits
    HAssert(reg.isAlloc)
    reg.dataVec.zip(io.task.bits.dataVec).map { case(v, t) => HAssert.withEn(v, t) }
  }

  // Get next dataVec
  when(io.alloc.fire) {
    next.dataVec  := io.alloc.bits.dataVec
  }.elsewhen(taskHit & io.task.bits.dataOp.onlyClean){
    next.dataVec  := VecInit((reg.dataVec.asUInt & ~io.task.bits.dataVec.asUInt).asBools)
  }.elsewhen(next.isClean) {
    next.dataVec  := VecInit((reg.dataVec.asUInt & ~reg.task.dataVec.asUInt).asBools)
  }

  // Get next wait read beat
  val dsWriDBHit  = reg.isValid & io.dsWriDB.valid & io.dsWriDB.bits.dcid === io.dcid
  when(io.alloc.fire) {
    next.wRBeat   := 0.U
  }.elsewhen(taskHit) {
    next.wRBeat   := Mux(io.task.bits.dataOp.read | io.task.bits.dataOp.repl, PopCount(io.task.bits.dataVec), 0.U)
    HAssert(reg.wRBeat === 0.U)
  }.elsewhen(dsWriDBHit) {
    next.wRBeat   := reg.wRBeat - 1.U
    HAssert(reg.wRBeat > 0.U)
    HAssert(reg.isWait | ((reg.isRepl |reg.isRead) & reg.opBeat =/= 0.U))
  }

  // Get next wait send beat
  val txDatHit    = reg.isValid & io.txDatFire.valid & io.txDatFire.bits.dcid === io.dcid
  when(io.alloc.fire) {
    next.wSBeat   := 0.U
  }.elsewhen(taskHit) {
    next.wSBeat   := Mux(io.task.bits.dataOp.send | io.task.bits.dataOp.repl, PopCount(io.task.bits.dataVec), 0.U)
    HAssert(reg.wSBeat === 0.U)
  }.elsewhen(txDatHit) {
    next.wSBeat   := reg.wSBeat - 1.U
    HAssert(reg.wSBeat > 0.U)
    HAssert(reg.isSend | reg.isSave | reg.isClean | reg.isResp | reg.isToFree | reg.isAlloc) // TODO: has risk when state is alloc
  }

  // Get next opBeat
  when(io.alloc.fire) {
    next.opBeat   := 0.U
  }.elsewhen(io.readToDB.fire | io.readToDS.fire | io.readToCHI.fire) {
    next.opBeat   := reg.opBeat + reg.task.isFullSize
    HAssert.withEn(next.opBeat === 0.U, next.state =/= reg.state)
    HAssert.withEn(next.opBeat === 1.U, next.state === reg.state)
    require(djparam.nrBeat == 2)
  }
  HAssert.withEn(reg.opBeat === 0.U, taskHit)

  // Get next hnTxnID
  val repIdHit = reg.isValid & io.cutHnTxnID.valid & io.cutHnTxnID.bits.before === reg.task.hnTxnID
  when(io.alloc.fire) {
    next.task.hnTxnID := io.alloc.bits.hnTxnID
  }.elsewhen(repIdHit) {
    next.task.hnTxnID := io.cutHnTxnID.bits.next
    HAssert(reg.isAlloc)
  }.elsewhen(taskHit) {
    next.task.hnTxnID := io.task.bits.hnTxnID
  }

  // Get next state
  val replFire = io.readToDB.fire & io.readToDS.fire
  switch(reg.state) {
    is(FREE) {
      when(io.alloc.fire)                 { next.state := ALLOC }
    }
    is(ALLOC) {
      when(taskHit)                       { next.state := io.task.bits.dataOp.getNextState(ALLOC) }
    }
    is(REPL) {
      when(replFire & reg.opAll)          { next.state := reg.task.dataOp.getNextState(REPL) }
    }
    is(READ) {
      when(io.readToDB.fire & reg.opAll)  { next.state := reg.task.dataOp.getNextState(READ) }
    }
    is(WAIT) {
      when(reg.waitall)                   { next.state := reg.task.dataOp.getNextState(WAIT) }
    }
    is(SEND) {
      when(io.readToCHI.fire & reg.opAll) { next.state := reg.task.dataOp.getNextState(SEND) }
    }
    is(SAVE) {
      when(io.readToDS.fire & reg.opAll)  { next.state := reg.task.dataOp.getNextState(SAVE) }
    }
    is(CLEAN) {
      when(io.releaseDB.fire)             { next.state := reg.task.dataOp.getNextState(CLEAN) }
    }
    is(RESP) {
      when(io.resp.fire)                  { next.state := Mux(reg.isZero, TOFREE, ALLOC) }
    }
    is(TOFREE) {
      when(reg.sendall)                   { next.state := FREE }
    }
  }
  HAssert.withEn(reg.isFree  | reg.isToFree,              next.isFree)
  HAssert.withEn(reg.isFree  | reg.isResp | reg.isAlloc,  next.isAlloc)
  HAssert.withEn(reg.isResp  | reg.isToFree,              next.state < reg.state)
  HAssert.withEn(next.isFree | next.isAlloc,              next.state < reg.state)


  /*
   * Set new task
   */
  val set = io.alloc.fire | reg.isValid; dontTouch(set)
  when(set) { reg := next }

  /*
   * Check timeout
   */
  HAssert.checkTimeout(reg.isFree, TIMEOUT_DATACM, cf"TIMEOUT: DataCM State[${reg.state}]")
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
    val cutHnTxnID    = Flipped(Valid(new CutHnTxnID))
    val reqDBIn       = Flipped(Decoupled(new HnTxnID with HasDataVec))
    val task          = Flipped(Decoupled(new DataTask)) // broadcast signal
    val resp          = Valid(new HnTxnID)
    // To DS/DB
    val readToDB      = Decoupled(new ReadDS)
    val readToDS      = Decoupled(new ReadDB)
    val readToCHI     = Decoupled(new ReadDB)
    val cleanMaskVec  = Vec(djparam.nrBeat, Valid(new DBID))
    // From/To DBID Pool
    val reqDBOut      = Decoupled(new HnTxnID with HasDataVec)
    val releaseDB     = Valid(new HnTxnID with HasDataVec)
    val getDBIDVec    = Vec(4, new GetDBID)
    // From/To CHI
    val txDataDCID    = Input(UInt(dcIdBits.W))
    val txDatBits     = Output(new DataFlit)
    // Other
    val dsWriDB       = Flipped(Valid(new DCID)) // broadcast signal
    val txDatFire     = Flipped(Valid(new DCID)) // broadcast signal
  })
  // HAssert is request DataBuffer, size can be zero
  HAssert.withEn(!io.reqDBIn.bits.isZero,   io.reqDBIn.valid)
  HAssert.withEn(!io.reqDBOut.bits.isZero,  io.reqDBOut.valid)
  HAssert.withEn(!io.releaseDB.bits.isZero, io.releaseDB.valid)
  HAssert.withEn(!io.task.bits.isZero,      io.task.valid & io.task.bits.dataOp.reqs)

  /*
   * Module declaration
   */
  val entries = Seq.fill(nrDataCM) { Module(new DataCtrlEntry()) }
  val taskReg = RegEnable(io.task.bits, io.task.fire)
  val dbgVec  = VecInit(entries.map(_.io.state))
  dontTouch(dbgVec)

  /*
   * Receive and send ReqDB
   */
  val freeDCVec             = VecInit(entries.map(_.io.alloc.ready))
  val hasFreeDC             = freeDCVec.asUInt.orR
  val freeDCID              = WireInit(PriorityEncoder(freeDCVec)); dontTouch(freeDCID)
  val taskReqDB             = io.task.valid & io.task.bits.dataOp.reqs
  // reqDBOut
  io.reqDBOut.valid         := (io.reqDBIn.valid | taskReqDB) & hasFreeDC
  io.reqDBOut.bits.hnTxnID  := Mux(taskReqDB, io.task.bits.hnTxnID, io.reqDBIn.bits.hnTxnID)
  io.reqDBOut.bits.dataVec  := Mux(taskReqDB, io.task.bits.dataVec, io.reqDBIn.bits.dataVec)
  // ready
  io.task.ready             := (io.reqDBOut.ready & hasFreeDC) | !taskReqDB
  io.reqDBIn.ready          :=  io.reqDBOut.ready & hasFreeDC  & !taskReqDB

  /*
   * Send alloc and task to entries
   */
  entries.zipWithIndex.foreach { case (e, i) =>
    e.io.dcid               := i.U
    // Alloc
    e.io.alloc.valid        := (io.reqDBIn.valid | taskReqDB) & io.reqDBOut.ready & freeDCID === i.U
    e.io.alloc.bits.hnTxnID := Mux(taskReqDB, io.task.bits.hnTxnID, io.reqDBIn.bits.hnTxnID)
    e.io.alloc.bits.dataVec := Mux(taskReqDB, io.task.bits.dataVec, io.reqDBIn.bits.dataVec)
    HAssert.withEn(io.reqDBOut.fire, e.io.alloc.fire, cf"DCID[${i.U}]")
    // Task
    e.io.task.valid         := RegNext(io.task.fire)
    e.io.task.bits          := taskReg
  }
  HAssert.withEn(io.reqDBOut.fire & PopCount(entries.map(_.io.alloc.fire)) === 1.U, io.reqDBIn.fire)
  HAssert.withEn(io.reqDBOut.fire & PopCount(entries.map(_.io.alloc.fire)) === 1.U, io.task.fire & taskReqDB)
  HAssert.withEn(PopCount(entries.map(e => e.io.state.valid & e.io.state.bits.hnTxnID === io.task.bits.hnTxnID)) === 1.U, io.task.fire & !taskReqDB)
  HAssert.withEn(io.task.bits.dataOp.valid ^ io.task.bits.dataOp.repl, io.task.valid)

  /*
   * Connect CM <- IO
   */
  entries.foreach(_.io.cutHnTxnID   := io.cutHnTxnID)
  entries.foreach(_.io.dsWriDB      := io.dsWriDB)
  entries.foreach(_.io.txDatFire    := io.txDatFire)
  HAssert.withEn(entries.map(e => e.io.state.valid & e.io.state.bits.hnTxnID === io.cutHnTxnID.bits.before).reduce(_ | _), io.cutHnTxnID.valid)

  /*
   * Connect IO <- CM
   */
  io.resp      := fastRRArb.validOut(entries.map(_.io.resp))
  io.txDatBits := VecInit(entries.map(_.io.txDatBits))(io.txDataDCID)

  /*
   * Connect ReadToX
   */
  def connectReadToX[T <: Data with HasCritical](inVec: Vec[DecoupledIO[T]], out: DecoupledIO[T]): Unit = {
    val criticalVec   = VecInit(inVec.map(e => e.valid & e.bits.critical))
    val hasCritical   = criticalVec.asUInt.orR
    val criticalId    = PriorityEncoder(criticalVec)
    when(hasCritical) {
      inVec.foreach(_.ready := false.B) // init
      out <> inVec(criticalId)
    }.otherwise {
      out <> fastRRArb(inVec)
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
   * Release DBID
   */
  io.releaseDB  := fastRRArb.validOut(entries.map(_.io.releaseDB))
  io.cleanMaskVec.zipWithIndex.foreach { case(c, i) =>
    c.valid     := io.releaseDB.valid & io.releaseDB.bits.dataVec(i)
    c.bits.dbid := io.getDBIDVec(3).dbidVec(i) // Set dbid
  }

  /*
   * Get DBID from DBIDPool
   */
  // Get dbid
  val hnTxnIDVec  = VecInit(entries.map(_.io.state.bits.hnTxnID))
  io.getDBIDVec(0).hnTxnID  := hnTxnIDVec(io.readToDB.bits.dcid)
  io.getDBIDVec(1).hnTxnID  := hnTxnIDVec(io.readToDS.bits.dcid)
  io.getDBIDVec(2).hnTxnID  := hnTxnIDVec(io.readToCHI.bits.dcid)
  io.getDBIDVec(3).hnTxnID  := hnTxnIDVec(PriorityEncoder(entries.map(_.io.releaseDB.fire)))
  // Set dbid
  io.readToDB.bits.dbid     := io.getDBIDVec(0).dbidVec(io.readToDB.bits.beatNum)
  io.readToDS.bits.dbid     := io.getDBIDVec(1).dbidVec(io.readToDS.bits.beatNum)
  io.readToCHI.bits.dbid    := io.getDBIDVec(2).dbidVec(io.readToCHI.bits.beatNum)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}




















