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
  val REPL    = 0x2.U // Read DS and DB at the same time to replace
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

  val state     = UInt(CTRLSTATE.width.W)
  val opBeat    = UInt(log2Ceil(djparam.nrBeat).W)    // The beat block being operate (repl/read/send/save)
  val wRBeat    = UInt(log2Ceil(djparam.nrBeat+1).W)  // Wait read DS data to DB
  val wSBeat    = UInt(log2Ceil(djparam.nrBeat+1).W)  // Wait read DB data to CHI

  def isCritical= opBeat =/= 0.U
  def waiting   = wRBeat.asUInt =/= 0.U
  def waitall   = wRBeat.asUInt === 0.U
  def sending   = wSBeat.asUInt =/= 0.U
  def sendall   = wSBeat.asUInt === 0.U

  def isFree    = state === FREE
  def isValid   = !isFree
  def isAlloc   = state === ALLOC
  def isRepl    = state === REPL
  def isRead    = state === READ
  def isSend    = state === SEND
  def isSave    = state === SAVE
  def isResp    = state === RESP
  def isClean   = state === CLEAN
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
    val alloc       = Flipped(Decoupled(new HnTxnID with HasDataVec))
    val task        = Flipped(Valid(new DataTask)) // broadcast signal
    val resp        = Decoupled(new HnTxnID)
    val clean       = Flipped(Valid(new HnTxnID with HasDataVec)) // broadcast signal
    // To DS/DB
    val readForRepl = Output(Bool())
    val readToDB    = Decoupled(new ReadDS)
    val readToDS    = Decoupled(new ReadDB)
    val readToCHI   = Decoupled(new ReadDB)
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
  val reg = RegInit(new PackDataTask with HasCtrlMes with HasDataVec {
    def opBeatNum  = Mux(opBeat === 0.U, PriorityEncoder(task.dataVec), 1.U)
    def willOpAll  = Mux(task.isFullSize, opBeat === 1.U, opBeat === 0.U)
  }.Lit(_.state -> FREE))
  val next = WireInit(reg)
  require(djparam.nrBeat == 2)

  /*
   * Set QoS
   */
  io.readToDB.bits.qos  := reg.task.qos
  io.readToDS.bits.qos  := reg.task.qos
  io.readToCHI.bits.qos := reg.task.qos

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
  io.readToDB.bits.beatNum    := reg.opBeatNum
  io.readToDB.bits.critical   := reg.isCritical
  // to DB to DS
  io.readToDS.valid           := reg.isRepl | (reg.isSave & reg.waitall)
  io.readToDS.bits.ds         := reg.task.ds
  io.readToDS.bits.dcid       := io.dcid
  io.readToDS.bits.dbid       := DontCare // remap in DataCM
  io.readToDS.bits.beatNum    := reg.opBeatNum
  io.readToDS.bits.critical   := reg.isCritical
  io.readToDS.bits.repl       := reg.isRepl
  // to DB to CHI
  io.readToCHI.valid          := reg.isSend & reg.waitall
  io.readToCHI.bits.ds        := DontCare
  io.readToCHI.bits.dcid      := io.dcid
  io.readToCHI.bits.dbid      := DontCare // remap in DataCM
  io.readToCHI.bits.beatNum   := reg.opBeatNum
  io.readToCHI.bits.critical  := reg.isCritical
  io.readToCHI.bits.repl      := false.B
  HAssert.withEn(!(io.readToDB.fire ^ io.readToDS.fire), reg.isRepl)

  /*
   * Send response to Bankend
   */
  io.resp.valid               := reg.isResp & reg.waitall & reg.sendall
  io.resp.bits.hnTxnID        := reg.task.hnTxnID

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

  // Get next dataVec
  val cleanHit = reg.isValid & io.clean.valid & io.clean.bits.hnTxnID === reg.task.hnTxnID
  when(io.alloc.fire) {
    next.dataVec  := io.alloc.bits.dataVec
  }.elsewhen(cleanHit) {
    next.dataVec  := VecInit((reg.dataVec.asUInt & ~io.clean.bits.dataVec.asUInt).asBools)
    HAssert(!io.clean.bits.isZero)
    HAssert(reg.isAlloc)
    HAssert(!taskHit)
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
    HAssert.withEn(reg.opBeat === 0.U, !(reg.isRead | reg.isRepl))
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
    HAssert(reg.isSend | reg.isSave | reg.isResp)
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
  val cutIdHit = reg.isValid & io.updHnTxnID.valid & io.updHnTxnID.bits.before === reg.task.hnTxnID
  when(io.alloc.fire) {
    next.task.hnTxnID := io.alloc.bits.hnTxnID
  }.elsewhen(cutIdHit) {
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
      when(io.readToDB.fire & reg.willOpAll) {
        next.state := SEND
      }
    }
    is(READ) {
      when(io.readToDB.fire & reg.willOpAll) {
        next.state := PriorityMux(Seq(
          reg.task.dataOp.send -> SEND,
          reg.task.dataOp.save -> SAVE,
          true.B               -> RESP
        ))
      }
    }
    is(SEND) {
      when(io.readToCHI.fire & reg.willOpAll) {
        next.state := Mux(reg.task.dataOp.save, SAVE, RESP)
      }
    }
    is(SAVE) {
      when(io.readToDS.fire & reg.willOpAll) {
        next.state := RESP
      }
    }
    is(RESP) {
      when(io.resp.fire) {
        next.state := ALLOC
      }
    }
    is(CLEAN) {
      when(true.B) {
        next.state := Mux(reg.isZero, FREE, ALLOC)
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
    val updHnTxnID    = Flipped(Valid(new UpdHnTxnID))
    val reqDBIn       = Flipped(Decoupled(new HnTxnID with HasDataVec))
    val task          = Flipped(Valid(new DataTask)) // broadcast signal
    val resp          = Valid(new HnTxnID)
    val clean         = Flipped(Valid(new HnTxnID with HasDataVec)) // broadcast signal
    // To DS/DB
    val readToDB      = Decoupled(new ReadDS)
    val readToDS      = Decoupled(new ReadDB)
    val readToCHI     = Decoupled(new ReadDB)
    // From/To DBID Pool
    val reqDBOut      = Decoupled(new HnTxnID with HasDataVec)
    val getDBIDVec    = Vec(3, new GetDBID)
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
  HAssert.withEn(!io.task.bits.isZero,      io.task.valid)

  /*
   * Module declaration
   */
  val entries     = Seq.fill(nrDataCM) { Module(new DataCtrlEntry()) }
  val taskFireReg = RegNext(io.task.fire)
  val taskReg     = RegEnable(io.task.bits, io.task.fire)
  val dbgVec      = VecInit(entries.map(_.io.state))
  dontTouch(dbgVec)

  /*
   * Receive and send ReqDB
   */
  val freeDCVec             = VecInit(entries.map(_.io.alloc.ready))
  val hasFreeDC             = freeDCVec.asUInt.orR
  val freeDCID              = WireInit(PriorityEncoder(freeDCVec)); dontTouch(freeDCID)

  // reqDBOut
  io.reqDBOut.valid         := io.reqDBIn.valid & hasFreeDC
  io.reqDBOut.bits          := io.reqDBIn.bits
  // ready
  io.reqDBIn.ready          :=  io.reqDBOut.ready & hasFreeDC

  /*
   * Send alloc and task to entries
   */
  entries.zipWithIndex.foreach { case (e, i) =>
    e.io.dcid               := i.U
    // Alloc
    e.io.alloc.valid        := io.reqDBIn.fire & freeDCID === i.U
    e.io.alloc.bits         := io.reqDBIn.bits
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
  entries.foreach(_.io.updHnTxnID   := io.updHnTxnID)
  entries.foreach(_.io.clean        := io.clean)
  entries.foreach(_.io.dsWriDB      := io.dsWriDB)
  entries.foreach(_.io.txDatFire    := io.txDatFire)
  HAssert.withEn(entries.map(e => e.io.state.valid & e.io.state.bits.hnTxnID === io.updHnTxnID.bits.before).reduce(_ | _), io.updHnTxnID.valid)

  /*
   * Connect IO <- CM
   */
  io.resp      := fastRRArb.validOut(entries.map(_.io.resp))
  io.txDatBits := VecInit(entries.map(_.io.txDatBits))(io.txDataDCID)

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
   * Get DBID from DBIDPool
   */
  // Get dbid
  val hnTxnIDVec  = VecInit(entries.map(_.io.state.bits.hnTxnID))
  io.getDBIDVec(0).hnTxnID  := hnTxnIDVec(io.readToDB.bits.dcid)
  io.getDBIDVec(1).hnTxnID  := hnTxnIDVec(io.readToDS.bits.dcid)
  io.getDBIDVec(2).hnTxnID  := hnTxnIDVec(io.readToCHI.bits.dcid)
  // Set dbid
  io.readToDB.bits.dbid     := io.getDBIDVec(0).dbidVec(io.readToDB.bits.beatNum).bits
  io.readToDS.bits.dbid     := io.getDBIDVec(1).dbidVec(io.readToDS.bits.beatNum).bits
  io.readToCHI.bits.dbid    := io.getDBIDVec(2).dbidVec(io.readToCHI.bits.beatNum).bits
  // HAssert
  HAssert.withEn(io.getDBIDVec(0).dbidVec(io.readToDB.bits.beatNum).valid,  io.readToDB.valid)
  HAssert.withEn(io.getDBIDVec(1).dbidVec(io.readToDS.bits.beatNum).valid,  io.readToDS.valid)
  HAssert.withEn(io.getDBIDVec(2).dbidVec(io.readToCHI.bits.beatNum).valid, io.readToCHI.valid)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}




















