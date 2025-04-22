package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.frontend.TASKSTATE._
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- Task Buffer State ----------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
object TASKSTATE {
  val width   = 4
  val FREE    = "b0001".U // 1
  val SEND    = "b0010".U // 2 // -> S0
  val WAIT    = "b0100".U // 4 // -> S1
  val SLEEP   = "b1000".U // 8
}

class TaskState(implicit p: Parameters) extends DJBundle {
  val state = UInt(TASKSTATE.width.W)

  def isFree    = state(0)
  def isValid   = !isFree
  def isSend    = state(1)
  def isWait    = state(2)
  def isSleep   = state(3)
}

// ----------------------------------------------------------------------------------------------------- //
// ------------------------------------------ Task Buffer Entry ---------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
// TODO: can receive device
class TaskEntry(nidBits: Int, sort: Boolean)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // task
    val chiTaskIn   = Flipped(Decoupled(new PackChi with HasAddr))
    val chiTask_s0  = Decoupled(new PackChi with HasAddr)
    // ctrl
    val retry_s1    = Input(Bool()) // Reject Task by Block or PoS Full
    val sleep_s1    = Input(Bool()) // Reject Task by PoS Match
    val wakeup      = Flipped(Valid(new Addr)) // PoS wakeup someone
    // nid
    val initNid     = if(sort) Some(Input(UInt(nidBits.W))) else None
    val othRel      = if(sort) Some(Input(Bool())) else None // Other Release
    val state       = Output(new DJBundle with HasAddr {
      val valid     = Bool()
      val release   = Bool()
    })
    // multiple cores are actively making requests
    val multicore   = Bool()
  })

  /*
   * Reg and Wire declaration
   */
  val taskReg = RegInit((new TaskState with HasPackChi with HasAddr).Lit(_.state -> FREE))
  val nidReg  = if(sort) Some(Reg(UInt(nidBits.W))) else None

  /*
   * Multiple cores are actively making requests
   */
  io.multicore := taskReg.isSleep

  /*
   * Hit Message
   */
  val recTaskHit  =  taskReg.isFree  & io.chiTaskIn.fire
  val sendTaskHit =  taskReg.isSend  & io.chiTask_s0.fire
  val wakeupHit   = (taskReg.isSleep | taskReg.isWait) & io.wakeup.valid & io.wakeup.bits.Addr.useAddr === taskReg.Addr.useAddr
  val sleepHit    =  taskReg.isWait  & io.sleep_s1
  val retryHit    =  taskReg.isWait  & io.retry_s1
  val toFreeHit   =  taskReg.isWait

  /*
   * Receive ChiTask
   */
  when(io.chiTaskIn.fire) {
    taskReg.chi  := io.chiTaskIn.bits.chi
    taskReg.addr := io.chiTaskIn.bits.addr
  }
  io.chiTaskIn.ready  := taskReg.isFree

  /*
   * Send Task
   */
  io.chiTask_s0.valid := taskReg.isSend & nidReg.getOrElse(0.U) === 0.U
  io.chiTask_s0.bits  := taskReg

  /*
   * Sorting Task By NID
   */
  if(sort) {
    // Set NID
    when(io.chiTaskIn.fire) {
      nidReg.get := io.initNid.get
    }.elsewhen(taskReg.isValid) {
      nidReg.get := nidReg.get - io.othRel.get
      HardwareAssertion.withEn(nidReg.get > 0.U, io.othRel.get)
    }
  }
  // Output State
  io.state.valid := taskReg.isValid
  io.state.release := RegNext(taskReg.isValid) & taskReg.isFree
  io.state.addr := taskReg.addr
  HardwareAssertion(!(io.state.valid & io.state.release))

  /*
   * State Transfer:
   *
   * --------------------------------------------
   * | Priority | Init   | Condition   | Next   |
   * --------------------------------------------
   * |    0     | FREE   | recTaskHit  | SEND   |
   * |    1     | SEND   | sendTaskHit | WAIT   |
   * |    2     | SLEEP  | wakeupHit   | SEND   |
   * |    2     | WAIT   | wakeupHit   | SEND   |
   * |    3     | WAIT   | sleepHit    | SLEEP  |
   * |    4     | WAIT   | retryHit    | SEND   |
   * |    5     | WAIT   | toFreeHit   | FREE   |
   * --------------------------------------------
   */
  taskReg.state := PriorityMux(Seq(
    recTaskHit  -> SEND,
    sendTaskHit -> WAIT,
    wakeupHit   -> SEND,
    sleepHit    -> SLEEP,
    retryHit    -> SEND,
    toFreeHit   -> FREE,
    true.B      -> taskReg.state,
  ))

  // assert hit num
  HardwareAssertion(PopCount(Seq(recTaskHit, sendTaskHit, wakeupHit | sleepHit | retryHit | toFreeHit)) <= 1.U,
                                                                          desc = cf"State[${taskReg.state}]")
  // assert timeout
  HardwareAssertion.checkTimeout(taskReg.isFree, TIMEOUT_TASKBUF,         desc = cf"State[${taskReg.state}]")
}


// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Task Buffer -------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class TaskBuffer(nrEntries: Int, sort: Boolean)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // task
    val chiTaskIn   = Flipped(Decoupled(new PackChi with HasAddr))
    val chiTask_s0  = Valid(new PackChi with HasAddr)
    val req2Pos_s0  = Valid(new Addr with HasChiChannel)
    // ctrl
    val retry_s1    = Input(Bool()) // Reject Task by Block or PoS Full
    val sleep_s1    = Input(Bool()) // Reject Task by PoS Match
    val wakeup      = Flipped(Valid(new Addr)) // PoS wakeup someone
    // multiple cores are actively making requests
    val multicore   = Bool()
    //  system is working
    val working     = Output(Bool())
  })


  /*
   * Module declaration
   */
  val entries = Seq.fill(nrEntries) { Module(new TaskEntry(log2Ceil(nrEntries), sort)) }

  /*
   * Receive Chi Task
   */
  Alloc(entries.map(_.io.chiTaskIn), io.chiTaskIn)

  /*
   * Send Task & Req
   */
  io.chiTask_s0               := fastRRArb.validOut(entries.map(_.io.chiTask_s0))
  io.req2Pos_s0.valid         := io.chiTask_s0.valid
  io.req2Pos_s0.bits.addr     := io.chiTask_s0.bits.addr
  io.req2Pos_s0.bits.channel  := io.chiTask_s0.bits.chi.channel

  /*
   * Connect Ctrl Signals
   */
  entries.foreach { case e =>
    e.io.retry_s1 := io.retry_s1
    e.io.sleep_s1 := io.sleep_s1
    e.io.wakeup   := io.wakeup
  }

  /*
   * Sorting Task By NID
   */
  if(sort) {
    entries.foreach { self =>
      self.io.initNid.get  := PopCount(entries.map(other => other.io.state.valid & other.io.state.Addr.useAddr === io.chiTaskIn.bits.Addr.useAddr))
      self.io.othRel.get   := entries.map(other => other.io.state.release & other.io.state.Addr.useAddr === self.io.state.Addr.useAddr).reduce(_ | _)
    }
    HardwareAssertion(PopCount(entries.map(e => e.io.state.release)) <= 1.U)
  }

  /*
   * Multiple cores are actively making requests
   */
  io.multicore := entries.map(_.io.multicore).reduce(_ | _)

  /*
   * Has DataBuffer valid
   */
  io.working := entries.map(_.io.state.valid).reduce(_ | _)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}