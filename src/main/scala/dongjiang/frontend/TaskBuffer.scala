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
      val value     = UInt(TASKSTATE.width.W)
      val release   = Bool()
    })
  })

  /*
   * Reg and Wire declaration
   */
  val taskReg = RegInit((new TaskState with HasPackChi with HasAddr).Lit(_.state -> FREE))
  val nidReg  = if(sort) Some(Reg(UInt(nidBits.W))) else None

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
  io.state.valid   := taskReg.isValid
  io.state.release := RegNext(taskReg.isValid) & taskReg.isFree
  io.state.addr    := taskReg.addr
  io.state.value   := taskReg.state
  HardwareAssertion(!(io.state.valid & io.state.release))

  /*
   * State Transfer:
   *                          |---------------------(wakeUpHit)--------------------------------|
   *                          v                                                                |
   * FREE --(taskIn.fire)--> SEND ---(taskOut_s0.fire)--> WAIT ---->|-->(sleep_s1)--> SLEEP -->|
   *  ^                       ^                                     |
   *  |                       |--(wakeUpHit | (retry_s1 & !sleep))--|
   *  |                                                             |
   *  |------------------(!retry_s1 & !sleep_s1)--------------------|
   *
   */
  val wakeUpHit = io.wakeup.valid && taskReg.Addr.useAddr === io.wakeup.bits.Addr.useAddr
  switch(taskReg.state) {
    is(FREE) {
      when(io.chiTaskIn.fire) { taskReg.state := SEND }
    }
    is(SEND) {
      when(io.chiTask_s0.fire) { taskReg.state := WAIT }
    }
    is(WAIT) {
      when(wakeUpHit)        { taskReg.state := SEND  }
      .elsewhen(io.sleep_s1) { taskReg.state := SLEEP }
      .elsewhen(io.retry_s1) { taskReg.state := SEND  }
      .otherwise             { taskReg.state := FREE  }
    }
    is(SLEEP) {
      when(wakeUpHit) { taskReg.state := SEND }
    }
  }
  // assert
  HAssert.withEn(nidReg.getOrElse(0.U) === 0.U, taskReg.isSleep & wakeUpHit)
  // assert timeout
  HAssert.checkTimeout(taskReg.isFree, TIMEOUT_TASKBUF, desc = cf"State[${taskReg.state}]")
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
    //  system is working
    val working     = Output(Bool())
  })


  /*
   * Module declaration
   */
  val entries  = Seq.fill(nrEntries) { Module(new TaskEntry(log2Ceil(nrEntries), sort)) }
  val debugVec = WireInit(VecInit(entries.map(_.io.state)))
  dontTouch(debugVec)

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
      self.io.initNid.get  := PopCount(Cat(entries.map(other => other.io.state.valid & other.io.state.Addr.useAddr === io.chiTaskIn.bits.Addr.useAddr)))
      self.io.othRel.get   := Cat(entries.map(other => other.io.state.release & other.io.state.Addr.useAddr === self.io.state.Addr.useAddr)).orR
    }
    HardwareAssertion(PopCount(Cat(entries.map(e => e.io.state.release))) <= 1.U)
  }

  /*
   * Has DataBuffer valid
   */
  io.working := Cat(entries.map(_.io.state.valid)).orR

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}