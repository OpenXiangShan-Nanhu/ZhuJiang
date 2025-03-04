package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.frontend.TaskState._

object TaskState {
  val width   = 4
  val FREE    = "b0001".U
  val BESEND  = "b0010".U // -> S0
  val WAIT    = "b0100".U // -> S1
  val SLEEP   = "b1000".U
}

class CtrlEntry(sort: Boolean, nidBits: Int)(implicit p: Parameters) extends DJBundle {
  val nid   = if(sort) Some(UInt(nidBits.W)) else None
  val state = UInt(TaskState.width.W)

  def isFree    = state(0)
  def isBeSend  = state(1)
  def isWait    = state(2)
  def isSleep   = state(3)
}


class TaskBuffer(sort: Boolean, nrEntry: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val chiTask     = Flipped(Decoupled(new ChiTask()))
    val task_s0     = Valid(new ChiTask)
    val req2Pos_s0  = Valid(new DJBundle with HasAddr {
      val isSnp     = Bool()
    })
    val retry_s1    = Input(Bool()) // Reject Task by Block or PoS Full
    val sleep_s1    = Input(Bool()) // Reject Task by PoS Match
    val wakeupVec   = Flipped(Vec(2, Valid(new Addr))) // PoS wakeup someone
  })
  dontTouch(io)

  /*
   * REG and Wire declaration
   */
  val taskIdBits        = log2Ceil(nrEntry)
  val ctrlInit          = WireInit(0.U.asTypeOf(new CtrlEntry(sort, taskIdBits))); ctrlInit.state := FREE
  val ctrlEntrys        = RegInit(VecInit(Seq.fill(nrEntry) { ctrlInit }))
  val taskEntrys        = Reg(Vec(nrEntry, new ChiTask()))
  val newTaskNID        = Wire(UInt(taskIdBits.W))
  val willBeFreeReg_s1  = RegInit(0.U.asTypeOf(Valid(UInt(taskIdBits.W))))
  val toBeFreeAddr_s1   = Wire(Valid(new Addr()))


  /*
   * Receive ChiTask
   */
  val freelist    = ctrlEntrys.map(_.isFree)
  val freeId      = PriorityEncoder(freelist)
  // Store chi task
  io.chiTask.ready     := freelist.reduce(_ | _)
  taskEntrys(freeId)  := io.chiTask.bits

  /*
   * Count NID:
   */
  val taskValidVec  = ctrlEntrys.map(!_.isFree)
  val addrMatchVec  = taskEntrys.map(_.useAddr === io.chiTask.bits.useAddr)
  val matchVec      = taskValidVec.zip(addrMatchVec).map { case(a, b) => a & b }
  val matchNum      = PopCount(matchVec)
  if(sort) {
    newTaskNID      := matchNum - (toBeFreeAddr_s1.valid & toBeFreeAddr_s1.bits.useAddr === io.chiTask.bits.useAddr)
  } else {
    newTaskNID      := 0.U
    HardwareAssertion(!matchVec.reduce(_ | _))
  }

  /*
   * Send Task And PosReq
   */
  val beSendList    = ctrlEntrys.map(_.isBeSend)
  val taskValid     = beSendList.reduce(_ | _)
  val beSendId      = StepRREncoder(beSendList, taskValid)
  // task to block
  io.task_s0.valid  := taskValid
  io.task_s0.bits   := taskEntrys(beSendId)
  // req to pos
  io.req2Pos_s0.valid       := taskValid
  io.req2Pos_s0.bits.isSnp  := taskEntrys(beSendId).isSnp
  io.req2Pos_s0.bits.addr   := taskEntrys(beSendId).addr
  // to be free in next cycle if not get ack
  willBeFreeReg_s1.valid    := taskValid
  willBeFreeReg_s1.bits     := beSendId

  /*
   * To Be Free
   */
  toBeFreeAddr_s1.valid  := willBeFreeReg_s1.valid & !io.retry_s1 & !io.sleep_s1
  toBeFreeAddr_s1.bits   := taskEntrys(willBeFreeReg_s1.bits)

  /*
   * Set Ctrl Entry
   */
  ctrlEntrys.zipWithIndex.foreach {
    case(ctrl, i) =>
      // hit
      val recTaskHit0  = io.chiTask.fire     & freeId                   === i.U & newTaskNID === 0.U
      val recTaskHit1  = io.chiTask.fire     & freeId                   === i.U & newTaskNID =/= 0.U
      val sendTaskHit  = taskValid           & beSendId                 === i.U
      val sleepHit     = io.sleep_s1         & willBeFreeReg_s1.bits    === i.U
      val retryHit     = io.retry_s1         & willBeFreeReg_s1.bits    === i.U
      val wakeupHitVec = io.wakeupVec.map(w => w.valid & w.bits.useAddr === taskEntrys(i).useAddr & ctrl.nid.getOrElse(0.U) === 0.U)
      val wakeupHit    = wakeupHitVec.reduce(_ | _) & ctrl.isSleep
      val toFreeHit    = willBeFreeReg_s1.valid & willBeFreeReg_s1.bits === i.U
      // state:
      // FREE   ---(NID=0)---> BESEND
      // FREE   ---(NID>0)---> SLEEP
      // BESEND -------------> WAIT
      // WAIT   ---(noAck)---> FREE
      // WAIT   ---(retry)---> BESEND
      // WAIT   ---(sleep)---> SLEEP
      // SLEEP  ---(wakeup)--> BESEND
      ctrl.state  := PriorityMux(Seq(
        recTaskHit0 -> BESEND,
        recTaskHit1 -> SLEEP,
        sendTaskHit -> WAIT,
        sleepHit    -> SLEEP,
        retryHit    -> BESEND,
        wakeupHit   -> BESEND,
        toFreeHit   -> FREE,
        true.B      -> ctrl.state,
      ))
      // nid
      val recTaskHit  = io.chiTask.fire & freeId === i.U
      val reduceHit   = toBeFreeAddr_s1.valid & toBeFreeAddr_s1.bits.useAddr === taskEntrys(i).useAddr
      val nextNID     = ctrl.nid.getOrElse(0.U) - reduceHit
      if(sort) {
        ctrl.nid.get  := Mux(recTaskHit, newTaskNID, nextNID)
      }
      // assert Hit
      HardwareAssertion(PopCount(Seq(recTaskHit0, recTaskHit1, sendTaskHit, sleepHit, retryHit, wakeupHit)) <= 1.U,
                                                              desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(PopCount(wakeupHitVec) <= 1.U, ctrl.isSleep,
                                                              desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(ctrl.isFree,     recTaskHit0,  desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(ctrl.isFree,     recTaskHit1,  desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(ctrl.isBeSend,   sendTaskHit,  desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(ctrl.isWait,     sleepHit,     desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(ctrl.isSleep,    wakeupHit,    desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(ctrl.isWait,     retryHit,     desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(ctrl.isWait,     toFreeHit,    desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      // assert Ack
      HardwareAssertion.withEn(toFreeHit,       sleepHit,     desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(toFreeHit,       retryHit,     desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      // assert NID
      HardwareAssertion.withEn(ctrl.nid.getOrElse(0.U) > 0.U, reduceHit, desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}