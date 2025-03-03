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
  val BESEND  = "b0010".U
  val WAIT    = "b0100".U
  val SLEEP   = "b1000".U
}

class CtrlEntry(sort: Boolean, index: Int, nidBits: Int)(implicit p: Parameters) extends DJBundle {
  val idx   = index
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
    val chiTask = Flipped(Decoupled(new ChiTask()))
    val task    = Valid(new ChiTask with HasTaskBufId)
    val taskAck = Input(Bool()) // Reject Task by Block
    val posAck  = Input(Bool()) // Reject Task by PoS
    val req2Pos = Valid(new Addr with HasTaskBufId)
    val wakeup  = Flipped(Valid(UInt(addrBits.W))) // PoS wakeup someone
  })

  /*
   * REG and Wire declaration
   */
  val taskIdBits  = log2Ceil(nrEntry)
  val ctrlEntrys  = RegInit(MixedVecInit(Seq.tabulate(nrEntry) { i =>
    val entry     = 0.U.asTypeOf(new CtrlEntry(sort, i, taskIdBits))
    entry.state   := FREE
    entry
  }))
  val taskEntrys    = Reg(Vec(nrEntry, new ChiTask()))
  val willBeFreeReg = RegInit(0.U.asTypeOf(Valid(UInt(taskIdBits.W))))
  val toBeFreeAddr  = Wire(Valid(new Addr()))
  val newTaskNID    = Wire(UInt(taskIdBits.W))

  /*
   * Receive ChiTask
   */
  val freelist    = ctrlEntrys.map(_.isFree)
  val freeId      = PriorityEncoder(freelist)
  // Store chi task
  io.chiTask.ready   := freelist.reduce(_ | _)
  taskEntrys(freeId) := io.chiTask.bits

  /*
   * Count NID:
   */
  val taskValidVec  = ctrlEntrys.map(!_.isFree)
  val addrMatchVec  = taskEntrys.map(_.useAddr === io.chiTask.bits.useAddr)
  val matchVec      = taskValidVec.zip(addrMatchVec).map { case(a, b) => a & b }
  val matchNum      = PopCount(matchVec)
  if(sort) {
    newTaskNID      := matchNum - (toBeFreeAddr.valid & toBeFreeAddr.bits.useAddr === io.chiTask.bits.useAddr)
  } else {
    newTaskNID      := 0.U
    HardwareAssertion(!matchVec.reduce(_ | _))
  }

  /*
   * Send Task And PosReq
   */
  val beSendList  = ctrlEntrys.map(_.isBeSend)
  val taskValid   = beSendList.reduce(_ | _)
  val beSendId    = StepRREncoder(beSendList, taskValid)
  // task to block
  io.task.valid             := taskValid
  io.task.bits              := taskEntrys(beSendId).asUInt.asTypeOf(io.task.bits)
  io.task.bits.taskBufId    := beSendId
  // req to pos
  io.req2Pos.valid          := taskValid
  io.req2Pos.bits.addr      := taskEntrys(beSendId).addr
  io.req2Pos.bits.taskBufId := beSendId
  // to be free in next cycle if not get ack
  willBeFreeReg.valid       := taskValid
  willBeFreeReg.bits        := beSendId

  /*
   * To Be Free
   */
  toBeFreeAddr.valid  := willBeFreeReg.valid & !io.posAck & !io.taskAck
  toBeFreeAddr.bits   := taskEntrys(willBeFreeReg.bits)

  /*
   * Set Ctrl Entry
   */
  ctrlEntrys.foreach {
    case ctrl =>
      // hit
      val recTaskHit0 = io.chiTask.fire     & freeId              === ctrl.idx.U & newTaskNID === 0.U
      val recTaskHit1 = io.chiTask.fire     & freeId              === ctrl.idx.U & newTaskNID =/= 0.U
      val sendTaskHit = taskValid           & beSendId            === ctrl.idx.U
      val sleepHit    = io.posAck           & willBeFreeReg.bits  === ctrl.idx.U
      val toBeSendHit = io.taskAck          & willBeFreeReg.bits  === ctrl.idx.U
      val wakeupHit   = io.wakeup.valid     & io.wakeup.bits      === taskEntrys(ctrl.idx).useAddr & ctrl.nid.getOrElse(0.U) === 0.U
      val toFreeHit   = willBeFreeReg.valid & willBeFreeReg.bits  === ctrl.idx.U
      // state:
      // FREE   ---(NID=0)---> BESEND
      // FREE   ---(NID>0)---> SLEEP
      // BESEND -------------> WAIT
      // WAIT   ---(noAck)---> FREE
      // WAIT   ---(taskAck)-> BESEND
      // WAIT   ---(posAck)--> SLEEP
      // SLEEP  ---(wakeup)--> BESEND
      ctrl.state  := PriorityMux(Seq(
        recTaskHit0 -> BESEND,
        recTaskHit1 -> SLEEP,
        sendTaskHit -> WAIT,
        sleepHit    -> SLEEP,
        toBeSendHit -> BESEND,
        wakeupHit   -> BESEND,
        toFreeHit   -> FREE,
        true.B      -> ctrl.state,
      ))
      // nid
      val recTaskHit  = io.chiTask.fire & freeId === ctrl.idx.U
      val reduceHit   = toBeFreeAddr.valid & toBeFreeAddr.bits.useAddr === taskEntrys(ctrl.idx).useAddr
      val nextNID     = ctrl.nid.getOrElse(0.U) - reduceHit
      if(sort) {
        ctrl.nid.get  := Mux(recTaskHit, newTaskNID, nextNID)
      }
      // assert Hit
      HardwareAssertion(PopCount(Seq(recTaskHit0, recTaskHit1, sendTaskHit, sleepHit, toBeSendHit, wakeupHit)) <= 1.U,
                                                              desc = cf"Task Buffer Index[${ctrl.idx}]")
      HardwareAssertion.withEn(ctrl.isFree,     recTaskHit0,  desc = cf"Task Buffer Index[${ctrl.idx}]")
      HardwareAssertion.withEn(ctrl.isFree,     recTaskHit1,  desc = cf"Task Buffer Index[${ctrl.idx}]")
      HardwareAssertion.withEn(ctrl.isBeSend,   sendTaskHit,  desc = cf"Task Buffer Index[${ctrl.idx}]")
      HardwareAssertion.withEn(ctrl.isWait,     sleepHit,     desc = cf"Task Buffer Index[${ctrl.idx}]")
      HardwareAssertion.withEn(ctrl.isSleep,    wakeupHit,    desc = cf"Task Buffer Index[${ctrl.idx}]")
      HardwareAssertion.withEn(ctrl.isWait,     toBeSendHit,  desc = cf"Task Buffer Index[${ctrl.idx}]")
      HardwareAssertion.withEn(ctrl.isWait,     toFreeHit,    desc = cf"Task Buffer Index[${ctrl.idx}]")
      // assert Ack
      HardwareAssertion.withEn(toFreeHit,       sleepHit,     desc = cf"Task Buffer Index[${ctrl.idx}]")
      HardwareAssertion.withEn(toFreeHit,       toBeSendHit,  desc = cf"Task Buffer Index[${ctrl.idx}]")
      // assert NID
      HardwareAssertion.withEn(ctrl.nid.getOrElse(0.U) > 0.U, reduceHit, desc = cf"Task Buffer Index[${ctrl.idx}]")
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}