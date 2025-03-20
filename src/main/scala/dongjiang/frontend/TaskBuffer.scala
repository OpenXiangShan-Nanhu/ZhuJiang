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
  val FREE    = "b0001".U // 1
  val BESEND  = "b0010".U // 2 // -> S0
  val WAIT    = "b0100".U // 4 // -> S1
  val SLEEP   = "b1000".U // 8
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
    val chiTaskIn   = Flipped(Decoupled(new ChiTask with HasAddr))
    val chiTask_s0  = Valid(new ChiTask with HasAddr)
    val req2Pos_s0  = Valid(new DJBundle with HasAddr {
      val isSnp     = Bool()
    })
    val retry_s1    = Input(Bool()) // Reject Task by Block or PoS Full
    val sleep_s1    = Input(Bool()) // Reject Task by PoS Match
    val wakeup      = Flipped(Valid(new Addr)) // PoS wakeup someone
  })
  dontTouch(io)

  /*
   * Reg and Wire declaration
   */
  val taskIdBits  = log2Ceil(nrEntry)
  val ctrlInit    = WireInit(0.U.asTypeOf(new CtrlEntry(sort, taskIdBits))); ctrlInit.state := FREE
  val ctrlEntrys  = RegInit(VecInit(Seq.fill(nrEntry) { ctrlInit }))
  val taskEntrys  = Reg(Vec(nrEntry, new ChiTask with HasAddr))
  val newTaskNID  = Wire(UInt(taskIdBits.W))
  // S1: toBeFree
  val toBeFreeReg_s1        = RegInit(false.B)
  val toBeFreeIdReg_s1      = Reg(UInt(taskIdBits.W))
  val toBeFreeUseAddrReg_s1 = Reg(UInt(useAddrBits.W))


  /*
   * Receive ChiTask
   */
  val freeVec     = ctrlEntrys.map(_.isFree)
  val freeId      = PriorityEncoder(freeVec)
  val freeNum     = PopCount(freeVec)
  // Store chi task, reserve an extra entry for the snp task
  io.chiTaskIn.ready  := Mux(io.chiTaskIn.bits.isSnp, freeVec.reduce(_ | _), freeNum > 1.U)
  taskEntrys(freeId)  := io.chiTaskIn.bits

  /*
   * Count NID
   */
  val taskValidVec  = ctrlEntrys.map(!_.isFree)
  val addrMatchVec  = taskEntrys.map(_.useAddr === io.chiTaskIn.bits.useAddr)
  val matchVec      = taskValidVec.zip(addrMatchVec).map { case(a, b) => a & b }
  val matchNum      = PopCount(matchVec)
  if(sort) {
    newTaskNID      := matchNum - (toBeFreeReg_s1 & toBeFreeUseAddrReg_s1 === io.chiTaskIn.bits.useAddr)
  } else {
    newTaskNID      := 0.U
    HardwareAssertion(!matchVec.reduce(_ | _))
  }

  /*
   * Send Task And PosReq
   */
  val beSendVec     = ctrlEntrys.map(_.isBeSend)
  val taskValid     = beSendVec.reduce(_ | _)
  val beSendId      = StepRREncoder(beSendVec, taskValid)
  // task to block
  io.chiTask_s0.valid  := taskValid
  io.chiTask_s0.bits   := taskEntrys(beSendId)
  // req to pos
  io.req2Pos_s0.valid       := taskValid
  io.req2Pos_s0.bits.isSnp  := taskEntrys(beSendId).isSnp
  io.req2Pos_s0.bits.addr   := taskEntrys(beSendId).addr
  // to be free in next cycle if not get ack
  toBeFreeReg_s1            := taskValid
  toBeFreeIdReg_s1          := beSendId
  toBeFreeUseAddrReg_s1     := taskEntrys(beSendId).useAddr

  /*
   * Set Ctrl Entry
   */
  ctrlEntrys.zipWithIndex.foreach {
    case(ctrl, i) =>
      // hit
      val recTaskHit0  = io.chiTaskIn.fire  & freeId                  === i.U & newTaskNID === 0.U
      val recTaskHit1  = io.chiTaskIn.fire  & freeId                  === i.U & newTaskNID =/= 0.U
      val sendTaskHit  = taskValid          & beSendId                === i.U
      val sleepHit     = io.sleep_s1        & toBeFreeIdReg_s1        === i.U
      val retryHit     = io.retry_s1        & toBeFreeIdReg_s1        === i.U
      val wakeupHit    = io.wakeup.valid    & io.wakeup.bits.useAddr  === taskEntrys(i).useAddr & ctrl.nid.getOrElse(0.U) === 0.U
      val toFreeHit    = toBeFreeReg_s1     & toBeFreeIdReg_s1        === i.U
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
      val recTaskHit  = io.chiTaskIn.fire & freeId === i.U
      val reduceHit   = !ctrl.isFree & toBeFreeReg_s1 & toBeFreeIdReg_s1 =/= i.U & toBeFreeUseAddrReg_s1 === taskEntrys(i).useAddr
      val nextNID     = ctrl.nid.getOrElse(0.U) - reduceHit
      if(sort) {
        ctrl.nid.get  := Mux(recTaskHit, newTaskNID, nextNID)
      }
      // assert Hit
      HardwareAssertion(PopCount(Seq(recTaskHit0, recTaskHit1, sendTaskHit, sleepHit, retryHit, wakeupHit)) <= 1.U,
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
      HardwareAssertion.withEn(ctrl.nid.getOrElse(0.U) === 0.U, recTaskHit | sendTaskHit | sleepHit | retryHit | toFreeHit,
                                                              desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.withEn(ctrl.nid.getOrElse(0.U) > 0.U, reduceHit,
                                                              desc = cf"Task Buffer Index[${i}] State[${ctrl.state}]")
      HardwareAssertion.placePipe(Int.MaxValue-3)
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}