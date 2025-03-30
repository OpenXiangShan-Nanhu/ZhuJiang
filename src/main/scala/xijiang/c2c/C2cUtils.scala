package xijiang.c2c
import chisel3._
import chisel3.util._
import xs.utils.queue.{CircularQueuePtr, HasCircularQueuePtrHelper, MimoQueue}

case class C2cParams(
  reqRxSize:Int = 16,
  rspRxSize:Int = 16,
  datRxSize:Int = 16,
  snpRxSize:Int = 16
)

object C2cUtils {
  val reqId = 0
  val rspId = 1
  val datId = 2
  val snpId = 3
  val nrChn = 4
  val payloadBits = 256
  val grantBits = 4
  val nrSlotsPerFrame = 3
  val slotBits = (payloadBits - 4 * grantBits) / nrSlotsPerFrame
  val slotDataBits = slotBits - 3
  val grantDataBits = grantBits - 1
  val nrMaxToken = 64
  require((nrSlotsPerFrame * slotBits + 4 * grantBits) == payloadBits)

  def getSlotsAndDeq(flitBits: Int): (Int, Int) = {
    val slots = (flitBits + slotDataBits - 1) / slotDataBits
    val deqs = slots.min(nrSlotsPerFrame)
    (slots, deqs)
  }

  def chnIdToStr(chnId:Int):String = {
    chnId match {
      case C2cUtils.reqId => "Req"
      case C2cUtils.rspId => "Rsp"
      case C2cUtils.datId => "Dat"
      case C2cUtils.snpId => "Snp"
      case _ => "Unknown"
    }
  }
}

class C2cSlot extends Bundle {
  val data = UInt(C2cUtils.slotDataBits.W)
  val chnId = UInt(2.W)
}

class C2cPayload extends Bundle {
  val slots = Vec(C2cUtils.nrSlotsPerFrame, Valid(new C2cSlot))
  val grants = Vec(C2cUtils.nrChn, Valid(UInt(C2cUtils.grantDataBits.W)))
}

class TxQueue[T <: Data](gen: T, val chnId:Int) extends Module with HasCircularQueuePtrHelper {
  private val (nrSlots, nrDeq) = C2cUtils.getSlotsAndDeq(gen.getWidth)
  private class SlotPtr extends CircularQueuePtr[SlotPtr](nrSlots)
  private object SlotPtr {
    def apply(f:Boolean, v:Int):SlotPtr = {
      val ptr = Wire(new SlotPtr)
      ptr.flag := f.B
      ptr.value := v.U
      ptr
    }
  }
  private val suffix = C2cUtils.chnIdToStr(chnId)
  override val desiredName = s"TxQueue$suffix"

  val io = IO(new Bundle {
    val enq = Flipped(Decoupled(gen))
    val deq = Vec(nrDeq, Decoupled(new C2cSlot))
    val grant = Input(Valid(UInt(C2cUtils.grantDataBits.W)))
  })
  private val flitQueue = Module(new Queue(UInt(gen.getWidth.W), entries = 2))
  private val slotQueue = Module(new MimoQueue(new C2cSlot, nrDeq, nrDeq, 2 * nrDeq, true, false))
  private val slotPtrVec = RegInit(VecInit(Seq.tabulate(nrDeq)(i => SlotPtr(f = false, v = i))))
  private val slotPtrVecNext = WireInit(slotPtrVec)
  private val tokens = RegInit(0.U(log2Ceil(C2cUtils.nrMaxToken + 1).W))
  private val padding = 0.U((nrSlots * C2cUtils.slotDataBits - gen.getWidth).W)
  private val flitVec = Cat(padding, flitQueue.io.deq.bits).asTypeOf(Vec(nrSlots, UInt(C2cUtils.slotDataBits.W)))

  flitQueue.io.enq <> io.enq
  io.deq <> slotQueue.io.deq
  slotPtrVec := slotPtrVecNext
  if(slotQueue.io.enq.length == 1) {
    slotQueue.io.enq.head.valid := flitQueue.io.deq.valid && tokens.orR
    slotQueue.io.enq.head.bits.chnId := chnId.U
    slotQueue.io.enq.head.bits.data := flitQueue.io.deq.bits
    flitQueue.io.deq.ready := slotQueue.io.enq.head.ready && tokens.orR
  } else {
    for(i <- slotQueue.io.enq.indices) {
      slotQueue.io.enq(i).valid := flitQueue.io.deq.valid && tokens.orR && slotPtrVec(i).flag === slotPtrVec.head.flag
      slotQueue.io.enq(i).bits.chnId := chnId.U
      slotQueue.io.enq(i).bits.data := flitVec(slotPtrVec(i).value)
    }
    flitQueue.io.deq.ready := slotQueue.io.enq.head.ready && tokens.orR && slotPtrVecNext.head.value === 0.U
  }

  private val slotQueueEnqFire = slotQueue.io.enq.head.fire
  private val slotQueueEnqNum = PriorityEncoder(slotQueue.io.enq.map(e => !e.fire) :+ true.B)
  when(slotQueueEnqFire) {
    slotPtrVecNext.zip(slotPtrVec).foreach(elm => elm._1 := elm._2 + slotQueueEnqNum)
  }

  private val grantTokenValidReg = RegNext(io.grant.valid, false.B)
  private val grantTokenNumReg = RegEnable(1.U << io.grant.bits, io.grant.valid)
  private val fireTokenNum = Mux(flitQueue.io.deq.fire, 1.U, 0.U)
  when(flitQueue.io.deq.fire) {
    assert(tokens.orR)
  }
  when(flitQueue.io.deq.fire || grantTokenValidReg) {
    tokens := tokens - fireTokenNum + Mux(grantTokenValidReg, grantTokenNumReg, 0.U)
  }
}

class RxQueue[T <: Data](gen: T, val chnId:Int, depth:Int) extends Module {
  private val (nrSlots, _) = C2cUtils.getSlotsAndDeq(gen.getWidth)
  require(depth <= C2cUtils.nrMaxToken)
  private val suffix = C2cUtils.chnIdToStr(chnId)
  override val desiredName = s"RxQueue$suffix"
  val io = IO(new Bundle {
    val enq = Input(Valid(new C2cPayload))
    val deq = Decoupled(gen)
    val grant = Decoupled(UInt(C2cUtils.grantDataBits.W))
  })
  private val slotQueueSize = if(nrSlots <= C2cUtils.nrSlotsPerFrame) {
    nrSlots * 2
  } else {
    nrSlots - 1 + C2cUtils.nrSlotsPerFrame + C2cUtils.nrSlotsPerFrame
  }
  private val flitQueue = Module(new Queue(gen, entries = depth, useSyncReadMem = true))
  private val slotQueue = Module(new MimoQueue(UInt(C2cUtils.slotDataBits.W), C2cUtils.nrSlotsPerFrame, nrSlots, slotQueueSize, false, false))
  private val tokens = RegInit(depth.U(log2Ceil(C2cUtils.nrMaxToken + 1).W))

  for(i <- 0 until C2cUtils.nrSlotsPerFrame) {
    val slot = io.enq.bits.slots(i)
    slotQueue.io.enq(i).valid := io.enq.valid && slot.valid && slot.bits.chnId === chnId.U
    slotQueue.io.enq(i).bits := slot.bits.data
    when(slotQueue.io.enq(i).valid) {
      assert(slotQueue.io.enq(i).ready)
    }
  }

  flitQueue.io.enq.valid := Cat(slotQueue.io.deq.map(_.valid)).andR
  flitQueue.io.enq.bits :=  Cat(slotQueue.io.deq.map(_.bits).reverse)
  slotQueue.io.deq.foreach(_.ready := flitQueue.io.enq.valid)
  when(flitQueue.io.enq.valid) {
    assert(flitQueue.io.enq.ready)
  }
  io.deq <> flitQueue.io.deq

  private val grantToken = WireInit(0.U(log2Ceil(C2cUtils.nrMaxToken + 1).W))
  io.grant.valid := tokens.orR
  io.grant.bits := 0.U
  for(i <- 0 until tokens.getWidth) {
    when(tokens(i)) {
      grantToken := io.grant.fire << i
      io.grant.bits := i.U
    }
  }
  assert(tokens <= depth.U)
  assert(grantToken <= tokens)
  private val tokenReleased = Mux(flitQueue.io.deq.fire, 1.U, 0.U)
  when(io.grant.fire || flitQueue.io.deq.fire) {
    tokens := tokens - grantToken + tokenReleased
  }
}