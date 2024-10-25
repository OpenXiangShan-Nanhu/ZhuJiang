package xijiang.router.base

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xs.utils.ResetRRArbiter
import zhujiang.{ZJModule, ZJParametersKey}
import zhujiang.chi.{Flit, NodeIdBundle}

class SingleChannelTap[T <: Flit](gen: T, channel: String, node: Node)(implicit p: Parameters) extends ZJModule {

  private val timerBits = p(ZJParametersKey).injectRsvdTimerShift
  val io = IO(new Bundle {
    val in = Input(new ChannelBundle(gen))
    val out = Output(new ChannelBundle(gen))
    val inject = Flipped(Decoupled(gen))
    val eject = Decoupled(gen)
    val matchTag = Input(UInt(niw.W))
    val tapIdx = Input(UInt(nodeAidBits.W))
  })
  private val local = !node.csnNode
  private val c2c = node.nodeType == NodeType.C
  private val modName = if(local) {
    s"SingleChannelTapLocal$channel"
  } else if(c2c) {
    s"SingleChannelTapC2c$channel"
  } else {
    s"SingleChannelTapCsn$channel"
  }
  override val desiredName = modName
  private val normalShift = 0
  private val injectRsvdShift = 1
  private val waitSlotShift = 2

  private val s_normal = (1 << normalShift).U(3.W)
  private val s_inject_reserved = (1 << injectRsvdShift).U(3.W)
  private val s_wait_slot = (1 << waitSlotShift).U(3.W)
  private val state = RegInit(s_normal)
  private val counter = RegInit(0.U(timerBits.W))
  private val flitNext = Wire(Valid(gen))
  private val rsvdNext = Wire(Valid(UInt(niw.W)))
  private val injectFire = io.inject.fire
  private val ejectFire = io.eject.fire
  private val rsvdMarkVal = Cat(io.matchTag(niw - 1, nodeAidBits), io.tapIdx)
  private val meetRsvdSlot = io.in.rsvd.bits === rsvdMarkVal

  when(injectFire) {
    counter := 0.U
  }.elsewhen(io.inject.valid && !io.inject.ready && state(normalShift) && !counter(timerBits - 1)) {
    counter := counter + 1.U
  }

  switch(state) {
    is(s_normal) {
      state := Mux(counter(timerBits - 1), s_inject_reserved, s_normal)
    }
    is(s_inject_reserved) {
      state := Mux(injectFire, s_normal, Mux(io.in.rsvd.valid, s_inject_reserved, s_wait_slot))
    }
    is(s_wait_slot) {
      state := Mux(injectFire, s_normal, s_wait_slot)
    }
  }
  when(io.in.rsvd.valid && meetRsvdSlot) {
    assert(state(waitSlotShift), "Unexpected reserved slot!")
  }

  private val emptySlot = Mux(io.in.flit.valid, ejectFire, true.B)
  private val availableSlot = Mux(io.in.rsvd.valid, meetRsvdSlot, !state(waitSlotShift))
  dontTouch(emptySlot)
  dontTouch(availableSlot)
  io.inject.ready := emptySlot && availableSlot

  flitNext.valid := injectFire || io.in.flit.valid && !ejectFire
  flitNext.bits := Mux(injectFire, io.inject.bits, io.in.flit.bits)

  rsvdNext.valid := (state(injectRsvdShift) || io.in.rsvd.valid) && !injectFire
  rsvdNext.bits := Mux(state(injectRsvdShift) && !io.in.rsvd.valid, rsvdMarkVal, io.in.rsvd.bits)

  io.out.flit := Pipe(flitNext)
  io.out.rsvd := Pipe(rsvdNext)

  private val matcher = io.matchTag.asTypeOf(new NodeIdBundle)
  if(local) {
    io.eject.valid := io.in.flit.bits.tgt.asTypeOf(new NodeIdBundle).router === matcher.router && io.in.flit.valid
  } else if(c2c) {
    io.eject.valid := io.in.flit.bits.tgt.asTypeOf(new NodeIdBundle).chip === matcher.chip && io.in.flit.valid
  } else {
    io.eject.valid := io.in.flit.bits.tgt.asTypeOf(new NodeIdBundle) === matcher && io.in.flit.valid
  }
  io.eject.bits := io.in.flit.bits
}

class ChannelTap[T <: Flit](
  val gen: T, channel: String,
  ejectBuf: Int, node: Node,
)(implicit p: Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val rx = Input(Vec(2, new ChannelBundle(gen)))
    val tx = Output(Vec(2, new ChannelBundle(gen)))
    val inject = Flipped(Decoupled(gen))
    val eject = Decoupled(gen)
    val matchTag = Input(UInt(niw.W))
    val injectTapSelOH = Input(Vec(2, Bool()))
  })
  private val local = !node.csnNode
  private val c2c = node.nodeType == NodeType.C
  override val desiredName = if(local) {
    s"ChannelTapLocal$channel"
  } else if(c2c) {
    s"ChannelTapC2c$channel"
  } else {
    s"ChannelTapCsn$channel"
  }
  private val taps = Seq.fill(2)(Module(new SingleChannelTap(gen, channel, node)))
  private val ejectArb = Module(new ResetRRArbiter(gen, 2))
  for(idx <- taps.indices) {
    taps(idx).io.in := io.rx(idx)
    io.tx(idx) := taps(idx).io.out
    taps(idx).io.inject.valid := io.inject.valid && io.injectTapSelOH(idx)
    taps(idx).io.inject.bits := io.inject.bits
    taps(idx).io.matchTag := io.matchTag
    taps(idx).io.tapIdx := idx.U
    if(ejectBuf > 0) {
      val ejectBuffer = Module(new EjectBuffer(gen, ejectBuf, channel))
      ejectBuffer.io.enq <> taps(idx).io.eject
      ejectArb.io.in(idx) <> ejectBuffer.io.deq
    } else {
      val ejectBuffer = Module(new Queue(gen, 2))
      ejectBuffer.io.enq <> taps(idx).io.eject
      ejectArb.io.in(idx) <> ejectBuffer.io.deq
    }
  }
  io.inject.ready := Mux1H(io.injectTapSelOH, taps.map(_.io.inject.ready))
  io.eject <> ejectArb.io.out
}