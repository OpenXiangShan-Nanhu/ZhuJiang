package dongjiang.utils

import chisel3._
import chisel3.util._
import xs.utils.arb._

trait HasQoS { this: Bundle => val qos = UInt(4.W) }

class ArbiterGenerator[T <: Bundle](gen:T, size:Int, rr:Boolean, qos: Boolean) extends Module {
  val io    = IO(new Bundle {
    val in  = Vec(size, Flipped(Decoupled(gen)))
    val out = Decoupled(gen)
  })
  dontTouch(io)

  // qos
  val qosName = if(gen.elements.contains("qos")) "qos" else "QoS"
  val rrName  = if(rr)  "RR"  else ""
  override val desiredName = "Hn" + (if(qos) qosName.toUpperCase else "") + rrName + "Arbiter"
  if(qos) require(gen.elements.contains(qosName))

  // define
  val lowVec  = WireInit(io.in)
  val highVec = WireInit(0.U.asTypeOf(Vec(size, Flipped(Decoupled(gen)))))
  val low     = WireInit(0.U.asTypeOf(Decoupled(gen)))
  val high    = WireInit(0.U.asTypeOf(Decoupled(gen)))
  val hasHigh = WireInit(false.B)

  // low priority
  if(rr) {
    low       <> VipArbiter(lowVec)
  } else {
    val arb   = Module(new Arbiter(gen, size))
    arb.io.in <> lowVec
    low       <> arb.io.out
  }

  // high priority
  if(qos) {
    highVec.zip(io.in).foreach { case(h, in) =>
      h.valid   := in.valid & in.bits.elements(qosName) === 0xF.U
      h.bits    := in.bits
    }
    hasHigh     := highVec.map(_.valid).reduce(_ | _)
    if (rr) {
      high      <> VipArbiter(highVec)
    } else {
      val arb   = Module(new Arbiter(gen, size))
      arb.io.in <> highVec
      high      <> arb.io.out
    }
  }

  // output
  when(hasHigh) {
    io.out      <> high
    low.ready   := false.B
  }.otherwise {
    io.out      <> low
    high.ready  := false.B
  }
  io.in.zip(lowVec.zip(highVec)).foreach { case(in, (l, h)) =>
    in.ready    := l.ready | h.ready
  }
  assert(PopCount(io.in.map(_.fire)) === io.out.fire)
}

class FastArbFactory(rr: Boolean, qos: Boolean) {
  def apply[T <: Bundle](in: Seq[DecoupledIO[T]]): DecoupledIO[T] = {
    val arb = Module(new ArbiterGenerator(chiselTypeOf(in.head.bits), in.size, rr, qos))
    val out = Wire(Decoupled(chiselTypeOf(in.head.bits)))
    arb.io.in.zip(in).foreach { case (a, b) => a.valid := b.valid; a.bits := b.bits; b.ready := a.ready }
    arb.io.out.ready := out.ready
    out.valid := arb.io.out.valid
    out.bits := arb.io.out.bits
    out
  }

  def apply[T <: Bundle](in: Seq[DecoupledIO[T]], out: DecoupledIO[T]): Unit = {
    val arbOut = apply(in)
    arbOut.ready := out.ready
    out.valid := arbOut.valid
    out.bits := arbOut.bits
  }

  def apply[T <: Bundle](in: Seq[ValidIO[T]]): ValidIO[T] = {
    val arb = Module(new ArbiterGenerator(chiselTypeOf(in.head.bits), in.size, rr, qos))
    val out = Wire(Valid(chiselTypeOf(in.head.bits)))
    arb.io.in.zip(in).foreach { case (a, b) => a.valid := b.valid; a.bits := b.bits }
    arb.io.out.ready := true.B
    out.valid := arb.io.out.valid
    out.bits := arb.io.out.bits
    out
  }

  def apply[T <: Bundle](in: Seq[ValidIO[T]], out: ValidIO[T]): Unit = {
    val arbOut = apply(in)
    out.valid := arbOut.valid
    out.bits := arbOut.bits
  }


  def validOut[T <: Bundle](in: Seq[DecoupledIO[T]]): ValidIO[T] = {
    val arb = Module(new ArbiterGenerator(chiselTypeOf(in.head.bits), in.size, rr, qos))
    val out = Wire(Valid(chiselTypeOf(in.head.bits)))
    arb.io.in.zip(in).foreach { case (a, b) => a.valid := b.valid; a.bits := b.bits; b.ready := a.ready }
    arb.io.out.ready := true.B
    out.valid := arb.io.out.valid
    out.bits := arb.io.out.bits
    out
  }

  def validOut[T <: Bundle](in: Seq[DecoupledIO[T]], out: ValidIO[T]): Unit = {
    val arbOut = validOut(in)
    out.valid := arbOut.valid
    out.bits := arbOut.bits
  }

}

object fastRRArb extends FastArbFactory(true, false)

object fastArb extends FastArbFactory(false, false)

object fastQosRRArb extends FastArbFactory(true, true)

object fastQosArb extends FastArbFactory(false, true)