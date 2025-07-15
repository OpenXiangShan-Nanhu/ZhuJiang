package dongjiang.utils

import chisel3._
import chisel3.util._

trait HasQoS { this: Bundle => val qos = UInt(4.W) }

class ArbiterGenerator[T <: Bundle](gen:T, size:Int, rr:Boolean, qos: Boolean) extends Module {
  val io    = IO(new Bundle {
    val in  = Vec(size, Flipped(Decoupled(gen)))
    val out = Decoupled(gen)
  })
  val qosName = if(gen.elements.contains("qos")) "qos" else "QoS"
  val rrName  = if(rr)  "RR"  else ""
  override val desiredName = "Hn" + (if(qos) qosName.toUpperCase else "") + rrName + "Arbiter"
  if(qos) require(gen.elements.contains(qosName))
  // define ids
  val selId   = Wire(UInt(log2Ceil(size).W))
  val lowId   = Wire(UInt(log2Ceil(size).W))
  val highId  = WireInit(0.U(log2Ceil(size).W))
  val lowVec  = VecInit(io.in.map(in => in.valid))
  val highVec = WireInit(VecInit(Seq.fill(size) { false.B }))
  // low
  if(rr) {
    lowId     := StepRREncoder(lowVec,  io.out.fire & !highVec.asUInt.orR)
  } else {
    lowId     := PriorityEncoder(lowVec)
  }
  // QoS
  if(qos) {
    // high
    highVec   := VecInit(io.in.map(in => in.valid & in.bits.elements(qosName) === 0xF.U))
    if (rr) {
      highId  := StepRREncoder(highVec, io.out.fire)
    } else {
      highId  := PriorityEncoder(highId)
    }
    selId     := Mux(highVec.asUInt.orR, highId, lowId)
  // no QoS
  } else {
    selId     := lowId
  }
  // Output
  io.in.zipWithIndex.foreach { case (in, i) => in.ready := io.out.ready & selId === i.U }
  io.out.valid  := io.in.map(_.valid).reduce(_ | _)
  io.out.bits   := io.in(selId).bits
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