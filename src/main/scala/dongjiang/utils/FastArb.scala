package dongjiang.utils

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._

class ArbiterWithReset[T <: Data](gen:T, size:Int, rr:Boolean) extends Module {
  val io = IO(new Bundle {
    val in = Vec(size, Flipped(Decoupled(gen)))
    val out = Decoupled(gen)
  })

  if(rr) {
    val arb = Module(new RRArbiter(gen, size))
    arb.io.in.zip(io.in).foreach { case (a, b) => a <> b }
    io.out <> arb.io.out
  } else {
    val arb = Module(new Arbiter(gen, size))
    arb.io.in.zip(io.in).foreach { case (a, b) => a <> b }
    io.out <> arb.io.out
  }

  if(size == 1){
    io.out.bits := Mux(io.out.valid, io.in.head.bits, 0.U.asTypeOf(io.out.bits))
  }
}


object fastRRArb {
  def apply[T <: Bundle](in: Seq[DecoupledIO[T]], out: DecoupledIO[T]): Unit = {
    val arb = Module(new ArbiterWithReset(chiselTypeOf(in(0).bits), in.size, true))
    arb.io.in.zip(in).foreach { case(a, b) => a <> b }
    out <> arb.io.out
  }

  def apply[T <: Bundle](in: Seq[DecoupledIO[T]]): DecoupledIO[T] = {
    val arb = Module(new ArbiterWithReset(chiselTypeOf(in(0).bits), in.size, true))
    arb.io.in.zip(in).foreach { case (a, b) => a <> b }
    arb.io.out
  }

  def apply[T <: Bundle](in: Seq[DecoupledIO[T]], out: ValidIO[T]): Unit = {
    val arb = Module(new ArbiterWithReset(chiselTypeOf(in.head.bits), in.size, true))
    arb.io.in.zip(in).foreach { case (a, b) => a <> b }
    arb.io.out.ready := true.B
    out.valid := arb.io.out.valid
    out.bits := arb.io.out.bits
  }

  def validOut[T <: Bundle](in: Seq[DecoupledIO[T]]): ValidIO[T] = {
    val arb = Module(new ArbiterWithReset(chiselTypeOf(in.head.bits), in.size, true))
    val out = Wire(Valid(chiselTypeOf(in.head.bits)))
    arb.io.in.zip(in).foreach { case (a, b) => a <> b }
    arb.io.out.ready := true.B
    out.valid := arb.io.out.valid
    out.bits := arb.io.out.bits
    out
  }
}

object fastArb {
  def apply[T <: Bundle](in: Seq[DecoupledIO[T]], out: DecoupledIO[T]): Unit = {
    val arb = Module(new ArbiterWithReset(chiselTypeOf(in.head.bits), in.size, false))
    arb.io.in.zip(in).foreach { case(a, b) => a <> b }
    out <> arb.io.out
  }

  def apply[T <: Bundle](in: Seq[DecoupledIO[T]]): DecoupledIO[T] = {
    val arb = Module(new ArbiterWithReset(chiselTypeOf(in.head.bits), in.size, false))
    arb.io.in.zip(in).foreach { case (a, b) => a <> b }
    arb.io.out
  }

  def apply[T <: Bundle](in: Seq[DecoupledIO[T]], out: ValidIO[T]): Unit = {
    val arb = Module(new ArbiterWithReset(chiselTypeOf(in.head.bits), in.size, false))
    arb.io.in.zip(in).foreach { case (a, b) => a <> b }
    arb.io.out.ready := true.B
    out.valid := arb.io.out.valid
    out.bits := arb.io.out.bits
  }

  def validOut[T <: Bundle](in: Seq[DecoupledIO[T]]): ValidIO[T] = {
    val arb = Module(new ArbiterWithReset(chiselTypeOf(in.head.bits), in.size, false))
    val out = Wire(Valid(chiselTypeOf(in.head.bits)))
    arb.io.in.zip(in).foreach { case (a, b) => a <> b }
    arb.io.out.ready := true.B
    out.valid := arb.io.out.valid
    out.bits := arb.io.out.bits
    out
  }
}
