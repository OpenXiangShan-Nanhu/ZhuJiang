package dongjiang.utils

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.FastArbiter

object FastArb {
    def fastArbDec2Dec[T <: Bundle](in: Seq[DecoupledIO[T]], out: DecoupledIO[T], name: Option[String] = None): Unit = {
        val arb = Module(new FastArbiter[T](chiselTypeOf(out.bits), in.size))
        if (name.nonEmpty) {
            arb.suggestName(s"${name.get}_arb")
        }
        for ((a, req) <- arb.io.in.zip(in)) {
            a <> req
        }
        out <> arb.io.out
    }

    def fastArbDec[T <: Bundle](in: Seq[DecoupledIO[T]], name: Option[String] = None): DecoupledIO[T] = {
        val arb = Module(new FastArbiter[T](chiselTypeOf(in(0).bits), in.size))
        if (name.nonEmpty) {
            arb.suggestName(s"${name.get}_arb")
        }
        for ((a, req) <- arb.io.in.zip(in)) {
            a <> req
        }
        arb.io.out
    }

    def fastArbDec2Val[T <: Bundle](in: Seq[DecoupledIO[T]], out: ValidIO[T], name: Option[String] = None): Unit = {
        val arb = Module(new FastArbiter[T](chiselTypeOf(out.bits), in.size))
        if (name.nonEmpty) {
            arb.suggestName(s"${name.get}_arb")
        }
        for ((a, req) <- arb.io.in.zip(in)) {
            a <> req
        }
        arb.io.out.ready := true.B
        out.bits := arb.io.out.bits
        out.valid := arb.io.out.valid
    }

    def fastPriorityArbDec2Val[T <: Bundle](in: Seq[DecoupledIO[T]], out: ValidIO[T], name: Option[String] = None): Unit = {
        val arb = Module(new Arbiter[T](chiselTypeOf(out.bits), in.size))
        if (name.nonEmpty) {
            arb.suggestName(s"${name.get}_arb")
        }
        for ((a, req) <- arb.io.in.zip(in)) {
            a <> req
        }
        arb.io.out.ready := true.B
        out.bits := arb.io.out.bits
        out.valid := arb.io.out.valid
    }

    def fastPriorityArbDec[T <: Bundle](in: Seq[DecoupledIO[T]], name: Option[String] = None): DecoupledIO[T] = {
        val arb = Module(new Arbiter[T](chiselTypeOf(in(0).bits), in.size))
        if (name.nonEmpty) {
            arb.suggestName(s"${name.get}_arb")
        }
        for ((a, req) <- arb.io.in.zip(in)) {
            a <> req
        }
        arb.io.out
    }
}
