package dongjiang.utils

import chisel3._
import chisel3.util._
import chisel3.util.RRArbiter
import xs.utils.ResetRRArbiter

object Encoder {
    def RREncoder(in: Seq[Bool]): UInt = {
        val arb = Module(new ResetRRArbiter(UInt(log2Ceil(in.size).W), in.size))
        arb.io.in.zipWithIndex.foreach {
            case(a, i) =>
                a.valid := in(i)
                a.bits  := i.U
        }
        arb.io.out.ready := true.B
        arb.io.out.bits
    }
}