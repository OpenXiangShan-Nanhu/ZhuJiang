package dongjiang.utils

import chisel3._
import chisel3.util._
import xs.utils.ResetRRArbiter

class StepRREncoder(size: Int) extends Module {
  val io = IO(new Bundle {
    val inVec   = Input(Vec(size, Bool()))
    val enable  = Input(Bool())
    val outIdx  = Output(UInt(log2Ceil(size).W))
  })
  require(isPow2(size))

  val indexReg  = RegInit(0.U(log2Ceil(size).W))
  val indexOut  = WireInit(0.U(log2Ceil(size).W))

  when(io.inVec(indexReg)) {
    indexReg    := Mux(io.enable, indexReg + 1.U, indexReg)
    indexOut    := indexReg
  }.otherwise {
    indexReg    := Mux(io.inVec.reduce(_ | _) & io.enable, Mux(indexOut > indexReg, indexOut, indexReg + 1.U), indexReg)
    indexOut    := PriorityEncoder(io.inVec)
  }

  io.outIdx     := indexOut
}

object StepRREncoder {
  def apply(in: Seq[Bool], enable: Bool): UInt = {
    val stepRREncoder = Module(new StepRREncoder(in.size))
    stepRREncoder.io.inVec.zip(in).foreach { case(a, b) => a := b }
    stepRREncoder.io.enable := enable
    stepRREncoder.io.outIdx
  }
}

object StepRREncoderOH {
  def apply(in: Seq[Bool], enable: Bool): UInt = {
    val stepRREncoder = Module(new StepRREncoder(in.size))
    stepRREncoder.io.inVec.zip(in).foreach { case(a, b) => a := b }
    stepRREncoder.io.enable := enable
    UIntToOH(stepRREncoder.io.outIdx)
  }
}


object RREncoder {
  def apply(in: Seq[Bool]): UInt = {
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
