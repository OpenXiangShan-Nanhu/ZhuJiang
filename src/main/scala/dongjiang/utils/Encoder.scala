package dongjiang.utils

import chisel3._
import chisel3.util._
import xs.utils.ResetRRArbiter

class StepRREncoder(size: Int, hasLock: Boolean = false, lockCnt: Int = 7) extends Module {
  val io = IO(new Bundle {
    val inVec   = Input(Vec(size, Bool()))
    val enable  = Input(Bool())
    val outIdx  = Output(UInt(log2Ceil(size).W))
    val vipIdx  = Output(UInt(log2Ceil(size).W))
    val lock    = Output(Bool())
  })

  val vipIdxReg = RegInit(0.U(log2Ceil(size).W))
  val priIdx    = PriorityEncoder(io.inVec)
  val lock      = WireInit(false.B)

  // Inc logic
  def indexInc(index: UInt): UInt = {
    if(isPow2(size)) {
      index + 1.U
    } else {
      val newIndex = Wire(UInt(log2Ceil(size).W))
      when(index +& 1.U === size.U) {
        newIndex := 0.U
      }.otherwise {
        newIndex := index + 1.U
      }
      newIndex
    }
  }

  // Select logic
  when(io.enable) {
    vipIdxReg := PriorityMux(Seq(
      (priIdx < vipIdxReg) -> indexInc(vipIdxReg),
      io.inVec(vipIdxReg)  -> indexInc(vipIdxReg),
      (priIdx > vipIdxReg) -> priIdx
    ))
  }
  assert(vipIdxReg < size.U)

  // Lock logic
  if (hasLock) {
    val cntReg  = RegInit(0.U(log2Ceil(lockCnt+1).W))
    // reset
    when(io.enable) {
      cntReg    := 0.U
    // inc
    }.elsewhen(io.inVec(vipIdxReg) & cntReg < lockCnt.U) {
      cntReg    := cntReg + 1.U
    }
    lock        := RegNext(cntReg === lockCnt.U)
  }

  // Output logic
  io.outIdx     := Mux(io.inVec(vipIdxReg) | lock, io.vipIdx, priIdx)
  io.vipIdx     := vipIdxReg
  io.lock       := lock
}

object StepRREncoder {
  def apply(in: Seq[Bool], enable: Bool): UInt = {
    val stepRREncoder = Module(new StepRREncoder(in.size))
    stepRREncoder.io.inVec.zip(in).foreach { case(a, b) => a := b }
    stepRREncoder.io.enable := enable
    stepRREncoder.io.outIdx
  }
  def apply(in: Bits, enable: Bool): UInt = {
    apply(in.asBools, enable)
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
  def apply(in: Bits): UInt = {
    apply(in.asBools)
  }
}
