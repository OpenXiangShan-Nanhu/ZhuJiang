package zhujiang.device.reset

import chisel3._
import chisel3.util.Cat

class ResetDevice extends Module {
  val io = IO(new Bundle {
    val resetInject = Output(Vec(2, Bool()))
    val resetState = Input(Vec(2, Bool()))
    val onReset = Output(Bool())
  })
  private val resetReg = RegInit(3.U(2.W))
  io.resetInject(0) := resetReg(0)
  io.resetInject(1) := resetReg(1)
  when(resetReg === 3.U) {
    resetReg := 1.U
  }.elsewhen(resetReg === 1.U && io.resetState(1) === false.B) {
    resetReg := 0.U
  }
  io.onReset := RegNext(Cat(io.resetState).orR)
}
