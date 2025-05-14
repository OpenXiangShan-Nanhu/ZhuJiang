package zhujiang.utils

import chisel3._
import chisel3.util._

class DoubleCounterClockGate(ckenWindow:Int = 7, idleWindow:Int = 7) extends Module {
  val io = IO(new Bundle {
    val te = Input(Bool())
    val inbound = Input(Bool())
    val working = Input(Bool())
    val ock = Output(Clock())
    val active = Output(Bool())
  })
  private val ckenWindowCnt = RegInit(ckenWindow.U(log2Ceil(ckenWindow).W))
  private val idleWindowCnt = RegInit(idleWindow.U(log2Ceil(idleWindow).W))
  private val cken = RegInit(true.B)
  private val active = RegInit(false.B)
  private val cg = Module(new xs.utils.ClockGate)
  cg.io.TE := io.te
  cg.io.CK := clock
  io.ock := cg.io.Q
  cg.io.E := cken | io.inbound
  io.active := active
  cken := Mux(cken, ckenWindowCnt.orR || idleWindowCnt.orR, io.inbound)
  active := Mux(active, ckenWindowCnt.orR || idleWindowCnt.orR, io.inbound)
  when(io.inbound) {
    ckenWindowCnt := Fill(ckenWindowCnt.getWidth, true.B)
  }.elsewhen(ckenWindowCnt.orR) {
    ckenWindowCnt := ckenWindowCnt - 1.U
  }
  when(io.working) {
    idleWindowCnt := Fill(idleWindowCnt.getWidth, true.B)
  }.elsewhen(idleWindowCnt.orR) {
    idleWindowCnt := idleWindowCnt - 1.U
  }
}
