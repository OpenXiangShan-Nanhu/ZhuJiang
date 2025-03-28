package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{DomainInfo, HardwareAssertion}
import xs.utils.queue.FastQueue

class DBIDPool(implicit p: Parameters) extends DJModule {
  val io = IO(new Bundle {
    val enq0    = Flipped(Valid(UInt(dbIdBits.W)))
    val enq1    = Flipped(Valid(UInt(dbIdBits.W)))
    val deq0    = Decoupled(UInt(dbIdBits.W))
    val deq1    = Decoupled(UInt(dbIdBits.W))
    val hasOne  = Output(Bool())
    val hasTwo  = Output(Bool())
  })
  dontTouch(io)
  val q0 = Module(new FastQueue(UInt(dbIdBits.W), djparam.nrDataBuf/2, true))
  val q1 = Module(new FastQueue(UInt(dbIdBits.W), djparam.nrDataBuf/2, true))

  // reset
  val rstCounter = Counter(djparam.nrDataBuf/2)
  val rstDoneReg = RegEnable(true.B, false.B, rstCounter.inc)

  // enq
  val enqOne  = io.enq0.valid ^ io.enq1.valid
  val enqTwo  = io.enq0.valid & io.enq1.valid
  val enqSel0 = q0.io.count === q1.io.count
  HardwareAssertion.withEn(!io.enq1.valid, enqOne)
  HardwareAssertion(q0.io.count >= q1.io.count)
  HardwareAssertion(q0.io.count - q1.io.count <= 1.U)

  // Reset
  q0.io.enq <> DontCare
  q1.io.enq <> DontCare
  when(!rstDoneReg) {
    q0.io.enq.valid := true.B
    q1.io.enq.valid := true.B
    q0.io.enq.bits  := Cat(0.U, rstCounter.value)
    q1.io.enq.bits  := Cat(1.U, rstCounter.value)
  // Enq
  }.elsewhen(enqOne & enqSel0) {
    q0.io.enq.valid := io.enq0.valid
    q0.io.enq.bits  := io.enq0.bits
  }.elsewhen(enqOne & !enqSel0) {
    q1.io.enq.valid := io.enq0.valid
    q1.io.enq.bits  := io.enq0.bits
  }.elsewhen(enqTwo) {
    q0.io.enq.valid := io.enq0.valid
    q0.io.enq.bits  := io.enq0.bits
    q1.io.enq.valid := io.enq1.valid
    q1.io.enq.bits  := io.enq1.bits
  }
  HardwareAssertion.withEn(q0.io.enq.ready, q0.io.enq.valid)
  HardwareAssertion.withEn(q0.io.enq.ready, q0.io.enq.valid)

  // Deq
  io.deq0 <> q0.io.deq
  io.deq1 <> q1.io.deq
  io.hasOne := q0.io.deq.ready
  io.hasTwo := q1.io.deq.ready

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 3)
}