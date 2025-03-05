package dongjiang.utils

import chisel3._
import chisel3.util._

class DecoupledQueue[T <: Data](gen:T) extends Module {
  val io = IO(new Bundle {
    val enq   = Flipped(Decoupled(gen))
    val deq   = Decoupled(gen)
    val count = Output(UInt(2.W))
  })

  val q = Module(new Queue(gen, entries = 2, flow = false, pipe = false))

  // Solving enq ready timing problems
  val enqReadyReg = RegInit(true.B)
  switch(q.io.count) {
    is(2.U) { enqReadyReg := io.deq.ready }
    is(1.U) { enqReadyReg := io.deq.ready | !io.enq.valid }
    is(0.U) { enqReadyReg := true.B }
  }
  assert(!(enqReadyReg ^ q.io.enq.ready))

  // Solving deq valid timing problems
  val deqValidReg = RegInit(false.B)
  switch(q.io.count) {
    is(2.U) { deqValidReg := true.B }
    is(1.U) { deqValidReg := io.enq.valid | !io.deq.ready }
    is(0.U) { deqValidReg := io.enq.valid }
  }
  assert(!(deqValidReg ^ q.io.deq.valid))

  io.enq <> q.io.enq
  io.deq <> q.io.deq
  io.deq.bits := Mux(deqValidReg, q.io.deq.bits, 0.U.asTypeOf(io.deq.bits))
  io.enq.ready := enqReadyReg
  io.deq.valid := deqValidReg
  io.count := q.io.count
}


// For timing
object fastDecoupledQueue {
  def apply[T <: Data](in: DecoupledIO[T], out: DecoupledIO[T]): Unit = {
    val q = Module(new DecoupledQueue(chiselTypeOf(in.bits)))
    q.io.enq <> in
    q.io.deq <> out
  }

  def apply[T <: Data](in: DecoupledIO[T]): DecoupledIO[T] = {
    val q = Module(new DecoupledQueue(chiselTypeOf(in.bits)))
    q.io.enq <> in
    q.io.deq
  }

  def apply[T <: Data](in: DecoupledIO[T], enable: Boolean): DecoupledIO[T] = {
    val out = WireInit(0.U.asTypeOf(in))
    if(enable) {
      val q = Module(new DecoupledQueue(chiselTypeOf(in.bits)))
      q.io.enq <> in
      out <> q.io.deq
    } else {
      out <> in
    }
    out
  }
}



class CounterQueue[T <: Data](gen:T, entries:Int=1, count:Int=1) extends Module {
  val io = IO(new Bundle {
    val enq   = Flipped(Decoupled(gen))
    val deq   = Decoupled(gen)
    val count = Output(UInt(2.W))
  })

  val q = Module(new Queue(gen, entries = entries, pipe = count == 1))

  val counter = Counter(1 to count, q.io.deq.valid, io.deq.fire)

  // enq
  q.io.enq <> io.enq

  // deq
  io.deq.valid := io.deq.valid & counter._2


}