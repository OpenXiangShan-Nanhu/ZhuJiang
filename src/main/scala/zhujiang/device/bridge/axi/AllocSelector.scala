package zhujiang.device.bridge.axi

import chisel3._
import chisel3.util._
import xs.utils.ResetRRArbiter

class DataBufferAllocReq(outstanding: Int) extends Bundle {
  val size = UInt(3.W)
  val waitNum = UInt(log2Ceil(outstanding).W)
}

class SelNto1(size:Int, outstanding: Int) extends Module {
  val io = IO(new Bundle {
    val in = Vec(size, Flipped(Valid(new DataBufferAllocReq(outstanding))))
    val out = Output(UInt(outstanding.W))
  })

  private val oldestOHMatrix = io.in.zipWithIndex.map({ case (self, idx) =>
    io.in.zipWithIndex.filterNot(_._2 == idx).map(i => (i._1.valid && self.valid && (self.bits.waitNum <= i._1.bits.waitNum)) ^ i._1.valid)
  })
  private val oldestOHSeq = oldestOHMatrix.map(_.reduce(_ | _)).map(!_)
  private val oldestOH = Cat(oldestOHSeq.reverse)
  private val valids = Cat(io.in.map(_.valid).reverse)
  io.out := oldestOH & valids
  when(valids.orR) {
    assert(io.out.orR)
  }
}

class DataBufferAllocReqSelector(outstanding: Int) extends Module {
  val io = IO(new Bundle {
    val in = Vec(outstanding, Flipped(Decoupled(new DataBufferAllocReq(outstanding))))
    val out = Decoupled(new AxiDataBufferAllocReq(outstanding))
  })

  private val selector = Module(new SelNto1(outstanding, outstanding))
  private val selReg = RegNext(selector.io.out) // Do not gate this reg
  private val selArb = Module(new ResetRRArbiter(new AxiDataBufferAllocReq(outstanding), outstanding))
  private val selPipe = Module(new Queue(new AxiDataBufferAllocReq(outstanding), entries = 2))

  for(i <- io.in.indices) {
    selector.io.in(i).valid := io.in(i).valid && !selArb.io.in(i).fire
    selector.io.in(i).bits := io.in(i).bits
    io.in(i).ready := selArb.io.in(i).fire

    selArb.io.in(i).valid := selReg(i)
    selArb.io.in(i).bits.idxOH := (1L << i).U
    selArb.io.in(i).bits.size := io.in(i).bits.size
  }
  selPipe.io.enq <> selArb.io.out
  io.out <> selPipe.io.deq
}
