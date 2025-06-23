package zhujiang.device.bridge.axi

import chisel3._
import chisel3.util._
import xs.utils.arb.{SelNto1, VipArbiter}

class DataBufferAllocReq(outstanding: Int) extends Bundle {
  val qos = UInt(4.W)
  val size = UInt(3.W)
  val dataIdOffset = UInt(2.W)
}

class DataBufferAllocReqSelector(outstanding: Int) extends Module {
  val io = IO(new Bundle {
    val in = Vec(outstanding, Flipped(Decoupled(new DataBufferAllocReq(outstanding))))
    val out = Decoupled(new AxiDataBufferAllocReq(outstanding))
  })
  private def selFunc(self:DataBufferAllocReq, other:DataBufferAllocReq):Bool = self.qos >= other.qos
  private val selector = Module(new SelNto1(new DataBufferAllocReq(outstanding), outstanding, selFunc))
  private val selReg = RegNext(selector.io.out) // Do not gate this reg
  private val selArb = Module(new VipArbiter(new AxiDataBufferAllocReq(outstanding), outstanding))
  private val selPipe = Module(new Queue(new AxiDataBufferAllocReq(outstanding), entries = 2))

  for(i <- io.in.indices) {
    selector.io.in(i).valid := io.in(i).valid && !selArb.io.in(i).fire
    selector.io.in(i).bits := io.in(i).bits
    io.in(i).ready := selArb.io.in(i).fire

    selArb.io.in(i).valid := selReg(i)
    selArb.io.in(i).bits.idxOH := (1.U(outstanding.W)) << i
    selArb.io.in(i).bits.size := io.in(i).bits.size
    selArb.io.in(i).bits.dataIdOffset := io.in(i).bits.dataIdOffset
  }
  selPipe.io.enq <> selArb.io.out
  io.out <> selPipe.io.deq
}
