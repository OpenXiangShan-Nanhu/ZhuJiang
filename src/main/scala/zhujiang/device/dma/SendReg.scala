package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.cde.config.Parameters
import zhujiang._
import zhujiang.chi._
import zhujiang.axi._
import xs.utils.sram._
import xijiang._
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper}
import freechips.rocketchip.diplomacy.BufferParams.pipe

class SendDataIn(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val data        = UInt(dw.W)
  val beat        = UInt(1.W)
  val resp        = UInt(2.W)
  val id          = UInt(rni.idBits.W)
  val idx         = UInt(log2Ceil(node.outstanding).W)
  val last        = Bool()
}
class SendDataOut(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val data        = UInt(dw.W)
  val id          = UInt(rni.idBits.W)
  val idx         = UInt(log2Ceil(node.outstanding).W)
  val resp        = UInt(2.W)
}
class Pointer(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val outBeat     = UInt(1.W)
  val nextShift   = UInt(rni.offset.W)
  val endShift    = UInt(rni.offset.W)
  val merFixLen   = UInt(8.W)
}
class RBundle(node: Node)(implicit p: Parameters) extends  ZJBundle {
  private val rni = DmaParams(node = node)
  val data        = Vec(2, UInt(dw.W))
  val idx         = UInt(log2Ceil(node.outstanding).W)
  val id          = UInt(rni.idBits.W)
  val resp        = UInt(2.W)
}

class SendReg(node: Node)(implicit p: Parameters) extends ZJModule {
  private val rni   = DmaParams(node = node)
  val io            = IO(new Bundle {
    val dataIn      = Flipped(Decoupled(new SendDataIn(node)))
    val ptr         = Input(new Pointer(node))
    val dataOut     = Decoupled(new SendDataOut(node))
  })

/* 
 * Reg and Wire Define
 */
  private val sendQueue = Module(new Queue(gen = new RBundle(node), entries = 2, pipe = false, flow = false))
  private val mergeData = RegInit(VecInit.fill(2){0.U(dw.W)})

  mergeData.zipWithIndex.foreach {
    case(e, i) => 
      when(io.dataIn.fire & (io.dataIn.bits.beat === i.U)){
        e   := io.dataIn.bits.data
      }
  }
/* 
 * IO Connection Logic
 */
  sendQueue.io.enq.valid       := RegNext(io.dataIn.fire & io.dataIn.bits.last, false.B)
  sendQueue.io.enq.bits.data   := mergeData
  sendQueue.io.enq.bits.id     := RegEnable(io.dataIn.bits.id, io.dataIn.fire)
  sendQueue.io.enq.bits.idx    := RegEnable(io.dataIn.bits.idx, io.dataIn.fire)
  sendQueue.io.enq.bits.resp   := RegEnable(io.dataIn.bits.resp, io.dataIn.fire)
  sendQueue.io.deq.ready       := ((io.ptr.nextShift === io.ptr.endShift) & (io.ptr.merFixLen === 0.U) || io.ptr.merFixLen === 1.U) && io.dataOut.fire


  io.dataIn.ready         := sendQueue.io.enq.ready & !(sendQueue.io.deq.valid & !sendQueue.io.deq.ready) || !io.dataIn.bits.last
  io.dataOut.valid        := sendQueue.io.deq.valid
  io.dataOut.bits.data    := sendQueue.io.deq.bits.data(io.ptr.outBeat)
  io.dataOut.bits.id      := sendQueue.io.deq.bits.id
  io.dataOut.bits.idx     := sendQueue.io.deq.bits.idx
  io.dataOut.bits.resp    := sendQueue.io.deq.bits.resp
}