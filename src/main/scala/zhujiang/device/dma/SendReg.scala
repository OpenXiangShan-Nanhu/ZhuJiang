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

class SendDataIn(implicit p: Parameters) extends ZJBundle {
  private val rni = zjParams.dmaParams
  val data        = UInt(dw.W)
  val beat        = UInt(1.W)
  val id          = UInt(rni.idBits.W)
  val idx         = UInt(log2Ceil(rni.chiEntrySize).W)
  val last        = Bool()
}
class SendDataOut(implicit p: Parameters) extends ZJBundle {
  private val rni = zjParams.dmaParams
  val data        = UInt(dw.W)
  val id          = UInt(rni.idBits.W)
  val idx         = UInt(log2Ceil(rni.chiEntrySize).W)
  val resp        = UInt(2.W)
}
class Pointer(implicit p: Parameters) extends ZJBundle {
  private val rni = zjParams.dmaParams
  val shift       = UInt(rni.offset.W)
  val nextShift   = UInt(rni.offset.W)
  val endShift    = UInt(rni.offset.W)
}
class RBundle(implicit p: Parameters) extends  ZJBundle {
  private val rni = zjParams.dmaParams
  val data        = Vec(2, UInt(dw.W))
  val idx         = UInt(log2Ceil(rni.chiEntrySize).W)
  val id          = UInt(rni.idBits.W)
}

class SendReg(implicit p: Parameters) extends ZJModule {
  private val rni   = zjParams.dmaParams
  val io            = IO(new Bundle {
    val dataIn      = Flipped(Decoupled(new SendDataIn))
    val ptr         = Input(new Pointer)
    val dataOut     = Decoupled(new SendDataOut)
  })

/* 
 * Reg and Wire Define
 */
  private val sendQueue = Module(new Queue(gen = new RBundle, entries = 2, pipe = true, flow = false))
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
  sendQueue.io.enq.valid       := RegNext(io.dataIn.fire & io.dataIn.bits.last)
  sendQueue.io.enq.bits.data   := mergeData
  sendQueue.io.enq.bits.id     := RegNext(io.dataIn.bits.id)
  sendQueue.io.enq.bits.idx    := RegNext(io.dataIn.bits.idx)
  sendQueue.io.deq.ready       := io.ptr.nextShift === io.ptr.endShift


  io.dataIn.ready         := sendQueue.io.enq.ready & !(sendQueue.io.deq.valid & !sendQueue.io.deq.ready) || !io.dataIn.bits.last
  io.dataOut.valid        := sendQueue.io.deq.valid
  io.dataOut.bits.data    := sendQueue.io.deq.bits.data(io.ptr.shift(rni.offset - 1))
  io.dataOut.bits.id      := sendQueue.io.deq.bits.id
  io.dataOut.bits.idx     := sendQueue.io.deq.bits.idx
  io.dataOut.bits.resp    := DontCare
}