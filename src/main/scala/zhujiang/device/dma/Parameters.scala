package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.cde.config.Parameters
import zhujiang._
import zhujiang.chi._
import zhujiang.axi._


object BurstMode {
  val width        = 2
  val Fix          = "b00".U
  val Incr         = "b01".U
  val Wrap         = "b10".U
  val Reserve      = "b11".U
}
object AXIRState {
  val width = 2
  val Free  = "b00".U
  val Send  = "b01".U
  val Comp  = "b10".U
}
class AXIREntry(implicit p: Parameters) extends ZJBundle {
    val state           = UInt(AXIRState.width.W)
    val addr            = UInt(raw.W)
    val burst           = UInt(BurstMode.width.W)
    val len             = UInt(8.W)
    val arid            = UInt(8.W)
    val nid             = UInt(log2Ceil(zjParams.dmaParams.axiEntrySize).W)
    val sendReqNum      = UInt(6.W)
    val sendDatNum      = UInt(6.W)  
}

class CHIREntry(implicit p : Parameters) extends ZJBundle {
  val areid          = UInt(log2Ceil(zjParams.dmaParams.axiEntrySize).W)
  val state          = UInt(CHIRState.width.W)
  val num            = UInt(6.W)
  val last           = Bool()
  val next           = UInt((log2Ceil(zjParams.dmaParams.chiEntrySize)).W)
  val full           = Bool()
  val homeNid        = UInt(niw.W)
  val dbid           = UInt(12.W)
  val haveRecReceipt = Bool()
  val haveRecDataFir = Bool()
  val haveRecDataSec = Bool()
}

object CHIRState {
  val width        = 3
  val Free         = "b000".U
  val Wait         = "b001".U
  val sendCompAck  = "b010".U
  val SendData     = "b011".U
  val Comp         = "b100".U
}
class SRAMSelector(implicit p: Parameters) extends ZJModule {
  private val dmaParams = zjParams.dmaParams
  val io = IO(new Bundle() {
    val idle = Input(Vec(dmaParams.chiEntrySize, Bool()))
    val idleNum = Output(UInt((log2Ceil(dmaParams.chiEntrySize) + 1).W))
    val out0 = UInt(log2Ceil(dmaParams.chiEntrySize).W)
    val out1 = UInt(log2Ceil(dmaParams.chiEntrySize).W)
  })
  io.idleNum := PopCount(io.idle)
  io.out0    := PriorityEncoder(io.idle)
  val idle1   = WireInit(io.idle)
  idle1(io.out0) := false.B
  io.out1    := PriorityEncoder(idle1)
}