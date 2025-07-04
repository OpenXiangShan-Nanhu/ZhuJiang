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

class MergeDataIn(implicit p: Parameters) extends ZJBundle {
  val data       = UInt(dw.W) 
  val strb       = UInt(bew.W)
  val fixMerge   = Bool()
  val beat       = UInt(1.W)
}
class MergeDataOut(implicit p: Parameters) extends ZJBundle {
  val data  = UInt(dw.W)
  val strb  = UInt(bew.W)
  val beat  = UInt(1.W)
}
class MergeReg(implicit p: Parameters) extends ZJModule {
  val io          = IO(new Bundle {
    val dataIn    = Flipped(Decoupled(new MergeDataIn))
    val dataOut   = Decoupled(new MergeDataOut)
  })
/* 
 * Reg and Wire Define
 */
  private val mergeDataBeat1 = Reg(UInt(dw.W))
  private val dataBeat1Next  = WireInit(mergeDataBeat1)
  private val mergeDataBeat2 = Reg(UInt(dw.W))
  private val dataBeat2Next  = WireInit(mergeDataBeat2)
  private val mergeStrbBeat1 = Reg(UInt(bew.W))
  private val strbBeat1Next  = WireInit(mergeStrbBeat1)
  private val mergeStrbBeat2 = Reg(UInt(bew.W))
  private val strbBeat2Next  = WireInit(mergeStrbBeat2)
  private val maskData       = WireInit(VecInit.fill(dw/8){0.U(8.W)})
  private val validBeat1     = RegInit(0.U(1.W))
  private val validBeat2     = RegInit(0.U(1.W))

/* 
 * Assign Logic
 */
  private val firInAndOut    =   io.dataIn.fire & !io.dataIn.bits.beat  &   io.dataOut.fire & !io.dataOut.bits.beat
  private val firInAndNOut   =   io.dataIn.fire & !io.dataIn.bits.beat  & !(io.dataOut.fire & !io.dataOut.bits.beat) & !io.dataIn.bits.fixMerge
  private val firNInAndOut   = !(io.dataIn.fire & !io.dataIn.bits.beat) &   io.dataOut.fire & !io.dataOut.bits.beat
  private val firNInAndNOut  = !(io.dataIn.fire & !io.dataIn.bits.beat) & !(io.dataOut.fire & !io.dataOut.bits.beat)
  private val firFixMerge    =   io.dataIn.fire & !io.dataIn.bits.beat  & !(io.dataOut.fire & !io.dataOut.bits.beat) & io.dataIn.bits.fixMerge
 
  private val secInAndOut    =   io.dataIn.fire &  io.dataIn.bits.beat  &   io.dataOut.fire &  io.dataOut.bits.beat
  private val secInAndNOut   =   io.dataIn.fire &  io.dataIn.bits.beat  & !(io.dataOut.fire &  io.dataOut.bits.beat) & !io.dataIn.bits.fixMerge
  private val secNInAndOut   = !(io.dataIn.fire &  io.dataIn.bits.beat) &   io.dataOut.fire &  io.dataOut.bits.beat
  private val secNInAndNout  = !(io.dataIn.fire &  io.dataIn.bits.beat) & !(io.dataOut.fire &  io.dataOut.bits.beat)
  private val secFixMerge    =   io.dataIn.fire &  io.dataIn.bits.beat  & !(io.dataOut.fire &  io.dataOut.bits.beat) & io.dataIn.bits.fixMerge

  maskData.zip(io.dataIn.bits.strb.asBools).foreach{
    case(m, b) =>
      when(b === 1.U){
        m := 255.U
      }.otherwise{
        m := 0.U
      }
  }
  private val dataIn   = io.dataIn.bits.data & maskData.asUInt

  dataBeat1Next := PriorityMux(Seq(
    firInAndOut.asBool   ->  dataIn,
    firInAndNOut.asBool  -> (dataIn | mergeDataBeat1),
    firFixMerge.asBool   ->  dataIn,
    firNInAndOut.asBool  ->  0.U,
    true.B               ->  mergeDataBeat1
  ))
  dataBeat2Next := PriorityMux(Seq(
    secInAndOut.asBool   ->  dataIn,
    secInAndNOut.asBool  -> (dataIn | mergeDataBeat2),
    secFixMerge.asBool   ->  dataIn,
    secNInAndOut.asBool  ->  0.U,
    true.B               ->  mergeDataBeat2
  ))
  strbBeat1Next := PriorityMux(Seq(
    firInAndOut.asBool   ->  io.dataIn.bits.strb,
    firInAndNOut.asBool  -> (io.dataIn.bits.strb | mergeStrbBeat1),
    firFixMerge.asBool   ->  io.dataIn.bits.strb,
    firNInAndOut.asBool  ->  0.U,
    true.B               ->  mergeStrbBeat1
  ))
  strbBeat2Next := PriorityMux(Seq(
    secInAndOut.asBool   ->  io.dataIn.bits.strb,
    secInAndNOut.asBool  -> (io.dataIn.bits.strb | mergeStrbBeat2),
    secFixMerge.asBool   ->  io.dataIn.bits.strb,
    secNInAndOut.asBool  ->  0.U,
    true.B               ->  mergeStrbBeat2
  ))
  validBeat1   := PriorityMux(Seq(
    firInAndOut.asBool   -> 1.U,
    firInAndNOut.asBool  -> 1.U,
    firFixMerge.asBool   -> 1.U,
    firNInAndOut.asBool  -> 0.U,
    firNInAndNOut.asBool -> validBeat1
  ))
  validBeat2   := PriorityMux(Seq(
    secInAndOut.asBool   -> 1.U,
    secInAndNOut.asBool  -> 1.U,
    secFixMerge.asBool   -> 1.U,
    secNInAndOut.asBool  -> 0.U,
    true.B               -> validBeat2
  ))
  when(io.dataIn.fire || io.dataOut.fire){
    mergeDataBeat1 := dataBeat1Next
    mergeDataBeat2 := dataBeat2Next
    mergeStrbBeat1 := strbBeat1Next
    mergeStrbBeat2 := strbBeat2Next
  }

/* 
 * IO Connection Logic
 */
  io.dataOut.valid     := validBeat1 | validBeat2
  io.dataOut.bits.beat := Mux(validBeat1.asBool, 0.U, 1.U)
  io.dataOut.bits.data := Mux(validBeat1.asBool, mergeDataBeat1, mergeDataBeat2)
  io.dataOut.bits.strb := Mux(validBeat1.asBool, mergeStrbBeat1, mergeStrbBeat2)

  io.dataIn.ready      := true.B
}