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
import freechips.rocketchip.regmapper.RegField.w

class AxiWSpilt(implicit p : Parameters) extends ZJModule with HasCircularQueuePtrHelper{
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits)

  private class CirQAxiEntryPtr extends CircularQueuePtr[CirQAxiEntryPtr](dmaParams.axiSpiltSize)
  private object CirQAxiEntryPtr {
  def apply(f: Bool, v: UInt): CirQAxiEntryPtr = {
        val ptr = Wire(new CirQAxiEntryPtr)
        ptr.flag := f
        ptr.value := v
        ptr
    }
  }
  private class CirQChiEntryPtr extends CircularQueuePtr[CirQChiEntryPtr](dmaParams.chiEntrySize)
  private object CirQChiEntryPtr {
  def apply(f: Bool, v: UInt): CirQChiEntryPtr = {
        val ptr = Wire(new CirQChiEntryPtr)
        ptr.flag := f
        ptr.value := v
        ptr
    }
  }

/* 
 * IO Interface Define
 */
  val io = IO(new Bundle {
    val uAxiAw = Flipped(Decoupled(new AWFlit(axiParams)))
    val uAxiW  = Flipped(Decoupled(new WFlit(axiParams)))
    val uAxiB  = Decoupled(new BFlit(axiParams))
    val dAxiAw = Decoupled(new AWFlit(axiParams))
    val dAxiW  = Decoupled(new WFlit(axiParams))
    val dAxiB  = Flipped(Decoupled(new BFlit(axiParams)))
  })

/* 
 * Reg and Wire Define
 */
  private val uAwEntrys      = Reg(Vec(dmaParams.axiEntrySize, new AxiWrEntry(isPipe = false)))
  private val uHeadPtr       = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uTailPtr       = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  private val dAwEntrys      = Reg(Vec(dmaParams.chiEntrySize, new dAwEntry))
  private val dHeadPtr       = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dTailPtr       = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val wDataPtr       = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))

  private val rxAwPipe       = Module(new Queue(gen = new AxiWrEntry(isPipe = true), entries = 2, pipe = false, flow = false))
  private val bIdQueue       = Module(new Queue(gen = UInt(axiParams.idBits.W), entries = 2, pipe = false, flow = false))
  private val merComReg      = RegInit(false.B)
  private val mergeLastReg   = RegInit(false.B)
  private val merDatReg      = RegInit(0.U(dw.W))
  private val merMaskReg     = RegInit(0.U(bew.W))
  private val maskData       = WireInit(VecInit.fill(dw/8){0.U(8.W)})

  private val rxAwBdl        = WireInit(0.U.asTypeOf(new AxiWrEntry(isPipe = true)))
  private val txAwBdl        = WireInit(0.U.asTypeOf(new AWFlit(axiParams)))

  private val uTailE         = uAwEntrys(uTailPtr.value)

/* 
 * Pointer Logic
 */
  uHeadPtr   := Mux(rxAwPipe.io.deq.fire, uHeadPtr + 1.U, uHeadPtr)
  uTailPtr   := Mux(io.dAxiAw.fire & ((uAwEntrys(uTailPtr.value).cnt.get + 1.U) === uAwEntrys(uTailPtr.value).num.get), uTailPtr + 1.U, uTailPtr)
  dHeadPtr   := Mux(io.dAxiAw.fire, dHeadPtr + 1.U, dHeadPtr)
  dTailPtr   := Mux(io.dAxiB.fire, dTailPtr + 1.U, dTailPtr)
  wDataPtr   := Mux(io.uAxiW.fire & (io.uAxiW.bits.last | dAwEntrys(wDataPtr.value).nextShift === 0.U | dAwEntrys(wDataPtr.value).burst =/= BurstMode.Incr), wDataPtr + 1.U, wDataPtr)

  maskData.zip(io.uAxiW.bits.strb.asBools).foreach{
    case(m, b) =>
      when(b === 1.U){
        m := 255.U
      }.otherwise{
        m := 0.U
      }
  }
  merDatReg    := Mux(io.dAxiW.fire, Mux(io.uAxiW.fire, maskData.asUInt & io.uAxiW.bits.data, 0.U), Mux(io.uAxiW.fire, merDatReg | maskData.asUInt & io.uAxiW.bits.data, merDatReg))
  merMaskReg   := Mux(io.dAxiW.fire, Mux(io.uAxiW.fire, io.uAxiW.bits.strb, 0.U), Mux(io.uAxiW.fire, io.uAxiW.bits.strb | merMaskReg, merMaskReg))

  merComReg    := Mux((!dAwEntrys(wDataPtr.value).nextShift(4, 0).orR | io.uAxiW.bits.last | dAwEntrys(wDataPtr.value).burst =/= BurstMode.Incr) & io.uAxiW.fire, true.B, Mux(io.dAxiW.fire, false.B, merComReg))
  mergeLastReg := Mux(!dAwEntrys(wDataPtr.value).nextShift(5, 0).orR | io.uAxiW.bits.last | dAwEntrys(wDataPtr.value).burst =/= BurstMode.Incr, true.B, Mux(io.dAxiW.fire, false.B, mergeLastReg))
  
  uAwEntrys.zipWithIndex.foreach{
    case(e, i) =>
      when(rxAwPipe.io.deq.fire & uHeadPtr.value === i.U){
        e.entryInit(rxAwPipe.io.deq.bits)
      }.elsewhen(uTailPtr.value === i.U & io.dAxiAw.fire){
        e.cnt.get := e.cnt.get + 1.U
        e.shiftAddr := Mux(e.burst === BurstMode.Incr, Cat((uTailE.shiftAddr(11, 6) + 1.U), 0.U(6.W)),
                        ~e.byteMask & e.shiftAddr | (e.shiftAddr + (1.U(7.W) << e.size)) & e.byteMask)
      }
  }
  dAwEntrys.zipWithIndex.foreach {
    case(e, i) =>
      when(io.dAxiAw.fire & dHeadPtr.value === i.U){
        e.burst     := uTailE.burst
        e.size      := 1.U(7.W) << uTailE.size
        e.shift     := uTailE.shiftAddr(5, 0)
        e.nextShift := uTailE.shiftAddr + (1.U(7.W) << uTailE.size)
        e.id        := uTailE.awid
        e.last      := (uTailE.cnt.get + 1.U) === uTailE.num.get
      }
      when(io.uAxiW.fire & wDataPtr.value === i.U){
        e.shift     := e.shift + e.size
        e.nextShift := e.nextShift + e.size
      }
  }

  txAwBdl                 := 0.U.asTypeOf(txAwBdl)
  txAwBdl.addr            := Mux(uTailE.prefixAddr(raw - 12 - 1) | uTailE.burst =/= BurstMode.Incr, 
                              Cat(uTailE.prefixAddr, uTailE.shiftAddr), Cat(uTailE.prefixAddr, uTailE.shiftAddr(11, 5), 0.U(5.W)))
  txAwBdl.size            := Mux(uTailE.prefixAddr(raw - 12 - 1) | uTailE.burst =/= BurstMode.Incr, uTailE.size, log2Ceil(dw/8).U)
  txAwBdl.burst           := BurstMode.Incr
  txAwBdl.id              := dHeadPtr.value
  txAwBdl.len             := Mux(uTailE.shiftAddr(5) | uTailE.burst =/= BurstMode.Incr | 
                               uTailE.endAddr(5, 0) <= "b100000".U & (uTailE.endAddr(11, 6) === uTailE.shiftAddr(11, 6)), 0.U, 1.U)

/* 
 * IO Connection
 */
  io.uAxiAw.ready         := rxAwPipe.io.enq.ready
  io.uAxiW.ready          := wDataPtr =/= dHeadPtr & io.dAxiW.ready
  io.dAxiAw.valid         := uHeadPtr =/= uTailPtr & !isFull(dHeadPtr, dTailPtr)
  io.dAxiAw.bits          := txAwBdl
  io.dAxiW.valid          := merComReg
  io.dAxiW.bits           := 0.U.asTypeOf(io.dAxiW.bits)
  io.dAxiW.bits.data      := merDatReg
  io.dAxiW.bits.strb      := merMaskReg
  io.dAxiW.bits.last      := mergeLastReg
  io.dAxiB.ready          := bIdQueue.io.enq.ready
  io.uAxiB.bits           := 0.U.asTypeOf(io.uAxiB.bits)
  io.uAxiB.bits.id        := bIdQueue.io.deq.bits
  io.uAxiB.valid          := bIdQueue.io.deq.valid

  bIdQueue.io.deq.ready   := io.uAxiB.ready
  bIdQueue.io.enq.valid   := dAwEntrys(dTailPtr.value).last & io.dAxiB.fire
  bIdQueue.io.enq.bits    := dAwEntrys(dTailPtr.value).id

  rxAwPipe.io.enq.bits    := rxAwBdl.pipeInit((io.uAxiAw.bits))
  rxAwPipe.io.enq.valid   := io.uAxiAw.valid
  rxAwPipe.io.deq.ready   := !isFull(uHeadPtr, uTailPtr)
/* 
 * Assertion
 */
  when(io.uAxiAw.fire & io.uAxiAw.bits.addr(raw - 1)){
    assert(io.uAxiAw.bits.len === 0.U, "The length of MMIO request must be 0")
  }
}