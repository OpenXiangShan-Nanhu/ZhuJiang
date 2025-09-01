package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.cde.config.Parameters
import chisel3.experimental.BundleLiterals._
import zhujiang._
import zhujiang.chi._
import zhujiang.axi._
import xs.utils.sram._
import xijiang._
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper}
import dongjiang.utils.StepRREncoder


class AxiDevSpilt(node: Node)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni           = DmaParams(node = node)
  private val axiParams     = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamsLast = axiParams.copy(userBits = 1)

  private class CirQAxiEntryPtr extends CircularQueuePtr[CirQAxiEntryPtr](rni.axiEntrySize)
  private object CirQAxiEntryPtr {
  def apply(f: Bool, v: UInt): CirQAxiEntryPtr = {
        val ptr = Wire(new CirQAxiEntryPtr)
        ptr.flag := f
        ptr.value := v
        ptr
    }
  }
/* 
 * IO Interface Define
 */
  val io = IO(new Bundle {
    val uAxiAx   = Flipped(Decoupled(new AXFlit(axiParams)))
    val dAxiAx   = Decoupled(new AXFlit(axiParamsLast))
    val working  = Output(Bool())
  })

/* 
 * Reg and Pointer declare
 */
  private val axiEntries = Reg(Vec(rni.axiEntrySize, new AxiOriEntry(node = node)))
  private val headPtr    = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val tailPtr    = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  //Wire declare
  private val dAxiAXWire  = WireInit(0.U.asTypeOf(io.dAxiAx.bits))
  private val entryInit   = io.uAxiAx.fire
  private val entryMod    = io.dAxiAx.fire

  // pointer movement logic 
  private val tailE       = axiEntries(tailPtr.value)
  private val headPtrAdd  = io.uAxiAx.fire
  private val tailPtrAdd  = io.dAxiAx.fire & (tailE.len === 0.U || tailE.cache(1)) 

  headPtr                := Mux(headPtrAdd, headPtr + 1.U, headPtr)
  tailPtr                := Mux(tailPtrAdd, tailPtr + 1.U, tailPtr)


/* 
 * Entrys Assignment
 */
  axiEntries.zipWithIndex.foreach {
    case(e, i) =>
      when(entryInit && (headPtr.value === i.U)){
        e.entryInit(io.uAxiAx.bits)
      }.elsewhen(entryMod && (tailPtr.value === i.U)) {
        e.exAddr  := (~e.byteMask & e.exAddr | (e.exAddr + (1.U((rni.offset).W) << e.size)) & e.byteMask)
        e.len     := e.len - 1.U
      }
  }
  dAxiAXWire               := 0.U.asTypeOf(dAxiAXWire)
  dAxiAXWire.addr          := Cat(tailE.addrPrefix, tailE.exAddr)
  dAxiAXWire.burst         := tailE.burst
  dAxiAXWire.cache         := tailE.cache
  dAxiAXWire.id            := tailE.id
  dAxiAXWire.len           := Mux(tailE.cache(1), tailE.len, 0.U)
  dAxiAXWire.qos           := tailE.qos
  dAxiAXWire.size          := tailE.size
  dAxiAXWire.user          := tailPtrAdd.asUInt

/* 
 * IO Connection
 */
  io.uAxiAx.ready      := !isFull(headPtr, tailPtr)
  io.dAxiAx.valid      := !isEmpty(headPtr, tailPtr)
  io.dAxiAx.bits       := dAxiAXWire

  io.working               := RegNext(headPtr =/= tailPtr)
  
/* 
 * Assertion
 */
  when(io.dAxiAx.fire && !io.dAxiAx.bits.cache(1)){
    assert(io.dAxiAx.bits.len === 0.U)
  }
  when(io.uAxiAx.fire) {
    assert(io.uAxiAx.bits.size <= 5.U)
  }
}