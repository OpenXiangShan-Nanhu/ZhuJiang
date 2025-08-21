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


class AxiDevSpilt(node: Node, write: Boolean, read: Boolean)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  require(write ^ read)
  private val rni           = DmaParams(node = node)
  private val axiParams     = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamsLast = AxiParams(addrBits = axiParams.addrBits, idBits = axiParams.idBits, userBits = 1, dataBits = axiParams.dataBits,
    attr = axiParams.attr, lenBits = axiParams.lenBits, qosBits = axiParams.qosBits, regionBits = axiParams.regionBits)

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
    val uAxiAr   = if(read)  Some(Flipped(Decoupled(new ARFlit(axiParams)))) else None
    val uAxiAw   = if(write) Some(Flipped(Decoupled(new AWFlit(axiParams)))) else None
    val dAxiAr   = if(read)  Some(Decoupled(new ARFlit(axiParamsLast)))      else None
    val dAxiAw   = if(write) Some(Decoupled(new ARFlit(axiParamsLast)))      else None
    val working  = Output(Bool())
  })

/* 
 * Reg and Pointer declare
 */
  private val axiEntries = Reg(Vec(rni.axiEntrySize, new AxiOriEntry(node = node)))
  private val headPtr    = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val tailPtr    = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  //Wire declare
  private val dAxiAXWire  = if(read) WireInit(0.U.asTypeOf(io.dAxiAr.get.bits)) else WireInit(0.U.asTypeOf(io.dAxiAw.get.bits))
  private val uAxiAX      = if(read) Some(io.uAxiAr.get.bits)                   else Some(io.uAxiAw.get.bits)
  private val entryInit   = if(read) Some(io.uAxiAr.get.fire)                   else Some(io.uAxiAw.get.fire)
  private val entryMod    = if(read) Some(io.dAxiAr.get.fire)                   else Some(io.dAxiAw.get.fire)

  // pointer movement logic 
  private val tailE       = axiEntries(tailPtr.value)
  private val headPtrAdd  = if(write) io.uAxiAw.get.fire else io.uAxiAr.get.fire
  private val tailPtrAdd  = if(write) io.dAxiAw.get.fire & (axiEntries(tailPtr.value).len === 0.U || axiEntries(tailPtr.value).cache(1)) else 
    io.dAxiAr.get.fire & (axiEntries(tailPtr.value).len === 0.U || axiEntries(tailPtr.value).cache(1))

  headPtr                := Mux(headPtrAdd, headPtr + 1.U, headPtr)
  tailPtr                := Mux(tailPtrAdd, tailPtr + 1.U, tailPtr)


/* 
 * Entrys Assignment
 */
  axiEntries.zipWithIndex.foreach {
    case(e, i) =>
      when(entryInit.get && (headPtr.value === i.U)){
        e.entryInit(uAxiAX.get)
      }.elsewhen(entryMod.get && (tailPtr.value === i.U)) {
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
  if(read) {
    io.uAxiAr.get.ready      := !isFull(headPtr, tailPtr)
    io.dAxiAr.get.valid      := !isEmpty(headPtr, tailPtr)
    io.dAxiAr.get.bits       := dAxiAXWire
  }
  if(write) {
    io.uAxiAw.get.ready      := !isFull(headPtr, tailPtr)
    io.dAxiAw.get.valid      := !isEmpty(headPtr, tailPtr)
    io.dAxiAw.get.bits       := dAxiAXWire
  }

  io.working               := headPtr =/= tailPtr
  
/* 
 * Assertion
 */
  if(read) {
    when(io.dAxiAr.get.fire && !io.dAxiAr.get.bits.cache(1)){
      assert(io.dAxiAr.get.bits.len === 0.U)
    }
  }
  if(write) {
    when(io.dAxiAw.get.fire && !io.dAxiAw.get.bits.cache(1)){
      assert(io.dAxiAw.get.bits.len === 0.U)
    }
  }
}