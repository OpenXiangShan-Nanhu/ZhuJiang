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
import chiseltest.Region

class AxiRSpilt(implicit p : Parameters) extends ZJModule with HasCircularQueuePtrHelper{
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits)
  private val axiParamsUser = AxiParams(dataBits = dw, addrBits = raw, idBits = log2Ceil(dmaParams.chiEntrySize), userBits = axiParams.idBits)
  require(axiParams.idBits >= log2Ceil(dmaParams.chiEntrySize))

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
    val uAxiAr   = Flipped(Decoupled(new ARFlit(axiParams)))
    val uAxiR    = Decoupled(new RFlit(axiParams))
    val dAxiAr   = Decoupled(new ARFlit(axiParamsUser))
    val dAxiR    = Flipped(Decoupled(new RFlit(axiParamsUser)))
  })

/* 
 * Reg and Wire define
 */
  private val uArEntrys   = Reg(Vec(dmaParams.axiEntrySize, new AxiRdEntry(isPipe = false)))
  private val uHeadPtr    = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uTailPtr    = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  private val dArEntrys   = Reg(Vec(dmaParams.chiEntrySize, new dArEntry))
  private val dHeadPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dTailPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))

  private val rxArPipe    = Module(new Queue(gen = new AxiRdEntry(isPipe = true), entries = 2, pipe = false, flow = false))
  private val dataQ       = Module(new Queue(gen = new dataWithid, entries = 4, pipe = true, flow = false))
  private val sendCntsReg = RegInit(0.U(8.W))

  private val rxArBdl     = WireInit(0.U.asTypeOf(new AxiRdEntry(isPipe = true)))
  private val validVec    = WireInit(VecInit.fill(dmaParams.chiEntrySize){false.B})
  private val blockVec    = WireInit(VecInit.fill(dmaParams.chiEntrySize){false.B})
  private val uTailE      = uArEntrys(uTailPtr.value)
  private val dTailE      = dArEntrys(dTailPtr.value)
  private val txArBdl     = WireInit(0.U.asTypeOf(io.dAxiAr.bits))


/* 
 * Pointer Logic
 */
  uHeadPtr      := Mux(rxArPipe.io.deq.fire, uHeadPtr + 1.U, uHeadPtr)
  uTailPtr      := Mux(io.dAxiAr.fire & ((uArEntrys(uTailPtr.value).cnt.get + 1.U) === uArEntrys(uTailPtr.value).num.get), uTailPtr + 1.U, uTailPtr)
  dHeadPtr      := Mux(io.dAxiAr.fire, dHeadPtr + 1.U, dHeadPtr)
  dTailPtr      := Mux(dArEntrys(dTailPtr.value).num === 0.U & dTailPtr =/= dHeadPtr, dTailPtr + 1.U, dTailPtr)
  

  uArEntrys.zipWithIndex.foreach {
    case(e, i) => 
      when(rxArPipe.io.deq.fire & uHeadPtr.value === i.U){
        e.entryInit(rxArPipe.io.deq.bits)
      }.elsewhen(io.dAxiAr.fire & uTailPtr.value === i.U){
        e.cnt.get   := e.cnt.get + 1.U
        e.shiftAddr := Mux(e.burst === BurstMode.Incr, Cat(uTailE.shiftAddr(11, 6) + 1.U, 0.U(6.W)), 
                        ~e.byteMask & e.shiftAddr | (e.shiftAddr + (1.U(7.W) << e.size)) & e.byteMask)
      }
  }
  dArEntrys.zipWithIndex.foreach {
    case(e, i) =>
      when(io.dAxiAr.fire & dHeadPtr.value === i.U){
        e.last  := (uTailE.cnt.get + 1.U) === uTailE.num.get
        e.size  := 1.U(7.W) << uTailE.size
        e.id    := uTailE.id
        e.shift := uTailE.shiftAddr(5, 0)
        e.num   := Mux(uTailE.burst =/= BurstMode.Incr, 1.U, Mux(uTailE.shiftAddr(11, 6) === uTailE.endAddr(11 ,6), (uTailE.endAddr(5, 0) - uTailE.shiftAddr(5, 0)) >> uTailE.size, ("b1000000".U - uTailE.shiftAddr(5, 0) >> uTailE.size)))
        e.error := uTailE.error.get
      }.elsewhen(io.uAxiR.fire & dataQ.io.deq.bits.entry === i.U){
        e.shift := e.shift + e.size
        e.num   := e.num - 1.U
      }
  }
  for(idx <- validVec.indices){
    when(dHeadPtr.flag === dTailPtr.flag){
      validVec(idx) := Mux((dTailPtr.value <= idx.U) & (dHeadPtr.value > idx.U), true.B, false.B)
    }.otherwise {
      validVec(idx) := Mux((idx.U < dHeadPtr.value) & (idx.U >= dTailPtr.value), true.B, false.B)
    }
  }
  blockVec := validVec.zip(dArEntrys.map(_.id === uTailE.id)).map{case(i, j) => i & j}

  txArBdl        := 0.U.asTypeOf(txArBdl)
  txArBdl.addr   := Mux(uTailE.prefixAddr(raw - 12 - 1) | uTailE.burst =/= BurstMode.Incr, Cat(uTailE.prefixAddr, uTailE.shiftAddr), Cat(uTailE.prefixAddr, uTailE.shiftAddr(11, 5), 0.U(5.W)))
  txArBdl.id     := dHeadPtr.value
  txArBdl.user   := uTailE.id
  txArBdl.burst  := BurstMode.Incr
  txArBdl.size   := Mux(uTailE.prefixAddr(raw - 12 - 1) | uTailE.burst =/= BurstMode.Incr, uTailE.size, log2Ceil(dw/8).U)
  txArBdl.len    := Mux(uTailE.burst =/= BurstMode.Incr | uTailE.shiftAddr(5) | 
                      uTailE.endAddr(11, 6) === uTailE.shiftAddr(11, 6) & uTailE.endAddr(5, 0) <= "b100000".U, 0.U, 1.U)


/* 
 * IO Connection
 */
  io.uAxiAr.ready       := rxArPipe.io.enq.ready
  io.uAxiR.bits         := 0.U.asTypeOf(io.dAxiR.bits)
  io.uAxiR.bits.data    := dataQ.io.deq.bits.data
  io.uAxiR.bits.id      := dataQ.io.deq.bits.id
  io.uAxiR.bits.last    := dArEntrys(dataQ.io.deq.bits.entry).last & dArEntrys(dataQ.io.deq.bits.entry).num === 1.U
  io.uAxiR.valid        := dataQ.io.deq.valid
  io.dAxiAr.valid       := uHeadPtr =/= uTailPtr & !isFull(dHeadPtr, dTailPtr)
  io.dAxiAr.bits        := txArBdl
  io.dAxiR.ready        := dataQ.io.enq.ready

  rxArPipe.io.enq.bits  := rxArBdl.pipeInit(io.uAxiAr.bits)
  rxArPipe.io.enq.valid := io.uAxiAr.valid
  rxArPipe.io.deq.ready := !isFull(uHeadPtr, uTailPtr)

  dataQ.io.enq.valid       := io.dAxiR.valid
  dataQ.io.enq.bits.data   := io.dAxiR.bits.data
  dataQ.io.enq.bits.id     := dArEntrys(io.dAxiR.bits.id).id
  dataQ.io.enq.bits.entry  := io.dAxiR.bits.id
  dataQ.io.deq.ready       := ((dArEntrys(dataQ.io.deq.bits.entry).shift + dArEntrys(dataQ.io.deq.bits.entry).size)(4, 0) === 0.U | 
                                (dArEntrys(dataQ.io.deq.bits.entry).num === 1.U)) & io.uAxiR.ready
}
