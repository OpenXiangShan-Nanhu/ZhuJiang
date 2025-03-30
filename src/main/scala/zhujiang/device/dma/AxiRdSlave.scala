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

class AxiRdSlave(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = rni.idBits)
  private val axiParamsUser = AxiParams(dataBits = dw, addrBits = raw, idBits = log2Ceil(rni.chiEntrySize), userBits = axiParams.idBits)
  require(axiParams.idBits >= log2Ceil(rni.chiEntrySize))

  private class CirQAxiEntryPtr extends CircularQueuePtr[CirQAxiEntryPtr](rni.axiEntrySize)
  private object CirQAxiEntryPtr {
  def apply(f: Bool, v: UInt): CirQAxiEntryPtr = {
        val ptr = Wire(new CirQAxiEntryPtr)
        ptr.flag := f
        ptr.value := v
        ptr
    }
  }
  private class CirQChiEntryPtr extends CircularQueuePtr[CirQChiEntryPtr](rni.chiEntrySize)
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
  private val uArEntrys = Reg(Vec(rni.axiEntrySize, new AxiRdEntry(isPipe = false)))
  private val uHeadPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uTailPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  private val dArEntrys = Reg(Vec(rni.chiEntrySize, new AxiRMstEntry))
  private val dHeadPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dTailPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))

  private val rxArPipe  = Module(new Queue(gen = new AxiRdEntry(isPipe = true), entries = 2, pipe = false, flow = false))
  private val dataCtrlQ = Module(new Queue(gen = new DataCtrl, entries = 4, pipe = true, flow = false))

  private val rxArBdl   = WireInit(0.U.asTypeOf(new AxiRdEntry(isPipe = true)))
  private val txArBdl   = WireInit(0.U.asTypeOf(io.dAxiAr.bits))
  
  private val uTailE    = uArEntrys(uTailPtr.value)
  private val dTailE    = dArEntrys(dTailPtr.value)


/* 
 * Pointer Logic
 */
  uHeadPtr   := Mux(rxArPipe.io.deq.fire, uHeadPtr + 1.U, uHeadPtr)
  uTailPtr   := Mux(io.dAxiAr.fire & ((uTailE.cnt.get + 1.U) === uTailE.num.get), uTailPtr + 1.U, uTailPtr)
  dHeadPtr   := Mux(io.dAxiAr.fire, dHeadPtr + 1.U, dHeadPtr)
  dTailPtr   := Mux(dTailE.txns === 0.U & dTailPtr =/= dHeadPtr, dTailPtr + 1.U, dTailPtr)

/* 
 * Assign Logic
 */
  uArEntrys.zipWithIndex.foreach{
    case(e, i) =>
      when(rxArPipe.io.deq.fire & uHeadPtr.value === i.U){
        e.entryInit(rxArPipe.io.deq.bits)
      }.elsewhen(io.dAxiAr.fire & uTailPtr.value === i.U){
        val incrNotModify  = !e.cache(1) &  Burst.isIncr(e.burst)
        val otherNotModify = !e.cache(1) & !Burst.isIncr(e.burst)
        val incrModify     =  e.cache(1) &  Burst.isIncr(e.burst)
        val otherModify    =  e.cache(1) & !Burst.isIncr(e.burst)
        e.cnt.get         := e.cnt.get + 1.U
        e.exAddr          := PriorityMux(Seq(
          incrNotModify   -> (e.exAddr + (1.U << e.size)),
          otherNotModify  -> (~e.byteMask & e.exAddr | (e.exAddr + (1.U((rni.offset + 1).W) << e.size)) & e.byteMask),
          incrModify      -> (Cat(e.exAddr(rni.pageBits - 1, rni.offset) + 1.U, 0.U(rni.offset.W))),
          otherModify     -> (Cat(e.exAddr(rni.pageBits - 1, rni.offset) + 1.U, 0.U(rni.offset.W)) & e.byteMask | ~e.byteMask & e.exAddr)
        ))
      }
  }
  dArEntrys.zipWithIndex.foreach{
    case(e, i) =>
      when(io.dAxiAr.fire & dHeadPtr.value === i.U) {
        e.id := uTailE.id
        e.size   := 1.U(log2Ceil(dw/8).W) << uTailE.size
        e.last   := (uTailE.cnt.get + 1.U) === uTailE.num.get
        e.shift  := uTailE.exAddr(rni.offset - 1, 0)
        //Compute the numbers of a transfer
        val canNotMerge = uTailE.cache(1).asBool
        val fixMerge    = (!uTailE.cache(1) & Burst.isFix(uTailE.burst)).asBool
        val wrapMerge   = (!uTailE.cache(1) & Burst.isWrap(uTailE.burst)).asBool
        val incrMerge   = (!uTailE.cache(1) & Burst.isIncr(uTailE.burst)).asBool
        val txnsIsOne   = canNotMerge || uTailE.size === 5.U
        val firWrpIncr  = (wrapMerge || incrMerge) & uTailE.cnt.get === 0.U
        val lastIncr    = incrMerge & (uTailE.cnt.get + 1.U === uTailE.num.get)
        e.txns          := PriorityMux(Seq(
          txnsIsOne    -> 1.U,
          fixMerge     -> uTailE.len,
          firWrpIncr   -> uTailE.firstTxn.get,
          lastIncr     -> uTailE.lastTxn.get,
          true.B       -> ("b100000".U >> uTailE.size)
        ))
      }.elsewhen(io.uAxiR.fire & dataCtrlQ.io.deq.bits.idx === i.U){
        e.shift  := e.shift + e.size
        e.txns   := e.txns - 1.U
      }
  }

  txArBdl        := 0.U.asTypeOf(txArBdl)
  txArBdl.addr   := Cat(uTailE.preAddr, uTailE.exAddr)
  txArBdl.cache  := uTailE.cache
  txArBdl.burst  := Burst.INCR
  txArBdl.user   := uTailE.id
  txArBdl.id     := dHeadPtr.value
  txArBdl.size   := Mux(!uTailE.cache(1) | Burst.isFix(uTailE.burst), uTailE.size , log2Ceil(dw/8).U)
  txArBdl.len    := Mux(!uTailE.cache(1) | Burst.isFix(uTailE.burst) | uTailE.exAddr(rni.offset - 1) |
                     uTailE.exAddr(rni.pageBits - 1, rni.offset - 1) === uTailE.endAddr(rni.pageBits - 1, rni.offset - 1), 0.U, 1.U)

  io.uAxiAr.ready     := rxArPipe.io.enq.ready
  io.uAxiR.bits       := 0.U.asTypeOf(io.uAxiR.bits)
  io.uAxiR.bits.data  := dataCtrlQ.io.deq.bits.data
  io.uAxiR.bits.id    := dataCtrlQ.io.deq.bits.id
  io.uAxiR.bits.resp  := dataCtrlQ.io.deq.bits.resp
  io.uAxiR.valid      := dataCtrlQ.io.deq.valid

  io.dAxiAr.bits      := txArBdl
  io.dAxiAr.valid     := uHeadPtr =/= uTailPtr & !isFull(dHeadPtr, dTailPtr)
  io.dAxiR.ready      := dataCtrlQ.io.enq.ready

  dataCtrlQ.io.enq.valid     := io.dAxiR.valid
  dataCtrlQ.io.enq.bits.id   := dArEntrys(io.dAxiR.bits.id).id
  dataCtrlQ.io.enq.bits.data := io.dAxiR.bits.data
  dataCtrlQ.io.enq.bits.idx  := io.dAxiR.bits.id
  dataCtrlQ.io.enq.bits.resp := io.dAxiR.bits.resp
  dataCtrlQ.io.deq.ready     := io.uAxiR.ready & (dArEntrys(dataCtrlQ.io.deq.bits.idx).txns === 1.U)
  
  rxArPipe.io.enq.valid   := io.uAxiAr.valid
  rxArPipe.io.enq.bits    := rxArBdl.pipeInit(io.uAxiAr.bits)
  rxArPipe.io.deq.ready   := !isFull(uHeadPtr, uTailPtr)
}
