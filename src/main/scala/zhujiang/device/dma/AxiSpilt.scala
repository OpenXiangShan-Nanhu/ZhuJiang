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

class AxiSpilt(implicit p : Parameters) extends ZJModule with HasCircularQueuePtrHelper{
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
  val rxAxi = IO(Flipped(new AxiBundle(axiParams)))
  val txAxi = IO(new AxiBundle(axiParams))

  rxAxi := DontCare
  txAxi := DontCare

/*
 * Reg and Wire define 
 */
  private val uArEntrys   = Reg(Vec(dmaParams.axiSpiltSize, new AxiRdEntry))
  private val uArHeadPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uArTailPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  private val spiltEntrys   = Reg(Vec(dmaParams.chiEntrySize, new SpiltValue))
  private val dArHeadPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dArTailPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  
  private val txArBdl     = WireInit(0.U.asTypeOf(new ARFlit(axiParams)))
  private val lenQueue    = Module(new Queue(gen = new lenValue, entries = dmaParams.chiEntrySize, pipe = true, flow = false))
  private val datQueue    = Module(new Queue(gen = new DataValue, entries = 2, pipe = true, flow = false))
  private val sendCnts    = RegInit(0.U(8.W))

/* 
 * Control Logic
 */
// uArHeadPtr add logic
  when(rxAxi.ar.fire){
    uArEntrys(uArHeadPtr.value).assignArVal(rxAxi.ar.bits)
    uArHeadPtr := uArHeadPtr + 1.U
  }
  lenQueue.io.enq.valid      := Mux(rxAxi.ar.fire, true.B, false.B)
  lenQueue.io.enq.bits.len   := rxAxi.ar.bits.len
  lenQueue.io.enq.bits.burst := rxAxi.ar.bits.burst

  private val uTailE = WireInit(uArEntrys(uArTailPtr.value))
  private val singleIncr = WireInit(uTailE.cnt + (1.U << 5.U >> uTailE.size) >= uTailE.len)
  private val doubleIncr = WireInit(uTailE.cnt + (1.U << 6.U >> uTailE.size) >= uTailE.len)
  
// Send the convert axi req
  txArBdl        := 0.U.asTypeOf(txArBdl)
  txArBdl.addr   := Mux(uTailE.prefixAddr(35) | uTailE.burst =/= BurstMode.Incr, Cat(uTailE.prefixAddr, uTailE.shiftAddr), Cat(uTailE.prefixAddr, uTailE.shiftAddr(11, 5), 0.U(5.W)))
  txArBdl.id     := uTailE.arId
  txArBdl.burst  := BurstMode.Incr
  txArBdl.size   := Mux(uTailE.prefixAddr(35) | uTailE.burst =/= BurstMode.Incr, uTailE.size, log2Ceil(dw/8).U)
  txArBdl.len    := Mux(uTailE.shiftAddr(5) | uTailE.burst =/= BurstMode.Incr | singleIncr, 0.U, 1.U)

// Update the value of uArEntry
  when(txAxi.ar.fire){
    spiltEntrys(dArHeadPtr.value).size    := 1.U << uTailE.size
    spiltEntrys(dArHeadPtr.value).shift   := uTailE.shiftAddr(5, 0)
    uArEntrys(uArTailPtr.value).shiftAddr := Mux(uTailE.burst === BurstMode.Incr, Cat((uTailE.shiftAddr(11,6) + 1.U), 0.U(6.W)), 
                                                Mux(uTailE.burst === BurstMode.Wrap, (uTailE.shiftAddr + (1.U << uTailE.size)) & uTailE.addrMask | ~(~uTailE.shiftAddr | uTailE.addrMask),
                                                     uTailE.shiftAddr))
    uArEntrys(uArTailPtr.value).cnt       := Mux(uTailE.burst === BurstMode.Incr, Mux(uTailE.shiftAddr(5, 0).orR, uTailE.cnt + ((0.U(6.W) - uTailE.shiftAddr(5, 0)) >> uTailE.size), uTailE.cnt + (1.U << 6.U >> uTailE.size)), uTailE.cnt + 1.U)
    dArHeadPtr                            := dArHeadPtr + 1.U
  }

  when(txAxi.ar.fire & (uTailE.burst === BurstMode.Incr & (txAxi.ar.bits.len === 0.U  & singleIncr | txAxi.ar.bits.len === 1.U && doubleIncr) | uTailE.burst =/= BurstMode.Incr & uTailE.cnt + 1.U === uTailE.len)){
    uArTailPtr := uArTailPtr + 1.U
  }

  private val dTailE    = WireInit(spiltEntrys(dArTailPtr.value))

  when((dTailE.shift + dTailE.size) === 0.U & rxAxi.r.fire){
    dArTailPtr := dArTailPtr + 1.U
  }
  when(rxAxi.r.fire){
    spiltEntrys(dArTailPtr.value).shift := dTailE.shift + dTailE.size
  }

// datQueue receive data and send data logic
  datQueue.io.enq.bits.data := txAxi.r.bits.data
  datQueue.io.enq.bits.rid  := txAxi.r.bits.id
  datQueue.io.enq.valid     := txAxi.r.valid

  datQueue.io.deq.ready     := (((dTailE.shift + dTailE.size)(4, 0).orR) === false.B | lenQueue.io.deq.bits.len === sendCnts) & rxAxi.r.fire & lenQueue.io.deq.bits.burst === BurstMode.Incr | lenQueue.io.deq.bits.burst =/= BurstMode.Incr
  lenQueue.io.deq.ready     := lenQueue.io.deq.bits.len === sendCnts & rxAxi.r.fire
  sendCnts                  := Mux(rxAxi.r.fire & lenQueue.io.deq.bits.len =/= sendCnts, sendCnts + 1.U, Mux(rxAxi.r.fire & lenQueue.io.deq.bits.len === sendCnts, 0.U, sendCnts))


/* 
 * IO Interface Connection
 */
  rxAxi.ar.ready  := !isFull(uArHeadPtr, uArTailPtr) & lenQueue.io.enq.ready
  txAxi.ar.valid  := uArHeadPtr =/= uArTailPtr & !isFull(dArHeadPtr, dArTailPtr)
  txAxi.ar.bits   := txArBdl
  txAxi.r.ready   := datQueue.io.enq.ready
  rxAxi.r.bits.data := datQueue.io.deq.bits.data
  rxAxi.r.bits.id   := datQueue.io.deq.bits.rid
  rxAxi.r.bits.last := sendCnts === lenQueue.io.deq.bits.len
  rxAxi.r.valid     := datQueue.io.deq.valid

/* 
 * Assertion
 */
  when(rxAxi.ar.fire){
    assert(rxAxi.ar.bits.size <= 5.U, "AxiMaster error!")
  }
  when(rxAxi.ar.fire & rxAxi.ar.bits.burst === BurstMode.Wrap){
    assert(PopCount(rxAxi.ar.bits.len + 1.U) === 1.U, "AxiMaster error!")
  }
}