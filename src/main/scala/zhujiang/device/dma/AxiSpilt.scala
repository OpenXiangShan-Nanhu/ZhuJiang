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

/*
 * Reg and Wire define 
 */
// Read module
  private val uArEntrys   = Reg(Vec(dmaParams.axiSpiltSize, new AxiRdEntry))
  private val uArHeadPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uArTailPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  private val spiltArEntrys = Reg(Vec(dmaParams.chiEntrySize, new SpiltArValue))
  private val dArHeadPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dArTailPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  
  private val txArBdl     = WireInit(0.U.asTypeOf(new ARFlit(axiParams)))
  private val lenQueue    = Module(new Queue(gen = new lenValue, entries = dmaParams.chiEntrySize, pipe = true, flow = false))
  private val datQueue    = Module(new Queue(gen = new DataValue, entries = 2, pipe = true, flow = false))
  private val sendCnts    = RegInit(0.U(8.W))

// Write module
  private val uAwEntrys   = Reg(Vec(dmaParams.axiSpiltSize, new AxiWrEntry))
  private val uAwHeadPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uAwTailPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  private val spiltAwEntrys = Reg(Vec(dmaParams.chiEntrySize, new SpiltAwValue))
  private val dAwHeadPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dAwTailPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val wDataPtr      = RegInit(0.U(log2Ceil(dmaParams.chiEntrySize).W))

  private val txAwBdl     = WireInit(0.U.asTypeOf(new AWFlit(axiParams)))
  private val wrDataReg   = RegInit(0.U(dw.W))
  private val wrMaskReg   = RegInit(0.U(bew.W))
  private val maskData    = WireInit(VecInit.fill(dw/8){0.U(8.W)})

// maskData logic
  maskData.zip(rxAxi.w.bits.strb.asBools).foreach{
    case(m, b) =>
      when(b === 1.U){
        m := 255.U
      }.otherwise{
        m := 0.U
      }
  }

/* 
 * Control Logic
 */
// uArHeadPtr add logic
  when(rxAxi.ar.fire){
    uArEntrys(uArHeadPtr.value).assignArVal(rxAxi.ar.bits)
    uArHeadPtr := uArHeadPtr + 1.U
  }
//uAwHeadPtr add logic
  when(rxAxi.aw.fire){
    uAwEntrys(uAwHeadPtr.value).assignAwVal(rxAxi.aw.bits)
    uAwHeadPtr := uAwHeadPtr + 1.U
  }
  lenQueue.io.enq.valid      := Mux(rxAxi.ar.fire, true.B, false.B)
  lenQueue.io.enq.bits.len   := rxAxi.ar.bits.len
  lenQueue.io.enq.bits.burst := rxAxi.ar.bits.burst

//Read wire declare
  private val uArTailE   = WireInit(uArEntrys(uArTailPtr.value))
  private val singleIncr = WireInit(uArTailE.cnt + (1.U << 5.U >> uArTailE.size) >= uArTailE.len)
  private val doubleIncr = WireInit(uArTailE.cnt + (1.U << 6.U >> uArTailE.size) >= uArTailE.len)
//Write wire declare
  private val uAwTailE   = WireInit(uAwEntrys(uAwTailPtr.value))
  private val singleAdd  = WireInit(uAwTailE.cnt + (1.U << 5.U >> uAwTailE.size) >= uAwTailE.len)
  private val doubleAdd  = WireInit(uAwTailE.cnt + (1.U << 6.U >> uAwTailE.size) >= uAwTailE.len)


// Send the convert axi Ar req
  txArBdl        := 0.U.asTypeOf(txArBdl)
  txArBdl.addr   := Mux(uArTailE.prefixAddr(35) | uArTailE.burst =/= BurstMode.Incr, Cat(uArTailE.prefixAddr, uArTailE.shiftAddr), Cat(uArTailE.prefixAddr, uArTailE.shiftAddr(11, 5), 0.U(5.W)))
  txArBdl.id     := uArTailE.arId
  txArBdl.burst  := BurstMode.Incr
  txArBdl.size   := Mux(uArTailE.prefixAddr(35) | uArTailE.burst =/= BurstMode.Incr, uArTailE.size, log2Ceil(dw/8).U)
  txArBdl.len    := Mux(uArTailE.shiftAddr(5) | uArTailE.burst =/= BurstMode.Incr | singleIncr, 0.U, 1.U)
// Send the convert axi Aw req
  txAwBdl        := 0.U.asTypeOf(txAwBdl)
  txAwBdl.addr   := Mux(uAwTailE.prefixAddr(35) | uAwTailE.burst =/= BurstMode.Incr, Cat(uAwTailE.prefixAddr, uAwTailE.shiftAddr), Cat(uAwTailE.prefixAddr, uAwTailE.shiftAddr(11, 5), 0.U(5.W)))
  txAwBdl.id     := dAwHeadPtr.value
  txAwBdl.burst  := BurstMode.Incr
  txAwBdl.size   := Mux(uAwTailE.prefixAddr(35) | uAwTailE.burst =/= BurstMode.Incr, uAwTailE.size, log2Ceil(dw/8).U)
  txAwBdl.len    := Mux(uAwTailE.shiftAddr(5) | uAwTailE.burst =/= BurstMode.Incr | singleAdd, 0.U, 1.U)


// Update the value of uArEntry and spiltArEntrys
  when(txAxi.ar.fire){
    spiltArEntrys(dArHeadPtr.value).size    := 1.U << uArTailE.size
    spiltArEntrys(dArHeadPtr.value).shift   := uArTailE.shiftAddr(5, 0)
    uArEntrys(uArTailPtr.value).shiftAddr := Mux(uArTailE.burst === BurstMode.Incr, Cat((uArTailE.shiftAddr(11,6) + 1.U), 0.U(6.W)), 
                                                Mux(uArTailE.burst === BurstMode.Wrap, (uArTailE.shiftAddr + (1.U << uArTailE.size)) & uArTailE.addrMask | ~(~uArTailE.shiftAddr | uArTailE.addrMask),
                                                     uArTailE.shiftAddr))
    uArEntrys(uArTailPtr.value).cnt       := Mux(uArTailE.burst === BurstMode.Incr, Mux(uArTailE.shiftAddr(5, 0).orR, uArTailE.cnt + ((0.U(6.W) - uArTailE.shiftAddr(5, 0)) >> uArTailE.size), uArTailE.cnt + (1.U << 6.U >> uArTailE.size)), uArTailE.cnt + 1.U)
    dArHeadPtr                            := dArHeadPtr + 1.U
  }
// Update the value of uAwEntry and spiltAwEntrys
  when(txAxi.aw.fire){
    spiltAwEntrys(dAwHeadPtr.value).size   := 1.U << uAwTailE.size
    spiltAwEntrys(dAwHeadPtr.value).shift  := uAwTailE.shiftAddr(5, 0)
    spiltAwEntrys(dAwHeadPtr.value).burst  := uAwTailE.burst
    spiltAwEntrys(dAwHeadPtr.value).id     := uAwTailE.awid
    spiltAwEntrys(dAwHeadPtr.value).last   := false.B
    uAwEntrys(uAwTailPtr.value).shiftAddr  := Mux(uAwTailE.burst === BurstMode.Incr, Cat((uAwTailE.shiftAddr(11,6) + 1.U), 0.U(6.W)), 
                                                Mux(uAwTailE.burst === BurstMode.Wrap, (uAwTailE.shiftAddr + (1.U << uAwTailE.size)) & uAwTailE.byteMask | ~(~uAwTailE.shiftAddr | uAwTailE.byteMask),
                                                    uAwTailE.shiftAddr))
    uAwEntrys(uAwTailPtr.value).cnt        := Mux(uAwTailE.burst === BurstMode.Incr, Mux(uAwTailE.shiftAddr(5, 0).orR, uAwTailE.cnt + ((0.U(6.W) - uAwTailE.shiftAddr(5, 0)) >> uAwTailE.size), uAwTailE.cnt + (1.U << 6.U >> uAwTailE.size)), uAwTailE.cnt + 1.U)
    dAwHeadPtr                             := dAwHeadPtr + 1.U
  }


// uArTailPtr add logic
  when(txAxi.ar.fire & (uArTailE.burst === BurstMode.Incr & (txAxi.ar.bits.len === 0.U & singleIncr | txAxi.ar.bits.len === 1.U & doubleIncr) | uArTailE.burst =/= BurstMode.Incr & uArTailE.cnt + 1.U === uArTailE.len)){
    uArTailPtr := uArTailPtr + 1.U
  }
// uAwTailPtr add logic
  when(txAxi.aw.fire & (uAwTailE.burst === BurstMode.Incr & (txAxi.aw.bits.len === 0.U & singleAdd | txAxi.aw.bits.len === 1.U & doubleAdd) | uAwTailE.burst =/= BurstMode.Incr & uAwTailE.cnt + 1.U === uAwTailE.len)){
    uAwTailPtr := uAwTailPtr + 1.U
    spiltAwEntrys(dAwHeadPtr.value).last := true.B
  }

  private val dArTailE    = WireInit(spiltArEntrys(dArTailPtr.value))
// Read dArTailPtr add logic
  when(rxAxi.r.fire & ((dArTailE.shift + dArTailE.size) === 0.U & lenQueue.io.deq.bits.burst === BurstMode.Incr | lenQueue.io.deq.bits.burst =/= BurstMode.Incr | lenQueue.io.deq.fire)){
    dArTailPtr := dArTailPtr + 1.U
  }
  when(rxAxi.r.fire){
    spiltArEntrys(dArTailPtr.value).shift := dArTailE.shift + dArTailE.size
  }
// Write SpiltAwEntrys shift
  when(rxAxi.w.fire){
    wDataPtr := Mux(rxAxi.w.bits.last | ((spiltAwEntrys(wDataPtr).shift + spiltAwEntrys(wDataPtr).size) === 0.U), wDataPtr + 1.U, wDataPtr)
    spiltAwEntrys(wDataPtr).shift := spiltAwEntrys(wDataPtr).shift + spiltAwEntrys(wDataPtr).size
  }

// datQueue receive data and send data logic
  datQueue.io.enq.bits.data := txAxi.r.bits.data
  datQueue.io.enq.bits.rid  := txAxi.r.bits.id
  datQueue.io.enq.valid     := txAxi.r.valid

  datQueue.io.deq.ready     := (((dArTailE.shift + dArTailE.size)(4, 0).orR) === false.B | lenQueue.io.deq.bits.len === sendCnts) & rxAxi.r.fire & lenQueue.io.deq.bits.burst === BurstMode.Incr | lenQueue.io.deq.bits.burst =/= BurstMode.Incr
  lenQueue.io.deq.ready     := lenQueue.io.deq.bits.len === sendCnts & rxAxi.r.fire
  sendCnts                  := Mux(rxAxi.r.fire & lenQueue.io.deq.bits.len =/= sendCnts, sendCnts + 1.U, Mux(rxAxi.r.fire & lenQueue.io.deq.bits.len === sendCnts, 0.U, sendCnts))

// B Resp cause dAwTailPtr to increase
  when(txAxi.b.fire & rxAxi.b.ready){
    dAwTailPtr    := dAwTailPtr + 1.U
  }

// Merge logic for writing data
    wrMaskReg := Mux(txAxi.w.fire, Mux(rxAxi.w.fire, rxAxi.w.bits.strb, 0.U), Mux(rxAxi.w.fire, rxAxi.w.bits.strb | wrMaskReg, wrMaskReg))
    wrDataReg := Mux(txAxi.w.fire, Mux(rxAxi.w.fire, maskData.asUInt & rxAxi.w.bits.data, 0.U), Mux(rxAxi.w.fire, wrDataReg | maskData.asUInt & rxAxi.w.bits.data, wrDataReg))

/* 
 * IO Interface Connection
 */
// Read interface
  rxAxi.ar.ready  := !isFull(uArHeadPtr, uArTailPtr) & lenQueue.io.enq.ready
  txAxi.ar.valid  := uArHeadPtr =/= uArTailPtr
  txAxi.ar.bits   := txArBdl
  txAxi.r.ready   := datQueue.io.enq.ready
  rxAxi.r.bits      := 0.U.asTypeOf(rxAxi.r.bits)
  rxAxi.r.bits.data := datQueue.io.deq.bits.data
  rxAxi.r.bits.id   := datQueue.io.deq.bits.rid
  rxAxi.r.bits.last := sendCnts === lenQueue.io.deq.bits.len
  rxAxi.r.valid     := datQueue.io.deq.valid
// Write interface
  rxAxi.aw.ready    := !isFull(uAwHeadPtr, uAwTailPtr)
  rxAxi.w.ready     := dAwHeadPtr =/= dAwTailPtr & wDataPtr =/= dAwHeadPtr.value
  txAxi.aw.valid    := uAwHeadPtr =/= uAwTailPtr & !isFull(dAwHeadPtr, dAwTailPtr)
  txAxi.aw.bits     := txAwBdl
  txAxi.w.valid     := (RegNext(!(spiltAwEntrys(wDataPtr).shift + spiltAwEntrys(wDataPtr).size)(4, 0).orR) | RegNext(rxAxi.w.bits.last)) & RegNext(rxAxi.w.fire)
  txAxi.w.bits      := 0.U.asTypeOf(txAxi.w.bits)
  txAxi.w.bits.data := wrDataReg
  txAxi.w.bits.strb := wrMaskReg
  txAxi.w.bits.last := RegNext(!(spiltAwEntrys(wDataPtr).shift + spiltAwEntrys(wDataPtr).size)(5, 0).orR) | RegNext(rxAxi.w.bits.last)
  txAxi.b.ready     := rxAxi.b.ready
  rxAxi.b.bits      := 0.U.asTypeOf(rxAxi.b.bits)
  rxAxi.b.bits.id   := spiltAwEntrys(dAwTailPtr.value).id
  rxAxi.b.valid     := spiltAwEntrys(dAwTailPtr.value).last & txAxi.b.valid

/* 
 * Assertion
 */
  when(rxAxi.ar.fire){
    assert(rxAxi.ar.bits.size <= 5.U, "AxiMaster error!")
  }
  when(rxAxi.aw.fire){
    assert(rxAxi.aw.bits.size <= 5.U, "AxiMaster error!")
  }
  when(rxAxi.aw.fire & rxAxi.aw.bits.addr(raw - 1)){
    assert(rxAxi.aw.bits.size <= 3.U, "AxiMaster error!")
  }
  when(rxAxi.ar.fire & rxAxi.ar.bits.addr(raw - 1)){
    assert(rxAxi.ar.bits.size <= 3.U, "AxiMaster error!")
  }
  when(rxAxi.ar.fire & rxAxi.ar.bits.burst === BurstMode.Wrap){
    assert(PopCount(rxAxi.ar.bits.len + 1.U) === 1.U, "AxiMaster error!")
  }
  when(rxAxi.aw.fire & rxAxi.aw.bits.burst === BurstMode.Wrap){
    assert(PopCount(rxAxi.aw.bits.len + 1.U) === 1.U, "AxiMaster error!")
  }
  when(txAxi.ar.fire){
    assert(txAxi.ar.bits.len === 0.U | txAxi.ar.bits.len === 1.U, "Spilt logic is error!")
  }
  when(txAxi.aw.fire){
    assert(txAxi.aw.bits.len === 0.U | txAxi.aw.bits.len === 1.U, "Spilt logic is error!")
  }
}