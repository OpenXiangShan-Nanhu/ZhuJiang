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

class AxiRdSlave(node: Node)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni = DmaParams(node = node)
  private val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamsUser = AxiParams(addrBits = axiParams.addrBits, idBits = log2Ceil(node.outstanding), userBits = axiParams.idBits, dataBits = axiParams.dataBits,
    attr = axiParams.attr, lenBits = axiParams.lenBits, qosBits = axiParams.qosBits, regionBits = axiParams.regionBits)
  require(axiParams.idBits >= log2Ceil(node.outstanding))

  private class CirQAxiEntryPtr extends CircularQueuePtr[CirQAxiEntryPtr](rni.axiEntrySize)
  private object CirQAxiEntryPtr {
  def apply(f: Bool, v: UInt): CirQAxiEntryPtr = {
        val ptr = Wire(new CirQAxiEntryPtr)
        ptr.flag := f
        ptr.value := v
        ptr
    }
  }
  private class CirQChiEntryPtr extends CircularQueuePtr[CirQChiEntryPtr](node.outstanding)
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
    val working  = Output(Bool())
  })

/* 
 * Reg and Wire define
 */
  private val uArEntrys = Reg(Vec(rni.axiEntrySize, new AxiRdEntry(isPipe = false, node = node)))
  private val uHeadPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uTailPtr  = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  private val dArEntrys = Reg(Vec(node.outstanding, new AxiRMstEntry(node)))
  private val dHeadPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dTailPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))

  private val rxArPipe  = Module(new Queue(gen = new AxiRdEntry(isPipe = true, node = node), entries = 2, pipe = false, flow = false))
  private val dataCtrlQ = Module(new SendReg(node))

  private val rxArBdl   = WireInit(0.U.asTypeOf(new AxiRdEntry(isPipe = true, node = node)))
  private val txArBdl   = WireInit(0.U.asTypeOf(io.dAxiAr.bits))
  
  private val uTailE    = uArEntrys(uTailPtr.value)
  private val dTailE    = dArEntrys(dTailPtr.value)

/* 
 * Pointer Logic
 */
  private val uTailPtrAdd   = io.dAxiAr.fire & ((uTailE.cnt.get + 1.U) === uTailE.num.get)
  private val dTailPtrAdd   = dTailPtr =/= dHeadPtr & dArEntrys(dTailPtr.value).finish

  uHeadPtr   := Mux(rxArPipe.io.deq.fire, uHeadPtr + 1.U, uHeadPtr)
  uTailPtr   := Mux(uTailPtrAdd         , uTailPtr + 1.U, uTailPtr)
  dHeadPtr   := Mux(io.dAxiAr.fire      , dHeadPtr + 1.U, dHeadPtr)
  dTailPtr   := Mux(dTailPtrAdd         , dTailPtr + 1.U, dTailPtr)

/* 
 * Assign Logic
 */
  uArEntrys.zipWithIndex.foreach{
    case(e, i) =>
      when(rxArPipe.io.deq.fire & uHeadPtr.value === i.U){
        e.entryInit(rxArPipe.io.deq.bits)
      }.elsewhen(io.dAxiAr.fire & uTailPtr.value === i.U){
        val notModify      = !e.cache(1)
        val modify         =  e.cache(1) &  Burst.isIncr(e.burst)
        e.cnt.get         := e.cnt.get + 1.U
        e.exAddr          := PriorityMux(Seq(
          notModify       -> (~e.byteMask & e.exAddr | (e.exAddr + (1.U((rni.offset).W) << e.size)) & e.byteMask),
          modify          -> (Cat(e.exAddr(rni.pageBits - 1, rni.offset) + 1.U, 0.U(rni.offset.W)) & e.byteMask | ~e.byteMask & e.exAddr)
        ))
      }
  }
  dArEntrys.zipWithIndex.foreach{
    case(e, i) =>
      when(io.dAxiAr.fire & dHeadPtr.value === i.U) {
        val nextAddr  = (uTailE.exAddr(rni.offset - 1, 0) + (1.U(rni.offset.W) << uTailE.size)) & uTailE.byteMask(rni.offset - 1, 0) | uTailE.exAddr(rni.offset - 1, 0) & ~uTailE.byteMask(rni.offset - 1, 0)
        e.id         := uTailE.id
        e.size       := 1.U(log2Ceil(dw/8).W) << uTailE.size
        e.last       := (uTailE.cnt.get + 1.U) === uTailE.num.get
        e.shift      := uTailE.exAddr(rni.offset - 1, 0)
        e.byteMask   := uTailE.byteMask(rni.offset - 1, 0)
        e.nextShift  := nextAddr
        e.beat       := uTailE.exAddr(rni.offset - 1)
        e.finish     := false.B
        val notModify  = !uTailE.cache(1)
        val lastEntry  = (uTailE.cnt.get + 1.U) === uTailE.num.get
        val specWrap   = Burst.isWrap(uTailE.burst) & !uTailE.byteMask(rni.offset)
        val fixMerge   = Burst.isFix(uTailE.burst)
        e.endShift    := Mux(!uTailE.cache(1), nextAddr, Mux((uTailE.cnt.get + 1.U) === uTailE.num.get, uTailE.endAddr(rni.offset - 1, 0), Mux(Burst.isWrap(uTailE.burst), uTailE.exAddr(rni.offset - 1, 0), 0.U)))
        e.endShift    := PriorityMux(Seq(
          notModify   -> nextAddr,
          specWrap    -> uTailE.exAddr(rni.offset - 1, 0),
          fixMerge    -> (Cat(uTailE.exAddr) + uTailE.len),
          true.B      -> Mux((uTailE.cnt.get + 1.U) === uTailE.num.get, uTailE.endAddr(rni.offset - 1, 0), 0.U)
        ))
      }.elsewhen(io.uAxiR.fire & dataCtrlQ.io.dataOut.bits.idx === i.U){
        e.shift     := e.nextShift
        e.nextShift := (e.nextShift + e.size) & e.byteMask | ~e.byteMask & e.nextShift
        e.finish    := Mux((e.nextShift === e.endShift), true.B, e.finish)
      }
      when(io.dAxiR.fire & io.dAxiR.bits.id === i.U){
        e.beat  := Mux(e.beat === 0.U, 1.U, e.beat)
      }
  }

  txArBdl                       := 0.U.asTypeOf(txArBdl)
  private val specWrapModify     = uTailE.cache(1) & (uTailE.byteMask(rni.offset) ^ uTailE.byteMask(rni.offset - 1))
  private val lessWrapModify     = uTailE.cache(1) & Burst.isWrap(uTailE.burst)   & !uTailE.byteMask(rni.offset - 1)
  private val specWrapModifyAddr = Cat(uTailE.preAddr, uTailE.exAddr(rni.pageBits - 1, rni.offset), 0.U(rni.offset.W))
  private val lessWrapModifyAddr = Cat(uTailE.preAddr, uTailE.exAddr(rni.pageBits - 1, rni.offset - 1), 0.U((rni.offset - 1).W))
  private val defaultAddr        = Cat(uTailE.preAddr, uTailE.exAddr)
  txArBdl.addr      := PriorityMux(Seq(
    specWrapModify  -> specWrapModifyAddr,
    lessWrapModify  -> lessWrapModifyAddr,
    true.B          -> defaultAddr
  ))
  txArBdl.cache     := uTailE.cache
  txArBdl.burst     := Burst.INCR
  txArBdl.user      := uTailE.id
  txArBdl.id        := dHeadPtr.value
  txArBdl.size      := Mux(!uTailE.cache(1), uTailE.size , log2Ceil(dw/8).U)

  private val incrHalf     = Burst.isIncr(uTailE.burst) &  uTailE.exAddr(rni.offset - 1)
  private val wrapHalf     = Burst.isWrap(uTailE.burst) & (uTailE.exAddr(rni.offset - 1) & uTailE.byteMask(rni.offset) | !uTailE.byteMask(rni.offset - 1))
  private val otherHalf    = (uTailE.exAddr(rni.pageBits - 1, rni.offset) === uTailE.endAddr(rni.pageBits- 1, rni.offset)) & (uTailE.endAddr(rni.offset - 1, 0) <= "b100000".U) & (uTailE.exAddr(rni.offset - 1, 0) =/= uTailE.endAddr(rni.offset - 1, 0))
  txArBdl.len             := Mux(!uTailE.cache(1) | incrHalf | wrapHalf | otherHalf | Burst.isFix(uTailE.burst), 0.U, 1.U)

  io.uAxiAr.ready     := rxArPipe.io.enq.ready
  io.uAxiR.bits       := 0.U.asTypeOf(io.uAxiR.bits)
  io.uAxiR.bits.data  := dataCtrlQ.io.dataOut.bits.data
  io.uAxiR.bits.id    := dataCtrlQ.io.dataOut.bits.id
  io.uAxiR.bits.last  := dArEntrys(dataCtrlQ.io.dataOut.bits.idx).last & (dArEntrys(dataCtrlQ.io.dataOut.bits.idx).nextShift === dArEntrys(dataCtrlQ.io.dataOut.bits.idx).endShift)
  io.uAxiR.bits.resp  := dataCtrlQ.io.dataOut.bits.resp
  io.uAxiR.valid      := dataCtrlQ.io.dataOut.valid

  io.dAxiAr.bits      := txArBdl
  io.dAxiAr.valid     := uHeadPtr =/= uTailPtr & !isFull(dHeadPtr, dTailPtr)
  io.dAxiR.ready      := dataCtrlQ.io.dataIn.ready

  io.working          := uHeadPtr =/= uTailPtr || dHeadPtr =/= dTailPtr || rxArPipe.io.count =/= 0.U

  dataCtrlQ.io.dataIn.valid       := io.dAxiR.valid
  dataCtrlQ.io.dataIn.bits.id     := dArEntrys(io.dAxiR.bits.id).id
  dataCtrlQ.io.dataIn.bits.data   := io.dAxiR.bits.data
  dataCtrlQ.io.dataIn.bits.idx    := io.dAxiR.bits.id
  dataCtrlQ.io.dataIn.bits.last   := io.dAxiR.bits.last
  dataCtrlQ.io.dataIn.bits.beat   := dArEntrys(io.dAxiR.bits.id).beat
  dataCtrlQ.io.dataOut.ready      := io.uAxiR.ready

  dataCtrlQ.io.ptr.endShift       := dArEntrys(dataCtrlQ.io.dataOut.bits.idx).endShift
  dataCtrlQ.io.ptr.nextShift      := dArEntrys(dataCtrlQ.io.dataOut.bits.idx).nextShift
  dataCtrlQ.io.ptr.shift          := dArEntrys(dataCtrlQ.io.dataOut.bits.idx).shift
  
  rxArPipe.io.enq.valid   := io.uAxiAr.valid
  rxArPipe.io.enq.bits    := rxArBdl.pipeInit(io.uAxiAr.bits)
  rxArPipe.io.deq.ready   := !isFull(uHeadPtr, uTailPtr)
}
