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

class AxiWrSlave(node: Node)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni = DmaParams(node = node)
  private val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())

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
    val uAxiAw  = Flipped(Decoupled(new AWFlit(axiParams)))
    val uAxiW   = Flipped(Decoupled(new WFlit(axiParams)))
    val uAxiB   = Decoupled(new BFlit(axiParams))
    val dAxiAw  = Decoupled(new AWFlit(axiParams))
    val dAxiW   = Decoupled(new WFlit(axiParams))
    val dAxiB   = Flipped(Decoupled(new BFlit(axiParams)))
    val working = Output(Bool())
  })

/* 
 * Reg and Wire Define
 */
  private val uAwEntrys      = Reg(Vec(rni.axiEntrySize, new AxiWrEntry(isPipe = false, node = node)))
  private val uHeadPtr       = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uTailPtr       = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))

  private val dAwEntrys      = Reg(Vec(node.outstanding, new AxiWMstEntry(node)))
  private val dHeadPtr       = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dTailPtr       = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val wDataPtr       = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val sDataPtr       = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))

  private val rxAwPipe       = Module(new Queue(gen = new AxiWrEntry(isPipe = true, node = node), entries = 2, pipe = false, flow = false))
  private val bIdQueue       = Module(new Queue(gen = UInt(axiParams.idBits.W), entries = 2, pipe = false, flow = false))
  private val mergeReg       = Module(new MergeReg)
  private val merComReg      = RegInit(0.U(2.W)) //this value can not be 3
  private val mergeLastReg   = RegInit(false.B)

  private val rxAwBdl        = WireInit(0.U.asTypeOf(new AxiWrEntry(isPipe = true, node = node)))
  private val txAwBdl        = WireInit(0.U.asTypeOf(new AWFlit(axiParams)))

  private val uTailE         = uAwEntrys(uTailPtr.value)

/* 
 * Pointer Logic
 */
  private val nextShiftHintCompValid = !dAwEntrys(wDataPtr.value).nextShift(log2Ceil(dw/8) - 1, 0).orR & !Burst.isFix(dAwEntrys(wDataPtr.value).burst) & !dAwEntrys(wDataPtr.value).specWrap
  private val nextShiftHintLastValid = !dAwEntrys(wDataPtr.value).nextShift(rni.offset - 1, 0).orR     & !Burst.isFix(dAwEntrys(wDataPtr.value).burst) & !dAwEntrys(wDataPtr.value).specWrap
  private val noFullWrapWDPtrAdd     = dAwEntrys(wDataPtr.value).dontMerge || io.uAxiW.bits._last
  private val fullWrapWDPtrAdd       = nextShiftHintLastValid & !dAwEntrys(wDataPtr.value).fullWrap
  private val fixWDataPtrAdd         = !dAwEntrys(wDataPtr.value).dontMerge & io.uAxiW.bits._last & Burst.isFix(dAwEntrys(wDataPtr.value).burst)

  private val wDataPtrAdd            = (fullWrapWDPtrAdd || fixWDataPtrAdd || noFullWrapWDPtrAdd) & io.uAxiW.fire
  private val reachPeak              = (uAwEntrys(uTailPtr.value).cnt.get + 1.U) === uAwEntrys(uTailPtr.value).num.get
  private val reachBottom            = (uTailE.num.get === 0.U) & (uTailE.cnt.get === 63.U)   // 4096 / 64 = 64
  private val tailPtrAdd             = io.dAxiAw.fire & (reachBottom | reachPeak)
  private val sDataPtrAdd            = io.dAxiW.fire & io.dAxiW.bits._last

  uHeadPtr   := Mux(rxAwPipe.io.deq.fire, uHeadPtr + 1.U, uHeadPtr)
  uTailPtr   := Mux(tailPtrAdd          , uTailPtr + 1.U, uTailPtr)
  dHeadPtr   := Mux(io.dAxiAw.fire      , dHeadPtr + 1.U, dHeadPtr)
  dTailPtr   := Mux(io.dAxiB.fire       , dTailPtr + 1.U, dTailPtr)
  wDataPtr   := Mux(wDataPtrAdd         , wDataPtr + 1.U, wDataPtr)
  sDataPtr   := Mux(sDataPtrAdd         , sDataPtr + 1.U, sDataPtr)

  //Merge Data Reg Logic
  private val dSendLast     = io.dAxiW.bits._last & io.dAxiW.fire
  private val dShodLast     = io.uAxiW.fire & (nextShiftHintLastValid | dAwEntrys(wDataPtr.value).dontMerge | io.uAxiW.bits._last)
  mergeLastReg             := PriorityMux(Seq(
    (dShodLast & !dSendLast) -> (!mergeLastReg),
    (!dShodLast & dSendLast) -> (!mergeLastReg),
    true.B                   -> mergeLastReg
  ))
  private val dSendComp     = io.dAxiW.fire
  private val dShodComp     = io.uAxiW.fire & (nextShiftHintCompValid | dAwEntrys(wDataPtr.value).dontMerge | io.uAxiW.bits._last)
  private val specShodComp  = dAwEntrys(wDataPtr.value).fullWrap & dShodComp
  private val normShodComp  = !dAwEntrys(wDataPtr.value).fullWrap & dShodComp
  merComReg                := PriorityMux(Seq(
    (dSendComp & !normShodComp) -> (merComReg - 1.U),
    (!dSendComp & normShodComp) -> (merComReg + 1.U),
    specShodComp                -> (merComReg + 2.U),
    true.B                      ->  merComReg
  ))

  uAwEntrys.zipWithIndex.foreach {
    case(e, i) =>
      when(rxAwPipe.io.deq.fire & (uHeadPtr.value === i.U)){
        e.entryInit(rxAwPipe.io.deq.bits)
      }.elsewhen((uTailPtr.value === i.U) & io.dAxiAw.fire){
        val notModify      = !e.cache(1) 
        val modify         =  e.cache(1)
        e.cnt.get         := e.cnt.get + 1.U
        e.exAddr          := PriorityMux(Seq(
          notModify       -> (~e.byteMask & e.exAddr | (e.exAddr + (1.U((rni.offset).W) << e.size)) & e.byteMask),
          modify          -> (Cat(e.exAddr(rni.pageBits - 1, rni.offset) + 1.U, 0.U(rni.offset.W)) & e.byteMask | ~e.byteMask & e.exAddr)
        ))
      }
  }
  dAwEntrys.zipWithIndex.foreach {
    case(e, i) =>
      when(io.dAxiAw.fire & (dHeadPtr.value === i.U)) {
        e.burst        :=  uTailE.burst
        e.dontMerge    := !uTailE.cache(1)
        e.id           :=  uTailE.id
        e.last         :=  reachBottom | reachPeak
        e.shift        :=  uTailE.exAddr(rni.offset - 1, 0)
        e.mask         :=  uTailE.byteMask(rni.offset - 1, 0)
        e.nextShift    :=  (uTailE.exAddr(rni.offset - 1, 0) + (1.U(rni.offset.W) << uTailE.size)) & uTailE.byteMask(rni.offset - 1, 0) | uTailE.exAddr(rni.offset - 1, 0) & ~uTailE.byteMask(rni.offset - 1, 0)
        e.size         :=  1.U(6.W) << uTailE.size
        e.specWrap     :=  Burst.isWrap(uTailE.burst) & uTailE.cache(1) & !uTailE.byteMask(rni.offset)
        e.fullWrap     :=  Burst.isWrap(uTailE.burst) & uTailE.cache(1) & (uTailE.byteMask(rni.offset) ^ uTailE.byteMask(rni.offset - 1))
      }.elsewhen(io.uAxiW.fire & (wDataPtr.value === i.U)) {
        e.shift     := e.nextShift
        e.nextShift := (e.nextShift + e.size) & e.mask | e.nextShift & ~e.mask
      }
  }
  txAwBdl                       := 0.U.asTypeOf(txAwBdl)
  private val specWrapModify     = uTailE.cache(1) & Burst.isWrap(uTailE.burst) & (uTailE.byteMask(rni.offset) ^  uTailE.byteMask(rni.offset - 1))
  private val lessWrapModify     = uTailE.cache(1) & Burst.isWrap(uTailE.burst) & !uTailE.byteMask(rni.offset - 1)
  private val normalModify       = uTailE.cache(1)
  private val specWrapModifyAddr = Cat(uTailE.preAddr, uTailE.exAddr(rni.pageBits - 1, rni.offset), 0.U(rni.offset.W))
  private val lessWrapModifyAddr = Cat(uTailE.preAddr, uTailE.exAddr(rni.pageBits - 1, rni.offset - 1), 0.U((rni.offset - 1).W))
  private val normalAddr         = Cat(uTailE.preAddr, uTailE.exAddr(rni.pageBits - 1, rni.offset - 1), 0.U((rni.offset - 1).W))
  private val defaultAddr        = Cat(uTailE.preAddr, uTailE.exAddr)

  txAwBdl.addr       := PriorityMux(Seq(
    specWrapModify   -> specWrapModifyAddr,
    lessWrapModify   -> lessWrapModifyAddr,
    normalModify     -> normalAddr,
    true.B           -> defaultAddr
  ))
  txAwBdl.qos        := uTailE.qos
  txAwBdl.size       := Mux(!uTailE.cache(1) || Burst.isFix(uTailE.burst), uTailE.size, log2Ceil(dw/8).U)
  txAwBdl.cache      := uTailE.cache
  txAwBdl.burst      := Burst.INCR
  txAwBdl.id         := dHeadPtr.value

  private val incrOffset        = uTailE.exAddr(rni.offset - 1) & Burst.isIncr(uTailE.burst)
  private val wrapOffset        = Burst.isWrap(uTailE.burst) & (uTailE.exAddr(rni.offset - 1) & uTailE.byteMask(rni.offset) | !uTailE.byteMask(rni.offset - 1))
  private val otherHalf         = (uTailE.exAddr(rni.pageBits - 1, rni.offset) === uTailE.endAddr(rni.pageBits - 1, rni.offset)) & (uTailE.endAddr(rni.offset - 1, 0) <= "b100000".U) & uTailE.exAddr(rni.offset - 1, 0) =/= uTailE.endAddr(rni.offset - 1, 0)
  txAwBdl.len                  := Mux(!uTailE.cache(1) | Burst.isFix(uTailE.burst) | incrOffset | wrapOffset | otherHalf, 0.U, 1.U)

/* 
 * IO Connection Logic
 */
  io.uAxiAw.ready    := rxAwPipe.io.enq.ready
  io.uAxiW.ready     := wDataPtr =/= dHeadPtr & io.dAxiW.ready & !(wDataPtr =/= sDataPtr & dAwEntrys(sDataPtr.value).fullWrap)
  io.uAxiB.bits      := 0.U.asTypeOf(io.uAxiB.bits)
  io.uAxiB.bits.id   := bIdQueue.io.deq.bits
  io.uAxiB.valid     := bIdQueue.io.deq.valid
  io.dAxiAw.valid    := uHeadPtr =/= uTailPtr & !isFull(dHeadPtr, dTailPtr)
  io.dAxiAw.bits     := txAwBdl
  io.dAxiW.bits      := 0.U.asTypeOf(io.dAxiW.bits)
  io.dAxiW.bits.data := mergeReg.io.dataOut.bits.data
  io.dAxiW.bits.strb := mergeReg.io.dataOut.bits.strb
  io.dAxiW.bits.last := mergeLastReg & (merComReg === 1.U)
  io.dAxiW.valid     := merComReg =/= 0.U
  io.dAxiB.ready     := bIdQueue.io.enq.ready

  io.working         := uHeadPtr =/= uTailPtr || dHeadPtr =/= dTailPtr || rxAwPipe.io.count =/= 0.U

  mergeReg.io.dataIn.valid         := io.uAxiW.fire
  mergeReg.io.dataIn.bits.fixMerge := !dAwEntrys(wDataPtr.value).dontMerge & Burst.isFix(dAwEntrys(wDataPtr.value).burst)
  mergeReg.io.dataIn.bits.strb     := io.uAxiW.bits.strb
  mergeReg.io.dataIn.bits.beat     := dAwEntrys(wDataPtr.value).shift(rni.offset - 1)
  mergeReg.io.dataIn.bits.data     := io.uAxiW.bits.data
  mergeReg.io.dataOut.ready        := merComReg =/= 0.U & io.dAxiW.ready

  bIdQueue.io.deq.ready   := io.uAxiB.ready
  bIdQueue.io.enq.valid   := dAwEntrys(dTailPtr.value).last & io.dAxiB.fire
  bIdQueue.io.enq.bits    := dAwEntrys(dTailPtr.value).id

  rxAwPipe.io.enq.bits    := rxAwBdl.pipeInit(io.uAxiAw.bits)
  rxAwPipe.io.enq.valid   := io.uAxiAw.valid
  rxAwPipe.io.deq.ready   := !isFull(uHeadPtr, uTailPtr)

  when(dShodLast & !dSendLast){
    assert(!mergeLastReg)
  }
  when(!dShodLast & dSendLast){
    assert(mergeLastReg)
  }
  assert(merComReg =/= 3.U)

  when(dSendComp & !normShodComp){
    assert(merComReg =/= 0.U)
  }
}