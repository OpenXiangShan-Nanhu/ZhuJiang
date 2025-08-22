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
import firrtl2.flattenType

class AxiWrSlave(node: Node)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni = DmaParams(node = node)
  private val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamsUser = AxiParams(addrBits = axiParams.addrBits, idBits = log2Ceil(node.outstanding), userBits = axiParams.idBits + log2Ceil(rni.axiEntrySize), dataBits = axiParams.dataBits,
    attr = axiParams.attr, lenBits = axiParams.lenBits, qosBits = axiParams.qosBits, regionBits = axiParams.regionBits)
  private val axiParamsLast  = AxiParams(addrBits = axiParams.addrBits, idBits = axiParams.idBits, userBits = 1, dataBits = axiParams.dataBits,
    attr = axiParams.attr, lenBits = axiParams.lenBits, qosBits = axiParams.qosBits, regionBits = axiParams.regionBits)
  private val axiWParamsUser = AxiParams(addrBits = axiParams.addrBits, idBits = log2Ceil(node.outstanding), userBits = log2Ceil(node.outstanding), dataBits = axiParams.dataBits,
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
    val uAxiAw  = Flipped(Decoupled(new AWFlit(axiParamsLast)))
    val uAxiW   = Flipped(Decoupled(new WFlit(axiParams)))
    val uAxiB   = Decoupled(new BFlit(axiParams))
    val dAxiAw  = Decoupled(new AWFlit(axiParamsUser))
    val dAxiW   = Decoupled(new WFlit(axiWParamsUser))
    val dAxiB   = Flipped(Decoupled(new BFlit(axiParams)))
    val finish  = Flipped(Valid(UInt(log2Ceil(node.outstanding).W)))
    val working = Output(Bool())
  })

/* 
 * Reg and Wire Define
 */
  private val uAwEntrys      = Reg(Vec(rni.axiEntrySize, new AxiWrEntry(isPipe = false, node = node)))
  private val uHeadPtr       = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val uTailPtr       = RegInit(CirQAxiEntryPtr(f = false.B, v = 0.U))
  private val dAwEntrys      = RegInit(VecInit(Seq.fill(node.outstanding)((new AxiWMstEntry(node)).Lit(_.busy -> false.B))))

// Vec declare
  private val freeVec        = dAwEntrys.map(e => !e.busy)
  private val validVec       = dAwEntrys.map(e => e.busy)


// Judge whether it is valid
  private val freeValid      = freeVec.reduce(_ | _)
  private val dAwWorking     = validVec.reduce(_ | _)

// Sellect from Vec
  private val selFree        = StepRREncoder(freeVec, io.dAxiAw.fire)

// SubModule declare
  private val rxAwPipe       = Module(new Queue(gen = new AxiWrEntry(isPipe = true, node = node), entries = 2, pipe = false, flow = false))
  private val wIdQueue       = Module(new Queue(gen = UInt(log2Ceil(node.outstanding).W), entries = node.outstanding, pipe = false, flow = false))
  private val bIdQueue       = Module(new Queue(gen = UInt(axiParams.idBits.W), entries = 2, pipe = false, flow = false))
  private val mergeReg       = Module(new MergeReg)

  private val merComReg      = RegInit(0.U(2.W)) //this value can not be 3
  private val mergeLastReg   = RegInit(false.B)

//Connect to wIdQueue
  private val wDataPtr       = Reg(UInt(log2Ceil(node.outstanding).W))
  private val wDataPtrValid  = RegInit(false.B)

  private val rxAwBdl        = WireInit(0.U.asTypeOf(new AxiWrEntry(isPipe = true, node = node)))
  private val txAwBdl        = WireInit(0.U.asTypeOf(new AWFlit(axiParamsUser)))
  private val uTailE         = uAwEntrys(uTailPtr.value)
  private val userArid_hi    = io.dAxiAw.bits.user.getWidth - 1
  private val userArid_lo    = log2Ceil(rni.axiEntrySize)
  private val userEid_hi     = log2Ceil(rni.axiEntrySize) - 1

/* 
 * Merge Logic
 */
  private val nextShiftHintCompValid = !dAwEntrys(wIdQueue.io.deq.bits).nextShift(log2Ceil(dw/8) - 1, 0).orR & !Burst.isFix(dAwEntrys(wIdQueue.io.deq.bits).burst) & !dAwEntrys(wIdQueue.io.deq.bits).specWrap
  private val nextShiftHintLastValid = !dAwEntrys(wIdQueue.io.deq.bits).nextShift(rni.offset - 1, 0).orR     & !Burst.isFix(dAwEntrys(wIdQueue.io.deq.bits).burst) & !dAwEntrys(wIdQueue.io.deq.bits).specWrap
  private val noMergeDeq             = dAwEntrys(wIdQueue.io.deq.bits).dontMerge
  private val mergeDeq               = io.uAxiW.bits._last || nextShiftHintLastValid && !dAwEntrys(wIdQueue.io.deq.bits).fullWrap

  private val wDataPtrDeq            = (mergeDeq || noMergeDeq) & io.uAxiW.fire
  private val sDataPtrDeq            = io.dAxiW.fire && io.dAxiW.bits._last
  private val reachPeak              = (uAwEntrys(uTailPtr.value).cnt.get + 1.U) === uAwEntrys(uTailPtr.value).num.get
  private val reachBottom            = (uTailE.num.get === 0.U) & (uTailE.cnt.get === 63.U)   // 4096 / 64 = 64
  private val tailPtrAdd             = io.dAxiAw.fire & (reachBottom | reachPeak)


/* 
 * Pointer movement logic
 */
  wDataPtrValid     := Mux(wIdQueue.io.deq.fire, true.B, Mux(sDataPtrDeq, false.B, wDataPtrValid))
  wDataPtr          := Mux(wIdQueue.io.deq.fire, wIdQueue.io.deq.bits, wDataPtr)
  uHeadPtr          := Mux(rxAwPipe.io.deq.fire, uHeadPtr + 1.U, uHeadPtr)
  uTailPtr          := Mux(tailPtrAdd          , uTailPtr + 1.U, uTailPtr)


  //Merge Data Reg Logic
  private val dSendLast     = io.dAxiW.bits._last & io.dAxiW.fire
  private val dShodLast     = io.uAxiW.fire & (nextShiftHintLastValid | dAwEntrys(wIdQueue.io.deq.bits).dontMerge | io.uAxiW.bits._last)
  mergeLastReg             := PriorityMux(Seq(
    (dShodLast & !dSendLast) -> (!mergeLastReg),
    (!dShodLast & dSendLast) -> (!mergeLastReg),
    true.B                   -> mergeLastReg
  ))
  private val dSendComp     = io.dAxiW.fire
  private val dShodComp     = io.uAxiW.fire & (nextShiftHintCompValid | dAwEntrys(wIdQueue.io.deq.bits).dontMerge | io.uAxiW.bits._last)
  private val specShodComp  = dAwEntrys(wIdQueue.io.deq.bits).fullWrap & dShodComp
  private val normShodComp  = !dAwEntrys(wIdQueue.io.deq.bits).fullWrap & dShodComp
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
        e.cnt.get         := e.cnt.get + 1.U
        e.exAddr          := Cat(e.exAddr(rni.pageBits - 1, rni.offset) + 1.U, 0.U(rni.offset.W)) & e.byteMask | ~e.byteMask & e.exAddr
      }
  }
  dAwEntrys.zipWithIndex.foreach {
    case(e, i) =>
      when(io.dAxiAw.fire & (selFree === i.U)) {
        e.burst        :=  uTailE.burst
        e.dontMerge    := !uTailE.cache(1)
        e.id           :=  uTailE.id
        e.last         :=  (reachBottom | reachPeak) & uTailE.spLast
        e.shift        :=  uTailE.exAddr(rni.offset - 1, 0)
        e.mask         :=  uTailE.byteMask(rni.offset - 1, 0)
        e.nextShift    :=  (uTailE.exAddr(rni.offset - 1, 0) + (1.U(rni.offset.W) << uTailE.size)) & uTailE.byteMask(rni.offset - 1, 0) | uTailE.exAddr(rni.offset - 1, 0) & ~uTailE.byteMask(rni.offset - 1, 0)
        e.size         :=  1.U(6.W) << uTailE.size
        e.busy         := true.B
        e.specWrap     :=  Burst.isWrap(uTailE.burst) & uTailE.cache(1) & !uTailE.byteMask(rni.offset)
        e.fullWrap     :=  Burst.isWrap(uTailE.burst) & uTailE.cache(1) & (uTailE.byteMask(rni.offset) ^ uTailE.byteMask(rni.offset - 1))
      }.elsewhen(io.uAxiW.fire & (wIdQueue.io.deq.bits === i.U)) {
        e.shift     := e.nextShift
        e.nextShift := (e.nextShift + e.size) & e.mask | e.nextShift & ~e.mask
      }
      when(io.finish.valid && io.finish.bits === i.U) {
        assert(e.busy)
        e.busy         := false.B
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
  txAwBdl.qos    := uTailE.qos
  txAwBdl.size   := Mux(!uTailE.cache(1), uTailE.size, log2Ceil(dw/8).U)
  txAwBdl.cache  := uTailE.cache
  txAwBdl.burst  := Burst.INCR
  txAwBdl.id     := selFree
  txAwBdl.user   := Cat(uTailE.id, uTailPtr.value)

  private val incrOffset        = uTailE.exAddr(rni.offset - 1) & Burst.isIncr(uTailE.burst)
  private val wrapOffset        = Burst.isWrap(uTailE.burst) & (uTailE.exAddr(rni.offset - 1) & uTailE.byteMask(rni.offset) | !uTailE.byteMask(rni.offset - 1))
  private val otherHalf         = (uTailE.exAddr(rni.pageBits - 1, rni.offset) === uTailE.endAddr(rni.pageBits - 1, rni.offset)) & (uTailE.endAddr(rni.offset - 1, 0) <= "b100000".U) & (uTailE.endAddr > uTailE.exAddr)
  txAwBdl.len                  := Mux(!uTailE.cache(1) | Burst.isFix(uTailE.burst) | incrOffset | wrapOffset | otherHalf, 0.U, 1.U)

// /* 
//  * IO Connection Logic
//  */
  io.uAxiAw.ready    := rxAwPipe.io.enq.ready
  io.uAxiW.ready     := wIdQueue.io.deq.valid && io.dAxiW.ready
  io.uAxiB.bits      := 0.U.asTypeOf(io.uAxiB.bits)
  io.uAxiB.bits.id   := bIdQueue.io.deq.bits
  io.uAxiB.valid     := bIdQueue.io.deq.valid
  io.dAxiAw.valid    := uHeadPtr =/= uTailPtr & freeValid && wIdQueue.io.enq.ready
  io.dAxiAw.bits     := txAwBdl
  io.dAxiW.bits      := 0.U.asTypeOf(io.dAxiW.bits)
  io.dAxiW.bits.data := mergeReg.io.dataOut.bits.data
  io.dAxiW.bits.strb := mergeReg.io.dataOut.bits.strb
  io.dAxiW.bits.last := mergeLastReg & (merComReg === 1.U)
  io.dAxiW.bits.user := Mux(!wDataPtrValid, wIdQueue.io.deq.bits, wDataPtr)
  io.dAxiW.valid     := merComReg =/= 0.U
  io.dAxiB.ready     := bIdQueue.io.enq.ready

  io.working         := RegNext(uHeadPtr =/= uTailPtr || dAwWorking || rxAwPipe.io.count =/= 0.U)

  mergeReg.io.dataIn.valid         := io.uAxiW.fire
  mergeReg.io.dataIn.bits.fixMerge := !dAwEntrys(wIdQueue.io.deq.bits).dontMerge & Burst.isFix(dAwEntrys(wIdQueue.io.deq.bits).burst)
  mergeReg.io.dataIn.bits.strb     := io.uAxiW.bits.strb
  mergeReg.io.dataIn.bits.beat     := dAwEntrys(wIdQueue.io.deq.bits).shift(rni.offset - 1)
  mergeReg.io.dataIn.bits.data     := io.uAxiW.bits.data
  mergeReg.io.dataOut.ready        := merComReg =/= 0.U & io.dAxiW.ready

  private val bid          = io.dAxiB.bits.id(log2Ceil(node.outstanding) - 1, 0)
  bIdQueue.io.deq.ready   := io.uAxiB.ready
  bIdQueue.io.enq.valid   := dAwEntrys(bid).last & io.dAxiB.valid
  bIdQueue.io.enq.bits    := dAwEntrys(bid).id

  rxAwPipe.io.enq.bits    := rxAwBdl.pipeInit(io.uAxiAw.bits)
  rxAwPipe.io.enq.valid   := io.uAxiAw.valid
  rxAwPipe.io.deq.ready   := !isFull(uHeadPtr, uTailPtr)

  wIdQueue.io.enq.valid      := io.dAxiAw.fire
  wIdQueue.io.enq.bits       := selFree
  wIdQueue.io.deq.ready      := wDataPtrDeq
  
/* 
 * Assertion
 */

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
  when(io.uAxiW.fire) {
    assert(wIdQueue.io.deq.valid)
  }
}