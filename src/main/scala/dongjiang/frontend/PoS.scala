package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import chisel3.experimental.BundleLiterals._
import dongjiang.backend.ReqPoS

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- PoS Bundle --------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class PosState(implicit p: Parameters) extends DJBundle with HasAddr {
  val req     = Bool()
  val snp     = Bool()
  val canNest = Bool()
  val tag     = UInt(posTagBits.W)

  // def
  def one  : Bool = req ^ snp
  def two  : Bool = req & snp
  def valid: Bool = req | snp
}

class PosCanNest(implicit p: Parameters) extends DJBundle with HasPackHnIdx  { val nest = Bool() }

class PosClean(implicit p: Parameters) extends DJBundle with HasPackHnIdx with HasChiChannel

// ----------------------------------------------------------------------------------------------------- //
// --------------------------------------------- PoS Entry --------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class PosEntry(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    val hnIdx       = Input(new HnIndex())
    // update PoS
    val alloc       = Flipped(Valid(new Addr with HasChiChannel))
    val updTag      = Flipped(Valid(new Addr with HasPackHnIdx)) // broadcast signal
    val updNest     = Flipped(Valid(new PosCanNest)) // broadcast signal
    val clean       = Flipped(Valid(new PosClean))   // broadcast signal
    // wakeup TaskBuf Entry
    val wakeup      = Valid(new Addr) // broadcast signal
    // PoS State Signal
    val state       = Output(new PosState)
  })
  dontTouch(io.state)

  /*
   * Reg and Wire declaration
   */
  val stateReg  = RegInit(new PosState().Lit(_.req -> false.B, _.snp -> false.B, _.canNest -> false.B))
  val stateNext = WireInit(stateReg)
  dontTouch(stateNext)
  stateReg.addr := DontCare // Dont use stateReg.addr

  /*
   * hit message
   */
  val updTagHit  = io.updTag.valid  & io.updTag.bits.hnIdx.asUInt  === io.hnIdx.asUInt
  val cleanHit   = io.clean.valid   & io.clean.bits.hnIdx.asUInt   === io.hnIdx.asUInt
  val canNestHit = io.updNest.valid & io.updNest.bits.hnIdx.asUInt === io.hnIdx.asUInt
  dontTouch(cleanHit)
  dontTouch(canNestHit)

  /*
   * Update PoS State
   */
  when(io.alloc.valid | updTagHit | cleanHit | canNestHit) {
    stateReg := stateNext
  }

  /*
   * Output state
   */
  io.state := stateReg
  io.state.Addr.catPoS(io.config.bankId, stateReg.tag, io.hnIdx.pos.set, io.hnIdx.dirBank)

  /*
   * Modify PoS tag
   */
  when(io.alloc.valid) {
    stateNext.tag := io.alloc.bits.Addr.posTag
  }.elsewhen(updTagHit) {
    stateNext.tag := io.updTag.bits.Addr.posTag
  }

  /*
   * Modify PoS valid signal
   */
  // release req
  when(cleanHit & io.clean.bits.isReq) {
    stateNext.req := false.B
    HAssert(stateReg.req)
  // alloc req
  }.elsewhen(io.alloc.valid & io.alloc.bits.isReq) {
    stateNext.req := true.B
    HAssert(!stateReg.req)
  }

  // release snp
  when(cleanHit & io.clean.bits.isSnp) {
    stateNext.snp := false.B
    HAssert(stateReg.snp)
  // alloc snp
  }.elsewhen(io.alloc.valid & io.alloc.bits.isSnp) {
    stateNext.snp := true.B
    HAssert(!stateReg.snp)
  }

  /*
   * Modify PoS canNest
   * if canNest is true, snp can nest req
   */
  when(canNestHit) {
    stateNext.canNest := io.updNest.bits.nest
    HAssert(stateReg.req)
  }
  HAssert.withEn(!stateReg.canNest, canNestHit      &  io.updNest.bits.nest)
  HAssert.withEn( stateReg.canNest, canNestHit      & !io.updNest.bits.nest)
  HAssert.withEn(!stateReg.canNest, io.alloc.valid  &  io.alloc.bits.isReq)
  HAssert.withEn( stateReg.canNest, io.alloc.valid  &  io.alloc.bits.isSnp & stateReg.req)

  // HAssert
  HAssert(!(io.alloc.valid & cleanHit & canNestHit))
  HAssert.checkTimeout(!stateReg.valid, TIMEOUT_POS, cf"PoS TIMEOUT")

  /*
   * Wakeup someone by addr
   */
  io.wakeup.valid     := RegNext(cleanHit & stateReg.one)
  io.wakeup.bits.Addr.catPoS(io.config.bankId, stateReg.tag, io.hnIdx.pos.set, io.hnIdx.dirBank)
}

// ----------------------------------------------------------------------------------------------------- //
// --------------------------------------------- PoS Set ----------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class PosSet(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    val dirBank     = Input(UInt(dirBankBits.W))
    val posSet      = Input(UInt(posSetBits.W))
    // connect to TaskBuffer and Block
    val alloc_s0    = Flipped(Valid(new Addr with HasChiChannel))
    val sleep_s1    = Output(Bool())
    val block_s1    = Output(Bool())
    val hnIdx_s1    = Valid(new HnIndex)
    val retry_s1    = Input(Bool())
    // connect to ReplaceCM
    val reqPoS      = Flipped(new ReqPoS())
    val updTag      = Flipped(Valid(new Addr with HasPackHnIdx))
    // update PoS
    val updNest     = Flipped(Valid(new PosCanNest)) // broadcast signal
    val clean       = Flipped(Valid(new PosClean))   // broadcast signal
    // wakeup TaskBuf Entry
    val wakeup      = Valid(new Addr)
    // PoS State Signal
    val stateVec    = Output(Vec(posWays, new PosState))
  })
  dontTouch(io.stateVec)


  /*
   * Module and Reg declaration
   */
  // Entries
  val entries         = Seq.fill(posWays) { Module(new PosEntry()) }
  val lockReg         = RegInit(false.B)
  // Entries StateVec
  val esVec           = VecInit(entries.map(_.io.state))
  // Save Alloc in S1
  val allocReg_s1     = RegInit(Valid(new Addr with HasChiChannel).Lit(_.valid -> false.B))
  val allocWayReg_s1  = Reg(UInt(posWayBits.W))


  /*
   * Receive alloc task from TaskBuffer
   *
   * 1. task is CHIREQ :
   *  a. no block
   *  b. block by addr
   *
   * 2. task is CHISNP :
   *  a. no block
   *  a. can nest  : nest someone
   *  b. cant nest : block by addr
   */
  // get base message
  val allocTag_s0     = io.alloc_s0.bits.Addr.posTag
  // match pos tag
  val matTagVec_s0    = Cat(esVec.map(s => s.valid & s.tag === allocTag_s0).reverse)
  val matTagWay_s0    = PriorityEncoder(matTagVec_s0)
  val hasMatTag       = matTagVec_s0.orR
  dontTouch(matTagVec_s0)
  HardwareAssertion.withEn(PopCount(matTagVec_s0) <= 1.U, io.alloc_s0.valid)

  // get free way
  val useWay_s1       = Mux(allocReg_s1.valid, ~UIntToOH(allocWayReg_s1), Fill(posWays, 1.U))
  val freeVec_s0      = Cat(esVec.map(!_.valid).reverse) & useWay_s1
  val hasFree_s0      = freeVec_s0(djparam.posWays-3, 0).orR
  val freeWay_s0      = PriorityEncoder(freeVec_s0)

  // judge block req
  val blockReq_s0     = Mux(hasMatTag, true.B, !hasFree_s0)

  // judge block snp
  val canNest_s0      = Mux(hasMatTag, esVec(matTagWay_s0).canNest, false.B)
  val blockSnp_s0     = Mux(hasMatTag, !canNest_s0, !hasFree_s0)

  // judge block
  val block_s0        = Mux(io.alloc_s0.bits.isSnp, blockSnp_s0, blockReq_s0) | lockReg

  /*
   * Store alloc task from taskBuf
   */
  allocReg_s1.valid := io.alloc_s0.valid & !block_s0
  when(io.alloc_s0.valid) {
    allocReg_s1.bits  := io.alloc_s0.bits
    allocWayReg_s1    := Mux(canNest_s0, matTagWay_s0, freeWay_s0)
  }
  HardwareAssertion.withEn(io.alloc_s0.bits.Addr.tag =/= allocReg_s1.bits.Addr.tag, io.alloc_s0.valid & allocReg_s1.valid)

  /*
   * Retrun ack to TaskBuffer and Block
   */
  io.sleep_s1               := RegNext(io.alloc_s0.valid & hasMatTag & !lockReg)
  io.block_s1               := RegNext(io.alloc_s0.valid & block_s0)
  io.hnIdx_s1.valid         := RegNext(io.alloc_s0.valid)
  io.hnIdx_s1.bits.dirBank  := io.dirBank
  io.hnIdx_s1.bits.pos.set  := io.posSet
  io.hnIdx_s1.bits.pos.way  := allocWayReg_s1

  /*
   * Lock PoS Set
   */
  when(io.reqPoS.req.fire) {
    lockReg := true.B
    HAssert(!lockReg)
  }.elsewhen(io.updTag.valid & io.updTag.bits.hnIdx.dirBank === io.dirBank & io.updTag.bits.hnIdx.pos.set === io.posSet) {
    lockReg := false.B
    HAssert(lockReg)
  }
  // HAssert
  HAssert.checkTimeout(!lockReg, 128, cf"PoS Lock Set TIMEOUT")


  /*
   * Receive replace req from ReplaceCM
   */
  val freeVec    = Cat(esVec.map(!_.valid).reverse)
  val replSelWay = PriorityMux(Seq(
    freeVec(djparam.posWays-3, 0).orR -> PriorityEncoder(freeVec),
    io.reqPoS.req.bits.isReq          -> (posWays-2).U,
    io.reqPoS.req.bits.isSnp          -> (posWays-1).U
  ))
  io.reqPoS.req.ready     := freeVec(replSelWay)
  // Return HnTxnID
  io.reqPoS.resp.hnTxnID  := VecInit(entries.map(_.io.hnIdx.getTxnID))(replSelWay)

  /*
   * Connect entries
   */
  entries.zipWithIndex.foreach { case(e, i) =>
    // hit
    val replReqHit  = io.reqPoS.req.fire & replSelWay === i.U
    val allocHit    = allocReg_s1.valid  & !io.retry_s1 & allocWayReg_s1 === i.U
    replReqHit.suggestName(f"replReqHit_${i}")
    allocHit.suggestName(f"allocHit${i}")
    // connect
    e.io.config             := io.config
    e.io.hnIdx.dirBank      := io.dirBank
    e.io.hnIdx.pos.set      := io.posSet
    e.io.hnIdx.pos.way      := i.U
    e.io.alloc.valid        := replReqHit | allocHit
    e.io.alloc.bits.addr    := Mux(io.reqPoS.req.valid, 0.U, allocReg_s1.bits.addr)
    e.io.alloc.bits.channel := Mux(io.reqPoS.req.valid, io.reqPoS.req.bits.channel, allocReg_s1.bits.channel)
    e.io.updTag             := io.updTag
    e.io.updNest            := io.updNest
    e.io.clean              := io.clean
    io.stateVec(i)          := e.io.state
  }
  io.wakeup                 := fastArb(entries.map(_.io.wakeup))
}


// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- PoS Table ---------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
/*
 * PoS Table Structure:
 *        -----------------------------
 *        | Way0 | Way1 | Way2 | Way3 |
 *        |----------------------------
 * Set0:  | 0_0  | 0_1  | 0_2* | 0_3# |
 * Set1:  | 1_0  | 1_1  | 1_2* | 1_3# |
 * Set2:  | 2_0  | 2_1  | 2_2* | 2_3# |
 * Set3:  | 3_0  | 3_1  | 3_2* | 3_3# |
 *        -----------------------------
 *
 * PoS Entry Fields:
 *  1. req    : Indicates that there is currently a CHI REQ running in the system.
 *  2. snp    : Indicates that there is currently a CHI SNP running in the system.
 *  3. canNest: CHI SNP can nest CHI REQ with same addr
 *  4. tag    : Part of the request address
 *
 * Special Entry:
 * Entry*: Reserve for LLC Replace
 * Entry#: Reserve for Snoop Evict
 */
class PosTable(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    val dirBank     = Input(UInt(dirBankBits.W))
    // connect to TaskBuffer and Block
    val alloc_s0    = Flipped(Valid(new Addr with HasChiChannel))
    val sleep_s1    = Output(Bool())
    val block_s1    = Output(Bool())
    val hnIdx_s1    = Output(new HnIndex)
    val retry_s1    = Input(Bool())
    // connect to ReplaceCM
    val reqPoS      = Flipped(new ReqPoS())
    val updTag      = Flipped(Valid(new Addr with HasPackHnIdx))
    // update PoS
    val updNest     = Flipped(Valid(new PosCanNest)) // broadcast signal
    val clean       = Flipped(Valid(new PosClean)) // broadcast signal
    // wakeup TaskBuf Entry
    val wakeup      = Valid(new Addr)
    // Get Full Addr In PoS
    val getAddrVec  = Vec(3, Flipped(new GetAddr()))
    // PoS Busy Signal
    val alrUsePoS   = Output(UInt(log2Ceil(nrPoS + 1).W))
    //  system is working
    val working     = Output(Bool())
  })

  /*
   * Module and Reg declaration
   */
  val sets      = Seq.fill(posSets) { Module(new PosSet()) }
  val debugVec  = WireInit(VecInit(sets.map(_.io.stateVec)))
  dontTouch(debugVec)

  /*
   * Connect PoS <- IO
   */
  sets.zipWithIndex.foreach { case(set, i) =>
    // base
    set.io.config         := io.config
    set.io.dirBank        := io.dirBank
    set.io.posSet         := i.U

    // alloc
    set.io.alloc_s0.valid := io.alloc_s0.valid & io.alloc_s0.bits.Addr.posSet === i.U
    set.io.alloc_s0.bits  := io.alloc_s0.bits

    // replReq
    set.io.reqPoS.req.valid := io.reqPoS.req.valid & io.reqPoS.req.bits.pos.set === i.U
    set.io.reqPoS.req.bits  := io.reqPoS.req.bits

    // retry from block
    set.io.retry_s1       := io.retry_s1

    // update PoS valid
    set.io.updNest        := io.updNest
    set.io.clean          := io.clean
    set.io.updTag         := io.updTag
  }
  io.reqPoS.req.ready     := VecInit(sets.map(_.io.reqPoS.req.ready))(io.reqPoS.req.bits.pos.set)

  /*
   * Connect IO <- PoS
   */
  io.sleep_s1     := Cat(sets.map(_.io.sleep_s1)).orR
  io.block_s1     := Cat(sets.map(_.io.block_s1)).orR
  io.hnIdx_s1     := fastArb(sets.map(_.io.hnIdx_s1)).bits
  io.wakeup       := fastArb(sets.map(_.io.wakeup))
  io.reqPoS.resp  := VecInit(sets.map(_.io.reqPoS.resp))(io.reqPoS.req.bits.pos.set)
  // HAssert
  HAssert(PopCount(sets.map(_.io.hnIdx_s1.valid)) <= 1.U)
  HAssert(PopCount(sets.map(_.io.wakeup.valid)) <= 1.U)

  /*
   * already use PoS num
   */
  io.alrUsePoS := PopCount(Cat(sets.flatMap(_.io.stateVec.map(_.valid))))

  /*
   * Get Addr
   */
  val addrVec2 = Wire(Vec(posSets, Vec(posWays, UInt(addrBits.W))))
  addrVec2.zip(sets.map(_.io.stateVec.map(_.addr))).foreach { case(a, b) => a := b }
  io.getAddrVec.foreach { get => get.result.addr := addrVec2(get.hnIdx.pos.set)(get.hnIdx.pos.way) }
  dontTouch(addrVec2)

  /*
   * Has PoS valid
   */
  io.working := Cat(sets.flatMap(_.io.stateVec.map(_.valid))).orR

  /*
   * HardwareAssertion placePipe
   */
  HAssert.placePipe(1)
}
