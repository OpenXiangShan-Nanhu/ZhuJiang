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

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- PoS Bundle --------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class PosState(implicit p: Parameters) extends DJBundle {
  val req     = Bool()
  val snp     = Bool()
  val canNest = if(hasBBN) Some(Bool()) else None
  val tagVal  = Bool() // tag is valid
  val tag     = UInt(posTagBits.W)
  val offset  = UInt(offsetBits.W)

  // def
  def one  : Bool = req ^ snp
  def two  : Bool = req & snp
  def valid: Bool = req | snp
}

class PosCanNest(implicit p: Parameters) extends DJBundle with HasPackHnIdx with HasQoS { val nest = Bool() }

class PosClean(implicit p: Parameters) extends DJBundle with HasPackHnIdx with HasChiChannel with HasQoS

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
    val alloc       = Flipped(Valid(new Addr with HasAddrValid with HasChiChannel))
    val updTag      = Flipped(Valid(new Addr with HasAddrValid with HasPackHnIdx)) // broadcast signal
    val updNest     = if(hasBBN) Some(Flipped(Valid(new PosCanNest))) else None // broadcast signal
    val clean       = Flipped(Valid(new PosClean))   // broadcast signal
    // wakeup TaskBuf Entry
    val wakeup      = Valid(new Addr) // broadcast signal
    // PoS State Signal
    val state       = Output(new PosState with HasAddr)
  })
  dontTouch(io.state)

  /*
   * Reg and Wire declaration
   */
  val stateReg  = RegInit(new PosState().Lit(_.req -> false.B, _.snp -> false.B))
  val stateNext = WireInit(stateReg)
  dontTouch(stateNext)

  /*
   * hit message
   */
  val updTagHit  = io.updTag.valid  & io.updTag.bits.hnIdx.asUInt  === io.hnIdx.asUInt
  val cleanHit   = io.clean.valid   & io.clean.bits.hnIdx.asUInt   === io.hnIdx.asUInt
  val canNestHit = if(hasBBN) io.updNest.get.valid & io.updNest.get.bits.hnIdx.asUInt === io.hnIdx.asUInt else false.B
  dontTouch(cleanHit)

  /*
   * Update PoS State
   */
  when(io.alloc.valid | updTagHit | cleanHit | canNestHit) {
    stateReg := stateNext
  }

  /*
   * Output state
   */
  io.state.req    := stateReg.req
  io.state.snp    := stateReg.snp
  io.state.tagVal := stateReg.tagVal
  io.state.tag    := stateReg.tag
  io.state.offset := stateReg.offset
  if(hasBBN) io.state.canNest.get := stateReg.canNest.get
  io.state.Addr.catPoS(io.config.bankId, stateReg.tag, io.hnIdx.pos.set, io.hnIdx.dirBank, stateReg.offset)
  HAssert.withEn(stateReg.offset === 0.U, io.state.snp)

  /*
   * Modify PoS tag
   */
  when(io.alloc.valid) {
    stateNext.tagVal  := io.alloc.bits.addrVal
    stateNext.tag     := io.alloc.bits.Addr.posTag
    stateNext.offset  := io.alloc.bits.Addr.offset
  }.elsewhen(updTagHit) {
    stateNext.tagVal  := io.updTag.bits.addrVal
    stateNext.tag     := io.updTag.bits.Addr.posTag
    stateNext.offset  := io.updTag.bits.Addr.offset
    HAssert(!stateReg.tagVal)
    HAssert(io.updTag.bits.Addr.offset === 0.U)
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
  if(hasBBN) {
    // set
    when(!stateNext.req) {
      stateNext.canNest.get := false.B
      HAssert(!canNestHit)
    // init
    }.elsewhen(canNestHit) {
      stateNext.canNest.get := io.updNest.get.bits.nest
      HAssert(stateReg.canNest.get ^ io.updNest.get.bits.nest)
    }
    HAssert.withEn(!stateReg.canNest.get, canNestHit     & io.updNest.get.bits.nest)
    HAssert.withEn( stateReg.canNest.get, canNestHit     & !io.updNest.get.bits.nest)
    HAssert.withEn(!stateReg.canNest.get, io.alloc.valid & io.alloc.bits.isReq)
    HAssert.withEn( stateReg.canNest.get, io.alloc.valid & io.alloc.bits.isSnp & stateReg.req)
  }

  // HAssert
  HAssert(!(io.alloc.valid & cleanHit & canNestHit))
  HAssert.checkTimeout(!stateReg.valid, TIMEOUT_POS, cf"PoS TIMEOUT")

  /*
   * Wakeup someone by addr
   */
  io.wakeup.valid := RegNext(cleanHit & stateReg.one & stateReg.tagVal)
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
    val reqPoS      = Flipped(Valid(new ChiChnlBundle))
    val posResp     = Valid(UInt(posWayBits.W))
    val updTag      = Flipped(Valid(new Addr with HasAddrValid with HasPackHnIdx))
    // update PoS
    val updNest     = if(hasBBN) Some(Flipped(Valid(new PosCanNest))) else None // broadcast signal
    val clean       = Flipped(Valid(new PosClean))   // broadcast signal
    // wakeup TaskBuf Entry
    val wakeup      = Valid(new Addr)
    // PoS State Signal
    val stateVec    = Output(Vec(posWays, new PosState with HasAddr))
  })
  dontTouch(io.stateVec)


  /*
   * Module and Reg declaration
   */
  // Entries
  val entries         = Seq.fill(posWays) { Module(new PosEntry()) }
  val lockReg         = RegInit(false.B) // Lock set when ReplaceCM req PoS
  // Entries StateVec
  val esVec           = VecInit(entries.map(_.io.state))
  // Save Alloc in S1
  val allocReg_s1     = RegInit(Valid(new Addr with HasChiChannel).Lit(_.valid -> false.B))
  val allocWayReg_s1  = Reg(UInt(posWayBits.W))

  /*
   * Receive alloc task from TaskBuffer
   */
  // get base message
  val allocTag_s0     = io.alloc_s0.bits.Addr.posTag; dontTouch(allocTag_s0)
  // match pos tag
  val matTagVec_s0    = Cat(esVec.map(s => s.valid & s.tagVal & s.tag === allocTag_s0).reverse)
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
  val canNest_s0      = if(hasBBN) hasMatTag & esVec(matTagWay_s0).canNest.get else false.B
  val blockSnp_s0     = Mux(hasMatTag, !canNest_s0, !hasFree_s0)

  // judge block by s1
  val matchReqS1_s0   = allocReg_s1.valid & allocTag_s0 === allocReg_s1.bits.Addr.posTag // Make the request for retry

  // judge block
  val block_s0        = Mux(io.alloc_s0.bits.isSnp, blockSnp_s0, blockReq_s0) | matchReqS1_s0 | lockReg | io.reqPoS.valid

  /*
   * Store alloc task from taskBuf
   */
  allocReg_s1.valid   := io.alloc_s0.valid & !block_s0
  when(io.alloc_s0.valid) {
    allocReg_s1.bits  := io.alloc_s0.bits
    allocWayReg_s1    := Mux(canNest_s0, matTagWay_s0, freeWay_s0)
  }

  /*
   * Retrun ack to TaskBuffer and Block
   */
  io.sleep_s1               := RegNext(io.alloc_s0.valid & hasMatTag)
  io.block_s1               := RegNext(io.alloc_s0.valid & block_s0) | io.reqPoS.valid
  io.hnIdx_s1.valid         := RegNext(io.alloc_s0.valid) & !io.reqPoS.valid // TODO: del it
  io.hnIdx_s1.bits.dirBank  := io.dirBank
  io.hnIdx_s1.bits.pos.set  := io.posSet
  io.hnIdx_s1.bits.pos.way  := allocWayReg_s1

  /*
   * Receive replace req from ReplaceCM
   */
  val freeVec       = Cat(esVec.map(!_.valid).reverse)
  val replSelWay    = PriorityMux(Seq(
    freeVec(djparam.posWays-3, 0).orR -> PriorityEncoder(freeVec),
    io.reqPoS.bits.isReq -> (posWays-2).U,
    io.reqPoS.bits.isSnp -> (posWays-1).U
  ))
  val reqPosFire    = io.reqPoS.valid & freeVec(replSelWay) & !lockReg
  // Return HnTxnID
  io.posResp.valid  := RegNext(reqPosFire, false.B)
  io.posResp.bits   := RegEnable(replSelWay, reqPosFire)

  /*
   * Lock PoS Set
   */
  when(reqPosFire) {
    lockReg := true.B
    HAssert(!lockReg)
  }.elsewhen(io.updTag.valid & io.updTag.bits.hnIdx.dirBank === io.dirBank & io.updTag.bits.hnIdx.pos.set === io.posSet) {
    lockReg := false.B
    HAssert(lockReg)
  }
  // HAssert
  HAssert.checkTimeout(!lockReg, 128, cf"PoS Lock Set TIMEOUT")

  /*
   * Connect entries
   */
  entries.zipWithIndex.foreach { case(e, i) =>
    // hit
    val replReqHit  = reqPosFire & replSelWay === i.U
    val allocHit    = allocReg_s1.valid  & !io.retry_s1 & allocWayReg_s1 === i.U // TODO: optimize hit logic
    replReqHit.suggestName(f"replReqHit_${i}")
    allocHit.suggestName(f"allocHit${i}")
    // connect
    e.io.config             := io.config
    e.io.hnIdx.dirBank      := io.dirBank
    e.io.hnIdx.pos.set      := io.posSet
    e.io.hnIdx.pos.way      := i.U
    e.io.alloc.valid        := Mux(io.reqPoS.valid, replReqHit, allocHit)
    e.io.alloc.bits.addrVal := !io.reqPoS.valid
    e.io.alloc.bits.addr    := Mux(io.reqPoS.valid, 0.U, allocReg_s1.bits.addr)
    e.io.alloc.bits.channel := Mux(io.reqPoS.valid, io.reqPoS.bits.channel, allocReg_s1.bits.channel)
    e.io.updTag             := io.updTag
    e.io.clean              := io.clean
    io.stateVec(i)          := e.io.state
    if(hasBBN) {
      e.io.updNest.get      := io.updNest.get
    }
  }
  io.wakeup                 := Mux1H(entries.map(_.io.wakeup.valid), entries.map(_.io.wakeup))
  //HAssert
  HAssert(PopCount(entries.map(_.io.wakeup.valid)) <= 1.U)
  HAssert.withEn(!VecInit(esVec.map(es => es.valid & es.tagVal & es.addr === allocReg_s1.bits.addr)).asUInt.orR, allocReg_s1.valid)
  HAssert.withEn(!VecInit(esVec.map(es => es.valid & es.tagVal & es.addr === io.updTag.bits.addr)).asUInt.orR, io.updTag.valid & io.updTag.bits.addrVal)
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
 *  4. tagVal : Indicates whether the saved tag is valid
 *  5. tag    : Part of the request address
 *  6. offset :
 *
 * Special Entry:
 * Entry*: Reserve for LLC Replace
 * Entry#: Reserve for Snoop Evict
 */
class PosTable(isTop: Boolean = false)(implicit p: Parameters) extends DJModule {
  override def isTopModule: Boolean = isTop
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
    val reqPosVec   = Vec(posSets, Flipped(Valid(new ChiChnlBundle)))
    val posRespVec  = Vec(posSets, Valid(UInt(posWayBits.W)))
    val updTag      = Flipped(Valid(new Addr with HasAddrValid with HasPackHnIdx))
    // update PoS
    val updNest     = if(hasBBN) Some(Flipped(Valid(new PosCanNest))) else None // broadcast signal
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
    set.io.reqPoS.valid   := io.reqPosVec(i).valid
    set.io.reqPoS.bits    := io.reqPosVec(i).bits

    // retry from block
    set.io.retry_s1       := io.retry_s1

    // update PoS valid
    set.io.clean          := io.clean
    set.io.updTag         := io.updTag
    if(hasBBN) {
      set.io.updNest.get  := io.updNest.get
    }
  }

  /*
   * Connect IO <- PoS
   */
  io.sleep_s1     := Cat(sets.map(_.io.sleep_s1)).orR
  io.block_s1     := Cat(sets.map(_.io.block_s1)).orR
  io.hnIdx_s1     := Mux1H(sets.map(_.io.hnIdx_s1.valid), sets.map(_.io.hnIdx_s1.bits))
  io.wakeup       := Mux1H(sets.map(_.io.wakeup.valid), sets.map(_.io.wakeup))
  io.posRespVec.zip(sets.map(_.io.posResp)).foreach { case(a, b) => a := b }
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
