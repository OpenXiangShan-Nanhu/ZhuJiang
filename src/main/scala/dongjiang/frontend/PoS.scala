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
// ------------------------------------------- PoS Bundle ---------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class PosWay(implicit p: Parameters) extends DJBundle {
  val way = UInt(posWayBits.W)
}

trait HasNest extends DJBundle { this: DJBundle =>
  val canNest = new Bool()
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
    // req and ack
    val req_s0      = Flipped(Valid(new Addr with HasChiChannel))
    val sleep_s1    = Output(Bool())
    val block_s1    = Output(Bool())
    val posWay_s1   = Valid(UInt(posWayBits.W))
    // retry from block
    val retry_s1    = Input(Bool())
    // update PoS
    val updNest     = Flipped(Valid(new PosWay with HasNest))
    val updTag      = Flipped(Valid(new PosWay with HasAddr))
    val clean       = Flipped(Valid(new PosWay with HasChiChannel))
    val lockSet     = Flipped(Valid(new PosWay))
    val unlockSet   = Flipped(Valid(new PosWay))
    // wakeup TaskBuf Entry
    val wakeup      = Valid(new Addr)
    // PoS State Signal
    val stateVec    = Output(Vec(posWays, Valid(new Addr)))
  })
  dontTouch(io.stateVec)

  /*
   * Reg and Wire declaration
   */
  // PoS Set Vec
  val posSetVecReg  = RegInit(VecInit(Seq.tabulate(posWays) { i => new DJBundle with HasNest {
    val req         = Bool()
    val snp         = Bool()
    val tag         = UInt(posTagBits.W)
    val lockDirSet  = Bool()
    // def
    def one  : Bool = req ^ snp
    def valid: Bool = req | snp
    def get  : Addr = { val a = Wire(new Addr()); a.Addr.catPoS(io.config.bankId, tag, io.posSet, io.dirBank); a }
  }.Lit(_.req -> false.B, _.snp -> false.B, _.canNest -> false.B, _.lockDirSet -> false.B) }))

  // Save Req
  val reqReg_s1     = RegInit(Valid(new DJBundle with HasChiChannel {
    val tag         = UInt(posTagBits.W)
    val way         = UInt(posWayBits.W)
  }).Lit(_.valid -> false.B))

  /*
   * Receive req from taskBuf
   *
   * 1. req is CHIREQ :
   *  a. no block
   *  b. block by addr
   *
   * 2. req is CHISNP :
   *  a. no block
   *  a. can nest  : nest someone
   *  b. cant nest : block by addr
   */

  // get base message
  val reqTag_s0       = io.req_s0.bits.Addr.posTag
  val reqDirSet_s0    = io.req_s0.bits.Addr.dirSet
  // match directory set
  val matDirSetVec_s0 = Wire(Vec(posWays, Bool()))
  matDirSetVec_s0     := posSetVecReg.map(way => way.valid & way.lockDirSet & way.get.Addr.dirSet === reqDirSet_s0)
  val hasMatLock      = matDirSetVec_s0.reduce(_ | _)
  // match pos tag
  val matTagVec_s0    = Wire(Vec(posWays, Bool()))
  matTagVec_s0        := posSetVecReg.map(way => way.valid & way.tag === reqTag_s0)
  val matTagWay_s0    = PriorityEncoder(matTagVec_s0)
  val hasMatTag       = matTagVec_s0.reduce(_ | _)
  dontTouch(matDirSetVec_s0)
  dontTouch(matTagVec_s0)
  HardwareAssertion(PopCount(matTagVec_s0) <= 1.U)

  // get free way
  val freeVec_s0      = posSetVecReg.map(!_.valid)
  val hasFree_s0      = freeVec_s0.reduce(_ | _)
  val freeWay_s0      = PriorityEncoder(freeVec_s0)

  // judge block req
  val blockReq_s0     = hasMatLock | Mux(hasMatTag, true.B, !hasFree_s0)

  // judge block snp
  val canNest_s0      = Mux(hasMatTag, posSetVecReg(matTagWay_s0).canNest, false.B)
  val blockSnp_s0     = hasMatLock | Mux(hasMatTag, !canNest_s0, !hasFree_s0)

  // judge block
  val block_s0        = Mux(io.req_s0.bits.isSnp, blockSnp_s0, blockReq_s0)

  /*
   * Store req from taskBuf
   */
  reqReg_s1.valid         := io.req_s0.valid & !block_s0
  reqReg_s1.bits.tag      := reqTag_s0
  reqReg_s1.bits.channel  := io.req_s0.bits.channel
  reqReg_s1.bits.way      := Mux(canNest_s0, matTagWay_s0, freeWay_s0)
  HardwareAssertion.withEn(reqTag_s0 =/= reqReg_s1.bits.tag, io.req_s0.valid & reqReg_s1.valid)

  /*
   * Retrun ack to taskBuf and block
   */
  io.sleep_s1         := RegNext(io.req_s0.valid & hasMatTag)
  io.block_s1         := RegNext(io.req_s0.valid & block_s0)
  io.posWay_s1.valid  := RegNext(io.req_s0.valid)
  io.posWay_s1.bits   := reqReg_s1.bits.way

  /*
   * Modify pos tag
   */
  posSetVecReg.map(_.tag).zipWithIndex.foreach { case(tag, i) =>
    val updHit  = io.updTag.valid & io.updTag.bits.way === i.U
    val reqHit  = reqReg_s1.valid & reqReg_s1.bits.way === i.U & !io.retry_s1
    when(updHit) {
      tag := io.updTag.bits.Addr.posTag
    }.elsewhen(reqHit) {
      tag := reqReg_s1.bits.tag
    }
    HardwareAssertion(!(updHit & reqHit))
  }

  /*
   * Modify pos ctrl signals
   */
  posSetVecReg.zipWithIndex.foreach { case (ctrl, i) =>
    // receive clean
    val cleanHit    = io.clean.valid & io.clean.bits.way === i.U
    val cleanSnp    = io.clean.bits.isSnp

    // store req
    val reqHit      = reqReg_s1.valid & reqReg_s1.bits.way === i.U & !io.retry_s1
    val reqIsSnp    = reqReg_s1.bits.isSnp

    // modify valid
    when(cleanHit) {
      ctrl.req      := Mux(!cleanSnp, false.B, ctrl.req)
      ctrl.snp      := Mux( cleanSnp, false.B, ctrl.snp)
      // HAssert
      HAssert.withEn(!ctrl.snp & ctrl.req, !cleanSnp, cf"PoS Way[$i]")
      HAssert.withEn(ctrl.snp, cleanSnp, cf"PoS Way[$i]")
    }.elsewhen(reqHit) {
      ctrl.req      := Mux(!reqIsSnp, true.B,  ctrl.req)
      ctrl.snp      := Mux( reqIsSnp, true.B,  ctrl.snp)
      // HAssert
      HAssert.withEn(!ctrl.snp & !ctrl.req, !reqIsSnp, cf"PoS Way[$i]")
      HAssert.withEn(!ctrl.snp, reqIsSnp, cf"PoS Way[$i]")
    }

    // modify lockDirSet
    val lockHit     = io.lockSet.valid   & io.lockSet.bits.way   === i.U
    val unlockHit   = io.unlockSet.valid & io.unlockSet.bits.way === i.U
    ctrl.lockDirSet := Mux(lockHit, true.B, Mux(unlockHit, false.B, ctrl.lockDirSet))
    // HAssert
    HAssert.withEn(!ctrl.lockDirSet, lockHit, cf"PoS Way[$i]")
    HAssert.withEn( ctrl.lockDirSet, unlockHit, cf"PoS Way[$i]")

    // modify canNest
    val updNestHit  = io.updNest.valid & io.updNest.bits.way === i.U
    ctrl.canNest    := Mux(reqHit, false.B, Mux(updNestHit, io.updNest.bits.canNest, ctrl.canNest))
    HAssert.withEn(!ctrl.canNest, updNestHit &  io.updNest.bits.canNest, cf"PoS Way[$i]")
    HAssert.withEn( ctrl.canNest, updNestHit & !io.updNest.bits.canNest, cf"PoS Way[$i]")

    // assert
    HardwareAssertion(PopCount(Seq(cleanHit, reqHit)) <= 1.U, cf"PoS Way[$i]")
    HardwareAssertion.checkTimeout(!ctrl.valid, TIMEOUT_POS, cf"TIMEOUT: PoS Way[$i]")
  }

  /*
   * Wakeup someone by addr
   */
  val cleanWay        = posSetVecReg(io.clean.bits.way)
  io.wakeup.valid     := io.clean.valid & cleanWay.one
  io.wakeup.bits.addr := cleanWay.get.addr

  /*
   * Output state
   */
  io.stateVec.zip(posSetVecReg).foreach { case(a, b) =>
    a.valid     := b.valid
    a.bits.addr := b.get.addr
  }
}


// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- PoS Table ---------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
/*
 * PoS Table Structure:
 *        -----------------------------
 *        | Way0 | Way1 | Way2 | Way3 |
 *        |----------------------------
 * Set0:  |  0_0 |  0_1 |  0_2 |  0_3 |
 * Set1:  |  1_0 |  1_1 |  1_2 |  1_3 |
 * Set2:  |  2_0 |  2_1 |  2_2 |  2_3 |
 * Set3:  |  3_0 |  3_1 |  3_2 |  3_3 |
 *        -----------------------------
 *
 * PoS Entry Fields:
 *  1. req: Indicates that there is currently a CHI REQ running in the system.
 *  2. snp: Indicates that there is currently a CHI SNP running in the system.
 *  3. tag: Part of the request address
 *  4. lockDirSet: Lock the directory set corresponding to the address in replace
 */
class PosTable(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    val dirBank     = Input(UInt(dirBankBits.W))
    // req and ack
    val req_s0      = Flipped(Valid(new Addr with HasChiChannel))
    val sleep_s1    = Output(Bool())
    val block_s1    = Output(Bool())
    val posIdx_s1   = Output(new PosIndex())
    // retry from block
    val retry_s1    = Input(Bool())
    // update PoS
    val updNest     = Flipped(Valid(new PackPosIndex with HasNest))
    val updTag      = Flipped(Valid(new PackPosIndex with HasAddr))
    val clean       = Flipped(Valid(new PackPosIndex with HasChiChannel))
    val lockSet     = Flipped(Valid(new PackPosIndex))
    val unlockSet   = Flipped(Valid(new PackPosIndex))
    // wakeup TaskBuf Entry
    val wakeup      = Valid(new Addr)
    // Get Full Addr In PoS
    val getAddrVec  = Vec(nrGetAddr, Flipped(new GetAddr(true)))
    // PoS Busy Signal
    val alrUsePoS   = Output(UInt(log2Ceil(nrPoS + 1).W))
  })

  /*
   * Module and Reg declaration
   */
  val posSet = Seq.fill(posSets) { Module(new PosSet()) }

  /*
   * Connect PoS <- IO
   */
  posSet.zipWithIndex.foreach { case(set, i) =>
    // base
    set.io.config       := io.config
    set.io.dirBank      := io.dirBank
    set.io.posSet       := i.U

    // req
    set.io.req_s0.valid := io.req_s0.valid & io.req_s0.bits.Addr.posSet === i.U
    set.io.req_s0.bits  := io.req_s0.bits

    // retry from block
    set.io.retry_s1     := io.retry_s1

    // update PoS valid and way
    val updSeq          = Seq(set.io.updNest, set.io.updTag, set.io.clean, set.io.lockSet, set.io.unlockSet)
    val ioUpdSeq        = Seq(io.updNest, io.updTag, io.clean, io.lockSet, io.unlockSet)
    updSeq.zip(ioUpdSeq).foreach { case(upd, ioUpd) =>
      upd.valid         := ioUpd.valid & ioUpd.bits.pos.set === i.U
      upd.bits.way      := ioUpd.bits.pos.way
    }

    // update PoS other
    set.io.updNest.bits.canNest := io.updNest.bits.canNest
    set.io.updTag.bits.addr     := io.updTag.bits.addr
    set.io.clean.bits.channel   := io.clean.bits.channel
  }

  /*
   * Connect IO <- PoS
   */
  val posSetIo        = posSet.map(_.io)
  val setVec_s1       = posSetIo.map(_.posWay_s1.valid)
  val wakeupVec_s1    = posSetIo.map(_.wakeup.valid)
  // ack
  io.sleep_s1         := posSetIo.map(_.sleep_s1).reduce(_ | _)
  io.block_s1         := posSetIo.map(_.block_s1).reduce(_ | _)
  io.posIdx_s1.set    := PriorityEncoder(setVec_s1)
  io.posIdx_s1.way    := PriorityMux(setVec_s1, posSetIo.map(_.posWay_s1.bits))
  // wakeup
  io.wakeup.valid     := wakeupVec_s1.reduce(_ | _)
  io.wakeup.bits.addr := PriorityMux(wakeupVec_s1, posSetIo.map(_.wakeup.bits.addr))
  // HAssert
  HAssert(PopCount(setVec_s1) <= 1.U)
  HAssert(PopCount(wakeupVec_s1) <= 1.U)

  /*
   * already use PoS num
   */
  io.alrUsePoS := PopCount(posSetIo.flatMap(_.stateVec.map(_.valid)))

  /*
   * Get Addr
   */
  val addrVec2 = Wire(Vec(posSets, Vec(posWays, UInt(addrBits.W))))
  addrVec2.zip(posSetIo.map(_.stateVec.map(_.bits.addr))).foreach { case(a, b) => a := b }
  io.getAddrVec.foreach { get =>
    get.result.addr := addrVec2(get.pos.set)(get.pos.way)
  }
  dontTouch(addrVec2)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}
