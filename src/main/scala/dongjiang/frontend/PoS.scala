package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._

class PoS(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config    = new DJConfigIO()
    // req and ack
    val reqIn     = Flipped(Valid(new DJBundle with HasAddr {
      val isSnp   = Bool()
    }))
    val sleepOut  = Output(Bool())
    val retryOut  = Output(Bool())
    val posIdxOut = Output(new PosIndex())
    // retry from block
    val retryIn   = Input(Bool())
    // update PoS
    val canNest   = Input(Valid(new PosIndex()))
    val updTag    = Input(Valid(new Addr with HasPosIndex))
    val clean     = Input(Vec(2, Valid(new DJBundle with HasPosIndex {
      val isSnp   = Bool()
    })))
    // wakeup TaskBuf Entry
    val wakeupVec = Vec(2, Valid(new Addr))
  })

  HardwareAssertion(!io.reqIn.valid)

  /*
   * REG and Wire declaration
   */
  val tagTable      = Reg(Vec(posSets, Vec(posWays, UInt(posTagBits.W))))
  val ctrlTable     = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { 0.U.asTypeOf(new DJBundle {
    val validVec    = UInt(2.W)
    val canNest     = Bool()

    def hasReq      = validVec(0)
    def hasSnp      = validVec(1)
    def valid       = validVec.orR
  }) }) }))
  val reqInReg      = Reg(Valid(new DJBundle with HasPosIndex {
    val tag         = UInt(posTagBits.W)
    val isSnp       = Bool()
  }))


  /*
   * Receive req from taskBuf
   *
   * 1. req is CHIREQ: not block by addr
   * 2. req is CHISNP:
   *  a. not block by addr
   *  b. nest someone
   */
  // get block message
  val reqSet      = io.reqIn.bits.posSet
  val ctrlSet     = ctrlTable(reqSet)
  val addrSet     = tagTable(reqSet)
  val posValidVec = ctrlSet.map(_.valid)
  val tagMatchVec = addrSet.map(_ === io.reqIn.bits.posTag)
  val matchVec    = posValidVec.zip(tagMatchVec).map { case(a, b) => a & b }
  // judge nest
  val nestVec     = matchVec.zip(ctrlSet.map(_.canNest)).map { case(a, b) => a & b }
  val nestWay     = PriorityEncoder(nestVec)
  HardwareAssertion.withEn(!ctrlSet(nestWay).hasSnp, io.reqIn.valid & io.reqIn.bits.isSnp)
  // get free way
  val freeWayVec  = ctrlSet.map(!_.valid)
  val hasFreeWay  = freeWayVec.reduce(_ | _)
  val freeWay     = PriorityEncoder(freeWayVec)
  // judge block
  val hasMatch    = matchVec.reduce(_ | _)
  val snpNest     = nestVec.reduce(_ | _) & io.reqIn.bits.isSnp
  val receive     = Mux(hasMatch, snpNest, hasFreeWay)

  /*
   * Store req from taskBuf
   */
  reqInReg.valid        := io.reqIn.valid & receive
  reqInReg.bits.tag     := io.reqIn.bits.posTag
  reqInReg.bits.isSnp   := io.reqIn.bits.isSnp
  reqInReg.bits.pos.set := reqSet
  reqInReg.bits.pos.way := Mux(snpNest, nestWay, freeWay)

  /*
   * Retrun ack to taskBuf and block
   */
  io.sleepOut   := RegNext(io.reqIn.valid & !receive & !io.reqIn.bits.isSnp)
  io.retryOut   := RegNext(io.reqIn.valid & !receive)
  io.posIdxOut  := reqInReg.bits.pos
  HardwareAssertion.withEn(io.sleepOut, io.retryOut)

  /*
   * Modify pos tag
   */
  tagTable.zipWithIndex.foreach {
    case(tagSet, i) =>
      tagSet.zipWithIndex.foreach {
        case(tag, j) =>
          val updHit  = io.updTag.valid & io.updTag.bits.pos.idxMatch(i, j)
          val reqHit  = reqInReg.valid  & reqInReg.bits.pos.idxMatch(i , j) & !io.retryIn
          when(updHit) {
            tag := io.updTag.bits.posTag
          }.elsewhen(reqHit) {
            tag := reqInReg.bits.tag
          }
          HardwareAssertion(!(updHit & reqHit))
      }
  }

  /*
   * Modify pos ctrl
   */
  ctrlTable.zipWithIndex.foreach {
    case (ctrlSet, i) =>
      ctrlSet.zipWithIndex.foreach {
        case (ctrl, j) =>
          // receive clean
          val cleanHitVec = io.clean.map(c => c.valid & c.bits.pos.idxMatch(i, j))
          val cleanHit    = cleanHitVec.reduce(_ | _)
          val cleanSnp    = io.clean(PriorityEncoder(cleanHitVec)).bits.isSnp
          // store req
          val reqHit      = reqInReg.valid & reqInReg.bits.pos.idxMatch(i, j) & !io.retryIn
          val reqIsSnp    = reqInReg.bits.isSnp
          // modify validVec
          val validVecNxt = WireInit(0.U(2.W))
          ctrl.validVec   := validVecNxt
          when(cleanHit) {
            validVecNxt   := ctrl.validVec & Mux(cleanSnp, "b01".U, "b10".U)
          }.elsewhen(reqHit) {
            validVecNxt   := ctrl.validVec | Mux(reqIsSnp, "b10".U, "b01".U)
          }
          // modify canNest
          val canNestHit  = io.canNest.valid & io.canNest.bits.idxMatch(i, j)
          ctrl.canNest    := canNestHit
          // assert
          HardwareAssertion.withEn(PopCount(ctrl.validVec ^ validVecNxt) === 1.U, cleanHit | reqHit,
                                                                    desc = cf"PoS Index[${i}][${j}]")
          HardwareAssertion(PopCount(cleanHitVec) <= 1.U,           desc = cf"PoS Index[${i}][${j}]")
          HardwareAssertion(PopCount(Seq(cleanHit, reqHit)) <= 1.U, desc = cf"PoS Index[${i}][${j}]")
          HardwareAssertion.withEn(!ctrl.canNest, canNestHit,       desc = cf"PoS Index[${i}][${j}]")
      }
  }

  /*
   * wakeup someone by addr
   */
  io.wakeupVec.zip(io.clean).foreach {
    case(wakeup, clean) =>
      // get wakeup addr
      val cleanWay = clean.bits.pos.way
      val cleanSet = clean.bits.pos.set
      val cleanTag = tagTable(cleanSet)(cleanWay)
      wakeup.bits.catPoS(io.config.bankId, cleanTag, cleanSet, dirBank.U(dirBankBits.W))
      // get wakeup valid
      wakeup.valid := clean.valid & ctrlTable(cleanSet)(cleanWay).validVec.xorR
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}