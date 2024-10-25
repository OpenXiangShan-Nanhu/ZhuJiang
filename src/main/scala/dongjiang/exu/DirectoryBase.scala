package dongjiang.pcu.exu

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import chisel3.{util, _}
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.sram.SRAMTemplate
import chisel3.util.random.LFSR
import freechips.rocketchip.util.ReplacementPolicy
import dongjiang.utils.SramWrapper

object DirCtrlState {
  // [Free] ---> [ReqFire] ---> [WaitMcp]  ---> [GetResp]
  //   ReplSram: [writeRepl]    [readRepl]      [getRepl]
  //                      MSHR: [readMSHR]      [mshrResp]
  //
  //
  // [Free] ---> [ReqFire] ---> [WaitMcp]  ---> [GetResp]
  //                            [EarlyReq] ---> [ReqFire] --->  [WaitMcp] ---> [GetResp] ---> [Free]
  //
  // [Free] ---> [ReqFire] ---> [WaitMcp]  ---> [GetResp]
  //                                            [EarlyReq] ---> [ReqFire] ---> [WaitMcp]  ---> [GetResp] ---> [Free]
  val width               = 3
  val Free                = "b000".U
  val ReqFire             = "b100".U
  val WaitMcp             = "b010".U
  val GetResp             = "b001".U
  val GetResp_ReqFire     = "b101".U

}

trait HasDirCtrlState {
  val shift = UInt(DirCtrlState.width.W)

// TODO:  def canRecReq = shift === DirCtrlState.Free    | shift === DirCtrlState.WaitMcp         | shift === DirCtrlState.GetResp
  def canRecReq = shift === DirCtrlState.Free    | shift === DirCtrlState.GetResp
  def isReqFire = shift === DirCtrlState.ReqFire | shift === DirCtrlState.GetResp_ReqFire
  def isWaitMcp = shift === DirCtrlState.WaitMcp
  def isGetResp = shift === DirCtrlState.GetResp | shift === DirCtrlState.GetResp_ReqFire

}

class DirCtrlBundle(setBits: Int)(implicit p: Parameters) extends DJBundle with HasDirCtrlState with HasMSHRWay with HasPipeID {
  val ren       = Bool()
  val set       = UInt(setBits.W)

  def wen       = !ren
  def mSet(dirBank: UInt) = Cat(set, dirBank)(mshrSetBits-1, 0)
  require((setBits + dirBankBits) >= mshrSetBits)
}


class DirEntry(tagBits: Int, nrMetas: Int = 1)(implicit p: Parameters) extends DJBundle {
  val tag         = UInt(tagBits.W)
  val bank        = UInt(bankBits.W)
  val metaVec     = Vec(nrMetas, new CHIStateBundle())
}

class DirectoryBase(
                      tagBits: Int,
                      sets: Int,
                      ways: Int = 4,
                      nrMetas: Int = 1,
                      replPolicy: String = "plru",
                      mcp: Int = 2, // TODO
                      holdMcp: Boolean = true, // TODO
                      nrWayBank: Int = 1,
                   )
  (implicit p: Parameters) extends DJModule {

  require(mcp == 2 & holdMcp)
  require(nrWayBank < ways)

  val repl        = ReplacementPolicy.fromString(replPolicy, ways)
  val useRepl     = replPolicy != "random"
  val replWayBits = if(useRepl) repl.nBits else 0
  val setBits     = log2Ceil(sets)
  val wayBits     = log2Ceil(ways)

  def parseDirAddress(x: UInt): (UInt, UInt, UInt, UInt, UInt) = parseAddress(x, dirBankBits, setBits, tagBits)

// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val id        = Input(UInt(dirBankBits.W))
    val earlyRReq = Flipped(Decoupled())
    val earlyWReq = Flipped(Decoupled())
    val dirRead   = Input(new DirReadBundle)
    val dirWrite  = Input(new DirWriteBaseBundle(ways, nrMetas, replWayBits))
    val dirResp   = Valid(new DirRespBaseBundle(ways, nrMetas, replWayBits))
    val readMshr  = Valid(new DirReadMSHRBundle())
    val mshrResp  = Input(new MSHRRespDirBundle())
  })

// --------------------- Modules declaration ------------------------//
  val metaArrays    = Seq.fill(nrWayBank) { Module(new SRAMTemplate(new DirEntry(tagBits, nrMetas), sets, ways / nrWayBank, singlePort = true, shouldReset = true, multicycle = mcp, holdMcp = holdMcp)) }

  val replArrayOpt  = if(!useRepl) None else Some(Module(new SRAMTemplate(UInt(repl.nBits.W), sets, way = 1, singlePort = true, shouldReset = true)))


//// ----------------------- Reg/Wire declaration --------------------------//
  val resetDone       = RegInit(false.B)
  // Base
  val sramCtrlReg     = RegInit(0.U.asTypeOf(new DirCtrlBundle(setBits)))
  // s2
  val valid_s2        = WireInit(false.B)
  val metaResp_s2     = Wire(Vec(ways, new DirEntry(tagBits, nrMetas)))
  val replResp_s2     = WireInit(0.U(repl.nBits.W))
  val addr_s2         = WireInit(0.U(addressBits.W))
  val mshrMes_s2      = Wire(Vec(djparam.nrMSHRWays, Valid(new Bundle {
    val tag           = UInt(tagBits.W)
    val bank          = UInt(bankBits.W)
  })))
  val pipeId_s2       = Wire(UInt(PipeID.width.W))
  // s3
  val valid_s3_g      = RegInit(false.B)
  val metaResp_s3_g   = Reg(Vec(ways, new DirEntry(tagBits, nrMetas)))
  val addr_s3_g       = RegInit(0.U(addressBits.W))
  val mshrMes_s3_g    = Reg(Vec(djparam.nrMSHRWays, Valid(new Bundle{
    val tag           = UInt(tagBits.W)
    val bank          = UInt(bankBits.W)
  })))
  val replResp_s3_g   = RegInit(0.U(repl.nBits.W))
  val selInvWayVec    = Wire(Vec(ways, Bool()))
  val hitWayVec       = Wire(Vec(ways, Bool()))
  val replWay         = WireInit(0.U(wayBits.W))
  val tag_s3          = WireInit(0.U(tagBits.W))
  val set_s3          = WireInit(0.U(setBits.W))
  val bank_s3         = WireInit(0.U(bankBits.W))
  val useWayVec       = Wire(Vec(ways, Bool()))
  val pipeId_s3_g     = Reg(UInt(PipeID.width.W))



// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- S1: Read / Write SRAM --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //

  /*
   * Check Reset Done
   */
  when(metaArrays.map { case m => m.io.w.req.ready & m.io.r.req.ready }.reduce(_ & _)){
    if(useRepl) {
      when(replArrayOpt.get.io.w.req.ready & replArrayOpt.get.io.r.req.ready) {
        resetDone := true.B
      }
    } else {
      resetDone := true.B
    }
  }

  /*
   * Parse Req Addr
   */
  val (rTag, rSet, rModBank, rBank, rOffset) = parseDirAddress(io.dirRead.addr)
  val (wTag, wSet, wModBank, wBank, wOffset) = parseDirAddress(io.dirWrite.addr)
  assert(Mux(RegNext(io.earlyRReq.fire), io.id === rModBank, true.B))
  assert(Mux(RegNext(io.earlyWReq.fire), io.id === wModBank, true.B))


  /*
   * Set SramCtrl Value
   */
  sramCtrlReg.shift       := Cat(io.earlyRReq.fire | io.earlyWReq.fire, sramCtrlReg.shift(DirCtrlState.width-1, 1))
  sramCtrlReg.ren         := Mux(io.earlyRReq.fire, true.B, Mux(io.earlyWReq.fire, false.B, sramCtrlReg.ren))
  when(sramCtrlReg.isReqFire) {
    sramCtrlReg.set       := rSet
    sramCtrlReg.mshrWay   := io.dirRead.mshrWay
    sramCtrlReg.pipeId    := io.dirRead.pipeId
  }
  assert(!(io.earlyRReq.fire & io.earlyWReq.fire))


  /*
   * Get Req Form MSHR or ProcessPipe_S3 EXU
   */
  io.earlyRReq.ready        := sramCtrlReg.canRecReq & !io.earlyWReq.valid & resetDone
  io.earlyWReq.ready        := sramCtrlReg.canRecReq & resetDone


  /*
   * Read / Write Req SRAM
   */
  metaArrays.zipWithIndex.foreach {
    case (m, i) =>
      // early
      m.io.earlyRen.get       := io.earlyRReq.fire
      m.io.earlyWen.get       := io.earlyWReq.fire
      // ren
      m.io.r.req.valid        := sramCtrlReg.isReqFire & sramCtrlReg.ren
      m.io.r.req.bits.setIdx  := rSet
      // wen
      m.io.w.req.valid        := sramCtrlReg.isReqFire & sramCtrlReg.wen & (i * ways/nrWayBank).U <= OHToUInt(io.dirWrite.wayOH) & OHToUInt(io.dirWrite.wayOH) <= ((i+1) * ways/nrWayBank - 1).U
      m.io.w.req.bits.setIdx  := wSet
      m.io.w.req.bits.data.foreach(_.tag      := wTag)
      m.io.w.req.bits.data.foreach(_.bank     := wBank)
      m.io.w.req.bits.data.foreach(_.metaVec  := io.dirWrite.metaVec)
      m.io.w.req.bits.waymask.get             := io.dirWrite.wayOH >> (i * ways/nrWayBank).U
  }

  when(sramCtrlReg.isReqFire & sramCtrlReg.ren) { assert(metaArrays.map(_.io.r.req.ready).reduce(_ & _)) }
  when(sramCtrlReg.isReqFire & sramCtrlReg.wen) { assert(metaArrays.map(_.io.w.req.ready).reduce(_ & _)) }


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- S2: Receive SRAM Resp ---------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Read Repl SRAM
   */
  if(useRepl) {
    replArrayOpt.get.io.r.req.valid       := sramCtrlReg.isWaitMcp & sramCtrlReg.ren
    replArrayOpt.get.io.r.req.bits.setIdx := sramCtrlReg.set
  }

  /*
   * Read MSHR Set Mes
   */
  io.readMshr.valid           := sramCtrlReg.isWaitMcp & sramCtrlReg.ren
  io.readMshr.bits.mshrSet    := sramCtrlReg.mSet(io.id)
  io.readMshr.bits.pipeId     := sramCtrlReg.pipeId
  io.readMshr.bits.dirBankId  := DontCare

  /*
   * Receive Pipe Id
   */
  pipeId_s2       := sramCtrlReg.pipeId

  /*
   * Receive Meta SRAM resp
   */
  valid_s2        := sramCtrlReg.isGetResp & sramCtrlReg.ren
  metaArrays.zipWithIndex.foreach {
    case (m, i) =>
      m.io.r.resp.data.zipWithIndex.foreach {
        case(d, j) =>
          metaResp_s2((i*(ways/nrWayBank))+j) := Mux(valid_s2, d, 0.U.asTypeOf(d))
      }
  }

  /*
   * Receive Repl SRAM resp
   */
  if (useRepl) {
    replResp_s2   := replArrayOpt.get.io.r.resp.data(0)
  }

  /*
   * Receive MSHR Repl
   */
  mshrMes_s2.zip(io.mshrResp.addrs).foreach {
    case (a, b) =>
      a.valid     := b.valid
      a.bits.tag  := parseDirAddress(b.bits)._1
      a.bits.bank := parseDirAddress(b.bits)._4
  }
  addr_s2         := io.mshrResp.addrs(sramCtrlReg.mshrWay).bits
  assert(Mux(sramCtrlReg.isGetResp & sramCtrlReg.ren, io.mshrResp.addrs(sramCtrlReg.mshrWay).valid, true.B))


// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- S3: Output DirResp  ----------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive S2
   */
  valid_s3_g      := valid_s2
  metaResp_s3_g   := metaResp_s2
  addr_s3_g       := addr_s2
  mshrMes_s3_g.zip(mshrMes_s2).foreach { case(a, b) => a := b }
  replResp_s3_g   := replResp_s2
  pipeId_s3_g     := pipeId_s2

  tag_s3          := parseDirAddress(addr_s3_g)._1
  set_s3          := parseDirAddress(addr_s3_g)._2
  bank_s3         := parseDirAddress(addr_s3_g)._4


  /*
   * Get Hit Vec and Hit State
   */
  val tagHitVec   = metaResp_s3_g.map(_.tag === tag_s3)
  val bankHitVec  = metaResp_s3_g.map(_.bank === bank_s3)
  val stateHitVec = metaResp_s3_g.map(_.metaVec.map(!_.isInvalid).reduce(_ | _))
  val hitMetaVec  = metaResp_s3_g(OHToUInt(hitWayVec)).metaVec
  val hit         = hitWayVec.asUInt.orR
  hitWayVec       := tagHitVec.zip(bankHitVec.zip(stateHitVec)).map{ case(t, (b, s)) => t & b & s }
  assert(PopCount(hitWayVec) <= 1.U)


  /*
   * Selet one invalid way
   */
  val invWayVec     = stateHitVec.map(!_)
  val hasInvWay     = invWayVec.reduce(_ | _)
  selInvWayVec      := PriorityEncoderOH(invWayVec)
  val invMetasVec   = Wire(Vec(nrMetas, new CHIStateBundle())); invMetasVec.foreach(_.state := ChiState.I)

  /*
   * Select one replace way
   */
  if (!useRepl) {
   replWay := LFSR(wayBits) // random
  } else {
   replWay := repl.get_replace_way(replResp_s3_g) // replace
  }
  val replWayAddr     = Cat(metaResp_s3_g(replWay).tag, set_s3, io.id, metaResp_s3_g(replWay).bank, 0.U(offsetBits.W)); require(replWayAddr.getWidth == addressBits)


  /*
   * repl way is conflict with unuse way
   * When noUseWay is required, all ways are not Invalid by default
   */
  useWayVec           := metaResp_s3_g.map { case meta => mshrMes_s3_g.map { case mshr => mshr.valid & mshr.bits.tag === meta.tag & mshr.bits.bank === meta.bank }.reduce(_ | _) }
  val replWayIsUsing  = useWayVec(replWay)
  val selUnuseWay     = PriorityEncoder(useWayVec.map(!_))
  val replRetry       = useWayVec.asUInt.andR
  val unUseWayAddr    = Cat(metaResp_s3_g(selUnuseWay).tag, set_s3, io.id,  metaResp_s3_g(selUnuseWay).bank, 0.U(offsetBits.W)); require(unUseWayAddr.getWidth == addressBits)


  /*
   * Output Resp
   */
  io.dirResp.valid          := valid_s3_g
  io.dirResp.bits.hit       := hit
  // [Resp Mes]                         [Hit Way Mes]                      [Invalid Way Mes]                        [Unuse Way Mes]                     [Replace Way Mes]
  io.dirResp.bits.wayOH     := Mux(hit, hitWayVec.asUInt,   Mux(hasInvWay, selInvWayVec.asUInt, Mux(replWayIsUsing, UIntToOH(selUnuseWay),              UIntToOH(replWay))))
  io.dirResp.bits.addr      := Mux(hit, addr_s3_g,          Mux(hasInvWay, 0.U,                 Mux(replWayIsUsing, unUseWayAddr,                       replWayAddr)))
  io.dirResp.bits.metaVec   := Mux(hit, hitMetaVec,         Mux(hasInvWay, invMetasVec,         Mux(replWayIsUsing, metaResp_s3_g(selUnuseWay).metaVec, metaResp_s3_g(replWay).metaVec)))
  io.dirResp.bits.replRetry := Mux(hit, false.B,            Mux(hasInvWay, false.B,             Mux(replWayIsUsing, replRetry,                          false.B)))
  io.dirResp.bits.pipeId    := pipeId_s3_g
  if(useRepl) { io.dirResp.bits.replMes := replResp_s3_g }


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- Update Replace SRAM Mes  --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * PLRU: update replacer only when read hit or write Dir
   */
  if (replPolicy == "plru") {
    replArrayOpt.get.io.w.req.valid               := metaArrays.map(_.io.w.req.fire).reduce(_ | _) | (io.dirResp.fire & hit)
    replArrayOpt.get.io.w.req.bits.setIdx         := Mux(metaArrays.map(_.io.w.req.fire).reduce(_ | _), wSet, set_s3)
    replArrayOpt.get.io.w.req.bits.data.foreach(_ := Mux(metaArrays.map(_.io.w.req.fire).reduce(_ | _),
                                                       repl.get_next_state(io.dirWrite.replMes, OHToUInt(io.dirWrite.wayOH)),
                                                       repl.get_next_state(replResp_s3_g, OHToUInt(io.dirResp.bits.wayOH))))
  } else if(replPolicy == "random") {
    // nothing to do
  } else {
    assert(false.B, "Dont support replacementPolicy except plru or random")
  }

}