package dongjiang.pcu.exu

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import chisel3.{util, _}
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.sram.{SinglePortSramTemplate, DualPortSramTemplate}
import chisel3.util.random.LFSR
import freechips.rocketchip.util.ReplacementPolicy
import dongjiang.utils.{fastDecoupledQueue, DecoupledQueue}
import xs.utils.debug.{DomainInfo, HardwareAssertion}


class DirCtrlBundle(implicit p: Parameters) extends DJBundle with HasMHSRIndex with HasPipeID

class DirEntry(tagBits: Int, nrMetas: Int = 1)(implicit p: Parameters) extends DJBundle {
  val tag         = UInt(tagBits.W)
  val metaVec     = Vec(nrMetas, new CHIStateBundle())
}

class DirectoryBase(
                      dirType:    String,
                      tagBits:    Int,
                      sets:       Int,
                      ways:       Int,
                      nrMetas:    Int = 1,
                      replPolicy: String = "plru",
                      setup:      Int = 1,
                      latency:    Int = 1,
                      extraHold:  Boolean = false,
                      nrWayBank:  Int = 1,
                   )
  (implicit p: Parameters) extends DJModule {

  require(nrWayBank < ways)
  require(dirType == "self" | dirType == "sf")

  val repl        = ReplacementPolicy.fromString(replPolicy, ways)
  val useRepl     = replPolicy != "random"
  val replWayBits = if(useRepl) repl.nBits else 0
  val setBits     = log2Ceil(sets)
  val wayBits     = log2Ceil(ways)

  print(s"DirectoryBase: dirType[${dirType}]\n")

// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val dirBank   = Input(UInt(dirBankBits.W))
    val dirRead   = Flipped(Decoupled(new DirReadBundle()))
    val dirWrite  = Flipped(Decoupled(new DirWriteBaseBundle(ways, nrMetas, replWayBits)))
    val dirResp   = Valid(new DirRespBaseBundle(ways, nrMetas, replWayBits))
    val readMshr  = Valid(new DirReadMSHRBundle())
    val mshrResp  = Input(new MSHRRespDirBundle())
  })

// --------------------- Modules declaration ------------------------//
  val metaArrays      = Seq.fill(nrWayBank) { Module(new SinglePortSramTemplate(new DirEntry(tagBits, nrMetas), sets, ways / nrWayBank, shouldReset = true, setup = setup, latency = latency, extraHold = extraHold)) }

  val replArrayOpt    = if(!useRepl) None else Some(Module(new DualPortSramTemplate(UInt(repl.nBits.W), sets, way = 1, shouldReset = true, setup = setup, latency = latency, extraHold = extraHold)))

  val rCtrlPipe       = Module(new Pipe(new DirCtrlBundle(), latency = setup + latency - 1))

  val updReplQOpt     = if(!useRepl) None else Some(Module(new DecoupledQueue(new Bundle { val set = UInt(setBits.W); val way = UInt(wayBits.W); val replMes = UInt(repl.nBits.W) })))

// ----------------------- Reg/Wire declaration --------------------------//
  val resetDone       = RegInit(false.B)
  val updReplByHit    = WireInit(true.B)
  val replRReady      = WireInit(true.B)
  val replWReady      = WireInit(true.B)
  if(useRepl) {
    replRReady        := replArrayOpt.get.io.rreq.ready
    replWReady        := replArrayOpt.get.io.wreq.ready
  }
  // s1
  val dirRead         = Wire(Decoupled(new DirReadBundle()))
  val dirWrite        = Wire(Decoupled(new DirWriteBaseBundle(ways, nrMetas, replWayBits)))
  val rTag_s1         = Wire(UInt(tagBits.W))
  val rSet_s1         = Wire(UInt(setBits.W))
  val rDirBank_s1     = Wire(UInt(dirBankBits.W))
  val wTag_s1         = Wire(UInt(tagBits.W))
  val wSet_s1         = Wire(UInt(setBits.W))
  val wDirBank_s1     = Wire(UInt(dirBankBits.W))
  // s2
  val valid_s2        = WireInit(false.B)
  val rCtrl_s2_g      = RegInit(0.U.asTypeOf(new DirCtrlBundle()))
  val metaResp_s2     = Wire(Vec(ways, new DirEntry(tagBits, nrMetas)))
  val replResp_s2     = WireInit(0.U(repl.nBits.W))
  val addr_s2         = WireInit(0.U(useAddrBits.W))
  val mshrMes_s2      = Wire(Vec(djparam.nrMSHRWays, Valid(UInt(tagBits.W))))
  // s3
  val valid_s3_g      = RegInit(false.B)
  val metaResp_s3_g   = Reg(Vec(ways, new DirEntry(tagBits, nrMetas)))
  val addr_s3_g       = RegInit(0.U(useAddrBits.W))
  val mshrMes_s3_g    = Reg(Vec(djparam.nrMSHRWays, Valid(UInt(tagBits.W))))
  val replResp_s3_g   = RegInit(0.U(repl.nBits.W))
  val selInvWayVec    = Wire(Vec(ways, Bool()))
  val hitWayVec       = Wire(Vec(ways, Bool()))
  val replWay         = WireInit(0.U(wayBits.W))
  val tag_s3          = WireInit(0.U(tagBits.W))
  val set_s3          = WireInit(0.U(setBits.W))
  val useWayVec       = Wire(Vec(ways, Bool()))
  val pipeId_s3_g     = Reg(UInt(PipeID.width.W))

  /*
   * Check Reset Done
   */
  when(metaArrays.map { case m => m.io.req.ready }.reduce(_ & _)) {
    if (useRepl) {
      when(replArrayOpt.get.io.wreq.ready & replArrayOpt.get.io.rreq.ready) {
        resetDone := true.B
      }
    } else {
      resetDone := true.B
    }
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- S1: Read / Write SRAM --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive directory input
   */
  dirWrite      <> fastDecoupledQueue(io.dirWrite)
  dirRead       <> io.dirRead

  /*
   * Parse Req Addr
   */
  if(dirType == "self"){
    rTag_s1 := dirRead.bits.sTag;   rSet_s1 := dirRead.bits.sSet;   rDirBank_s1 := dirRead.bits.dirBank
    wTag_s1 := dirWrite.bits.sTag;  wSet_s1 := dirWrite.bits.sSet;  wDirBank_s1 := dirWrite.bits.dirBank
  } else {
    rTag_s1 := dirRead.bits.sfTag;  rSet_s1 := dirRead.bits.sfSet;  rDirBank_s1 := dirRead.bits.dirBank
    wTag_s1 := dirWrite.bits.sfTag; wSet_s1 := dirWrite.bits.sfSet; wDirBank_s1 := dirWrite.bits.dirBank
  }
  HardwareAssertion.withEn(io.dirBank === rDirBank_s1, dirRead.valid)
  HardwareAssertion.withEn(io.dirBank === wDirBank_s1, dirWrite.valid)

  /*
   * Set SramCtrl Value
   */
  rCtrlPipe.io.enq.valid        := dirRead.fire
  rCtrlPipe.io.enq.bits.mshrSet := dirRead.bits.mSet
  rCtrlPipe.io.enq.bits.mshrWay := dirRead.bits.mshrWay
  rCtrlPipe.io.enq.bits.pipeID  := dirRead.bits.pipeID

  /*
   * Get Req Form MSHR or ProcessPipe_S3 EXU
   */
  val wMetaCango      = !updReplByHit & replWReady
  val rMetaCango      = !dirWrite.valid & replRReady
  dirWrite.ready      := metaArrays.map(_.io.req.ready).reduce(_ & _) & wMetaCango
  dirRead.ready       := metaArrays.map(_.io.req.ready).reduce(_ & _) & rMetaCango


  /*
   * Read / Write Req SRAM
   */
  metaArrays.zipWithIndex.foreach {
    case (m, i) =>
      val writeHit        = dirWrite.valid & wMetaCango & (i * ways/nrWayBank).U <= OHToUInt(dirWrite.bits.wayOH) & OHToUInt(dirWrite.bits.wayOH) <= ((i+1) * ways/nrWayBank - 1).U
      val readHit         = dirRead.valid  & rMetaCango

      m.io.req.valid      := writeHit| readHit
      m.io.req.bits.addr  := Mux(writeHit, wSet_s1, rSet_s1)
      m.io.req.bits.write := writeHit
      m.io.req.bits.data.foreach(_.tag      := wTag_s1)
      m.io.req.bits.data.foreach(_.metaVec  := dirWrite.bits.metaVec)
      m.io.req.bits.mask.get                := dirWrite.bits.wayOH >> (i * ways/nrWayBank).U
  }
  HardwareAssertion.withEn(PopCount(metaArrays.map(_.io.req.fire)) === 1.U, dirWrite.fire)

  if (useRepl) {
    replArrayOpt.get.io.rreq.valid  := dirRead.fire
    replArrayOpt.get.io.rreq.bits   := rSet_s1
    HardwareAssertion.withEn(replArrayOpt.get.io.rreq.fire, dirRead.fire)
    HardwareAssertion.withEn(dirRead.fire, replArrayOpt.get.io.rreq.fire)
  }





// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------ S2: Receive SRAM Resp And Read MSHR Set ----------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Read MSHR Set Mes
   */
  io.readMshr.valid           := rCtrlPipe.io.deq.valid
  io.readMshr.bits.mshrSet    := rCtrlPipe.io.deq.bits.mshrSet
  io.readMshr.bits.pipeID     := rCtrlPipe.io.deq.bits.pipeID
  io.readMshr.bits.dirBank    := io.dirBank

  /*
   * Receive rCtrl Mes
   */
  rCtrl_s2_g  := Mux(rCtrlPipe.io.deq.valid, rCtrlPipe.io.deq.bits, 0.U.asTypeOf(rCtrlPipe.io.deq.bits))

  /*
   * Receive Meta SRAM resp
   */
  valid_s2    := metaArrays(0).io.resp.valid
  metaArrays.zipWithIndex.foreach {
    case (m, i) =>
      m.io.resp.bits.data.zipWithIndex.foreach {
        case(d, j) =>
          metaResp_s2((i*(ways/nrWayBank))+j) := Mux(valid_s2, d, 0.U.asTypeOf(d))
      }
  }
  HardwareAssertion.withEn(metaArrays.map(_.io.resp.valid).reduce(_ & _), RegNext(rCtrlPipe.io.deq.valid))
  HardwareAssertion.withEn(RegNext(rCtrlPipe.io.deq.valid), metaArrays.map(_.io.resp.valid).reduce(_ & _))

  /*
   * Receive Repl SRAM resp
   */
  if (useRepl) {
    replResp_s2   := replArrayOpt.get.io.rresp.bits(0)
    HardwareAssertion(!(valid_s2 ^ replArrayOpt.get.io.rresp.valid))
  }

  /*
   * Receive MSHR Resp
   */
  mshrMes_s2.zip(io.mshrResp.addrs).foreach {
    case (a, b) =>
      a.valid     := b.valid
      if(dirType == "self") a.bits := parseSelfAddr(b.bits)._1
      else                  a.bits := parseSFAddr(b.bits)._1
  }
  addr_s2         := io.mshrResp.addrs(rCtrl_s2_g.mshrWay).bits
  HardwareAssertion.withEn(io.mshrResp.addrs(rCtrl_s2_g.mshrWay).valid, valid_s2)


// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- S3: Output DirResp  ----------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive S2
   */
  valid_s3_g      := valid_s2
  metaResp_s3_g   := metaResp_s2
  addr_s3_g       := addr_s2
  replResp_s3_g   := replResp_s2
  pipeId_s3_g     := rCtrl_s2_g.pipeID
  mshrMes_s3_g.zip(mshrMes_s2).foreach { case(a, b) => a := b }

  if(dirType == "self") { tag_s3 := parseSelfAddr(addr_s3_g)._1; set_s3 := parseSelfAddr(addr_s3_g)._2 }
  else                  { tag_s3 := parseSFAddr(addr_s3_g)._1;   set_s3 := parseSFAddr(addr_s3_g)._2 }


  /*
   * Get Hit Vec and Hit State
   */
  val tagHitVec   = metaResp_s3_g.map(_.tag === tag_s3)
  val stateHitVec = metaResp_s3_g.map(_.metaVec.map(!_.isInvalid).reduce(_ | _))
  val hitMetaVec  = metaResp_s3_g(OHToUInt(hitWayVec)).metaVec
  val hit         = hitWayVec.asUInt.orR
  hitWayVec       := tagHitVec.zip(stateHitVec).map{ case(t, s) => t & s }
  HardwareAssertion(PopCount(hitWayVec) <= 1.U)


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
  val replWayAddr   = Cat(metaResp_s3_g(replWay).tag, set_s3, io.dirBank)
  // print(s"${replWayAddr.getWidth} = ${metaResp_s3_g(replWay).tag.getWidth} + ${set_s3.getWidth} + ${io.dirBank.getWidth} = ${useAddrBits}\n")
  require(replWayAddr.getWidth == useAddrBits)


  /*
   * repl way is conflict with unuse way
   * When noUseWay is required, all ways are not Invalid by default
   */
  useWayVec           := metaResp_s3_g.map { case meta => mshrMes_s3_g.map { case mshr => mshr.valid & mshr.bits === meta.tag }.reduce(_ | _) }
  val replWayIsUsing  = useWayVec(replWay)
  val selUnuseWay     = PriorityEncoder(useWayVec.map(!_)) // TODO
  val replRetry       = useWayVec.asUInt.andR
  val unUseWayAddr    = Cat(metaResp_s3_g(selUnuseWay).tag, set_s3, io.dirBank); require(unUseWayAddr.getWidth == useAddrBits)


  /*
   * Output Resp
   */
  io.dirResp.valid          := valid_s3_g
  io.dirResp.bits.hit       := hit
  // TODO: block same set
  // [Resp Mes]                         [Hit Way Mes]                      [Invalid Way Mes]                        [Unuse Way Mes]                     [Replace Way Mes]
  io.dirResp.bits.wayOH     := Mux(hit, hitWayVec.asUInt,   Mux(hasInvWay, selInvWayVec.asUInt, Mux(replWayIsUsing, UIntToOH(selUnuseWay),              UIntToOH(replWay))))
  io.dirResp.bits.useAddr   := Mux(hit, addr_s3_g,          Mux(hasInvWay, 0.U,                 Mux(replWayIsUsing, unUseWayAddr,                       replWayAddr)))
  io.dirResp.bits.metaVec   := Mux(hit, hitMetaVec,         Mux(hasInvWay, invMetasVec,         Mux(replWayIsUsing, metaResp_s3_g(selUnuseWay).metaVec, metaResp_s3_g(replWay).metaVec)))
  io.dirResp.bits.replRetry := Mux(hit, false.B,            Mux(hasInvWay, false.B,             Mux(replWayIsUsing, replRetry,                          false.B)))
  io.dirResp.bits.pipeID    := pipeId_s3_g
  if(useRepl) { io.dirResp.bits.replMes := replResp_s3_g }

  /*
   * update repl mes
   */
  if(useRepl) {
    updReplQOpt.get.io.enq.valid        := io.dirResp.fire & io.dirResp.bits.hit
    updReplQOpt.get.io.enq.bits.set     := set_s3
    updReplQOpt.get.io.enq.bits.way     := OHToUInt(io.dirResp.bits.wayOH)
    updReplQOpt.get.io.enq.bits.replMes := replResp_s3_g
    updReplQOpt.get.io.deq.ready        := replArrayOpt.get.io.wreq.ready
    updReplByHit                        := updReplQOpt.get.io.deq.valid
    HardwareAssertion.withEn(updReplQOpt.get.io.enq.ready, updReplQOpt.get.io.enq.valid)
  }



// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- Update Replace SRAM Mes  --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * PLRU: update replacer only when read hit or write Dir
   */
    if (replPolicy == "plru") {
      replArrayOpt.get.io.wreq.valid               := updReplByHit | dirWrite.fire
      replArrayOpt.get.io.wreq.bits.addr           := Mux(updReplByHit, updReplQOpt.get.io.deq.bits.set, wSet_s1)
      replArrayOpt.get.io.wreq.bits.data.foreach(_ := Mux(updReplByHit,
                                                          repl.get_next_state(updReplQOpt.get.io.deq.bits.replMes, updReplQOpt.get.io.deq.bits.way),
                                                          repl.get_next_state(dirWrite.bits.replMes,               OHToUInt(dirWrite.bits.wayOH))))
      HardwareAssertion.withEn(replArrayOpt.get.io.wreq.fire, updReplQOpt.get.io.deq.fire)
      HardwareAssertion.withEn(replArrayOpt.get.io.wreq.fire, dirWrite.fire)
      HardwareAssertion.withEn(dirWrite.fire | updReplQOpt.get.io.deq.fire, replArrayOpt.get.io.wreq.fire)
    } else if(replPolicy == "random") {
      // nothing to do
    }
    require(replPolicy == "plru" | replPolicy == "random", "Dont support replacementPolicy except plru or random")

  /*
   * Hardware Assertion Place Pipe
   */
  HardwareAssertion.placePipe(1)

}