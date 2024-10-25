package dongjiang.pcu.exu

import dongjiang._
import dongjiang.pcu._
import _root_.dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.util.ReplacementPolicy
import xs.utils.ParallelPriorityMux
import xs.utils.sram.SRAMTemplate
import xs.utils.perf.{DebugOptions, DebugOptionsKey}

class DirectoryWrapper()(implicit p: Parameters) extends DJModule {
// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val earlyRReqVec  = Vec(djparam.nrDirBank, Flipped(Decoupled()))

    val dirRead       = Vec(2, Flipped(Valid(new DirReadBundle())))
    val dirResp       = Vec(2, Valid(new DirRespBundle()))
    val dirWrite      = Vec(2, Flipped(new DirWriteBundle()))

    val readMshr      = Vec(2, Valid(new DirReadMSHRBundle()))
    val mshrResp      = Vec(2, Flipped(Valid(new MSHRRespDirBundle())))
  })

// -------------------------- Modules declaration ------------------------//
  val selfs = Seq.fill(djparam.nrDirBank) { Module(new DirectoryBase( tagBits = sTagBits,
                                                                      sets = djparam.selfSets / djparam.nrDirBank,
                                                                      ways = djparam.selfWays,
                                                                      nrMetas = 1,
                                                                      replPolicy = djparam.selfReplacementPolicy,
                                                                      mcp = djparam.dirMulticycle,
                                                                      holdMcp = djparam.dirHoldMcp,
                                                                      nrWayBank = 4)) }

  selfs.zipWithIndex.foreach { case(s, i) => s.io.id := i.U }

  val sfs   = Seq.fill(djparam.nrDirBank) { Module(new DirectoryBase( tagBits = sfTagBits,
                                                                      sets = djparam.sfDirSets / djparam.nrDirBank,
                                                                      ways = djparam.sfDirWays,
                                                                      nrMetas = nrRnfNode,
                                                                      replPolicy = djparam.sfReplacementPolicy,
                                                                      mcp = djparam.dirMulticycle,
                                                                      holdMcp = djparam.dirHoldMcp,
                                                                      nrWayBank = 4)) }

  sfs.zipWithIndex.foreach { case(sf, i) => sf.io.id := i.U }

// -------------------------- Reg and Wire declaration ------------------------//
  val dirWSRegVec   = RegInit(VecInit(Seq.fill(2) { 0.U.asTypeOf(Valid(new DirWriteBaseBundle(djparam.selfWays, 1, sReplWayBits))) }))
  val dirWSFRegVec  = RegInit(VecInit(Seq.fill(2) { 0.U.asTypeOf(Valid(new DirWriteBaseBundle(djparam.sfDirWays, nrRnfNode, sfReplWayBits))) }))

  val wSelfReadVec  = Wire(Vec(djparam.nrDirBank, Bool()))
  val wSFReadVec    = Wire(Vec(djparam.nrDirBank, Bool()))

  val readMSHRVec   = Wire(Vec(djparam.nrDirBank, new DirReadMSHRBundle()))

  val dirRespVec    = Wire(Vec(djparam.nrDirBank, new DirRespBundle()))

// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- Receive Req From IO ------------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive Read Req
   */
  io.earlyRReqVec.zipWithIndex.foreach {
    case(rReq, i) =>
      rReq.ready  := selfs(i).io.earlyRReq.ready & sfs(i).io.earlyRReq.ready
      selfs(i).io.earlyRReq.valid := rReq.valid
      sfs(i).io.earlyRReq.valid   := rReq.valid
  }


  /*
   * Set Early Write Req Value
   */
  val wSBankVec   = io.dirWrite.map { case w => getDirBank(w.s.bits.addr) }
  val wSFBankVec  = io.dirWrite.map { case w => getDirBank(w.sf.bits.addr) }

  selfs.map(_.io.earlyWReq).zipWithIndex.foreach {
    case(wReq, i) =>
      val hitVec      = wSBankVec.zip(io.dirWrite.map(_.s.valid)).map { case(b, v) => b === i.U & v }
      wReq.valid      := hitVec.reduce(_ | _)
      wSelfReadVec(i) := wReq.ready
  }

  sfs.map(_.io.earlyWReq).zipWithIndex.foreach {
    case (wReq, i) =>
      val hitVec      = wSFBankVec.zip(io.dirWrite.map(_.sf.valid)).map { case (b, v) => b === i.U & v }
      wReq.valid      := hitVec.reduce(_ | _)
      wSFReadVec(i)   := wReq.ready
  }



  /*
   * Receive IO DirWrite Req
   */
  io.dirWrite(0).s.ready  := wSelfReadVec(wSBankVec(0))
  io.dirWrite(0).sf.ready := wSelfReadVec(wSBankVec(0))

  io.dirWrite(1).s.ready  := wSelfReadVec(wSBankVec(1)) & !(io.dirWrite(0).s.valid  & wSBankVec(0)  === wSBankVec(1))
  io.dirWrite(1).sf.ready := wSelfReadVec(wSBankVec(1)) & !(io.dirWrite(0).sf.valid & wSFBankVec(0) === wSFBankVec(1))

  io.dirWrite.zipWithIndex.foreach {
    case (w, i) =>
      dirWSRegVec(i).valid  := w.s.fire
      dirWSFRegVec(i).valid := w.sf.fire

      dirWSRegVec(i).bits   := w.s.bits
      dirWSFRegVec(i).bits  := w.sf.bits
  }

  /*
   * Set selfs and sfs dirRead and dirWrite IO Value
   */
  selfs.zip(sfs).zipWithIndex.foreach {
    case ((s, sf), i) =>
      val rHitVec     = io.dirRead.map { case r => r.valid & getDirBank(r.bits.addr) === i.U }
      val wSHitVec    = dirWSRegVec.map { case w => w.valid & getDirBank(w.bits.addr) === i.U }
      val wSFHitVec   = dirWSFRegVec.map { case w => w.valid & getDirBank(w.bits.addr) === i.U }
      // Read
      s.io.dirRead    := Mux(rHitVec(0), io.dirRead(0).bits, io.dirRead(1).bits)
      sf.io.dirRead   := Mux(rHitVec(0), io.dirRead(0).bits, io.dirRead(1).bits)
      // Write
      s.io.dirWrite   := Mux(wSHitVec(0), dirWSRegVec(0).bits, dirWSRegVec(1).bits)
      sf.io.dirWrite  := Mux(wSFHitVec(0), dirWSFRegVec(0).bits, dirWSFRegVec(1).bits)
      // Assert
      assert(PopCount(rHitVec) <= 1.U)
      assert(PopCount(wSHitVec) <= 1.U)
      assert(PopCount(wSFHitVec) <= 1.U)
  }

// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- Receive Req From IO ------------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Mes From MSHR
   */
  readMSHRVec.zip(selfs.map(_.io.readMshr.bits)).foreach { case(a, b) => a := b }

  val rMSHRVal0 = selfs.map(_.io.readMshr).map { case s => s.valid & s.bits.pipeId === 0.U }
  val rMSHRId0  = PriorityEncoder(rMSHRVal0)

  val rMSHRVal1 = selfs.map(_.io.readMshr).map { case s => s.valid & s.bits.pipeId === 1.U }
  val rMSHRId1  = PriorityEncoder(rMSHRVal1)

  assert(PopCount(rMSHRVal0) <= 1.U)
  assert(PopCount(rMSHRVal1) <= 1.U)

  io.readMshr(0).valid  := rMSHRVal0.reduce(_ | _)
  io.readMshr(0).bits   := readMSHRVec(rMSHRId0)
  io.readMshr(0).bits.dirBankId := rMSHRId0

  io.readMshr(1).valid := rMSHRVal1.reduce(_ | _)
  io.readMshr(1).bits  := readMSHRVec(rMSHRId1)
  io.readMshr(1).bits.dirBankId := rMSHRId1

  /*
   * Receive MSHR Mes
   */
  selfs.zip(sfs).zipWithIndex.foreach {
    case ((s, sf), i) =>
      val hitVec = io.mshrResp.map { case r => r.valid & r.bits.dirBankId === i.U }
      // MSHR Resp
      s.io.mshrResp := Mux(hitVec(0), io.mshrResp(0).bits, io.mshrResp(1).bits)
      sf.io.mshrResp := Mux(hitVec(0), io.mshrResp(0).bits, io.mshrResp(1).bits)
      // Assert
      assert(PopCount(hitVec) <= 1.U)
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- Resp DirResult To Pipe ----------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive
   */
  dirRespVec.zip(selfs.map(_.io.dirResp.bits).zip(sfs.map(_.io.dirResp.bits))).foreach { case(r, (s, sf)) => r.s := s; r.sf := sf }
  dirRespVec.zipWithIndex.foreach { case(r, i) => r.pipeId := i.U }

  val respPipe0   = selfs.map(_.io.dirResp).map { case r => r.valid & r.bits.pipeId === 0.U }
  val respId0     = PriorityEncoder(respPipe0)

  val respPipe1   = selfs.map(_.io.dirResp).map { case r => r.valid & r.bits.pipeId === 1.U }
  val respId1     = PriorityEncoder(respPipe1)

  io.dirResp(0).valid := respPipe0.reduce(_ | _)
  io.dirResp(0).bits  := dirRespVec(respId0)

  io.dirResp(1).valid := respPipe1.reduce(_ | _)
  io.dirResp(1).bits  := dirRespVec(respId1)



// ------------------------------------------------------- Assertion --------------------------------------------------- //
  assert(PopCount(selfs.map(_.io.earlyRReq.valid)) <= 2.U, "selfDirs: no more than two read request can be entered at the same time")
  assert(PopCount(selfs.map(_.io.earlyWReq.valid)) <= 2.U, "selfDirs: no more than two write request can be entered at the same time")
  assert(PopCount(sfs.map(_.io.earlyRReq.valid)) <= 2.U, "sfDirs: no more than two read request can be entered at the same time")
  assert(PopCount(sfs.map(_.io.earlyWReq.valid)) <= 2.U, "sfDirs: no more than two write request can be entered at the same time")
  assert(!selfs.map(_.io.earlyRReq.fire).zip(sfs.map(_.io.earlyRReq.fire)).map { case(s, sf) => s ^ sf }.reduce(_ | _), "selfDirs and sfDirs dirRead must be fire at the same time")

  assert(PopCount(selfs.map(_.io.dirResp.valid)) <= 2.U, "selfDirs dirResp: no more than two resp can be output at a time")
  assert(PopCount(sfs.map(_.io.dirResp.valid)) <= 2.U, "sfDirs dirResp: no more than two resp can be output at a time")

  val sResValVec = selfs.map(_.io.dirResp.valid)
  val sfResValVec = sfs.map(_.io.dirResp.valid)
  assert(sResValVec.zip(sfResValVec).map { case (s, sf) => s === sf }.reduce(_ & _))
}