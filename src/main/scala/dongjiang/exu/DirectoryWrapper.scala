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
import xs.utils.debug.{DomainInfo, HardwareAssertion}

class DirectoryWrapper()(implicit p: Parameters) extends DJModule {
// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val dirRReadyVec  = Output(Vec(djparam.nrDirBank, Bool()))
    val dirRead       = Vec(2, Flipped(Valid(new DirReadBundle())))
    val dirResp       = Vec(2, Valid(new DirRespBundle()))
    val dirWrite      = Vec(2, Flipped(new DirWriteBundle()))

    val readMshr      = Vec(2, Valid(new DirReadMSHRBundle()))
    val mshrResp      = Vec(2, Flipped(Valid(new MSHRRespDirBundle())))
  })

  print(
    s"""
       |DongJiang Drectory Message: {
       |  Self:
       |  tagBits: ${sTagBits}
       |  setBits: ${sSetBits}
       |  bankBits: ${dirBankBits}
       |  SF:
       |  tagBits: ${sfTagBits}
       |  setBits: ${sfSetBits}
       |  bankBits: ${dirBankBits}
       |}
       |""".stripMargin)


  // -------------------------- Modules declaration ------------------------//
  val selfs = Seq.fill(djparam.nrDirBank) { Module(new DirectoryBase( dirType     = "self",
                                                                      tagBits     = sTagBits,
                                                                      sets        = djparam.selfSets / djparam.nrDirBank,
                                                                      ways        = djparam.selfWays,
                                                                      nrMetas     = 1,
                                                                      replPolicy  = djparam.selfReplacementPolicy,
                                                                      setup       = djparam.dirSetup,
                                                                      latency     = djparam.dirLatency,
                                                                      extraHold   = djparam.dirExtraHold,
                                                                      nrWayBank   = 4)) }

  selfs.zipWithIndex.foreach { case(s, i) => s.io.dirBank := i.U }

  val sfs   = Seq.fill(djparam.nrDirBank) { Module(new DirectoryBase( dirType     = "sf",
                                                                      tagBits     = sfTagBits,
                                                                      sets        = djparam.sfDirSets / djparam.nrDirBank,
                                                                      ways        = djparam.sfDirWays,
                                                                      nrMetas     = nrCcNode,
                                                                      replPolicy  = djparam.sfReplacementPolicy,
                                                                      setup       = djparam.dirSetup,
                                                                      latency     = djparam.dirLatency,
                                                                      extraHold   = djparam.dirExtraHold,
                                                                      nrWayBank   = 4)) }

  sfs.zipWithIndex.foreach { case(sf, i) => sf.io.dirBank := i.U }

// -------------------------- Reg and Wire declaration ------------------------//
  val selfWReadyVec = Wire(Vec(2, Vec(djparam.nrDirBank, Bool())))

  val sfWReadyVec   = Wire(Vec(2, Vec(djparam.nrDirBank, Bool())))

  val readMSHRVec   = Wire(Vec(djparam.nrDirBank, new DirReadMSHRBundle()))

  val dirRespVec    = Wire(Vec(djparam.nrDirBank, new DirRespBundle()))

// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------------------- Receive Req From MSHR / Pipe ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  io.dirRReadyVec.zipWithIndex.foreach { case(r, i) => r := selfs(i).io.dirRead.ready & sfs(i).io.dirRead.ready }

  /*
   * read dir
   */
  selfs.zip(sfs).zipWithIndex.foreach {
    case((s, sf), i) =>
      val rHit0 = io.dirRead(0).valid & io.dirRead(0).bits.dirBank === i.U
      val rHit1 = io.dirRead(1).valid & io.dirRead(1).bits.dirBank === i.U
      // self
      s.io.dirRead.valid  := rHit0 | rHit1
      s.io.dirRead.bits   := Mux(rHit0, io.dirRead(0).bits, io.dirRead(1).bits)
      // sf
      sf.io.dirRead.valid := rHit0 | rHit1
      sf.io.dirRead.bits  := Mux(rHit0, io.dirRead(0).bits, io.dirRead(1).bits)
      HardwareAssertion(!(rHit0 & rHit1))
      HardwareAssertion(Mux(s.io.dirRead.valid,  s.io.dirRead.ready,  true.B))
      HardwareAssertion(Mux(sf.io.dirRead.valid, sf.io.dirRead.ready, true.B))
  }


  /*
   * write self dir
   */
  selfs.zipWithIndex.foreach {
    case (s, i) =>
      val wSHit0 = io.dirWrite(0).s.valid & io.dirWrite(0).s.bits.dirBank === i.U
      val wSHit1 = io.dirWrite(1).s.valid & io.dirWrite(1).s.bits.dirBank === i.U
      s.io.dirWrite.valid     := wSHit0 | wSHit1
      s.io.dirWrite.bits      := Mux(wSHit0, io.dirWrite(0).s.bits, io.dirWrite(1).s.bits)
      selfWReadyVec(0)(i)     := wSHit0 & s.io.dirWrite.ready
      selfWReadyVec(1)(i)     := wSHit1 & s.io.dirWrite.ready & !wSHit0
      // assert
      val wSFire0 = io.dirWrite(0).s.fire & io.dirWrite(0).s.bits.dirBank === i.U
      val wSFire1 = io.dirWrite(1).s.fire & io.dirWrite(1).s.bits.dirBank === i.U
      HardwareAssertion(!(wSFire0 & wSFire1))
  }
  io.dirWrite.map(_.s.ready).zip(selfWReadyVec).foreach { case(a, b) => a := b.reduce(_ | _); HardwareAssertion(PopCount(b) <= 1.U) }



  /*
   * write sf dir
   */
  sfs.zipWithIndex.foreach {
    case (sf, i) =>
      val wSFHit0 = io.dirWrite(0).sf.valid & io.dirWrite(0).sf.bits.dirBank === i.U
      val wSFHit1 = io.dirWrite(1).sf.valid & io.dirWrite(1).sf.bits.dirBank === i.U
      sf.io.dirWrite.valid    := wSFHit0 | wSFHit1
      sf.io.dirWrite.bits     := Mux(wSFHit0, io.dirWrite(0).sf.bits, io.dirWrite(1).sf.bits)
      sfWReadyVec(0)(i)       := wSFHit0 & sf.io.dirWrite.ready
      sfWReadyVec(1)(i)       := wSFHit1 & sf.io.dirWrite.ready & !wSFHit0
      // assert
      val wSfFire0 = io.dirWrite(0).sf.fire & io.dirWrite(0).sf.bits.dirBank === i.U
      val wSfFire1 = io.dirWrite(1).sf.fire & io.dirWrite(1).sf.bits.dirBank === i.U
      HardwareAssertion(!(wSfFire0 & wSfFire1))
  }
  io.dirWrite.map(_.sf.ready).zip(sfWReadyVec).foreach { case(a, b) => a := b.reduce(_ | _); HardwareAssertion(PopCount(b) <= 1.U) }



// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------------------------- Send Req To MSHR ------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Mes From MSHR
   */
  readMSHRVec.zip(selfs.map(_.io.readMshr.bits)).foreach { case(a, b) => a := b }

  val rMSHRVal0 = selfs.map(_.io.readMshr).map { case s => s.valid & s.bits.pipeID === 0.U }
  val rMSHRId0  = PriorityEncoder(rMSHRVal0)

  val rMSHRVal1 = selfs.map(_.io.readMshr).map { case s => s.valid & s.bits.pipeID === 1.U }
  val rMSHRId1  = PriorityEncoder(rMSHRVal1)

  HardwareAssertion(PopCount(rMSHRVal0) <= 1.U)
  HardwareAssertion(PopCount(rMSHRVal1) <= 1.U)

  io.readMshr(0).valid  := rMSHRVal0.reduce(_ | _)
  io.readMshr(0).bits   := readMSHRVec(rMSHRId0)
  io.readMshr(0).bits.dirBank := rMSHRId0

  io.readMshr(1).valid := rMSHRVal1.reduce(_ | _)
  io.readMshr(1).bits  := readMSHRVec(rMSHRId1)
  io.readMshr(1).bits.dirBank := rMSHRId1

  /*
   * Receive MSHR Mes
   */
  selfs.zip(sfs).zipWithIndex.foreach {
    case ((s, sf), i) =>
      val hitVec = io.mshrResp.map { case r => r.valid & r.bits.dirBank === i.U }
      // MSHR Resp
      s.io.mshrResp := Mux(hitVec(0), io.mshrResp(0).bits, io.mshrResp(1).bits)
      sf.io.mshrResp := Mux(hitVec(0), io.mshrResp(0).bits, io.mshrResp(1).bits)
      // Assert
      HardwareAssertion(PopCount(hitVec) <= 1.U)
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- Resp DirResult To Pipe ----------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive
   */
  dirRespVec.zip(selfs.map(_.io.dirResp.bits).zip(sfs.map(_.io.dirResp.bits))).foreach { case(r, (s, sf)) => r.s := s; r.sf := sf }
  dirRespVec.zipWithIndex.foreach { case(r, i) => r.pipeID := i.U }

  val respPipe0   = selfs.map(_.io.dirResp).map { case r => r.valid & r.bits.pipeID === 0.U }
  val respId0     = PriorityEncoder(respPipe0)

  val respPipe1   = selfs.map(_.io.dirResp).map { case r => r.valid & r.bits.pipeID === 1.U }
  val respId1     = PriorityEncoder(respPipe1)

  io.dirResp(0).valid := respPipe0.reduce(_ | _)
  io.dirResp(0).bits  := dirRespVec(respId0)

  io.dirResp(1).valid := respPipe1.reduce(_ | _)
  io.dirResp(1).bits  := dirRespVec(respId1)



// ------------------------------------------------------- Assertion --------------------------------------------------- //
  HardwareAssertion(PopCount(selfs.map(_.io.dirRead.valid)) <= 2.U, "selfDirs: no more than two read request can be entered at the same time")
  HardwareAssertion(PopCount(selfs.map(_.io.dirWrite.valid)) <= 2.U, "selfDirs: no more than two write request can be entered at the same time")
  HardwareAssertion(PopCount(sfs.map(_.io.dirRead.valid)) <= 2.U, "sfDirs: no more than two read request can be entered at the same time")
  HardwareAssertion(PopCount(sfs.map(_.io.dirWrite.valid)) <= 2.U, "sfDirs: no more than two write request can be entered at the same time")
  HardwareAssertion(!selfs.map(_.io.dirRead.fire).zip(sfs.map(_.io.dirRead.fire)).map { case(s, sf) => s ^ sf }.reduce(_ | _), "selfDirs and sfDirs dirRead must be fire at the same time")

  HardwareAssertion(PopCount(selfs.map(_.io.dirResp.valid)) <= 2.U, "selfDirs dirResp: no more than two resp can be output at a time")
  HardwareAssertion(PopCount(sfs.map(_.io.dirResp.valid)) <= 2.U, "sfDirs dirResp: no more than two resp can be output at a time")

  val sReadFireVec  = selfs.map(_.io.dirRead.fire)
  val sfReadFireVec = sfs.map(_.io.dirRead.fire)
  HardwareAssertion(sReadFireVec.zip(sfReadFireVec).map { case (s, sf) => s === sf }.reduce(_ & _))

  val sResValVec    = selfs.map(_.io.dirResp.valid)
  val sfResValVec   = sfs.map(_.io.dirResp.valid)
  HardwareAssertion(sResValVec.zip(sfResValVec).map { case (s, sf) => s === sf }.reduce(_ & _))

  HardwareAssertion.placePipe(2)
}