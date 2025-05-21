package dongjiang.directory

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import xs.utils.mbist.MbistPipeline

class Directory(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = Input(new DJConfigIO())
    // Read from frontends
    val readVec     = Vec(djparam.nrDirBank, Flipped(Decoupled(new Addr with HasPackHnIdx)))
    // Resp to frontends
    val rRespVec    = Vec(djparam.nrDirBank, Valid(new DirMsg))
    // Write From backend
    val write       = Flipped(Decoupled(new DJBundle {
      val llc       = Valid(new DirEntry("llc") with HasPackHnIdx)
      val sf        = Valid(new DirEntry("sf") with HasPackHnIdx)
    }))
    // Resp to backend
    val wResp       = new DJBundle {
      val llc       = Valid(new DirEntry("llc") with HasHnTxnID)
      val sf        = Valid(new DirEntry("sf")  with HasHnTxnID)
    }
    // Clean Signal from backend(Replace CM and Commit CM)
    val unlock      = Flipped(Valid(new PackHnIdx)) // broadcast signal
  })

  /*
   * Module declaration
   */
  val llcs            = Seq.fill(djparam.nrDirBank)(Module(new DirectoryBase("llc")))
  val sfs             = Seq.fill(djparam.nrDirBank)(Module(new DirectoryBase("sf")))
  val wriLLCBankPipe  = Module(new Pipe(UInt(dirBankBits.W), readDirLatency))
  val wriSFBankPipe   = Module(new Pipe(UInt(dirBankBits.W), readDirLatency))
  MbistPipeline.PlaceMbistPipeline(2, "HomeDirectory", hasMbist)

  /*
   * Connect llcs and sfs
   */
  llcs.zip(sfs).zipWithIndex.foreach {
    case((llc, sf), i) =>
      // config
      llc.io.config         := io.config
      sf.io.config          := io.config
      llc.io.dirBank        := i.U
      sf.io.dirBank         := i.U

      // read valid
      llc.io.read.valid     := io.readVec(i).valid & sf.io.read.ready
      sf.io.read.valid      := io.readVec(i).valid & llc.io.read.ready
      // read bits
      llc.io.read.bits      := io.readVec(i).bits
      sf.io.read.bits       := io.readVec(i).bits
      // read ready
      io.readVec(i).ready   := llc.io.read.ready & sf.io.read.ready

      // write valid
      llc.io.write.valid    := io.write.fire & io.write.bits.llc.valid & io.write.bits.llc.bits.Addr.dirBank === i.U
      sf.io.write.valid     := io.write.fire & io.write.bits.sf.valid  & io.write.bits.sf.bits.Addr.dirBank === i.U
      // write bits
      llc.io.write.bits     := io.write.bits.llc.bits
      sf.io.write.bits      := io.write.bits.sf.bits

      // Clean Lock Table
      llc.io.unlock         := io.unlock
      sf.io.unlock          := io.unlock
  }
  val llcWReady             = VecInit(llcs.map(_.io.write.ready))(io.write.bits.llc.bits.Addr.dirBank)
  val sfWReady              = VecInit(sfs.map (_.io.write.ready))(io.write.bits.sf.bits.Addr.dirBank)
  io.write.ready            := (llcWReady | !io.write.bits.llc.valid) & (sfWReady | !io.write.bits.sf.valid)
  HAssert.withEn(io.write.bits.llc.valid | io.write.bits.sf.valid, io.write.valid)

  /*
   * Connect IO
   */
  // Read Resp
  io.rRespVec.map(_.valid).zip(llcs.map(_.io.resp)).foreach    { case(a, b) => a := b.valid }
  io.rRespVec.map(_.bits.llc).zip(llcs.map(_.io.resp)).foreach { case(a, b) => a := b.bits }
  io.rRespVec.map(_.bits.sf ).zip( sfs.map(_.io.resp)).foreach { case(a, b) => a := b.bits }

  // Store Write LLC Resp DirBank
  wriLLCBankPipe.io.enq.valid := io.write.fire & io.write.bits.llc.valid & !io.write.bits.llc.bits.hit
  wriLLCBankPipe.io.enq.bits  := io.write.bits.llc.bits.hnIdx.dirBank

  // Store Write SF Resp DirBank
  wriSFBankPipe.io.enq.valid  := io.write.fire & io.write.bits.sf.valid & !io.write.bits.sf.bits.hit
  wriSFBankPipe.io.enq.bits   := io.write.bits.sf.bits.hnIdx.dirBank

  // Output wResp and rHitMesVec
  val llcRespVec      = VecInit(llcs.map(_.io.resp))
  val sfRespVec       = VecInit(sfs.map(_.io.resp))
  // llc
  io.wResp.llc.valid  := wriLLCBankPipe.io.deq.valid
  io.wResp.llc.bits   := llcRespVec(wriLLCBankPipe.io.deq.bits).bits
  // sf
  io.wResp.sf.valid   := wriSFBankPipe.io.deq.valid
  io.wResp.sf.bits    := sfRespVec(wriSFBankPipe.io.deq.bits).bits
  // HardwareAssertion
  HardwareAssertion.withEn(llcRespVec(wriLLCBankPipe.io.deq.bits).valid, wriLLCBankPipe.io.deq.valid)
  HardwareAssertion.withEn(sfRespVec(wriSFBankPipe.io.deq.bits).valid,   wriSFBankPipe.io.deq.valid)

  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(2)
}