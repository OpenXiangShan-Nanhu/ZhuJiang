package dongjiang.directory

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.HardwareAssertion

class Directory(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = Input(new DJConfigIO())
    // Read from frontends
    val readVec     = Vec(djparam.nrDirBank, Flipped(Decoupled(new Addr with HasPackPosIndex)))
    // Resp to frontends
    val rRespVec    = Vec(djparam.nrDirBank, Valid(new PackDirMsg))
    // Write From backend
    val write       = new DJBundle {
      val llc       = Flipped(Decoupled(new DirEntry("llc") with HasPackPosIndex))
      val sf        = Flipped(Decoupled(new DirEntry("sf")  with HasPackPosIndex))
    }
    // Resp to backend
    val wResp       = new DJBundle {
      val llc       = Valid(new DirEntry("llc"))
      val sf        = Valid(new DirEntry("sf"))
    }
    // Clean Signal from backend(Replace CM and Commit CM)
    val unlockVec   = Vec(djparam.nrDirBank, Flipped(Valid(new PosIndex())))
  })

  /*
   * Module declaration
   */
  val llcs              = Seq.tabulate(djparam.nrDirBank)(i => Module(new DirectoryBase("llc", i)))
  val sfs               = Seq.tabulate(djparam.nrDirBank)(i => Module(new DirectoryBase("sf", i)))
  val wriLLCBankPipe    = Module(new Pipe(UInt(dirBankBits.W), readDirLatency))
  val wriSFBankPipe     = Module(new Pipe(UInt(dirBankBits.W), readDirLatency))

  /*
   * Connect llcs and sfs
   */
  val llcWReadyVec    = Wire(Vec(djparam.nrDirBank, Bool()))
  val sfWReadyVec     = Wire(Vec(djparam.nrDirBank, Bool()))
  llcWReadyVec        := llcs.map(_.io.write.ready)
  sfWReadyVec         := sfs.map(_.io.write.ready)
  io.write.llc.ready  := llcWReadyVec(io.write.llc.bits.Addr.dirBank)
  io.write.sf.ready   := sfWReadyVec(io.write.sf.bits.Addr.dirBank)
  llcs.zip(sfs).zipWithIndex.foreach {
    case((llc, sf), i) =>
      // config
      llc.io.config         := io.config
      sf.io.config          := io.config

      // read valid
      llc.io.read.valid     := io.readVec(i).valid & sf.io.read.ready
      sf.io.read.valid      := io.readVec(i).valid & llc.io.read.ready
      // read bits
      llc.io.read.bits      := io.readVec(i).bits
      sf.io.read.bits       := io.readVec(i).bits
      // read ready
      io.readVec(i).ready   := llc.io.read.ready & sf.io.read.ready

      // write valid
      llc.io.write.valid    := io.write.llc.valid & io.write.llc.bits.Addr.dirBank === i.U
      sf.io.write.valid     := io.write.sf.valid  & io.write.sf.bits.Addr.dirBank === i.U
      // write bits
      llc.io.write.bits     := io.write.llc.bits
      sf.io.write.bits      := io.write.sf.bits

      // Clean Lock Table
      llc.io.unlock         := io.unlockVec(i)
      sf.io.unlock          := io.unlockVec(i)
  }

  /*
   * Connect IO
   */
  // Read Resp
  io.rRespVec.map(_.valid).zip(llcs.map(_.io.resp)).foreach        { case(a, b) => a := b.valid }
  io.rRespVec.map(_.bits.dir.llc).zip(llcs.map(_.io.resp)).foreach { case(a, b) => a := b.bits }
  io.rRespVec.map(_.bits.dir.sf ).zip( sfs.map(_.io.resp)).foreach { case(a, b) => a := b.bits }

  // Store Write LLC Resp DirBank
  wriLLCBankPipe.io.enq.valid := io.write.llc.fire
  wriLLCBankPipe.io.enq.bits  := io.write.llc.bits.Addr.dirBank

  // Store Write SF Resp DirBank
  wriSFBankPipe.io.enq.valid  := io.write.sf.fire
  wriSFBankPipe.io.enq.bits   := io.write.sf.bits.Addr.dirBank

  // Output wResp and rHitMesVec
  val llcWRespVec = Wire(Vec(djparam.nrDirBank, Valid(new DirEntry("llc"))))
  val sfWRespVec  = Wire(Vec(djparam.nrDirBank, Valid(new DirEntry("sf"))))
  llcWRespVec     := llcs.map(_.io.resp)
  sfWRespVec      := sfs.map(_.io.resp)
  // llc
  io.wResp.llc.valid  := wriLLCBankPipe.io.deq.valid
  io.wResp.llc.bits   := llcWRespVec(wriLLCBankPipe.io.deq.bits).bits
  // sf
  io.wResp.sf.valid   := wriSFBankPipe.io.deq.valid
  io.wResp.sf.bits    := sfWRespVec(wriSFBankPipe.io.deq.bits).bits
  // HardwareAssertion
  HardwareAssertion.withEn(llcWRespVec(wriLLCBankPipe.io.deq.bits).valid, wriLLCBankPipe.io.deq.valid)
  HardwareAssertion.withEn(sfWRespVec(wriSFBankPipe.io.deq.bits).valid,   wriSFBankPipe.io.deq.valid)

  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(Int.MaxValue-1)
}