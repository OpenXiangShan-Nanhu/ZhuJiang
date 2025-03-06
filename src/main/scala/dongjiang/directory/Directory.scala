package dongjiang.directory

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{DomainInfo, HardwareAssertion}

class Directory(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Read from frontends
    val readVec     = Vec(djparam.nrDirBank, Flipped(Decoupled(new DJBundle with HasAddr with HasDCID {
      val early     = Bool() // Early access to data
    })))
    // Resp to frontends
    val rRespVec    = Output(Vec(djparam.nrDirBank, new DJBundle {
      val llc       = new DirEntry("llc")
      val sf        = new DirEntry("sf")
    }))
    // Write From backend
    val write       = new DJBundle {
      val llc       = Flipped(Decoupled(new DirEntry("llc")))
      val sf        = Flipped(Decoupled(new DirEntry("sf")))
    }
    // Resp to backend
    val wResp       = Output(new DJBundle {
      val llc       = new DirEntry("llc")
      val sf        = new DirEntry("sf")
    })
    // Read LLC Hit Message to Data
    val rHitMesVec  = Vec(djparam.nrDirBank, Valid(new DJBundle with HasDCID {
      val set       = UInt(llcSetBits.W)
      val way       = UInt(llcWayBits.W)
    }))
  })

  /*
   * Module declaration
   */
  val llcs            = Seq.tabulate(djparam.nrDirBank)(i => Module(new DirectoryBase("llc", i)))
  val sfs             = Seq.tabulate(djparam.nrDirBank)(i => Module(new DirectoryBase("sf", i)))
  val wriLLCBankPipe  = Module(new Pipe(UInt(dirBankBits.W), readDirLatency))
  val wriSFBankPipe   = Module(new Pipe(UInt(dirBankBits.W), readDirLatency))
  val hitDCIDPipes    = Seq.fill(djparam.nrDirBank) { Module(new Pipe(UInt(dcIdBits.W), readDirLatency)) }

  /*
   * Connect llcs and sfs
   */
  io.write.llc.ready := DontCare
  io.write.sf.ready  := DontCare
  llcs.zip(sfs).zipWithIndex.foreach {
    case((llc, sf), i) =>
      // read valid
      llc.io.read.valid     := io.readVec(i).valid & sf.io.read.ready
      sf.io.read.valid      := io.readVec(i).valid & llc.io.read.ready
      // read bits
      llc.io.read.bits      := io.readVec(i).bits
      sf.io.read.bits       := io.readVec(i).bits
      // read ready
      io.readVec(i).ready   := llc.io.read.ready & sf.io.read.ready

      // write valid
      llc.io.write.valid    := io.write.llc.valid & io.write.llc.bits.dirBank === i.U
      sf.io.write.valid     := io.write.sf.valid  & io.write.sf.bits.dirBank === i.U
      // write bits
      llc.io.write.bits     := io.write.llc.bits
      sf.io.write.bits      := io.write.sf.bits
      // write read
      when(io.write.llc.bits.bankId === i.U) {
        io.write.llc.ready  := llc.io.write.ready
      }
      when(io.write.sf.bits.bankId === i.U) {
        io.write.sf.ready   := sf.io.write.ready
      }
  }

  /*
   * Connect IO
   */
  // Read Resp
  io.rRespVec.map(_.llc).zip(llcs.map(_.io.resp)).foreach { case(a, b) => a := b }
  io.rRespVec.map(_.sf ).zip( sfs.map(_.io.resp)).foreach { case(a, b) => a := b }

  // Store Write LLC Resp DirBank
  wriLLCBankPipe.io.enq.valid := io.write.llc.fire
  wriLLCBankPipe.io.enq.bits  := io.write.llc.bits.bankId

  // Store Write SF Resp DirBank
  wriSFBankPipe.io.enq.valid  := io.write.sf.fire
  wriSFBankPipe.io.enq.bits   := io.write.sf.bits.bankId

  // Store DCID
  hitDCIDPipes.zip(io.readVec).foreach {
    case(pipe, read) =>
      pipe.io.enq.valid       := read.fire
      pipe.io.enq.bits        := read.bits.dcid
  }

  // Output wResp and rHitMesVec
  io.wResp := DontCare
  llcs.zip(sfs).zipWithIndex.foreach {
    case ((llc, sf), i) =>
      // LLC wResp
      when(wriLLCBankPipe.io.deq.bits === i.U) {
        io.wResp.llc  := llc.io.resp
      }
      // SF wResp
      when(wriSFBankPipe.io.deq.bits === i.U) {
        io.wResp.sf   := sf.io.resp
      }
      // Hit Resp
      io.rHitMesVec(i).valid      := hitDCIDPipes(i).io.deq.valid & llc.io.resp.hit
      io.rHitMesVec(i).bits.dcid  := hitDCIDPipes(i).io.deq.bits
      io.rHitMesVec(i).bits.set   := llc.io.resp.set
      io.rHitMesVec(i).bits.way   := llc.io.resp.way
  }

  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(Int.MaxValue - 1)
}