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

class Directory(isTop: Boolean = false)(implicit p: Parameters) extends DJModule {
  override def isTopModule: Boolean = isTop
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
    val write       = Flipped(Decoupled(new DJBundle { // TODO: split llc and sf
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
  val llcs  = Seq.fill(djparam.nrDirBank)(Module(new DirectoryBase("llc", !isTop)))
  val sfs   = Seq.fill(djparam.nrDirBank)(Module(new DirectoryBase("sf",  !isTop)))
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
  io.rRespVec.map(_.valid).zip(llcs.map(_.io.resp)).foreach    { case(a, b) => a := b.valid & !b.bits.toRepl }
  io.rRespVec.map(_.bits.llc).zip(llcs.map(_.io.resp)).foreach { case(a, b) => a := b.bits }
  io.rRespVec.map(_.bits.sf ).zip( sfs.map(_.io.resp)).foreach { case(a, b) => a := b.bits }
  // Output llc wResp
  val llcRespVec      = VecInit(llcs.map(_.io.resp))
  val llctoReplVec    = VecInit(llcRespVec.map(r => r.valid & r.bits.toRepl))
  val llcToReplId     = PriorityEncoder(llctoReplVec)
  io.wResp.llc.valid  := llctoReplVec.asUInt.orR
  io.wResp.llc.bits   := llcRespVec(llcToReplId).bits
  HAssert(PopCount(llctoReplVec) <= 1.U)
  // Output sf wResp
  val sfRespVec       = VecInit(sfs.map(_.io.resp))
  val sftoReplVec     = VecInit(sfRespVec.map(r => r.valid & r.bits.toRepl))
  val sfToReplId      = PriorityEncoder(sftoReplVec)
  io.wResp.sf.valid   := sftoReplVec.asUInt.orR
  io.wResp.sf.bits    := sfRespVec(sfToReplId).bits
  HAssert(PopCount(sftoReplVec) <= 1.U)


  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(2)
}