package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._

class Frontend(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config = new DJConfigIO()
    // CHI REQ/SNP
    val rxReq   = Flipped(Decoupled(new ReqFlit(false)))
    val rxSnp   = Flipped(Decoupled(new SnoopFlit()))
  })
  io <> DontCare


  /*
   * Module declaration
   */
  val req2Task    = Module(new ReqToChiTask())
  val snp2Task    = Module(new SnpToChiTask())
  val taskBuffer  = Module(new TaskBuffer())


  /*
   * Connect
   */
  req2Task.io.config  := io.config
  req2Task.io.rxReq   <> io.rxReq

  snp2Task.io.config := io.config
  snp2Task.io.rxSnp <> io.rxSnp

  taskBuffer.io.chiTask <> fastArb(Seq(snp2Task.io.chiTask, req2Task.io.chiTask))




  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-1)
}