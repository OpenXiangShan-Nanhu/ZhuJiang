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
    val rxReq = Flipped(Decoupled(new ReqFlit(false)))
    val rxSnp = Flipped(Decoupled(new SnoopFlit()))
  })
  io <> DontCare

  HardwareAssertion(!io.rxReq.valid)
  HardwareAssertion.placePipe(Int.MaxValue-1)
  /*
   * Check Req
   */
  awhen(io.rxReq.valid) {
    // TODO
  }


}