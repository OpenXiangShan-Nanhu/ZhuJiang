package dongjiang.backend.dataless

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg}
import dongjiang.frontend._
import dongjiang.frontend.decode._
import zhujiang.chi.RspOpcode._
import dongjiang.backend._
import dongjiang.backend.read.State._


class DatalessCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config        = new DJConfigIO()
    // Get Full Addr In PoS
    val getAddr       = new GetAddr()
    // Commit Task In
    val alloc         = Flipped(Decoupled(new CMTask))
    // CHI
    val txReq         = Decoupled(new ReqFlit(true))
    val txRsp         = Decoupled(new RespFlit())
    val rxRsp         = Flipped(Valid(new RespFlit()))
    // Resp To Commit
    val respCmt       = Decoupled(new RespToCmt)
  })
  HardwareAssertion(!io.alloc.valid)
  io <> DontCare

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}