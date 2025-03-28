package dongjiang.backend.snoop

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


class SnoopCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Commit Task In
    val alloc       = Flipped(Decoupled(new CMTask))
    // CHI
    val txSnp       = Decoupled(new SnoopFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Resp To Commit
    val respCmt     = Decoupled(new RespToCmt)
    // Req To Data
    val reqDB       = Decoupled(new PackLLCTxnID with HasDataVec)
  })
  HardwareAssertion(!io.alloc.valid)
  io <> DontCare

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}