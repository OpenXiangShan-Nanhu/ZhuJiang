package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.directory.DirEntry
import xijiang.Node
import xs.utils.debug.{DomainInfo, HardwareAssertion}
import dongjiang.directory.{DirEntry, DirMsg}
import dongjiang.frontend.decode._

class Backend(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // CHI TX
    val txReq         = Decoupled(new ReqFlit(true))
    val txSnp         = Decoupled(new SnoopFlit())
    val txRsp         = Decoupled(new RespFlit())
    // CHI RX
    val rxRsp         = Flipped(Decoupled(new RespFlit()))
    val rxDat         = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // CHI RESP From Frontend
    val fastResp      = Flipped(Decoupled(new RespFlit()))
    // Update PoS Message
    val updPosTagVec  = Vec(djparam.nrDirBank, Valid(new Addr with HasPosIndex))
    val updPosNestVec = Vec(djparam.nrDirBank, Valid(new DJBundle with HasPosIndex {
      val canNest     = Bool()
    }))
    val cleanPosVec   = Vec(djparam.nrDirBank, Valid(new DJBundle with HasPosIndex {
      val isSnp       = Bool()
    }))
    // Write Directory
    val writeDir      = new DJBundle {
      val llc         = Decoupled(new DirEntry("llc") with HasPosIndex)
      val sf          = Decoupled(new DirEntry("sf")  with HasPosIndex)
    }
    // Write Directory Resp
    val respDir       = new DJBundle {
      val llc         = Flipped(Valid(new DirEntry("llc")))
      val sf          = Flipped(Valid(new DirEntry("sf")))
    }
    // Clean Signal to Directory
    val unlockVec2    = Vec(djparam.nrDirBank, Vec(2, Valid(new PosIndex())))
    // Multicore Req running in LAN // TODO
    val multicore     = Bool()
    // Task From Frontend
    val cmtAllocVec   = Vec(djparam.nrDirBank, Flipped(Valid(new DJBundle {
      val chi         = new ChiTask
      val pos         = new PosIndex()
      val dir         = new DirMsg()
      val alrDeqDB    = Bool()
      val hasOps      = Bool()
      val commit      = new CommitCode()
    })))
    val cmAllocVec2   = Vec(djparam.nrDirBank, Vec(nrTaskCM, Flipped(Decoupled(new DJBundle {
      val chi         = new ChiTask with HasAddr
      val txnID       = new LLCTxnID
      val needDB      = Bool()
      val alrReqDB    = Bool()
      val snpVec      = Vec(nrSfMetas, Bool())
    }))))
  })
  io <> DontCare
  HardwareAssertion(!io.cmtAllocVec.map(_.valid).reduce(_ | _))


  /*
   * Module declaration
   */


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 1)
}