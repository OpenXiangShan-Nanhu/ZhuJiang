package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xijiang.Node
import xs.utils.debug.{DomainInfo, HardwareAssertion}
import dongjiang.frontend._
import dongjiang.frontend.decode._
import dongjiang.data._
import dongjiang.directory._

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
    val updPosNestVec = Vec(djparam.nrDirBank, Decoupled(new PackPosIndex with HasNest))
    val updPosTagVec  = Vec(djparam.nrDirBank, Valid(new PackPosIndex with HasAddr)) // Only from replace
    val cleanPosVec   = Vec(djparam.nrDirBank, Decoupled(new PackPosIndex with HasChiChannel))
    val lockPosSetVec = Vec(djparam.nrDirBank, Valid(new PackPosIndex with HasLockSet)) // Only from replace
    // Write Directory
    val writeDir      = new DJBundle {
      val llc         = Decoupled(new DirEntry("llc") with HasPackPosIndex)
      val sf          = Decoupled(new DirEntry("sf")  with HasPackPosIndex)
    }
    // Write Directory Resp
    val respDir       = new DJBundle {
      val llc         = Flipped(Valid(new DirEntry("llc")))
      val sf          = Flipped(Valid(new DirEntry("sf")))
    }
    // Clean Signal to Directory
    val unlockVec2    = Vec(djparam.nrDirBank, Vec(2, Valid(new PosIndex())))
    // Task From Frontend
    val cmtAllocVec   = Vec(djparam.nrDirBank, Flipped(Valid(new CommitTask())))
    val cmAllocVec2   = Vec(djparam.nrDirBank, Vec(nrTaskCM, Flipped(Decoupled(new CMTask()))))
    // Send Task To DB
    val reqDB         = Decoupled(new PackLLCTxnID with HasChiSize)
    val dataTask      = Decoupled(new DataTask)
    // Multicore Req running in LAN
    val multicore     = Bool() // TODO
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