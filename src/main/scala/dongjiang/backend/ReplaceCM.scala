package dongjiang.backend.replace

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle.{HasAddr, HasPackLLCTxnID, PackChi, _}
import dongjiang.data.HasAlrDB
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, HasPackDirMsg}
import dongjiang.frontend.decode.{HasPackCmtCode, _}
import dongjiang.data._
import zhujiang.chi.ReqOpcode._
import dongjiang.frontend._
import dongjiang.backend._

class ReplaceCM(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config          = new DJConfigIO()
    // Commit Task In
    val alloc           = Flipped(Decoupled(new Addr with HasPackWriDirCode with HasPackLLCTxnID))
    val resp            = Valid(new LLCTxnID())
    // Get Full Addr In PoS
    val getPosAddrVec   = Vec(djparam.nrDirBank, Decoupled(new PosIndex()))
    val posRespAddrVec  = Input(Vec(djparam.nrDirBank, new Addr()))
    // Send Task To CM
    val cmAllocVec      = Vec(nrTaskCM, Decoupled(new CMTask))
    // Send Task To Data
    val dataTask        = Decoupled(new DataTask)
    val dataResp        = Valid(new PackLLCTxnID())
    // Update PoS Message
    val updPosTag       = Valid(new LLCTxnID with HasAddr)
    val lockPosSet      = Valid(new LLCTxnID with HasLockSet)
    // Write Directory
    val writeDir        = new DJBundle {
      val llc           = Decoupled(new DirEntry("llc") with HasPackPosIndex)
      val sf            = Decoupled(new DirEntry("sf") with HasPackPosIndex)
    }
    // Write Directory Resp
    val respDir         = new DJBundle {
      val llc           = Flipped(Valid(new DirEntry("llc")))
      val sf            = Flipped(Valid(new DirEntry("sf")))
    }
  })
  HardwareAssertion(!io.alloc.valid)
  io <> DontCare

  /*
   * Reg and Wire declaration
   */
  val cmRegVec  = RegInit(VecInit(Seq.fill(nrReplaceCM) { 0.U.asTypeOf(new CMState) }))
  val msgRegVec = Reg(Vec(nrReplaceCM, chiselTypeOf(io.alloc.bits)))


  /*
   * [REC] Receive Task IO
   */
  val cmVec_rec   = cmRegVec.map(!_.asUInt.orR) // Free Vec
  val cmId_rec    = PriorityEncoder(cmVec_rec)
  io.alloc.ready  := cmVec_rec.reduce(_ | _)

  /*
   * Modify Ctrl Machine Table
   */
  cmRegVec.zip(msgRegVec).zipWithIndex.foreach {
    case ((cm, msg), i) =>
      val allocHit = io.alloc.fire & i.U === cmId_rec

      // Store Msg From Frontend
      when(allocHit) {
        msg := io.alloc.bits
      }
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}