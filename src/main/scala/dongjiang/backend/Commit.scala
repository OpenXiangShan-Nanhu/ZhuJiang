package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg}
import dongjiang.frontend.decode._
import zhujiang.chi.ReqOpcode._


class CMState(implicit p: Parameters) extends DJBundle {
  val waitResp  = Bool() // Wait TaskCM Resp
  val waitAck   = Bool() // Wait CompAck From RN
  val resp2Rn   = Bool() // Send Resp To RN Or Data
  val resp2Hn   = Bool() // Send Resp To HN Or Data
  val dbid2Rn   = Bool() // Send XDBIDResp To RN
  val comp2Rn   = Bool() // Send Comp To RN
  val wriDir    = Bool() // Send Write Task To Replace
  val cleanPoS  = Bool() // Clean PoS Tag
  val canNest   = Bool() // Indicate PoS This Entry Can Be Nested
  val secTask   = Bool() // Send Task To TaskCM
}

class Commit(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Commit Task In
    val commit      = Flipped(Valid(new DJBundle {
      val chi       = new ChiTask
      val pos       = new PosIndex()
      val dir       = new DirMsg()
      val alrDeqDB  = Bool()
      val hasOps    = Bool()
      val commit    = new CommitCode()
    }))
  })
  HardwareAssertion(!io.commit.valid)

  /*
   * Reg and Wire declaration
   */
  val cmTable     = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { 0.U.asTypeOf(new DJBundle {
    val cm        = new CMState()
    val chi       = new ChiTask()
    val dir       = new DirMsg()
    val alrDeqDB  = Bool()
  }) }) }))
  val recCM       = WireInit(0.U.asTypeOf(cmTable.head.head))


  /*
   * [REC] Receive Task IO
   */
  val commit        = io.commit.bits
  recCM.chi         := commit.chi
  recCM.dir         := commit.dir
  recCM.alrDeqDB    := commit.alrDeqDB




  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}