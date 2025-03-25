package dongjiang.backend.commit

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
    val alloc       = Flipped(Valid(new DJBundle {
      val chi       = new ChiTask()
      val pos       = new PosIndex()
      val dir       = new DirMsg()
      val alrDeqDB  = Bool()
      val hasOps    = Bool()
      val commit    = new CommitCode()
    }))
    // CHI
    val txRsp       = Decoupled(new RespFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Send Task To CM
    val cmAllocOH   = Output(UInt(nrTaskCM.W))
    val cmAlloc     = Flipped(Decoupled(new DJBundle {
      val chi       = new ChiTask with HasAddr
      val txnID     = new LLCTxnID()
      val needDB    = Bool()
      val alrReqDB  = Bool()
      val snpVec    = Vec(nrSfMetas, Bool())
    }))
    // Resp From TaskCM
    val respCmt     = Decoupled(new DJBundle with HasPosIndex {
      val inst      = new TaskInst()
      val alrReqDB  = Bool()
    })
    // Send Task To Replace
    val replAlloc   = Decoupled(new DJBundle with HasAddr {
      val code      = new WriDirCode()
      val txnID     = new LLCTxnID()
    })
    // Send Task To Data
    val dataAlloc   = Decoupled(new DJBundle with HasChiSize {
      val read      = Bool()
      val save      = Bool()
      val send      = Bool()
      val txDat     = new DataFlit()      // Use in send
      val txnID     = new LLCTxnID()      // Use in all
      val set       = UInt(llcSetBits.W)  // Use in read/save
      val way       = UInt(llcWayBits.W)  // Use in read/save
    })
  })
  HardwareAssertion(!io.alloc.valid)
  io <> DontCare

  /*
   * Reg and Wire declaration
   */
  val cmTable     = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { new CMState() }) }))
  val msgTable    = Reg(Vec(posSets, Vec(posWays, new DJBundle {
    val cm        = new CMState()
    val chi       = new ChiTask()
    val dir       = new DirMsg()
    val alrDeqDB  = Bool()
  })))
  val msg_rec     = Wire(chiselTypeOf(msgTable.head.head))
  val cm_rec      = Wire(new CMState())


  /*
   * [REC] Receive Task IO
   */
  val alloc_rec       = io.alloc.bits
  val wriHitSF_rec    = alloc_rec.chi.isReq(WriteEvictOrEvict) & alloc_rec.dir.sf.hit
  val owoCanComp_rec  = alloc_rec.chi.isOWO & alloc_rec.dir.sf.hit

  msg_rec.chi      := alloc_rec.chi
  msg_rec.dir      := alloc_rec.dir
  msg_rec.alrDeqDB := alloc_rec.alrDeqDB
  cm_rec.waitResp  := alloc_rec.hasOps
  cm_rec.waitAck   := alloc_rec.chi.expCompAck
  cm_rec.resp2Rn   := alloc_rec.chi.isReq & alloc_rec.commit.commit
  cm_rec.resp2Hn   := alloc_rec.chi.isSnp & alloc_rec.commit.commit
  cm_rec.dbid2Rn   := false.B
  cm_rec.comp2Rn   := wriHitSF_rec & owoCanComp_rec
  cm_rec.wriDir    := alloc_rec.commit.wriSF | alloc_rec.commit.wriLLC
  cm_rec.cleanPoS  := false.B
  cm_rec.canNest   := false.B
  cm_rec.secTask   := false.B
  HardwareAssertion.withEn(!alloc_rec.hasOps ^ alloc_rec.commit.asUInt =/= Code.error, io.alloc.valid)


  /*
   *
   */


  /*
   * Modify Ctrl Machine Table
   */
  cmTable.zip(msgTable).zipWithIndex.foreach {
    case((cmVec, msgVec), i) =>
      cmVec.zip(msgVec).zipWithIndex.foreach {
        case((cm, msg), j) =>
          val allocHit  = io.alloc.valid & io.alloc.bits.pos.idxMatch(i, j)
          // Receive new task from frontend
          when(allocHit) {
            cm  := cm_rec
            msg := msg_rec
          }
      }
  }


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}