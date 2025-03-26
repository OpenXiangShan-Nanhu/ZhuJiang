package dongjiang.backend

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

class CMState(implicit p: Parameters) extends DJBundle {
  val waitResp  = Bool() // Wait TaskCM Resp
  val waitAck   = Bool() // Wait CompAck From RN
  val resp2Rn   = Bool() // Send Resp To RN Or Data
  val resp2Hn   = Bool() // Send Resp To HN Or Data
  val reqDB     = Bool() // Req DB
  val dbid2Rn   = Bool() // Send XDBIDResp To RN
  val comp2Rn   = Bool() // Send Comp To RN
  val wriDir    = Bool() // Send Write Task To Replace
  val cleanPoS  = Bool() // Clean PoS Tag
  val canNest   = Bool() // Indicate PoS This Entry Can Be Nested
  val secTask   = Bool() // Send Task To TaskCM
}

class CommitTask(implicit p: Parameters) extends DJBundle with HasPackChi with HasPackPosIndex with HasPackDirMsg with HasAlrDB with HasPackCmtCode

// TODO: no need Addr
class CMTask(implicit p: Parameters) extends DJBundle with HasPackChi with HasAddr with HasPackLLCTxnID with HasAlrDB {
  val needDB = Bool()
  val snpVec = Vec(nrSfMetas, Bool())
  def doDMT  = !needDB
}

class PackCMTask(implicit p: Parameters) extends DJBundle {
  val task = new CMTask()
}

class RespToCmt(implicit p: Parameters) extends DJBundle with HasPackLLCTxnID with HasPackTaskInst with HasAlrDB

class CommitCM(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Commit Task In
    val alloc       = Flipped(Valid(new CommitTask))
    // CHI
    val txRsp       = Decoupled(new RespFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Get Full Addr In PoS
    val getPosAddr  = Decoupled(new PosIndex())
    val posRespAddr = Input(new Addr())
    // Send Task To CM
    val cmAllocVec  = Vec(nrTaskCM, Decoupled(new CMTask))
    // Resp From TaskCM
    val respCmt     = Flipped(Valid(new RespToCmt))
    // Send Task To Replace
    val replAlloc   = Decoupled(new Addr with HasPackWriDirCode with HasPackLLCTxnID)
    val replResp    = Flipped(Valid(new PackPosIndex()))
    // Send Task To Data
    val dataTask    = Decoupled(new DataTask)
    val dataResp    = Flipped(Valid(new PackPosIndex()))
    // Update PoS Message
    val updPosNest  = Decoupled(new PackPosIndex with HasNest)
    val cleanPos    = Valid(new PackPosIndex with HasChiChannel)
    val unlock      = Valid(new PosIndex())
  })
  HardwareAssertion(!io.respCmt.valid)
  io <> DontCare

  /*
   * Reg and Wire declaration
   */
  val cmTable     = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { 0.U.asTypeOf(new CMState()) }) }))
  val msgTable    = Reg(Vec(posSets, Vec(posWays, new PackChi with HasPackDirMsg with HasAlrDB)))
  val msg_rec     = Wire(chiselTypeOf(msgTable.head.head))
  val cm_rec      = Wire(new CMState())


  /*
   * [REC] Receive Task IO
   */
  val alloc_rec       = io.alloc.bits
  val wriHitSF_rec    = alloc_rec.chi.isReq(WriteEvictOrEvict) & alloc_rec.dir.sf.hit
  val owoCanComp_rec  = alloc_rec.chi.isOWO & alloc_rec.dir.sf.hit

  msg_rec.chi     := alloc_rec.chi
  msg_rec.dir     := alloc_rec.dir
  msg_rec.alrDB   := alloc_rec.alrDB
  cm_rec.waitResp := alloc_rec.commit.invalid
  cm_rec.waitAck  := alloc_rec.chi.expCompAck
  cm_rec.resp2Rn  := alloc_rec.chi.isReq & alloc_rec.commit.commit
  cm_rec.resp2Hn  := alloc_rec.chi.isSnp & alloc_rec.commit.commit
  cm_rec.reqDB    := alloc_rec.chi.needSendDBID(alloc_rec.dir.sf.hit) & !alloc_rec.alrDB.reqs
  cm_rec.dbid2Rn  := false.B
  cm_rec.comp2Rn  := wriHitSF_rec & owoCanComp_rec
  cm_rec.wriDir   := alloc_rec.commit.wriSF | alloc_rec.commit.wriLLC
  cm_rec.cleanPoS := false.B
  cm_rec.canNest  := false.B
  cm_rec.secTask  := false.B



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