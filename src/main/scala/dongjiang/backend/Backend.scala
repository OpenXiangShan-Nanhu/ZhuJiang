package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xijiang.Node
import xs.utils.debug._
import xs.utils.queue._
import dongjiang.frontend._
import dongjiang.frontend.decode._
import dongjiang.data._
import dongjiang.directory._
import dongjiang.backend.snoop._
import dongjiang.backend.read._
import dongjiang.backend.dataless._
import dongjiang.backend.wrioratm._
import dongjiang.backend.replace._
import dongjiang.backend.receive._

class Backend(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config          = new DJConfigIO()
    // CHI TX
    val txReq           = Decoupled(new ReqFlit(true))
    val txSnp           = Decoupled(new SnoopFlit())
    val txRsp           = Decoupled(new RespFlit())
    // CHI RX
    val rxRsp           = Flipped(Decoupled(new RespFlit()))
    val rxDat           = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // CHI RESP From Frontend
    val fastResp        = Flipped(Decoupled(new RespFlit()))
    // Update PoS Message
    val updPosNestVec   = Vec(djparam.nrDirBank, Decoupled(new PackPosIndex with HasNest))
    val updPosTagVec    = Vec(djparam.nrDirBank, Valid(new PackPosIndex with HasAddr)) // Only from replace
    val cleanPosVec     = Vec(djparam.nrDirBank, Valid(new PackPosIndex with HasChiChannel))
    val lockPosSetVec   = Vec(djparam.nrDirBank, Valid(new PackPosIndex)) // Only from replace
    val unlockPosSetVec = Vec(djparam.nrDirBank, Valid(new PackPosIndex)) // Only from replace
    // Write Directory
    val writeDir        = new DJBundle {
      val llc           = Decoupled(new DirEntry("llc") with HasPackPosIndex)
      val sf            = Decoupled(new DirEntry("sf")  with HasPackPosIndex)
    }
    // Write Directory Resp
    val respDir         = new DJBundle {
      val llc           = Flipped(Valid(new DirEntry("llc")))
      val sf            = Flipped(Valid(new DirEntry("sf")))
    }
    // Clean Signal to Directory
    val unlockVec       = Vec(djparam.nrDirBank, Valid(new PosIndex()))
    // Task From Frontend
    val cmtAllocVec     = Vec(djparam.nrDirBank, Flipped(Valid(new CommitTask())))
    val cmAllocVec2     = Vec(djparam.nrDirBank, Vec(nrTaskCM, Flipped(Decoupled(new CMTask()))))
    // Send Task To DB
    val reqDB           = Decoupled(new PackLLCTxnID with HasDataVec)
    val dataTask        = Decoupled(new DataTask())
    val dataResp        = Flipped(Valid(new PackLLCTxnID()))
  })

  /*
   * Module declaration
   */
  val cmtCMs      = Seq.tabulate(djparam.nrDirBank)(i => Module(new CommitCM(i)))
  val replCM      = Module(new ReplaceCM())
  val snoopCM     = Module(new SnoopCM())
  val wriOrAtmCM  = Module(new WriOrAtmCM())
  val readCM      = Module(new ReadCM())
  val datalessCM  = Module(new DatalessCM())
  val receiveCM   = Module(new ReceiveCM())

  /*
   * Connect Config
   */
  cmtCMs.foreach(_.io.config := io.config)
  replCM.io.config      := io.config
  snoopCM.io.config     := io.config
  readCM.io.config      := io.config
  datalessCM.io.config  := io.config
  wriOrAtmCM.io.config  := io.config
  receiveCM.io.config   := io.config

  /*
   * Connect CHI IO
   */
  io.txReq        <> fastRRArb(Seq(readCM.io.txReq, datalessCM.io.txReq, wriOrAtmCM.io.txReq))
  io.txSnp        <> snoopCM.io.txSnp
  io.txRsp        <> fastRRArb(cmtCMs.map(_.io.txRsp) ++ Seq(readCM.io.txRsp, datalessCM.io.txRsp, receiveCM.io.txRsp, FastQueue(io.fastResp, fastRespQSzie.max(2))))
  io.rxRsp.ready  := true.B

  /*
   * Connect Frontend
   */
  // updPosNestVec
  io.updPosNestVec.zipWithIndex.foreach { case(upd, i) => upd <> fastRRArb(Seq(cmtCMs(i).io.updPosNest, wriOrAtmCM.io.updPosNestVec(i))) }

  // updPosTagVec
  io.updPosTagVec.zipWithIndex.foreach { case(upd, i) =>
    upd.valid := replCM.io.updPosTag.valid & replCM.io.updPosTag.bits.dirBank === i.U
    upd.bits  := replCM.io.updPosTag.bits
  }

  // cleanPosVec
  io.cleanPosVec.zip(cmtCMs.map(_.io.cleanPos)).foreach { case(a, b) => a := b }

  // lockPosSetVec
  io.lockPosSetVec.zip(io.unlockPosSetVec).zipWithIndex.foreach { case ((lock, unlock), i) =>
    lock.valid    := replCM.io.lockPosSet.valid & replCM.io.lockPosSet.bits.dirBank === i.U
    lock.bits     := replCM.io.lockPosSet.bits
    unlock.valid  := replCM.io.unlockPosSet.valid & replCM.io.unlockPosSet.bits.dirBank === i.U
    unlock.bits   := replCM.io.unlockPosSet.bits
  }

  /*
   * Connect Directory
   */
  io.writeDir <> replCM.io.writeDir
  io.unlockVec.zip(cmtCMs.map(_.io.unlock)).foreach { case (a, b) => a <> b }

  /*
   * Connect Data Block
   */
  io.reqDB    <> fastRRArb(Seq(snoopCM.io.reqDB, readCM.io.reqDB, receiveCM.io.reqDB))
  io.dataTask <> fastRRArb(cmtCMs.map(_.io.dataTask) ++ Seq(wriOrAtmCM.io.dataTask))


  /*
   * Connect Commits
   */
  val respCmtVec      = Wire(Vec(nrTaskCM, Decoupled(new RespToCmt())))
  val respCmtSelIdVec = Wire(Vec(djparam.nrDirBank, UInt(log2Ceil(nrTaskCM).W)))
  respCmtVec          <> Seq(snoopCM.io.respCmt, wriOrAtmCM.io.respCmt, readCM.io.respCmt, datalessCM.io.respCmt, receiveCM.io.respCmt)
  respCmtVec.zipWithIndex.foreach { case(r, i) => r.ready := respCmtSelIdVec(r.bits.llcTxnID.dirBank) === i.U }
  cmtCMs.zipWithIndex.foreach {
    case(cmt, i) =>
      cmt.io.alloc              := io.cmtAllocVec(i)
      cmt.io.rxRsp              := io.rxRsp
      val respCmtValVec         = respCmtVec.map(r => r.valid & r.bits.llcTxnID.dirBank === i.U)
      respCmtSelIdVec(i)        := RREncoder(respCmtValVec)
      cmt.io.respCmt.valid      := respCmtValVec.reduce(_ | _)
      cmt.io.respCmt.bits       := respCmtVec(respCmtSelIdVec(i)).bits
      // replResp
      cmt.io.replResp.valid     := replCM.io.resp.valid & replCM.io.resp.bits.dirBank === i.U
      cmt.io.replResp.bits.pos  := replCM.io.resp.bits.pos
      // dataResp
      cmt.io.dataResp.valid     := io.dataResp.valid & io.dataResp.bits.llcTxnID.dirBank === i.U
      cmt.io.dataResp.bits.pos  := io.dataResp.bits.llcTxnID.pos
  }

  /*
   * Connect replCM
   */
  replCM.io.alloc     <> fastRRArb(cmtCMs.map(_.io.replAlloc))
  replCM.io.respDir   := io.respDir
  replCM.io.respRepl  <> fastArb.validOut(Seq(snoopCM.io.respRepl, wriOrAtmCM.io.respRepl))

  /*
   * Connect replCM
   */
  snoopCM.io.alloc    <> fastArb(cmtCMs.map(_.io.cmAllocVec(CMID.SNP)) ++ Seq(replCM.io.cmAllocVec(CMID.SNP)) ++ io.cmAllocVec2.map(_(CMID.SNP)))
  snoopCM.io.rxRsp    := io.rxRsp
  snoopCM.io.rxDat    := io.rxDat

  /*
   * Connect wriOrAtmCM
   */
  wriOrAtmCM.io.alloc     <> fastArb(cmtCMs.map(_.io.cmAllocVec(CMID.WOA)) ++ Seq(replCM.io.cmAllocVec(CMID.WOA)) ++ io.cmAllocVec2.map(_(CMID.WOA)))
  wriOrAtmCM.io.rxRsp     := io.rxRsp
  wriOrAtmCM.io.dataResp  := io.dataResp

  /*
   * Connect readCM
   */
  readCM.io.alloc <> fastArb(cmtCMs.map(_.io.cmAllocVec(CMID.READ)) ++ io.cmAllocVec2.map(_(CMID.READ)))
  readCM.io.rxDat := io.rxDat

  /*
   * Connect datalessCM
   */
  datalessCM.io.alloc <> fastArb(cmtCMs.map(_.io.cmAllocVec(CMID.DL)) ++ io.cmAllocVec2.map(_(CMID.DL)))
  datalessCM.io.rxRsp := io.rxRsp

  /*
   * Connect receiveCM
   */
  receiveCM.io.alloc <> fastArb(cmtCMs.map(_.io.cmAllocVec(CMID.REC)) ++ io.cmAllocVec2.map(_(CMID.REC)))
  receiveCM.io.rxRsp := io.rxRsp
  receiveCM.io.rxDat := io.rxDat

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 1)
}