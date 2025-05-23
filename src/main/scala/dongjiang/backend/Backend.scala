package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import zhujiang.chi.RspOpcode._
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

class Backend(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config          = new DJConfigIO()
    // CHI TX
    val txReq           = Decoupled(new ReqFlit(true))
    val txSnp           = Decoupled(new SnoopFlit)
    val txRsp           = Decoupled(new RespFlit)
    // CHI RX
    val rxRsp           = Flipped(Decoupled(new RespFlit))
    val rxDat           = Flipped(Valid(new DataFlit)) // Dont use rxDat.Data/BE in Backend
    // Write Directory
    val writeDir        = Decoupled(new DJBundle {
      val llc           = Valid(new DirEntry("llc") with HasPackHnIdx)
      val sf            = Valid(new DirEntry("sf") with HasPackHnIdx)
    })
    // Write Directory Resp
    val respDir         = new DJBundle {
      val llc           = Flipped(Valid(new DirEntry("llc") with HasHnTxnID))
      val sf            = Flipped(Valid(new DirEntry("sf")  with HasHnTxnID))
    }
    // Frontend <> ReplaceCM
    val reqPosVec       = Vec(djparam.nrDirBank, new ReqPoS)
    val updPosTag       = Valid(new Addr with HasAddrValid with HasPackHnIdx)
    // Frontend -> Commit/TaskCM
    val cmtTaskVec      = Vec(djparam.nrDirBank, Flipped(Valid(new CommitTask with HasHnTxnID)))
    // Update PoS Message
    val updPosNest      = Valid(new PosCanNest)
    val cleanPos        = Valid(new PosClean)
    // Clean Signal to Directory
    val unlock          = Valid(new PackHnIdx)
    // Frontend <> ReceiveCM
    val fastResp        = Flipped(Decoupled(new FastResp))
    val recRespType     = Flipped(Decoupled(new RecRespType))
    // Send Task To DB
    val cutHnTxnID      = Valid(new CutHnTxnID)
    val reqDB           = Decoupled(new HnTxnID with HasDataVec)
    val dataTask        = Decoupled(new DataTask)
    val dataResp        = Flipped(Valid(new HnTxnID))
    // Get addr from PoS
    val getAddrVec      = Vec(3, new GetAddr) // txReq + txSnp + writeDir
  })

  /*
   * Module declaration
   */
  val commit      = Module(new Commit())
  val replCM      = Module(new ReplaceCM())
  val snoopCM     = Module(new SnoopCM())
  val writeCM     = Module(new WriteCM())
  val readCM      = Module(new ReadCM())
  val datalessCM  = Module(new DatalessCM())
  val receiveCM   = Module(new ReceiveCM())
  val respQueue   = Module(new FastQueue(new RespFlit, fastRespQSzie.max(2), false))

  /*
   * Connect Config
   */
  commit.io.config          := io.config
  replCM.io.config          := io.config
  snoopCM.io.config         := io.config
  readCM.io.config          := io.config
  datalessCM.io.config      := io.config
  writeCM.io.config         := io.config
  receiveCM.io.config       := io.config

  /*
   * Connect To CHI IO
   */
  io.txReq                  <> fastRRArb(Seq(readCM.io.txReq, datalessCM.io.txReq, writeCM.io.txReq))
  io.txSnp                  <> snoopCM.io.txSnp
  io.txRsp                  <> fastRRArb(Seq(commit.io.txRsp, readCM.io.txRsp, datalessCM.io.txRsp, receiveCM.io.txRsp, respQueue.io.deq))
  io.rxRsp.ready            := true.B

  /*
   * Connect To Frontend IO
   */
  io.reqPosVec.zip(replCM.io.reqPosVec).foreach { case(a, b) => a <> b }
  io.updPosTag              := replCM.io.updPosTag
  io.updPosNest             := fastRRArb.validOut(Seq(readCM.io.updPosNest, datalessCM.io.updPosNest, writeCM.io.updPosNest))
  io.cleanPos               := fastRRArb.validOut(Seq(commit.io.cleanPoS, replCM.io.cleanPoS))

  /*
   * Connect To Directory IO
   */
  io.writeDir               <> replCM.io.writeDir
  io.unlock.valid           := io.cleanPos.valid
  io.unlock.bits.hnIdx      := io.cleanPos.bits.hnIdx

  /*
   * Connect To DataBlock IO
   */
  io.cutHnTxnID             := replCM.io.cutHnTxnID
  io.reqDB                  <> fastRRArb(Seq(snoopCM.io.reqDB, readCM.io.reqDB, receiveCM.io.reqDB))
  io.dataTask               <> fastRRArb(Seq(commit.io.dataTask, replCM.io.dataTask, writeCM.io.dataTask))

  /*
   * Connect Commit
   */
  commit.io.cmtTaskVec      <> io.cmtTaskVec
  commit.io.rxRsp           := io.rxRsp
  commit.io.rxDat           := io.rxDat
  commit.io.replResp        := replCM.io.resp
  commit.io.dataResp        := io.dataResp


  /*
   * Connect replCM
   */
  replCM.io.task            <> commit.io.replTask
  replCM.io.respDir         := io.respDir
  replCM.io.dataResp        := io.dataResp

  /*
   * Connect CMResp
   */
  val cmResp = fastRRArb.validOut(Seq(snoopCM.io.resp, readCM.io.resp, datalessCM.io.resp, writeCM.io.resp, receiveCM.io.resp))
  commit.io.cmResp.valid    := cmResp.valid & !cmResp.bits.toRepl
  replCM.io.cmResp.valid    := cmResp.valid &  cmResp.bits.toRepl
  commit.io.cmResp.bits     := cmResp.bits
  replCM.io.cmResp.bits     := cmResp.bits


  /*
   * Connect replCM
   */
  snoopCM.io.alloc          <> fastRRArb(Seq(commit.io.cmTaskVec(CMID.SNP), replCM.io.cmTaskVec(CMID.SNP)))
  snoopCM.io.rxRsp          := io.rxRsp
  snoopCM.io.rxDat          := io.rxDat

  /*
   * Connect writeCM
   */
  writeCM.io.alloc          <> fastRRArb(Seq(commit.io.cmTaskVec(CMID.WRI), replCM.io.cmTaskVec(CMID.WRI)))
  writeCM.io.rxRsp          := io.rxRsp
  writeCM.io.dataResp       := io.dataResp

  /*
   * Connect readCM
   */
  readCM.io.alloc           <> fastRRArb(Seq(commit.io.cmTaskVec(CMID.READ)))
  readCM.io.rxDat           := io.rxDat

  /*
   * Connect datalessCM
   */
  datalessCM.io.alloc       <> fastRRArb(Seq(commit.io.cmTaskVec(CMID.DL)))
  datalessCM.io.rxRsp       := io.rxRsp

  /*
   * Connect receiveCM
   */
  receiveCM.io.rxRsp        := io.rxRsp
  receiveCM.io.rxDat        := io.rxDat
  receiveCM.io.respType     <> io.recRespType

  /*
   * Connect fastResp
   */
  receiveCM.io.alloc.valid  := io.fastResp.valid & io.fastResp.bits.rsp.Opcode =/= ReadReceipt
  respQueue.io.enq.valid    := io.fastResp.valid & io.fastResp.bits.rsp.Opcode === ReadReceipt
  receiveCM.io.alloc.bits   := io.fastResp.bits
  respQueue.io.enq.bits     := io.fastResp.bits.rsp
  io.fastResp.ready         := Mux(io.fastResp.bits.rsp.Opcode === ReadReceipt, respQueue.io.enq.ready, receiveCM.io.alloc.ready)

  /*
   * Remap addr
   */
  // hnIdx
  io.getAddrVec(0).hnIdx    := io.txReq.bits.TxnID.asTypeOf(new HnTxnID).getHnIdx
  io.getAddrVec(1).hnIdx    := io.txSnp.bits.TxnID.asTypeOf(new HnTxnID).getHnIdx
  io.getAddrVec(2).hnIdx    := replCM.io.writeDir.bits.llc.bits.addr.asTypeOf(new HnTxnID).getHnIdx
  HAssert.withEn(replCM.io.writeDir.bits.llc.bits.hnIdx.asUInt === replCM.io.writeDir.bits.sf.bits.hnIdx.asUInt, replCM.io.writeDir.valid)
  HAssert.withEn(replCM.io.writeDir.bits.llc.bits.addr         === replCM.io.writeDir.bits.sf.bits.addr,         replCM.io.writeDir.valid)
  // result
  io.txReq.bits.Addr              := io.getAddrVec(0).result.addr
  io.txSnp.bits.Addr              := io.getAddrVec(1).result.addr >> 3.U
  io.writeDir.bits.llc.bits.addr  := io.getAddrVec(2).result.addr
  io.writeDir.bits.sf.bits.addr   := io.getAddrVec(2).result.addr

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}