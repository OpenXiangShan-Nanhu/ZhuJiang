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

class Backend(isTop: Boolean = false)(implicit p: Parameters) extends DJModule {
  override def isTopModule: Boolean = isTop
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
    val updPosNest      = if(hasBBN) Some(Valid(new PosCanNest)) else None
    val cleanPos        = Valid(new PosClean)
    // Clean Signal to Directory
    val unlock          = Valid(new PackHnIdx)
    // Frontend <> io.txRsp
    val fastResp        = Flipped(Decoupled(new RespFlit))
    // Send Task To DB
    val updHnTxnID      = Valid(new UpdHnTxnID)
    val reqDB           = Decoupled(new HnTxnID with HasDataVec)
    val dataTask        = Decoupled(new DataTask)
    val dataResp        = Flipped(Valid(new HnTxnID))
    val cleanDB         = Decoupled(new HnTxnID with HasDataVec)
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
  // val datalessCM  = Module(new DatalessCM()) // TODO
  val txReqArb    = Module(new ArbiterGenerator(new ReqFlit(true), 2, true, true))

  /*
   * Connect Config
   */
  commit.io.config          := io.config
  replCM.io.config          := io.config
  snoopCM.io.config         := io.config
  readCM.io.config          := io.config
  writeCM.io.config         := io.config

  /*
   * Connect To CHI IO
   */
  val txRspVec              = if(hasBBN) Seq(commit.io.txRsp, readCM.io.txRsp.get, FastQueue(io.fastResp)) else Seq(commit.io.txRsp, FastQueue(io.fastResp))
  txReqArb.io.in(0)         <> readCM.io.txReq
  txReqArb.io.in(1)         <> writeCM.io.txReq
  io.txReq                  <> txReqArb.io.out
  io.txSnp                  <> snoopCM.io.txSnp
  io.txRsp                  <> fastQosRRArb(txRspVec)
  io.rxRsp.ready            := true.B

  /*
   * Connect To Frontend IO
   */
  io.reqPosVec.zip(replCM.io.reqPosVec).foreach { case(a, b) => a <> b }
  io.updPosTag              := replCM.io.updPosTag
  if(hasBBN) {
    io.updPosNest.get       := fastQosRRArb.validOut(Seq(readCM.io.updPosNest.get, writeCM.io.updPosNest.get))
  }

  /*
   * Release PoS / LockTable / DataBuffer
   */
  val cleanPos              = FastQueue(fastQosRRArb(Seq(commit.io.cleanPoS, replCM.io.cleanPoS)))
  cleanPos.ready            := io.cleanDB.ready
  io.cleanPos.valid         := cleanPos.fire
  io.cleanPos.bits          := cleanPos.bits
  io.unlock.valid           := cleanPos.fire
  io.unlock.bits.hnIdx      := cleanPos.bits.hnIdx
  io.cleanDB.valid          := cleanPos.valid
  io.cleanDB.bits.hnTxnID   := cleanPos.bits.hnIdx.getTxnID
  io.cleanDB.bits.dataVec   := DataVec.Full

  /*
   * Connect To Directory IO
   */
  io.writeDir               <> replCM.io.writeDir

  /*
   * Connect To DataBlock IO
   */
  io.updHnTxnID             := replCM.io.updHnTxnID
  io.reqDB                  <> fastArb(Seq(replCM.io.reqDB, commit.io.reqDB))
  io.dataTask               <> fastQosRRArb(Seq(commit.io.dataTask, replCM.io.dataTask, writeCM.io.dataTask))

  /*
   * Connect Commit
   */
  commit.io.cmtTaskVec      <> io.cmtTaskVec.map(t => Pipe(t))
  commit.io.rxRsp           := io.rxRsp
  commit.io.rxDat           := io.rxDat
  commit.io.replResp        := replCM.io.resp
  commit.io.dataResp        := io.dataResp


  /*
   * Connect replCM
   */
  replCM.io.task            <> FastQueue(commit.io.replTask)
  replCM.io.respDir         := io.respDir
  replCM.io.dataResp        := io.dataResp

  /*
   * Connect CMResp
   */
  val cmResp = Pipe(fastQosRRArb.validOut(Seq(snoopCM.io.resp, readCM.io.resp, writeCM.io.resp)))
  commit.io.cmResp.valid    := cmResp.valid & !cmResp.bits.toRepl
  replCM.io.cmResp.valid    := cmResp.valid &  cmResp.bits.toRepl
  commit.io.cmResp.bits     := cmResp.bits
  replCM.io.cmResp.bits     := cmResp.bits


  /*
   * Connect replCM
   */
  snoopCM.io.alloc          <> FastQueue(fastQosRRArb(Seq(commit.io.cmTaskVec(CMID.SNP), replCM.io.cmTaskVec(CMID.SNP))))
  snoopCM.io.rxRsp          := io.rxRsp
  snoopCM.io.rxDat          := io.rxDat

  /*
   * Connect writeCM
   */
  writeCM.io.alloc          <> FastQueue(fastQosRRArb(Seq(commit.io.cmTaskVec(CMID.WRI), replCM.io.cmTaskVec(CMID.WRI))))
  writeCM.io.rxRsp          := io.rxRsp
  writeCM.io.dataResp       := io.dataResp

  /*
   * Connect readCM
   */
  readCM.io.alloc           <> FastQueue(fastQosRRArb(Seq(commit.io.cmTaskVec(CMID.READ))))
  readCM.io.rxDat           := io.rxDat

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
  io.txReq.bits.Addr              := Cat(io.getAddrVec(0).result.addr(addrBits-1, offsetBits), txReqArb.io.out.bits.Addr(offsetBits-1, 0))
  io.txSnp.bits.Addr              := io.getAddrVec(1).result.addr >> 3.U
  io.writeDir.bits.llc.bits.addr  := io.getAddrVec(2).result.addr
  io.writeDir.bits.sf.bits.addr   := io.getAddrVec(2).result.addr

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}