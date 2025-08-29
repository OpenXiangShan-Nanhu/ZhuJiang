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
    val reqPosVec2      = Vec(djparam.nrDirBank, Vec(posSets, Valid(new ChiChnlBundle)))
    val posRespVec2     = Vec(djparam.nrDirBank, Vec(posSets, Flipped(Valid(UInt(posWayBits.W)))))
    val updPosTag       = Valid(new Addr with HasAddrValid with HasPackHnIdx)
    // Frontend -> Commit/TaskCM
    val cmtTaskVec      = Vec(djparam.nrDirBank, Flipped(Valid(new CommitTask with HasHnTxnID)))
    // Update PoS Message
    val updPosNest      = if(hasBBN) Some(Valid(new PosCanNest)) else None
    val cleanPoS        = Valid(new PosClean)
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
  io.txReq                  <> FastQueue(fastQosRRArb(Seq(readCM.io.txReq, writeCM.io.txReq)))
  io.txSnp                  <> snoopCM.io.txSnp
  io.txRsp                  <> fastQosRRArb(txRspVec)
  io.rxRsp.ready            := true.B

  /*
   * Connect To Frontend IO
   */
  io.reqPosVec2             <> replCM.io.reqPosVec2
  io.updPosTag              := replCM.io.updPosTag
  if(hasBBN) {
    io.updPosNest.get       := fastQosRRArb.validOut(Seq(readCM.io.updPosNest.get, writeCM.io.updPosNest.get))
  }

  /*
   * Release PoS / LockTable / DataBuffer
   */
  val cleanPoS              = FastQueue(fastQosRRArb(Seq(commit.io.cleanPoS, replCM.io.cleanPoS)))
  cleanPoS.ready            := io.cleanDB.ready
  io.cleanPoS.valid         := cleanPoS.fire
  io.cleanPoS.bits          := cleanPoS.bits
  io.unlock.valid           := cleanPoS.fire
  io.unlock.bits.hnIdx      := cleanPoS.bits.hnIdx
  io.cleanDB.valid          := cleanPoS.valid
  io.cleanDB.bits.hnTxnID   := cleanPoS.bits.hnIdx.getTxnID
  io.cleanDB.bits.dataVec   := DataVec.Full

  /*
   * Connect To Directory IO
   */
  val wDirQ                 = Module(new Queue(chiselTypeOf(io.writeDir.bits), entries = 1, pipe = true, flow = false))
  wDirQ.io.enq              <> replCM.io.writeDir
  io.writeDir               <> wDirQ.io.deq

  /*
   * Connect To DataBlock IO
   */
  io.updHnTxnID             := replCM.io.updHnTxnID
  io.reqDB                  <> fastArb(Seq(replCM.io.reqDB, commit.io.reqDB))
  io.dataTask               <> fastQosRRArb(Seq(FastQueue(commit.io.dataTask), FastQueue(replCM.io.dataTask), FastQueue(writeCM.io.dataTask)))

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
  replCM.io.posRespVec2     := io.posRespVec2

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
  snoopCM.io.alloc          <> fastQosRRArb(Seq(FastQueue(commit.io.cmTaskVec(CMID.SNP)), FastQueue(replCM.io.cmTaskVec(CMID.SNP))))
  snoopCM.io.rxRsp          := io.rxRsp
  snoopCM.io.rxDat          := io.rxDat

  /*
   * Connect writeCM
   */
  writeCM.io.alloc          <> fastQosRRArb(Seq(FastQueue(commit.io.cmTaskVec(CMID.WRI)), FastQueue(replCM.io.cmTaskVec(CMID.WRI))))
  writeCM.io.rxRsp          := io.rxRsp
  writeCM.io.dataResp       := io.dataResp

  /*
   * Connect readCM
   */
  readCM.io.alloc           <> fastQosRRArb(Seq(FastQueue(commit.io.cmTaskVec(CMID.READ))))
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
  // req
  val txReqMemAttr          = io.txReq.bits.MemAttr.asTypeOf(MemAttr())
  val txReqAddr             = io.getAddrVec(0).result.addr
  val txReqSize             = io.txReq.bits.Size
  io.txReq.bits.Addr        := PriorityMux(Seq(
    !txReqMemAttr.cacheable -> txReqAddr,
    (txReqSize === "b110".U)-> Cat(txReqAddr(addrBits-1, offsetBits),   0.U(offsetBits.W)),     // full size
    true.B                  -> Cat(txReqAddr(addrBits-1, offsetBits-1), 0.U((offsetBits-1).W)), // half size
  ))
  require(djparam.nrBeat == 2)
  // snp
  io.txSnp.bits.Addr              := (io.getAddrVec(1).result.addr >> 6.U) << 3.U
  // write dir
  wDirQ.io.enq.bits.llc.bits.addr :=  io.getAddrVec(2).result.addr
  wDirQ.io.enq.bits.sf.bits.addr  :=  io.getAddrVec(2).result.addr

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}