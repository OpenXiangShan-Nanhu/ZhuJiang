package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.{CMTask, CommitTask, FastResp, RecRespType, ReqPoS}
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.data.DataTask
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, PackDirMsg}
import dongjiang.frontend.decode.{CommitCode, Operations}
import xs.utils.queue.FastQueue

class Frontend(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config          = new DJConfigIO()
    val dirBank         = Input(UInt(dirBankBits.W))
    // CHI REQ/SNP
    val rxReq           = Flipped(Decoupled(new ReqFlit(false)))
    val rxHpr           = Flipped(Decoupled(new ReqFlit(false)))
    val rxSnp           = Flipped(Decoupled(new SnoopFlit))
    // To DataBlock
    val reqDB_s1        = Decoupled(new HnTxnID with HasDataVec)
    val reqDB_s3        = Decoupled(new HnTxnID with HasDataVec)
    val fastData        = Decoupled(new DataTask)
    val cleanDB         = Decoupled(new HnTxnID with HasDataVec)
    // DIR Read/Resp
    val readDir         = Decoupled(new Addr with HasPackHnIdx)
    val respDir         = Flipped(Valid(new DirMsg))
    // To Backend
    val cmtTask         = Valid(new CommitTask with HasHnTxnID)
    // Get addr from PoS
    val getAddrVec      = Vec(3, Flipped(new GetAddr)) // txReq + txSnp + writeDir
    // Update PoS Message
    val reqPoS          = Flipped(new ReqPoS)
    val updPosTag       = Flipped(Valid(new Addr with HasAddrValid with HasPackHnIdx))
    val updPosNest      = Flipped(Valid(new PosCanNest))
    val cleanPos        = Flipped(Valid(new PosClean))
    // Resp to Node(RN/SN): ReadReceipt, DBIDResp, CompDBIDResp
    val fastResp        = Decoupled(new FastResp)
    val recRespType     = Decoupled(new RecRespType)
    // PoS Busy Signal
    val alrUsePoS       = Output(UInt(log2Ceil(nrPoS + 1).W))
    //  system is working
    val working         = Output(Bool())
  })


  /*
   * Module declaration
   */
  val req2Task    = Module(new ReqToChiTask())
  val hpr2Task    = Module(new ReqToChiTask())
  val snp2Task    = Module(new SnpToChiTask())
  // S0
  val hprTaskBuf  = Module(new TaskBuffer(nrHprTaskBuf, sort = true))
  val reqTaskBuf  = Module(new TaskBuffer(nrReqTaskBuf, sort = true))
  val snpTaskBuf  = Module(new TaskBuffer(nrSnpTaskBuf, sort = false))
  // S1
  val posTable    = Module(new PosTable())
  val block       = Module(new Block())
  // S2: Wait Directory Response
  val pipe_s2     = Module(new Pipe(chiselTypeOf(block.io.task_s1.bits), readDirLatency-1))
  // S3: Receive DirResp and Decode
  val decode      = Module(new Decode())
  // Queue
  val fastRespQ   = Module(new FastQueue(new FastResp, size = djparam.nrDirBank.max(2), deqDataNoX = false))
  val respTypeQ   = Module(new Queue(new RecRespType, entries = readDirLatency, flow = true, pipe = true))
  val cleanDBQ    = Module(new Queue(new HnTxnID with HasDataVec, entries = readDirLatency, flow = true, pipe = true))

  /*
   * Connect
   */
  // config
  req2Task.io.config        := io.config
  hpr2Task.io.config        := io.config
  snp2Task.io.config        := io.config
  posTable.io.config        := io.config
  block.io.config           := io.config
  decode.io.config          := io.config
  // dirBank
  posTable.io.dirBank       := io.dirBank

  // io
  io.getAddrVec.zip(posTable.io.getAddrVec).foreach { case(a, b) => a <> b }
  io.reqDB_s1               <> block.io.reqDB_s1
  io.reqDB_s3               <> decode.io.reqDB_s3
  io.readDir                <> block.io.readDir_s1
  io.fastResp               <> fastRespQ.io.deq
  io.recRespType            <> respTypeQ.io.deq
  io.fastData               <> decode.io.fastData_s3
  io.cleanDB                <> cleanDBQ.io.deq
  io.cmtTask                := decode.io.cmtTask_s3
  io.alrUsePoS              := posTable.io.alrUsePoS
  io.working                := hprTaskBuf.io.working | reqTaskBuf.io.working | snpTaskBuf.io.working | posTable.io.working

  // io.fastResp <--- [Queue] --- block.io.fastResp_s1
  //                        ^
  //                        |
  // block fastResp enq when it cant deq (deq.valid & !deq.ready)
  //                        |
  // io.recRespType <------ [Queue] -------- decode.io.recRespType_s3
  // io.cleanDB     <------ [Queue] -------- decode.io.cleanDB_s3
  // fastRespQ
  val blockFastResp         = respTypeQ.io.deq.valid & (!respTypeQ.io.deq.ready | !cleanDBQ.io.enq.ready)
  fastRespQ.io.enq.valid    := block.io.fastResp_s1.valid & !blockFastResp
  fastRespQ.io.enq.bits     := block.io.fastResp_s1.bits
  block.io.fastResp_s1.ready:= fastRespQ.io.enq.ready & !blockFastResp
  // respTypeQ
  respTypeQ.io.enq.valid    := decode.io.recRespType_s3.valid
  respTypeQ.io.enq.bits     := decode.io.recRespType_s3.bits
  HAssert.withEn(respTypeQ.io.enq.ready, respTypeQ.io.enq.valid)
  // cleanDBQ
  cleanDBQ.io.enq.valid     := decode.io.cleanDB_s3.valid
  cleanDBQ.io.enq.bits      := decode.io.cleanDB_s3.bits
  HAssert.withEn(cleanDBQ.io.enq.ready, cleanDBQ.io.enq.valid)

  // hpr2Task
  hpr2Task.io.rxReq         <> io.rxHpr

  // req2Task
  req2Task.io.rxReq         <> io.rxReq

  // hprTaskBuf [S0]
  hprTaskBuf.io.chiTaskIn   <> hpr2Task.io.chiTask
  hprTaskBuf.io.retry_s1    := block.io.retry_s1
  hprTaskBuf.io.sleep_s1    := posTable.io.sleep_s1
  hprTaskBuf.io.wakeup      := posTable.io.wakeup

  // reqTaskBuf [S0]
  reqTaskBuf.io.chiTaskIn   <> req2Task.io.chiTask
  reqTaskBuf.io.retry_s1    := block.io.retry_s1
  reqTaskBuf.io.sleep_s1    := posTable.io.sleep_s1
  reqTaskBuf.io.wakeup      := posTable.io.wakeup

  // snp2Task and snpTaskBuf [S0]
  if(hasBBN) {
    // snp2Task
    snp2Task.io.rxSnp       <> io.rxSnp
    // snpTaskBuf [S0]
    snpTaskBuf.io.chiTaskIn <> snp2Task.io.chiTask
    snpTaskBuf.io.retry_s1  := block.io.retry_s1
    snpTaskBuf.io.sleep_s1  := DontCare // snp never sleep
    snpTaskBuf.io.wakeup    := DontCare // not need to wakeup
    HardwareAssertion(!snpTaskBuf.io.chiTask_s0.valid)
  } else {
    // DontCare
    io.rxSnp                <> DontCare
    snp2Task.io             <> DontCare
    snpTaskBuf.io           <> DontCare
  }

  // posTable [S1]
  posTable.io.alloc_s0      := fastArb(Seq(snpTaskBuf.io.allocPos_s0, hprTaskBuf.io.allocPos_s0 , reqTaskBuf.io.allocPos_s0))
  posTable.io.retry_s1      := block.io.retry_s1
  posTable.io.updNest       := io.updPosNest
  posTable.io.clean         := io.cleanPos
  posTable.io.updTag        := io.updPosTag
  posTable.io.reqPoS        <> io.reqPoS
  posTable.io.reqPoS.req.valid := io.reqPoS.req.valid & io.reqPoS.req.bits.dirBank === io.dirBank

  // block [S1]
  block.io.chiTask_s0       := fastArb(Seq(snpTaskBuf.io.chiTask_s0 , hprTaskBuf.io.chiTask_s0, reqTaskBuf.io.chiTask_s0))
  block.io.posBlock_s1      := posTable.io.block_s1
  block.io.hnIdx_s1         := posTable.io.hnIdx_s1

  // buffer [S2]
  pipe_s2.io.enq.valid      := block.io.task_s1.valid
  pipe_s2.io.enq.bits       := block.io.task_s1.bits

  // decode [S3]
  decode.io.task_s2.valid   := pipe_s2.io.deq.valid
  decode.io.task_s2.bits    := pipe_s2.io.deq.bits
  decode.io.respDir_s3      := io.respDir

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}