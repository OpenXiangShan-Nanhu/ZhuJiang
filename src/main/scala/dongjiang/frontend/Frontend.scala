package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.{CMTask, CommitTask, ReqPoS}
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.data.DataTask
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, PackDirMsg}
import dongjiang.frontend.decode.{CommitCode, Operations}
import xs.utils.queue.FastQueue

class Frontend(isTop: Boolean = false)(implicit p: Parameters) extends DJModule {
  override def isTopModule: Boolean = isTop
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
    val rxSnp           = if(hasBBN) Some(Flipped(Decoupled(new SnoopFlit))) else None
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
    val updPosNest      = if(hasBBN) Some(Flipped(Valid(new PosCanNest))) else None
    val cleanPos        = Flipped(Valid(new PosClean)) // Dont care QoS
    // Resp to Node(RN/SN): ReadReceipt, DBIDResp
    val fastResp        = Decoupled(new RespFlit)
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
  val snp2Task    = if(hasBBN) Some(Module(new SnpToChiTask())) else None
  // S0
  val hprTaskBuf  = Module(new TaskBuffer(nrHprTaskBuf, sort = true))
  val reqTaskBuf  = Module(new TaskBuffer(nrReqTaskBuf, sort = true))
  val snpTaskBuf  = if(hasBBN) Some(Module(new TaskBuffer(nrSnpTaskBuf, sort = false))) else None
  // S1
  val posAlloc_s0 = Wire(Valid(new Addr with HasChiChannel))
  val posTable    = Module(new PosTable())
  val block       = Module(new Block())
  // S2: Wait Directory Response
  val pipe        = Module(new Pipe(chiselTypeOf(block.io.task_s1.bits), readDirLatency-1))
  // S3: Receive DirResp and Decode
  val decode      = Module(new Decode())
  // Queue
  val cleanDBQ    = Module(new FastQueue(new HnTxnID with HasDataVec, size = djparam.nrDirBank, deqDataNoX = false))

  /*
   * Connect
   */
  // config
  req2Task.io.config        := io.config
  hpr2Task.io.config        := io.config
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
  io.fastResp               <> FastQueue(block.io.fastResp_s1)
  io.fastData               <> decode.io.fastData_s3
  io.cleanDB                <> cleanDBQ.io.deq
  io.cmtTask                := decode.io.cmtTask_s3
  io.alrUsePoS              := posTable.io.alrUsePoS
  io.working                := hprTaskBuf.io.working | reqTaskBuf.io.working | snpTaskBuf.map(_.io.working).getOrElse(false.B) | posTable.io.working

  // cleanDBQ
  cleanDBQ.io.enq.valid     := decode.io.cleanDB_s3.valid
  cleanDBQ.io.enq.bits      := decode.io.cleanDB_s3.bits
  HAssert.withEn(cleanDBQ.io.enq.ready, cleanDBQ.io.enq.valid)

  // hpr2Task
  hpr2Task.io.rxReq         <> FastQueue(io.rxHpr)

  // req2Task
  req2Task.io.rxReq         <> FastQueue(io.rxReq)

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
    snp2Task.get.io.config      := io.config
    snp2Task.get.io.rxSnp       <> FastQueue(io.rxSnp.get)
    // snpTaskBuf [S0]
    snpTaskBuf.get.io.chiTaskIn <> snp2Task.get.io.chiTask
    snpTaskBuf.get.io.retry_s1  := block.io.retry_s1
    snpTaskBuf.get.io.sleep_s1  := DontCare // snp never sleep
    snpTaskBuf.get.io.wakeup    := DontCare // not need to wakeup
    assert(!snpTaskBuf.get.io.chiTask_s0.valid," TODO")
  }

  // posTable [S1]
  if (hasBBN) {
    posAlloc_s0.bits.channel:= block.io.chiTask_s0.bits.chi.channel
    posTable.io.updNest.get := io.updPosNest.get
  } else {
    posAlloc_s0.bits.channel:= ChiChannel.REQ
  }
  posAlloc_s0.valid         := block.io.chiTask_s0.valid
  posAlloc_s0.bits.addr     := block.io.chiTask_s0.bits.addr
  posTable.io.alloc_s0      := posAlloc_s0
  posTable.io.retry_s1      := block.io.retry_s1
  posTable.io.clean         := io.cleanPos
  posTable.io.updTag        := io.updPosTag
  posTable.io.reqPoS        <> io.reqPoS
  posTable.io.reqPoS.req.valid := io.reqPoS.req.valid & io.reqPoS.req.bits.dirBank === io.dirBank

  // block [S1]
  val taskVec_s0            = if(hasBBN) Seq(snpTaskBuf.get.io.chiTask_s0, hprTaskBuf.io.chiTask_s0, reqTaskBuf.io.chiTask_s0) else Seq(hprTaskBuf.io.chiTask_s0, reqTaskBuf.io.chiTask_s0)
  block.io.chiTask_s0       := fastArb.validOut(taskVec_s0)
  block.io.posBlock_s1      := posTable.io.block_s1
  block.io.hnIdx_s1         := posTable.io.hnIdx_s1
  HAssert(PopCount(taskVec_s0.map(_.fire)) <= 1.U)

  // buffer [S2]
  pipe.io.enq.valid         := block.io.task_s1.valid
  pipe.io.enq.bits          := block.io.task_s1.bits

  // decode [S3]
  decode.io.task_s2.valid   := pipe.io.deq.valid
  decode.io.task_s2.bits    := pipe.io.deq.bits
  decode.io.respDir_s3      := io.respDir

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}