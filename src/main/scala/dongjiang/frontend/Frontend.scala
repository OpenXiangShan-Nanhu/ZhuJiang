package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.{CMTask, CommitTask}
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.data.DataTask
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, PackDirMsg}
import dongjiang.frontend.decode.{CommitCode, Operations}
import xs.utils.queue.FastQueue

class Frontend(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config        = new DJConfigIO()
    val dirBank       = Input(UInt(dirBankBits.W))
    // CHI REQ/SNP
    val rxReq         = Flipped(Decoupled(new ReqFlit(false)))
    val rxSnp         = Flipped(Decoupled(new SnoopFlit()))
    // To Data
    val reqDB_s1      = Decoupled(new PackLLCTxnID with HasDataVec)
    val fastData_s3   = Decoupled(new DataTask)
    // DIR Read/Resp
    val readDir_s1    = Decoupled(new Addr with HasPackPosIndex)
    val respDir_s3    = Flipped(Valid(new PackDirMsg))
    // To Backend
    val cmtAlloc_s3   = Valid(new CommitTask())
    val cmAllocVec_s4 = Vec(nrTaskCM, Decoupled(new CMTask()))
    // Get Full Addr In PoS
    val getAddrVec    = Vec(nrGetAddr, Flipped(new GetAddr(true)))
    // Update PoS Message
    val updPosNest    = Flipped(Decoupled(new PackPosIndex with HasNest))
    val updPosTag     = Flipped(Valid(new PackPosIndex with HasAddr))
    val cleanPos      = Flipped(Valid(new PackPosIndex with HasChiChannel))
    val lockPosSet    = Flipped(Valid(new PackPosIndex))
    val unlockPosSet  = Flipped(Valid(new PackPosIndex))
    // Resp to Node(RN/SN): ReadReceipt, DBIDResp, CompDBIDResp
    val fastResp      = Decoupled(new RespFlit())
    // PoS Busy Signal
    val alrUsePoS     = Output(UInt(log2Ceil(nrPoS + 1).W))
    // Multiple cores are actively making requests
    val multicore     = Bool()
  })


  /*
   * Module declaration
   */
  val req2Task    = Module(new ReqToChiTask())
  val snp2Task    = Module(new SnpToChiTask())
  // S0
  val reqTaskBuf  = Module(new TaskBuffer(nrReqTaskBuf, sort = true))
  val snpTaskBuf  = Module(new TaskBuffer(nrSnpTaskBuf, sort = false))
  // S1
  val posTable    = Module(new PosTable())
  val block       = Module(new Block(dirBank))
  // S2: Wait Directory Response
  val bufReg_s2   = RegInit(0.U.asTypeOf(block.io.task_s1.bits))
  val shiftReg_s2 = RegInit(0.U.asTypeOf(new Shift(readDirLatency-1)))
  // S3: Receive DirResp and Decode
  val decode      = Module(new Decode(dirBank))
  // S4: Issue Task to Backend
  val issue       = Module(new Issue(dirBank))

  /*
   * Connect
   */
  // config
  req2Task.io.config        := io.config
  snp2Task.io.config        := io.config
  posTable.io.config        := io.config
  block.io.config           := io.config
  decode.io.config          := io.config
  issue.io.config           := io.config
  // dirBank
  posTable.io.dirBank       := io.dirBank

  // io
  io.getAddrVec.zip(posTable.io.getAddrVec).foreach { case(a, b) => a <> b }
  io.reqDB_s1               <> block.io.reqDB_s1
  io.fastData_s3            <> decode.io.fastData_s3
  io.readDir_s1             <> block.io.readDir_s1
  io.fastResp               <> FastQueue(block.io.fastResp_s1, djparam.nrDirBank.max(2))
  io.alrUsePoS              := posTable.io.alrUsePoS
  io.cmtAlloc_s3            := issue.io.cmtAlloc_s3
  io.cmAllocVec_s4          <> issue.io.cmAllocVec_s4
  io.multicore              := reqTaskBuf.io.multicore

  // req2Task
  req2Task.io.rxReq         <> io.rxReq

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
    reqTaskBuf.io.retry_s1  := block.io.retry_s1
    reqTaskBuf.io.sleep_s1  := DontCare // snp never sleep
    reqTaskBuf.io.wakeup    := DontCare // not need to wakeup
    HardwareAssertion(!snpTaskBuf.io.chiTask_s0.valid)
  } else {
    // DontCare
    io.rxSnp                <> DontCare
    snp2Task.io             <> DontCare
    snpTaskBuf.io           <> DontCare
  }

  // posTable [S1]
  posTable.io.req_s0        := fastRRArb(Seq(snpTaskBuf.io.req2Pos_s0, reqTaskBuf.io.req2Pos_s0))
  posTable.io.retry_s1      := block.io.retry_s1
  posTable.io.updNest       := fastArb.validOut(Seq(decode.io.updNest_s3, io.updPosNest))
  posTable.io.updTag        := io.updPosTag
  posTable.io.clean         := io.cleanPos
  posTable.io.lockSet       := io.lockPosSet
  posTable.io.unlockSet     := io.unlockPosSet

  // block [S1]
  block.io.chiTask_s0       := fastRRArb(Seq(snpTaskBuf.io.chiTask_s0, reqTaskBuf.io.chiTask_s0))
  block.io.posBlock_s1      := posTable.io.block_s1
  block.io.posIdx_s1        := posTable.io.posIdx_s1
  block.io.alrUseBuf        := shiftReg_s2.s.orR +& decode.io.task_s3.valid + issue.io.alrUseBuf
  HardwareAssertion((shiftReg_s2.s.orR +& decode.io.task_s3.valid + issue.io.alrUseBuf) <= nrIssueBuf.U)

  // buffer [S2]
  bufReg_s2                 := Mux(block.io.task_s1.valid, block.io.task_s1.bits, bufReg_s2)
  shiftReg_s2.input(block.io.task_s1.valid)
  HardwareAssertion(PopCount(shiftReg_s2.s) <= 1.U)

  // decode [S3]
  decode.io.task_s2.valid   := shiftReg_s2.isValid
  decode.io.task_s2.bits    := bufReg_s2
  decode.io.respDir_s3      := io.respDir_s3

  // issue [S4]
  issue.io.task_s3          := decode.io.task_s3

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-1)
}