package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.{CMTask, CommitTask, FastResp, ReqPoS, RespComp}
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
    val config        = new DJConfigIO()
    val dirBank       = Input(UInt(dirBankBits.W))
    // CHI REQ/SNP
    val rxReq         = Flipped(Decoupled(new ReqFlit(false)))
    val rxSnp         = Flipped(Decoupled(new SnoopFlit()))
    // To Data
    val reqDB_s1      = Decoupled(new HnTxnID with HasDataVec)
    val fastData_s3   = Decoupled(new DataTask)
    // DIR Read/Resp
    val readDir_s1    = Decoupled(new Addr with HasPackHnIdx)
    val respDir_s3    = Flipped(Valid(new PackDirMsg))
    // To Backend
    val cmtTask_s3    = Valid(new CommitTask with HasHnTxnID)
    val cmTaskVec     = Vec(nrTaskCM, Decoupled(new CMTask))
    // Get addr from PoS
    val getAddrVec    = Vec(3, Flipped(new GetAddr())) // txReq + txSnp + writeDir
    // Update PoS Message
    val reqPoS        = Flipped(new ReqPoS())
    val updPosTag     = Flipped(Valid(new Addr with HasAddrValid with HasPackHnIdx))
    val updPosNest    = Flipped(Valid(new PosCanNest))
    val cleanPos      = Flipped(Valid(new PosClean))
    // Resp to Node(RN/SN): ReadReceipt, DBIDResp, CompDBIDResp
    val fastResp_s1   = Decoupled(new FastResp())
    val respComp_s3   = Valid(new RespComp)
    // PoS Busy Signal
    val alrUsePoS     = Output(UInt(log2Ceil(nrPoS + 1).W))
    //  system is working
    val working       = Output(Bool())
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
  val block       = Module(new Block())
  // S2: Wait Directory Response
  val bufReg_s2   = RegInit(0.U.asTypeOf(block.io.task_s1.bits))
  val shiftReg_s2 = RegInit(0.U.asTypeOf(new Shift(readDirLatency-1)))
  // S3: Receive DirResp and Decode
  val decode      = Module(new Decode())
  // S4: Issue Task to Backend
  val issue       = Module(new Issue())

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
  io.readDir_s1             <> block.io.readDir_s1
  io.fastResp_s1            <> FastQueue(block.io.fastResp_s1, djparam.nrDirBank.max(2))
  io.alrUsePoS              := posTable.io.alrUsePoS
  io.respComp_s3            := decode.io.respComp_s3
  io.fastData_s3            <> decode.io.fastData_s3
  io.cmtTask_s3             := decode.io.cmtTask_s3
  io.cmTaskVec              <> issue.io.cmTaskVec
  io.working                := reqTaskBuf.io.working | snpTaskBuf.io.working | posTable.io.working

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
  posTable.io.alloc_s0      := fastArb(Seq(snpTaskBuf.io.allocPos_s0, reqTaskBuf.io.allocPos_s0))
  posTable.io.retry_s1      := block.io.retry_s1
  posTable.io.updNest       := io.updPosNest
  posTable.io.clean         := io.cleanPos
  posTable.io.updTag        := io.updPosTag
  posTable.io.reqPoS        <> io.reqPoS
  posTable.io.reqPoS.req.valid := io.reqPoS.req.valid & io.reqPoS.req.bits.dirBank === io.dirBank

  // block [S1]
  block.io.chiTask_s0       := fastArb(Seq(snpTaskBuf.io.chiTask_s0, reqTaskBuf.io.chiTask_s0))
  block.io.posBlock_s1      := posTable.io.block_s1
  block.io.hnIdx_s1         := posTable.io.hnIdx_s1
  block.io.alrUseBuf        := shiftReg_s2.s.orR +& decode.io.cmtTask_s3.valid + issue.io.alrUseBuf
  HardwareAssertion((shiftReg_s2.s.orR +& decode.io.cmtTask_s3.valid + issue.io.alrUseBuf) <= nrIssueBuf.U)

  // buffer [S2]
  bufReg_s2                 := Mux(block.io.task_s1.valid, block.io.task_s1.bits, bufReg_s2)
  shiftReg_s2.input(block.io.task_s1.valid)
  HardwareAssertion(PopCount(shiftReg_s2.s) <= 1.U)

  // decode [S3]
  decode.io.task_s2.valid   := shiftReg_s2.isValid
  decode.io.task_s2.bits    := bufReg_s2
  decode.io.respDir_s3      := io.respDir_s3

  // issue [S4]
  issue.io.cmTask_s3        := decode.io.cmTask_s3

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}