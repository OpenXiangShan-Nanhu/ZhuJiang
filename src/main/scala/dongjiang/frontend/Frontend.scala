package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg}
import dongjiang.frontend.decode.{CommitCode, Operations}

class Frontend(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config = new DJConfigIO()
    // CHI REQ/SNP
    val rxReq         = Flipped(Decoupled(new ReqFlit(false)))
    val rxSnp         = Flipped(Decoupled(new SnoopFlit()))
    // DIR Read/Resp
    val readDir_s1    = Decoupled(new DJBundle with HasAddr with HasPosIndex {
      val early       = Bool() // Early access to data
    })
    // From Directory
    val respDir_s3    = Flipped(Valid(new DJBundle {
      val dir         = new DirMsg()
      val alrDeqDB    = Bool()
    }))
    // To Backend
    val commit_s3     = Valid(new DJBundle {
      val chi         = new ChiTask
      val pos         = new PosIndex()
      val dir         = new DirMsg()
      val alrDeqDB    = Bool()
      val hasOps      = Bool()
      val commit      = new CommitCode()
    })
    val cmAlloc_s4    = new DJBundle {
      val recOps      = Input(new Operations())
      val ops         = Output(new Operations())
      val task        = new DJBundle {
        val chi       = new ChiTask with HasAddr
        val pos       = new PosIndex()
        val ops       = new Operations()
        val alrDeqDB  = Bool()
        val snpVec    = Vec(nrSfMetas, Bool())
      }
    }
    // Update PoS Message
    val updPosTag     = Input(Valid(new Addr with HasPosIndex))
    val cleanPos      = Input(Valid(new DJBundle with HasPosIndex {
      val isSnp       = Bool()
    }))
    // PoS Busy Signal
    val posBusy       = Output(UInt(2.W))
    // Resp to Node(RN/SN): ReadReceipt, DBIDResp, CompDBIDResp
    val fastResp      = Decoupled(new RespFlit())
  })


  /*
   * Module declaration
   */
  val req2Task    = Module(new ReqToChiTask())
  val snp2Task    = Module(new SnpToChiTask())
  // S0
  val reqTaskBuf  = Module(new TaskBuffer(sort = true, nrReqTaskBuf))
  val snpTaskBuf  = Module(new TaskBuffer(sort = false, nrSnpTaskBuf))
  // S1
  val posTable    = Module(new PoS(dirBank))
  val block       = Module(new Block(dirBank))
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
  decode.io.config          := io.config
  issue.io.config           := io.config

  // io
  io.readDir_s1             <> block.io.readDir_s1
  io.fastResp               <> fastDecoupledQueue(block.io.fastResp_s1) // TODO: queue size = nrDirBank
  io.posBusy                := posTable.io.busy
  io.commit_s3              := issue.io.commit_s3
  io.cmAlloc_s4             <> issue.io.cmAlloc_s4

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
  posTable.io.canNest       := decode.io.canNest_s3
  posTable.io.updTag        := io.updPosTag
  posTable.io.clean         := io.cleanPos


  // block [S1]
  block.io.chiTask_s0       := fastRRArb(Seq(snpTaskBuf.io.chiTask_s0, reqTaskBuf.io.chiTask_s0))
  block.io.posRetry_s1      := posTable.io.full_s1 | posTable.io.sleep_s1
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