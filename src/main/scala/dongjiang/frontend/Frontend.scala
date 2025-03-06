package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import dongjiang.directory.DirEntry

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
    val readDir_s1    = Decoupled(new Addr with HasDCID {
      val early       = Bool() // Early access to data
    })
    val respDir_s3    = Input(new DJBundle {
      val llc         = new DirEntry("llc")
      val sf          = new DirEntry("sf")
    })
    // DB Req/Resp
    val reqDB_s1      = Decoupled(new DJBundle with HasLLCTxnID {
      val double      = Bool()
    })
    val respDB_s1     = Input(new DCID())
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
  val shiftReg_s2 = RegInit(0.U.asTypeOf(new Shift(readDirLatency)))

  /*
   * Connect
   */
  // config
  req2Task.io.config        := io.config
  snp2Task.io.config        := io.config
  posTable.io.config        := io.config

  // io
  io.readDir_s1             <> block.io.readDir_s1
  io.reqDB_s1               <> block.io.reqDB_s1
  io.fastResp               <> fastDecoupledQueue(block.io.fastResp_s1)
  io.posBusy                := posTable.io.busy

  // req2Task
  req2Task.io.rxReq         <> io.rxReq

  // reqTaskBuf [S0]
  reqTaskBuf.io.chiTask     <> req2Task.io.chiTask
  reqTaskBuf.io.retry_s1    := block.io.retry_s1
  reqTaskBuf.io.sleep_s1    := posTable.io.sleep_s1
  reqTaskBuf.io.wakeup      := posTable.io.wakeup

  // snp2Task and snpTaskBuf [S0]
  if(hasHnx) {
    // snp2Task
    snp2Task.io.rxSnp       <> io.rxSnp
    // snpTaskBuf [S0]
    snpTaskBuf.io.chiTask   <> snp2Task.io.chiTask
    reqTaskBuf.io.retry_s1  := block.io.retry_s1
    reqTaskBuf.io.sleep_s1  := DontCare // snp never sleep
    reqTaskBuf.io.wakeup    := DontCare // not need to wakeup
    HardwareAssertion(!snpTaskBuf.io.task_s0.valid)
  } else {
    // DontCare
    io.rxSnp                <> DontCare
    snp2Task.io             <> DontCare
    snpTaskBuf.io           <> DontCare
  }

  // posTable [S1]
  posTable.io.req_s0        := fastRRArb(Seq(snpTaskBuf.io.req2Pos_s0, reqTaskBuf.io.req2Pos_s0))
  posTable.io.retry_s1      := block.io.retry_s1
  posTable.io.canNest       := DontCare // TODO
  posTable.io.updTag        := io.updPosTag
  posTable.io.clean         := io.cleanPos


  // block [S1]
  block.io.task_s0          := fastRRArb(Seq(snpTaskBuf.io.task_s0, reqTaskBuf.io.task_s0))
  block.io.respDB_s1        := io.respDB_s1
  block.io.posRetry_s1      := posTable.io.full_s1 | posTable.io.sleep_s1
  block.io.posIdx_s1        := posTable.io.posIdx_s1
  block.io.willUseBufNum    := 0.U + shiftReg_s2.s.orR // TODO

  // buffer [S2]
  bufReg_s2                 := Mux(block.io.task_s1.valid, block.io.task_s1.bits, bufReg_s2)
  shiftReg_s2.input(block.io.task_s1.valid)
  HardwareAssertion(PopCount(shiftReg_s2.s) <= 1.U)
  HardwareAssertion(!shiftReg_s2.isValid) // TODO



  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-1)
}