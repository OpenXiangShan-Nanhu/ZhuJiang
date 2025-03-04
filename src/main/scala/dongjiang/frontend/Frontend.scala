package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._

class Frontend(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config = new DJConfigIO()
    // CHI REQ/SNP
    val rxReq   = Flipped(Decoupled(new ReqFlit(false)))
    val rxSnp   = Flipped(Decoupled(new SnoopFlit()))
  })
  io <> DontCare


  /*
   * Module declaration
   */
  val req2Task    = Module(new ReqToChiTask())
  val snp2Task    = Module(new SnpToChiTask())
  val reqTaskBuf  = Module(new TaskBuffer(sort = true, nrReqTaskBuf))
  val snpTaskBuf  = Module(new TaskBuffer(sort = false, nrSnpTaskBuf))
  val posTable    = Module(new PoS(dirBank))
  val block       = Module(new Block(dirBank))


  /*
   * Connect
   */
  // config
  req2Task.io.config        := io.config
  snp2Task.io.config        := io.config
  posTable.io.config        := io.config

  // req2Task
  req2Task.io.rxReq         <> io.rxReq

  // reqTaskBuf
  reqTaskBuf.io.chiTask     <> req2Task.io.chiTask
  reqTaskBuf.io.retry_s1    := block.io.retry_s1
  reqTaskBuf.io.sleep_s1    := posTable.io.sleep_s1
  reqTaskBuf.io.wakeupVec   := posTable.io.wakeupVec

  // snp2Task and snpTaskBuf
  if(hasHnx) {
    // snp2Task
    snp2Task.io.rxSnp       <> io.rxSnp
    // snpTaskBuf
    snpTaskBuf.io.chiTask   <> snp2Task.io.chiTask
    reqTaskBuf.io.retry_s1  := block.io.retry_s1
    reqTaskBuf.io.sleep_s1  := DontCare // snp never sleep
    reqTaskBuf.io.wakeupVec := DontCare // not need to wakeup
    HardwareAssertion(!snpTaskBuf.io.task_s0.valid)
  } else {
    // DontCare snp2Task and snpTaskBuf
    snp2Task.io             <> DontCare
    snpTaskBuf.io           <> DontCare
  }

  // posTable
  posTable.io               <> DontCare
  posTable.io.req_s0         := fastRRArb(Seq(snpTaskBuf.io.req2Pos_s0, reqTaskBuf.io.req2Pos_s0))

  // block
  block.io                  <> DontCare
  block.io.task_s0          := fastRRArb(Seq(snpTaskBuf.io.task_s0, reqTaskBuf.io.task_s0))
  block.io.posRetry_s1      := posTable.io.full_s1 | posTable.io.sleep_s1
  block.io.posIdx_s1        := posTable.io.posIdx_s1
  block.io.useNum           := Fill(retryBits, 1.U)
  block.io.readDir_s1.ready := true.B

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-1)
}