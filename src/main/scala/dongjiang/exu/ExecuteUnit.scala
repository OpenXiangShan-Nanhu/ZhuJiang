package dongjiang.pcu.exu

import dongjiang._
import dongjiang.pcu._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils._
import dongjiang.utils.FastArb._

class ExecuteUnit(implicit p: Parameters) extends DJModule {
// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val hnfID           = Input(UInt(chiNodeIdBits.W))
    val valid           = Input(Bool())
    val sliceId         = Input(UInt(bankBits.W))
    // slice ctrl signals: RnNode <> Slice
    val req2Slice       = Flipped(Decoupled(new Req2SliceBundle()))
    val reqAck2Node     = Decoupled(new ReqAck2NodeBundle())
    val resp2Node       = Decoupled(new Resp2NodeBundle())
    val req2Node        = Decoupled(new Req2NodeBundle())
    val resp2Slice      = Flipped(Decoupled(new Resp2SliceBundle()))
    // Req To DataBuffer
    val dbRCReq         = Decoupled(new DBRCReq())
  })

// --------------------- Modules declaration ------------------------//
  val directory     = Module(new DirectoryWrapper())
  val reqPipe       = Module(new ProcessPipe())
  val respPipe      = Module(new ProcessPipe())
  val mshrCtl       = Module(new MSHRCtl())
  val mpReqQueue    = Module(new Queue(gen = new Req2NodeBundle(), entries = djparam.nrMpReqQueue, pipe = true, flow = true))
  val mpRespQueue   = Module(new Queue(gen = new Resp2NodeBundle(),entries = djparam.nrMpRespQueue, pipe = true, flow = true))

// --------------------------- Connection ---------------------------//
  directory.io.earlyRReqVec <> mshrCtl.io.earlyRReqVec
  directory.io.dirRead      <> mshrCtl.io.dirRead
  directory.io.dirWrite(0)  <> respPipe.io.dirWrite // Low bit is high priority
  directory.io.dirWrite(1)  <> reqPipe.io.dirWrite
  directory.io.dirResp(0)   <> respPipe.io.dirResp
  directory.io.dirResp(1)   <> reqPipe.io.dirResp
  directory.io.readMshr     <> mshrCtl.io.dirReadMshr
  directory.io.mshrResp     <> mshrCtl.io.mshrResp2Dir


  mshrCtl.io.sliceId        := io.sliceId
  mshrCtl.io.req2Slice      <> io.req2Slice
  mshrCtl.io.reqAck2node    <> io.reqAck2Node
  mshrCtl.io.resp2Slice     <> io.resp2Slice
  mshrCtl.io.pipeTask(0)    <> respPipe.io.task;  assert(!mshrCtl.io.pipeTask(PipeID.RESP).valid | mshrCtl.io.pipeTask(PipeID.RESP).bits.pipeId === PipeID.RESP)
  mshrCtl.io.pipeTask(1)    <> reqPipe.io.task;   assert(!mshrCtl.io.pipeTask(PipeID.REQ).valid  | mshrCtl.io.pipeTask(PipeID.REQ).bits.pipeId === PipeID.REQ)
  mshrCtl.io.updMSHR        <> fastPriorityArbDec(Seq(respPipe.io.updMSHR, reqPipe.io.updMSHR))
  mshrCtl.io.updLockMSHR(0) <> respPipe.io.updLockMSHR
  mshrCtl.io.updLockMSHR(1) <> reqPipe.io.updLockMSHR

  reqPipe.io.hnfID          := io.hnfID
  respPipe.io.hnfID         := io.hnfID
  reqPipe.io.sliceId        := io.sliceId
  respPipe.io.sliceId       := io.sliceId

  mpReqQueue.io.enq         <> fastPriorityArbDec(Seq(respPipe.io.req2Node, reqPipe.io.req2Node))
  mpRespQueue.io.enq        <> fastPriorityArbDec(Seq(respPipe.io.resp2Node, reqPipe.io.resp2Node))
  io.req2Node               <> mpReqQueue.io.deq
  io.resp2Node              <> mpRespQueue.io.deq
  io.dbRCReq                <> fastPriorityArbDec(Seq(respPipe.io.dbRCReq, reqPipe.io.dbRCReq))

  //  object connectPipe {
  //    def apply[T <: Bundle with HasPipeID](in: DecoupledIO[T], out: Seq[DecoupledIO[T]]): Unit = {
  //      out.head.valid          := in.valid & in.bits.toReqPipe
  //      out.last.valid          := in.valid & in.bits.toRespPipe
  //      out.foreach(_.bits      := in.bits)
  //      in.ready                := out.head.ready & in.bits.toReqPipe | out.last.ready & in.bits.toRespPipe
  //    }
  //    def apply[T <: Bundle with HasPipeID](in: ValidIO[T], out: Seq[ValidIO[T]]): Unit = {
  //      out.head.valid          := in.valid & in.bits.toReqPipe
  //      out.last.valid          := in.valid & in.bits.toRespPipe
  //      out.foreach(_.bits      := in.bits)
  //    }
  //  }
}