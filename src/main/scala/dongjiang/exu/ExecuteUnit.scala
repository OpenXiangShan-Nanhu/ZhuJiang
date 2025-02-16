package dongjiang.pcu.exu

import dongjiang._
import dongjiang.pcu._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.fastArb
import xs.utils.debug.{DomainInfo, HardwareAssertion}

class ExecuteUnit(implicit p: Parameters) extends DJModule {
// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val dcuID           = Input(UInt(dcuBankBits.W))
    val pcuID           = Input(UInt(pcuBankBits.W))
    // Intf <> Exu
    val req2Exu         = Flipped(Decoupled(new Req2ExuBundle()))
    val reqAck2Intf     = Decoupled(new ReqAck2IntfBundle())
    val resp2Intf       = Decoupled(new Resp2IntfBundle())
    val req2Intf        = Decoupled(new Req2IntfBundle())
    val resp2Exu        = Flipped(Decoupled(new Resp2ExuBundle()))
    // Req To DataBuffer
    val dbRCReq         = Decoupled(new DBRCReq())
  })

// --------------------- Modules declaration ------------------------//
  val directory     = Module(new DirectoryWrapper())
  val reqPipe       = Module(new ProcessPipe())
  val respPipe      = Module(new ProcessPipe())
  val mshrCtl       = Module(new MSHRCtl())

// --------------------------- Connection ---------------------------//
  directory.io.dirRReadyVec           <> mshrCtl.io.dirRReadyVec
  directory.io.dirRead                <> mshrCtl.io.dirRead
  directory.io.dirWrite(PipeID.RESP)  <> respPipe.io.dirWrite // Low bit is high priority
  directory.io.dirWrite(PipeID.REQ)   <> reqPipe.io.dirWrite
  directory.io.dirResp(PipeID.RESP)   <> respPipe.io.dirResp
  directory.io.dirResp(PipeID.REQ)    <> reqPipe.io.dirResp
  directory.io.readMshr               <> mshrCtl.io.dirReadMshr
  directory.io.mshrResp               <> mshrCtl.io.mshrResp2Dir


  mshrCtl.io.req2Exu                  <> io.req2Exu
  mshrCtl.io.reqAck2Intf              <> io.reqAck2Intf
  mshrCtl.io.resp2Exu                 <> io.resp2Exu
  mshrCtl.io.pipeTask(PipeID.RESP)    <> respPipe.io.task;  HardwareAssertion(!mshrCtl.io.pipeTask(PipeID.RESP).valid | mshrCtl.io.pipeTask(PipeID.RESP).bits.taskMes.pipeID === PipeID.RESP)
  mshrCtl.io.pipeTask(PipeID.REQ)     <> reqPipe.io.task;   HardwareAssertion(!mshrCtl.io.pipeTask(PipeID.REQ).valid  | mshrCtl.io.pipeTask(PipeID.REQ).bits.taskMes.pipeID === PipeID.REQ)
  mshrCtl.io.updMSHRVec(PipeID.RESP)  <> respPipe.io.updMSHR
  mshrCtl.io.updMSHRVec(PipeID.REQ)   <> reqPipe.io.updMSHR


  mshrCtl.io.pcuID          := io.pcuID
  mshrCtl.io.dcuID          := io.dcuID
  respPipe.io.pcuID         := io.pcuID
  respPipe.io.dcuID         := io.dcuID
  reqPipe.io.pcuID          := io.pcuID
  reqPipe.io.dcuID          := io.dcuID


  io.req2Intf               <> fastArb(Seq(respPipe.io.req2Intf, reqPipe.io.req2Intf))
  io.resp2Intf              <> fastArb(Seq(respPipe.io.resp2Intf, reqPipe.io.resp2Intf))
  io.dbRCReq                <> fastArb(Seq(respPipe.io.dbRCReq, reqPipe.io.dbRCReq))

// --------------------------- Assertion ---------------------------//
  HardwareAssertion(io.req2Exu.bits.from       <= IncoID.max.U | !io.req2Exu.valid)
  HardwareAssertion(io.resp2Intf.bits.from     === io.dcuID    | !io.resp2Intf.valid)
  HardwareAssertion(io.req2Intf.bits.from      === io.dcuID    | !io.req2Intf.valid)
  HardwareAssertion(io.resp2Exu.bits.from      <= IncoID.max.U | !io.resp2Exu.valid)

  HardwareAssertion(io.req2Exu.bits.to         === io.dcuID    | !io.req2Exu.valid)
  HardwareAssertion(io.reqAck2Intf.bits.to     <= IncoID.max.U | !io.reqAck2Intf.valid)
  HardwareAssertion(io.resp2Intf.bits.to       <= IncoID.max.U | !io.resp2Intf.valid)
  HardwareAssertion(io.req2Intf.bits.to        <= IncoID.max.U | !io.req2Intf.valid)
  HardwareAssertion(io.resp2Exu.bits.to        === io.dcuID    | !io.resp2Exu.valid)
  HardwareAssertion(io.dbRCReq.bits.to         <= IncoID.max.U | !io.resp2Exu.valid)

  HardwareAssertion.placePipe(3)
}