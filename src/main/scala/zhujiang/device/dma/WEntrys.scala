package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.cde.config.Parameters
import zhujiang._
import zhujiang.chi._
import zhujiang.axi._
import xs.utils.sram._
import xijiang._
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper}


class ChiWEntrys(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits)

  private class CirQChiEntryPtr extends CircularQueuePtr[CirQChiEntryPtr](dmaParams.chiEntrySize)
  private object CirQChiEntryPtr {
  def apply(f: Bool, v: UInt): CirQChiEntryPtr = {
        val ptr = Wire(new CirQChiEntryPtr)
        ptr.flag := f
        ptr.value := v
        ptr
    }
  }

  val io = IO(new Bundle {
    val alloc    = Flipped(Decoupled(new AllocWrChiReq))
    val reqDB    = Decoupled(Bool())
    val respDB   = Input(Valid(new ChiDataBufferCtrlEntry(dmaParams.bufferSize)))
    val uW       = Flipped(Decoupled(new WFlit(axiParams))) 
    val uB       = Decoupled(new BFlit(axiParams))
    val dTxReq   = Decoupled(new ReqFlit)
    val dRxRsp   = Flipped(Decoupled(new RespFlit))
    val dTxRsp   = Decoupled(new RespFlit)
    val rdDB     = Decoupled(new readWrDataBuffer(dmaParams.bufferSize)) // Read DataBuffer
    val wrDB     = Decoupled(new writeWrDataBuffer(dmaParams.bufferSize))
  })

  private val dEntrys  = Reg(Vec(dmaParams.chiEntrySize, new CHIWEntry))
  private val dHeadPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dTailPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val uWPtr    = RegInit(0.U.asTypeOf(new ChiDBPtr(dmaParams.chiEntrySize)))

  private val chiReqBdl  = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val chiRspBdl  = WireInit(0.U.asTypeOf(new RespFlit))
  private val axiBBdl    = WireInit(0.U.asTypeOf(new BFlit(axiParams)))
  private val rdDBBdl    = WireInit(0.U.asTypeOf(new readWrDataBuffer(dmaParams.bufferSize)))

  private val sendReqPtr = RegInit(0.U(log2Ceil(dmaParams.chiEntrySize).W))
  private val reqDBPtr   = RegInit(0.U(log2Ceil(dmaParams.bufferSize).W))
  private val sendAckPtr = RegInit(0.U(log2Ceil(dmaParams.chiEntrySize).W))
  private val sendDatPtr = RegInit(0.U.asTypeOf(new ChiDBPtr(dmaParams.chiEntrySize)))
  private val sendBPtr   = RegInit(0.U(log2Ceil(dmaParams.chiEntrySize).W))

  when(io.alloc.fire){
    dEntrys(dHeadPtr.value).chiWEntryInit(io.alloc.bits)
    dHeadPtr := dHeadPtr + 1.U
  }
  when(io.respDB.valid){
    dEntrys(reqDBPtr).haveAllocDB := true.B
    dEntrys(reqDBPtr).dbSite1     := io.respDB.bits.buf(0)
    dEntrys(reqDBPtr).dbSite2     := io.respDB.bits.buf(1)
    reqDBPtr                      := reqDBPtr + 1.U
  }

  io.wrDB.bits.data := io.uW.bits.data
  io.wrDB.bits.mask := io.uW.bits.strb
  io.wrDB.bits.set  := Mux(uWPtr.poi === 0.U, dEntrys(uWPtr.set).dbSite1, dEntrys(uWPtr.set).dbSite2)
  
  when(io.uW.fire){
    dEntrys(uWPtr.set).wPtrAdd(io.uW.bits, uWPtr)
  }
  chiReqBdl.WReqInit(dEntrys(sendReqPtr), sendReqPtr)
  when(io.dTxReq.fire){
    dEntrys(sendReqPtr).haveSendReq := true.B
  }
  when(io.dRxRsp.fire && io.dRxRsp.bits.TxnID === sendReqPtr && (io.dRxRsp.bits.Opcode === RspOpcode.DBIDResp || io.dRxRsp.bits.Opcode === RspOpcode.CompDBIDResp)){
    sendReqPtr := sendReqPtr + 1.U
    dEntrys(sendReqPtr).dbid := io.dRxRsp.bits.DBID
    dEntrys(sendReqPtr).tgtid := io.dRxRsp.bits.SrcID
  }
  when(io.dRxRsp.fire & (io.dRxRsp.bits.Opcode === RspOpcode.Comp | io.dRxRsp.bits.Opcode === RspOpcode.CompDBIDResp)){
    dEntrys(io.dRxRsp.bits.TxnID).haveRcvComp := true.B
  }

  chiRspBdl       := 0.U.asTypeOf(chiRspBdl)
  chiRspBdl.SrcID := 1.U
  chiRspBdl.TxnID := dEntrys(sendAckPtr).dbid
  chiRspBdl.Opcode := RspOpcode.CompAck
  chiRspBdl.TgtID  := dEntrys(sendAckPtr).tgtid

  when(io.dTxRsp.bits.Opcode === RspOpcode.CompAck && io.dTxRsp.fire){
    sendAckPtr := sendAckPtr + 1.U
  }
  when(io.rdDB.fire){
    sendDatPtr.PtrWrAdd(dEntrys(sendDatPtr.set))
  }
  rdDBBdl.dataID := Mux(sendDatPtr.poi === 0.U, 0.U, 2.U)
  rdDBBdl.set    := Mux(sendDatPtr.poi === 0.U, dEntrys(sendDatPtr.set).dbSite1, dEntrys(sendDatPtr.set).dbSite2)
  rdDBBdl.tgtId  := dEntrys(sendDatPtr.set).tgtid
  rdDBBdl.txnID  := dEntrys(sendDatPtr.set).dbid

  sendBPtr   := Mux(!dEntrys(sendBPtr).last & sendBPtr < uWPtr.set | dEntrys(sendBPtr).last & io.uB.fire, sendBPtr + 1.U, sendBPtr)
  axiBBdl    := 0.U.asTypeOf(axiBBdl)
  axiBBdl.id := dEntrys(sendBPtr).awId

/* 
 * IO Interface Connection
 */
  io.uW.ready      := dEntrys(uWPtr.set).haveAllocDB & io.wrDB.ready
  io.alloc.ready   := !isFull(dHeadPtr, dTailPtr)
  io.reqDB.valid   := reqDBPtr =/= dHeadPtr.value
  io.reqDB.bits    := dEntrys(reqDBPtr).double
  io.dTxReq.valid  := sendReqPtr =/= dHeadPtr.value & !dEntrys(sendReqPtr).haveSendReq
  io.dTxReq.bits   := chiReqBdl
  io.dTxRsp.valid  := dEntrys(sendAckPtr).haveRcvComp
  io.dTxRsp.bits   := chiRspBdl
  io.dRxRsp.ready  := true.B
  io.wrDB.valid    := io.uW.valid
  io.rdDB.valid    := sendDatPtr.set < uWPtr.set
  io.rdDB.bits     := rdDBBdl
  io.uB.valid      := dEntrys(sendBPtr).last & sendBPtr < uWPtr.set
  io.uB.bits       := axiBBdl
}