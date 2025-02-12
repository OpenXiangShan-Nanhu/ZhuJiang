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
    val reqDB    = Decoupled(Bool())
    val respDB   = Input(Valid(new ChiDataBufferCtrlEntry(dmaParams.bufferSize)))
    val axiAw    = Flipped(Decoupled(new AWFlit(axiParams)))
    val axiW     = Flipped(Decoupled(new WFlit(axiParams))) 
    val axiB     = Decoupled(new BFlit(axiParams))
    val chiReq   = Decoupled(new ReqFlit)
    val chiRxRsp = Flipped(Decoupled(new RespFlit))
    val chiTxRsp = Decoupled(new RespFlit)
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

  private val sendReqPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val reqDBPtr   = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val sendDatPtr = RegInit(0.U.asTypeOf(new ChiDBPtr(dmaParams.chiEntrySize)))
  private val sendBPtr   = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))

  when(io.axiAw.fire){
    dEntrys(dHeadPtr.value).AWMesInit(io.axiAw.bits)
    dHeadPtr := dHeadPtr + 1.U
  }
  when(io.respDB.valid){
    dEntrys(reqDBPtr.value).haveAllocDB := true.B
    dEntrys(reqDBPtr.value).dbSite1     := io.respDB.bits.buf(0)
    dEntrys(reqDBPtr.value).dbSite2     := io.respDB.bits.buf(1)
    reqDBPtr                            := reqDBPtr + 1.U
  }

  io.wrDB.bits.data := io.axiW.bits.data
  io.wrDB.bits.mask := io.axiW.bits.strb
  io.wrDB.bits.set  := Mux(uWPtr.poi === 0.U, dEntrys(uWPtr.set).dbSite1, dEntrys(uWPtr.set).dbSite2)
  
  when(io.axiW.fire){
    uWPtr.PtrWrAdd(dEntrys(uWPtr.set))
  }
  chiReqBdl.WReqInit(dEntrys(sendReqPtr.value), sendReqPtr.value)
  when(io.chiReq.fire){
    dEntrys(sendReqPtr.value).haveSendReq := true.B
  }
  when(io.chiRxRsp.fire && io.chiRxRsp.bits.TxnID === sendReqPtr.value && (io.chiRxRsp.bits.Opcode === RspOpcode.DBIDResp || io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp)){
    sendReqPtr := sendReqPtr + 1.U
    dEntrys(sendReqPtr.value).dbid := io.chiRxRsp.bits.DBID
    dEntrys(sendReqPtr.value).tgtid := io.chiRxRsp.bits.SrcID
  }
  when(io.chiRxRsp.fire & (io.chiRxRsp.bits.Opcode === RspOpcode.Comp | io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp)){
    dEntrys(io.chiRxRsp.bits.TxnID(log2Ceil(dmaParams.chiEntrySize) - 1, 0)).haveRcvComp := true.B
  }
  when(io.chiRxRsp.fire & (io.chiRxRsp.bits.Opcode === RspOpcode.DBIDResp | io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp)){
    dEntrys(io.chiRxRsp.bits.TxnID(log2Ceil(dmaParams.chiEntrySize) - 1, 0)).haveRcvDBID := true.B
  }

  chiRspBdl       := 0.U.asTypeOf(chiRspBdl)
  chiRspBdl.SrcID := 1.U
  chiRspBdl.TxnID := dEntrys(dTailPtr.value).dbid
  chiRspBdl.Opcode := RspOpcode.CompAck
  chiRspBdl.TgtID  := dEntrys(dTailPtr.value).tgtid

  when(io.chiTxRsp.bits.Opcode === RspOpcode.CompAck && io.chiTxRsp.fire){
    dTailPtr := dTailPtr + 1.U
  }
  when(io.rdDB.fire){
    sendDatPtr.PtrWrAdd(dEntrys(sendDatPtr.set))
  }
  when(io.rdDB.fire & (dEntrys(sendDatPtr.set).double & sendDatPtr.poi === 1.U | !dEntrys(sendDatPtr.set).double)){
    dEntrys(sendDatPtr.set).haveRdData := true.B
  }

  rdDBBdl.dataID := Mux(sendDatPtr.poi === 0.U, 0.U, 2.U)
  rdDBBdl.set    := Mux(sendDatPtr.poi === 0.U, dEntrys(sendDatPtr.set).dbSite1, dEntrys(sendDatPtr.set).dbSite2)
  rdDBBdl.tgtId  := dEntrys(sendDatPtr.set).tgtid
  rdDBBdl.txnID  := dEntrys(sendDatPtr.set).dbid

  sendBPtr   := Mux(io.axiB.fire, sendBPtr + 1.U, sendBPtr)
  axiBBdl    := 0.U.asTypeOf(axiBBdl)
  axiBBdl.id := dEntrys(sendBPtr.value).awId

/* 
 * IO Interface Connection
 */
  io.axiAw.ready   := !isFull(dHeadPtr, dTailPtr)
  io.axiW.ready    := dEntrys(uWPtr.set).haveAllocDB & io.wrDB.ready
  io.reqDB.bits    := dEntrys(reqDBPtr.value).double
  io.reqDB.valid   := reqDBPtr =/= dHeadPtr 
  io.chiReq.valid  := sendReqPtr =/= dHeadPtr & !dEntrys(sendReqPtr.value).haveSendReq
  io.chiReq.bits   := chiReqBdl
  io.chiTxRsp.valid  := dEntrys(dTailPtr.value).haveRcvComp & dEntrys(dTailPtr.value).haveRdData & dTailPtr =/= dHeadPtr
  io.chiTxRsp.bits   := chiRspBdl
  io.chiRxRsp.ready  := true.B
  io.wrDB.valid      := io.axiW.valid
  io.rdDB.valid      := !(sendDatPtr.set === uWPtr.set & sendDatPtr.flag === uWPtr.flag) & dEntrys(sendDatPtr.set).haveRcvDBID
  io.rdDB.bits       := rdDBBdl
  io.axiB.valid      := !(sendBPtr.value === uWPtr.set & sendBPtr.flag === uWPtr.flag)
  io.axiB.bits       := axiBBdl
}