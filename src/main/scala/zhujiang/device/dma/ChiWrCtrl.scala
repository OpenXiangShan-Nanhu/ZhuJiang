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


class ChiWrCtrl(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
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

/* 
 * Reg/Wire Define
 */
  //AxiWr to ChiWr entrys
  private val chiEntrys = Reg(Vec(dmaParams.chiEntrySize, new CHIWEntry))
  //Pointer
  private val headPtr   = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val tailPtr   = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txReqPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val rxDBIDPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val reqDBPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val rxDatPtr  = RegInit(0.U.asTypeOf(new ChiDBPtr(dmaParams.chiEntrySize)))
  private val txDatPtr  = RegInit(0.U.asTypeOf(new ChiDBPtr(dmaParams.chiEntrySize)))
  private val txBPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txAckPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  //Wire Define
  private val txReqBdl  = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val axiBBdl   = WireInit(0.U.asTypeOf(io.axiB.bits))
  private val rdDBBdl   = WireInit(0.U.asTypeOf(io.rdDB.bits))
  private val txAckBdl  = WireInit(0.U.asTypeOf(io.chiTxRsp.bits))
  private val rcvIsDBID = WireInit(io.chiRxRsp.fire & (io.chiRxRsp.bits.Opcode === RspOpcode.DBIDResp | io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp))
  private val rcvIsComp = WireInit(io.chiRxRsp.fire & (io.chiRxRsp.bits.Opcode === RspOpcode.Comp | io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp))
  private val rspTxnid  = WireInit(io.chiRxRsp.bits.TxnID(log2Ceil(dmaParams.chiEntrySize) - 1, 0))
  private val txIsAck   = WireInit(io.rdDB.fire & io.rdDB.bits.withAck | io.chiTxRsp.fire)
  

/* 
 * Pointer logic
 */
  headPtr     := Mux(io.axiAw.fire, headPtr + 1.U, headPtr)
  reqDBPtr    := Mux(io.reqDB.fire, reqDBPtr + 1.U, reqDBPtr)
  txReqPtr    := Mux(io.chiReq.fire, txReqPtr + 1.U, txReqPtr)
  rxDBIDPtr   := Mux(rcvIsDBID, rxDBIDPtr + 1.U, rxDBIDPtr)
  txAckPtr    := Mux(txIsAck, txAckPtr + 1.U, txAckPtr)
  txBPtr      := Mux(io.axiB.fire, txBPtr + 1.U, txBPtr)
  tailPtr     := Mux(txBPtr =/= tailPtr & txAckPtr =/= tailPtr & !(txDatPtr.flag === tailPtr.flag & txDatPtr.set === tailPtr.value), tailPtr + 1.U, tailPtr)

  when(io.axiW.fire){
    rxDatPtr.PtrWrAdd(chiEntrys(rxDatPtr.set))
  }
  when(io.rdDB.fire){
    txDatPtr.PtrWrAdd(chiEntrys(txDatPtr.set))
  }

/* 
 * Assign logic
 */
  chiEntrys.zipWithIndex.foreach{
    case(e, i) =>
      when(io.axiAw.fire & headPtr.value === i.U){
        e.AWMesInit(io.axiAw.bits)
      }.elsewhen(io.reqDB.fire & reqDBPtr.value === i.U){
        e.dbSite1 := io.respDB.bits.buf(0)
        e.dbSite2 := io.respDB.bits.buf(1)
      }
      when(rcvIsDBID & rspTxnid === i.U){
        e.dbid   := io.chiRxRsp.bits.DBID
        e.tgtid  := io.chiRxRsp.bits.SrcID
      }
      when(rcvIsComp & rspTxnid === i.U){
        e.haveRcvComp := true.B
      }
  }

  txReqBdl.WReqInit(chiEntrys(txReqPtr.value), txReqPtr.value)
  rdDBBdl.dataID    := Mux(txDatPtr.poi === 0.U, 0.U, 2.U)
  rdDBBdl.set       := Mux(txDatPtr.poi === 0.U, chiEntrys(txDatPtr.set).dbSite1, chiEntrys(txDatPtr.set).dbSite2)
  rdDBBdl.tgtId     := chiEntrys(txDatPtr.set).tgtid
  rdDBBdl.txnID     := chiEntrys(txDatPtr.set).dbid
  rdDBBdl.withAck   := (chiEntrys(txDatPtr.set).haveRcvComp | (rcvIsComp & rspTxnid === txDatPtr.set )) &
  txAckPtr.flag === txDatPtr.flag & txAckPtr.value === txDatPtr.set

  axiBBdl           := 0.U.asTypeOf(axiBBdl)
  axiBBdl.id        := chiEntrys(txBPtr.value).awId

  txAckBdl          := 0.U.asTypeOf(txAckBdl)
  txAckBdl.SrcID    := 1.U
  txAckBdl.TxnID    := chiEntrys(txAckPtr.value).dbid
  txAckBdl.TgtID    := chiEntrys(txAckPtr.value).tgtid
  txAckBdl.Opcode   := RspOpcode.CompAck

/* 
 * IO Connection
 */

  io.axiAw.ready    := !isFull(headPtr, tailPtr)
  io.reqDB.bits     := chiEntrys(reqDBPtr.value).double
  io.reqDB.valid    := reqDBPtr =/= headPtr
  io.wrDB.bits.data := io.axiW.bits.data
  io.wrDB.bits.mask := io.axiW.bits.strb
  io.wrDB.bits.set  := Mux(rxDatPtr.poi === 0.U, chiEntrys(rxDatPtr.set).dbSite1, chiEntrys(rxDatPtr.set).dbSite2)
  io.wrDB.valid     := io.axiW.fire
  io.axiW.ready     := io.wrDB.ready & !(rxDatPtr.flag === reqDBPtr.flag & rxDatPtr.set === reqDBPtr.value)
  io.chiReq.valid   := (txReqPtr =/= headPtr) & ((rxDBIDPtr === txReqPtr) | (rxDBIDPtr =/= txReqPtr) & rcvIsDBID)
  io.chiReq.bits    := txReqBdl
  io.chiRxRsp.ready := true.B
  io.axiB.valid     := chiEntrys(txBPtr.value).haveRcvComp & txBPtr =/= txReqPtr
  io.axiB.bits      := axiBBdl
  io.rdDB.bits      := rdDBBdl
  io.rdDB.valid     := txDatPtr.asUInt =/= rxDatPtr.asUInt & !(txDatPtr.flag === rxDBIDPtr.flag & txDatPtr.set === rxDBIDPtr.value)
  io.chiTxRsp.valid := chiEntrys(txAckPtr.value).haveRcvComp & txAckPtr =/= txReqPtr
  io.chiTxRsp.bits  := txAckBdl

/* 
 * Assertion
 */
  when(rcvIsDBID){
    assert(rxDBIDPtr.value === rspTxnid)
  }

}
