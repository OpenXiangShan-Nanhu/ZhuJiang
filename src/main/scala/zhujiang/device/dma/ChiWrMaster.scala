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
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper, UIntToMask}


class ChiWrMaster(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = rni.idBits)

  private class CirQChiEntryPtr extends CircularQueuePtr[CirQChiEntryPtr](rni.chiEntrySize)
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
    val respDB   = Input(Valid(new DataBufferAlloc(rni.dbEntrySize)))
    val axiAw    = Flipped(Decoupled(new AWFlit(axiParams)))
    val axiW     = Flipped(Decoupled(new WFlit(axiParams))) 
    val axiB     = Decoupled(new BFlit(axiParams))
    val chiReq   = Decoupled(new ReqFlit)
    val chiRxRsp = Flipped(Decoupled(new RespFlit))
    val chiTxRsp = Decoupled(new RespFlit)
    val rdDB     = Decoupled(new readWrDataBuffer(rni.dbEntrySize)) 
    val wrDB     = Decoupled(new writeWrDataBuffer(rni.dbEntrySize))
  })

/* 
 * Reg/Wire Define
 */
  //AxiWr to ChiWr entrys
  private val chiEntrys = Reg(Vec(rni.chiEntrySize, new CHIWEntry))
  private val chiEntriesNext = WireInit(chiEntrys)
  //Pointer
  private val headPtr   = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val tailPtr   = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txReqPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val rxDBIDPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val reqDBPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val rxDatPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txDatPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txBPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txAckPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))

  private val rxDatBeat = RegInit(0.U(1.W))
  private val txDatBeat = RegInit(0.U(1.W))
  //Wire Define
  private val txReqBdl  = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val axiBBdl   = WireInit(0.U.asTypeOf(io.axiB.bits))
  private val rdDBBdl   = WireInit(0.U.asTypeOf(io.rdDB.bits))
  private val txAckBdl  = WireInit(0.U.asTypeOf(io.chiTxRsp.bits))
  //Simplified Writing
  private val rcvResp    = io.chiRxRsp.fire & io.chiRxRsp.bits.TxnID(io.chiRxRsp.bits.TxnID.getWidth - 1)
  private val rcvIsDBID  = io.chiRxRsp.fire & (io.chiRxRsp.bits.Opcode === RspOpcode.DBIDResp | io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp) & io.chiRxRsp.bits.TxnID(io.chiRxRsp.bits.TxnID.getWidth - 1)
  private val rcvIsComp  = io.chiRxRsp.fire & (io.chiRxRsp.bits.Opcode === RspOpcode.Comp | io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp) & io.chiRxRsp.bits.TxnID(io.chiRxRsp.bits.TxnID.getWidth - 1)
  private val rspTxnid   = io.chiRxRsp.bits.TxnID(log2Ceil(rni.chiEntrySize) - 1, 0)
  private val  txIsAck   = io.rdDB.fire & io.rdDB.bits.withAck | io.chiTxRsp.fire

  private val tailPtrAdd = txBPtr =/= tailPtr & txAckPtr =/= tailPtr & txDatPtr =/=tailPtr

  // private val rxDatPtrAdd  = io.axiW.fire & ((rxDatBeat === 1.U) | (rxDatBeat === 0.U) & !chiEntrys(rxDatPtr.value).double)
  private val rxDatPtrAdd  = io.axiW.fire & io.axiW.bits.last
  private val txDatPtrAdd  = io.rdDB.fire & ((txDatBeat === 1.U) | (txDatBeat === 0.U) & !chiEntrys(txDatPtr.value).double)
  private val rxDatBeatAdd = (chiEntrys(rxDatPtr.value).double & (rxDatBeat === 0.U) || rxDatBeat === 1.U) & io.wrDB.fire
  private val txDatBeatAdd = (chiEntrys(txDatPtr.value).double & (txDatBeat === 0.U) || txDatBeat === 1.U) & io.rdDB.fire

  private val headPtrMask = UIntToMask(headPtr.value, rni.chiEntrySize)
  private val tailPtrMask = UIntToMask(tailPtr.value, rni.chiEntrySize)
  private val headXorTail = headPtrMask ^ tailPtrMask
  private val validVec   = Mux(headPtr.flag ^ tailPtr.flag, ~headXorTail, headXorTail)
  
/* 
 * Pointer logic
 */

  headPtr   := Mux(io.axiAw.fire , headPtr   + 1.U, headPtr  )
  rxDatPtr  := Mux(rxDatPtrAdd   , rxDatPtr  + 1.U, rxDatPtr )
  reqDBPtr  := Mux(io.reqDB.fire , reqDBPtr  + 1.U, reqDBPtr )
  txReqPtr  := Mux(io.chiReq.fire, txReqPtr  + 1.U, txReqPtr )
  rxDBIDPtr := Mux(rcvIsDBID     , rxDBIDPtr + 1.U, rxDBIDPtr)
  txDatPtr  := Mux(txDatPtrAdd   , txDatPtr  + 1.U, txDatPtr )
  txAckPtr  := Mux(txIsAck       , txAckPtr  + 1.U, txAckPtr )
  txBPtr    := Mux(io.axiB.fire  , txBPtr    + 1.U, txBPtr   )
  tailPtr   := Mux(tailPtrAdd    , tailPtr   + 1.U, tailPtr  )
  rxDatBeat := Mux(rxDatBeatAdd  , rxDatBeat + 1.U, rxDatBeat)
  txDatBeat := Mux(txDatBeatAdd  , txDatBeat + 1.U, txDatBeat)

/* 
 * Assign logic
 */
  for(i <- chiEntriesNext.indices) {
    when(io.axiAw.fire & (headPtr.value === i.U)) {
      chiEntrys(i) := chiEntrys(i).awMesInit(io.axiAw.bits)
    }.elsewhen(validVec(i)) {
      chiEntrys(i) := chiEntriesNext(i)
    }
  }
  chiEntriesNext.zipWithIndex.foreach{
    case(e, i) =>
      when(io.reqDB.fire & reqDBPtr.value === i.U){
        e.dbSite1  := io.respDB.bits.buf(0)
        e.dbSite2  := io.respDB.bits.buf(1)
      }
      when(rcvResp & rspTxnid === i.U){
        e.dbid     := io.chiRxRsp.bits.DBID
        e.tgtid    := io.chiRxRsp.bits.SrcID
      }
      when(rcvIsComp & rspTxnid === i.U){
        e.haveRcvComp := true.B
      }
  }

  txReqBdl.wReqInit(chiEntrys(txReqPtr.value), txReqPtr.value)
  rdDBBdl.dataID   := Mux((txDatBeat === 0.U) & !chiEntrys(txDatPtr.value).addr(rni.offset - 1), 0.U, 2.U)
  rdDBBdl.set      := Mux(txDatBeat === 0.U, chiEntrys(txDatPtr.value).dbSite1, chiEntrys(txDatPtr.value).dbSite2)
  rdDBBdl.tgtId    := chiEntrys(txDatPtr.value).tgtid
  rdDBBdl.txnID    := chiEntrys(txDatPtr.value).dbid
  if(rni.merWrDatAndAck) {
    rdDBBdl.withAck  := (txAckPtr === txDatPtr) & (chiEntrys(txDatPtr.value).haveRcvComp | (rcvIsComp & (rspTxnid === txDatPtr.value)))
  } else {
    rdDBBdl.withAck  := false.B
  }
  axiBBdl          := 0.U.asTypeOf(axiBBdl)
  axiBBdl.id       := chiEntrys(txBPtr.value).awId

  txAckBdl         := 0.U.asTypeOf(txAckBdl)
  txAckBdl.SrcID   := rni.rniID.U
  txAckBdl.TxnID   := chiEntrys(txAckPtr.value).dbid
  txAckBdl.TgtID   := chiEntrys(txAckPtr.value).tgtid
  txAckBdl.Opcode  := RspOpcode.CompAck

  
/* 
 * IO Connection Logic
 */
  io.axiAw.ready    := !isFull(headPtr, tailPtr)
  io.reqDB.bits     := chiEntrys(reqDBPtr.value).double
  io.reqDB.valid    := reqDBPtr =/= headPtr
  io.wrDB.bits.data := io.axiW.bits.data
  io.wrDB.bits.mask := io.axiW.bits.strb
  io.wrDB.bits.set  := Mux(rxDatBeat === 0.U, chiEntrys(rxDatPtr.value).dbSite1, chiEntrys(rxDatPtr.value).dbSite2)
  io.wrDB.valid     := io.axiW.fire
  io.axiW.ready     := io.wrDB.ready & (rxDatPtr =/= reqDBPtr)
  io.chiReq.valid   := (txReqPtr =/= headPtr) & ((rxDBIDPtr === txReqPtr) | (rxDBIDPtr =/= txReqPtr) & rcvIsDBID)
  io.chiReq.bits    := txReqBdl
  io.chiRxRsp.ready := true.B
  io.axiB.valid     := chiEntrys(txBPtr.value).haveRcvComp & txBPtr =/= txReqPtr
  io.axiB.bits      := axiBBdl
  io.rdDB.bits      := rdDBBdl
  io.rdDB.valid     := (txDatPtr =/= rxDatPtr) & (txDatPtr =/= rxDBIDPtr)
  io.chiTxRsp.bits  := txAckBdl
  io.chiTxRsp.valid := txAckPtr =/= rxDBIDPtr & !(io.rdDB.fire & io.rdDB.bits.withAck)

/* 
 * Assertion
 */
  assert(txReqPtr <= headPtr  )
  assert(reqDBPtr <= headPtr  )
  assert(txBPtr   <= txReqPtr )
  assert(txAckPtr <= rxDBIDPtr )
  assert(txDatPtr <= rxDBIDPtr)
  assert(tailPtr  <= txDatPtr )
  assert(tailPtr  <= txBPtr   )
  assert(tailPtr  <= txAckPtr )
  if(!rni.merWrDatAndAck) {
    assert(!io.rdDB.bits.withAck)
  }
}