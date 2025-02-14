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


class ChiREntrys(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
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
    val axiAr  = Flipped(Decoupled(new ARFlit(axiParams)))
    val reqDB  = Decoupled(Bool())
    val respDB = Input(Valid(new ChiDataBufferCtrlEntry(dmaParams.bufferSize)))
    val chiReq = Decoupled(new ReqFlit)
    val chiRsp = Flipped(Decoupled(new RespFlit))
    val chiDat = Flipped(Decoupled(new DataFlit))
    val wrDB   = Decoupled(new writeRdDataBuffer(dmaParams.bufferSize))
    val rdDB   = Decoupled(new readRdDataBuffer(dmaParams.bufferSize))
  })

/* 
 * Reg/Wire Define
 */
  private val dEntrys  = Reg(Vec(dmaParams.chiEntrySize, new CHIREntry))
  private val dHeadPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val dTailPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))

  private val allocDBPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val sendReqPtr = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val readDBPtr  = RegInit(0.U.asTypeOf(new ChiDBPtr(dmaParams.chiEntrySize)))
  
  private val chiReqBdl  = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val readDBBdl  = WireInit(0.U.asTypeOf(io.rdDB.bits))
  private val dataTxnid  = io.chiDat.bits.TxnID(log2Ceil(dmaParams.chiEntrySize) - 1, 0)
  
/* 
 * Pointers Logic
 */
  when(io.axiAr.fire){
    dEntrys(dHeadPtr.value).ARMesInit(io.axiAr.bits)
    dHeadPtr := dHeadPtr + 1.U
  }

  when(io.respDB.valid){
    dEntrys(allocDBPtr.value).chiRAllocDB(io.respDB.bits)
    allocDBPtr := allocDBPtr + 1.U
  }

/* 
 * Send CHI Req Logic
 */
  when(io.chiReq.fire){
    dEntrys(sendReqPtr.value).haveSendReq := true.B
  }

  chiReqBdl.RReqInit(dEntrys(sendReqPtr.value), sendReqPtr.value)

/* 
 * Receive CHI Resp Logic
 */
  when(io.chiRsp.fire & io.chiRsp.bits.Opcode === RspOpcode.ReadReceipt){
    assert(io.chiRsp.bits.TxnID === sendReqPtr.value)
    sendReqPtr := sendReqPtr + 1.U
  }

/* 
 * Receive CHI Data Logic 
 */
  when(io.chiDat.fire){
    assert(io.chiDat.bits.Opcode === DatOpcode.CompData)
  }

  def fromDCT(x: UInt): Bool = {
  require(x.getWidth == niw)
  val fromCC = WireInit(false.B)
  if(zjParams.localRing.filter(_.nodeType == NodeType.CC).nonEmpty){
    fromCC := zjParams.localRing.filter(_.nodeType == NodeType.CC).map(_.nodeId.asUInt >> nodeAidBits === x >> nodeAidBits).reduce(_ | _)
  }
  else {
    fromCC := false.B
  }
  fromCC
  }
  io.wrDB.bits.data := io.chiDat.bits.Data
  io.wrDB.bits.set  := Mux(dEntrys(dataTxnid).double && io.chiDat.bits.DataID === 2.U, dEntrys(dataTxnid).dbSite2, dEntrys(dataTxnid).dbSite1)
  io.wrDB.valid     := Mux(dEntrys(dataTxnid).double, io.chiDat.valid, 
                            Mux(fromDCT(io.chiDat.bits.SrcID), Mux(dEntrys(dataTxnid).addr(5), io.chiDat.bits.DataID === 2.U && io.chiDat.valid, 
                              io.chiDat.bits.DataID === 0.U & io.chiDat.valid), io.chiDat.valid))

  when(io.wrDB.valid & io.chiDat.bits.DataID === 0.U){
    dEntrys(dataTxnid).haveWrDB1 := true.B
  }
  when(io.wrDB.valid & io.chiDat.bits.DataID === 2.U){
    dEntrys(dataTxnid).haveWrDB2 := true.B
  }

/* 
 * Read DataBuffer Logic 
 */
  readDBBdl.setBdl(dEntrys(readDBPtr.set), readDBPtr)
  when(io.rdDB.fire){
    readDBPtr.PtrRdAdd(dEntrys(readDBPtr.set))
  }
  when(io.rdDB.fire & io.rdDB.bits.last){
    dTailPtr := dTailPtr + 1.U
  }
 
/* 
 * IO Interface
 */
  io.axiAr.ready       := !isFull(dHeadPtr, dTailPtr)
  io.reqDB.valid       := allocDBPtr =/= dHeadPtr
  io.reqDB.bits        := dEntrys(allocDBPtr.value).double
  io.chiReq.valid      := sendReqPtr =/= dHeadPtr && !dEntrys(sendReqPtr.value).haveSendReq && (dEntrys(sendReqPtr.value).haveAllocDB | io.respDB.valid)
  io.chiReq.bits       := chiReqBdl
  io.chiRsp.ready      := true.B
  io.chiDat.ready      := true.B
  io.rdDB.valid        := dTailPtr =/= dHeadPtr & (dEntrys(readDBPtr.set).haveWrDB1 & dEntrys(readDBPtr.set).haveWrDB2 & dEntrys(readDBPtr.set).double | 
                          dEntrys(readDBPtr.set).haveWrDB1 & !dEntrys(readDBPtr.set).double)
  io.rdDB.bits         := readDBBdl

/* 
 * Assertion
 */
  when(io.chiReq.fire && io.chiReq.bits.Addr(raw - 1)){
    assert(io.chiReq.bits.Size <= 3.U, "AxiSpilt error, MMIO req conversion error")
  }
}