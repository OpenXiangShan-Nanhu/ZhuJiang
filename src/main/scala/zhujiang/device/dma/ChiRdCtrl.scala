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
import dongjiang.utils.StepRREncoder

class ChiRdCtrl(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits)
  private val axiParamsUser = AxiParams(dataBits = dw, addrBits = raw, idBits = log2Ceil(dmaParams.chiEntrySize), userBits = axiParams.idBits)
  require(axiParams.idBits >= log2Ceil(dmaParams.chiEntrySize))

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
    val axiAr  = Flipped(Decoupled(new ARFlit(axiParamsUser)))
    val reqDB  = Decoupled(Bool())
    val respDB = Input(Valid(new ChiDataBufferCtrlEntry(dmaParams.bufferSize)))
    val chiReq = Decoupled(new ReqFlit)
    val chiRsp = Flipped(Decoupled(new RespFlit))
    val chiDat = Flipped(Decoupled(new DataFlit))
    val wrDB   = Decoupled(new writeRdDataBuffer(dmaParams.bufferSize))
    val rdDB   = Decoupled(new readRdDataBuffer(dmaParams.bufferSize, axiParams))
  })

/* 
 * Reg/Wire Define
 */
  // AxiRd to ChiRd Entrys
  private val chiEntrys = Reg(Vec(dmaParams.chiEntrySize, new CHIREntry))
  // Pointer
  private val headPtr   = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val tailPtr   = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val reqDBPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txReqPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val rxRctPtr  = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txDatPtr  = RegInit(0.U(1.W))
  //Wire define
  private val rcvIsRct   = io.chiRsp.fire & io.chiRsp.bits.Opcode === RspOpcode.ReadReceipt
  private val dataTxnid  = io.chiDat.bits.TxnID(log2Ceil(dmaParams.chiEntrySize) - 1, 0)
  private val txReqBdl   = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val txDatBdl   = WireInit(0.U.asTypeOf(io.rdDB.bits))
  // Vec define
  private val validVec   = WireInit(VecInit.fill(dmaParams.chiEntrySize){false.B})
  private val blockVec   = WireInit(VecInit.fill(dmaParams.chiEntrySize){false.B})
  private val sendVec    = WireInit(VecInit.fill(dmaParams.chiEntrySize){false.B})
  // Pipe Reg
  private val arFireReg    = RegNext(io.axiAr.fire)
  private val arBitsReg    = RegNext(io.axiAr.bits)
  private val blkNumReg    = RegNext(PopCount(blockVec) + (io.axiAr.bits.user === arBitsReg.user && arFireReg).asUInt - (io.rdDB.fire & io.rdDB.bits.last & (io.rdDB.bits.originId === io.axiAr.bits.user)).asUInt)
  private val selIdxReg    = RegInit(0.U(log2Ceil(dmaParams.chiEntrySize).W))

/* 
 * Pointer logic
 */
  headPtr  := Mux(arFireReg & !isFull(headPtr, tailPtr), headPtr + 1.U, headPtr)
  reqDBPtr := Mux(io.reqDB.fire, reqDBPtr + 1.U, reqDBPtr)
  txReqPtr := Mux(io.chiReq.fire, txReqPtr + 1.U, txReqPtr)
  rxRctPtr := Mux(rcvIsRct, rxRctPtr + 1.U, rxRctPtr)
  tailPtr  := Mux(tailPtr =/= rxRctPtr & chiEntrys(tailPtr.value).sendComp, tailPtr + 1.U, tailPtr)
  txDatPtr := Mux(!io.rdDB.bits.last & io.rdDB.fire, 1.U, 0.U)


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

  for(idx <- validVec.indices){
    when(headPtr.flag === tailPtr.flag){
      validVec(idx) := Mux((tailPtr.value <= idx.U) & (headPtr.value > idx.U), true.B, false.B)
    }.otherwise {
      validVec(idx) := Mux((idx.U < headPtr.value) || (idx.U >= tailPtr.value), true.B, false.B)
    }
  }
  
  private val sameVec = chiEntrys.zipWithIndex.map{ case(c, i) => c.arId === io.axiAr.bits.user & !c.sendComp}
  private val zeroNid = chiEntrys.map(c => c.nid === 0.U & !c.sendComp & c.rcvDatComp)

  blockVec           := validVec.zip(sameVec).map{case(i, j) => i & j}
  sendVec            := validVec.zip(zeroNid).map{case(i, j) => i & j}
  when(io.rdDB.fire & io.rdDB.bits.last){
    selIdxReg := StepRREncoder(sendVec, io.rdDB.fire & io.rdDB.bits.last)
  }

/* 
 * Assign logic
 */
  chiEntrys.zipWithIndex.foreach{
    case(e, i) =>
      when(headPtr.value === i.U & arFireReg & !isFull(headPtr, tailPtr)){
        e.ARMesInit(arBitsReg)
        e.nid := blkNumReg - (io.rdDB.fire & io.rdDB.bits.last & io.rdDB.bits.originId === arBitsReg.user).asUInt
      }.elsewhen(reqDBPtr.value === i.U & io.reqDB.fire){
        e.dbSite1    := io.respDB.bits.buf(0)
        e.dbSite2    := io.respDB.bits.buf(1)
      }.elsewhen((!e.double & !e.fromDCT & (e.haveWrDB1 ^ e.haveWrDB2)) | (e.fromDCT | e.double) & e.haveWrDB1 & e.haveWrDB2){
        e.rcvDatComp := true.B
      }
      when(dataTxnid === i.U & io.wrDB.fire & io.chiDat.bits.DataID === 0.U){
        e.haveWrDB1  := true.B
      }
      when(dataTxnid === i.U & io.wrDB.fire & io.chiDat.bits.DataID === 2.U){
        e.haveWrDB2  := true.B
      }
      when(io.rdDB.fire & io.rdDB.bits.last & (io.rdDB.bits.originId === e.arId) & (e.nid =/= 0.U)){
        e.nid        := e.nid - 1.U
      }
      when(io.chiDat.fire & fromDCT(io.chiDat.bits.SrcID) & dataTxnid === i.U){
        e.fromDCT    := true.B
      }
      when(io.rdDB.fire & e.nid === 0.U & io.rdDB.bits.id === e.idx & io.rdDB.bits.last){
        e.sendComp   := true.B
      }
  }
  txReqBdl.RReqInit(chiEntrys(txReqPtr.value), txReqPtr.value)
  txDatBdl.SetBdl(chiEntrys(selIdxReg), txDatPtr)


/* 
 * IO logic
 */
  io.axiAr.ready     := !isFull(headPtr, tailPtr)
  io.reqDB.valid     := reqDBPtr =/= headPtr
  io.reqDB.bits      := chiEntrys(reqDBPtr.value).double
  io.chiReq.valid    := (txReqPtr =/= reqDBPtr) & ((rxRctPtr === txReqPtr) | (rxRctPtr =/= txReqPtr) & rcvIsRct)
  io.chiReq.bits     := txReqBdl
  io.wrDB.bits.data  := io.chiDat.bits.Data
  io.wrDB.bits.set   := Mux(chiEntrys(dataTxnid).double & io.chiDat.bits.DataID === 2.U, chiEntrys(dataTxnid).dbSite2, chiEntrys(dataTxnid).dbSite1)
  io.wrDB.valid      := Mux(chiEntrys(dataTxnid).double, io.chiDat.valid, 
                          Mux(fromDCT(io.chiDat.bits.SrcID), Mux(chiEntrys(dataTxnid).addr(5), io.chiDat.bits.DataID === 2.U & io.chiDat.valid, 
                            io.chiDat.valid & io.chiDat.bits.DataID === 0.U), io.chiDat.valid))
  io.chiDat.ready    := io.wrDB.ready
  io.chiRsp.ready    := true.B
  io.rdDB.bits       := txDatBdl
  io.rdDB.valid      := RegNext(sendVec.reduce(_|_))
  

/* 
 * Assertion
 */

 when(io.chiDat.valid){
  assert(io.chiDat.bits.Opcode === DatOpcode.CompData)
 }
 assert(reqDBPtr <= headPtr)
 assert(txReqPtr <= reqDBPtr)
 assert(tailPtr <= rxRctPtr)
 assert(rxRctPtr <= txReqPtr)
}
