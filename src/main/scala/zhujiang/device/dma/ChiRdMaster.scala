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

class ChiRdMaster(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = rni.idBits)
  private val axiParamsUser = AxiParams(dataBits = dw, addrBits = raw, idBits = log2Ceil(rni.chiEntrySize), userBits = axiParams.idBits)
  require(axiParams.idBits >= log2Ceil(rni.chiEntrySize))

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
    val axiAr    = Flipped(Decoupled(new ARFlit(axiParamsUser)))
    val reqDB    = Decoupled(Bool())
    val respDB   = Input(Valid(new DataBufferAlloc(rni.dbEntrySize)))
    val chiReq   = Decoupled(new ReqFlit)
    val chiRxRsp = Flipped(Decoupled(new RespFlit))
    val chiTxRsp = if(rni.readDMT) Some(Decoupled(new RespFlit)) else None
    val chiDat   = Flipped(Decoupled(new DataFlit))
    val wrDB     = Decoupled(new writeRdDataBuffer(rni.dbEntrySize))
    val rdDB     = Decoupled(new readRdDataBuffer(rni.dbEntrySize, axiParams))
  })
/* 
 * Reg/Wire Define
 */
  private val chiEntrys   = Reg(Vec(rni.chiEntrySize, new CHIREntry(dmt = rni.readDMT)))
  // Pointer
  private val headPtr     = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val tailPtr     = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val reqDBPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txReqPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txDatPtr    = RegInit(0.U(1.W))
  //Wire Define
  private val rcvIsRct   = io.chiRxRsp.fire & io.chiRxRsp.bits.Opcode === RspOpcode.ReadReceipt
  private val dataTxnid  = io.chiDat.bits.TxnID(log2Ceil(rni.chiEntrySize) - 1, 0)
  private val txReqBdl   = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val txDatBdl   = WireInit(0.U.asTypeOf(io.rdDB.bits))
  private val rdDBQBdl   = WireInit(0.U.asTypeOf(new RdDBEntry))
  private val txRspBdl   = WireInit(0.U.asTypeOf(new DmaRspFlit))
  //Pipe Reg
  private val selIdx     = WireInit(0.U(log2Ceil(rni.chiEntrySize).W))
  private val rdDBQueue  = Module(new Queue(gen = new RdDBEntry, entries = 2, flow = false, pipe = true))
  // Vec Define
  private val validVec   = WireInit(VecInit.fill(rni.chiEntrySize){false.B})
  private val blockVec   = WireInit(VecInit.fill(rni.chiEntrySize){false.B})
  private val sendDBVec  = WireInit(VecInit.fill(rni.chiEntrySize){false.B})
  private val sendAckVec = WireInit(VecInit.fill(rni.chiEntrySize){false.B})

/* 
 * Pointer logic
 */
  private val txReqPtrAdd = io.chiReq.fire & (io.chiReq.bits.Order === 0.U | (io.chiReq.bits.Order =/= 0.U) & rcvIsRct)
  headPtr  := Mux(io.axiAr.fire, headPtr  + 1.U, headPtr )
  reqDBPtr := Mux(io.reqDB.fire, reqDBPtr + 1.U, reqDBPtr)
  txReqPtr := Mux(txReqPtrAdd  , txReqPtr + 1.U, txReqPtr)

  if(!rni.readDMT){
    tailPtr  := Mux(tailPtr =/= txReqPtr & chiEntrys(tailPtr.value).sendComp, tailPtr + 1.U, tailPtr)
  } else {
    tailPtr  := Mux(chiEntrys(tailPtr.value).haveSendAck.get & tailPtr =/= txReqPtr & chiEntrys(tailPtr.value).sendComp, tailPtr + 1.U, tailPtr)
  }

  when(io.rdDB.fire & rdDBQueue.io.deq.valid & !io.rdDB.bits.last & rdDBQueue.io.deq.bits.double){
    txDatPtr := 1.U
  }.elsewhen(io.rdDB.fire & io.rdDB.bits.last){
    txDatPtr := 0.U
  }
  for(idx <- validVec.indices){
    when(headPtr.flag === tailPtr.flag){
      validVec(idx) := Mux((tailPtr.value <= idx.U) & (headPtr.value > idx.U), true.B, false.B)
    }.otherwise {
      validVec(idx) := Mux((idx.U < headPtr.value) || (idx.U >= tailPtr.value), true.B, false.B)
    }
  }
  private val sameVec    = chiEntrys.zipWithIndex.map{ case(c, i) => (c.arId === io.axiAr.bits.user) & !c.sendComp}
  private val zeroNid    = chiEntrys.map(c => c.nid === 0.U & !c.sendComp & c.rcvDatComp)
  private val rcvCompVec = chiEntrys.map(c => c.rcvDatComp && !c.haveSendAck.get)


  blockVec       := validVec.zip(sameVec).map{case(i, j) => i & j}
  sendDBVec      := validVec.zip(zeroNid).map{case(i, j) => i & j}
  sendAckVec     := validVec.zip(rcvCompVec).map{case(i, j) => i & j}
  selIdx         := StepRREncoder(sendDBVec, rdDBQueue.io.enq.ready)

/* 
 * CHI Entrys Assign Logic
 */
  chiEntrys.zipWithIndex.foreach{
    case(e, i) =>
      when(headPtr.value === i.U & io.axiAr.fire) {
        e.ARMesInit(io.axiAr.bits)
        e.nid := PopCount(blockVec) - (rdDBQueue.io.enq.fire & (rdDBQueue.io.enq.bits.arID === io.axiAr.bits.user)).asUInt
      }.elsewhen(reqDBPtr.value === i.U & io.reqDB.fire) {
        e.dbSite1  := io.respDB.bits.buf(0)
        e.dbSite2  := io.respDB.bits.buf(1)
      }.elsewhen(!e.double & !e.fromDCT & (e.haveWrDB1 =/= e.haveWrDB2) | (e.fromDCT | e.double) & e.haveWrDB1 & e.haveWrDB2) {
        e.rcvDatComp := true.B
      }
      when(rdDBQueue.io.enq.fire & (selIdx === i.U) & !(headPtr.value === i.U && io.axiAr.fire)){
        e.sendComp := true.B
      }
      when(rdDBQueue.io.enq.fire & (rdDBQueue.io.enq.bits.arID === e.arId) & (e.nid =/= 0.U) & !(headPtr.value === i.U && io.axiAr.fire)){
        e.nid := e.nid - 1.U
      }
      when(dataTxnid === i.U & io.wrDB.fire & e.double){
        e.haveWrDB1 := io.chiDat.bits.DataID === 0.U
        e.haveWrDB2 := io.chiDat.bits.DataID === 2.U
      }
      when(dataTxnid === i.U & io.chiDat.fire & !e.double & fromDCT(io.chiDat.bits.SrcID)){
        e.fromDCT   := true.B
        e.haveWrDB1 := Mux(io.wrDB.fire, true.B, e.haveWrDB1)
      }
      if(rni.readDMT){
        when(io.wrDB.fire & dataTxnid === i.U){
          e.homeNid.get   := io.chiDat.bits.HomeNID
          e.dbid.get      := io.chiDat.bits.DBID
        }
        when(io.chiTxRsp.get.fire & (io.chiTxRsp.get.bits.TxnID === e.dbid.get) & (headPtr.value =/= i.U)){
          e.haveSendAck.get := true.B
        }
      }
  }

  def fromDCT(x: UInt): Bool = {
  require(x.getWidth == niw)
  val fromCC = WireInit(false.B)
  if(zjParams.island.exists(_.nodeType == NodeType.CC)){
    fromCC := zjParams.island.filter(_.nodeType == NodeType.CC).map(_.nodeId.asUInt >> nodeAidBits === x >> nodeAidBits).reduce(_ | _)
  }
  else {
    fromCC := false.B
  }
  fromCC
  }

  rdDBQBdl.rdDBInit(chiEntrys(selIdx))
  txReqBdl.RReqInit(chiEntrys(txReqPtr.value), txReqPtr.value)
  txDatBdl.SetBdl(rdDBQueue.io.deq.bits, txDatPtr)
  txRspBdl.compAckInit(chiEntrys(PriorityEncoder(sendAckVec)))

/* 
 * IO Connection
 */
  io.axiAr.ready              := !isFull(headPtr, tailPtr)
  io.reqDB.valid              := reqDBPtr =/= headPtr
  io.reqDB.bits               := chiEntrys(reqDBPtr.value).double
  io.chiReq.valid             := txReqPtr =/= reqDBPtr
  io.chiReq.bits              := txReqBdl
  io.wrDB.bits.data           := io.chiDat.bits.Data
  io.wrDB.bits.set            := Mux(chiEntrys(dataTxnid).double & io.chiDat.bits.DataID === 2.U, chiEntrys(dataTxnid).dbSite2, chiEntrys(dataTxnid).dbSite1)
  io.wrDB.valid               := Mux(chiEntrys(dataTxnid).double, io.chiDat.valid, 
                                  Mux(fromDCT(io.chiDat.bits.SrcID), Mux(chiEntrys(dataTxnid).addr(rni.offset - 1), io.chiDat.bits.DataID === 2.U & io.chiDat.valid, io.chiDat.valid & io.chiDat.bits.DataID === 0.U), io.chiDat.valid))
  if(rni.readDMT){
    io.chiTxRsp.get.valid         := sendAckVec.reduce(_|_)
    io.chiTxRsp.get.bits          := txRspBdl
  } 
  io.chiDat.ready             := io.wrDB.ready
  io.chiRxRsp.ready           := true.B
  io.rdDB.bits                := txDatBdl
  io.rdDB.valid               := rdDBQueue.io.deq.valid

  rdDBQueue.io.enq.valid      := sendDBVec.reduce(_|_)
  rdDBQueue.io.enq.bits       := rdDBQBdl
  rdDBQueue.io.deq.ready      := io.rdDB.ready & io.rdDB.bits.last
}