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
import dongjiang.utils.StepRREncoder

class ChiRdMaster(node: Node)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni = DmaParams(node = node)
  private val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamsUser = AxiParams(addrBits = axiParams.addrBits, idBits = axiParams.idBits, userBits = axiParams.idBits, dataBits = axiParams.dataBits,
    attr = axiParams.attr, lenBits = axiParams.lenBits, qosBits = axiParams.qosBits, regionBits = axiParams.regionBits)
  require(axiParams.idBits >= log2Ceil(node.outstanding))

  private class CirQChiEntryPtr extends CircularQueuePtr[CirQChiEntryPtr](node.outstanding)

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
    val respDB   = Input(Valid(new DataBufferAlloc(node.outstanding)))
    val chiReq   = Decoupled(new ReqFlit)
    val chiRxRsp = Flipped(Decoupled(new RespFlit))
    val chiTxRsp = if(rni.readDMT) Some(Decoupled(new RespFlit)) else None
    val chiDat   = Flipped(Decoupled(new DataFlit))
    val wrDB     = Decoupled(new writeRdDataBuffer(node.outstanding))
    val rdDB     = Decoupled(new readRdDataBuffer(node.outstanding, axiParams))
    val working  = Output(Bool())
  })
/* 
 * Reg/Wire Define
 */
  private val chiEntries     = Reg(Vec(node.outstanding, new CHIREntry(node)))
  private val chiEntriesNext = WireInit(chiEntries)
  // Pointer
  private val headPtr     = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val tailPtr     = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val reqDBPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txReqPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val rxRspPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txRspPtr    = RegInit(CirQChiEntryPtr(f = false.B, v = 0.U))
  private val txDatPtr    = RegInit(0.U(1.W))
  //Wire Define
  private val rcvIsRct   = io.chiRxRsp.fire & io.chiRxRsp.bits.Opcode === RspOpcode.ReadReceipt
  private val rctTxnid   = io.chiRxRsp.bits.TxnID(log2Ceil(node.outstanding) - 1, 0)
  private val dataTxnid  = io.chiDat.bits.TxnID(log2Ceil(node.outstanding) - 1, 0)
  private val txReqBdl   = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val txDatBdl   = WireInit(0.U.asTypeOf(io.rdDB.bits))
  private val rdDBQBdl   = WireInit(0.U.asTypeOf(new RdDBEntry(node)))
  private val txRspBdl   = WireInit(0.U.asTypeOf(new DmaRspFlit))
  //Pipe Reg
  private val selIdx     = WireInit(0.U(log2Ceil(node.outstanding).W))
  private val rdDBQueue  = Module(new Queue(gen = new RdDBEntry(node), entries = 2, flow = false, pipe = false))
  // Vec Define
  private val headPtrMask = UIntToMask(headPtr.value, node.outstanding)
  private val tailPtrMask = UIntToMask(tailPtr.value, node.outstanding)
  private val headXorTail = headPtrMask ^ tailPtrMask
  private val validVec   = Mux(headPtr.flag ^ tailPtr.flag, ~headXorTail, headXorTail)
  private val blockVec   = WireInit(VecInit.fill(node.outstanding){false.B})
  private val sendDBVec  = WireInit(VecInit.fill(node.outstanding){false.B})

/* 
 * Pointer logic
 */
  private val txReqPtrAdd       = io.chiReq.fire
  private val rxRspPtrAdd       = rcvIsRct
  private val rxRspPtrAddDouble = io.chiReq.fire & (io.chiReq.bits.Order === 0.U) & rcvIsRct
  private val txRspPtrAdd       = chiEntries(txRspPtr.value).haveSendAck.get & validVec(txRspPtr.value)

  headPtr  := Mux(io.axiAr.fire    , headPtr  + 1.U, headPtr )
  reqDBPtr := Mux(io.reqDB.fire    , reqDBPtr + 1.U, reqDBPtr)
  txReqPtr := Mux(txReqPtrAdd      , txReqPtr + 1.U, txReqPtr)
  txRspPtr := Mux(txRspPtrAdd      , txRspPtr + 1.U, txRspPtr)
  rxRspPtr := Mux(rxRspPtrAddDouble, rxRspPtr + 2.U, Mux(rxRspPtrAdd, rxRspPtr + 1.U, rxRspPtr))

  if(!rni.readDMT){
    tailPtr  := Mux(tailPtr =/= txReqPtr & chiEntries(tailPtr.value).sendComp, tailPtr + 1.U, tailPtr)
  } else {
    tailPtr  := Mux(chiEntries(tailPtr.value).haveSendAck.get & tailPtr =/= txReqPtr & chiEntries(tailPtr.value).sendComp, tailPtr + 1.U, tailPtr)
  }

  when(io.rdDB.fire & rdDBQueue.io.deq.valid & !io.rdDB.bits.last & rdDBQueue.io.deq.bits.double){
    txDatPtr := 1.U
  }.elsewhen(io.rdDB.fire & io.rdDB.bits.last){
    txDatPtr := 0.U
  }
  private val sameVec    = chiEntries.zipWithIndex.map{ case(c, i) => (c.arId === io.axiAr.bits.user) & !c.sendComp}
  private val zeroNid    = chiEntries.map(c => c.nid === 0.U & !c.sendComp & c.rcvDatComp)


  blockVec       := validVec.asBools.zip(sameVec).map{case(i, j) => i & j}
  sendDBVec      := validVec.asBools.zip(zeroNid).map{case(i, j) => i & j}
  selIdx         := StepRREncoder(sendDBVec, rdDBQueue.io.enq.ready)

  private val selIdxPipe = Pipe(rdDBQueue.io.enq.fire, selIdx)

/* 
 * CHI Entrys Assign Logic
 */
  for(i <- chiEntries.indices) {
    when(headPtr.value === i.U && io.axiAr.fire){
      chiEntries(i).ARMesInit(io.axiAr.bits)   // true
      chiEntries(i).nid := PopCount(blockVec)
    }.elsewhen(validVec(i)) {
      chiEntries(i) := chiEntriesNext(i)
    }
  }
  for(((en, e), i) <- chiEntriesNext.zip(chiEntries).zipWithIndex) {
      when(reqDBPtr.value === i.U & io.reqDB.fire) {
        en.dbSite1  := io.respDB.bits.buf(0)
        en.dbSite2  := io.respDB.bits.buf(1)
      }.elsewhen(!e.double & !e.fromDCT & (e.haveWrDB1 =/= e.haveWrDB2) | (e.fromDCT | e.double) & e.haveWrDB1 & e.haveWrDB2) {
        en.rcvDatComp := true.B
      }
      when(rdDBQueue.io.enq.fire & (selIdx === i.U) & !(headPtr.value === i.U && io.axiAr.fire)){
        en.sendComp := true.B
      }
      when(selIdxPipe.valid & (chiEntries(selIdxPipe.bits).arId === en.arId) & (e.nid =/= 0.U) & !(headPtr.value === i.U && io.axiAr.fire)){
        en.nid := e.nid - 1.U
      }
      when(dataTxnid === i.U & io.chiDat.fire){
        en.haveWrDB1 := Mux(io.chiDat.bits.DataID === 0.U, true.B, e.haveWrDB1)
        en.haveWrDB2 := Mux(io.chiDat.bits.DataID === 2.U, true.B, e.haveWrDB2)
        en.respErr := Mux(e.respErr =/= 0.U, e.respErr, io.chiDat.bits.RespErr)
      }
      when(dataTxnid === i.U & io.chiDat.fire & !e.double & fromDCT(io.chiDat.bits.SrcID)){
        en.fromDCT   := true.B
      }
      when(rcvIsRct & (rctTxnid === i.U)){
        en.respErr  := io.chiRxRsp.bits.RespErr
      }
      if(rni.readDMT){
        when(io.wrDB.fire & dataTxnid === i.U){
          en.homeNid.get   := io.chiDat.bits.HomeNID
          en.dbid.get      := io.chiDat.bits.DBID
        }
        when(io.chiTxRsp.get.fire & txRspPtr.value === i.U || io.chiReq.fire & en.memAttr.device & (io.chiReq.bits.TxnID === i.U)){
          en.haveSendAck.get := true.B
        }
      }
  }

  def fromDCT(x: UInt): Bool = {
  require(x.getWidth == niw)
  val fromCC = WireInit(false.B)
  val rnfAid = (x.asTypeOf(new NodeIdBundle).aid === 1.U)
  if(zjParams.island.exists(_.nodeType == NodeType.CC)){
    fromCC := zjParams.island.filter(_.nodeType == NodeType.CC).map(_.nodeId.asUInt >> nodeAidBits === x >> nodeAidBits).reduce(_ | _)
  }
  else {
    fromCC := false.B
  }
  fromCC & rnfAid
  }

  rdDBQBdl.rdDBInit(chiEntries(selIdx))
  txReqBdl.RReqInit(chiEntries(txReqPtr.value), txReqPtr.value, node)
  txDatBdl.SetBdl(rdDBQueue.io.deq.bits, txDatPtr)
  txRspBdl.compAckInit(chiEntries(txRspPtr.value), node)

/* 
 * IO Connection
 */
  io.axiAr.ready              := !isFull(headPtr, tailPtr)
  io.reqDB.valid              := reqDBPtr =/= headPtr
  io.reqDB.bits               := chiEntries(reqDBPtr.value).double
  io.chiReq.valid             := (txReqPtr === rxRspPtr || rcvIsRct) & txReqPtr =/= reqDBPtr
  io.chiReq.bits              := txReqBdl
  io.wrDB.bits.data           := io.chiDat.bits.Data
  io.wrDB.bits.set            := Mux(chiEntries(dataTxnid).double & io.chiDat.bits.DataID === 2.U, chiEntries(dataTxnid).dbSite2, chiEntries(dataTxnid).dbSite1)
  io.wrDB.valid               := Mux(chiEntries(dataTxnid).double, io.chiDat.valid, 
                                  Mux(fromDCT(io.chiDat.bits.SrcID), Mux(chiEntries(dataTxnid).addr(rni.offset - 1), io.chiDat.bits.DataID === 2.U & io.chiDat.valid, io.chiDat.valid & io.chiDat.bits.DataID === 0.U), io.chiDat.valid))
  io.working                  := headPtr =/= tailPtr
  if(rni.readDMT){
    io.chiTxRsp.get.valid         := !chiEntries(txRspPtr.value).haveSendAck.get & (txRspPtr =/= rxRspPtr) & chiEntries(txRspPtr.value).rcvDatComp & validVec(txRspPtr.value)
    io.chiTxRsp.get.bits          := txRspBdl
  } 
  io.chiDat.ready             := io.wrDB.ready
  io.chiRxRsp.ready           := true.B
  io.rdDB.bits                := txDatBdl
  io.rdDB.valid               := rdDBQueue.io.deq.valid

  rdDBQueue.io.enq.valid      := sendDBVec.reduce(_|_)
  rdDBQueue.io.enq.bits       := rdDBQBdl
  rdDBQueue.io.deq.ready      := io.rdDB.ready & io.rdDB.bits.last

  when(rcvIsRct){
    assert(rctTxnid === rxRspPtr.value, "ReadReceipt Txnid is error!")
  }
}