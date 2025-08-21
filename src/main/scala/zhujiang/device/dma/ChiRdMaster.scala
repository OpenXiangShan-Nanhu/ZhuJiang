package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.cde.config.Parameters
import chisel3.experimental.BundleLiterals._
import zhujiang._
import zhujiang.chi._
import zhujiang.axi._
import xs.utils.sram._
import xijiang._
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper, UIntToMask}
import dongjiang.utils.StepRREncoder
import freechips.rocketchip.util.DataToAugmentedData

class ChiRdMaster(node: Node)(implicit p: Parameters) extends ZJModule {
  private val rni = DmaParams(node = node)
  private val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamsUser = AxiParams(addrBits = axiParams.addrBits, idBits = axiParams.idBits, userBits = axiParams.idBits + log2Ceil(rni.axiEntrySize), dataBits = axiParams.dataBits,
    attr = axiParams.attr, lenBits = axiParams.lenBits, qosBits = axiParams.qosBits, regionBits = axiParams.regionBits)
  require(axiParams.idBits >= log2Ceil(node.outstanding))

  val io = IO(new Bundle {
    val axiAr    = Flipped(Decoupled(new ARFlit(axiParamsUser)))
    val reqDB    = Decoupled(Bool())
    val respDB   = Input(Valid(new DataBufferAlloc(node.outstanding)))
    val chiReq   = Decoupled(new ReqFlit)
    val chiRxRsp = Flipped(Decoupled(new RespFlit))
    val chiTxRsp = Decoupled(new RespFlit)
    val chiDat   = Flipped(Decoupled(new DataFlit))
    val wrDB     = Decoupled(new writeRdDataBuffer(node.outstanding))
    val rdDB     = Decoupled(new readRdDataBuffer(node.outstanding, axiParams))
    val working  = Output(Bool())
  })

/* 
 * Reg/Wire Define
 */
  private val chiEntries     = RegInit(VecInit(Seq.fill(node.outstanding)((new CHIRdEntry(node)).Lit(_.state -> ChiRState.FREE))))
  private val chiEntriesNext = WireInit(chiEntries)
  private val rdDBQueue      = Module(new Queue(gen = new RdDBEntry(node), entries = 2, flow = false, pipe = false))
  private val txDatPtr       = RegInit(0.U(1.W))

  //Wire Define
  private val rcvIsRct   = io.chiRxRsp.fire & io.chiRxRsp.bits.Opcode === RspOpcode.ReadReceipt
  private val rctTxnid   = io.chiRxRsp.bits.TxnID(log2Ceil(node.outstanding) - 1, 0)
  private val dataTxnid  = io.chiDat.bits.TxnID(log2Ceil(node.outstanding) - 1, 0)
  private val addrInTag  = io.axiAr.bits.addr(rni.matchSet - 1, rni.offset)

  private val txReqBdl   = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val txDatBdl   = WireInit(0.U.asTypeOf(io.rdDB.bits))
  private val rdDBQBdl   = WireInit(0.U.asTypeOf(new RdDBEntry(node)))
  private val txRspBdl   = WireInit(0.U.asTypeOf(new DmaRspFlit))

  private val userArid_hi = io.axiAr.bits.user.getWidth - 1
  private val userArid_lo = log2Ceil(rni.axiEntrySize)
  private val userEid_hi  = log2Ceil(rni.axiEntrySize) - 1

/* 
 * Vec compute
 */

  private val validVec       = chiEntries.map(c => c.isValid)
  private val blockReqVec    = chiEntries.map(c => (c.arId === io.axiAr.bits.user(userArid_hi, userArid_lo)) && (c.eId =/= io.axiAr.bits.user(userEid_hi, 0)) && c.isToOrder)

  private val shodSendReqVec = chiEntries.map(c => (c.reqNid === 0.U) && (c.state === ChiRState.SENDREQ))
  private val readDataVec    = chiEntries.map(c => (c.state === ChiRState.SENDDAT) && c.ackNid === 0.U)
  private val sendAckVec     = chiEntries.map(c => c.state === ChiRState.SENDACK & !c.memAttr.device)
  private val sameARIdVec    = chiEntries.map(c => !c.haveSendData && (c.arId === io.axiAr.bits.user(userArid_hi, userArid_lo)) && c.isValid)

  private val sendReqValid   = shodSendReqVec.reduce(_ | _)
  private val readDBValid    = readDataVec.reduce(_ | _)
  private val sendAckValid   = sendAckVec.reduce(_ | _)

// Sellect from Vec
  private val sameARIdCnt       = PopCount(sameARIdVec)
  private val selSendReq        = StepRREncoder(shodSendReqVec, sendReqValid)
  private val selReadDB         = StepRREncoder(readDataVec, readDBValid)
  private val selSendAck        = StepRREncoder(sendAckVec, sendAckValid)

  //Pipe Reg
  private val shodBlockCnt  = PopCount(blockReqVec)
  private val rctTxnidPipe  = Pipe(rcvIsRct, rctTxnid)
  private val selReadDBPipe = Pipe(rdDBQueue.io.enq.fire, selReadDB)

/* 
 * CHI Entrys Assign logic
 */
  for(i <- chiEntries.indices) {
    when(io.axiAr.fire && (io.axiAr.bits.id === i.U)) {
        assert(chiEntries(i).state === ChiRState.FREE)
        assert(io.reqDB.ready)
        chiEntries(i).arToChi(io.axiAr.bits, shodBlockCnt, sameARIdCnt, io.respDB.bits)
    }.elsewhen(validVec(i)) {
        chiEntries(i)  := chiEntriesNext(i)
    }
  }

  for(((en, e), i) <- chiEntriesNext.zip(chiEntries).zipWithIndex) {
    when(io.chiReq.fire && (io.chiReq.bits.TxnID === i.U)){
        assert(e.state === ChiRState.SENDREQ)
        en.state    := ChiRState.WAITRCT
    }

    val fullRcvDataComp =  e.rcvData0 & e.rcvData1
    val halfRcvDataComp = (e.rcvData0 =/= e.rcvData1) && !e.fromDCT && !e.fullSize

    when(rcvIsRct & (rctTxnid === i.U)){
        assert(e.state === ChiRState.WAITRCT)
        en.respErr  := io.chiRxRsp.bits.RespErr
        en.state    := Mux(fullRcvDataComp | halfRcvDataComp, ChiRState.SENDACK, ChiRState.WAITDATA)
    }
    when(rctTxnidPipe.valid & (e.reqNid =/= 0.U) & (chiEntries(rctTxnidPipe.bits).arId === e.arId) && (chiEntries(rctTxnidPipe.bits).eId =/= e.eId)) {
        en.reqNid   := e.reqNid - 1.U
    }

    when(dataTxnid === i.U & io.chiDat.fire){
        assert(e.state === ChiRState.WAITDATA || e.state === ChiRState.WAITRCT)
        en.rcvData0 := Mux(io.chiDat.bits.DataID === 0.U, true.B, e.rcvData0)
        en.rcvData1 := Mux(io.chiDat.bits.DataID === 2.U, true.B, e.rcvData1)
        en.respErr  := Mux(e.respErr =/= 0.U, e.respErr, io.chiDat.bits.RespErr)
        en.homeNid  := io.chiDat.bits.HomeNID
        en.dbid     := io.chiDat.bits.DBID
        val fullDataRcvComp = (e.rcvData0 =/= e.rcvData1) && (e.state === ChiRState.WAITDATA)
        val halfDataRcvComp = !e.fullSize && !fromDCT(io.chiDat.bits.SrcID) && (e.state === ChiRState.WAITDATA)
        en.state    := Mux(fullDataRcvComp | halfDataRcvComp, ChiRState.SENDACK, e.state)
    }

    when((dataTxnid === i.U) & io.chiDat.fire & !e.fullSize & fromDCT(io.chiDat.bits.SrcID)) {
        assert(e.state === ChiRState.WAITDATA || e.state === ChiRState.WAITRCT)
        en.fromDCT := true.B
    }

    when(selReadDBPipe.valid && (chiEntries(selReadDBPipe.bits).arId === e.arId) && (e.ackNid =/= 0.U)) {
        en.ackNid      := e.ackNid - 1.U
    }
    when((selSendAck === i.U) && io.chiTxRsp.fire) {
        assert(e.state === ChiRState.SENDACK)
        en.state    := ChiRState.SENDDAT
    }
    when((selReadDB === i.U) & rdDBQueue.io.enq.fire) {
        assert(e.state === ChiRState.SENDDAT)
        en.state := ChiRState.FREE
    }
    when((e.state === ChiRState.SENDACK) & e.memAttr.device){
        en.state := ChiRState.SENDDAT
    }
  }


/* 
 * Bundle Assignment
 */

  when(io.rdDB.fire & !io.rdDB.bits.last & rdDBQueue.io.deq.bits.double){
    txDatPtr := 1.U
  }.elsewhen(io.rdDB.fire & io.rdDB.bits.last){
    txDatPtr := 0.U
  }
  txReqBdl.rdReqInit(chiEntries(selSendReq), selSendReq, node)
  txDatBdl.SetBdl(rdDBQueue.io.deq.bits, txDatPtr)
  rdDBQBdl.rdDBInit(chiEntries(selReadDB), selReadDB)
  txRspBdl.compAckInit(chiEntries(selSendAck), node)

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


/* 
 * IO Connection
 */
  io.axiAr.ready               := io.reqDB.ready
  io.reqDB.valid               := io.axiAr.fire
  io.reqDB.bits                := io.axiAr.bits.len(0)
  io.chiReq.valid              := sendReqValid
  io.chiReq.bits               := txReqBdl
  io.chiRxRsp.ready            := true.B
  io.chiTxRsp.valid            := sendAckVec.reduce(_ | _)
  io.chiTxRsp.bits             := txRspBdl
  io.working                   := validVec.reduce(_ | _)
  io.chiDat.ready              := io.wrDB.ready
  io.rdDB.valid                := rdDBQueue.io.deq.valid
  io.rdDB.bits                 := txDatBdl
  io.wrDB.bits.data            := io.chiDat.bits.Data
  io.wrDB.bits.set             := Mux(chiEntries(dataTxnid).fullSize & io.chiDat.bits.DataID === 2.U, chiEntries(dataTxnid).dbSite2, chiEntries(dataTxnid).dbSite1)
  io.wrDB.valid                := Mux(chiEntries(dataTxnid).fullSize, io.chiDat.valid, 
                                  Mux(fromDCT(io.chiDat.bits.SrcID), Mux(chiEntries(dataTxnid).addr(rni.offset - 1), io.chiDat.bits.DataID === 2.U & io.chiDat.valid, io.chiDat.valid & io.chiDat.bits.DataID === 0.U), io.chiDat.valid))


  rdDBQueue.io.enq.valid        := readDBValid
  rdDBQueue.io.enq.bits         := rdDBQBdl
  rdDBQueue.io.deq.ready        := io.rdDB.ready && (!rdDBQueue.io.deq.bits.double || rdDBQueue.io.deq.bits.double && (txDatPtr === 1.U))

/* 
 * Assertion
 */
  when(rcvIsRct) {
    assert(!io.chiDat.fire)
  }
}