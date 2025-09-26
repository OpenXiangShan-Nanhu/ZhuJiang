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
import dongjiang.utils.VipEncoder
import freechips.rocketchip.util.DataToAugmentedData

class ChiRdMaster(node: Node)(implicit p: Parameters) extends ZJModule {
  private val rni = DmaParams(node = node)
  private val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamsUser = axiParams.copy(userBits = axiParams.idBits + log2Ceil(node.outstanding), idBits = log2Ceil(node.outstanding))

  val io = IO(new Bundle {
    val axiAr    = Flipped(Decoupled(new ARFlit(axiParamsUser)))
    val reqDB    = Decoupled(Bool())
    val respDB   = Input(Valid(new DataBufferAlloc(rni.bufferSize)))
    val chiReq   = Decoupled(new ReqFlit)
    val chiRxRsp = Flipped(Decoupled(new RespFlit))
    val chiTxRsp = Decoupled(new RespFlit)
    val chiDat   = Flipped(Decoupled(new DataFlit))
    val wrDB     = Decoupled(new writeRdDataBuffer(rni.bufferSize))
    val rdDB     = Decoupled(new readRdDataBuffer(rni.bufferSize, node.outstanding, axiParams))
    val finish   = Valid(new Finish(node))
  })

/* 
 * Reg/Wire Define
 */
  private val chiEntries     = RegInit(VecInit(Seq.fill(node.outstanding)((new CHIRdEntry(node)).Lit(_.valid -> false.B))))
  private val chiEntriesNext = WireInit(chiEntries)
  private val rdDBQueue      = Module(new Queue(gen = new RdDBEntry(node), entries = 2, flow = false, pipe = false))
  private val txDatPtr       = RegInit(0.U(1.W))
  private val finishBdl      = WireInit(0.U.asTypeOf(new Finish(node)))

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
  private val userArid_lo = log2Ceil(node.outstanding)
  private val userEid_hi  = log2Ceil(node.outstanding) - 1

  private val axiARID     = io.axiAr.bits.user(userArid_hi, userArid_lo)
  private val axiEid      = io.axiAr.bits.user(userEid_hi, 0)
  private val maxNid      = Fill(log2Ceil(node.outstanding), true.B)
  private val selReadDB   = WireInit(0.U(log2Ceil(node.outstanding).W))

/* 
 * Vec compute
 */


  private val sameARIdVec    = VecInit(chiEntries.zipWithIndex.map{case(c, i) => c.shodBlockRResp(axiARID) && !((selReadDB === i.U) && rdDBQueue.io.enq.fire)})
  private val sameARIdVecReg = RegEnable(sameARIdVec, io.axiAr.fire)

  private val shodSendReqVec = chiEntries.map(c => (c.reqNid === 0.U) && (c.shodSendReq))
  private val readDataVec    = chiEntries.map(c =>  c.shodSendData && c.ackNid === 0.U)
  private val sendAckVec     = chiEntries.map(c =>  c.shodSendAck)
  private val shodSendComVec = chiEntries.map(c =>  c.complete && c.valid && (c.finishNid === 0.U))

  private val sendReqValid   = shodSendReqVec.reduce(_ | _)
  private val readDBValid    = readDataVec.reduce(_ | _)
  private val sendAckValid   = sendAckVec.reduce(_ | _)
  private val sendFshValid   = shodSendComVec.reduce(_ | _)

// Sellect from Vec
  private val selSendReq        = VipEncoder(shodSendReqVec, sendReqValid)
  selReadDB                    := VipEncoder(readDataVec, readDBValid)
  private val selSendAck        = VipEncoder(sendAckVec, sendAckValid)
  private val selSendFinish     = PriorityEncoder(shodSendComVec)

  private val validVec       = chiEntries.map(c => c.valid)
  private val blockReqVec    = VecInit(chiEntries.zipWithIndex.map{case(c, i) => c.shodBlockReq(axiARID, axiEid) && !(rcvIsRct && (rctTxnid === i.U))})
  private val blockFshVec    = VecInit(chiEntries.zipWithIndex.map{case(c, i) => c.shodBlockFinish(axiEid) && !(sendFshValid && (selSendFinish === i.U))})
  private val blockReqVecReg = RegEnable(blockReqVec, io.axiAr.fire)
  private val blockFshVecReg = RegEnable(blockFshVec, io.axiAr.fire)

  //Pipe Reg
  private val rctTxnidPipe  = Pipe(rcvIsRct, rctTxnid)
  private val selReadDBPipe = Pipe(rdDBQueue.io.enq.fire, selReadDB)

  private val waitNidSetEn  = RegNext(io.axiAr.fire)
  private val selIdNextReg  = RegNext(io.axiAr.bits.id, 0.U)

  finishBdl.idx            := selSendFinish
  finishBdl.streamID       := chiEntries(selSendFinish).eId

  private val finishPipe    = Pipe(shodSendComVec.reduce(_ | _), finishBdl)

/* 
 * CHI Entrys Assign logic
 */
  for(i <- chiEntries.indices) {
    when(io.axiAr.fire && (io.axiAr.bits.id === i.U)) {
      assert(!chiEntries(i).valid)
      assert(io.reqDB.ready)
      chiEntries(i).arToChi(io.axiAr.bits, maxNid, maxNid, maxNid, io.respDB.bits)
    }.elsewhen(validVec(i)) {
      chiEntries(i)  := chiEntriesNext(i)
    }
  }

  for(((en, e), i) <- chiEntriesNext.zip(chiEntries).zipWithIndex) {
    when(io.chiReq.fire && (io.chiReq.bits.TxnID === i.U)){
      assert(!e.state.sendReq && !e.state.sendData && !e.state.rcvRct && !e.state.rcvDataCmp)
      en.state.sendReq := true.B
    }

    when(rcvIsRct & (rctTxnid === i.U)){
      assert(!e.state.rcvRct)
      en.respErr      := io.chiRxRsp.bits.RespErr
      en.state.rcvRct := true.B
    }
    when(rctTxnidPipe.valid & (e.reqNid =/= 0.U) & (chiEntries(rctTxnidPipe.bits).arId === e.arId) && (chiEntries(rctTxnidPipe.bits).eId =/= e.eId)) {
      en.reqNid   := e.reqNid - 1.U
    }

    when(dataTxnid === i.U & io.chiDat.fire){
      assert(!e.state.rcvDataCmp)
      en.state.rcvData0 := Mux(io.chiDat.bits.DataID === 0.U, true.B, e.state.rcvData0)
      en.state.rcvData1 := Mux(io.chiDat.bits.DataID === 2.U, true.B, e.state.rcvData1)
      en.respErr        := Mux(e.respErr =/= 0.U, e.respErr, io.chiDat.bits.RespErr)
      en.homeNid        := io.chiDat.bits.HomeNID
      en.dbid           := io.chiDat.bits.DBID
    }

    when(e.state.rcvData0 && e.state.rcvData1 || (!e.fromDCT && !e.fullSize && (e.state.rcvData0 =/= e.state.rcvData1))) {
      en.state.rcvDataCmp := true.B
    }
    when(e.allTaskComp) {
      en.complete := true.B
    }

    when((dataTxnid === i.U) & io.chiDat.fire & !e.fullSize & fromDCT(io.chiDat.bits.SrcID)) {
      assert(!e.state.rcvDataCmp)
      en.fromDCT := true.B
    }

    when(selReadDBPipe.valid && (chiEntries(selReadDBPipe.bits).arId === e.arId) && (e.ackNid =/= 0.U)) {
      en.ackNid      := e.ackNid - 1.U
    }
    when(finishPipe.valid && (finishPipe.bits.streamID === e.eId) && (e.finishNid =/= 0.U)){
      en.finishNid   := e.finishNid - 1.U
    }

    when((selSendAck === i.U) && io.chiTxRsp.fire) {
      assert(!e.state.sendAck)
      en.state.sendAck    := true.B
    }
    when((selReadDB === i.U) & rdDBQueue.io.enq.fire) {
      assert(!e.state.sendData)
      en.state.sendData := true.B
    }
    when(shodSendComVec.reduce(_|_) && (selSendFinish === i.U)) {
      en.valid := false.B
    }
    when(waitNidSetEn && (selIdNextReg === i.U)) {
      en.reqNid   := PopCount(blockReqVecReg)
      en.ackNid   := PopCount(sameARIdVecReg)
      en.finishNid := PopCount(blockFshVecReg)
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
  io.chiDat.ready              := io.wrDB.ready
  io.rdDB.valid                := rdDBQueue.io.deq.valid
  io.rdDB.bits                 := txDatBdl
  io.finish.valid              := finishPipe.valid
  io.finish.bits.idx           := finishPipe.bits.idx
  io.finish.bits.streamID      := finishPipe.bits.streamID
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
}
