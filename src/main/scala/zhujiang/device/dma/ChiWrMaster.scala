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
import freechips.rocketchip.diplomacy.BufferParams.pipe
import dongjiang.bundle.Chi


class ChiWrMaster(node: Node)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private val rni           = DmaParams(node = node)
  private val axiParams     = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamsUser = AxiParams(addrBits = axiParams.addrBits, idBits = axiParams.idBits, userBits = axiParams.idBits + log2Ceil(rni.axiEntrySize), dataBits = axiParams.dataBits,
    attr = axiParams.attr, lenBits = axiParams.lenBits, qosBits = axiParams.qosBits, regionBits = axiParams.regionBits)
  private val axiWParamsUser = AxiParams(addrBits = axiParams.addrBits, idBits = log2Ceil(node.outstanding), userBits = log2Ceil(node.outstanding), dataBits = axiParams.dataBits,
    attr = axiParams.attr, lenBits = axiParams.lenBits, qosBits = axiParams.qosBits, regionBits = axiParams.regionBits)
  require(log2Ceil(node.outstanding) <= axiParamsUser.idBits)

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
    val reqDB    = Decoupled(Bool())
    val respDB   = Input(Valid(new DataBufferAlloc(node.outstanding)))
    val axiAw    = Flipped(Decoupled(new AWFlit(axiParamsUser)))
    val axiW     = Flipped(Decoupled(new WFlit(axiWParamsUser))) 
    val axiB     = Decoupled(new BFlit(axiParams))
    val chiReq   = Decoupled(new ReqFlit)
    val chiRxRsp = Flipped(Decoupled(new RespFlit))
    val chiTxRsp = Decoupled(new RespFlit)
    val rdDB     = Decoupled(new readWrDataBuffer(node.outstanding))
    val wrDB     = Decoupled(new writeWrDataBuffer(node.outstanding))
    val finish   = Valid(UInt(log2Ceil(node.outstanding).W))
    val working  = Output(Bool())
  })

/* 
 * Reg/Wire Define
 */
  //AxiWr to ChiWr entrys
  private val chiEntries     = RegInit(VecInit(Seq.fill(node.outstanding)((new CHIWEntry(node)).Lit(_.state -> ChiWState.FREE))))
  private val chiEntriesNext = WireInit(chiEntries)
  private val rdDBQueue      = Module(new Queue(gen = new RdDBWrEntry(node), entries = 2, flow = false, pipe = false))
  private val rdDBSitePtr    = RegInit(0.U(1.W))

  //Wire Define
  private val txReqBdl  = WireInit(0.U.asTypeOf(new DmaReqFlit))
  private val axiBBdl   = WireInit(0.U.asTypeOf(io.axiB.bits))
  private val toDBQBdl  = WireInit(0.U.asTypeOf(new RdDBWrEntry(node)))
  private val rdDBBdl   = WireInit(0.U.asTypeOf(io.rdDB.bits))
  private val txAckBdl  = WireInit(0.U.asTypeOf(io.chiTxRsp.bits))
  //Simplified Writing
  private val rcvResp    = io.chiRxRsp.fire & io.chiRxRsp.bits.TxnID(io.chiRxRsp.bits.TxnID.getWidth - 1)
  private val rcvIsDBID  = io.chiRxRsp.fire & (io.chiRxRsp.bits.Opcode === RspOpcode.DBIDResp | io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp) & io.chiRxRsp.bits.TxnID(io.chiRxRsp.bits.TxnID.getWidth - 1)
  private val rcvIsComp  = io.chiRxRsp.fire & (io.chiRxRsp.bits.Opcode === RspOpcode.Comp | io.chiRxRsp.bits.Opcode === RspOpcode.CompDBIDResp) & io.chiRxRsp.bits.TxnID(io.chiRxRsp.bits.TxnID.getWidth - 1)
  private val rspTxnid   = io.chiRxRsp.bits.TxnID(log2Ceil(node.outstanding) - 1, 0)
  private val bTransid   = io.axiB.bits.id(log2Ceil(node.outstanding) - 1, 0)
  private val txIsAck    = io.chiTxRsp.fire
  private val addrInTag  = io.axiAw.bits.addr(rni.matchSet - 1, rni.offset)
  private val rdDBPtrMod = io.rdDB.fire && ((rdDBQueue.io.deq.bits.fullSize && rdDBSitePtr === 0.U) || (rdDBSitePtr === 1.U))

  private val userArid_hi    = io.axiAw.bits.user.getWidth - 1
  private val userArid_lo    = log2Ceil(rni.axiEntrySize)
  private val userEid_hi     = log2Ceil(rni.axiEntrySize) - 1


/* 
 * Vec declare
 */
  private val validVec   = chiEntries.map(c => c.isValid)
  private val sendReqVec = chiEntries.map(c => (c.state === ChiWState.SENDREQ) && c.reqNid === 0.U)

  private val blockReqVec     = chiEntries.map(c => (c.awId  === io.axiAw.bits.user(userArid_hi, userArid_lo)) && (c.eId =/= io.axiAw.bits.user(userEid_hi, 0)) && c.isToOrder)
  private val blockAckVec     = chiEntries.map(c => (c.awId  === io.axiAw.bits.user(userArid_hi, userArid_lo)) && (c.eId =/= io.axiAw.bits.user(userEid_hi, 0)) && !c.haveRcvComp)
  private val waitDataVec     = chiEntries.map(c => c.isValid && !c.rcvDataComp)
  private val sendAckVec      = chiEntries.map(c => (c.state === ChiWState.SENDACK) && c.ackNid === 0.U)
  private val sendDataVec     = chiEntries.map(c => (c.state === ChiWState.SENDDAT))
  private val finishVec       = chiEntries.map(c => (c.state === ChiWState.FINISH)  && c.haveRcvComp && c.haveSendB)
  private val sendBVec        = chiEntries.map(c => c.haveRcvComp && c.rcvDataComp && !c.haveSendB && c.isValid && (c.bNid === 0.U))
  private val sameAWIdVec     = chiEntries.map(c => !c.haveSendB && (c.awId === io.axiAw.bits.user(userArid_hi, userArid_lo) && c.isValid))


  private val sendReqValid   = sendReqVec.reduce(_ | _)
  private val sendAckValid   = sendAckVec.reduce(_ | _)
  private val finishValid    = finishVec.reduce(_ | _)
  private val sendDataValid  = sendDataVec.reduce(_ | _)
  private val sendBValid     = sendBVec.reduce(_ | _)

//sellect from Vec
  private val selSendReq  = StepRREncoder(sendReqVec, sendReqValid)
  private val selSendAck  = StepRREncoder(sendAckVec, sendAckValid)
  private val selFinish   = StepRREncoder(finishVec, finishValid)
  private val selSendData = StepRREncoder(sendDataVec, sendDataValid)
  private val selSendB    = StepRREncoder(sendBVec, sendBValid)

// Pipe Reg
  private val dbidTxnidPipe   = Pipe(rcvIsDBID, rspTxnid)
  private val compTxnidPipe   = Pipe(rcvIsComp, rspTxnid)
  private val finishPipe      = Pipe(finishValid, selFinish)
  private val bidPipe         = Pipe(sendBValid, selSendB)

/* 
 * Assign logic
 */
  for(i <- chiEntriesNext.indices) {
    when(io.axiAw.fire & (io.axiAw.bits.id === i.U)) {
      assert(chiEntries(i).isFree)
      assert(io.reqDB.ready === true.B)
      chiEntries(i).awMesInit(io.axiAw.bits, PopCount(blockReqVec), PopCount(blockAckVec), PopCount(sameAWIdVec), io.respDB.bits)
    }.elsewhen(validVec(i)) {
      chiEntries(i) := chiEntriesNext(i)
    }
  }

  for(((en, e), i) <- chiEntriesNext.zip(chiEntries).zipWithIndex) {
    when(rcvResp & rspTxnid === i.U){
      assert((e.state =/= ChiWState.FREE) && (e.state =/= ChiWState.SENDREQ))
      en.dbid        := io.chiRxRsp.bits.DBID
      en.tgtid       := io.chiRxRsp.bits.SrcID
      en.state       := Mux(rcvIsDBID, Mux(e.rcvDataComp, ChiWState.SENDACK, ChiWState.WAITDATA), e.state)
    }
    when(dbidTxnidPipe.valid && (chiEntries(dbidTxnidPipe.bits).awId === e.awId) && (chiEntries(dbidTxnidPipe.bits).eId =/= e.eId) && (e.reqNid =/= 0.U)) {
      en.reqNid      := e.reqNid - 1.U
    }
    when(rcvIsComp & rspTxnid === i.U){
      assert((e.state =/= ChiWState.FREE) && (e.state =/= ChiWState.SENDREQ))
      en.haveRcvComp := true.B
    }
    when(compTxnidPipe.valid && (chiEntries(compTxnidPipe.bits).awId === e.awId) && (chiEntries(compTxnidPipe.bits).eId =/= e.eId) && (e.ackNid =/= 0.U)) {
      en.ackNid      := e.ackNid - 1.U
    }
    when((selSendAck === i.U) && io.chiTxRsp.fire) {
      assert(e.state === ChiWState.SENDACK)
      en.state       := ChiWState.SENDDAT
    }
    when(io.axiW.fire && io.axiW.bits._last && (io.axiW.bits.user === i.U)) {
      assert(!e.rcvDataComp)
      en.rcvDataComp := true.B
    }
    when(io.chiReq.fire && (selSendReq === i.U)) {
      assert(e.state === ChiWState.SENDREQ)
      en.state       := ChiWState.WAITDBID
    }
    when((selFinish === i.U) && finishValid) {
      assert(e.haveRcvComp && (e.state === ChiWState.FINISH))
      en.state       := ChiWState.FREE
    }
    when(io.axiB.fire && selSendB === i.U) {
      en.haveSendB   := true.B
    }
    when(bidPipe.valid && (chiEntries(bidPipe.bits).awId === e.awId) && (e.bNid =/= 0.U)) {
      en.bNid        := e.bNid - 1.U
    }
    when(e.state === ChiWState.WAITDATA) {
      en.state       := Mux(e.rcvDataComp, ChiWState.SENDACK, e.state)
    }
    when((selSendData === i.U) && rdDBQueue.io.enq.fire) {
      assert(e.state === ChiWState.SENDDAT)
      en.state       := ChiWState.FINISH
    }
  }

  rdDBSitePtr     := Mux(rdDBPtrMod, rdDBSitePtr + 1.U, rdDBSitePtr)

  txReqBdl.wReqInit(chiEntries(selSendReq), selSendReq, node)
  toDBQBdl.rdWrDBInit(chiEntries(selSendData))
  rdDBBdl.dataID   := Mux((rdDBSitePtr === 0.U) && (rdDBQueue.io.deq.bits.shift === 0.U), 0.U, 2.U)
  rdDBBdl.set      := Mux(rdDBSitePtr === 0.U, rdDBQueue.io.deq.bits.dbSite1, rdDBQueue.io.deq.bits.dbSite2)
  rdDBBdl.tgtId    := rdDBQueue.io.deq.bits.tgtId
  rdDBBdl.txnID    := rdDBQueue.io.deq.bits.txnID
  axiBBdl          := 0.U.asTypeOf(axiBBdl)
  axiBBdl.id       := selSendB
  txAckBdl         := 0.U.asTypeOf(txAckBdl)
  txAckBdl.SrcID   := rni.rniID.U
  txAckBdl.TxnID   := chiEntries(selSendAck).dbid
  txAckBdl.TgtID   := chiEntries(selSendAck).tgtid
  txAckBdl.Opcode  := RspOpcode.CompAck

  
/* 
 * IO Connection Logic
 */
  io.reqDB.bits     := io.axiAw.bits.len(0)
  io.reqDB.valid    := io.axiAw.fire
  io.wrDB.bits.data := io.axiW.bits.data
  io.wrDB.bits.mask := io.axiW.bits.strb
  io.wrDB.bits.set  := Mux(io.axiW.bits._last && chiEntries(io.axiW.bits.user).fullSize, chiEntries(io.axiW.bits.user).dbSite2, chiEntries(io.axiW.bits.user).dbSite1)
  io.wrDB.valid     := io.axiW.valid
  io.axiAw.ready    := io.reqDB.ready
  io.axiW.ready     := waitDataVec.reduce(_ | _) && io.wrDB.ready
  io.chiReq.valid   := sendReqValid
  io.chiReq.bits    := txReqBdl
  io.finish.valid   := finishPipe.valid
  io.finish.bits    := finishPipe.bits
  io.chiRxRsp.ready := true.B
  io.axiB.valid     := sendBValid
  io.axiB.bits      := axiBBdl
  io.rdDB.bits      := rdDBBdl
  io.rdDB.valid     := rdDBQueue.io.deq.valid
  io.chiTxRsp.bits  := txAckBdl
  io.chiTxRsp.valid := sendAckValid
  io.working        := RegNext(validVec.reduce(_ | _))


  rdDBQueue.io.enq.bits  := toDBQBdl
  rdDBQueue.io.enq.valid := sendDataValid
  rdDBQueue.io.deq.ready := io.rdDB.ready && (rdDBQueue.io.deq.bits.fullSize && (rdDBSitePtr === 1.U) || (!rdDBQueue.io.deq.bits.fullSize))
  

/* 
 * Assertion
 */
  when(rdDBQueue.io.deq.valid && !rdDBQueue.io.deq.bits.fullSize) {
    assert(rdDBSitePtr === 0.U)
  }

  when(io.axiW.fire && !io.axiW.bits.last.asBool) {
    assert(chiEntries(io.axiW.bits.user).fullSize)
  }
  when(io.axiB.fire) {
    assert(chiEntries(bTransid).bNid === 0.U)
  }

}