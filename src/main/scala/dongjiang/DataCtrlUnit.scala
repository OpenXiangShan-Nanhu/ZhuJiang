package dongjiang.dcu

import zhujiang.chi._
import xijiang.Node
import dongjiang._
import dongjiang.chi._
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi._
import chisel3._
import chisel3.experimental.hierarchy.{instantiable, public}
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.perf.{DebugOptions, DebugOptionsKey, HasPerfLogging}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.sram._
import dongjiang.utils.FastArb._

/*
 * Read Req:
 * Free ----> ReadSram ----> Reading
 */
object DCURState {
  val width             = 2
  val Free              = "b00".U
  val ReadSram          = "b01".U
}


class DCUREntry(implicit p: Parameters) extends DJBundle {
  val state             = UInt(DCURState.width.W)
  val dsIndex           = UInt(dsIndexBits.W)
  val dsBank            = UInt(dsBankBits.W)
  val srcID             = UInt(fullNodeIdBits.W)
  val txnID             = UInt(chiTxnIdBits.W)
  val returnNID         = UInt(fullNodeIdBits.W)
  val returnTxnID       = UInt(chiTxnIdBits.W)
  val resp              = UInt(ChiResp.width.W)
}


/*
 * Write Req:
 * Free ----> SendDBIDResp ----> WaitData -----> WriteSram -----> Writting -----> (SendComp) -----> Free
 *                                                           ^
 *                                                    WaitReplReadDone
 */
object DCUWState {
  val width = 3
  // commom
  val Free              = "b000".U
  val SendDBIDResp      = "b001".U
  val WaitData          = "b010".U
  val WriteSram         = "b011".U
  val SendComp          = "b100".U
}

class DCUWEntry(implicit p: Parameters) extends DJBundle {
  val state             = UInt(DCUWState.width.W)
  val replRState        = UInt(DCURState.width.W)
  val data              = Vec(nrBeat, Valid(UInt(beatBits.W)))
  val dsIndex           = UInt(dsIndexBits.W)
  val dsBank            = UInt(dsBankBits.W)
  val srcID             = UInt(fullNodeIdBits.W)
  val txnID             = UInt(chiTxnIdBits.W)
  val returnTxnID       = UInt(chiTxnIdBits.W)

  def canWrite          = replRState === DCURState.Free
  def isLast            = PopCount(data.map(_.valid)) === (nrBeat - 1).U
}



// TODO: DMT
// TODO: DWT
/*
 * DCUs do not have sorting capabilities and must use the DWT transfer structure to sort by using Comp
 */
@instantiable
class DataCtrlUnit(nodes: Seq[Node])(implicit p: Parameters) extends DJRawModule
  with ImplicitClock with ImplicitReset with HasPerfLogging {
  // ------------------------------------------ IO declaration --------------------------------------------- //
  @public val io = IO(new Bundle {
    val friendsNodeIDVec = Input(Vec(nodes.length, Vec(nrFriendsNodeMax, UInt(fullNodeIdBits.W)))) // RN/HN Friend Node ID Vec
    val icns = MixedVec(nodes.map(n => new DeviceIcnBundle(n)))
  })
  @public val reset = IO(Input(AsyncReset()))
  @public val clock = IO(Input(Clock()))
  val implicitClock = clock
  val implicitReset = reset

  if(p(DebugOptionsKey).EnableDebug) {
    dontTouch(io)
  }

  val nrIcn = nodes.length

  require(1 <= nodes.length & nodes.length <= 2)

  io <> DontCare

// ----------------------------------------- Reg and Wire declaration ------------------------------------ //
  // CHI
  val rxReq     = Wire(new DecoupledIO(new ReqFlit))
  val rxDat     = Wire(new DecoupledIO(new DataFlit))
  val txRsp     = WireInit(0.U.asTypeOf(Decoupled(new RespFlit)))
  val txDat     = WireInit(0.U.asTypeOf(Decoupled(new DataFlit)))

  val rxReqVec  = Wire(Vec(nrIcn, new DecoupledIO(new ReqFlit)))
  val rxDatVec  = Wire(Vec(nrIcn, new DecoupledIO(new DataFlit)))
  val txRspVec  = Wire(Vec(nrIcn, new DecoupledIO(new RespFlit)))
  val txDatVec  = Wire(Vec(nrIcn, new DecoupledIO(new DataFlit)))

  io.icns.zip(rxReqVec).foreach { case(a, b) => a.rx.req.get  <> b }
  io.icns.zip(rxDatVec).foreach { case(a, b) => a.rx.data.get <> b }
  io.icns.zip(txRspVec).foreach { case(a, b) => a.tx.resp.get <> b}
  io.icns.zip(txDatVec).foreach { case(a, b) => a.tx.data.get <> b}

  rxReq <> fastArbDec(rxReqVec)
  rxDat <> fastArbDec(rxDatVec)

  val txDatDirVec = Wire(Vec(nrIcn, Bool()))
  val txRspDirVec = Wire(Vec(nrIcn, Bool()))
  txDatDirVec     := getDCUDirectByTgtID(txDat.bits.TgtID, io.friendsNodeIDVec)
  txRspDirVec     := getDCUDirectByTgtID(txRsp.bits.TgtID, io.friendsNodeIDVec)

  // txDat
  txDatVec.zipWithIndex.foreach {
    case(t, i) =>
      t.valid     := txDat.valid & txDatDirVec(i)
      t.bits      := txDat.bits
  }
  txDat.ready     := txDatVec(OHToUInt(txDatDirVec)).ready
  assert(PopCount(txDatDirVec) <= 1.U)
  assert(PopCount(txDatDirVec) === 1.U | !txDat.valid)

  // txRsp
  txRspVec.zipWithIndex.foreach {
    case(t, i) =>
      t.valid     := txRsp.valid & txRspDirVec(i)
      t.bits      := txRsp.bits
  }
  txRsp.ready     := txRspVec(OHToUInt(txRspDirVec)).ready
  assert(PopCount(txRspDirVec) <= 1.U)
  assert(PopCount(txRspDirVec) === 1.U | !txRsp.valid)


  // ReqBuf
  val wBufRegVec            = RegInit(VecInit(Seq.fill(djparam.nrDCUWBuf) { 0.U.asTypeOf(new DCUWEntry) }))
  val rBufRegVec            = RegInit(VecInit(Seq.fill(djparam.nrDCURBuf) { 0.U.asTypeOf(new DCUREntry) }))

  // sram read vec
  val sramRReadyVec         = Wire(Vec(djparam.nrDSBank, Bool()))
  val sramWReadyVec         = Wire(Vec(djparam.nrDSBank, Bool()))

  // DataStorage
  val ds                    = Seq.fill(djparam.nrDSBank) { Module(new DataStorage(sets = nrDSEntry)) }

  // ChiRespQueue
  val rRespQ                = Module(new Queue(new DataFlit(), entries = djparam.nrDCURespQ, flow = false, pipe = true))
  val rDatQ                 = Module(new Queue(Vec(nrBeat, UInt(beatBits.W)), entries = djparam.nrDCURespQ, flow = false, pipe = true))
  val sendBeatNumReg        = RegInit(0.U(beatNumBits.W))
  val respVec               = Wire(Vec(djparam.nrDSBank, Valid(UInt(dataBits.W))))


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------ S0: Receive Req From CHI -------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive Req
   */
  val wBufFreeVec           = wBufRegVec.map(_.state === DCUWState.Free)
  val rBufFreeVec           = rBufRegVec.map(_.state === DCURState.Free)
  val selRecWID             = PriorityEncoder(wBufFreeVec)
  val selRecRID             = PriorityEncoder(rBufFreeVec)
  val reqIsW                = isWriteX(rxReq.bits.Opcode)
  val reqIsRepl             = isReplace(rxReq.bits.Opcode)
  when(reqIsW | reqIsRepl) {
    rxReq.ready             := wBufFreeVec.reduce(_ | _)
  }.otherwise {
    rxReq.ready             := rBufFreeVec.reduce(_ | _)
  }


  /*
   * Send DBID Or Comp To Src
   */
  val wBufSCompVec          = wBufRegVec.map(_.state === DCUWState.SendComp)
  val wBufSDBIDVec          = wBufRegVec.map(_.state === DCUWState.SendDBIDResp)
  val selSCompID            = PriorityEncoder(wBufSCompVec)
  val selSDBID              = PriorityEncoder(wBufSDBIDVec)

  txRsp.valid               := wBufSDBIDVec.reduce(_ | _) | wBufSCompVec.reduce(_ | _)
  txRsp.bits.Opcode         := Mux(wBufSCompVec.reduce(_ | _), Comp,                         DBIDResp)
  txRsp.bits.DBID           := Mux(wBufSCompVec.reduce(_ | _), selSCompID,                   selSDBID)
  txRsp.bits.TgtID          := Mux(wBufSCompVec.reduce(_ | _), wBufRegVec(selSCompID).srcID, wBufRegVec(selSDBID).srcID)
  txRsp.bits.TxnID          := Mux(wBufSCompVec.reduce(_ | _), wBufRegVec(selSCompID).txnID, wBufRegVec(selSDBID).txnID)


  /*
    * Select Buf Send Req To SRAM
    */
  sramRReadyVec             := ds.map(_.io.read.ready)
  sramWReadyVec             := ds.map(_.io.write.ready)

  val willSendRVec          = rBufRegVec.map { case r => r.state === DCURState.ReadSram      & sramRReadyVec(r.dsBank) & rRespQ.io.enq.ready }
  val willSendReplVec       = wBufRegVec.map { case r => r.replRState === DCURState.ReadSram & sramRReadyVec(r.dsBank) & rRespQ.io.enq.ready }
  val willSendWVec          = wBufRegVec.map { case w => w.state === DCUWState.WriteSram     & sramWReadyVec(w.dsBank) & w.canWrite}

  val sramWID               = PriorityEncoder(willSendWVec) // TODO: dont use PriorityEncoder
  val sramRID               = PriorityEncoder(willSendRVec) // TODO: dont use PriorityEncoder
  val sramReplID            = PriorityEncoder(willSendReplVec) // TODO: dont use PriorityEncoder
  val sramRead              = willSendRVec.reduce(_ | _)
  val sramRepl              = !sramRead & willSendReplVec.reduce(_ | _)

  ds.zipWithIndex.foreach {
    case(d, i) =>
      d.io.read.valid       := rBufRegVec(sramRID).dsBank === i.U & (willSendRVec.reduce(_ | _) | willSendReplVec.reduce(_ | _))
      d.io.write.valid      := wBufRegVec(sramWID).dsBank === i.U &  willSendWVec.reduce(_ | _)
      d.io.read.bits        := rBufRegVec(sramRID).dsIndex
      d.io.write.bits.index := wBufRegVec(sramWID).dsIndex
      d.io.write.bits.data  := Cat(wBufRegVec(sramWID).data.map(_.bits).reverse)
      assert(wBufRegVec(sramWID).data.map(_.valid).reduce(_ & _) | !d.io.write.valid)
  }


  /*
    * Get Read CHI Resp
    */
  val sramRFire               = ds.map(_.io.read.fire).reduce(_ | _)
  val sramWFire               = ds.map(_.io.write.fire).reduce(_ | _)
  rRespQ.io.enq.valid         := sramRFire; assert(Mux(sramRFire, rRespQ.io.enq.ready, true.B))
  rRespQ.io.enq.bits          := DontCare
  rRespQ.io.enq.bits.Opcode   := Mux(sramRead, CompData,                        NonCopyBackWriteData)
  rRespQ.io.enq.bits.TgtID    := Mux(sramRead, rBufRegVec(sramRID).returnNID,   ddrcNodeId.U)
  rRespQ.io.enq.bits.TxnID    := Mux(sramRead, rBufRegVec(sramRID).returnTxnID, wBufRegVec(sramReplID).returnTxnID)
  rRespQ.io.enq.bits.Resp     := Mux(sramRead, rBufRegVec(sramRID).resp,        0.U)
  rRespQ.io.enq.bits.HomeNID  := Mux(sramRead, rBufRegVec(sramRID).srcID,       0.U)
  rRespQ.io.enq.bits.DBID     := Mux(sramRead, rBufRegVec(sramRID).txnID,       0.U)



  /*
    * Read Req Buf
    */
  rBufRegVec.zipWithIndex.foreach {
    case (r, i) =>
      switch(r.state) {
        is(DCURState.Free) {
          val hit       = rxReq.fire & !reqIsW & !reqIsRepl & selRecRID === i.U
          when(hit) {
            r.state     := DCURState.ReadSram
            r.dsIndex   := parseDCUAddr(rxReq.bits.Addr)._1
            r.dsBank    := parseDCUAddr(rxReq.bits.Addr)._2
            r.srcID     := rxReq.bits.SrcID
            r.txnID     := rxReq.bits.TxnID
            r.returnNID := rxReq.bits.ReturnNID
            r.returnTxnID := rxReq.bits.ReturnTxnID
            r.resp      := rxReq.bits.MemAttr(ChiResp.width - 1, 0)
          }
        }
        is(DCURState.ReadSram) {
          val hit       = sramRFire & sramRID === i.U & sramRead
          when(hit) {
            r           := 0.U.asTypeOf(r)
            r.state     := DCURState.Free
          }
        }
      }
  }


  /*
    * Write Req Buf Write State
    */
  wBufRegVec.zipWithIndex.foreach {
    case(w, i) =>
      switch(w.state) {
        is(DCUWState.Free) {
          val hit       = rxReq.fire & (reqIsW | reqIsRepl) & selRecWID === i.U
          when(hit) {
            w.state     := DCUWState.SendDBIDResp
            w.dsIndex   := parseDCUAddr(rxReq.bits.Addr)._1
            w.dsBank    := parseDCUAddr(rxReq.bits.Addr)._2
            w.srcID     := rxReq.bits.SrcID
            w.txnID     := rxReq.bits.TxnID
          }
        }
        is(DCUWState.SendDBIDResp) {
          val hit       = txRsp.fire & txRsp.bits.Opcode === DBIDResp & selSDBID === i.U
          when(hit) {
            w.state     := DCUWState.WaitData
          }
        }
        is(DCUWState.WaitData) {
          val hit       = rxDat.fire & rxDat.bits.TxnID === i.U
          when(hit) {
            w.state     := Mux(hit & w.isLast, DCUWState.WriteSram, w.state)
            w.data(toBeatNum(rxDat.bits.DataID)).valid  := true.B
            w.data(toBeatNum(rxDat.bits.DataID)).bits   := rxDat.bits.Data
          }
        }
        is(DCUWState.WriteSram) {
          val hit       = sramWFire & sramWID === i.U
          when(hit) {
            w.state     := DCUWState.SendComp
          }
        }
        is(DCUWState.SendComp) {
          val hit       = txRsp.fire & txRsp.bits.Opcode === Comp & selSCompID === i.U
          when(hit) {
            w           := 0.U.asTypeOf(w)
            w.state     := DCUWState.Free
          }
        }
      }
  }
  rxDat.ready := true.B


  /*
    * Write Req Buf Replace Read State
    */
  wBufRegVec.zipWithIndex.foreach {
    case (r, i) =>
      switch(r.replRState) {
        is(DCURState.Free) {
          val hit         = rxReq.fire & reqIsRepl & selRecWID === i.U
          when(hit) {
            r.replRState  := DCURState.ReadSram
            r.returnTxnID := rxReq.bits.ReturnTxnID
          }
        }
        is(DCURState.ReadSram) {
          val hit         = sramRFire & sramReplID === i.U & sramRepl
          when(hit) {
            r.replRState  := DCURState.Free
            r.returnTxnID := 0.U
          }
        }
      }
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------- S1: Receive SRAM Resp and Send RxDat ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
    * Receive SRAM Resp
    */
  val respId          = PriorityEncoder(respVec.map(_.valid))
  respVec             := ds.map(_.io.resp)
  rDatQ.io.enq.valid  := respVec.map(_.valid).reduce(_ | _)
  rDatQ.io.enq.bits.zipWithIndex.foreach { case (beat, i) =>
    beat := respVec(respId).bits(beatBits * (i + 1) - 1, beatBits * i)
  }
  assert(PopCount(respVec.map(_.valid)) <= 1.U)

  /*
    * Send Resp To CHI RxDat
    */
  sendBeatNumReg      := sendBeatNumReg + txDat.fire.asUInt
  txDat.valid         := rRespQ.io.deq.valid & rDatQ.io.deq.valid
  txDat.bits          := Mux(txDat.valid, rRespQ.io.deq.bits, 0.U.asTypeOf(txDat.bits))
  txDat.bits.Data     := rDatQ.io.deq.bits(sendBeatNumReg)
  txDat.bits.DataID   := toDataID(sendBeatNumReg)
  txDat.bits.BE       := Fill(rxDat.bits.BE.getWidth, 1.U(1.W))

  rRespQ.io.deq.ready := sendBeatNumReg === (nrBeat - 1).U & txDat.fire
  rDatQ.io.deq.ready  := sendBeatNumReg === (nrBeat - 1).U & txDat.fire



// ------------------------------------------------------------ Assertion ----------------------------------------------- //

  assert(Mux(rxReq.valid, rxReq.bits.Addr(fullAddrBits - 1, fullAddrBits - sTagBits) === 0.U, true.B))

  assert(Mux(rxReq.valid, rxReq.bits.Size === log2Ceil(djparam.blockBytes).U, true.B))

  assert(Mux(rDatQ.io.enq.valid, rDatQ.io.enq.ready, true.B))

  assert(Mux(rxReq.valid, rxReq.bits.Opcode === ReadNoSnp | rxReq.bits.Opcode === WriteNoSnpFull | rxReq.bits.Opcode === Replace, true.B))



// -------------------------------------------------- Perf Counter ------------------------------------------------------ //
  // DCURBuf
  require(djparam.nrDCURBuf >= 4 & djparam.nrDCURBuf % 4 == 0)
  for (i <- 0 until (djparam.nrDCURBuf / 4)) {
    XSPerfAccumulate(s"dcu_rBuf_group[${i}]_deal_req_cnt", rxReq.fire & isReadX(rxReq.bits.Opcode) & (i * 4).U <= selRecRID & selRecRID <= (i * 4 + 3).U)
  }
  XSPerfAccumulate("dcu_rBuf_deal_req_cnt", rxReq.fire & isReadX(rxReq.bits.Opcode))
  XSPerfAccumulate("dcu_rBuf_req_block_cnt", rxReq.valid & isReadX(rxReq.bits.Opcode) & PopCount(rBufFreeVec) === 0.U)
  // DCURBuf
  require(djparam.nrDCUWBuf >= 4 & djparam.nrDCUWBuf % 4 == 0)
  for (i <- 0 until (djparam.nrDCUWBuf / 4)) {
    XSPerfAccumulate(s"dcu_wBuf_group[${i}]_deal_req_cnt", rxReq.fire & (isWriteX(rxReq.bits.Opcode) | isReplace(rxReq.bits.Opcode)) & (i * 4).U <= selRecWID & selRecWID <= (i * 4 + 3).U)
  }
  XSPerfAccumulate("dcu_wBuf_deal_req_cnt", rxReq.fire & (isWriteX(rxReq.bits.Opcode) | isReplace(rxReq.bits.Opcode)))
  XSPerfAccumulate("dcu_wBuf_req_block_cnt", rxReq.valid & (isWriteX(rxReq.bits.Opcode) | isReplace(rxReq.bits.Opcode)) & PopCount(wBufFreeVec) === 0.U)
  // count replace cnt
  XSPerfAccumulate("dcu_wBuf_deal_replace_cnt", rxReq.fire & isReplace(rxReq.bits.Opcode))
}
