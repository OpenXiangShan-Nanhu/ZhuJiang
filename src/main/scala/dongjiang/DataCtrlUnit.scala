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
import dongjiang.utils.fastArb
import dongjiang.utils.StepRREncoder
import dongjiang.utils.fastDecoupledQueue
import xs.utils.debug.{DomainInfo, HardwareAssertion}

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
  val isFlush           = Bool()
  val dsIndex           = UInt(dsIndexBits.W)
  val dsBank            = UInt(dsBankBits.W)
  val srcID             = UInt(fullNodeIdBits.W)
  val txnID             = UInt(chiTxnIdBits.W)
  val returnNID         = UInt(fullNodeIdBits.W)
  val returnTxnID       = UInt(chiTxnIdBits.W)
  val resp              = UInt(ChiResp.width.W)
  val beatOH            = UInt(2.W)

  def respOpcode        = Mux(isFlush, NonCopyBackWriteData, CompData)
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
  val beats             = Vec(nrBeat, Valid(new Bundle {
    val data            = UInt(beatBits.W)
    val mask            = UInt(maskBits.W)
  }))
  val dsIndex           = UInt(dsIndexBits.W)
  val dsBank            = UInt(dsBankBits.W)
  val srcID             = UInt(fullNodeIdBits.W)
  val txnID             = UInt(chiTxnIdBits.W)
  val returnTxnID       = UInt(chiTxnIdBits.W)
  val beatOH            = UInt(2.W)

  def canWrite          = replRState === DCURState.Free
}



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

// ----------------------------------------- Reg and Wire declaration ------------------------------------ //
  // CHI
  val rxReq     = Wire(new DecoupledIO(new ReqFlit(true)))
  val rxDat     = Wire(new DecoupledIO(new DataFlit))
  val txRsp     = WireInit(0.U.asTypeOf(Decoupled(new RespFlit)))
  val txDat     = WireInit(0.U.asTypeOf(Decoupled(new DataFlit)))

  val rxReqVec  = Wire(Vec(nrIcn, new DecoupledIO(new ReqFlit(true))))
  val rxDatVec  = Wire(Vec(nrIcn, new DecoupledIO(new DataFlit)))
  val txRspVec  = Wire(Vec(nrIcn, new DecoupledIO(new RespFlit)))
  val txDatVec  = Wire(Vec(nrIcn, new DecoupledIO(new DataFlit)))

  io.icns.zip(rxReqVec).foreach { case(a, b) => fastDecoupledQueue(a.rx.req.get,  b) }
  io.icns.zip(rxDatVec).foreach { case(a, b) => fastDecoupledQueue(a.rx.data.get, b) }
  io.icns.zip(txRspVec).foreach { case(a, b) => fastDecoupledQueue(b, a.tx.resp.get) }
  io.icns.zip(txDatVec).foreach { case(a, b) => fastDecoupledQueue(b, a.tx.data.get) }

  rxReq <> fastArb(rxReqVec) // Adding queues for timing considerations
  rxDat <> fastArb(rxDatVec) // Adding queues for timing considerations

  val txDatDirVec = Wire(Vec(nrIcn, Bool()))
  val txRspDirVec = Wire(Vec(nrIcn, Bool()))
  txDatDirVec     := getDCUDirectByTgtID(txDat.bits.TgtID, io.friendsNodeIDVec)
  txRspDirVec     := getDCUDirectByTgtID(txRsp.bits.TgtID, io.friendsNodeIDVec)

  // rxReq
  val beatOff     = parseDCUAddr(rxReq.bits.Addr)._1
  HardwareAssertion.withEn(!beatOff, rxReq.bits.Size === chiFullSize.U & rxReq.valid, rxReq.bits.Addr)
  HardwareAssertion.withEn(rxReq.bits.Size === chiFullSize.U, isReplace(rxReq.bits.Opcode) & rxReq.valid, rxReq.bits.Size)

  // txDat
  txDatVec.zipWithIndex.foreach {
    case(t, i) =>
      t.valid     := txDat.valid & txDatDirVec(i)
      t.bits      := txDat.bits
  }
  txDat.ready     := txDatVec(OHToUInt(txDatDirVec)).ready



  HardwareAssertion.withEn(PopCount(txDatDirVec) === 1.U, txDat.valid, cf"Illegal TxDat TgtID[${txDat.bits.TgtID}]", txDat.bits.TgtID)

  // txRsp
  txRspVec.zipWithIndex.foreach {
    case(t, i) =>
      t.valid     := txRsp.valid & txRspDirVec(i)
      t.bits      := txRsp.bits
  }
  txRsp.ready     := txRspVec(OHToUInt(txRspDirVec)).ready
  HardwareAssertion.withEn(PopCount(txRspDirVec) === 1.U, txRsp.valid, cf"Illegal TxRsp TgtID[${txRsp.bits.TgtID}]", txRsp.bits.TgtID)


  // ReqBuf
  val wBufRegVec            = RegInit(VecInit(Seq.fill(djparam.nrDCUWBuf) { 0.U.asTypeOf(new DCUWEntry) }))
  val rBufRegVec            = RegInit(VecInit(Seq.fill(djparam.nrDCURBuf) { 0.U.asTypeOf(new DCUREntry) }))

  // sram read vec
  val sramRReadyVec         = Wire(Vec(djparam.nrDSBank, Bool()))
  val sramWReadyVec         = Wire(Vec(djparam.nrDSBank, Bool()))

  // DataStorage
  val dsVec                 = Seq.fill(djparam.nrDSBank) { Seq.fill(nrBeat) { Module(new DataStorage(sets = nrDSEntry)) } }
  val dsRespIdPipe          = Module(new Pipe(new Bundle { val id = UInt(dsBankBits.W); val beatOH = UInt(2.W) }, latency = djparam.dcuSetup + djparam.dcuLatency + 1)) // Designed for timing optimization

  // ChiRespQueue
  val rRespQ                = Module(new Queue(new DataFlit(), entries = djparam.nrDCURespQ, flow = false, pipe = true))
  val rDataQ                = Module(new Queue(new Bundle { val beats = Vec(2, UInt(beatBits.W)); val beatOH = UInt(2.W) }, entries = djparam.nrDCURespQ, flow = false, pipe = true))
  val sendBeatNumReg        = RegInit(0.U(1.W))
  val dsRespVec             = Wire(Vec(djparam.nrDSBank, Vec(nrBeat, Valid(UInt(dataBits.W)))))


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
  val reqIsR                = isReadX(rxReq.bits.Opcode)
  val reqIsW                = isWriteX(rxReq.bits.Opcode)
  val reqIsRepl             = isReplace(rxReq.bits.Opcode)
  val reqIsFlush            = isFlush(rxReq.bits.Opcode)
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
  sramRReadyVec             := dsVec.map(_.map(_.io.read.ready).reduce(_ & _))
  sramWReadyVec             := dsVec.map(_.map(_.io.write.ready).reduce(_ & _))

  val willSendRVec          = rBufRegVec.map { case r => r.state === DCURState.ReadSram      & sramRReadyVec(r.dsBank) & rRespQ.io.enq.ready }
  val willSendReplVec       = wBufRegVec.map { case r => r.replRState === DCURState.ReadSram & sramRReadyVec(r.dsBank) & rRespQ.io.enq.ready }
  val willSendWVec          = wBufRegVec.map { case w => w.state === DCUWState.WriteSram     & sramWReadyVec(w.dsBank) & w.canWrite}
  val sramRead              = willSendRVec.reduce(_ | _)
  val sramRepl              = !sramRead & willSendReplVec.reduce(_ | _)

  val sramRFire             = dsVec.map(_.map(_.io.read.fire).reduce(_ | _)).reduce(_ | _); HardwareAssertion(PopCount(dsVec.map(_.map(_.io.read.fire).reduce(_ | _))) <= 1.U)
  val sramWFire             = dsVec.map(_.map(_.io.write.fire).reduce(_ | _)).reduce(_ | _); HardwareAssertion(PopCount(dsVec.map(_.map(_.io.write.fire).reduce(_ | _))) <= 1.U)

  val sramWID               = StepRREncoder(willSendWVec, sramWFire)
  val sramRID               = StepRREncoder(willSendRVec, sramRFire)
  val sramReplID            = StepRREncoder(willSendReplVec, sramRFire & !sramRead)


  dsVec.zipWithIndex.foreach {
    case (ds, i) =>
      ds.zipWithIndex.foreach {
        case (d, j) =>
          d.io.read.valid       := (sramRead & rBufRegVec(sramRID).dsBank === i.U & rBufRegVec(sramRID).beatOH(j)) | (sramRepl & wBufRegVec(sramReplID).dsBank === i.U)
          d.io.read.bits        := Mux(sramRead, rBufRegVec(sramRID).dsIndex, wBufRegVec(sramReplID).dsIndex)
          d.io.write.valid      := willSendWVec.reduce(_ | _) & wBufRegVec(sramWID).dsBank === i.U & wBufRegVec(sramWID).beatOH(j)
          d.io.write.bits.index := wBufRegVec(sramWID).dsIndex
          d.io.write.bits.beat  := wBufRegVec(sramWID).beats(j).bits.data
          d.io.write.bits.mask  := wBufRegVec(sramWID).beats(j).bits.mask
          HardwareAssertion.withEn(wBufRegVec(sramWID).beats(j).valid, d.io.write.valid, cf"WriteBuffer[${sramWID}] write DataStorage[${i}][${j}] with valid beats")
    }
    HardwareAssertion.placePipe(1)
  }
  HardwareAssertion.placePipe(2)
  dsRespIdPipe.io.enq.valid       := sramRFire
  dsRespIdPipe.io.enq.bits.id     := Mux(sramRead, rBufRegVec(sramRID).dsBank, wBufRegVec(sramReplID).dsBank)
  dsRespIdPipe.io.enq.bits.beatOH := Mux(sramRead, rBufRegVec(sramRID).beatOH, wBufRegVec(sramReplID).beatOH)

  /*
    * Get Read CHI Resp
    */
  rRespQ.io.enq.valid         := sramRFire; HardwareAssertion.withEn(rRespQ.io.enq.ready, sramRFire)
  rRespQ.io.enq.bits          := DontCare
  rRespQ.io.enq.bits.Opcode   := Mux(sramRead, rBufRegVec(sramRID).respOpcode,  NonCopyBackWriteData)
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
          val hit       = rxReq.fire & (reqIsR | reqIsFlush) & selRecRID === i.U
          when(hit) {
            r.state     := DCURState.ReadSram
            r.isFlush   := reqIsFlush
            r.dsIndex   := parseDCUAddr(rxReq.bits.Addr)._2
            r.dsBank    := parseDCUAddr(rxReq.bits.Addr)._3
            r.srcID     := rxReq.bits.SrcID
            r.txnID     := rxReq.bits.TxnID
            r.returnNID := rxReq.bits.ReturnNID.get
            r.returnTxnID := rxReq.bits.ReturnTxnID.get
            r.resp      := rxReq.bits.MemAttr(ChiResp.width - 1, 0)
            r.beatOH    := Mux(rxReq.bits.Size === chiFullSize.U, "b11".U, Cat(beatOff, !beatOff))
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
            w.dsIndex   := parseDCUAddr(rxReq.bits.Addr)._2
            w.dsBank    := parseDCUAddr(rxReq.bits.Addr)._3
            w.srcID     := rxReq.bits.SrcID
            w.txnID     := rxReq.bits.TxnID
            w.beatOH    := Mux(rxReq.bits.Size === chiFullSize.U, "b11".U, Cat(beatOff, !beatOff))
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
          val beatNum   = Mux(w.beatOH === "b10".U, 1.U, toBeatNum(rxDat.bits.DataID))
          val isLast    = Mux(w.beatOH === "b11".U, PopCount(w.beats.map(_.valid)) === 1.U, true.B)
          when(hit) {
            w.state     := Mux(hit & isLast, DCUWState.WriteSram, w.state)
            w.beats(beatNum).valid     := true.B
            w.beats(beatNum).bits.data := rxDat.bits.Data
            w.beats(beatNum).bits.mask := rxDat.bits.BE
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
  // HardwareAssertion
  val bufSel = rxDat.bits.TxnID(log2Ceil(wBufRegVec.length) - 1, 0)
  val beatNum = Mux(wBufRegVec(bufSel).beatOH === "b10".U, 1.U, toBeatNum(rxDat.bits.DataID))
  HardwareAssertion.withEn(!wBufRegVec(bufSel).beats(beatNum).valid, rxDat.fire, cf"WriteBuffer[${rxDat.bits.TxnID}] [${beatNum}]", rxDat.bits.TxnID, beatNum)

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
            r.returnTxnID := rxReq.bits.ReturnTxnID.get
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
  val dsRespID              = dsRespIdPipe.io.deq.bits.id
  val dsRespOH              = dsRespIdPipe.io.deq.bits.beatOH
  dsRespVec.zip(dsVec).foreach { case(a, b) => a := b.map(_.io.resp) }

  rDataQ.io.enq.valid       := dsRespIdPipe.io.deq.valid
  rDataQ.io.enq.bits.beats  := dsRespVec(dsRespID).map(_.bits)
  rDataQ.io.enq.bits.beatOH := dsRespOH
  // HardwareAssertion
  HardwareAssertion(Mux(dsRespIdPipe.io.deq.valid, PopCount(dsRespVec.map(_.map(_.valid).reduce(_ | _))) === 1.U, PopCount(dsRespVec.map(_.map(_.valid).reduce(_ | _))) === 0.U))
  HardwareAssertion.withEn(OHToUInt(dsRespVec.map(_.map(_.valid).reduce(_ | _))) === dsRespID, dsRespIdPipe.io.deq.valid)
  HardwareAssertion.withEn(Cat(dsRespVec(dsRespID).map(_.valid).reverse) === dsRespOH, dsRespIdPipe.io.deq.valid)

  /*
    * Send Resp To CHI RxDat
    */
  val sendBeat        = Mux(rDataQ.io.deq.bits.beatOH === "b10".U, 1.U, sendBeatNumReg)
  val sendIsLast      = Mux(rDataQ.io.deq.bits.beatOH === "b11".U, 1.U, 0.U)
  sendBeatNumReg      := Mux(rDataQ.io.deq.fire, 0.U, sendBeatNumReg + txDat.fire.asUInt)

  txDat.valid         := rRespQ.io.deq.valid & rDataQ.io.deq.valid
  txDat.bits          := Mux(txDat.valid, rRespQ.io.deq.bits, 0.U.asTypeOf(txDat.bits))
  txDat.bits.Data     := rDataQ.io.deq.bits.beats(sendBeat)
  txDat.bits.DataID   := toDataID(sendBeatNumReg)
  txDat.bits.BE       := Fill(rxDat.bits.BE.getWidth, 1.U(1.W))

  rRespQ.io.deq.ready := sendBeatNumReg === sendIsLast & txDat.fire
  rDataQ.io.deq.ready := sendBeatNumReg === sendIsLast & txDat.fire


// ------------------------------------------------------------ Assertion ----------------------------------------------- //

  HardwareAssertion.withEn(rxReq.bits.Addr(fullAddrBits - 1, fullAddrBits - sTagBits) === 0.U, rxReq.valid, cf"Illegal DCU address")

  HardwareAssertion.withEn(rDataQ.io.enq.ready, rDataQ.io.enq.valid, cf"rdata queue block [${rDataQ.io.enq.bits.beatOH}]", rDataQ.io.enq.bits.beatOH)

  HardwareAssertion.withEn(rxReq.bits.Opcode === ReadNoSnp | rxReq.bits.Opcode === WriteNoSnpFull | rxReq.bits.Opcode === WriteNoSnpPtl | rxReq.bits.Opcode === Replace |  rxReq.bits.Opcode === FlushDCU,
                           rxReq.valid, cf"Illegal DCU RxReq Opcode[${rxReq.bits.Opcode}]", rxReq.bits.Opcode)

  // Read Buffer Timeout Check
  rBufRegVec.zipWithIndex.foreach { { case (r, i) => HardwareAssertion.checkTimeout(r.state === DCURState.Free, TIMEOUT_DCU_R, cf"DCU ReadBuffer[${i}] STATE[${r.state}] TIMEOUT", r.state) } }
  HardwareAssertion.placePipe(2)
  // Write Buffer Timeout Check
  wBufRegVec.zipWithIndex.foreach { { case (w, i) => HardwareAssertion.checkTimeout(w.state === DCUWState.Free, TIMEOUT_DCU_W, cf"DCU ReadBuffer[${i}] STATE[${w.state}] TIMEOUT", w.state) } }
  HardwareAssertion.placePipe(2)
  /*
   * Hardware Assertion Node And IO
   */
  private val assertionNode = HardwareAssertion.placePipe(Int.MaxValue-1, true)
  @public
  val assertionOut = IO(assertionNode.assertion.cloneType)
  @public
  val assertionInfo = DomainInfo(assertionNode.desc)
  assertionOut <> assertionNode.assertion

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
