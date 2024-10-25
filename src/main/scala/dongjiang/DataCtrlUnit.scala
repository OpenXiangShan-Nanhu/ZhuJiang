package dongjiang.dcu

import zhujiang.chi._
import xijiang.Node
import dongjiang._
import dongjiang.chi._
import dongjiang.chi.CHIOp.REQ._
import dongjiang.chi.CHIOp.RSP._
import dongjiang.chi.CHIOp.DAT._
import chisel3._
import chisel3.experimental.hierarchy.{instantiable, public}
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.perf.{DebugOptions, DebugOptionsKey}
import xijiang.router.base.IcnBundle
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
  val Reading           = "b10".U
}


class DCUREntry(implicit p: Parameters) extends DJBundle {
  val state             = UInt(DCURState.width.W)
  val dsIndex           = UInt(dsIndexBits.W)
  val dsBank            = UInt(dsBankBits.W)
  val srcID             = UInt(chiNodeIdBits.W)
  val txnID             = UInt(djparam.chiTxnidBits.W)
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
  val WriteSram         = "b011".U // Send Write Req To DataStorage And Send Comp To Src
  val Writting          = "b100".U // Write Data To DataStorage
  val SendComp          = "b101".U
}

class DCUWEntry(implicit p: Parameters) extends DJBundle {
  val state             = UInt(DCUWState.width.W)
  val replRState        = UInt(DCURState.width.W)
  val data              = Vec(nrBeat, Valid(UInt(beatBits.W)))
  val dsIndex           = UInt(dsIndexBits.W)
  val dsBank            = UInt(dsBankBits.W)
  val srcID             = UInt(chiNodeIdBits.W)
  val txnID             = UInt(djparam.chiTxnidBits.W)
  val ddrDBID           = UInt(djparam.chiDBIDBits.W)

  def canWrite          = replRState === DCURState.Free
  def isLast            = PopCount(data.map(_.valid)) === (nrBeat - 1).U
}



// TODO: DMT
// TODO: DWT
/*
 * DCUs do not have sorting capabilities and must use the DWT transfer structure to sort by using Comp
 */
@instantiable
class DataCtrlUnit(node: Node, nrIntf: Int = 1, hasReset: Boolean = true)(implicit p: Parameters) extends DJRawModule
  with ImplicitClock with ImplicitReset {
  // ------------------------------------------ IO declaration --------------------------------------------- //
  @public val io = IO(new Bundle {
    val sn = Vec(nrIntf, Flipped(new IcnBundle(node, hasReset)))
  })
  @public val reset = IO(Input(AsyncReset()))
  @public val clock = IO(Input(Clock()))
  val implicitClock = clock
  val implicitReset = reset

  if(p(DebugOptionsKey).EnableDebug) {
    dontTouch(io)
  }

  require(node.splitFlit)
  require(1 <= nrIntf & nrIntf <= 2)

  io <> DontCare

// ----------------------------------------- Reg and Wire declaration ------------------------------------ //
  // CHI
  val txReq                 = Wire(new DecoupledIO(new ReqFlit))
  val txDat                 = Wire(new DecoupledIO(new DataFlit))
  val rxRsp                 = WireInit(0.U.asTypeOf(Decoupled(new RespFlit)))
  val rxDat                 = WireInit(0.U.asTypeOf(Decoupled(new DataFlit)))
  // TODO
  io.sn(0).tx.req.get       <> txReq
  io.sn(0).tx.data.get      <> txDat
  io.sn(0).rx.resp.get      <> rxRsp
  io.sn(0).rx.data.get      <> rxDat


  // ReqBuf
  val wBufRegVec            = RegInit(VecInit(Seq.fill(djparam.nrDCUWBuf) { 0.U.asTypeOf(new DCUWEntry) }))
  val rBufRegVec            = RegInit(VecInit(Seq.fill(djparam.nrDCURBuf) { 0.U.asTypeOf(new DCUREntry) }))

  // sram read vec
  val sramRReadyVec         = Wire(Vec(djparam.nrDSBank, Bool()))
  val sramWReadyVec         = Wire(Vec(djparam.nrDSBank, Bool()))

  // DataStorage
  val ds                    = Seq.fill(djparam.nrDSBank) { Module(new DataStorage(sets = nrDSEntry)) }
  ds.zipWithIndex.foreach { case(d, i) => d.io.id := i.U }

  // ChiRespQueue
  val rRespReg              = RegInit(0.U.asTypeOf(Valid(new DataFlit)))
  val rRespCanGo            = Wire(Bool())
  val rRespQ                = Module(new Queue(new DataFlit(), entries = djparam.nrDCURespQ - 1, flow = false, pipe = true))
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
  val reqIsW                = isWriteX(txReq.bits.Opcode)
  val reqIsRepl             = isReplace(txReq.bits.Opcode)
  when(reqIsW | reqIsRepl) {
    txReq.ready             := wBufFreeVec.reduce(_ | _)
  }.otherwise {
    txReq.ready             := rBufFreeVec.reduce(_ | _)
  }


  /*
   * Send DBID Or Comp To Src
   */
  val wBufSCompVec          = wBufRegVec.map { case w => w.state === DCUWState.Writting | w.state === DCUWState.SendComp}
  val wBufSDBIDVec          = wBufRegVec.map(_.state === DCUWState.SendDBIDResp)
  val selSCompID            = PriorityEncoder(wBufSCompVec)
  val selSDBID              = PriorityEncoder(wBufSDBIDVec)

  rxRsp.valid               := wBufSDBIDVec.reduce(_ | _) | wBufSCompVec.reduce(_ | _)
  rxRsp.bits.Opcode         := Mux(wBufSCompVec.reduce(_ | _), Comp,                         DBIDResp)
  rxRsp.bits.DBID           := Mux(wBufSCompVec.reduce(_ | _), selSCompID,                   selSDBID)
  rxRsp.bits.TgtID          := Mux(wBufSCompVec.reduce(_ | _), wBufRegVec(selSCompID).srcID, wBufRegVec(selSDBID).srcID)
  rxRsp.bits.TxnID          := Mux(wBufSCompVec.reduce(_ | _), wBufRegVec(selSCompID).txnID, wBufRegVec(selSDBID).txnID)


  /*
      * Select Buf Send Early Req To SRAM
      */
  sramRReadyVec             := ds.map(_.io.earlyRReq.ready)
  sramWReadyVec             := ds.map(_.io.earlyWReq.ready)

  val willSendRVec          = rBufRegVec.map { case r => r.state === DCURState.ReadSram      & sramRReadyVec(r.dsBank) & (!rRespReg.valid | rRespCanGo) }
  val willSendReplVec       = wBufRegVec.map { case r => r.replRState === DCURState.ReadSram & sramRReadyVec(r.dsBank) & (!rRespReg.valid | rRespCanGo) & !willSendRVec.reduce(_ | _) }
  val willSendWVec          = wBufRegVec.map { case w => w.state === DCUWState.WriteSram     & w.canWrite & sramWReadyVec(w.dsBank) }

  val earlyWID              = PriorityEncoder(willSendWVec)
  val earlyRID              = PriorityEncoder(willSendRVec)
  val earlyReplID           = PriorityEncoder(willSendReplVec)
  val earlyRepl             = !willSendRVec.reduce(_ | _) & willSendReplVec.reduce(_ | _)

  ds.zipWithIndex.foreach {
    case(d, i) =>
      d.io.earlyRReq.valid := (willSendRVec.reduce(_ | _) | willSendReplVec.reduce(_ | _)) & rBufRegVec(earlyRID).dsBank === i.U
      d.io.earlyWReq.valid := willSendWVec.reduce(_ | _)                                   & wBufRegVec(earlyWID).dsBank === i.U
  }


  /*
    * Get Read CHI Resp
    */
  val earlyRValid           = ds.map(_.io.earlyRReq.fire).reduce(_ | _)
  rRespReg.valid            := Mux(earlyRValid, true.B, rRespReg.valid & !rRespCanGo)
  rRespCanGo                := rRespQ.io.enq.ready
  when(earlyRValid & rRespCanGo) {
    rRespReg.bits.Opcode    := Mux(earlyRepl, CHIOp.DAT.NonCopyBackWrData,      CHIOp.DAT.CompData)
    rRespReg.bits.TgtID     := Mux(earlyRepl, ddrcNodeId.U,                     rBufRegVec(earlyRID).srcID)
    rRespReg.bits.TxnID     := Mux(earlyRepl, wBufRegVec(earlyReplID).ddrDBID,  rBufRegVec(earlyRID).txnID)
    rRespReg.bits.Resp      := Mux(earlyRepl, 0.U,                              rBufRegVec(earlyRID).resp)
  }


  /*
    * Read Req Buf
    */
  rBufRegVec.zipWithIndex.foreach {
    case (r, i) =>
      switch(r.state) {
        is(DCURState.Free) {
          val hit       = txReq.fire & !reqIsW & !reqIsRepl & selRecRID === i.U
          when(hit) {
            r.state     := DCURState.ReadSram
            r.dsIndex   := parseDCUAddress(txReq.bits.Addr)._1
            r.dsBank    := parseDCUAddress(txReq.bits.Addr)._2
            r.srcID     := txReq.bits.SrcID
            r.txnID     := txReq.bits.TxnID
            r.resp      := txReq.bits.MemAttr(ChiResp.width - 1, 0)
          }
        }
        is(DCURState.ReadSram) {
          val hit       = ds.map(_.io.earlyRReq.fire).reduce(_ | _) & earlyRID === i.U & !earlyRepl
          when(hit) {
              r.state   := DCURState.Reading
          }
        }
        is(DCURState.Reading) {
          r             := 0.U.asTypeOf(r)
          r.state       := DCURState.Free
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
          val hit       = txReq.fire & (reqIsW | reqIsRepl) & selRecWID === i.U
          when(hit) {
            w.state     := DCUWState.SendDBIDResp
            w.dsIndex   := parseDCUAddress(txReq.bits.Addr)._1
            w.dsBank    := parseDCUAddress(txReq.bits.Addr)._2
            w.srcID     := txReq.bits.SrcID
            w.txnID     := txReq.bits.TxnID
          }
        }
        is(DCUWState.SendDBIDResp) {
          val hit       = rxRsp.fire & rxRsp.bits.Opcode === DBIDResp & selSDBID === i.U
          when(hit) {
            w.state     := DCUWState.WaitData
          }
        }
        is(DCUWState.WaitData) {
          val hit       = txDat.fire & txDat.bits.TxnID === i.U
          when(hit) {
            w.state     := Mux(hit & w.isLast, DCUWState.WriteSram, w.state)
            w.data(toBeatNum(txDat.bits.DataID)).valid  := true.B
            w.data(toBeatNum(txDat.bits.DataID)).bits   := txDat.bits.Data
          }
        }
        is(DCUWState.WriteSram) {
          val hit       = ds.map(_.io.earlyWReq.fire).reduce(_ | _) & earlyWID === i.U
          when(hit) {
            w.state     := DCUWState.Writting
          }
        }
        is(DCUWState.Writting) {
          val hit       = rxRsp.fire & rxRsp.bits.Opcode === Comp & selSCompID === i.U
          when(hit) {
            w           := 0.U.asTypeOf(w)
            w.state     := DCUWState.Free
          }.otherwise {
            w.state     := DCUWState.SendComp
          }
        }
        is(DCUWState.SendComp) {
          val hit       = rxRsp.fire & rxRsp.bits.Opcode === Comp & selSCompID === i.U
          when(hit) {
            w           := 0.U.asTypeOf(w)
            w.state     := DCUWState.Free
          }
        }
      }
  }
  txDat.ready := true.B


  /*
    * Write Req Buf Replace Read State
    */
  wBufRegVec.zipWithIndex.foreach {
    case (r, i) =>
      switch(r.replRState) {
        is(DCURState.Free) {
          val hit         = txReq.fire & reqIsRepl & selRecWID === i.U
          when(hit) {
            r.replRState  := DCURState.ReadSram
            r.ddrDBID     := txReq.bits.ReturnTxnID
          }
        }
        is(DCURState.ReadSram) {
          val hit         = ds.map(_.io.earlyRReq.fire).reduce(_ | _) & earlyReplID === i.U & earlyRepl
          when(hit) {
            r.replRState  := DCURState.Reading
          }
        }
        is(DCURState.Reading) {
          r.ddrDBID       := 0.U
          r.replRState    := DCURState.Free
        }
      }
  }



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------- S1: Send Req To DataStorage ---------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
    * Send Write / Read Req
    */
  val sendWReqID = PriorityEncoder(wBufRegVec.map(_.state === DCUWState.Writting))
  val sendRReqID = PriorityEncoder(rBufRegVec.map(_.state === DCURState.Reading))
  ds.foreach(_.io.read  := DontCare)
  ds.foreach(_.io.write := DontCare)
  ds.zipWithIndex.foreach {
    case (d, i) =>
      val wHit = wBufRegVec(sendWReqID).state === DCUWState.Writting & wBufRegVec(sendWReqID).dsBank === i.U
      val rHit = rBufRegVec(sendRReqID).state === DCURState.Reading & rBufRegVec(sendRReqID).dsBank === i.U
      when(wHit) {
        d.io.write.index := wBufRegVec(sendWReqID).dsIndex
        d.io.write.data  := Cat(wBufRegVec(sendWReqID).data.map(_.bits).reverse)
        assert(wBufRegVec(sendWReqID).data.map(_.valid).reduce(_ & _))
      }.elsewhen(rHit) {
        d.io.read        := rBufRegVec(sendRReqID).dsIndex
      }
      assert(!(wHit & rHit))
  }
  assert(PopCount(rBufRegVec.map(_.state === DCURState.Reading))  <= 1.U)
  assert(PopCount(wBufRegVec.map(_.state === DCUWState.Writting)) <= 1.U)


  /*
    * Get Read CHI Resp
    */
  rRespQ.io.enq.valid := rRespReg.valid
  rRespQ.io.enq.bits  := rRespReg.bits


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------- S2: Receive SRAM Resp and Send RxDat ------------------------------------------- //
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
  sendBeatNumReg      := sendBeatNumReg + rxDat.fire.asUInt
  rxDat.valid         := rRespQ.io.deq.valid & rDatQ.io.deq.valid
  rxDat.bits          := rRespQ.io.deq.bits
  rxDat.bits.Data     := rDatQ.io.deq.bits(sendBeatNumReg)
  rxDat.bits.DataID   := toDataID(sendBeatNumReg)

  rRespQ.io.deq.ready := sendBeatNumReg === (nrBeat - 1).U & rxDat.fire
  rDatQ.io.deq.ready  := sendBeatNumReg === (nrBeat - 1).U & rxDat.fire



// ------------------------------------------------------------ Assertion ----------------------------------------------- //

  assert(Mux(txReq.valid, txReq.bits.Addr(addressBits - 1, addressBits - sTagBits) === 0.U, true.B))

  assert(Mux(txReq.valid, txReq.bits.Size === log2Ceil(djparam.blockBytes).U, true.B))

  assert(Mux(rDatQ.io.enq.valid, rDatQ.io.enq.ready, true.B))

  assert(Mux(txReq.valid, txReq.bits.Opcode === ReadNoSnp | txReq.bits.Opcode === WriteNoSnpFull | txReq.bits.Opcode === Replace, true.B))

}
