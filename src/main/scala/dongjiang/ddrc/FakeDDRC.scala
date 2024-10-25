package dongjiang.ddrc

import zhujiang.chi._
import xijiang.Node
import dongjiang._
import dongjiang.chi._
import dongjiang.chi.CHIOp.REQ._
import dongjiang.chi.CHIOp.RSP._
import dongjiang.chi.CHIOp.DAT._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.perf.{DebugOptions, DebugOptionsKey}
import xijiang.router.base.IcnBundle
import xs.utils.sram._
import dongjiang.utils.FastArb._


object DDRCWState {
    val width = 3
    // commom
    val Free            = "b000".U
    val SendDBIDResp    = "b001".U
    val WaitData        = "b010".U
    val WriteDDR        = "b011".U // Send Write Req To DataStorage And Send Comp To Src
    val SendComp        = "b101".U
}

class DDRCWEntry(implicit p: Parameters) extends DJBundle {
    val state           = UInt(DDRCWState.width.W)
    val data            = Vec(nrBeat, Valid(UInt(beatBits.W)))
    val index           = UInt(64.W)
    val srcID           = UInt(chiNodeIdBits.W)
    val txnID           = UInt(djparam.chiTxnidBits.W)

    def isLast          = PopCount(data.map(_.valid)) === (nrBeat - 1).U
}

object DDRCRState {
    val width           = 2
    val Free            = "b00".U
    val ReadDDR         = "b01".U
    val GetResp         = "b10".U
}


class DDRCREntry(implicit p: Parameters) extends DJBundle {
    val state           = UInt(DDRCRState.width.W)
    val index           = UInt(64.W)
    val srcID           = UInt(chiNodeIdBits.W)
    val txnID           = UInt(djparam.chiTxnidBits.W)
}


// TODO: DMT
// TODO: DWT
class FakeDDRC(node: Node)(implicit p: Parameters) extends DJModule {
// ------------------------------------------ IO declaration --------------------------------------------- //
    val io = IO(new Bundle {
        val sn = Flipped(new IcnBundle(node))
    })

    require(node.splitFlit)

    io <> DontCare

// ----------------------------------------- Reg and Wire declaration ------------------------------------ //
    // CHI
    val txReq               = Wire(new DecoupledIO(new ReqFlit))
    val txDat               = Wire(new DecoupledIO(new DataFlit))
    val rxRsp               = WireInit(0.U.asTypeOf(Decoupled(new RespFlit)))
    val rxDat               = WireInit(0.U.asTypeOf(Decoupled(new DataFlit)))
    // TODO
    io.sn.tx.req.get        <> txReq
    io.sn.tx.data.get       <> txDat
    io.sn.rx.resp.get       <> rxRsp
    io.sn.rx.data.get       <> rxDat


    // ReqBuf
    val wBufRegVec          = RegInit(VecInit(Seq.fill(nrDDRCWBuf) { 0.U.asTypeOf(new DDRCWEntry) }))
    val rBufRegVec          = RegInit(VecInit(Seq.fill(nrDDRCRBuf) { 0.U.asTypeOf(new DDRCREntry) }))

    // DataStorage
    val ddr                 = Seq.fill(nrDDRCBank) { Module(new MemHelper()) }

    // ChiRespQueue
    val rRespQ              = Module(new Queue(new DataFlit(), entries = nrDDRCRespQ, flow = false, pipe = true))
    val rDatQ               = Module(new Queue(Vec(nrBeat, UInt(beatBits.W)), entries = nrDDRCRespQ, flow = false, pipe = true))
    val sendBeatNumReg      = RegInit(0.U(beatNumBits.W))


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------ S0: Receive Req From CHI -------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
    /*
     * Receive Req
     */
    val wBufFreeVec         = wBufRegVec.map(_.state === DDRCWState.Free)
    val rBufFreeVec         = rBufRegVec.map(_.state === DDRCRState.Free)
    val selRecWID           = PriorityEncoder(wBufFreeVec)
    val selRecRID           = PriorityEncoder(rBufFreeVec)
    val reqIsW              = isWriteX(txReq.bits.Opcode)
    when(reqIsW) {
        txReq.ready         := wBufFreeVec.reduce(_ | _)
    }.otherwise {
        txReq.ready         := rBufFreeVec.reduce(_ | _)
    }


    /*
     * Send DBID Or Comp To Src
     */
    val wBufSCompVec        = wBufRegVec.map(_.state === DDRCWState.SendComp)
    val wBufSDBIDVec        = wBufRegVec.map(_.state === DDRCWState.SendDBIDResp)
    val selSCompID          = PriorityEncoder(wBufSCompVec)
    val selSDBID            = PriorityEncoder(wBufSDBIDVec)

    rxRsp.valid             := wBufSDBIDVec.reduce(_ | _) | wBufSCompVec.reduce(_ | _)
    rxRsp.bits.Opcode       := Mux(wBufSCompVec.reduce(_ | _), Comp,                         DBIDResp)
    rxRsp.bits.DBID         := Mux(wBufSCompVec.reduce(_ | _), selSCompID,                   selSDBID)
    rxRsp.bits.TgtID        := Mux(wBufSCompVec.reduce(_ | _), wBufRegVec(selSCompID).srcID, wBufRegVec(selSDBID).srcID)
    rxRsp.bits.TxnID        := Mux(wBufSCompVec.reduce(_ | _), wBufRegVec(selSCompID).txnID, wBufRegVec(selSDBID).txnID)


    /*
     * Select Buf Send Req To DDR
     */
    val willSendRVec         = rBufRegVec.map { case r => r.state === DDRCRState.ReadDDR  & rRespQ.io.count <= (nrDDRCRespQ - 2).U }; require(nrDDRCRespQ >= 2)
    val willSendWVec         = wBufRegVec.map { case w => w.state === DDRCWState.WriteDDR }
    val rDDRID               = PriorityEncoder(willSendRVec)
    val wDDRID               = PriorityEncoder(willSendWVec)


    ddr.foreach { case d => d.clk := clock }
    ddr.foreach {
        case d =>
            d.rIdx  := rBufRegVec(rDDRID).index
            d.ren   := willSendRVec.reduce(_ | _) & !willSendWVec.reduce(_ | _)
    }
    ddr.zipWithIndex.foreach {
        case (d, i) =>
            d.wIdx  := wBufRegVec(wDDRID).index
            d.wen   := willSendWVec.reduce(_ | _)
            d.wdata := Cat(wBufRegVec(wDDRID).data.map(_.bits).reverse).asTypeOf(Vec(nrDDRCBank, UInt(64.W)))(i)
    }

    /*
     * Read Req Buf
     */
    rBufRegVec.zipWithIndex.foreach {
        case (r, i) =>
            switch(r.state) {
                is(DDRCRState.Free) {
                    val hit         = txReq.fire & !reqIsW & selRecRID === i.U;
                    assert(txReq.bits.Opcode === ReadNoSnp | !hit)
                    when(hit) {
                        r.state     := DDRCRState.ReadDDR
                        r.index     := txReq.bits.Addr(addressBits - 1, offsetBits)
                        r.srcID     := txReq.bits.SrcID
                        r.txnID     := txReq.bits.TxnID
                    }
                }
                is(DDRCRState.ReadDDR) {
                    val hit         = ddr.map(_.ren).reduce(_ | _) & rDDRID === i.U
                    when(hit) {
                        r.state     := DDRCRState.GetResp
                    }
                }
                is(DDRCRState.GetResp) {
                    r               := 0.U.asTypeOf(r)
                    r.state         := DDRCRState.Free
                }
            }
    }


    /*
     * Write Req Buf
     */
    wBufRegVec.zipWithIndex.foreach {
        case(w, i) =>
            switch(w.state) {
                is(DDRCWState.Free) {
                    val hit = txReq.fire & reqIsW & selRecWID === i.U; assert(txReq.bits.Opcode === WriteNoSnpFull | !hit)
                    when(hit) {
                        w.state     := DDRCWState.SendDBIDResp
                        w.index     := txReq.bits.Addr(addressBits - 1, offsetBits)
                        w.srcID     := txReq.bits.SrcID
                        w.txnID     := txReq.bits.TxnID
                    }
                }
                is(DDRCWState.SendDBIDResp) {
                    val hit         = rxRsp.fire & rxRsp.bits.Opcode === DBIDResp & selSDBID === i.U
                    when(hit) {
                        w.state     := DDRCWState.WaitData
                    }
                }
                is(DDRCWState.WaitData) {
                    val hit         = txDat.fire & txDat.bits.TxnID === i.U
                    when(hit) {
                        w.state     := Mux(hit & w.isLast, DDRCWState.WriteDDR, w.state)
                        w.data(toBeatNum(txDat.bits.DataID)).valid  := true.B
                        w.data(toBeatNum(txDat.bits.DataID)).bits   := txDat.bits.Data
                    }
                }
                is(DDRCWState.WriteDDR) {
                    val hit         = ddr.map(_.wen).reduce(_ | _) & wDDRID === i.U
                    when(hit) {
                        w.state     := DDRCWState.SendComp
                    }
                }
                is(DDRCWState.SendComp) {
                    val hit         = rxRsp.fire & rxRsp.bits.Opcode === Comp & selSCompID === i.U
                    when(hit) {
                        w           := 0.U.asTypeOf(w)
                        w.state     := DDRCWState.Free
                    }
                }
            }
    }
    txDat.ready := true.B



// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- S1: Get Resp From DDR ------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
    /*
     * Get Read CHI Resp
     */
    val getRespVec              = rBufRegVec.map(_.state === DDRCRState.GetResp)
    val getRespID               = PriorityEncoder(getRespVec)
    rRespQ.io.enq.valid         := getRespVec.reduce(_ | _)
    rRespQ.io.enq.bits          := DontCare
    rRespQ.io.enq.bits.Opcode   := CHIOp.DAT.CompData
    rRespQ.io.enq.bits.Resp     := ChiResp.UC
    rRespQ.io.enq.bits.TgtID    := rBufRegVec(getRespID).srcID
    rRespQ.io.enq.bits.TxnID    := rBufRegVec(getRespID).txnID
    assert(PopCount(rBufRegVec.map(_.state === DDRCRState.GetResp))  <= 1.U)


    /*
     * Receive SRAM Resp
     */
    rDatQ.io.enq.valid          := getRespVec.reduce(_ | _)
    rDatQ.io.enq.bits.zipWithIndex.foreach { case (beat, i) =>
        beat := Cat(ddr.map(_.rdata))(beatBits * (i + 1) - 1, beatBits * i)
    }


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------ S2: Send RxDat ------------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
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
    assert(Mux(txReq.valid, txReq.bits.Size === log2Ceil(djparam.blockBytes).U, true.B))

    assert(Mux(rRespQ.io.enq.valid, rDatQ.io.enq.ready, true.B))

    assert(Mux(rDatQ.io.enq.valid, rDatQ.io.enq.ready, true.B))

    assert(!(rRespQ.io.enq.valid ^ rDatQ.io.enq.valid))

    assert(Mux(txReq.valid, txReq.bits.Opcode === ReadNoSnp | txReq.bits.Opcode === WriteNoSnpFull, true.B))
}
