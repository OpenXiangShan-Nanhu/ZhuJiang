package zhujiang.device.bridge.chi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import zhujiang.ZJParametersKey
import zhujiang.chi._
import zhujiang.device.bridge.BaseCtrlMachine

class ChiSnBridgeCtrlMachine(
  icnNode: Node,
  outstanding: Int,
  ioDataBits: Int,
  compareTag: (UInt, UInt) => Bool
)(implicit p: Parameters) extends BaseCtrlMachine(
  genOpVec = new ChiSnBridgeCtrlOpVec,
  genInfo = new ChiSnBridgeCtrlInfo(ioDataBits),
  genRsEntry = new ChiSnBridgeRsEntry(ioDataBits),
  node = icnNode,
  outstanding = outstanding,
  ioDataBits = ioDataBits,
  slvBusDataBits = p(ZJParametersKey).dataBits,
  compareTag = compareTag
) {
  val sn = IO(new Bundle {
    val tx = new Bundle {
      val req = Decoupled(new ReqFlit)
      val data = Decoupled(new DataFlit)
    }
    val rx = new Bundle {
      val resp = Flipped(Decoupled(new RespFlit))
    }
  })
  wakeupOutCond := (payload.state.d.dbidResp && payload.state.d.receiptResp && payload.info.order.orR || allDone) && valid && payload.info.isSnooped
  when(io.wakeupOut.valid) {
    payloadMiscNext.info.isSnooped := false.B
  }

  when(icn.rx.resp.get.valid || icn.rx.data.valid || io.readDataFire || sn.rx.resp.valid) {
    assert(valid)
  }

  sn.rx.resp.ready := true.B

  private val plmnd = payloadMiscNext.state.d
  private val pld = payload.state.d
  private val snRspOp = sn.rx.resp.bits.asTypeOf(new RespFlit).Opcode
  when(sn.rx.resp.fire) {
    plmnd.receiptResp := snRspOp === RspOpcode.ReadReceipt || pld.receiptResp
    plmnd.dbidResp := snRspOp === RspOpcode.CompDBIDResp || snRspOp === RspOpcode.DBIDResp || pld.dbidResp
    plmnd.comp := snRspOp === RspOpcode.CompDBIDResp || snRspOp === RspOpcode.Comp || pld.comp
    when(snRspOp === RspOpcode.CompDBIDResp || snRspOp === RspOpcode.DBIDResp) {
      payloadMiscNext.info.dbid := sn.rx.resp.bits.asTypeOf(new RespFlit).DBID
    }
  }

  when(io.readDataFire) {
    plmnd.rdata := io.readDataLast || pld.rdata
    plmnd.comp := io.readDataLast || pld.comp
  }

  when(io.readDataFire || sn.rx.resp.valid) {
    assert(io.readDataFire =/= sn.rx.resp.valid)
  }

  private val snWriteNoSnpPtl = payload.state.snWriteNoSnpPtl
  private val snReadNoSnp = payload.state.snReadNoSnp
  private val snNCBWrDataCompAck = payload.state.snNCBWrDataCompAck
  private val snReqB = Wire(new ReqFlit)
  private val snDatB = Wire(new DataFlit)

  sn.tx.req.valid := valid && (snWriteNoSnpPtl || snReadNoSnp) && !waiting.orR
  snReqB := DontCare
  snReqB.Opcode := MuxCase(0.U, Seq(
    snWriteNoSnpPtl -> ReqOpcode.WriteNoSnpPtl,
    snReadNoSnp -> ReqOpcode.ReadNoSnp
  ))
  snReqB.TxnID := io.idx
  snReqB.Addr := payload.info.addr
  snReqB.Size := payload.info.size
  snReqB.Order := payload.info.order
  snReqB.ExpCompAck := false.B
  snReqB.ReturnTxnID := io.idx
  snReqB.DoDWT := false.B
  sn.tx.req.bits := snReqB

  sn.tx.data.valid := valid && snNCBWrDataCompAck && !waiting.orR
  private val segNum = dw / ioDataBits
  private val segOff = payload.info.addr(log2Ceil(dw / 8) - 1, log2Ceil(ioDataBits / 8))
  private val maskVec = Wire(Vec(segNum, UInt((bew / segNum).W)))
  for(idx <- maskVec.indices) maskVec(idx) := Mux(segOff === idx.U, payload.info.mask.get, 0.U)

  snDatB := DontCare
  snDatB.Opcode := DatOpcode.NCBWrDataCompAck
  snDatB.Data := slvData
  snDatB.BE := maskVec.asUInt
  snDatB.TxnID := payload.info.dbid
  sn.tx.data.bits := snDatB

  when(sn.tx.req.fire) {
    plmnd.wreq := snReqB.Opcode === ReqOpcode.WriteNoSnpPtl || pld.wreq
    plmnd.rreq := snReqB.Opcode === ReqOpcode.ReadNoSnp || pld.rreq
  }
  when(sn.tx.data.fire) {
    plmnd.wdata := snDatB.Opcode === DatOpcode.NCBWrDataCompAck || pld.wdata
  }
}
