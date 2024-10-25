package zhujiang.device.bridge

import chisel3.util._
import chisel3._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import zhujiang.ZJModule
import zhujiang.chi._

abstract class BaseCtrlMachine[
  T <: IcnIoDevCtrlOpVecCommon,
  K <: IcnIoDevCtrlInfoCommon,
  S <: IcnIoDevRsEntryCommon[T, K]
](
  genOpVec: T,
  genInfo: K,
  genRsEntry: S,
  node: Node,
  outstanding: Int,
  ioDataBits: Int,
  slvBusDataBits: Int,
  compareTag: (UInt, UInt) => Bool
)(implicit p: Parameters) extends ZJModule {
  val icn = IO(new Bundle {
    val rx = new Bundle {
      val req = Flipped(Decoupled(new ReqFlit))
      val resp = if(node.nodeType == NodeType.HI) Some(Flipped(Decoupled(new RespFlit))) else None
      val data = Flipped(Decoupled(new DataFlit))
    }
    val tx = new Bundle {
      val resp = Decoupled(new RespFlit)
    }
  })
  val io = IO(new Bundle {
    val idx = Input(UInt(log2Ceil(outstanding).W))
    val readDataFire = Input(Bool())
    val readDataLast = Input(Bool())
    val info = Output(Valid(genInfo))
    val waitNum = Input(UInt(log2Ceil(outstanding).W))
    val wakeupIns = Input(Vec(outstanding - 1, Valid(UInt(raw.W))))
    val wakeupOut = Output(Valid(UInt(raw.W)))
  })

  val wakeupOutCond = Wire(Bool())
  val payload = Reg(genRsEntry)
  val payloadMiscNext = WireInit(payload)

  val allDone = payload.state.u.completed && payload.state.d.completed
  val valid = RegInit(false.B)
  val waiting = RegInit(0.U.asTypeOf(io.waitNum))
  private val payloadEnqNext = WireInit(payload)

  valid := Mux(valid, !allDone, icn.rx.req.fire)
  io.info.valid := valid
  io.info.bits := payload.info

  private val payloadUpdate = icn.rx.req.fire || valid
  when(payloadUpdate) {
    payload := Mux(valid, payloadMiscNext, payloadEnqNext)
  }

  icn.rx.req.ready := !valid
  icn.rx.resp.foreach(_.ready := true.B)
  icn.rx.data.ready := true.B

  private val wakeupVec = Cat(io.wakeupIns.map(wkp => wkp.valid && compareTag(wkp.bits, payload.info.addr) && valid))
  private val wakeupValid = wakeupVec.orR
  private val wakeupValidReg = RegNext(wakeupValid, false.B)
  private val wakeupNumReg = RegEnable(PopCount(wakeupVec), wakeupValid)
  when(icn.rx.req.fire) {
    waiting := io.waitNum
  }.elsewhen(wakeupValidReg) {
    assert(wakeupNumReg === 1.U)
    waiting := waiting - 1.U
  }

  io.wakeupOut.valid := wakeupOutCond
  io.wakeupOut.bits := payload.info.addr

  private val req = icn.rx.req.bits.asTypeOf(new ReqFlit)
  payloadEnqNext.enq(req, icn.rx.req.valid)

  private val plmnu = payloadMiscNext.state.u
  private val plu = payload.state.u

  when(io.readDataFire) {
    plmnu.rdata := io.readDataLast || plu.rdata
    payloadMiscNext.info.readCnt := payload.info.readCnt + 1.U
  }

  if(icn.rx.resp.isDefined) {
    when(icn.rx.resp.get.valid) {
      plmnu.compAck := icn.rx.resp.get.bits.Opcode === RspOpcode.CompAck || plu.compAck
    }
    when(icn.rx.resp.get.valid || icn.rx.data.valid) {
      assert(Cat(icn.rx.data.valid =/= icn.rx.resp.get.valid) =/= "b11".U)
    }
  }

  private val icnDatOp = icn.rx.data.bits.Opcode
  when(icn.rx.data.valid) {
    if(payload.info.data.isDefined) {
      val icnDataVec = icn.rx.data.bits.Data.asTypeOf(Vec(dw / ioDataBits, UInt(ioDataBits.W)))
      val icnDataMaskVec = icn.rx.data.bits.BE.asTypeOf(Vec(bew / (ioDataBits / 8), UInt((ioDataBits / 8).W)))
      val dataIdx = payload.info.addr(log2Ceil(dw / 8) - 1, log2Ceil(ioDataBits / 8))
      payloadMiscNext.info.data.get := icnDataVec(dataIdx)
      payloadMiscNext.info.mask.get := icnDataMaskVec(dataIdx)
    }
    plmnu.wdata := icnDatOp === DatOpcode.NCBWrDataCompAck || icnDatOp === DatOpcode.NonCopyBackWriteData || plu.wdata
    plmnu.compAck := icnDatOp === DatOpcode.NCBWrDataCompAck || plu.compAck
  }

  private val dwt = payload.info.dwt.getOrElse(false.B)
  private val icnReadReceipt = payload.state.icnReadReceipt
  private val icnDBID = payload.state.icnDBID
  private val icnComp = Mux(dwt, payload.state.icnComp && payload.state.u.wdata, payload.state.icnComp)
  private val icnCompDBID = icnDBID && icnComp
  icn.tx.resp.valid := valid & (icnReadReceipt || icnDBID || icnComp)
  icn.tx.resp.bits := DontCare
  icn.tx.resp.bits.Opcode := Mux(icnCompDBID, RspOpcode.CompDBIDResp, MuxCase(0.U, Seq(
    icnReadReceipt -> RspOpcode.ReadReceipt,
    icnDBID -> RspOpcode.DBIDResp,
    icnComp -> RspOpcode.Comp
  )))
  icn.tx.resp.bits.DBID := io.idx
  icn.tx.resp.bits.TxnID := Mux(icnDBID && dwt, payload.info.returnTxnId.getOrElse(0.U), payload.info.txnId)
  icn.tx.resp.bits.SrcID := 0.U
  icn.tx.resp.bits.TgtID := Mux(icnDBID && dwt, payload.info.returnNid.getOrElse(0.U), payload.info.srcId)
  icn.tx.resp.bits.Resp := Mux(icnDBID || icnComp, "b000".U, "b010".U)
  when(icn.tx.resp.fire) {
    plmnu.receiptResp := icn.tx.resp.bits.Opcode === RspOpcode.ReadReceipt || plu.receiptResp
    plmnu.dbidResp := icn.tx.resp.bits.Opcode === RspOpcode.DBIDResp || icn.tx.resp.bits.Opcode === RspOpcode.CompDBIDResp || plu.dbidResp
    plmnu.comp := icn.tx.resp.bits.Opcode === RspOpcode.Comp || icn.tx.resp.bits.Opcode === RspOpcode.CompDBIDResp || plu.comp
  }

  private val busDataBytes = slvBusDataBits / 8
  private val bufDataBytes = ioDataBits / 8
  private val segNum = busDataBytes / bufDataBytes
  private val segIdx = if(segNum > 1) payload.info.addr(log2Ceil(busDataBytes) - 1, log2Ceil(bufDataBytes)) else 0.U
  private val maskVec = Wire(Vec(segNum, UInt(bufDataBytes.W)))
  maskVec.zipWithIndex.foreach({ case (a, b) => a := Mux(b.U === segIdx, payload.info.mask.getOrElse(0.U), 0.U) })
  val slvMask = maskVec.asUInt
  val slvData = Fill(segNum, payload.info.data.getOrElse(0.U(ioDataBits.W)))
}
