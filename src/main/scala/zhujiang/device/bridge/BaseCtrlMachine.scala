package zhujiang.device.bridge

import chisel3.util._
import chisel3._
import freechips.rocketchip.util.MaskGen
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xs.utils.perf.DebugOptionsKey
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
      val req = Flipped(Decoupled(new ReqFlit(node.nodeType == NodeType.S)))
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

  val payload = Reg(genRsEntry)
  val payloadMiscNext = WireInit(payload)

  val allDone = payload.state.u.completed && payload.state.d.completed
  val valid = RegInit(false.B)
  val waiting = Reg(UInt(log2Ceil(outstanding).W))
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
  private val wakeupValidReg = RegNext(wakeupValid)
  private val wakeupNumReg = RegEnable(PopCount(wakeupVec), wakeupValid)
  private val waitNumSetEn = RegNext(icn.rx.req.fire)
  when(icn.rx.req.fire) {
    waiting := Fill(log2Ceil(outstanding), true.B) // to optimize waiting num calculation timing
  }.elsewhen(waitNumSetEn){
    waiting := io.waitNum // set waiting num at next cycle of enqueue
  }.elsewhen(wakeupValidReg) {
    assert(wakeupNumReg === 1.U)
    waiting := waiting - 1.U
  }

  io.wakeupOut.valid := payload.state.wakeup && valid && payload.info.isSnooped
  io.wakeupOut.bits := payload.info.addr
  when(io.wakeupOut.valid) {
    payloadMiscNext.info.isSnooped := false.B
  }

  private val req = icn.rx.req.bits.asTypeOf(new ReqFlit(node.nodeType == NodeType.S))
  payloadEnqNext.enq(req, icn.rx.req.valid)

  private val plmnu = payloadMiscNext.state.u
  private val plu = payload.state.u

  when(io.readDataFire) {
    plmnu.rdata := io.readDataLast || plu.rdata
    payloadMiscNext.info.readCnt := payload.info.readCnt + 1.U
  }

  if(icn.rx.resp.isDefined && plmnu.compAck.isDefined) {
    when(icn.rx.resp.get.valid) {
      plmnu.compAck.get := icn.rx.resp.get.bits.Opcode === RspOpcode.CompAck || plu.compAck.get
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
    plmnu.wdata := icnDatOp === DatOpcode.NCBWrDataCompAck || icnDatOp === DatOpcode.NonCopyBackWriteData || icnDatOp === DatOpcode.WriteDataCancel || plu.wdata
    if(plmnu.compAck.isDefined) {
      plmnu.compAck.get := icnDatOp === DatOpcode.NCBWrDataCompAck || icnDatOp === DatOpcode.WriteDataCancel || plu.compAck.get
    }
  }

  private val dwt = payload.info.dwt.getOrElse(false.B)
  private val ewa = payload.info.ewa
  private val allowComp = Mux(!ewa, payload.state.d.completed, Mux(dwt, payload.state.u.wdata, true.B))
  private val icnReadReceipt = payload.state.icnReadReceipt
  private val icnDBID = payload.state.icnDBID
  private val icnComp = allowComp && payload.state.icnComp
  private val icnCompDBID = icnDBID && icnComp
  icn.tx.resp.valid := valid & (icnReadReceipt || icnDBID || icnComp)
  icn.tx.resp.bits := DontCare
  icn.tx.resp.bits.Opcode := Mux(icnCompDBID, RspOpcode.CompDBIDResp, MuxCase(0.U, Seq(
    icnReadReceipt -> RspOpcode.ReadReceipt,
    icnDBID -> RspOpcode.DBIDResp,
    icnComp -> RspOpcode.Comp
  )))
  icn.tx.resp.bits.QoS := payload.info.qos
  icn.tx.resp.bits.DBID := io.idx
  icn.tx.resp.bits.TxnID := Mux(icnDBID && dwt, payload.info.returnTxnId.getOrElse(0.U), payload.info.txnId)
  icn.tx.resp.bits.SrcID := 0.U
  icn.tx.resp.bits.TgtID := Mux(icnDBID && dwt, payload.info.returnNid.getOrElse(0.U), payload.info.srcId)
  icn.tx.resp.bits.Resp := "b000".U
  when(icn.tx.resp.fire) {
    plmnu.receiptResp := icn.tx.resp.bits.Opcode === RspOpcode.ReadReceipt || plu.receiptResp
    plmnu.dbidResp := icn.tx.resp.bits.Opcode === RspOpcode.DBIDResp || icn.tx.resp.bits.Opcode === RspOpcode.CompDBIDResp || plu.dbidResp
    plmnu.comp := icn.tx.resp.bits.Opcode === RspOpcode.Comp || icn.tx.resp.bits.Opcode === RspOpcode.CompDBIDResp || plu.comp
  }

  private val busDataBytes = slvBusDataBits / 8
  private val bufDataBytes = ioDataBits / 8
  private val segNum = busDataBytes / bufDataBytes
  val slvMask = MaskGen(payload.info.addr, payload.info.size, busDataBytes)
  val slvData = Fill(segNum, payload.info.data.getOrElse(0.U(ioDataBits.W)))

  if(!p(DebugOptionsKey).FPGAPlatform) {
    val timer = Reg(UInt(64.W))
    when(icn.rx.req.fire) {
      timer := 0.U
    }.elsewhen(valid) {
      timer := timer + 1.U
    }
    when(valid) {
      assert(timer < 50000.U, "bridge CM time out!")
    }
  }
}
