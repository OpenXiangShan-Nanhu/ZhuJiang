package zhujiang.device.bridge.axilite

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import zhujiang.axi.{ARFlit, AWFlit, AxiParams, BFlit, WFlit}
import zhujiang.device.bridge.BaseCtrlMachine

class AxiLiteBridgeCtrlMachine(
  icnNode: Node,
  axiParams: AxiParams,
  outstanding: Int,
  bufDataBits: Int,
  compareTag: (UInt, UInt) => Bool
)(implicit p: Parameters)
  extends BaseCtrlMachine(
    genOpVec = new AxiLiteBridgeCtrlOpVec,
    genInfo = new AxiLiteCtrlInfo(bufDataBits),
    genRsEntry = new AxiLiteRsEntry(bufDataBits),
    node = icnNode,
    outstanding = outstanding,
    ioDataBits = bufDataBits,
    slvBusDataBits = axiParams.dataBits,
    compareTag = compareTag
  ) {
  val axi = IO(new Bundle {
    val aw = Decoupled(new AWFlit(axiParams))
    val ar = Decoupled(new ARFlit(axiParams))
    val w = Decoupled(new WFlit(axiParams))
    val b = Flipped(Decoupled(new BFlit(axiParams)))
  })
  require(axiParams.dataBits >= bufDataBits)

  axi.b.ready := true.B
  private val plmnd = payloadMiscNext.state.d
  private val pld = payload.state.d

  when(axi.b.valid) {
    plmnd.wresp := true.B
  }

  when(io.readDataFire) {
    plmnd.rdata := io.readDataLast || pld.rdata
  }

  axi.aw.valid := valid && payload.state.axiWaddr && !waiting.orR
  axi.aw.bits := DontCare
  axi.aw.bits.burst := 1.U
  axi.aw.bits.id := io.idx
  axi.aw.bits.addr := payload.info.addr
  axi.aw.bits.len := 0.U
  axi.aw.bits.size := payload.info.size
  axi.aw.bits.cache := "b0000".U
  axi.aw.bits.qos := payload.info.qos

  axi.ar.valid := valid && payload.state.axiRaddr && !waiting.orR
  axi.ar.bits := DontCare
  axi.ar.bits.burst := 1.U
  axi.ar.bits.id := io.idx
  axi.ar.bits.addr := payload.info.addr
  axi.ar.bits.len := 0.U
  axi.ar.bits.size := payload.info.size
  axi.ar.bits.cache := "b0000".U
  axi.ar.bits.qos := payload.info.qos

  axi.w.valid := valid && payload.state.axiWdata && !waiting.orR
  axi.w.bits := DontCare
  axi.w.bits.data := slvData
  axi.w.bits.strb := slvMask
  axi.w.bits.last := true.B

  when(axi.aw.fire) {
    plmnd.waddr := true.B || pld.waddr
  }
  when(axi.ar.fire) {
    plmnd.raddr := true.B || pld.raddr
  }
  when(axi.w.fire) {
    plmnd.wdata := true.B || pld.wdata
  }
}