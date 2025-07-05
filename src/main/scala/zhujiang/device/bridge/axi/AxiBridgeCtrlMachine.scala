package zhujiang.device.bridge.axi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import zhujiang.ZJParametersKey
import zhujiang.axi.{ARFlit, AWFlit, AxiParams, BFlit, WFlit}
import zhujiang.chi.DatOpcode
import zhujiang.device.bridge.{BaseCtrlMachine, IcnIoDevCtrlOpVecCommon}

class AxiBridgeCtrlMachine(
  icnNode: Node,
  axiParams: AxiParams,
  outstanding: Int,
  compareTag: (UInt, UInt) => Bool
)(implicit p: Parameters)
  extends BaseCtrlMachine(
    genOpVec = new AxiBridgeCtrlOpVec,
    genInfo = new AxiCtrlInfo,
    genRsEntry = new AxiRsEntry,
    node = icnNode,
    outstanding = outstanding,
    ioDataBits = 64,
    slvBusDataBits = p(ZJParametersKey).dataBits,
    compareTag = compareTag
  ) {
  val axi = IO(new Bundle {
    val aw = Decoupled(new AWFlit(axiParams))
    val ar = Decoupled(new ARFlit(axiParams))
    val w = Decoupled(new WFlit(axiParams))
    val b = Flipped(Decoupled(new BFlit(axiParams)))
  })
  val dataBufferAlloc = IO(new Bundle {
    val req = Decoupled(new DataBufferAllocReq(outstanding))
    val resp = Input(Bool())
  })

  private val allocReqIssued = Reg(Bool())
  dataBufferAlloc.req.valid := valid && !allocReqIssued && !payload.state.bufferAllocated && waiting === 0.U
  dataBufferAlloc.req.bits.size := payload.info.size
  dataBufferAlloc.req.bits.qos := payload.info.qos
  if(dw == 128) {
    dataBufferAlloc.req.bits.dataIdOffset := payload.info.addr(5, 4)
  } else if(dw == 256) {
    dataBufferAlloc.req.bits.dataIdOffset := Cat(payload.info.addr(5), false.B)
  } else {
    dataBufferAlloc.req.bits.dataIdOffset := 0.U
  }

  when(icn.rx.req.fire) {
    allocReqIssued := false.B
  }.elsewhen(dataBufferAlloc.req.fire) {
    allocReqIssued := true.B
  }

  when(dataBufferAlloc.resp) {
    payloadMiscNext.state.bufferAllocated := true.B || payload.state.bufferAllocated
  }

  axi.b.ready := true.B
  private val plmnd = payloadMiscNext.state.d
  private val pld = payload.state.d

  when(axi.b.valid) {
    plmnd.wresp := true.B
  }

  when(io.readDataFire) {
    plmnd.rdata := io.readDataLast || pld.rdata
  }

  private val busSize = log2Ceil(axiParams.dataBits / 8).U
  private val largeLen = (1.U(8.W) << (payload.info.size - busSize)).asUInt
  private val len = Mux(payload.info.size > busSize, largeLen - 1.U, 0.U)
  private val size = Mux(payload.info.size > busSize, busSize, payload.info.size)
  private val cache = Cat(false.B, false.B, !payload.info.device, payload.info.ewa)
  axi.aw.valid := valid && payload.state.axiWaddr && !waiting.orR
  axi.aw.bits := DontCare
  axi.aw.bits.id := io.idx
  axi.aw.bits.addr := payload.info.addr
  axi.aw.bits.len := len
  axi.aw.bits.burst := 1.U
  axi.aw.bits.size := size
  axi.aw.bits.cache := cache
  axi.aw.bits.qos := payload.info.qos

  axi.ar.valid := valid && payload.state.axiRaddr && !waiting.orR
  axi.ar.bits := DontCare
  axi.ar.bits.id := io.idx
  axi.ar.bits.addr := payload.info.addr
  axi.ar.bits.len := len
  axi.ar.bits.burst := 1.U
  axi.ar.bits.size := size
  axi.ar.bits.cache := cache
  axi.ar.bits.qos := payload.info.qos

  axi.w.valid := valid && payload.state.axiWdata && !waiting.orR
  axi.w.bits := DontCare

  when(axi.aw.fire) {
    plmnd.waddr := true.B || pld.waddr
  }
  when(axi.ar.fire) {
    plmnd.raddr := true.B || pld.raddr
  }
  when(axi.w.fire) {
    plmnd.wdata := true.B || pld.wdata
  }
  when(icn.rx.data.valid && icn.rx.data.bits.Opcode === DatOpcode.WriteDataCancel) {
    plmnd.waddr := true.B || pld.waddr
    plmnd.wdata := true.B || pld.wdata
    plmnd.wresp := true.B || pld.wresp
  }
}
