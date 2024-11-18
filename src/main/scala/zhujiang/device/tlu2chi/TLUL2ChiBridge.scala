package zhujiang.device.tlu2chi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import zhujiang.tilelink._
import xijiang.router.base.DeviceIcnBundle
import xijiang.{Node, NodeType}
import zhujiang.ZJModule

class RequestArbiter(tlParams: TilelinkParams, outstanding: Int)(implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val a = Flipped(Decoupled(new AFlit(tlParams)))
    val alloc_s1 = Decoupled(new TaskBundle(tlParams))
    val machineStatus = Vec(outstanding, Input(new MachineStatus(tlParams)))
  })

  when(io.a.valid) {
    assert(io.a.bits.size <= 3.U, cf"Invalid size ${io.a.bits.size} for request")
  }

  private val addrMatchVec = VecInit(io.machineStatus.map(s => s.address === io.a.bits.address && s.state =/= MachineState.IDLE)).asUInt
  private val blockVec = VecInit(io.machineStatus.map(s => s.state =/= MachineState.IDLE && s.state <= MachineState.RETURN_ACK)).asUInt
  private val blockA = (addrMatchVec & blockVec).orR
  io.a.ready := io.alloc_s1.ready && !blockA

  io.alloc_s1.valid := io.a.fire
  io.alloc_s1.bits.address := io.a.bits.address
  io.alloc_s1.bits.opcode := io.a.bits.opcode
  io.alloc_s1.bits.param := io.a.bits.param
  io.alloc_s1.bits.source := io.a.bits.source
  io.alloc_s1.bits.data := io.a.bits.data
  io.alloc_s1.bits.mask := io.a.bits.mask
  io.alloc_s1.bits.size := io.a.bits.size
}

class TLUL2ChiBridge(node: Node, tlParams: TilelinkParams)(implicit p: Parameters) extends ZJModule {
  require(node.nodeType == NodeType.RI)
  require(tlParams.dataBits == 64)
  require(tlParams.addrBits == raw)
  val tlm = IO(Flipped(new TLULBundle(tlParams)))
  val icn = IO(new DeviceIcnBundle(node))

  private val reqArb = Module(new RequestArbiter(tlParams, node.outstanding))
  private val machineHandler = Module(new MachineHandler(node, tlParams, node.outstanding))
  machineHandler.io <> DontCare

  tlm.d <> machineHandler.io.tld
  reqArb.io.a <> tlm.a
  reqArb.io.machineStatus := machineHandler.io.status
  machineHandler.io.alloc_s1 <> reqArb.io.alloc_s1
  icn <> machineHandler.io.icn
}
