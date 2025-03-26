package zhujiang.device.misc

import chisel3._
import chisel3.util.{Cat, Decoupled, Queue}
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.debug.HardwareAssertionKey
import zhujiang.{ZJBundle, ZJModule}
import zhujiang.chi.{NodeIdBundle, RingFlit}

class ResetDevice extends Module {
  val io = IO(new Bundle {
    val resetInject = Output(Vec(2, Bool()))
    val resetState = Input(Vec(2, Bool()))
    val onReset = Output(Bool())
  })
  private val resetReg = RegInit(3.U(2.W))
  io.resetInject(0) := resetReg(0)
  io.resetInject(1) := resetReg(1)
  when(resetReg === 3.U) {
    resetReg := 1.U
  }.elsewhen(resetReg === 1.U && io.resetState(1) === false.B) {
    resetReg := 0.U
  }
  io.onReset := RegNext(Cat(io.resetState).orR)
}

class ZJDebugBundle(implicit p:Parameters) extends ZJBundle {
  val src = UInt(nodeNidBits.W)
  val id = UInt(p(HardwareAssertionKey).maxInfoBits.W)
}

class MiscDevice(node: Node)(implicit p:Parameters) extends ZJModule {
  require(node.nodeType == NodeType.M)
  private val hwaP = p(HardwareAssertionKey)
  val io = IO(new Bundle {
    val icn = new DeviceIcnBundle(node, true)
    val onReset = Output(Bool())
    val hwa = Option.when(hwaP.enable)(Decoupled(new ZJDebugBundle))
  })
  private val resetDev = Module(new ResetDevice)
  resetDev.io.resetState := io.icn.resetState.get
  io.icn.resetInject.get := resetDev.io.resetInject
  io.onReset := resetDev.io.onReset
  if(hwaP.enable) {
    val hwaQ = Module(new Queue(new ZJDebugBundle, entries = 2))
    io.hwa.get <> hwaQ.io.deq

    hwaQ.io.enq.valid := io.icn.rx.debug.get.valid
    io.icn.rx.debug.get.ready := hwaQ.io.enq.ready
    val dbgFlit = io.icn.rx.debug.get.bits.asTypeOf(new RingFlit(debugFlitBits))
    hwaQ.io.enq.bits.src := dbgFlit.SrcID.asTypeOf(new NodeIdBundle).nid
    hwaQ.io.enq.bits.id := dbgFlit.Payload
  }
}