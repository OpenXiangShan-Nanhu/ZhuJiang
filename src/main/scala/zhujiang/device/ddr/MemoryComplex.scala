package zhujiang.device.ddr

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import zhujiang.ZJModule
import zhujiang.axi.{AxiBuffer, AxiBundle, AxiParams, BaseAxiXbar}
import zhujiang.device.bridge.axi.AxiBridge
import zhujiang.device.bridge.axilite.AxiLiteBridge

sealed class MemoryComplexCrossBar(mstParams: Seq[AxiParams]) extends BaseAxiXbar(mstParams) {
  val slvMatchersSeq = Seq((_: UInt) => true.B)
  initialize()
}

class MemoryComplex(cfgNode: Node, memNode: Node)(implicit p: Parameters) extends ZJModule {
  require(cfgNode.nodeType == NodeType.HI)
  require(memNode.nodeType == NodeType.S && memNode.mainMemory)
  private val chiCfgBridge = Module(new AxiLiteBridge(cfgNode, dw, 3))
  private val chiMemBridge = Module(new AxiBridge(memNode))
  private val memXBar = Module(new MemoryComplexCrossBar(Seq(chiCfgBridge.axi.params, chiMemBridge.axi.params)))
  private val memBuffer = Module(new AxiBuffer(memXBar.io.downstream.head.params))
  val io = IO(new Bundle {
    val icn = new Bundle {
      val cfg = new DeviceIcnBundle(cfgNode)
      val mem = new DeviceIcnBundle(memNode)
    }
    val ddr = new AxiBundle(memBuffer.io.out.params)
  })
  chiCfgBridge.icn <> io.icn.cfg
  chiMemBridge.icn <> io.icn.mem
  memXBar.io.upstream.head <> chiCfgBridge.axi
  memXBar.io.upstream.last <> chiMemBridge.axi
  memBuffer.io.in <> memXBar.io.downstream.head
  io.ddr <> memBuffer.io.out
}
