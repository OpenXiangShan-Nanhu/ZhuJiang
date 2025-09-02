package zhujiang.system

import chisel3._
import org.chipsalliance.cde.config.Parameters
import zhujiang.chip.ChipWrapper

class SocSystem(implicit p:Parameters) extends BaseSystem {
  private val chip = Module(new ChipWrapper)
  io.onReset := chip.io.onReset
  chip.io.ci := 0.U

  val ccnDrv = chip.ccnPorts.map(c => (c, s"_${c.node.domainId}"))
  val saxiDrv = chip.dmaPorts.zipWithIndex.map(s => (s._1, s"${attrStr(s._1.params.attr, s._2, chip.dmaPorts.size)}"))
  val hwaDrv = chip.hwaPorts.map(h => (h, "")).toSeq

  runIOAutomation()
}


