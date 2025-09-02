package zhujiang.chip

import chisel3._
import chisel3.experimental.hierarchy.{instantiable, public}
import chisel3.experimental.noPrefix
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.IcnBundle
import zhujiang._
import zhujiang.axi.AxiBundle
import zhujiang.device.socket.SocketDevSide
import zhujiang.{ZJDftWires, ZJModule, Zhujiang}
import zhujiang.axi.{AxiParams, AxiUtils, BaseAxiXbar}

trait AttrHelper {
  def attrStr(attr:String, idx:Int, size:Int):String = {
    if(attr != "") {
      s"_$attr"
    } else if(size == 1) {
      s""
    } else {
      s"_$idx"
    }
  }
}

@instantiable
class ChipWrapper(implicit p:Parameters) extends ZJModule with AttrHelper {
  private val noc = Module(new Zhujiang)

  @public val io = IO(new Bundle {
    val ci = Input(UInt(ciIdBits.W))
    val onReset = Output(Bool())
  })

  noc.io.ci := io.ci
  io.onReset := noc.io.onReset
  noc.io.dft := 0.U.asTypeOf(new ZJDftWires)
  noc.io.ramctl := DontCare

  for ((ddr, i) <- noc.ddrIO.zipWithIndex) yield noPrefix {
    val ram = Module(new AxiRam(ddr.params))
    ddr <> ram.axi
    ram.suggestName(String.format("ram_bank_%d_%s", i, ddr.params.attr))
  }

  @public val ccnPorts = for ((ccn, i) <- noc.ccnIO.zipWithIndex) yield noPrefix {
    val socket = Module(new SocketDevSide(ccn.node))
    val iop = IO(new IcnBundle(ccn.node))
    socket.io.socket <> ccn
    socket.reset := ccn.resetTx
    iop <> socket.io.icn
    socket.suggestName(ccn.node.deviceName)
    iop.suggestName(s"ccn_${ccn.node.domainId}")
    iop
  }

  for ((cfg, i) <- noc.cfgIO.zipWithIndex) yield noPrefix {
    val ram = Module(new AxiRam(cfg.params))
    cfg <> ram.axi
    ram.suggestName(String.format("ram_cfg_%d_%s", i, cfg.params.attr))
  }

  @public val dmaPorts = for ((dma, i) <- noc.dmaIO.zipWithIndex) yield noPrefix {
    val iop = IO(Flipped(new AxiBundle(dma.params)))
    val attr = attrStr(dma.params.attr, i, noc.dmaIO.size)
    dma <> iop
    iop.suggestName(s"s_axi$attr")
    iop
  }

  @public val hwaPorts = for (hwa <- noc.hwaIO) yield noPrefix {
    val iop = IO(Flipped(new AxiBundle(hwa.params)))
    hwa <> iop
    iop.suggestName(s"s_axi_hwa")
    iop
  }
}
