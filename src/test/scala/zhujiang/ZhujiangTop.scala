package zhujiang

import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.FirtoolOption
import org.chipsalliance.cde.config.{Config, Parameters}
import xijiang.tfb.TrafficBoardFileManager
import xijiang.{NodeParam, NodeType}
import xs.utils.FileRegisters
import xs.utils.debug.{HardwareAssertionKey, HwaParams}
import xs.utils.perf._
import xs.utils.stage.XsStage
import zhujiang.axi.AxiParams
import zhujiang.device.AxiDeviceParams

import scala.annotation.tailrec

object AddrConfig {
  // interleaving granularity: 1KiB
  val mem0 = Seq(
    (0x0000_8000_0000L, 0x000F_8000_0400L),
    (0x0001_0000_0000L, 0x000F_0000_0400L),
    (0x0002_0000_0000L, 0x000F_0000_0400L),
    (0x0003_0000_0000L, 0x000F_0000_0400L),
    (0x0004_0000_0000L, 0x000F_0000_0400L),
  )
  val mem1 = Seq(
    (0x0000_8000_0400L, 0x000F_8000_0400L),
    (0x0001_0000_0400L, 0x000F_0000_0400L),
    (0x0002_0000_0400L, 0x000F_0000_0400L),
    (0x0003_0000_0400L, 0x000F_0000_0400L),
    (0x0004_0000_0400L, 0x000F_0000_0400L),
  )
  val mem2 = Seq(
    (0x0005_0000_0000L, 0x000F_0000_0000L),
    (0x0006_0000_0000L, 0x000F_0000_0000L),
    (0x0007_0000_0000L, 0x000F_0000_0000L),
    (0x0008_0000_0000L, 0x000F_0000_0000L),
    (0x0009_0000_0000L, 0x000F_0000_0000L),
    (0x000A_0000_0000L, 0x000F_0000_0000L),
    (0x000B_0000_0000L, 0x000F_0000_0000L),
    (0x000C_0000_0000L, 0x000F_0000_0000L),
  )
  val mem3 = Seq(
    (0x0005_0000_0000L, 0x000F_0000_0000L),
    (0x0006_0000_0000L, 0x000F_0000_0000L),
    (0x0007_0000_0000L, 0x000F_0000_0000L),
    (0x0008_0000_0000L, 0x000F_0000_0000L),
    (0x0009_0000_0000L, 0x000F_0000_0000L),
    (0x000A_0000_0000L, 0x000F_0000_0000L),
    (0x000B_0000_0000L, 0x000F_0000_0000L),
    (0x000C_0000_0000L, 0x000F_0000_0000L),
  )
  val memUc = Seq(
    (0x0000_8000_0000L, 0x000F_8000_0000L),
  )
}

class ZhujiangTopConfig extends Config((site, here, up) => {
  case HardwareAssertionKey => HwaParams(enable = true)
  case PerfCounterOptionsKey => PerfCounterOptions(enablePerfPrint = false, enablePerfDB = false, XSPerfLevel.VERBOSE, 0)
  case ZJParametersKey => ZJParameters(
    nodeParams = Seq(
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.HF, bankId = 0, hfpId = 0),
      NodeParam(nodeType = NodeType.HF, bankId = 1, hfpId = 0),
      NodeParam(nodeType = NodeType.RI, axiDevParams = Some(AxiDeviceParams(1, 64, "north", "mem_n_0"))),
      NodeParam(nodeType = NodeType.RI, axiDevParams = Some(AxiDeviceParams(1, 64, "north", "mem_n_1"))),
      NodeParam(nodeType = NodeType.HF, bankId = 2, hfpId = 0),
      NodeParam(nodeType = NodeType.HF, bankId = 3, hfpId = 0),
      NodeParam(nodeType = NodeType.P),

      NodeParam(nodeType = NodeType.S,  axiDevParams = Some(AxiDeviceParams(1, 32, "east", "e_0")),  addrSets = AddrConfig.mem2),
      NodeParam(nodeType = NodeType.S,  axiDevParams = Some(AxiDeviceParams(1, 32, "east", "e_1")),  addrSets = AddrConfig.mem3),

      NodeParam(nodeType = NodeType.CC, socket = "async"),
      NodeParam(nodeType = NodeType.HF, bankId = 3, hfpId = 1),
      NodeParam(nodeType = NodeType.CC, socket = "async"),
      NodeParam(nodeType = NodeType.HF, bankId = 2, hfpId = 1),
      NodeParam(nodeType = NodeType.HI, axiDevParams = Some(AxiDeviceParams(0, 32, "south", "main")), defaultHni = true),
      NodeParam(nodeType = NodeType.RH, axiDevParams = Some(AxiDeviceParams(0, 32, "south", "main", Some(AxiParams(idBits = 15))))),
      NodeParam(nodeType = NodeType.M,  axiDevParams = Some(AxiDeviceParams(3, 32, "south"))),
      NodeParam(nodeType = NodeType.HF, bankId = 1, hfpId = 1),
      NodeParam(nodeType = NodeType.CC, socket = "async"),
      NodeParam(nodeType = NodeType.HF, bankId = 0, hfpId = 1),
      NodeParam(nodeType = NodeType.CC, socket = "async"),

      NodeParam(nodeType = NodeType.S,  axiDevParams = Some(AxiDeviceParams(1, 32, "west", "w_0")),  addrSets = AddrConfig.mem0),
      NodeParam(nodeType = NodeType.S,  axiDevParams = Some(AxiDeviceParams(1, 32, "west", "w_1")),  addrSets = AddrConfig.mem1),
    )
  )
  case DebugOptionsKey => DebugOptions(EnablePerfDebug = false)
})

object ZhujiangTopParser {
  def apply(args: Array[String]): (Parameters, Array[String]) = {
    val defaultConfig = new ZhujiangTopConfig
    var firrtlOpts = Array[String]()
    var hasHelp: Boolean = false

    @tailrec
    def parse(config: Parameters, args: List[String]): Parameters = {
      args match {
        case Nil => config

        case "--help" :: tail =>
          hasHelp = true
          parse(config, tail)

        case "--prefix" :: confString :: tail =>
          parse(config.alter((site, here, up) => {
            case ZJParametersKey => up(ZJParametersKey).copy(modulePrefix = confString)
          }), tail)

        case option :: tail =>
          firrtlOpts :+= option
          parse(config, tail)
      }
    }

    val cfg = parse(defaultConfig, args.toList)
    if(hasHelp) firrtlOpts :+= "--help"
    (cfg, firrtlOpts)
  }
}

object ZhujiangTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, Seq(
    FirtoolOption("-O=release"),
    FirtoolOption("--disable-all-randomization"),
    FirtoolOption("--disable-annotation-unknown"),
    FirtoolOption("--strip-debug-info"),
    FirtoolOption("--lower-memories"),
    FirtoolOption("--add-vivado-ram-address-conflict-synthesis-bug-workaround"),
    FirtoolOption("--lowering-options=noAlwaysComb," +
      " disallowLocalVariables, disallowMuxInlining," +
      " emittedLineLength=120, explicitBitcast, locationInfoStyle=plain"),
    ChiselGeneratorAnnotation(() => new Zhujiang()(config))
  ))
  if(config(ZJParametersKey).tfbParams.isDefined) TrafficBoardFileManager.release(config)
  FileRegisters.write()
}
