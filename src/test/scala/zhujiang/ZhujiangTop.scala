package zhujiang

import org.chipsalliance.cde.config.{Config, Parameters}
import xijiang.{NodeParam, NodeType}
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.{ChiselStage, FirtoolOption}
import xijiang.tfb.TrafficBoardFileManager
import xs.utils.FileRegisters
import xs.utils.perf.{DebugOptions, DebugOptionsKey}

import scala.annotation.tailrec

/**
 * CC----S0----HF0----S1----CC
 * |                        |
 * SMem                     RI
 * |         Local          |
 * HI                       HI default
 * |                        |
 * CC----S0----HF1----S1----CC
 */

class ZhujiangTopConfig extends Config((site, here, up) => {
  case ZJParametersKey => ZJParameters(
    localNodeParams = Seq(
      NodeParam(nodeType = NodeType.S, bankId = 0, splitFlit = true),
      NodeParam(nodeType = NodeType.CC, cpuNum = 2, splitFlit = true, outstanding = 8, attr = "nanhu"),
      NodeParam(nodeType = NodeType.HF, bankId = 0, splitFlit = true),
      NodeParam(nodeType = NodeType.CC, cpuNum = 2, splitFlit = true, outstanding = 8, attr = "nanhu"),
      NodeParam(nodeType = NodeType.S, bankId = 1, splitFlit = true),
      NodeParam(nodeType = NodeType.HI, defaultHni = true, splitFlit = true, attr = "cfg"),
      NodeParam(nodeType = NodeType.RI, attr = "dma", splitFlit = true),
      NodeParam(nodeType = NodeType.HI, addressRange = (0x3803_0000, 0x3804_0000), splitFlit = true, attr = "ddr_cfg"),
      NodeParam(nodeType = NodeType.S, mainMemory = true, splitFlit = true, outstanding = 32, attr = "ddr_data")
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
  (new ChiselStage).execute(firrtlOpts, Seq(
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
