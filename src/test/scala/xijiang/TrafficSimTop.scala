package xijiang

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.{ChiselStage, FirtoolOption}
import org.chipsalliance.cde.config.{Config, Parameters}
import xijiang.tfb.TrafficBoardFileManager
import xijiang.tfs.{TrafficSimFileManager, TrafficSimParams}
import xs.utils.FileRegisters
import xs.utils.debug.{HardwareAssertionKey, HwaParams}
import xs.utils.perf.{PerfCounterOptions, PerfCounterOptionsKey, XSPerfLevel}
import zhujiang.{ZJModule, ZJParameters, ZJParametersKey}

import scala.annotation.tailrec

class TfsTopConfig extends Config((site, here, up) => {
  case HardwareAssertionKey => HwaParams(enable = true)
  case PerfCounterOptionsKey => PerfCounterOptions(enablePerfPrint = false, enablePerfDB = false, XSPerfLevel.VERBOSE, 0)
  case ZJParametersKey => ZJParameters(
    nodeParams = Seq(
      NodeParam(nodeType = NodeType.HF, bankId = 0, hfpId = 0),
      NodeParam(nodeType = NodeType.CC, cpuNum = 2, outstanding = 8, attr = "nanhu", socket = "c2c"),
      NodeParam(nodeType = NodeType.HF, bankId = 1, hfpId = 0),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.HF, bankId = 2, hfpId = 0),
      NodeParam(nodeType = NodeType.CC, cpuNum = 2, outstanding = 8, attr = "nanhu", socket = "c2c"),
      NodeParam(nodeType = NodeType.HF, bankId = 3, hfpId = 0),

      NodeParam(nodeType = NodeType.RI, attr = "main"),
      NodeParam(nodeType = NodeType.HI, defaultHni = true, attr = "main"),
      NodeParam(nodeType = NodeType.M),

      NodeParam(nodeType = NodeType.HF, bankId = 3, hfpId = 1),
      NodeParam(nodeType = NodeType.CC, cpuNum = 2, outstanding = 8, attr = "nanhu", socket = "c2c"),
      NodeParam(nodeType = NodeType.HF, bankId = 2, hfpId = 1),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.HF, bankId = 1, hfpId = 1),
      NodeParam(nodeType = NodeType.CC, cpuNum = 2, outstanding = 8, attr = "nanhu", socket = "c2c"),
      NodeParam(nodeType = NodeType.HF, bankId = 0, hfpId = 1),

      NodeParam(nodeType = NodeType.S, addrSegment = (0x0L << 6, 0x1L << 6)),
      NodeParam(nodeType = NodeType.S, addrSegment = (0x1L << 6, 0x1L << 6)),
      NodeParam(nodeType = NodeType.P)
    ),
    tfsParams = Some(TrafficSimParams())
  )
})

object TfsTopParser {
  def apply(args: Array[String]): (Parameters, Array[String]) = {
    val defaultConfig = new TfsTopConfig
    var firrtlOpts = Array[String]()
    var hasHelp: Boolean = false

    @tailrec
    def parse(config: Parameters, args: List[String]): Parameters = {
      args match {
        case Nil => config

        case "--help" :: tail =>
          hasHelp = true
          parse(config, tail)

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

class TrafficSimTop(implicit p: Parameters) extends ZJModule {
  require(p(ZJParametersKey).tfsParams.isDefined)
  private val reset_state = IO(Output(Bool()))
  private val localRing = Module(new Ring)
  localRing.io_ci := 0.U
  localRing.dfx_reset := DontCare
  reset_state := localRing.reset_state.get
}

object TrafficSimTopMain extends App {
  val (config, firrtlOpts) = TfsTopParser(args)
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
    ChiselGeneratorAnnotation(() => new TrafficSimTop()(config))
  ))
  if(config(ZJParametersKey).tfbParams.isDefined) TrafficBoardFileManager.release(config)
  if(config(ZJParametersKey).tfsParams.isDefined) TrafficSimFileManager.release(config)
  FileRegisters.write()
}