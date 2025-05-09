package xijiang

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.FirtoolOption
import org.chipsalliance.cde.config.Parameters
import xijiang.tfb.TrafficBoardFileManager
import xijiang.tfs.{TrafficSimFileManager, TrafficSimParams}
import xs.utils.FileRegisters
import xs.utils.stage.XsStage
import zhujiang.{ZJModule, ZJParametersKey, ZhujiangTopConfig}

import scala.annotation.tailrec

object TfsTopParser {
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
  val (_config, firrtlOpts) = TfsTopParser(args)
  val config = _config.alterPartial({
    case ZJParametersKey => _config(ZJParametersKey).copy(tfsParams = Some(TrafficSimParams()))
  })
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
    ChiselGeneratorAnnotation(() => new TrafficSimTop()(config))
  ))
  if(config(ZJParametersKey).tfbParams.isDefined) TrafficBoardFileManager.release(config)
  if(config(ZJParametersKey).tfsParams.isDefined) TrafficSimFileManager.release(config)
  FileRegisters.write()
}