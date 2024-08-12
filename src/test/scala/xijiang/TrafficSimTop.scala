package xijiang

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.{ChiselStage, FirtoolOption}
import org.chipsalliance.cde.config.{Config, Parameters}
import xijiang.tfb.TrafficBoardFileManager
import xijiang.tfs.{TrafficSimFileManager, TrafficSimParams}
import xs.utils.FileRegisters
import zhujiang.{ZJModule, ZJParameters, ZJParametersKey}

import scala.annotation.tailrec

class TfsTopConfig extends Config((site, here, up) => {
  case ZJParametersKey => ZJParameters(
    localNodeParams = Seq(
      NodeParam(nodeType = NodeType.R),
      NodeParam(nodeType = NodeType.R),
      NodeParam(nodeType = NodeType.HF),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.R),
      NodeParam(nodeType = NodeType.R),
      NodeParam(nodeType = NodeType.HF),
      NodeParam(nodeType = NodeType.HI),
    ),
    csnNodeParams = Seq(
      NodeParam(nodeType = NodeType.R),
      NodeParam(nodeType = NodeType.HF),
      NodeParam(nodeType = NodeType.C),
      NodeParam(nodeType = NodeType.P),
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
  private val localRing = Module(new Ring(true))
  private val csnRing = Module(new Ring(false))
  localRing.tfsio.get.chip := 0.U
  csnRing.tfsio.get.chip := 0.U
  csnRing.tfsio.get.remoteChip.zipWithIndex.foreach({ case (c, i) => c := (i + 1).U })
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