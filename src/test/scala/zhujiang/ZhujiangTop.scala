package zhujiang

import chisel3._
import chisel3.util.log2Ceil
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.FirtoolOption
import org.chipsalliance.cde.config.{Config, Parameters}
import xijiang.tfb.TrafficBoardFileManager
import xijiang.{NodeParam, NodeType}
import xs.utils.FileRegisters
import xs.utils.debug.{HardwareAssertionKey, HwaParams}
import xs.utils.perf._
import xs.utils.stage.XsStage
import zhujiang.system._
import zhujiang.axi.AxiParams
import zhujiang.device.AxiDeviceParams
import scala.annotation.tailrec
import freechips.rocketchip.diplomacy.{AddressRange, AddressSet}

case class MemoryRange(lower: BigInt, upper: BigInt) {
  def cover(addr: BigInt): Boolean = addr >= lower && addr < upper
  def cover(addr: UInt): Bool = addr >= lower.U && addr < upper.U
}

object AddrConfig {
  // interleaving granularity: 64B
  // DDR: 0x0_8000_0000 ~ 0x1F_FFFF_FFFF
  val interleaveOffset = 6
  val pmemRange = MemoryRange(0x00_8000_0000L, 0x20_0000_0000L)
  private val interleaveBits = 1
  private val interleaveMask = ((0x1L << interleaveBits) - 1) << interleaveOffset
  private val memFullMask = (1L << log2Ceil(pmemRange.upper)) - 1

  val memFullAddrSet = AddressSet(0x0L, memFullMask).subtract(AddressSet(0x0L, (1L << log2Ceil(pmemRange.lower)) - 1))
  private val fullMask = (1L << 44) - 1

  def memBank(bank: Long):Seq[(Long, Long)] = {
    require(bank < (0x1L << interleaveBits))
    memFullAddrSet.map(as => {
      val base = as.base | (bank << interleaveOffset)
      val mask = (as.mask.toLong ^ fullMask) | interleaveMask
      (base.toLong, mask)
    })
  }

  val mem0 = memBank(0)
  val mem1 = memBank(1)

  val mem_nc = Seq(
    (0x300_0000_0000L, 0xF00_0000_0000L),
  )
}

class ZhujiangTopConfig extends Config((site, here, up) => {
  case HardwareAssertionKey => HwaParams(enable = true)
  case PerfCounterOptionsKey => PerfCounterOptions(enablePerfPrint = false, enablePerfDB = false, XSPerfLevel.VERBOSE, 0)
  case ZJParametersKey => ZJParameters(
    hnxBankOff = AddrConfig.interleaveOffset,
    nodeParams = Seq(
      NodeParam(nodeType = NodeType.CC, socket = "sync"),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.HF, bankId = 0, hfpId = 0),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.S, axiDevParams = Some(AxiDeviceParams(4, 32, "north", "mem_0")), addrSets = AddrConfig.mem0),
      NodeParam(nodeType = NodeType.S, axiDevParams = Some(AxiDeviceParams(4, 32, "north", "mem_1")), addrSets = AddrConfig.mem1),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.HF, bankId = 1, hfpId = 0),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.CC, socket = "sync"),

      NodeParam(nodeType = NodeType.CC, socket = "sync"),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.HF, bankId = 3, hfpId = 0),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.RI, axiDevParams = Some(AxiDeviceParams(1, 64, "south", "main", Some(AxiParams(idBits = 14))))),
      NodeParam(nodeType = NodeType.HI, axiDevParams = Some(AxiDeviceParams(1, 8, "south", "main")), defaultHni = true),
      NodeParam(nodeType = NodeType.S, axiDevParams = Some(AxiDeviceParams(1, 32, "south", "uc")), addrSets = AddrConfig.mem_nc),
      NodeParam(nodeType = NodeType.M, axiDevParams = Some(AxiDeviceParams(5, 32, "south", "hwa"))),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.HF, bankId = 2, hfpId = 0),
      NodeParam(nodeType = NodeType.P),
      NodeParam(nodeType = NodeType.CC, socket = "sync"),
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

object SocSystemTop extends App {
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
    ChiselGeneratorAnnotation(() => new SocSystem()(config))
  ))
  FileRegisters.write()
}