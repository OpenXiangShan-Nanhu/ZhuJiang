package zhujiang

import xijiang.{Node, NodeParam, NodeType}
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.FirtoolOption
import xijiang.c2c.C2cLoopBack
import xijiang.router.base.{EjectBuffer, SingleChannelTap}
import xs.utils.stage.XsStage
import zhujiang.UnitTop.{firtoolOpts, firtoolOptsDebug, firtoolOptsWithDebugInfo, firtoolOptsDebugWithDebugInfo}
import zhujiang.chi.{RingFlit, SnoopFlit}
import zhujiang.device.bridge.axi.AxiBridge
import zhujiang.device.bridge.axilite.AxiLiteBridge
import zhujiang.device.bridge.tlul.TLULBridge
import zhujiang.device.dma.Axi2Chi
import zhujiang.device.home.HomeWrapper
import zhujiang.device.tlu2chi.TLUL2ChiBridge
import zhujiang.tilelink.TilelinkParams
import zhujiang.{ZJParameters, ZJParametersKey}
import dongjiang.{DJParam, DongJiang, DongJiangTop}
import dongjiang.directory._
import dongjiang.data._
import dongjiang.backend._
import dongjiang.frontend._
import xs.utils.debug.{HardwareAssertionKey, HwaParams}
import zhujiang.device.AxiDeviceParams

object UnitTop {
  val _firtoolOpts = Seq(
    FirtoolOption("--export-module-hierarchy"),
    FirtoolOption("--disable-all-randomization"),
    FirtoolOption("--disable-annotation-unknown"),
    FirtoolOption("--lower-memories"),
    FirtoolOption("--add-vivado-ram-address-conflict-synthesis-bug-workaround"),
    FirtoolOption("--lowering-options=noAlwaysComb," +
      " disallowLocalVariables, disallowMuxInlining," +
      " emittedLineLength=120, explicitBitcast, locationInfoStyle=plain"))
  val firtoolOpts = Seq(FirtoolOption("-O=release"), FirtoolOption("--strip-debug-info")) ++ _firtoolOpts
  val firtoolOptsWithDebugInfo = Seq(FirtoolOption("-O=release")) ++ _firtoolOpts
  val firtoolOptsDebug = Seq(FirtoolOption("-O=debug"), FirtoolOption("--strip-debug-info")) ++ _firtoolOpts
  val firtoolOptsDebugWithDebugInfo = Seq(FirtoolOption("-O=debug")) ++ _firtoolOpts
}

object AxiBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new AxiBridge(Node(nodeType = NodeType.S, axiDevParams = Some(AxiDeviceParams())))(config))
  ))
}

object AxiLiteBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new AxiLiteBridge(Node(nodeType = NodeType.HI, axiDevParams = Some(AxiDeviceParams())), 64, 3)(config))
  ))
}

object TLULBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new TLULBridge(Node(nodeType = NodeType.HI, axiDevParams = Some(AxiDeviceParams())), 64, 3)(config))
  ))
}

object TLUL2ChiBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new TLUL2ChiBridge(Node(nodeType = NodeType.RI, axiDevParams = Some(AxiDeviceParams())), TilelinkParams(addrBits = 48, userBits = 2 /* Extra two bits for svbpmt */))(config))
  ))
}

object DmaTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new Axi2Chi(Node(nodeType = NodeType.RI, axiDevParams = Some(AxiDeviceParams(outstanding = 32))))(config))
  ))
}

object SctTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => {
      val hrf = new SnoopFlit()(config)
      val ringf = new RingFlit(hrf.getWidth)(config)
      new SingleChannelTap(ringf, "SNP")(config)
    })
  ))
}

object EbTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => {
      val hrf = new SnoopFlit()(config)
      val ringf = new RingFlit(hrf.getWidth)(config)
      new EjectBuffer(ringf, 5, "SNP")(config)
    })
  ))
}

object ClbTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new C2cLoopBack()(config))
  ))
}

object HomeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  val homeNodeGrps = config(ZJParametersKey).island.filter(_.nodeType == NodeType.HF).groupBy(_.bankId)
  val nrHfFrnd = homeNodeGrps.flatMap(_._2).map(_.friends.size).max
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new HomeWrapper(homeNodeGrps.head._2, nrHfFrnd)(config))
  ))
}

object DirectoryTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new Directory(powerCtl = false)(config))
  ))
}

object DataBlockTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new DataBlock(powerCtl = false)(config))
  ))
}

object BackendTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new Backend()(config))
  ))
}

object FrontendTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new Frontend()(config))
  ))
}

object DongJiangTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  val nrDirBank = 2
  val nrRNF = 4
  (new XsStage).execute(firrtlOpts, firtoolOptsWithDebugInfo ++ Seq(
    ChiselGeneratorAnnotation(() => new DongJiangTop()(config.alterPartial({
      case HardwareAssertionKey => config(HardwareAssertionKey).copy(enable = false)
      case ZJParametersKey => config(ZJParametersKey).copy(
        djParamsOpt = Some(DJParam(
          llcSizeInB = 8 * 1024 / nrRNF,
          sfSizeInB = 8 * 2 * 1024 / nrRNF,
          nrReqTaskBuf = 4 * nrDirBank, // 2 dirBank, each has 4 reqTaskBuf
          nrSnpTaskBuf = 2,
          nrPoS = 8 * nrDirBank, // posWays = if(hasLLC) min(llcWays, sfWays) else sfWays, posSets = nrPoS / posWays, posSets(for each bank) = posSets / nrDirBank
          nrReadCM = 4,
          dataBufSizeInByte = 16 * 32, // nrDataBuf = dataBufSizeInByte / BeatByte
          nrDSBank = 2,
          nrDirBank = nrDirBank,
          llcWays = 4,
          sfWays = 4,
        ))
      )
    })))
  ))
}