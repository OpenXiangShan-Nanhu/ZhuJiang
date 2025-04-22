package zhujiang

import xijiang.{Node, NodeParam, NodeType}
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.FirtoolOption
import xijiang.c2c.C2cLoopBack
import xijiang.router.base.{EjectBuffer, SingleChannelTap}
import xijiang.{Node, NodeType}
import xs.utils.stage.XsStage
import zhujiang.UnitTop.{firtoolOpts, firtoolOptsDebug}
import zhujiang.chi.{RingFlit, SnoopFlit}
import zhujiang.device.bridge.axi.AxiBridge
import zhujiang.device.bridge.axilite.AxiLiteBridge
import zhujiang.device.bridge.tlul.TLULBridge
import zhujiang.device.ddr.MemoryComplex
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

object UnitTop {
  val _firtoolOpts = Seq(
    FirtoolOption("--export-module-hierarchy"),
    FirtoolOption("--disable-all-randomization"),
    FirtoolOption("--disable-annotation-unknown"),
    FirtoolOption("--strip-debug-info"),
    FirtoolOption("--lower-memories"),
    FirtoolOption("--add-vivado-ram-address-conflict-synthesis-bug-workaround"),
    FirtoolOption("--lowering-options=noAlwaysComb," +
      " disallowLocalVariables, disallowMuxInlining," +
      " emittedLineLength=120, explicitBitcast, locationInfoStyle=plain"))
  val firtoolOpts = Seq(FirtoolOption("-O=release")) ++ _firtoolOpts
  val firtoolOptsDebug = Seq(FirtoolOption("-O=debug")) ++ _firtoolOpts
}

object AxiBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new AxiBridge(Node(nodeType = NodeType.S, outstanding = 8))(config))
  ))
}

object AxiLiteBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new AxiLiteBridge(Node(nodeType = NodeType.HI, outstanding = 8), 64, 3)(config))
  ))
}

object TLULBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new TLULBridge(Node(nodeType = NodeType.HI, outstanding = 8), 64, 3)(config))
  ))
}

object TLUL2ChiBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new TLUL2ChiBridge(Node(nodeType = NodeType.RI, outstanding = 16), TilelinkParams(addrBits = 48, userBits = 2 /* Extra two bits for svbpmt */))(config))
  ))
}

object DmaTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new Axi2Chi(Node(nodeType = NodeType.RI))(config))
  ))
}

object MemCxTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  val cfgNode = Node(nodeType = NodeType.HI)
  val memNode = Node(nodeType = NodeType.S)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new MemoryComplex(cfgNode, memNode)(config))
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
    ChiselGeneratorAnnotation(() => new Directory()(config))
  ))
}

object DataBlockTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new DataBlock()(config))
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
    ChiselGeneratorAnnotation(() => new Frontend(0)(config))
  ))
}

object DongJiangTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new XsStage).execute(firrtlOpts, firtoolOptsDebug ++ Seq(
    ChiselGeneratorAnnotation(() => new DongJiangTop()(config.alterPartial({
      case HardwareAssertionKey => config(HardwareAssertionKey).copy(enable = false)
      case ZJParametersKey => config(ZJParametersKey).copy(
        djParamsOpt = Some(DJParam(
          llcSizeInB = 4 * 1024 / 4,
          sfSizeInB = 8 * 2 * 1024 / 4,
          nrReqTaskBuf = 4,
          nrSnpTaskBuf = 2,
          nrPoS = 64,
          dataBufSizeInByte = 8 * 32,
          nrDSBank = 2,
          nrDirBank = 2,
          llcWays = 4,
          sfWays = 4,
        ))
      )
    })))
  ))
}