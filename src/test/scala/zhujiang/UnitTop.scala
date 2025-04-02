package zhujiang

import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.FirtoolOption
import xijiang.c2c.C2cLoopBack
import xijiang.router.base.{EjectBuffer, SingleChannelTap}
import xijiang.{Node, NodeType}
import xs.utils.stage.XsStage
import zhujiang.UnitTop.firtoolOpts
import zhujiang.chi.{RingFlit, SnoopFlit}
import zhujiang.device.bridge.axi.AxiBridge
import zhujiang.device.bridge.axilite.AxiLiteBridge
import zhujiang.device.bridge.tlul.TLULBridge
import zhujiang.device.ddr.MemoryComplex
import zhujiang.device.dma.Axi2Chi
import zhujiang.device.home.HomeWrapper
import zhujiang.device.tlu2chi.TLUL2ChiBridge
import zhujiang.tilelink.TilelinkParams

object UnitTop {
  val firtoolOpts = Seq(
    FirtoolOption("-O=release"),
    FirtoolOption("--export-module-hierarchy"),
    FirtoolOption("--disable-all-randomization"),
    FirtoolOption("--disable-annotation-unknown"),
    FirtoolOption("--strip-debug-info"),
    FirtoolOption("--lower-memories"),
    FirtoolOption("--add-vivado-ram-address-conflict-synthesis-bug-workaround"),
    FirtoolOption("--lowering-options=noAlwaysComb," +
      " disallowLocalVariables, disallowMuxInlining," +
      " emittedLineLength=120, explicitBitcast, locationInfoStyle=plain"))
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