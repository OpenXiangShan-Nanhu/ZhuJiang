package zhujiang

import xijiang.{Node, NodeType}
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.{ChiselStage, FirtoolOption}
import xijiang.c2c.C2cLoopBack
import xijiang.router.base.{EjectBuffer, SingleChannelTap}
import zhujiang.UnitTop.firtoolOpts
import zhujiang.chi.SnoopFlit
import zhujiang.device.bridge.axi.AxiBridge
import zhujiang.device.bridge.axilite.AxiLiteBridge
import zhujiang.device.bridge.chi.ChiSnBridge
import zhujiang.device.bridge.tlul.TLULBridge
import zhujiang.device.tlu2chi.TLUL2ChiBridge
import zhujiang.device.dma.Axi2Chi
import zhujiang.device.ddr.MemoryComplex
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
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new AxiBridge(Node(nodeType = NodeType.S, splitFlit = true, outstanding = 8))(config))
  ))
}

object AxiLiteBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new AxiLiteBridge(Node(nodeType = NodeType.HI, splitFlit = true, outstanding = 8), 64, 3)(config))
  ))
}

object TLULBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new TLULBridge(Node(nodeType = NodeType.HI, splitFlit = true, outstanding = 8), 64, 3)(config))
  ))
}

object TLUL2ChiBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new TLUL2ChiBridge(Node(nodeType = NodeType.RI, splitFlit = true, outstanding = 16), TilelinkParams(addrBits = 48))(config))
  ))
}

object ChiSnBridgeTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new ChiSnBridge(Node(nodeType = NodeType.HI, splitFlit = true, outstanding = 8))(config))
  ))
}

object DmaTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new Axi2Chi(Node(nodeType = NodeType.RI, splitFlit = true))(config))
  ))
}

object MemCxTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  val cfgNode = Node(nodeType = NodeType.HI)
  val memNode = Node(nodeType = NodeType.S)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new MemoryComplex(cfgNode, memNode)(config))
  ))
}

object SctTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  val ccNode = Node(nodeType = NodeType.C)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => {
      new SingleChannelTap(new SnoopFlit()(config), "SNP", ccNode)(config)
    })
  ))
}

object EbTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => {
      new EjectBuffer(new SnoopFlit()(config), 5, "SNP")(config)
    })
  ))
}

object ClbTop extends App {
  val (config, firrtlOpts) = ZhujiangTopParser(args)
  (new ChiselStage).execute(firrtlOpts, firtoolOpts ++ Seq(
    ChiselGeneratorAnnotation(() => new C2cLoopBack()(config))
  ))
}