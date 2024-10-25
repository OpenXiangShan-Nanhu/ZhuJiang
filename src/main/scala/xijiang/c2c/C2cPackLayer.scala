package xijiang.c2c

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import xijiang.router.base.{IcnBundle, IcnRxBundle, IcnTxBundle}
import xs.utils.ResetRRArbiter
import zhujiang.chi._
import zhujiang.{ZJBundle, ZJModule, ZJParametersKey}

trait C2cUtils {
  m: ZJModule =>
  val c2cParams = p(ZJParametersKey).c2cParams
  val flitBitsMap = Map[String, Int](
    "REQ" -> reqFlitBits,
    "RSP" -> respFlitBits,
    "DAT" -> dataFlitBits,
    "SNP" -> snoopFlitBits
  )
  val txBufSizeMap = Map[String, Int](
    "REQ" -> c2cParams.reqBufTxDepth,
    "RSP" -> c2cParams.respBufTxDepth,
    "DAT" -> c2cParams.dataBufTxDepth,
    "SNP" -> c2cParams.snoopBufTxDepth,
  )
  val rxBufSizeMap = Map[String, Int](
    "REQ" -> c2cParams.reqBufRxDepth,
    "RSP" -> c2cParams.respBufRxDepth,
    "DAT" -> c2cParams.dataBufRxDepth,
    "SNP" -> c2cParams.snoopBufRxDepth,
  )
}

class C2cPayload(implicit p: Parameters) extends ZJBundle {
  val flit = UInt(maxFlitBits.W)
  val cid = UInt(2.W)
}

class C2cRingPort(node: Node)(implicit p: Parameters) extends ZJBundle {
  val icn = Flipped(new IcnBundle(node))
  val chip = Output(UInt(nodeAidBits.W))
}

class C2cLinkPort(implicit p: Parameters) extends ZJBundle {
  val tx = Decoupled(new C2cPayload)
  val rx = Flipped(Decoupled(new C2cPayload))
  val chip = Input(UInt(nodeAidBits.W))
}

class C2cPackLayerTx(node: Node)(implicit p: Parameters) extends ZJModule with C2cUtils {
  val io = IO(new Bundle {
    val fromRing = Flipped(new IcnTxBundle(node))
    val toLink = Decoupled(new C2cPayload)
  })
  private val arb = Module(new ResetRRArbiter(UInt(maxFlitBits.W), node.ejects.size))

  private def connRing(chn: String): Unit = {
    val icnPort = io.fromRing.getBundle(chn)
    if(icnPort.isDefined) {
      val txBuf = Module(new Queue(UInt(flitBitsMap(chn).W), txBufSizeMap(chn)))
      txBuf.suggestName(s"${chn.toLowerCase}TxBuffer")
      txBuf.io.enq.valid := icnPort.get.valid
      txBuf.io.enq.bits := icnPort.get.bits.asUInt
      icnPort.get.ready := txBuf.io.enq.ready
      arb.io.in(ChannelEncodings.encodingsMap(chn)) <> txBuf.io.deq
    }
  }
  node.ejects.foreach(connRing)
  io.toLink.valid := arb.io.out.valid
  io.toLink.bits.flit := arb.io.out.bits
  io.toLink.bits.cid := arb.io.chosen
  arb.io.out.ready := io.toLink.ready
}

class C2cPackLayerRx(node: Node)(implicit p: Parameters) extends ZJModule with C2cUtils {
  val io = IO(new Bundle {
    val toRing = Flipped(new IcnRxBundle(node))
    val fromLink = Flipped(Decoupled(new C2cPayload))
  })
  private def connRing(chn: String): Bool = {
    val ready = WireInit(false.B)
    val icnPort = io.toRing.getBundle(chn)
    if(icnPort.isDefined) {
      val rxBuf = Module(new Queue(UInt(flitBitsMap(chn).W), rxBufSizeMap(chn)))
      rxBuf.suggestName(s"${chn.toLowerCase}RxBuffer")
      icnPort.get.valid := rxBuf.io.deq.valid
      icnPort.get.bits := rxBuf.io.deq.bits.asTypeOf(icnPort.get.bits)
      rxBuf.io.deq.ready := icnPort.get.ready
      val enqHit = ChannelEncodings.encodingsMap(chn).U === io.fromLink.bits.cid
      rxBuf.io.enq.valid := io.fromLink.valid && enqHit
      rxBuf.io.enq.bits := io.fromLink.bits.flit(flitBitsMap(chn) - 1, 0)
      ready := rxBuf.io.enq.ready && enqHit
    }
    ready
  }
  private val readies = node.injects.map(connRing)
  io.fromLink.ready := Cat(readies).orR
}

class C2cPackLayer(node: Node)(implicit p: Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val local = new C2cRingPort(node)
    val remote = new C2cLinkPort
  })
  private val transmitter = Module(new C2cPackLayerTx(node))
  private val receiver = Module(new C2cPackLayerRx(node))
  private val txBufs = Seq.fill(p(ZJParametersKey).c2cParams.externalBuffers)(Module(new Queue(new C2cPayload, 2)))
  private val rxBufs = Seq.fill(p(ZJParametersKey).c2cParams.externalBuffers)(Module(new Queue(new C2cPayload, 2)))

  private val txDst = txBufs.map(_.io.enq) ++ Seq(io.remote.tx)
  private val txSrc = Seq(transmitter.io.toLink) ++ txBufs.map(_.io.deq)
  private val rxDst = Seq(receiver.io.fromLink) ++ rxBufs.map(_.io.enq)
  private val rxSrc = rxBufs.map(_.io.deq) ++ Seq(io.remote.rx)
  for((dst, src) <- (txDst ++ rxDst).zip(txSrc ++ rxSrc)) dst <> src

  transmitter.io.fromRing <> io.local.icn.tx
  io.remote.tx <> transmitter.io.toLink
  receiver.io.fromLink <> io.remote.rx
  io.local.icn.rx <> receiver.io.toRing
  io.local.chip := io.remote.chip
}