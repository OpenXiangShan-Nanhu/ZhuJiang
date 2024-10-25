package xijiang.router.base

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.tfb.{FlitMonitor, NodeRegister}
import xijiang.{Node, NodeType}
import zhujiang.chi._
import zhujiang.{ZJBundle, ZJModule, ZJParametersKey}

import scala.collection.mutable

class ChannelBundle[T <: Flit](gen: T)(implicit p: Parameters) extends ZJBundle {
  val flit = Valid(gen)
  val rsvd = Valid(UInt(niw.W))
}

class RingSide(local: Boolean)(implicit p: Parameters) extends ZJBundle {
  val req = new ChannelBundle(new ReqFlit)
  val resp = new ChannelBundle(new RespFlit)
  val data = new ChannelBundle(new DataFlit)
  val snoop = new ChannelBundle(new SnoopFlit)
  val ereq = if(local) Some(new ChannelBundle(new ReqFlit)) else None

  def getBundle(chn: String) = {
    chn match {
      case "REQ" => req
      case "RSP" => resp
      case "DAT" => data
      case "SNP" => snoop
      case "ERQ" => ereq.get
    }
  }
}

class RouterRingIO(csn: Boolean)(implicit p: Parameters) extends ZJBundle {
  val tx = Output(new RingSide(csn))
  val rx = Input(new RingSide(csn))
}

class ResetRingIO extends Bundle {
  val tx = Output(Vec(2, Bool()))
  val rx = Input(Vec(2, Bool()))
}

trait BaseRouterUtils {
  m: ZJModule =>
  def node: Node
  override val desiredName = node.routerStr
  val tfbNodeType = if(node.csnNode) ((1 << NodeType.width) | node.nodeType).U else node.nodeType.U
  private val isDefaultHi = node.nodeType == NodeType.HI && node.defaultHni

  val local = !node.csnNode
  val c2c = node.csnNode && node.nodeType == NodeType.C
  private val csnRing = m.p(ZJParametersKey).csnRing
  private val csnC2cNum = csnRing.count(_.nodeType == NodeType.C)

  val router = IO(new Bundle {
    val rings = Vec(2, new RouterRingIO(local))
    val chip = Input(UInt(p(ZJParametersKey).nodeAidBits.W))
    val nodeId = Output(UInt(niw.W))
    val c2cIds = if(!local && !c2c && node.nodeType != NodeType.P) Some(Input(Vec(csnC2cNum, new NodeIdBundle))) else None
    val reset = new ResetRingIO
  })
  val icn = IO(new IcnBundle(node, true))

  private val resetReg0 = withReset(router.reset.rx(0).asAsyncReset)(RegInit(3.U(2.W)))
  private val resetReg1 = withReset(router.reset.rx(1).asAsyncReset)(RegInit(3.U(2.W)))
  if(isDefaultHi) {
    router.reset.tx := icn.resetInject.get
  } else {
    router.reset.tx(0) := resetReg0(0)
    router.reset.tx(1) := resetReg1(0)
  }
  icn.resetState.get(0) := resetReg0(0)
  icn.resetState.get(1) := resetReg1(0)
  resetReg0 := Cat(false.B, resetReg0(1))
  resetReg1 := Cat(false.B, resetReg1(1))

  val nid = if(local) node.nodeId.U(niw.W) else node.nodeId.U(niw.W) | router.chip

  private val flitMap = Map[String, Flit](
    "REQ" -> new ReqFlit,
    "RSP" -> new RespFlit,
    "DAT" -> new DataFlit,
    "SNP" -> new SnoopFlit,
    "ERQ" -> new ReqFlit
  )
  val injectsMap = mutable.Map[String, DecoupledIO[Flit]]()

  private val ejectBufSizeMap = Map[String, Int](
    "REQ" -> p(ZJParametersKey).reqEjectBufDepth,
    "RSP" -> 0,
    "DAT" -> 0,
    "SNP" -> p(ZJParametersKey).snoopEjectBufDepth,
    "ERQ" -> p(ZJParametersKey).reqEjectBufDepth
  )

  def connectRing[K <: Flit](chn: String): Unit = {
    val icnRx = icn.rx.getBundle(chn)
    val icnTx = icn.tx.getBundle(chn)
    val tap = if(icnRx.isDefined || icnTx.isDefined) Some(Module(new ChannelTap(flitMap(chn), chn, ejectBufSizeMap(chn), node))) else None
    if(tap.isDefined) {
      tap.get.suggestName(s"${chn.toLowerCase()}ChannelTap")
      tap.get.io.matchTag := nid
      tap.get.io.rx(0) := router.rings(0).rx.getBundle(chn)
      tap.get.io.rx(1) := router.rings(1).rx.getBundle(chn)
      router.rings(0).tx.getBundle(chn) := tap.get.io.tx(0)
      router.rings(1).tx.getBundle(chn) := tap.get.io.tx(1)
      tap.get.io.inject.valid := false.B
      tap.get.io.inject.bits := DontCare
      tap.get.io.eject.ready := false.B
      tap.get.io.injectTapSelOH := DontCare
    } else {
      router.rings.foreach(r => {
        r.tx.getBundle(chn).flit := Pipe(r.rx.getBundle(chn).flit)
        r.tx.getBundle(chn).rsvd := Pipe(r.rx.getBundle(chn).rsvd)
      })
    }

    if(icnRx.isDefined) {
      val buf = Module(new Queue(flitMap(chn), 2))
      injectsMap.addOne(chn -> buf.io.enq)
      val mon = if(hasTfb) Some(Module(new FlitMonitor)) else None
      buf.suggestName(s"inject${chn.toLowerCase().capitalize}Buffer")
      buf.io.enq.valid := icnRx.get.valid
      buf.io.enq.bits := icnRx.get.bits.asTypeOf(flitMap(chn))
      icnRx.get.ready := buf.io.enq.ready
      tap.get.io.inject <> buf.io.deq
      val tgt = buf.io.deq.bits.tgt.asTypeOf(new NodeIdBundle)
      val routerTgt = Wire(UInt(niw.W))
      dontTouch(routerTgt)
      routerTgt.suggestName(s"routerTgt$chn")
      if(local) {
        routerTgt := Cat(tgt.net, tgt.nid, 0.U(nodeAidBits.W))
      } else if(c2c) {
        routerTgt := tgt.router
      } else {
        val c2cSelOH = router.c2cIds.get.map(_.chip === tgt.chip)
        val c2cRouters = router.c2cIds.get.map(_.router)
        when(buf.io.deq.valid) {
          assert(PopCount(c2cSelOH) === 1.U, cf"Illegal chip id on $chn of node 0x${node.nodeId.toHexString}, flit tgt chip: ${tgt.chip}%x")
        }
        routerTgt := Mux1H(c2cSelOH, c2cRouters)
      }

      tap.get.io.injectTapSelOH(0) := node.rightNodes.map(_.nodeId.U === routerTgt).reduce(_ || _)
      tap.get.io.injectTapSelOH(1) := node.leftNodes.map(_.nodeId.U === routerTgt).reduce(_ || _)
      when(tap.get.io.inject.valid) {
        assert(PopCount(tap.get.io.injectTapSelOH) === 1.U, cf"Unknown routing path on $chn of node 0x${node.nodeId.toHexString}, flit tgt: 0x${buf.io.deq.bits.tgt}%x")
      }

      val src = WireInit(nid.asTypeOf(new NodeIdBundle))
      if(local) {
        src.aid := buf.io.deq.bits.src(nodeAidBits - 1, 0)
      } else if(c2c) {
        src := buf.io.deq.bits.src.asTypeOf(src)
      } else {
        src.aid := router.chip
      }
      tap.get.io.inject.bits.src := src.asUInt

      mon.foreach(m => {
        m.suggestName(s"inject${chn.toLowerCase().capitalize}Monitor")
        m.io.clock := clock
        m.io.valid := tap.get.io.inject.fire
        m.io.nodeId := nid
        m.io.nodeType := tfbNodeType
        m.io.inject := true.B
        m.io.flitType := ChannelEncodings.encodingsMap(chn).U
        m.io.flit := tap.get.io.inject.bits.asUInt
        when(m.io.valid) {
          assert(!m.io.fault, s"channel $chn inject wrong flit!")
        }
      })
      if(m.p(ZJParametersKey).tfsParams.isEmpty && (local || c2c)) {
        val ring = if(local) m.p(ZJParametersKey).localRing else m.p(ZJParametersKey).csnRing
        node.checkLegalInjectTarget(ring, chn, buf.io.deq.bits.tgt.asTypeOf(new NodeIdBundle), buf.io.deq.valid, nid)
      }
    }

    if(icnTx.isDefined) {
      val mon = if(hasTfb) Some(Module(new FlitMonitor)) else None
      icnTx.get.valid := tap.get.io.eject.valid
      icnTx.get.bits := tap.get.io.eject.bits.asTypeOf(icnTx.get.bits)
      tap.get.io.eject.ready := icnTx.get.ready
      mon.foreach(m => {
        m.suggestName(s"eject${chn.toLowerCase().capitalize}Monitor")
        m.io.clock := clock
        m.io.valid := tap.get.io.eject.fire
        m.io.nodeId := nid
        m.io.nodeType := tfbNodeType
        m.io.inject := false.B
        m.io.flitType := ChannelEncodings.encodingsMap(chn).U
        m.io.flit := tap.get.io.eject.bits.asUInt
        when(m.io.valid) {
          assert(!m.io.fault, s"channel $chn ejects wrong flit!")
        }
      })
    }
  }
}

class BaseRouter(val node: Node)(implicit p: Parameters) extends ZJModule with BaseRouterUtils {
  router.nodeId := nid
  dontTouch(router.nodeId)
  private val tfbNodeRegister = if((node.injects ++ node.ejects).nonEmpty && hasTfb) Some(Module(new NodeRegister)) else None
  tfbNodeRegister.foreach(r => {
    r.io.nodeId := nid
    r.io.nodeType := tfbNodeType
  })

  connectRing("REQ")
  connectRing("RSP")
  connectRing("DAT")
  connectRing("SNP")
  if(local) connectRing("ERQ")
  print(node)
}
