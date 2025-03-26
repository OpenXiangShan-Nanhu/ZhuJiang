package xijiang.router.base

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.tfb.{FlitMonitor, NodeRegister}
import xijiang.{Node, NodeType}
import xs.utils.ResetRRArbiter
import xs.utils.debug.HardwareAssertionKey
import zhujiang.chi._
import zhujiang.{ZJBundle, ZJModule, ZJParametersKey}

import scala.collection.mutable

class ChannelBundle[T <: RingFlit](gen: T)(implicit p: Parameters) extends ZJBundle {
  val flit = Valid(gen)
  val rsvd = Valid(UInt(niw.W))
}

class RingSide(implicit p: Parameters) extends ZJBundle {
  val req = new ChannelBundle(new RingFlit(rreqFlitBits))
  val rsp = new ChannelBundle(new RingFlit(respFlitBits))
  val dat = new ChannelBundle(new RingFlit(dataFlitBits))
  val hrq = new ChannelBundle(new RingFlit(ringHrqFlitBits))
  val dbg = Option.when(p(HardwareAssertionKey).enable)(new ChannelBundle(new RingFlit(debugFlitBits)))

  def getBundle(chn: String) = {
    chn match {
      case "REQ" => req
      case "RSP" => rsp
      case "DAT" => dat
      case "HRQ" => hrq
      case "DBG" => dbg.get
    }
  }
}

class RouterRingIO(implicit p: Parameters) extends ZJBundle {
  val tx = Output(new RingSide)
  val rx = Input(new RingSide)
}

class ResetRingIO extends Bundle {
  val tx = Output(Vec(2, Bool()))
  val rx = Input(Vec(2, Bool()))
}

trait BaseRouterUtils {
  m: ZJModule =>
  def node: Node
  override val desiredName = node.routerStr
  val tfbNodeType = node.nodeType.U
  private val isMiscNode = node.nodeType == NodeType.M

  val router = IO(new Bundle {
    val rings = Vec(2, new RouterRingIO)
    val nodeId = Output(UInt(niw.W))
    val reset = new ResetRingIO
    val ci = Input(UInt(ciIdBits.W))
  })
  val icn = IO(new IcnBundle(node, true))

  private val resetReg0 = withReset(router.reset.rx(0).asAsyncReset)(RegInit(3.U(2.W)))
  private val resetReg1 = withReset(router.reset.rx(1).asAsyncReset)(RegInit(3.U(2.W)))
  if(isMiscNode) {
    router.reset.tx := icn.resetInject.get
  } else {
    router.reset.tx(0) := resetReg0(0)
    router.reset.tx(1) := resetReg1(0)
  }
  icn.resetState.get(0) := resetReg0(0)
  icn.resetState.get(1) := resetReg1(0)
  resetReg0 := Cat(false.B, resetReg0(1))
  resetReg1 := Cat(false.B, resetReg1(1))

  val nid = node.nodeId.U(niw.W)

  private val flitMap = Map[String, RingFlit](
    "REQ" -> new RingFlit(rreqFlitBits),
    "RSP" -> new RingFlit(respFlitBits),
    "DAT" -> new RingFlit(dataFlitBits),
    "SNP" -> new RingFlit(snoopFlitBits),
    "ERQ" -> new RingFlit(hreqFlitBits),
    "HRQ" -> new RingFlit(ringHrqFlitBits),
    "DBG" -> new RingFlit(debugFlitBits)
  )

  private val ejectBufSizeMap = Map[String, Int](
    "REQ" -> p(ZJParametersKey).reqEjectBufDepth,
    "RSP" -> 3,
    "DAT" -> 3,
    "HRQ" -> p(ZJParametersKey).reqEjectBufDepth,
    "DBG" -> 3
  )

  val injectsMap = node.injects.map({inj =>
    val iw = Wire(Decoupled(flitMap(inj)))
    val in = icn.rx.getBundle(inj).get
    iw.valid := in.valid
    iw.bits := in.bits.asTypeOf(iw.bits)
    in.ready := iw.ready
    if(inj == "DBG") iw.bits.TgtID := zjParams.island.filter(_.nodeType == NodeType.M).head.nodeId.U
    if(m.p(ZJParametersKey).tfsParams.isEmpty) {
      val ring = m.p(ZJParametersKey).island
      node.checkLegalInjectTarget(ring, inj, iw.bits.tgt.asTypeOf(new NodeIdBundle), iw.valid, nid)
    }
    (inj, iw)
  }).toMap

  private def getInjectBuf(chn:String): Queue[RingFlit] = {
    val buf = Module(new Queue(flitMap(chn), entries = 2))
    buf.suggestName(s"inject${chn.toLowerCase().capitalize}Buffer")
    if(chn == "HRQ") {
      if(node.injects.contains("ERQ") && node.injects.contains("SNP")) {
        val arb = Module(new ResetRRArbiter(new RingFlit(ringHrqFlitBits), 2))
        arb.io.in(0) <> injectsMap("ERQ")
        arb.io.in(1) <> injectsMap("SNP")
        buf.io.enq <> arb.io.out
      } else if(node.injects.contains("ERQ") && !node.injects.contains("SNP")) {
        buf.io.enq <> injectsMap("ERQ")
      } else if(!node.injects.contains("ERQ") && node.injects.contains("SNP")) {
        buf.io.enq <> injectsMap("SNP")
      }
    } else {
      buf.io.enq <> injectsMap(chn)
    }
    buf
  }

  def connectRing[K <: Flit](chn: String): Unit = {
    val hasRx = icn.rx.testBundle(chn)
    val hasTx = icn.tx.testBundle(chn)
    val tap = if(hasRx || hasTx) Some(Module(new ChannelTap(flitMap(chn), chn, ejectBufSizeMap(chn), node))) else None
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

    if(hasRx) {
      val buf = getInjectBuf(chn)
      val mon = if(hasTfb) Some(Module(new FlitMonitor)) else None
      tap.get.io.inject <> buf.io.deq
      val tgt = buf.io.deq.bits.tgt.asTypeOf(new NodeIdBundle)
      tap.get.io.injectTapSelOH(0) := node.rightNodes.map(_.nodeId.U === tgt.router).reduce(_ || _)
      tap.get.io.injectTapSelOH(1) := node.leftNodes.map(_.nodeId.U === tgt.router).reduce(_ || _)
      when(tap.get.io.inject.valid) {
        assert(
          PopCount(tap.get.io.injectTapSelOH) === 1.U,
          cf"Unknown routing path on $chn of node 0x${node.nodeId.toHexString}, flit tgt: 0x${buf.io.deq.bits.tgt}%x"
        )
      }

      val src = WireInit(nid.asTypeOf(new NodeIdBundle))
      src.aid := buf.io.deq.bits.src.asTypeOf(new NodeIdBundle).aid
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
    }

    if(hasTx) {
      val mon = if(hasTfb) Some(Module(new FlitMonitor)) else None
      val icnTx = icn.tx.getBundle(chn)
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
  connectRing("HRQ")
  if(p(HardwareAssertionKey).enable) connectRing("DBG")
  print(node)
}
