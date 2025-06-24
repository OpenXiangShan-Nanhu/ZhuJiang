package xijiang.router.base

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.tfb.{FlitMonitor, NodeRegister}
import xijiang.{Node, NodeType}
import xs.utils.ResetRRArbiter
import xs.utils.debug.HardwareAssertionKey
import zhujiang.chi.FlitHelper.connIcn
import zhujiang.chi._
import zhujiang.{ZJBundle, ZJModule, ZJParametersKey}

object RingEncodings {
  val encodingsMap = Map[String, Int](
    "REQ" -> 0,
    "RSP" -> 1,
    "DAT" -> 2,
    "HRQ" -> 3,
    "HPR" -> 4,
    "DBG" -> 5,
  )

  def MAX_LEGAL_RING_CHN: Int = encodingsMap("DBG")
  val allRingSeq:Seq[String] = encodingsMap.keys.toSeq
  val cppDefines:String = encodingsMap.map(elm => s"#define ${elm._1} 0x${elm._2.toHexString}\n").reduce(_ + _)
}

class ChannelBundle[T <: RingFlit](gen: T)(implicit p: Parameters) extends ZJBundle {
  val flit = Valid(gen)
  val rsvd = Valid(UInt(niw.W))
}

class RingSide(implicit p: Parameters) extends ZJBundle {
  val hpr = Option.when(hasHprRing)(new ChannelBundle(new RingFlit(rreqFlitBits)))
  val req = new ChannelBundle(new RingFlit(rreqFlitBits))
  val rsp = new ChannelBundle(new RingFlit(respFlitBits))
  val dat = new ChannelBundle(new RingFlit(dataFlitBits))
  val hrq = new ChannelBundle(new RingFlit(ringHrqFlitBits))
  val dbg = Option.when(p(HardwareAssertionKey).enable)(new ChannelBundle(new RingFlit(debugFlitBits)))

  def getRingBundle(chn: String) = {
    chn match {
      case "REQ" => req
      case "RSP" => rsp
      case "DAT" => dat
      case "HRQ" => hrq
      case "DBG" => dbg.get
      case "HPR" => hpr.get
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

object RouterHelper {
  def testRingRx(node:Node, ring:String)(implicit p:Parameters):Boolean = {
    val injects = node.injects
    ring match {
      case "REQ" => injects.contains("REQ")
      case "RSP" => injects.contains("RSP")
      case "DAT" => injects.contains("DAT")
      case "HRQ" => injects.contains("SNP") || injects.contains("ERQ")
      case "HPR" => p(ZJParametersKey).hasHprRing && injects.contains("HPR")
      case "DBG" => injects.contains("DBG")
    }
  }

  def testRingTx(node:Node, ring:String)(implicit p:Parameters):Boolean = {
    val ejects = node.ejects
    ring match {
      case "REQ" => ejects.contains("REQ")
      case "RSP" => ejects.contains("RSP")
      case "DAT" => ejects.contains("DAT")
      case "HRQ" => ejects.contains("SNP") || ejects.contains("ERQ")
      case "HPR" => p(ZJParametersKey).hasHprRing && (ejects.contains("HPR") || ejects.contains("REQ"))
      case "DBG" => ejects.contains("DBG")
    }
  }
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
    "HRQ" -> new RingFlit(ringHrqFlitBits),
    "DBG" -> new RingFlit(debugFlitBits),
    "HPR" -> new RingFlit(rreqFlitBits),
  )

  private val ejectBufSizeMap = Map[String, Int](
    "REQ" -> p(ZJParametersKey).reqEjectBufDepth,
    "RSP" -> 3,
    "DAT" -> 3,
    "HRQ" -> p(ZJParametersKey).reqEjectBufDepth,
    "DBG" -> 3,
    "HPR" -> p(ZJParametersKey).reqEjectBufDepth,
  )
  private val hasHwa = p(HardwareAssertionKey).enable
  private val ringSeq = RingEncodings.allRingSeq.filterNot(_ == "HPR" && !hasHprRing).filterNot(_ == "DBG" && !hasHwa)
  val ringInjectsMap = ringSeq.flatMap(r => Option.when(RouterHelper.testRingRx(node, r))(r -> Wire(Decoupled(flitMap(r))))).toMap
  val ringEjectsMap = ringSeq.flatMap(r => Option.when(RouterHelper.testRingTx(node, r))(r -> Wire(Decoupled(flitMap(r))))).toMap
  ringInjectsMap.foreach(_._2 := DontCare)
  ringEjectsMap.foreach(_._2 := DontCare)

  for((r, rx) <- ringInjectsMap) {
    if(r == "HRQ") {
      if(node.injects.contains("ERQ") && node.injects.contains("SNP")) {
        val rxHrqArb = Module(new ResetRRArbiter(flitMap(r), 2))
        connIcn(rxHrqArb.io.in(0), icn.rx.req.getOrElse(WireInit(0.U.asTypeOf(Decoupled(flitMap(r))))), checkWidth = false)
        connIcn(rxHrqArb.io.in(1), icn.rx.snp.getOrElse(WireInit(0.U.asTypeOf(Decoupled(flitMap(r))))), checkWidth = false)
        connIcn(rx, rxHrqArb.io.out)
      } else if(node.injects.contains("ERQ") && !node.injects.contains("SNP")) {
        connIcn(rx, icn.rx.req.get, checkWidth = false)
      } else if(!node.injects.contains("ERQ") && node.injects.contains("SNP")) {
        connIcn(rx, icn.rx.snp.get, checkWidth = false)
      }
    } else if (r == "DBG") {
      connIcn(rx, icn.rx.getRingBundle(r).get)
      rx.bits.TgtID := p(ZJParametersKey).mnid.U
    } else {
      connIcn(rx, icn.rx.getRingBundle(r).get)
    }
  }

  for((r, tx) <- ringEjectsMap) if(icn.tx.getRingBundle(r).isDefined) connIcn(icn.tx.getRingBundle(r).get, tx, checkWidth = false)
  if(node.ejects.contains("REQ") && !node.ejects.contains("HPR") && hasHprRing) {
    val txReqArb = Module(new Arbiter(flitMap("REQ"), 2))
    connIcn(txReqArb.io.in(0), ringEjectsMap("HPR"))
    connIcn(txReqArb.io.in(1), ringEjectsMap("REQ"))
    connIcn(icn.tx.req.get, txReqArb.io.out)
  }

  def connectRing[K <: Flit](chn: String): Unit = {
    val hasRx = ringInjectsMap.contains(chn)
    val hasTx = ringEjectsMap.contains(chn)
    val tap = Option.when(hasTx || hasRx)(Module(new ChannelTap(flitMap(chn), chn, ejectBufSizeMap(chn), node)))
    if(tap.isDefined) {
      tap.get.suggestName(s"${chn.toLowerCase()}ChannelTap")
      tap.get.io.matchTag := nid
      tap.get.io.rx(0) := router.rings(0).rx.getRingBundle(chn)
      tap.get.io.rx(1) := router.rings(1).rx.getRingBundle(chn)
      router.rings(0).tx.getRingBundle(chn) := tap.get.io.tx(0)
      router.rings(1).tx.getRingBundle(chn) := tap.get.io.tx(1)
      tap.get.io.inject.valid := false.B
      tap.get.io.inject.bits := DontCare
      tap.get.io.eject.ready := false.B
      tap.get.io.injectTapSelOH := DontCare
    } else {
      router.rings.foreach(r => {
        r.tx.getRingBundle(chn).flit := Pipe(r.rx.getRingBundle(chn).flit)
        r.tx.getRingBundle(chn).rsvd := Pipe(r.rx.getRingBundle(chn).rsvd)
      })
    }

    if(hasRx) {
      val mon = if(hasTfb) Some(Module(new FlitMonitor)) else None
      val buf = Module(new Queue(flitMap(chn), 2))
      connIcn(buf.io.enq, ringInjectsMap(chn))
      connIcn(tap.get.io.inject, buf.io.deq)
      buf.suggestName(s"inj_buf_${chn.toLowerCase}")
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
        m.suggestName(s"cosim_inject${chn.toLowerCase()}_mon")
        m.io.clock := clock
        m.io.valid := tap.get.io.inject.fire
        m.io.nodeId := nid
        m.io.nodeType := tfbNodeType
        m.io.inject := true.B
        m.io.flitType := RingEncodings.encodingsMap(chn).U
        m.io.flit := tap.get.io.inject.bits.asUInt
        when(m.io.valid) {
          assert(!m.io.fault, s"channel $chn inject wrong flit!")
        }
      })
    }

    if(hasTx) {
      val mon = if(hasTfb) Some(Module(new FlitMonitor)) else None
      connIcn(ringEjectsMap(chn), tap.get.io.eject)
      mon.foreach(m => {
        m.suggestName(s"cosim_eject_${chn.toLowerCase()}_mon")
        m.io.clock := clock
        m.io.valid := tap.get.io.eject.fire
        m.io.nodeId := nid
        m.io.nodeType := tfbNodeType
        m.io.inject := false.B
        m.io.flitType := RingEncodings.encodingsMap(chn).U
        m.io.flit := tap.get.io.eject.bits.asUInt
        when(m.io.valid) {
          assert(!m.io.fault, s"channel $chn ejects wrong flit!")
        }
      })
    }
  }
  ringSeq.foreach(connectRing)
}

class BaseRouter(val node: Node)(implicit p: Parameters) extends ZJModule with BaseRouterUtils {
  router.nodeId := nid
  dontTouch(router.nodeId)
  private val tfbNodeRegister = if((node.injects ++ node.ejects).nonEmpty && hasTfb) Some(Module(new NodeRegister)) else None
  tfbNodeRegister.foreach(r => {
    r.suggestName("cosim_node_register")
    r.io.nodeId := nid
    r.io.nodeType := tfbNodeType
  })
  print(node)
}
