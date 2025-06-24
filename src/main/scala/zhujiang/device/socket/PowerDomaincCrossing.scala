package zhujiang.device.socket

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}
import zhujiang.{ZJBundle, ZJModule}

class PowerDomainCrossingBundle[T <: Data](gen:T) extends Bundle {
  val valid = Output(Bool())
  val bits = Output(gen)
  val grant = Input(Bool())
}

object PowerDomainCrossing {
  val tokens = 5
}

class PowerDomainCrossingTx[T <: Data](gen:T) extends Module {
  val io = IO(new Bundle {
    val enq = Flipped(Decoupled(gen))
    val pdc = new PowerDomainCrossingBundle(gen)
    val clean = Output(Bool())
  })
  private val enqFire = io.enq.fire
  private val tokens = RegInit(PowerDomainCrossing.tokens.U(log2Ceil(PowerDomainCrossing.tokens + 1).W))
  private val rxg = RegNext(io.pdc.grant, false.B)
  private val txv = RegNext(enqFire, false.B)
  private val txd = RegEnable(io.enq.bits, enqFire)
  io.pdc.valid := txv
  io.pdc.bits := txd
  io.enq.ready := tokens.orR
  io.clean := tokens === PowerDomainCrossing.tokens.U

  when(enqFire && !rxg) {
    tokens := tokens - 1.U
  }.elsewhen(!enqFire && rxg) {
    tokens := tokens + 1.U
  }
  assert(tokens <= PowerDomainCrossing.tokens.U)
}

class PowerDomainCrossingRx[T <: Data](gen:T) extends Module {
  val io = IO(new Bundle {
    val pdc = Flipped(new PowerDomainCrossingBundle(gen))
    val deq = Decoupled(gen)
    val clean = Output(Bool())
  })
  private val rxq = Module(new Queue(gen = gen, entries = PowerDomainCrossing.tokens, flow = true))
  private val rxv = RegNext(io.pdc.valid, false.B)
  private val rxd = RegNext(io.pdc.bits)
  private val txg = RegNext(io.deq.fire, false.B)
  rxq.io.enq.valid := rxv
  rxq.io.enq.bits := rxd
  io.pdc.grant := txg
  io.deq <> rxq.io.deq
  io.clean := rxq.io.count === 0.U
  when(rxv) {
    assert(rxq.io.enq.ready)
  }
}

abstract class PdcBundle(implicit p: Parameters) extends ZJBundle {
  def req: Option[PowerDomainCrossingBundle[Data]]
  def rsp: Option[PowerDomainCrossingBundle[Data]]
  def dat: Option[PowerDomainCrossingBundle[Data]]
  def snp: Option[PowerDomainCrossingBundle[Data]]
  def hpr: Option[PowerDomainCrossingBundle[Data]]
  def dbg: Option[PowerDomainCrossingBundle[Data]]
  private lazy val _bundleMap = Seq(
    "REQ" -> req,
    "RSP" -> rsp,
    "DAT" -> dat,
    "SNP" -> snp,
    "HPR" -> hpr,
    "DBG" -> dbg,
    "ERQ" -> req
  )
  lazy val bundleMap = _bundleMap.flatMap(elm => Option.when(elm._2.isDefined)(elm._1, elm._2.get)).toMap
}

class ChiPdcTxBundle(node: Node)(implicit p: Parameters) extends PdcBundle {
  val req = if(node.ejects.contains("REQ")) {
    Some(new PowerDomainCrossingBundle(UInt(rreqFlitBits.W)))
  } else if(node.ejects.contains("ERQ")) {
    Some(new PowerDomainCrossingBundle(UInt(hreqFlitBits.W)))
  } else {
    None
  }

  val hpr = Option.when(node.ejects.contains("HPR") && hasHprRing)(new PowerDomainCrossingBundle(UInt(rreqFlitBits.W)))
  val rsp = Option.when(node.ejects.contains("RSP"))(new PowerDomainCrossingBundle(UInt(respFlitBits.W)))
  val dat = Option.when(node.ejects.contains("DAT"))(new PowerDomainCrossingBundle(UInt(dataFlitBits.W)))
  val snp = Option.when(node.ejects.contains("SNP"))(new PowerDomainCrossingBundle(UInt(snoopFlitBits.W)))
  val dbg = Option.when(node.ejects.contains("DBG"))(new PowerDomainCrossingBundle(UInt(debugFlitBits.W)))
}

class ChiPdcRxBundle(node: Node)(implicit p: Parameters) extends PdcBundle {
  val req = if(node.injects.contains("REQ")) {
    Some(Flipped(new PowerDomainCrossingBundle(UInt(rreqFlitBits.W))))
  } else if(node.injects.contains("ERQ")) {
    Some(Flipped(new PowerDomainCrossingBundle(UInt(hreqFlitBits.W))))
  } else {
    None
  }

  val hpr = Option.when(node.injects.contains("HPR") && hasHprRing)(Flipped(new PowerDomainCrossingBundle(UInt(rreqFlitBits.W))))
  val rsp = Option.when(node.injects.contains("RSP"))(Flipped(new PowerDomainCrossingBundle(UInt(respFlitBits.W))))
  val dat = Option.when(node.injects.contains("DAT"))(Flipped(new PowerDomainCrossingBundle(UInt(dataFlitBits.W))))
  val snp = Option.when(node.injects.contains("SNP"))(Flipped(new PowerDomainCrossingBundle(UInt(snoopFlitBits.W))))
  val dbg = Option.when(node.injects.contains("DBG"))(Flipped(new PowerDomainCrossingBundle(UInt(debugFlitBits.W))))
}

class IcnPdcBundle(node:Node)(implicit p:Parameters) extends ZJBundle {
  val tx = new ChiPdcTxBundle(node)
  val rx = new ChiPdcRxBundle(node)
  def <>(that:DevPdcBundle):Unit = {
    this.tx <> that.rx
    this.rx <> that.tx
  }

}

class DevPdcBundle(node:Node)(implicit p:Parameters) extends ZJBundle {
  val tx = Flipped(new ChiPdcRxBundle(node))
  val rx = Flipped(new ChiPdcTxBundle(node))
  def <>(that:IcnPdcBundle):Unit = {
    this.tx <> that.rx
    this.rx <> that.tx
  }
}

trait PdcConnHelper {
  def syncToPdc(tx: PowerDomainCrossingBundle[Data], rx: DecoupledIO[Data], chn: String) = {
    val pdc = Module(new PowerDomainCrossingTx(tx.bits.cloneType))
    pdc.io.enq.valid := rx.valid
    pdc.io.enq.bits := rx.bits.asTypeOf(pdc.io.enq.bits)
    rx.ready := pdc.io.enq.ready
    tx <> pdc.io.pdc
    pdc
  }

  def pdcToSync(tx: DecoupledIO[Data], rx: PowerDomainCrossingBundle[Data], chn: String) = {
    val pdc = Module(new PowerDomainCrossingRx(rx.bits.cloneType))
    pdc.io.pdc <> rx
    tx.valid := pdc.io.deq.valid
    tx.bits := pdc.io.deq.bits.asTypeOf(tx.bits)
    pdc.io.deq.ready := tx.ready
    pdc
  }
}

class ChiPdcIcnSide(node: Node)(implicit p: Parameters) extends ZJModule with PdcConnHelper {
  val io = IO(new Bundle {
    val icn = new IcnPdcBundle(node)
    val dev = new DeviceIcnBundle(node)
    val clean = Output(Bool())
  })

  private val ejCleanVec = Wire(Vec(node.ejects.size, Bool()))
  for((chn, idx) <- node.ejects.zipWithIndex) {
    val rx = io.dev.rx.bundleMap(chn)
    val tx = io.icn.tx.bundleMap(chn)
    val pdc = syncToPdc(tx, rx, chn)
    pdc.suggestName(s"${chn.toLowerCase}_pdx_tx")
    ejCleanVec(idx) := pdc.io.clean
  }

  private val ijCleanVec = Wire(Vec(node.injects.size, Bool()))
  for((chn, idx) <- node.injects.zipWithIndex) {
    val tx = io.dev.tx.bundleMap(chn)
    val rx = io.icn.rx.bundleMap(chn)
    val pdc = pdcToSync(tx, rx, chn)
    pdc.suggestName(s"${chn.toLowerCase}_pdx_rx")
    ijCleanVec(idx) := pdc.io.clean
  }

  io.clean := RegNext(Cat(ejCleanVec ++ ijCleanVec).andR)
}

class ChiPdcDevSide(node:Node)(implicit p:Parameters) extends ZJModule with PdcConnHelper {
  val io = IO(new Bundle {
    val icn = new IcnBundle(node)
    val dev = new DevPdcBundle(node)
    val clean = Output(Bool())
  })

  private val ejCleanVec = Wire(Vec(node.ejects.size, Bool()))
  for((chn, idx) <- node.ejects.zipWithIndex) {
    val rx = io.dev.rx.bundleMap(chn)
    val tx = io.icn.tx.bundleMap(chn)
    val pdc = pdcToSync(tx, rx, chn)
    pdc.suggestName(s"${chn.toLowerCase}_pdx_rx")
    ejCleanVec(idx) := pdc.io.clean
  }

  private val ijCleanVec = Wire(Vec(node.injects.size, Bool()))
  for((chn, idx) <- node.injects.zipWithIndex) {
    val tx = io.dev.tx.bundleMap(chn)
    val rx = io.icn.rx.bundleMap(chn)
    val pdc = syncToPdc(tx, rx, chn)
    pdc.suggestName(s"${chn.toLowerCase}_pdx_tx")
    ijCleanVec(idx) := pdc.io.clean
  }

  io.clean := RegNext(Cat(ejCleanVec ++ ijCleanVec).andR)
}