package zhujiang.device.socket

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}
import xs.utils.debug.HardwareAssertionKey
import xs.utils.queue.FastQueue
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

class ChiPdcTxBundle(node:Node)(implicit p:Parameters) extends ZJBundle {
  val req = if(node.ejects.contains("REQ")) {
    Some(new PowerDomainCrossingBundle(UInt(rreqFlitBits.W)))
  } else if(node.ejects.contains("ERQ")) {
    Some(new PowerDomainCrossingBundle(UInt(hreqFlitBits.W)))
  } else {
    None
  }

  val rsp = Option.when(node.ejects.contains("RSP"))(new PowerDomainCrossingBundle(UInt(respFlitBits.W)))
  val dat = Option.when(node.ejects.contains("DAT"))(new PowerDomainCrossingBundle(UInt(dataFlitBits.W)))
  val snp = Option.when(node.ejects.contains("SNP"))(new PowerDomainCrossingBundle(UInt(snoopFlitBits.W)))
  val dbg = Option.when(node.ejects.contains("DBG") && p(HardwareAssertionKey).enable)(new PowerDomainCrossingBundle(UInt(debugFlitBits.W)))

  def getBundle(chn:String): PowerDomainCrossingBundle[UInt] = {
    chn match {
      case "REQ" => req.get
      case "RSP" => rsp.get
      case "DAT" => dat.get
      case "SNP" => snp.get
      case "ERQ" => req.get
      case "DBG" => dbg.get
    }
  }
}

class ChiPdcRxBundle(node:Node)(implicit p:Parameters) extends ZJBundle {
  val req = if(node.injects.contains("REQ")) {
    Some(Flipped(new PowerDomainCrossingBundle(UInt(rreqFlitBits.W))))
  } else if(node.injects.contains("ERQ")) {
    Some(Flipped(new PowerDomainCrossingBundle(UInt(hreqFlitBits.W))))
  } else {
    None
  }

  val rsp = Option.when(node.injects.contains("RSP"))(Flipped(new PowerDomainCrossingBundle(UInt(respFlitBits.W))))
  val dat = Option.when(node.injects.contains("DAT"))(Flipped(new PowerDomainCrossingBundle(UInt(dataFlitBits.W))))
  val snp = Option.when(node.injects.contains("SNP"))(Flipped(new PowerDomainCrossingBundle(UInt(snoopFlitBits.W))))
  val dbg = Option.when(node.injects.contains("DBG") && p(HardwareAssertionKey).enable)(Flipped(new PowerDomainCrossingBundle(UInt(debugFlitBits.W))))

  def getBundle(chn:String): PowerDomainCrossingBundle[UInt] = {
    chn match {
      case "REQ" => req.get
      case "RSP" => rsp.get
      case "DAT" => dat.get
      case "SNP" => snp.get
      case "ERQ" => req.get
      case "DBG" => dbg.get
    }
  }
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
  def syncToPdc(tx:PowerDomainCrossingBundle[UInt], rx:DecoupledIO[Data], chn:String) = {
    val pdc = Module(new PowerDomainCrossingTx(tx.bits.cloneType))
    pdc.io.enq.valid := rx.valid
    pdc.io.enq.bits := rx.bits.asTypeOf(pdc.io.enq.bits)
    rx.ready := pdc.io.enq.ready
    tx <> pdc.io.pdc
    pdc
  }

  def pdcToSync(tx:DecoupledIO[Data], rx:PowerDomainCrossingBundle[UInt], chn:String) = {
    val pdc = Module(new PowerDomainCrossingRx(rx.bits.cloneType))
    pdc.io.pdc <> rx
    tx.valid := pdc.io.deq.valid
    tx.bits := pdc.io.deq.bits.asTypeOf(tx.bits)
    pdc.io.deq.ready := tx.ready
    pdc
  }
}

class ChiPdcIcnSide(node:Node)(implicit p:Parameters) extends ZJModule with PdcConnHelper {
  val io = IO(new Bundle {
    val icn = new IcnPdcBundle(node)
    val dev = new DeviceIcnBundle(node)
    val clean = Output(Bool())
  })

  private val ejCleanVec = Wire(Vec(node.ejects.size, Bool()))
  for((chn, idx) <- node.ejects.zipWithIndex) {
    val rx = io.dev.rx.getBundle(chn).get
    val tx = io.icn.tx.getBundle(chn)
    val pdc = syncToPdc(tx, rx, chn)
    pdc.suggestName(s"${chn.toLowerCase}_pwr_dmn_x_tx")
    ejCleanVec(idx) := pdc.io.clean
  }

  private val ijCleanVec = Wire(Vec(node.injects.size, Bool()))
  for((chn, idx) <- node.injects.zipWithIndex) {
    val tx = io.dev.tx.getBundle(chn).get
    val rx = io.icn.rx.getBundle(chn)
    val pdc = pdcToSync(tx, rx, chn)
    pdc.suggestName(s"${chn.toLowerCase}_pwr_dmn_x_rx")
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
    val rx = io.dev.rx.getBundle(chn)
    val tx = io.icn.tx.getBundle(chn).get
    val pdc = pdcToSync(tx, rx, chn)
    pdc.suggestName(s"${chn.toLowerCase}_pwr_dmn_x_rx")
    ejCleanVec(idx) := pdc.io.clean
  }

  private val ijCleanVec = Wire(Vec(node.injects.size, Bool()))
  for((chn, idx) <- node.injects.zipWithIndex) {
    val tx = io.dev.tx.getBundle(chn)
    val rx = io.icn.rx.getBundle(chn).get
    val pdc = syncToPdc(tx, rx, chn)
    pdc.suggestName(s"${chn.toLowerCase}_pwr_dmn_x_tx")
    ijCleanVec(idx) := pdc.io.clean
  }

  io.clean := RegNext(Cat(ejCleanVec ++ ijCleanVec).andR)
}