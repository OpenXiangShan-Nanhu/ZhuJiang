package zhujiang.device.async

import chisel3._
import chisel3.util.{Decoupled, DecoupledIO}
import freechips.rocketchip.util.{AsyncBundle, AsyncQueueParams, AsyncQueueSink, AsyncQueueSource}
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.{BaseIcnMonoBundle, DeviceIcnBundle, IcnBundle}
import zhujiang.chi.{DataFlit, Flit, ReqFlit, RespFlit, SnoopFlit}
import zhujiang.{ZJBundle, ZJModule}

import scala.collection.immutable.SeqMap

object AsyncUtils {
  val params = AsyncQueueParams(depth = 8)
}

class AsyncSink[T <: Data](gen: T) extends AsyncQueueSink(gen, AsyncUtils.params)

class AsyncSource[T <: Data](gen: T) extends AsyncQueueSource(gen, AsyncUtils.params)

trait BaseAsyncIcnMonoBundle {
  def req: Option[AsyncBundle[UInt]]
  def resp: Option[AsyncBundle[UInt]]
  def data: Option[AsyncBundle[UInt]]
  def snoop: Option[AsyncBundle[UInt]]
  def chnMap: Map[String, Option[AsyncBundle[UInt]]] = Map(
    ("REQ", req),
    ("RSP", resp),
    ("DAT", data),
    ("SNP", snoop),
    ("ERQ", req)
  )
}

class IcnTxAsyncBundle(node: Node)(implicit p: Parameters) extends ZJBundle with BaseAsyncIcnMonoBundle {
  private val illegal = node.ejects.contains("REQ") && node.ejects.contains("ERQ")
  require(!illegal)
  val req = if(node.ejects.contains("REQ") || node.ejects.contains("ERQ") && !node.csnNode) {
    Some(new AsyncBundle(UInt(reqFlitBits.W), AsyncUtils.params))
  } else None

  val resp = if(node.ejects.contains("RSP")) {
    Some(new AsyncBundle(UInt(respFlitBits.W), AsyncUtils.params))
  } else None

  val data = if(node.ejects.contains("DAT")) {
    Some(new AsyncBundle(UInt(dataFlitBits.W), AsyncUtils.params))
  } else None

  val snoop = if(node.ejects.contains("SNP")) {
    Some(new AsyncBundle(UInt(snoopFlitBits.W), AsyncUtils.params))
  } else None
}

class IcnRxAsyncBundle(node: Node)(implicit p: Parameters) extends ZJBundle with BaseAsyncIcnMonoBundle {
  private val illegal = node.injects.contains("REQ") && node.injects.contains("ERQ")
  require(!illegal)
  val req = if(node.injects.contains("REQ") || node.injects.contains("ERQ") && !node.csnNode) {
    Some(Flipped(new AsyncBundle(UInt(reqFlitBits.W), AsyncUtils.params)))
  } else None

  val resp = if(node.injects.contains("RSP")) {
    Some(Flipped(new AsyncBundle(UInt(respFlitBits.W), AsyncUtils.params)))
  } else None

  val data = if(node.injects.contains("DAT")) {
    Some(Flipped(new AsyncBundle(UInt(dataFlitBits.W), AsyncUtils.params)))
  } else None

  val snoop = if(node.injects.contains("SNP")) {
    Some(Flipped(new AsyncBundle(UInt(snoopFlitBits.W), AsyncUtils.params)))
  } else None
}

class IcnAsyncBundle(val node: Node)(implicit p: Parameters) extends ZJBundle {
  val tx = new IcnTxAsyncBundle(node)
  val rx = new IcnRxAsyncBundle(node)
  val resetTx = Output(AsyncReset())
  def <>(that: DeviceIcnAsyncBundle): Unit = {
    this.rx <> that.tx
    that.rx <> this.tx
    that.resetRx := this.resetTx
  }
}

class DeviceIcnAsyncBundle(val node: Node)(implicit p: Parameters) extends ZJBundle {
  val tx = Flipped(new IcnRxAsyncBundle(node))
  val rx = Flipped(new IcnTxAsyncBundle(node))
  val resetRx = Input(AsyncReset())
  def <>(that: IcnAsyncBundle): Unit = {
    this.rx <> that.tx
    that.rx <> this.tx
    this.resetRx := that.resetTx
  }
}

abstract class BaseIcnAsyncModule(node: Node, icnSide:Boolean)(implicit p: Parameters) extends ZJModule {
  private val flitBitsMap = Map[String, Int](
    "REQ" -> reqFlitBits,
    "RSP" -> respFlitBits,
    "DAT" -> dataFlitBits,
    "SNP" -> snoopFlitBits,
    "ERQ" -> reqFlitBits
  )

  def icnRxBundle: BaseIcnMonoBundle
  def asyncTxBundle: BaseAsyncIcnMonoBundle

  def icnTxBundle: BaseIcnMonoBundle
  def asyncRxBundle: BaseAsyncIcnMonoBundle

  private var initialized = false
  def makeConnections(): Unit = {
    if(initialized) return
    initialized = true
    val toAsync = if(icnSide) node.ejects else node.injects
    val fromAsync = if(icnSide) node.injects else node.ejects
    for(chn <- toAsync) {
      val icnRx = icnRxBundle.chnMap(chn).get
      val asyncTx = asyncTxBundle.chnMap(chn).get
      val flitType = UInt(flitBitsMap(chn).W)
      val asyncSource = Module(new AsyncSource(flitType))
      asyncSource.io.enq.valid := icnRx.valid
      asyncSource.io.enq.bits := icnRx.bits.asTypeOf(flitType)
      icnRx.ready := asyncSource.io.enq.ready
      asyncTx <> asyncSource.io.async
      asyncSource.suggestName(s"${chn}AsyncSource")
    }

    for(chn <- fromAsync) {
      val icnTx = icnTxBundle.chnMap(chn).get
      val asyncRx = asyncRxBundle.chnMap(chn).get
      val flitType = UInt(flitBitsMap(chn).W)
      val asyncSink = Module(new AsyncSink(flitType))
      asyncSink.io.async <> asyncRx
      icnTx.valid := asyncSink.io.deq.valid
      icnTx.bits := asyncSink.io.deq.bits.asTypeOf(icnTx.bits)
      asyncSink.io.deq.ready := icnTx.ready
      asyncSink.suggestName(s"${chn}AsyncSink")
    }
  }
}

class IcnSideAsyncModule(node: Node)(implicit p: Parameters) extends BaseIcnAsyncModule(node = node, icnSide = true) {
  val io = IO(new Bundle {
    val icn = new DeviceIcnBundle(node)
    val async = new IcnAsyncBundle(node)
  })
  val icnRxBundle = io.icn.rx
  val asyncTxBundle = io.async.tx
  val icnTxBundle = io.icn.tx
  val asyncRxBundle = io.async.rx
  io.async.resetTx := reset
  makeConnections()
}

class DeviceSideAsyncModule(node: Node)(implicit p: Parameters) extends BaseIcnAsyncModule(node = node, icnSide = false) {
  val io = IO(new Bundle {
    val icn = new IcnBundle(node)
    val async = new DeviceIcnAsyncBundle(node)
  })
  val icnRxBundle = io.icn.rx
  val asyncTxBundle = io.async.tx
  val icnTxBundle = io.icn.tx
  val asyncRxBundle = io.async.rx
  makeConnections()
}