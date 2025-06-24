package xijiang.router.base

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xs.utils.debug.HardwareAssertionKey
import zhujiang.ZJBundle
import zhujiang.chi._

trait BaseIcnMonoBundle {
  def req: Option[DecoupledIO[Data]]
  def hpr: Option[DecoupledIO[Data]]
  def resp: Option[DecoupledIO[Data]]
  def data: Option[DecoupledIO[Data]]
  def snoop: Option[DecoupledIO[Data]]
  def debug: Option[DecoupledIO[Data]]
  private lazy val _bundleMap = Seq(
    "REQ" -> req,
    "RSP" -> resp,
    "DAT" -> data,
    "SNP" -> snoop,
    "HPR" -> hpr,
    "DBG" -> debug,
    "ERQ" -> req
  )
  lazy val bundleMap = _bundleMap.flatMap(elm => Option.when(elm._2.isDefined)(elm._1, elm._2.get)).toMap
}

class IcnTxBundle(node: Node)(implicit p: Parameters) extends ZJBundle with BaseIcnMonoBundle {
  private val split = zjParams.splitFlit
  private val hwaP = p(HardwareAssertionKey)
  val req = if(node.ejects.contains("REQ")) {
    if(split) Some(Decoupled(new RReqFlit)) else Some(Decoupled(UInt(rreqFlitBits.W)))
  } else if(node.ejects.contains("ERQ")) {
    if(split) Some(Decoupled(new HReqFlit)) else Some(Decoupled(UInt(hreqFlitBits.W)))
  } else None

  val hpr = if(node.ejects.contains("HPR") && hasHprRing) {
    if(split) Some(Decoupled(new RReqFlit)) else Some(Decoupled(UInt(rreqFlitBits.W)))
  } else None

  val resp = if(node.ejects.contains("RSP")) {
    if(split) Some(Decoupled(new RespFlit)) else Some(Decoupled(UInt(respFlitBits.W)))
  } else None

  val data = if(node.ejects.contains("DAT")) {
    if(split) Some(Decoupled(new DataFlit)) else Some(Decoupled(UInt(dataFlitBits.W)))
  } else None

  val snoop = if(node.ejects.contains("SNP")) {
    if(split) Some(Decoupled(new SnoopFlit)) else Some(Decoupled(UInt(snoopFlitBits.W)))
  } else None

  val debug = if(node.ejects.contains("DBG") && hwaP.enable) {
    if(split) Some(Decoupled(new RingFlit(debugFlitBits))) else Some(Decoupled(UInt(debugFlitBits.W)))
  } else None

  def rsp: Option[DecoupledIO[Data]] = resp
  def dat: Option[DecoupledIO[Data]] = data
  def snp: Option[DecoupledIO[Data]] = snoop
  def dbg: Option[DecoupledIO[Data]] = debug

  def getRingBundle(chn: String): Option[DecoupledIO[Data]] = {
    val ej = node.ejects
    chn match {
      case "REQ" => req
      case "RSP" => rsp
      case "DAT" => dat
      case "HRQ" => if(ej.contains("ERQ")) req else snp
      case "DBG" => dbg
      case "HPR" => hpr
      case _ => None
    }
  }
}

class IcnRxBundle(node: Node)(implicit p: Parameters) extends ZJBundle with BaseIcnMonoBundle {
  private val split = zjParams.splitFlit
  private val hwaP = p(HardwareAssertionKey)
  val req = if(node.injects.contains("REQ")) {
    if(split) Some(Flipped(Decoupled(new RReqFlit))) else Some(Flipped(Decoupled(UInt(rreqFlitBits.W))))
  } else if(node.injects.contains("ERQ")) {
    if(split) Some(Flipped(Decoupled(new HReqFlit))) else Some(Flipped(Decoupled(UInt(hreqFlitBits.W))))
  } else None

  val hpr = if(node.injects.contains("HPR") && hasHprRing) {
    if(split) Some(Flipped(Decoupled(new RReqFlit))) else Some(Flipped(Decoupled(UInt(rreqFlitBits.W))))
  } else None

  val resp = if(node.injects.contains("RSP")) {
    if(split) Some(Flipped(Decoupled(new RespFlit))) else Some(Flipped(Decoupled(UInt(respFlitBits.W))))
  } else None

  val data = if(node.injects.contains("DAT")) {
    if(split) Some(Flipped(Decoupled(new DataFlit))) else Some(Flipped(Decoupled(UInt(dataFlitBits.W))))
  } else None

  val snoop = if(node.injects.contains("SNP")) {
    if(split) Some(Flipped(Decoupled(new SnoopFlit))) else Some(Flipped(Decoupled(UInt(snoopFlitBits.W))))
  } else None

  val debug = if(node.injects.contains("DBG") && hwaP.enable) {
    if(split) Some(Flipped(Decoupled(new RingFlit(debugFlitBits)))) else Some(Flipped(Decoupled(UInt(debugFlitBits.W))))
  } else None

  def rsp: Option[DecoupledIO[Data]] = resp
  def dat: Option[DecoupledIO[Data]] = data
  def snp: Option[DecoupledIO[Data]] = snoop
  def dbg: Option[DecoupledIO[Data]] = debug

  def getRingBundle(chn: String): Option[DecoupledIO[Data]] = {
    val ij = node.injects
    chn match {
      case "REQ" => req
      case "RSP" => rsp
      case "DAT" => dat
      case "HRQ" => if(ij.contains("ERQ")) req else snp
      case "DBG" => dbg
      case "HPR" => hpr
      case _ => None
    }
  }
}

class IcnBundle(val node: Node, hasReset:Boolean = false)(implicit p: Parameters) extends ZJBundle {
  val tx = new IcnTxBundle(node)
  val rx = new IcnRxBundle(node)
  val resetState = if(hasReset) Some(Output(Vec(2, Bool()))) else None
  val resetInject = if(hasReset && node.nodeType == NodeType.M) Some(Input(Vec(2, Bool()))) else None
  def <>(that: DeviceIcnBundle): Unit = {
    this.rx <> that.tx
    that.rx <> this.tx
    if(this.resetState.isDefined && that.resetState.isDefined) {
      this.resetState.get <> that.resetState.get
    }
    if(this.resetInject.isDefined && that.resetInject.isDefined) {
      this.resetInject.get <> that.resetInject.get
    }
  }
}

class DeviceIcnBundle(val node: Node, hasReset:Boolean = false)(implicit p: Parameters) extends ZJBundle {
  val tx = Flipped(new IcnRxBundle(node))
  val rx = Flipped(new IcnTxBundle(node))
  val resetState = if(hasReset) Some(Input(Vec(2, Bool()))) else None
  val resetInject = if(hasReset && node.nodeType == NodeType.M) Some(Output(Vec(2, Bool()))) else None
  def <>(that: IcnBundle): Unit = {
    this.rx <> that.tx
    that.rx <> this.tx
    if(this.resetState.isDefined && that.resetState.isDefined) {
      this.resetState.get <> that.resetState.get
    }
    if(this.resetInject.isDefined && that.resetInject.isDefined) {
      this.resetInject.get <> that.resetInject.get
    }
  }
}