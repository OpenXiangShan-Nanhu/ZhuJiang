package zhujiang.chi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}

class ChiBuffer(node: Node, depth:Int = 2)(implicit p:Parameters) extends Module {
  val io = IO(new Bundle{
    val dev = new DeviceIcnBundle(node)
    val icn = new IcnBundle(node)
  })
  private val pipe = depth == 1
  for((chn, src) <- io.dev.rx.elements) {
    val sink = io.icn.tx.elements(chn)
    sink <> Queue(src.asInstanceOf[DecoupledIO[Data]], entries = depth, pipe = pipe)
  }
  for((chn, sink) <- io.dev.tx.elements) {
    val src = io.icn.rx.elements(chn)
    sink <> Queue(src.asInstanceOf[DecoupledIO[Data]], entries = depth, pipe = pipe)
  }
}

object ChiBuffer {
  def apply(icn: IcnBundle, p: Parameters, depth: Int, name: Option[String]): IcnBundle = {
    val buf = Module(new ChiBuffer(icn.node, depth)(p))
    buf.io.dev <> icn
    if(name.isDefined) buf.suggestName(name.get)
    buf.io.icn
  }

  def apply(icn: DeviceIcnBundle, p: Parameters, depth:Int, name: Option[String]): IcnBundle = {
    val buf = Module(new ChiBuffer(icn.node, depth)(p))
    buf.io.dev <> icn
    if(name.isDefined) buf.suggestName(name.get)
    buf.io.icn
  }

  def apply(in: IcnBundle, p: Parameters, depth:Int): IcnBundle = {
    apply(in, p, depth, None)
  }

  def apply(in: DeviceIcnBundle, p: Parameters, depth:Int): IcnBundle = {
    apply(in, p, depth, None)
  }

  def apply(in: IcnBundle, p: Parameters): IcnBundle = {
    apply(in, p, 2, None)
  }

  def apply(in: DeviceIcnBundle, p: Parameters): IcnBundle = {
    apply(in, p, 2, None)
  }

  def apply(in: IcnBundle, p: Parameters, name: Option[String]): IcnBundle = {
    apply(in, p, 2, name)
  }

  def apply(in: DeviceIcnBundle, p: Parameters, name: Option[String]): IcnBundle = {
    apply(in, p, 2, name)
  }
}

