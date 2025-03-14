package zhujiang.chi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}

class ChiBuffer(node: Node, depth:Int = 2)(implicit p:Parameters) extends Module {
  val io = IO(new Bundle{
    val in = new DeviceIcnBundle(node)
    val out = new IcnBundle(node)
  })
  for((chn, src) <- io.in.rx.elements) {
    val sink = io.out.tx.elements(chn)
    sink <> Queue(src.asInstanceOf[DecoupledIO[Data]], entries = depth, pipe = true)
  }
  for((chn, sink) <- io.in.tx.elements) {
    val src = io.out.rx.elements(chn)
    sink <> Queue(src.asInstanceOf[DecoupledIO[Data]], entries = depth, pipe = true)
  }
}

object ChiBuffer {
  def apply(in: IcnBundle, p: Parameters, depth: Int, name: Option[String]): IcnBundle = {
    val buf = Module(new ChiBuffer(in.node, depth)(p))
    buf.io.in <> in
    if(name.isDefined) buf.suggestName(name.get)
    buf.io.out
  }

  def apply(in: DeviceIcnBundle, p: Parameters, depth:Int, name: Option[String]): IcnBundle = {
    val buf = Module(new ChiBuffer(in.node, depth)(p))
    buf.io.in <> in
    if(name.isDefined) buf.suggestName(name.get)
    buf.io.out
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

