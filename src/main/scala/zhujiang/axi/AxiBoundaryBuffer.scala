package zhujiang.axi

import chisel3._
import xs.utils.queue.{FastQueue, RegInQueue}

class AxiBoundaryBuffer(params: AxiParams) extends Module {
  val io = IO(new Bundle {
    val mst = Flipped(new AxiBundle(params))
    val slv = new AxiBundle(params)
  })

  io.slv.aw <> FastQueue(io.mst.aw, Some("awq"))
  io.slv.ar <> FastQueue(io.mst.ar, Some("arq"))
  io.slv.w <> FastQueue(io.mst.w, Some("wq"))
  io.mst.b <> RegInQueue(io.slv.b, "bq")
  io.mst.r <> RegInQueue(io.slv.r, "rq")
}

object AxiBoundaryBuffer {
  def apply(mst: AxiBundle, name: Option[String]): AxiBundle = {
    val buf = Module(new AxiBoundaryBuffer(mst.params))
    buf.io.mst <> mst
    name.foreach(n => buf.suggestName(n))
    buf.io.slv
  }
  def apply(mst: AxiBundle, slv: AxiBundle, name: Option[String]): AxiBoundaryBuffer = {
    val buf = Module(new AxiBoundaryBuffer(mst.params))
    buf.io.mst <> mst
    name.foreach(n => buf.suggestName(n))
    slv <> buf.io.slv
    buf
  }

  def apply(mst: AxiBundle, name:String): AxiBundle = apply(mst, Some(name))
  def apply(mst: AxiBundle, slv: AxiBundle, name:String): AxiBoundaryBuffer = apply(mst, slv, Some(name))

  def apply(mst: AxiBundle): AxiBundle = apply(mst, None)
  def apply(mst: AxiBundle, slv: AxiBundle): AxiBoundaryBuffer = apply(mst, slv, None)
}