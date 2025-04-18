package zhujiang.axi

import chisel3._
import chisel3.util._

class AxiBuffer(axiParams: AxiParams, depth:Int = 2) extends Module {
  val io = IO(new Bundle {
    val in = Flipped(new AxiBundle(axiParams))
    val out = new AxiBundle(axiParams)
  })
  io.out.aw <> Queue(io.in.aw, entries = depth, pipe = true)
  io.out.ar <> Queue(io.in.ar, entries = depth, pipe = true)
  io.out.w <> Queue(io.in.w, entries = depth, pipe = true)
  io.in.r <> Queue(io.out.r, entries = depth, pipe = true)
  io.in.b <> Queue(io.out.b, entries = depth, pipe = true)
}

object AxiBuffer {
  def apply(in: AxiBundle, depth:Int = 2, name:Option[String] = None):AxiBundle = {
    val buffer = Module(new AxiBuffer(in.params, depth))
    buffer.io.in <> in
    if(name.isDefined) buffer.suggestName(name.get)
    buffer.io.out
  }
  def chain(in: AxiBundle, length:Int, name:Option[String]):AxiBundle = {
    val bufferSeq = Seq.fill(length)(Module(new AxiBuffer(in.params, 2)))
    val out = if(length > 0) {
      bufferSeq.foldLeft(in)((enq:AxiBundle, buf:AxiBuffer) => {
        buf.io.in <> enq
        buf.io.out
      })
    } else {
      in
    }
    if(name.isDefined) for((buf, i) <- bufferSeq.zipWithIndex) buf.suggestName(s"${name.get}_buf_$i")
    out
  }
  def chain(in: AxiBundle, length:Int, name:String):AxiBundle = chain(in, length, Some(name))
  def chain(in: AxiBundle, length:Int):AxiBundle = chain(in, length, None)

  def chain(in: ExtAxiBundle, length:Int, name:Option[String]):AxiBundle = {
    val in_cvt = Wire(new AxiBundle(in.params))
    in_cvt <> in
    chain(in_cvt, length, name)
  }
  def chain(in: ExtAxiBundle, length:Int, name:String):AxiBundle = chain(in, length, Some(name))
  def chain(in: ExtAxiBundle, length:Int):AxiBundle = chain(in, length, None)
}
