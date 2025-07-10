package zhujiang.axi

import chisel3._
import chisel3.util._

class AxiBuffer(axiParams: AxiParams, depth:Int = 2) extends Module {
  val io = IO(new Bundle {
    val in = Flipped(new AxiBundle(axiParams))
    val out = new AxiBundle(axiParams)
  })
  private val pipe = depth == 1
  io.out.aw <> Queue(io.in.aw, entries = depth, pipe = pipe)
  io.out.ar <> Queue(io.in.ar, entries = depth, pipe = pipe)
  io.out.w <> Queue(io.in.w, entries = depth, pipe = pipe)
  io.in.r <> Queue(io.out.r, entries = depth, pipe = pipe)
  io.in.b <> Queue(io.out.b, entries = depth, pipe = pipe)
}

class AxiBufferChain(axiParams: AxiParams, chain:Int) extends Module {
  val io = IO(new Bundle {
    val in = Flipped(new AxiBundle(axiParams))
    val out = new AxiBundle(axiParams)
  })
  private val bufSeq = Seq.fill(chain)(Module(new AxiBuffer(axiParams)))
  bufSeq.zipWithIndex.foreach({case(a, b) => a.suggestName(s"buf_$b")})
  if(chain == 0) {
    io.out <> io.in
  } else {
    io.out <> bufSeq.foldLeft(io.in)((enq:AxiBundle, buf:AxiBuffer) => {
      buf.io.in <> enq
      buf.io.out
    })
  }
}

object AxiBuffer {
  def apply(in: AxiBundle, depth:Int = 2, name:Option[String] = None):AxiBundle = {
    val buffer = Module(new AxiBuffer(in.params, depth))
    buffer.io.in <> in
    if(name.isDefined) buffer.suggestName(name.get)
    buffer.io.out
  }
  def chain(in: AxiBundle, length:Int, name:Option[String]):AxiBundle = {
    val bufChain = Module(new AxiBufferChain(in.params, length))
    if(name.isDefined) bufChain.suggestName(s"${name.get}_buf_chain")
    bufChain.io.in <> in
    bufChain.io.out
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
