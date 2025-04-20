package dongjiang.utils

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._

object Alloc {
  def apply[T <: Data](out: Seq[DecoupledIO[T]], in: DecoupledIO[T]): Unit = {
    val freeVec = out.map(_.ready)
    val freeId = PriorityEncoder(freeVec);
    dontTouch(freeId)
    out.zipWithIndex.foreach { case (o, i) =>
      o.valid := in.valid & freeId === i.U
      o.bits  := in.bits
    }
    in.ready := freeVec.reduce(_ | _)
  }
}