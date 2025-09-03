package dongjiang.utils

import chisel3._
import chisel3.util._
import xs.utils.ResetRRArbiter
import xs.utils.arb.VipArbiter

object VipEncoder {
  def apply(in: Seq[Bool], enable: Bool): UInt = {
    val vipArb = Module(new VipArbiter(UInt(0.W), in.size))
    vipArb.io <> DontCare
    vipArb.io.in.zip(in).foreach(in => in._1.valid := in._2)
    vipArb.io.out.ready := enable
    vipArb.io.chosen
  }
  def apply(in: Bits, enable: Bool): UInt = {
    apply(in.asBools, enable)
  }
}
