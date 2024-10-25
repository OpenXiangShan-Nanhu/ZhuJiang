package zhujiang.device.cluster.peripheral

import xs.utils.ClockManagerWrapper
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import zhujiang.tilelink.{BaseTLULPeripheral, TilelinkParams}

class ClusterPLL(tlParams: TilelinkParams)(implicit p: Parameters) extends BaseTLULPeripheral(tlParams) {
  val io = IO(new Bundle {
    val cfg = Output(Vec(8, UInt(32.W)))
    val lock = Input(Bool())
  })
  private val cfgReg = RegInit(VecInit(Seq.fill(8)(0.U(32.W))))
  private val lock = WireInit(0.U(32.W))
  val regSeq = Seq.tabulate(8)(i =>
    (s"cfg_$i", cfgReg(i), cfgReg(i), i * cfgReg(i).getWidth / 8, None, None)
  ) ++ Seq(
    (s"lock", io.lock.asTypeOf(UInt(32.W)), lock, cfgReg.length * cfgReg.head.getWidth / 8, Some(0.U(32.W)), Some(1.U(32.W)))
  )
  io.cfg := cfgReg
  private val dummy = genWriteMap()
}
