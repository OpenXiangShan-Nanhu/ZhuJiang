package zhujiang.device.cluster.peripheral

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import zhujiang.tilelink.{BaseTLULPeripheral, TilelinkParams}

class DistributedAclint(tlParams: TilelinkParams)(implicit p: Parameters) extends BaseTLULPeripheral(tlParams) {
  val io = IO(new Bundle {
    val msip = Output(Bool())
    val ssip = Output(Bool())
    val mtip = Output(Bool())
    val mtime = Input(UInt(64.W))
    val mtimeUpdate = Output(Valid(UInt(64.W)))
  })
  private val msip = RegInit(0.U(32.W))
  private val ssip = RegInit(0.U(32.W))
  private val mtimecmp = RegInit(0.U(64.W))
  private val mtimeWire = WireInit(io.mtime)
  val regSeq = Seq(
    ("msip", msip, msip, 0x0, Some(0x1.U), Some(0x1.U)),
    ("ssip", ssip, ssip, 0x4, Some(0x1.U), Some(0x1.U)),
    ("mtimecmp", mtimecmp, mtimecmp, 0x8, None, None),
    ("mtime", io.mtime, mtimeWire, 0x10, None, None)
  )
  io.msip := msip(0)
  io.ssip := ssip(0)
  io.mtip := mtimecmp <= io.mtime
  private val updateMap = genWriteMap()
  io.mtimeUpdate.valid := updateMap("mtime")
  io.mtimeUpdate.bits := mtimeWire
}
