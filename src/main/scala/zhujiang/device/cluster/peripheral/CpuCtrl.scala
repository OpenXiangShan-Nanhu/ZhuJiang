package zhujiang.device.cluster.peripheral

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import zhujiang.tilelink.{BaseTLULPeripheral, TilelinkParams}

class CpuCtrl(tlParams: TilelinkParams)(implicit p: Parameters) extends BaseTLULPeripheral(tlParams) {
  val io = IO(new Bundle {
    val defaultBootAddr = Input(UInt(64.W))
    val defaultEnable = Input(Bool())
    val cpuBootAddr = Output(UInt(64.W))
    val cpuReset = Output(Bool())
  })
  private val addrReg = Reg(UInt(64.W))
  private val ctrlReg = Reg(UInt(64.W))
  private val addrWire = WireInit(addrReg)
  private val ctrlWire = WireInit(ctrlReg)

  val regSeq = Seq(
    ("addr", addrReg, addrWire, 0x0, None, None),
    ("ctrl", ctrlReg, ctrlWire, 0x8, Some(0x1.U), Some(0x1.U))
  )

  private val writeMap = genWriteMap()

  when(reset.asBool) {
    addrReg := io.defaultBootAddr
  }.elsewhen(writeMap("addr")) {
    addrReg := addrWire
  }

  when(reset.asBool) {
    ctrlReg := Cat(0.U(63.W), io.defaultEnable)
  }.elsewhen(writeMap("ctrl")) {
    ctrlReg := ctrlWire
  }

  io.cpuBootAddr := addrReg
  io.cpuReset := ctrlReg(0)
}
