package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.cde.config.Parameters


object BurstMode {
  val width        = 2
  val Fix          = "b00".U
  val Incr         = "b01".U
  val Wrap         = "b10".U
  val Reserve      = "b11".U
}