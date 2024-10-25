package zhujiang.tilelink

import chisel3._
import chisel3.util.Decoupled

case class TilelinkParams(
  addrBits: Int = 32,
  sourceBits: Int = 5,
  sinkBits: Int = 5,
  dataBits: Int = 64
)

class AFlit(params: TilelinkParams) extends Bundle {
  val opcode = UInt(3.W)
  val param = UInt(3.W)
  val size = UInt(3.W)
  val source = UInt(params.sourceBits.W)
  val address = UInt(params.addrBits.W)
  val mask = UInt((params.dataBits / 8).W)
  val data = UInt(params.dataBits.W)
  val corrupt = Bool()
}

class DFlit(params: TilelinkParams) extends Bundle {
  val opcode = UInt(3.W)
  val param = UInt(2.W)
  val size = UInt(3.W)
  val source = UInt(params.sourceBits.W)
  val sink = UInt(params.sinkBits.W)
  val denied = UInt((params.dataBits / 8).W)
  val data = UInt(params.dataBits.W)
  val corrupt = Bool()
}

class TLULBundle(val params: TilelinkParams) extends Bundle {
  val a = Decoupled(new AFlit(params))
  val d = Flipped(Decoupled(new DFlit(params)))
}