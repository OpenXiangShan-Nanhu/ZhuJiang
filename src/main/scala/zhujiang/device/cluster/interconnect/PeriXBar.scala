package zhujiang.device.cluster.interconnect

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import zhujiang.ZJParametersKey
import zhujiang.tilelink.{BaseTLULXbar, TLULBundle, TilelinkParams}

class PeriXBar(tlParams: Seq[TilelinkParams], sharePeriNum: Int, coreNum: Int, privatePeriPerCore: Int)(implicit p: Parameters) extends BaseTLULXbar {
  private val coreIdBits = clusterIdBits - nodeAidBits
  private val cpuSpaceBits = p(ZJParametersKey).cpuSpaceBits
  private val devSpaceBits = p(ZJParametersKey).cpuDevSpaceBits
  private val mstAddrBits = cpuSpaceBits + coreIdBits
  val mstParams = tlParams.map(_.copy(addrBits = mstAddrBits))
  val slvAddrBits = devSpaceBits

  val misc = IO(new Bundle {
    val core = Input(Vec(coreNum, UInt(coreIdBits.W)))
  })
  private val sharedAddrMatcher = Seq.tabulate(sharePeriNum)(i => (addr: UInt) => {
    val matchRes = Wire(Bool())
    matchRes := addr(cpuSpaceBits - 1) && addr(cpuSpaceBits - 2, devSpaceBits) === i.U
    matchRes
  })
  private val privateAddrMatcher = Seq.tabulate(coreNum, privatePeriPerCore)(
    (c, i) => (addr: UInt) => {
      val matchRes = Wire(Bool())
      matchRes := addr(mstAddrBits - 1, cpuSpaceBits) === misc.core(c) && !addr(cpuSpaceBits - 1) && addr(cpuSpaceBits - 2, devSpaceBits) === i.U
      matchRes
    }
  )

  val slvMatchersSeq = sharedAddrMatcher ++ privateAddrMatcher.flatten
  private val slvMax = 1 << (cpuSpaceBits - 1 - devSpaceBits)
  require(slvMatchersSeq.length < slvMax)
  initialize()
}
