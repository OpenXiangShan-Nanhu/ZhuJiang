package zhujiang.device.cluster.interconnect

import chisel3._
import chisel3.util.Cat
import org.chipsalliance.cde.config.Parameters
import zhujiang.ZJParametersKey
import zhujiang.chi.ReqAddrBundle
import zhujiang.tilelink.{BaseTLULXbar, TilelinkParams}

class CioXBar(val mstParams: Seq[TilelinkParams])(implicit p: Parameters) extends BaseTLULXbar {
  private val coreIdBits = clusterIdBits - nodeAidBits
  private val cpuSpaceBits = p(ZJParametersKey).cpuSpaceBits
  mstParams.foreach(m => require(m.addrBits == raw))
  val slvAddrBits = raw
  val misc = IO(new Bundle {
    val chip = Input(UInt(nodeAidBits.W))
    val core = Input(Vec(mstParams.length, UInt(coreIdBits.W)))
  })
  private def slvMatcher(local: Boolean)(addr: UInt): Bool = {
    val reqAddr = addr.asTypeOf(new ReqAddrBundle)
    val mmio = reqAddr.mmio
    val chipMatch = reqAddr.chip === misc.chip
    val tagMatch = addr(raw - nodeAidBits - 2, cpuSpaceBits + coreIdBits) === 0.U
    val coreMatch = Cat(misc.core.map(_ === addr(cpuSpaceBits + coreIdBits - 1, cpuSpaceBits))).orR
    val matchRes = WireInit(mmio & chipMatch & tagMatch & coreMatch)
    if(local) matchRes else !matchRes
  }
  val slvMatchersSeq = Seq(slvMatcher(local = true), slvMatcher(local = false))
  initialize()
}
