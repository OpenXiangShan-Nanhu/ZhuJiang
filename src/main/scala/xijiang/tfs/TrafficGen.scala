package xijiang.tfs

import chisel3._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import xijiang.router.base.IcnBundle
import zhujiang.ZJModule
import zhujiang.chi.ChannelEncodings

class TrafficGen(node: Node)(implicit p: Parameters) extends ZJModule {
  override val desiredName = s"TrafficGen${node.nodeStr}"
  val nodeId = IO(Input(UInt(niw.W)))
  val icn = IO(Flipped(new IcnBundle(node, true)))

  private def generate(chn: String): Unit = {
    val txp = icn.rx.getBundle(chn)
    if(txp.isDefined) {
      val txGen = Module(new TrafficSimTx)
      txp.get.valid := txGen.io.tx.valid
      txp.get.bits := txGen.io.tx.bits.asTypeOf(txp.get.bits)
      txGen.io.tx.ready := txp.get.ready
      txGen.io.nodeId := nodeId
      txGen.io.chn := ChannelEncodings.encodingsMap(chn).U
      txGen.io.clock := clock
      txGen.io.reset := !icn.resetState.get.reduce(_ | _)
    }
    val rxp = icn.tx.getBundle(chn)
    if(rxp.isDefined) {
      val rxGen = Module(new TrafficSimRx)
      rxGen.io.rx.valid := rxp.get.valid
      rxGen.io.rx.bits := rxp.get.bits.asUInt
      rxp.get.ready := rxGen.io.rx.ready
      rxGen.io.nodeId := nodeId
      rxGen.io.chn := ChannelEncodings.encodingsMap(chn).U
      rxGen.io.clock := clock
      rxGen.io.reset := !icn.resetState.get.reduce(_ | _)
    }
  }
  icn.resetInject.foreach(_ := DontCare)
  generate("REQ")
  generate("RSP")
  generate("DAT")
  generate("SNP")
  generate("ERQ")
  generate("DBG")
}
