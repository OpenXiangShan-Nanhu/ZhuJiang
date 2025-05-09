package xijiang.tfs

import chisel3._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}
import zhujiang.ZJModule
import zhujiang.chi.FlitType

class TrafficGen(node: Node)(implicit p: Parameters) extends ZJModule {
  override val desiredName = s"TrafficGen${node.nodeStr}"
  val nodeId = IO(Input(UInt(niw.W)))
  val icn = IO(new DeviceIcnBundle(node, true))
  icn.resetInject.foreach(_ := DontCare)

  for(chn <- node.injects) {
    val txGen = Module(new TrafficSimTx)
    val tx = icn.tx.bundleMap(chn)
    tx.valid := txGen.io.tx.valid
    tx.bits := txGen.io.tx.bits.asTypeOf(tx.bits)
    txGen.io.tx.ready := tx.ready
    txGen.io.nodeId := nodeId
    txGen.io.chn := FlitType.encodings(chn).U
    txGen.io.clock := clock
    txGen.io.reset := !icn.resetState.get.reduce(_ | _)
  }
  for(chn <- node.ejects) {
    icn.rx.bundleMap(chn).ready := true.B
  }
}
