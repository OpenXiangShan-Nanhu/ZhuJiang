package zhujiang.device.cluster.tlu2chi

import chisel3._
import chisel3.util._
import xijiang.Node
import xijiang.router.base.DeviceIcnBundle
import org.chipsalliance.cde.config.Parameters
import xs.utils.FastArbiter
import zhujiang.ZJModule
import zhujiang.chi.{DataFlit, RespFlit}
import zhujiang.tilelink.{DFlit, TilelinkParams}

class MachineHandler(node: Node, tlParams: TilelinkParams, outstanding: Int)(implicit p: Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val allocOH = Output(UInt(outstanding.W))
    val alloc_s1 = Flipped(Decoupled(new TaskBundle(tlParams)))
    val status = Vec(outstanding, Output(new MachineStatus(tlParams)))
    val icn = new DeviceIcnBundle(node)
    val tld = Decoupled(new DFlit(tlParams))
  })

  io <> DontCare

  private val machines = (0 until outstanding).map(i => Module(new TransactionMachine(node, tlParams, outstanding)))

  private val freeVec = VecInit(machines.map(_.io.status.state === MachineState.IDLE)).asUInt
  private val freeOH = PriorityEncoderOH(freeVec)

  machines.zipWithIndex.foreach { case (machine, i) =>
    machine.io <> DontCare
    machine.io.alloc.valid := io.alloc_s1.valid && freeOH(i)
    machine.io.alloc.bits := io.alloc_s1.bits
    machine.io.id := i.U
    io.status(i) := machine.io.status

    val rxRespFlit = io.icn.rx.resp.get.bits
    machine.io.icn.rx.resp.get.valid := io.icn.rx.resp.get.valid && rxRespFlit.asTypeOf(new RespFlit).TxnID === i.U
    machine.io.icn.rx.resp.get.bits := rxRespFlit
    when(machine.io.icn.rx.resp.get.valid) {
      assert(machine.io.icn.rx.resp.get.ready)
    }

    val rxDataFlit = io.icn.rx.data.get.bits
    machine.io.icn.rx.data.get.valid := io.icn.rx.data.get.valid && rxDataFlit.asTypeOf(new DataFlit).TxnID === i.U
    machine.io.icn.rx.data.get.bits := rxDataFlit
    when(machine.io.icn.rx.data.get.valid) {
      assert(machine.io.icn.rx.data.get.ready)
    }
  }

  io.alloc_s1.ready := freeVec.orR
  io.allocOH := freeVec
  io.icn.rx.resp.get.ready := true.B
  io.icn.rx.data.get.ready := true.B

  FastArbiter(machines.map(_.io.tld), io.tld)
  FastArbiter(machines.map(_.io.icn.tx.req.get), io.icn.tx.req.get)
  FastArbiter(machines.map(_.io.icn.tx.data.get), io.icn.tx.data.get)
}
