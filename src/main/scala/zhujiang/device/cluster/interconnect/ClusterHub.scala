package zhujiang.device.cluster.interconnect

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}
import xs.utils.{DFTResetSignals, ResetRRArbiter}
import xs.utils.sram.SramBroadcastBundle
import zhujiang.chi._
import zhujiang.device.async.{DeviceIcnAsyncBundle, DeviceSideAsyncModule, IcnAsyncBundle}
import zhujiang.{ZJBundle, ZJModule}

class ClusterMiscWires(node: Node)(implicit p: Parameters) extends ZJBundle {
  val msip = Input(Vec(node.cpuNum, Bool()))
  val mtip = Input(Vec(node.cpuNum, Bool()))
  val meip = Input(Vec(node.cpuNum, Bool()))
  val seip = Input(Vec(node.cpuNum, Bool()))
  val dbip = Input(Vec(node.cpuNum, Bool()))
  val resetVector = Input(Vec(node.cpuNum, UInt(raw.W)))
  val resetEnable = Input(Vec(node.cpuNum, Bool()))
  val resetState = Output(Vec(node.cpuNum, Bool()))
  val mhartid = Input(Vec(node.cpuNum, UInt(clusterIdBits.W)))
  val halt = Output(Vec(node.cpuNum, Bool()))
  val beu = Output(Vec(node.cpuNum, Bool()))
}

class DftWires extends Bundle {
  val reset = new DFTResetSignals
  val func = new SramBroadcastBundle
}

class ClusterDeviceBundle(node: Node)(implicit p: Parameters) extends ZJBundle {
  val async = new DeviceIcnAsyncBundle(node)
  val misc = new ClusterMiscWires(node)
  val osc_clock = Input(Clock())
  val dft = Input(new DftWires)
}

class ClusterHub(node: Node)(implicit p: Parameters) extends ZJModule {
  require(node.nodeType == NodeType.CC)
  val io = IO(new Bundle {
    val icn = new ClusterDeviceBundle(node)
    val peripheral = new IcnBundle(node.copy(nodeType = NodeType.HI))
    val l2cache = new IcnBundle(node.copy(nodeType = NodeType.RF))
    val cio = new IcnBundle(node.copy(nodeType = NodeType.RI))
    val cpu = Flipped(new ClusterMiscWires(node))
    val dft = Output(new DftWires)
  })
  io.peripheral.rx.req.get.ready := false.B
  io.cpu <> io.icn.misc
  io.dft := io.icn.dft
  private val asyncModule = Module(new DeviceSideAsyncModule(node))
  asyncModule.io.async <> io.icn.async

  private val rxChnMap = node.ejects.map({ chn =>
    val eject = asyncModule.io.icn.tx.getBundle(chn).get
    val pipe = Module(new Queue(UInt(eject.bits.getWidth.W), entries = 2))
    pipe.suggestName(s"rxPipe$chn")
    pipe.io.enq.valid := eject.valid
    pipe.io.enq.bits := eject.bits.asTypeOf(pipe.io.enq.bits)
    eject.ready := pipe.io.enq.ready
    (chn, pipe.io.deq)
  }).toMap

  private val icnRxReq = rxChnMap("REQ")
  private val icnRxRsp = rxChnMap("RSP")
  private val icnRxDat = rxChnMap("DAT")
  private val icnRxSnp = rxChnMap("SNP")
  private val icnRxRspAid = icnRxRsp.bits.asTypeOf(new RespFlit).TgtID.asTypeOf(new NodeIdBundle).aid
  private val icnRxDatAid = icnRxDat.bits.asTypeOf(new DataFlit).TgtID.asTypeOf(new NodeIdBundle).aid

  io.peripheral.tx.req.get.valid := icnRxReq.valid
  io.peripheral.tx.req.get.bits := icnRxReq.bits.asTypeOf(io.peripheral.tx.req.get.bits)
  io.peripheral.tx.resp.get.valid := icnRxRsp.valid && icnRxRspAid === 0.U
  io.peripheral.tx.resp.get.bits := icnRxRsp.bits.asTypeOf(io.peripheral.tx.resp.get.bits)
  io.peripheral.tx.data.get.valid := icnRxDat.valid && icnRxDatAid === 0.U
  io.peripheral.tx.data.get.bits := icnRxDat.bits.asTypeOf(io.peripheral.tx.data.get.bits)

  io.l2cache.tx.snoop.get.valid := icnRxSnp.valid
  io.l2cache.tx.snoop.get.bits := icnRxSnp.bits.asTypeOf(io.l2cache.tx.snoop.get.bits)
  io.l2cache.tx.resp.get.valid := icnRxRsp.valid && icnRxRspAid === 1.U
  io.l2cache.tx.resp.get.bits := icnRxRsp.bits.asTypeOf(io.l2cache.tx.resp.get.bits)
  io.l2cache.tx.data.get.valid := icnRxDat.valid && icnRxDatAid === 1.U
  io.l2cache.tx.data.get.bits := icnRxDat.bits.asTypeOf(io.l2cache.tx.data.get.bits)

  io.cio.tx.resp.get.valid := icnRxRsp.valid && icnRxRspAid === 2.U
  io.cio.tx.resp.get.bits := icnRxRsp.bits.asTypeOf(io.cio.tx.resp.get.bits)
  io.cio.tx.data.get.valid := icnRxDat.valid && icnRxDatAid === 2.U
  io.cio.tx.data.get.bits := icnRxDat.bits.asTypeOf(io.cio.tx.data.get.bits)

  icnRxReq.ready := io.peripheral.tx.req.get.ready
  icnRxSnp.ready := io.l2cache.tx.snoop.get.ready
  icnRxRsp.ready := MuxCase(false.B, Seq(
    (icnRxRspAid === 0.U, io.peripheral.tx.resp.get.ready),
    (icnRxRspAid === 1.U, io.l2cache.tx.resp.get.ready),
    (icnRxRspAid === 2.U, io.cio.tx.resp.get.ready)
  ))
  icnRxDat.ready := MuxCase(false.B, Seq(
    (icnRxDatAid === 0.U, io.peripheral.tx.data.get.ready),
    (icnRxDatAid === 1.U, io.l2cache.tx.data.get.ready),
    (icnRxDatAid === 2.U, io.cio.tx.data.get.ready)
  ))

  private val txChnMap = node.injects.map({ chn =>
    val inject = asyncModule.io.icn.rx.getBundle(chn).get
    val pipe = Module(new Queue(UInt(inject.bits.getWidth.W), entries = 2))
    pipe.suggestName(s"txPipe$chn")
    inject.valid := pipe.io.deq.valid
    inject.bits := pipe.io.deq.bits.asTypeOf(inject.bits)
    pipe.io.deq.ready := inject.ready
    (chn, pipe.io.enq)
  }).toMap

  private val icnTxReqArb = Module(new ResetRRArbiter(UInt(reqFlitBits.W), 2))
  txChnMap("REQ") <> icnTxReqArb.io.out
  private val icnTxRespArb = Module(new ResetRRArbiter(UInt(respFlitBits.W), 3))
  txChnMap("RSP") <> icnTxRespArb.io.out
  private val icnTxDataArb = Module(new ResetRRArbiter(UInt(dataFlitBits.W), 3))
  txChnMap("DAT") <> icnTxDataArb.io.out

  private val flitMap = Seq(
    ("REQ", new ReqFlit),
    ("RSP", new RespFlit),
    ("DAT", new DataFlit),
  ).toMap

  private def txConn(arbin: DecoupledIO[UInt], icn: IcnBundle, aid: Int, chn: String): Unit = {
    val icnRxChn = icn.rx.getBundle(chn).get
    arbin.valid := icnRxChn.valid
    icnRxChn.ready := arbin.ready
    val icnRxChnFlit = WireInit(icnRxChn.bits.asTypeOf(flitMap(chn)))
    val icnRxChnSrcId = WireInit(0.U.asTypeOf(new NodeIdBundle))
    icnRxChnFlit.src := icnRxChnSrcId.asUInt
    icnRxChnSrcId.aid := aid.U
    arbin.bits := icnRxChnFlit.asUInt
  }

  txConn(icnTxReqArb.io.in(0), io.l2cache, 1, "REQ")
  txConn(icnTxReqArb.io.in(1), io.cio, 2, "REQ")

  txConn(icnTxRespArb.io.in(0), io.peripheral, 0, "RSP")
  txConn(icnTxRespArb.io.in(1), io.l2cache, 1, "RSP")
  txConn(icnTxRespArb.io.in(2), io.cio, 2, "RSP")

  txConn(icnTxDataArb.io.in(0), io.peripheral, 0, "DAT")
  txConn(icnTxDataArb.io.in(1), io.l2cache, 1, "DAT")
  txConn(icnTxDataArb.io.in(2), io.cio, 2, "DAT")
}
