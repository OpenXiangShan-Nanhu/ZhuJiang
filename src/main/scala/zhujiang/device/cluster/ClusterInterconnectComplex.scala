package zhujiang.device.cluster

import chisel3._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.IcnBundle
import zhujiang.ZJModule
import zhujiang.device.bridge.tlul.TLULBridge
import zhujiang.device.cluster.interconnect._
import zhujiang.device.cluster.peripheral.{ClusterPLL, CpuCtrl}
import zhujiang.device.cluster.tlu2chi.TLUL2ChiBridge
import zhujiang.tilelink.{TLULBundle, TilelinkParams}

class ClusterInterconnectComplex(node: Node, cioParams: TilelinkParams)(implicit p: Parameters) extends ZJModule {
  private val sharePeriNum = 1
  private val privatePeriNum = 1
  private val clusterHub = Module(new ClusterHub(node))
  private val chi2tl = Module(new TLULBridge(clusterHub.io.peripheral.node, 64, 3))
  private val cioXbar = Module(new CioXBar(Seq.fill(node.cpuNum)(cioParams)))
  private val periXbar = Module(new PeriXBar(Seq(chi2tl.tl.params, cioXbar.io.downstream.head.params), sharePeriNum, node.cpuNum, privatePeriNum))
  private val tl2chi = Module(new TLUL2ChiBridge(clusterHub.io.cio.node, cioXbar.io.downstream.last.params))
  private val periParams = periXbar.io.downstream.head.params
  private val cpuCtrlSeq = Seq.fill(node.cpuNum)(Module(new CpuCtrl(periParams)))
  private val pllCtrl = Module(new ClusterPLL(periParams))

  private val sharePeriPortSeq = periXbar.io.downstream.take(sharePeriNum)
  private val privatePeriPortSeq = for(i <- 0 until node.cpuNum) yield {
    periXbar.io.downstream.drop(sharePeriNum).slice(i * privatePeriNum, i * privatePeriNum + 1)
  }

  chi2tl.icn <> clusterHub.io.peripheral
  periXbar.io.upstream.head <> chi2tl.tl
  periXbar.io.upstream.last <> cioXbar.io.downstream.head
  tl2chi.tlm <> cioXbar.io.downstream.last
  clusterHub.io.cio <> tl2chi.icn

  val io = IO(new Bundle {
    val icn = new ClusterDeviceBundle(node)
    val l2cache = new IcnBundle(clusterHub.io.l2cache.node)
    val cio = Vec(node.cpuNum, Flipped(new TLULBundle(cioParams)))
    val cpu = Flipped(new ClusterMiscWires(node))
    val dft = Output(new DftWires)
    val pllCfg = Output(Vec(8, UInt(32.W)))
    val pllLock = Input(Bool())
  })

  io.icn <> clusterHub.io.icn
  clusterHub.io.l2cache <> io.l2cache
  io.cpu <> clusterHub.io.cpu
  io.dft := clusterHub.io.dft
  cioXbar.misc.chip := clusterHub.io.cpu.mhartid(0)(clusterIdBits - 1, nodeAidBits)
  pllCtrl.tls <> sharePeriPortSeq.head
  io.pllCfg := pllCtrl.io.cfg
  pllCtrl.io.lock := io.pllLock

  for(i <- 0 until node.cpuNum) {
    cioXbar.io.upstream(i) <> io.cio(i)
    cpuCtrlSeq(i).tls <> privatePeriPortSeq(i).head
    cpuCtrlSeq(i).io.defaultBootAddr := clusterHub.io.cpu.resetVector(i)
    cpuCtrlSeq(i).io.defaultEnable := clusterHub.io.cpu.resetEnable(i)
    io.cpu.resetVector(i) := cpuCtrlSeq(i).io.cpuBootAddr
    io.cpu.resetEnable(i) := cpuCtrlSeq(i).io.cpuReset
    cioXbar.misc.core(i) := clusterHub.io.cpu.mhartid(i)
    periXbar.misc.core(i) := clusterHub.io.cpu.mhartid(i)
  }
}
