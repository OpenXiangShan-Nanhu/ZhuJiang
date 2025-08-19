package zhujiang.device.misc

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.{DFTResetSignals, ResetGen}
import xs.utils.debug.HardwareAssertionKey
import xs.utils.mbist.MbistPipeline
import xs.utils.queue.FastQueue
import xs.utils.sram.{SinglePortSramTemplate, SpSramReq, SpSramResp}
import zhujiang.axi._
import zhujiang.chi.FlitHelper.extractHwaId
import zhujiang.{ZJBundle, ZJModule}
import zhujiang.chi.{NodeIdBundle, RingFlit}

class ZJDebugBundle(implicit p:Parameters) extends ZJBundle {
  val src = UInt(nodeNidBits.W)
  val id = UInt(p(HardwareAssertionKey).maxInfoBits.W)
}

class HwaCollectorEntry(implicit p:Parameters) extends ZJBundle {
  val vld = Bool()
  val src = UInt(niw.W)
  val id = UInt(p(HardwareAssertionKey).maxInfoBits.W)
}

class AxiLiteWrMachine(axiP:AxiParams)(implicit p:Parameters) extends Module {
  private val hwaP = p(HardwareAssertionKey)
  private val entryBits = (new HwaCollectorEntry).getWidth

  val io = IO(new Bundle {
    val aw = Flipped(Decoupled(new AWFlit(axiP)))
    val w = Flipped(Decoupled(new WFlit(axiP)))
    val b = Decoupled(new BFlit(axiP))
    val req = Decoupled(new SpSramReq(UInt(entryBits.W), hwaP.hwaDevDepth, 1))
  })
  private val awvld = RegInit(false.B)
  private val awbits = RegEnable(io.aw.bits, io.aw.fire)
  private val wvld = RegInit(false.B)
  private val wbits = RegEnable(io.w.bits, io.w.fire)
  private val bvld = RegInit(false.B)

  io.aw.ready := !awvld
  io.w.ready := awvld && !wvld
  io.b.valid := bvld
  io.b.bits.user := awbits.user
  io.b.bits.resp := 0.U
  io.b.bits.id := awbits.id
  when(io.b.fire) {
    awvld := false.B
  }.elsewhen(io.aw.fire) {
    awvld := true.B
  }
  when(io.b.fire) {
    wvld := false.B
  }.elsewhen(io.w.fire) {
    wvld := true.B
  }
  when(io.b.fire) {
    bvld := false.B
  }.elsewhen(io.req.fire) {
    bvld := true.B
  }
  io.req.valid := awvld && wvld
  io.req.bits.addr := awbits.addr.head(log2Ceil(hwaP.hwaDevDepth))
  io.req.bits.data.foreach(_ := wbits.data)
  io.req.bits.write := true.B
}

class AxiLiteRdMachine(axiP:AxiParams)(implicit p:Parameters) extends Module {
  private val hwaP = p(HardwareAssertionKey)
  private val entryBits = (new HwaCollectorEntry).getWidth
  val io = IO(new Bundle {
    val ar = Flipped(Decoupled(new AWFlit(axiP)))
    val r = Decoupled(new RFlit(axiP))
    val req = Decoupled(new SpSramReq(UInt(entryBits.W), hwaP.hwaDevDepth, 1))
    val resp = Flipped(Valid(new SpSramResp(UInt(entryBits.W), 1)))
  })
  private val arvld = RegInit(false.B)
  private val arbits = RegEnable(io.ar.bits, io.ar.fire)
  private val rvld = RegInit(false.B)
  private val rdata = RegEnable(io.resp.bits.data.head, io.resp.valid)
  private val reqFired = RegInit(false.B)

  io.ar.ready := !arvld
  io.r.valid := rvld
  io.r.bits.data := rdata
  io.r.bits.id := arbits.id
  io.r.bits.resp := 0.U
  io.r.bits.user := arbits.user
  when(io.r.fire) {
    arvld := false.B
  }.elsewhen(io.ar.fire) {
    arvld := true.B
  }
  when(io.r.fire) {
    rvld := false.B
  }.elsewhen(io.resp.valid) {
    rvld := true.B
  }
  when(io.r.fire) {
    reqFired := false.B
  }.elsewhen(io.req.fire) {
    reqFired := true.B
  }
  io.req.valid := arvld && !reqFired
  io.req.bits.addr := arbits.addr.head(log2Ceil(hwaP.hwaDevDepth))
  io.req.bits.data.foreach(_ := DontCare)
  io.req.bits.write := false.B
}

class HardwareAssertionDevice(implicit p:Parameters) extends ZJModule {
  private val hwaP = p(HardwareAssertionKey)
  private val axiP = new AxiLiteParams(addrBits = log2Ceil(hwaP.hwaDevDepth) + 2, dataBits = 32, attr = "debug")
  val io = IO(new Bundle {
    val axi = Flipped(new AxiBundle(axiP))
    val hwa = Flipped(Decoupled(new ZJDebugBundle))
    val intr = Output(Bool())
  })
  require(io.hwa.bits.getWidth < axiP.dataBits)
  private val entryBits = (new HwaCollectorEntry).getWidth
  private val ram = Module(new SinglePortSramTemplate(
    gen = UInt(entryBits.W),
    set = hwaP.hwaDevDepth,
    way = 1,
    shouldReset = true,
    latency = 2,
    suffix  = "_hwa",
    hasMbist = hasMbist
  ))
  private val wrm = Module(new AxiLiteWrMachine(axiP))
  private val rdm = Module(new AxiLiteRdMachine(axiP))
  private val arb = Module(new Arbiter(new SpSramReq(UInt(entryBits.W), hwaP.hwaDevDepth, 1), 3))
  private val wq = FastQueue(io.hwa)
  private val wcnt = RegInit(0.U(log2Ceil(hwaP.hwaDevDepth + 1).W))

  wrm.io.aw <> io.axi.aw
  wrm.io.w <> io.axi.w
  rdm.io.ar <> io.axi.ar
  io.axi.b <> wrm.io.b
  io.axi.r <> rdm.io.r
  arb.io.in(0) <> wrm.io.req
  arb.io.in(1) <> rdm.io.req
  ram.io.req <> arb.io.out
  rdm.io.resp := ram.io.resp

  private val wp = arb.io.in(2)
  private val wd = Wire(new HwaCollectorEntry)
  wd.id := wq.bits.id
  wd.src := wq.bits.src
  wd.vld := true.B

  wp.valid := wq.valid && wcnt < hwaP.hwaDevDepth.U
  wq.ready := wp.ready || wcnt >= hwaP.hwaDevDepth.U
  wp.bits.data.head := wd.asUInt
  wp.bits.addr := wcnt
  wp.bits.write := true.B
  when(wp.fire && wcnt < hwaP.hwaDevDepth.U) {
    wcnt := wcnt + 1.U
  }
  when(wq.fire) {
    printf(cf"[Error]: Hardware assertion is collected! SrcID: 0x${wq.bits.src}%x AsrtID: ${wq.bits.id}")
  }
  io.intr := RegNext(wcnt.orR)
}

class ResetDevice extends Module {
  val io = IO(new Bundle {
    val resetInject = Output(Vec(2, Bool()))
    val resetState = Input(Vec(2, Bool()))
    val onReset = Output(Bool())
    val dft = Input(new DFTResetSignals)
  })
  private val resetReg = RegInit("b11".U)
  private val resetDriveReg = RegNext(resetReg, "b11".U)
  private val resetState0 = withReset(io.resetState(0).asAsyncReset) {ResetGen(2, Some(io.dft)).asBool}
  private val resetState1 = withReset(io.resetState(1).asAsyncReset) {ResetGen(2, Some(io.dft)).asBool}
  io.resetInject(0) := resetDriveReg(0)
  io.resetInject(1) := resetDriveReg(1)
  when(resetReg === 3.U) {
    resetReg := 1.U
  }.elsewhen(resetReg === 1.U && resetState1 === false.B) {
    resetReg := 0.U
  }
  io.onReset := RegNext(resetState0 | resetState1)
}

class MiscDevice(node: Node)(implicit p:Parameters) extends ZJModule {
  require(node.nodeType == NodeType.M)
  private val hwaP = p(HardwareAssertionKey)
  private val hwaDev = Option.when(hwaP.enable)(Module(new HardwareAssertionDevice))
  val io = IO(new Bundle {
    val icn = new DeviceIcnBundle(node, true)
    val onReset = Output(Bool())
    val axi = Option.when(hwaP.enable)(Flipped(new AxiBundle(hwaDev.get.io.axi.params)))
    val intr = Option.when(hwaP.enable)(Output(Bool()))
    val dft = Input(new DFTResetSignals)
  })
  private val resetDev = Module(new ResetDevice)
  resetDev.io.resetState := io.icn.resetState.get
  io.icn.resetInject.get := resetDev.io.resetInject
  io.onReset := resetDev.io.onReset
  resetDev.io.dft := io.dft
  if(hwaP.enable) {
    val dbgFlit = io.icn.rx.debug.get.bits.asTypeOf(new RingFlit(debugFlitBits))
    hwaDev.get.io.hwa.valid := io.icn.rx.debug.get.valid
    hwaDev.get.io.hwa.bits.src := dbgFlit.SrcID.asTypeOf(new NodeIdBundle).nid
    hwaDev.get.io.hwa.bits.id := extractHwaId(dbgFlit)
    io.icn.rx.debug.get.ready := hwaDev.get.io.hwa.ready
    io.axi.get <> hwaDev.get.io.axi
    io.intr.get := hwaDev.get.io.intr
    MbistPipeline.PlaceMbistPipeline(1, "MbistPipelineMn", hasMbist)
  }
}