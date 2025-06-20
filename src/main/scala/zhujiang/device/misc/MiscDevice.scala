package zhujiang.device.misc

import chisel3._
import chisel3.util.{Arbiter, Cat, Decoupled, Queue, log2Ceil}
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.debug.HardwareAssertionKey
import xs.utils.mbist.MbistPipeline
import xs.utils.queue.FastQueue
import xs.utils.sram.{SinglePortSramTemplate, SpSramReq}
import zhujiang.axi.{AxiBundle, AxiLiteParams, RFlit}
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
  private val arb = Module(new Arbiter(new SpSramReq(UInt(entryBits.W), hwaP.hwaDevDepth, 1), 2))
  private val rq = FastQueue(io.axi.ar)
  private val wq = FastQueue(io.hwa)
  private val wcnt = RegInit(0.U(log2Ceil(hwaP.hwaDevDepth + 1).W))
  private val oq = Module(new FastQueue(new RFlit(axiP), size = 2, deqDataNoX = false))

  ram.io.req <> arb.io.out

  io.axi.w.ready := true.B
  io.axi.b.valid := io.axi.aw.valid
  io.axi.aw.ready := io.axi.b.ready
  io.axi.b.bits := DontCare
  io.axi.b.bits.resp := "b10".U
  io.axi.b.bits.id := io.axi.aw.bits.id

  // Read S0
  private val rp = arb.io.in.head
  private val rvalid = RegInit(false.B)
  private val rpipe = oq.io.enq.ready || !rvalid
  rp.valid := rq.valid && rpipe
  rp.bits.data := DontCare
  rp.bits.write := false.B
  rp.bits.addr := rq.bits.addr.head(log2Ceil(hwaP.hwaDevDepth))
  rq.ready := rp.ready && rpipe
  // Read S1
  when(rpipe) {
    rvalid := rp.fire
  }
  // Read S2
  oq.io.enq.valid := rvalid
  oq.io.enq.bits := DontCare
  oq.io.enq.bits.data := ram.io.resp.bits.data.asUInt
  oq.io.enq.bits.resp := "b00".U
  // Read S3
  io.axi.r <> oq.io.deq

  private val wp = arb.io.in.last
  private val wd = Wire(new HwaCollectorEntry)
  wd.id := wq.bits.id
  wd.src := wq.bits.src
  wd.vld := true.B

  wp.valid := wq.valid && wcnt =/= hwaP.hwaDevDepth.U
  wq.ready := wp.ready && wcnt =/= hwaP.hwaDevDepth.U
  wp.bits.data.head := wd.asUInt
  wp.bits.addr := wcnt
  wp.bits.write := true.B
  when(wp.fire && wcnt =/= hwaP.hwaDevDepth.U) {
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
  })
  private val resetReg = RegInit("b11".U)
  private val resetDriveReg = RegNext(resetReg, "b11".U)
  private val resetState0 = withReset(io.resetState(0).asAsyncReset) { RegNext(false.B, true.B) }
  private val resetState1 = withReset(io.resetState(1).asAsyncReset) { RegNext(false.B, true.B) }
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
  })
  private val resetDev = Module(new ResetDevice)
  resetDev.io.resetState := io.icn.resetState.get
  io.icn.resetInject.get := resetDev.io.resetInject
  io.onReset := resetDev.io.onReset
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