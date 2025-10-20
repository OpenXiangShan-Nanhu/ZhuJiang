package zhujiang.axi

import chisel3._
import chisel3.experimental.noPrefix
import chisel3.util._
import xs.utils.PickOneLow

class AxiWidthAdapterWBundle(axiP: AxiParams) extends Bundle {
  val addr = UInt(axiP.addrBits.W)
  val size = UInt(axiP.sizeBits.W)
  val id = UInt(axiP.idBits.W)

  def := (in: AWFlit): Unit = {
    this.addr := in.addr
    this.size := in.size
    this.id   := in.id
  }
}

class AxiWidthAdapterRBundle(axiP: AxiParams, outstanding: Int) extends Bundle {
  val addr = UInt(axiP.addrBits.W)
  val size = UInt(axiP.sizeBits.W)
  val id = UInt(axiP.idBits.W)
  val nid = UInt(log2Ceil(outstanding).W)
}

class AxiWidthAdapter(slvParams: AxiParams, mstParams: AxiParams, outstanding:Int) extends Module {
  val io = IO(new Bundle {
    val mst = Flipped(new AxiBundle(mstParams))
    val slv = new AxiBundle(slvParams)
  })
  private val mdw = mstParams.dataBits
  private val sdw = slvParams.dataBits
  private val seg = sdw / mdw
  private val awq = Module(new Queue(new AxiWidthAdapterWBundle(mstParams), entries = outstanding))
  private val wq = Module(new Queue(new WFlit(mstParams), entries = 2))
  private val rq = Module(new Queue(new RFlit(slvParams), entries = 1, pipe = true))
  private val arvld = RegInit(VecInit(Seq.fill(outstanding)(false.B)))
  private val arinfo = Reg(Vec(outstanding, new AxiWidthAdapterRBundle(mstParams, outstanding)))
  private val arsel = PickOneLow(arvld)
  private val setNid = PopCount(arinfo.zip(arvld).map{case(i, v) => (i.id === io.mst.ar.bits.id) && v})
  private val infoSelOH = Wire(Vec(outstanding, Bool()))
  require(mdw <= sdw, "AXI width adapter dose not support wide-to-narrow convert for now!")

  for(i <- arvld.indices) noPrefix {
    val rFireMayHit = WireInit(io.mst.r.valid && io.mst.r.ready && io.mst.r.bits.id === arinfo(i).id && arvld(i))
    rFireMayHit.suggestName(s"r_fire_may_hit_$i")
    val arFireHit = WireInit(io.mst.ar.fire && arsel.bits(i))
    arFireHit.suggestName(s"ar_fire_hit_$i")

    when(arFireHit) {
      arvld(i) := true.B
    }.elsewhen(rFireMayHit && io.mst.r.bits._last && arinfo(i).nid === 0.U) {
      arvld(i) := false.B
    }

    when(arFireHit) {
      arinfo(i).size := io.mst.ar.bits.size
      arinfo(i).id := io.mst.ar.bits.id
    }

    when(arFireHit) {
      arinfo(i).nid := setNid
    }.elsewhen(rFireMayHit && io.mst.r.bits._last && arinfo(i).nid =/= 0.U) {
      arinfo(i).nid := arinfo(i).nid - 1.U
    }

    when(arFireHit) {
      arinfo(i).addr := io.mst.ar.bits.addr
    }.elsewhen(rFireMayHit && arinfo(i).nid === 0.U) {
      arinfo(i).addr := arinfo(i).addr + (1.U << arinfo(i).size)
    }

    infoSelOH(i) := arvld(i) && arinfo(i).id === io.slv.r.bits.id && arinfo(i).nid === 0.U
  }

  //AW Channel Connection
  awq.io.enq.valid := io.mst.aw.valid && io.slv.aw.ready
  awq.io.enq.bits := io.mst.aw.bits
  io.slv.aw.valid := io.mst.aw.valid && awq.io.enq.ready
  io.slv.aw.bits := io.mst.aw.bits
  io.mst.aw.ready := io.slv.aw.ready && awq.io.enq.ready

  //AR Channel Connection
  io.slv.ar.valid := io.mst.ar.valid && arsel.valid
  io.slv.ar.bits := io.mst.ar.bits
  io.mst.ar.ready := io.slv.ar.ready && arsel.valid

  //W Channel Connection
  private val strb = Wire(Vec(seg, UInt((mdw / 8).W)))
  private val waddrcvt = if(sdw > mdw) awq.io.deq.bits.addr(log2Ceil(sdw / 8) - 1, log2Ceil(mdw / 8)) else 0.U
  strb.zipWithIndex.foreach({case(s, i) => s := Mux(waddrcvt === i.U, wq.io.deq.bits.strb, 0.U)})

  wq.io.enq <> io.mst.w
  io.slv.w.valid := wq.io.deq.valid && awq.io.deq.valid
  io.slv.w.bits := wq.io.deq.bits
  io.slv.w.bits.data := Fill(seg, wq.io.deq.bits.data)
  io.slv.w.bits.strb := strb.asUInt
  wq.io.deq.ready := io.slv.w.ready && awq.io.deq.valid
  awq.io.deq.ready := io.slv.w.ready && wq.io.deq.valid && wq.io.deq.bits._last

  //B Channel Connection
  io.mst.b <> io.slv.b

  //R Channel Connection
  private val infoSelOHReg = RegEnable(infoSelOH, io.slv.r.fire)
  private val infoSel = Mux1H(infoSelOHReg, arinfo)
  private val raddrcvt = if(sdw > mdw) infoSel.addr(log2Ceil(sdw / 8) - 1, log2Ceil(mdw / 8)) else 0.U
  private val rdata = rq.io.deq.bits.data.asTypeOf(Vec(seg, UInt(mdw.W)))
  rq.io.enq <> io.slv.r
  io.mst.r <> rq.io.deq
  io.mst.r.bits.data := rdata(raddrcvt)

  when(io.slv.r.fire) {
    assert(PopCount(infoSelOH) === 0.U, s"Multiple R entries are hit!")
  }
}

object AxiWidthAdapter {
  def apply(slv: AxiBundle, mst:AxiBundle, outstanding:Int):AxiWidthAdapter = {
    val wadpt = Module(new AxiWidthAdapter(slv.params, mst.params, outstanding))
    wadpt.io.mst <> mst
    slv <> wadpt.io.slv
    wadpt
  }
  def apply(slv: AxiBundle, mst:AxiBundle):AxiWidthAdapter = apply(slv, mst, 4)
}