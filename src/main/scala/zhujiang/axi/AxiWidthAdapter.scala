package zhujiang.axi

import chisel3._
import chisel3.util._
import xs.utils.PickOneLow

class AxiWidthAdapterInfoBundle(axiP: AxiParams) extends Bundle {
  val addr = UInt(axiP.addrBits.W)
  val size = UInt(axiP.sizeBits.W)
  val id = UInt(axiP.idBits.W)

  def := (in: AWFlit): Unit = {
    this.addr := in.addr
    this.size := in.size
    this.id   := in.id
  }
  def := (in: ARFlit): Unit = {
    this.addr := in.addr
    this.size := in.size
    this.id   := in.id
  }
}

class AxiWidthAdapter(slvParams: AxiParams, mstParams: AxiParams, outstanding:Int) extends Module {
  val io = IO(new Bundle {
    val mst = Flipped(new AxiBundle(mstParams))
    val slv = new AxiBundle(slvParams)
  })
  private val mdw = mstParams.dataBits
  private val sdw = slvParams.dataBits
  private val seg = sdw / mdw
  private val awq = Module(new Queue(new AxiWidthAdapterInfoBundle(mstParams), entries = outstanding))
  private val wq = Module(new Queue(new WFlit(mstParams), entries = 2))
  private val arvld = RegInit(VecInit(Seq.fill(outstanding)(false.B)))
  private val arinfo = Reg(Vec(outstanding, new AxiWidthAdapterInfoBundle(mstParams)))
  private val arsel = PickOneLow(arvld)
  require(mdw <= sdw, "AXI width adapter dose not support wide-to-narrow convert for now!")

  for(i <- arvld.indices) {
    when(io.mst.ar.fire && arsel.bits(i)) {
      arvld(i) := true.B
    }.elsewhen(io.slv.r.fire && io.slv.r.bits.id === i.U) {
      arvld(i) := false.B
    }
    when(io.mst.ar.fire && arsel.bits(i)) {
      arinfo(i) := io.mst.ar.bits
    }
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
  io.slv.ar.bits.id := OHToUInt(arsel.bits)
  io.mst.ar.ready := io.slv.ar.ready && arsel.valid

  //W Channel Connection
  private val strb = Wire(Vec(seg, UInt((mdw / 8).W)))
  private val waddrcvt = if(sdw > mdw) awq.io.deq.bits.addr(log2Ceil(sdw) - 1, log2Ceil(mdw)) else 0.U
  strb.zipWithIndex.foreach({case(s, i) => s := Mux(waddrcvt === i.U, wq.io.deq.bits.strb, 0.U)})

  wq.io.enq <> io.mst.w
  io.slv.w.valid := wq.io.deq.valid && awq.io.deq.valid
  io.slv.w.bits := wq.io.deq.bits
  io.slv.w.bits.data := Fill(seg, wq.io.deq.bits.data)
  io.slv.w.bits.strb := strb.asUInt
  wq.io.deq.ready := io.slv.w.ready && awq.io.deq.valid
  awq.io.deq.ready := io.slv.w.ready && wq.io.deq.valid && wq.io.deq.bits.last

  //B Channel Connection
  io.mst.b <> io.slv.b

  //R Channel Connection
  private val infoSel = arinfo(io.slv.r.bits.id(log2Ceil(outstanding) - 1, 0))
  private val raddrcvt = if(sdw > mdw) infoSel.addr(log2Ceil(sdw) - 1, log2Ceil(mdw)) else 0.U
  private val rdata = io.slv.r.bits.data.asTypeOf(Vec(seg, UInt(mdw.W)))
  io.mst.r.valid := io.slv.r.valid
  io.mst.r.bits := io.slv.r.bits
  io.mst.r.bits.data := rdata(raddrcvt)
  io.mst.r.bits.id := infoSel.id
  io.slv.r.ready := io.mst.r.ready
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