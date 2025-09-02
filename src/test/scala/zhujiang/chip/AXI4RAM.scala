package zhujiang.chip

import chisel3._
import chisel3.util._
import freechips.rocketchip.amba.axi4.{AXI4Buffer, AXI4Delayer, AXI4MasterNode, AXI4MasterParameters, AXI4MasterPortParameters}
import freechips.rocketchip.diplomacy.{AddressSet, IdRange}
import org.chipsalliance.cde.config.Parameters
import org.chipsalliance.diplomacy.lazymodule._
import zhujiang.axi.{AxiBundle, AxiParams}
import zhujiang.{HasZJParams, ZJModule}

class AXI4RAM
(
  address: Seq[AddressSet],
  memByte: Long,
  executable: Boolean = true,
  beatBytes: Int = 8,
  burstLen: Int = 16,
)(implicit p: Parameters) extends AXI4SlaveModule(address, executable, beatBytes, burstLen) {
  class Impl extends AXI4SlaveModuleImp(this)
  lazy val module = new Impl {
    val offsetBits = log2Up(memByte)
    require(address.nonEmpty)
    val baseAddress = address.head.base

    def index(addr: UInt) = ((addr - baseAddress.U)(offsetBits - 1, 0) >> log2Ceil(beatBytes)).asUInt

    def inRange(addr: UInt) = addr < (baseAddress + memByte).U

    val wIdx = index(waddr) + writeBeatCnt
    val rIdx = index(raddr) + readBeatCnt
    val wen = in.w.fire && inRange(waddr)
    require(beatBytes >= 8)
    val mem = Module(new SparseMem(log2Ceil(memByte / beatBytes), beatBytes * 8, beatBytes))
    mem.io.i_ck := clock
    mem.io.i_ra := rIdx
    mem.io.i_wa := wIdx
    mem.io.i_wd := in.w.bits.data
    mem.io.i_wm := in.w.bits.strb
    mem.io.i_we := wen
    in.r.bits.data := mem.io.o_rd
  }
}

class SparseMem(aw:Int, dw:Int, mw:Int) extends BlackBox with HasBlackBoxInline {
  val io = IO(new Bundle {
    val i_ck = Input(Clock())
    val i_ra = Input(UInt(aw.W))
    val i_wa = Input(UInt(aw.W))
    val i_wd = Input(UInt(dw.W))
    val i_wm = Input(UInt(mw.W))
    val i_we = Input(Bool())
    val o_rd = Output(UInt(dw.W))
  })
  private val modName = s"SparseMemA${aw}D${dw}M${mw}"
  override val desiredName = modName
  setInline(s"$modName.sv",
    s"""// VCS coverage exclude_file
       |module $modName (
       |  input  logic          i_ck,
       |  input  logic [${aw - 1}: 0] i_ra,
       |  input  logic [${aw - 1}: 0] i_wa,
       |  input  logic [${dw - 1}: 0] i_wd,
       |  input  logic [${mw - 1}: 0] i_wm,
       |  input  logic         i_we,
       |  output logic [${dw - 1}: 0] o_rd
       |);
       |  bit   [${dw - 1}: 0] mem [bit[${aw - 1}: 0]];
       |  logic [${dw - 1}: 0] mask;
       |
       |  generate
       |    for (genvar i = 0; i < $mw; i++) begin
       |      assign mask[i * 8 +: 8] = {8{i_wm[i]}};
       |    end
       |  endgenerate
       |
       |  always @(posedge i_ck) begin
       |    if(i_we) begin
       |      if(mem.exists(i_wa) == 1) begin
       |        mem[i_wa] = (i_wd & mask) | (mem[i_wa] & ~mask);
       |      end else begin
       |        mem[i_wa] = (i_wd & mask) | ($dw'b0 & ~mask);;
       |      end
       |    end
       |  end
       |
       |  always @(negedge i_ck) begin
       |    if(mem.exists(i_ra) == 1) begin
       |      o_rd <= mem[i_ra];
       |    end else begin
       |      o_rd <= $dw'b0;
       |    end
       |  end
       |endmodule""".stripMargin)
}

class AxiRamInner(axiP:AxiParams)(implicit p:Parameters) extends LazyModule with HasZJParams {
  private val memDplmcMstParams = AXI4MasterPortParameters(
    masters = Seq(
      AXI4MasterParameters(
        name = "mem",
        id = IdRange(0, 1 << axiP.idBits)
      )
    )
  )
  private val mstNode = AXI4MasterNode(Seq(memDplmcMstParams))
  private val ram = LazyModule(new AXI4RAM (
    address = Seq(AddressSet(0, (1L << (raw - ciIdBits)) - 1)),
    memByte = 1L << (raw - ciIdBits),
    executable = true,
    beatBytes = dw / 8,
    burstLen = 1
  ))

  ram.node :*= AXI4Buffer.chainNode(60) :*= AXI4Delayer(0.25) :*= AXI4Buffer.chainNode(60) :*= mstNode

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val axi = mstNode.makeIOs()
  }
}

class AxiRam(axiP:AxiParams)(implicit p:Parameters) extends ZJModule {
  val axi = IO(Flipped(new AxiBundle(axiP)))
  private val ram = LazyModule(new AxiRamInner(axiP))
  private val _ram = Module(ram.module)
  private def connectByName[T <: Bundle, K <: Bundle](sink: ReadyValidIO[T], src: ReadyValidIO[K]):Unit = {
    sink.valid := src.valid
    src.ready := sink.ready
    sink.bits := DontCare
    val recvMap = sink.bits.elements.map(e => (e._1.toLowerCase, e._2))
    val sendMap = src.bits.elements.map(e => (e._1.toLowerCase, e._2))
    for((name, data) <- recvMap) {
      if(sendMap.contains(name)) data := sendMap(name).asTypeOf(data)
    }
  }

  connectByName(_ram.axi.head.aw, axi.aw)
  connectByName(_ram.axi.head.ar, axi.ar)
  connectByName(_ram.axi.head.w, axi.w)
  connectByName(axi.r, _ram.axi.head.r)
  connectByName(axi.b, _ram.axi.head.b)
}
