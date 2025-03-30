package zhujiang.axi

import chisel3._
import chisel3.util.{Decoupled, DecoupledIO}

case class AxiParams(
  addrBits: Int = 32,
  idBits: Int = 5,
  userBits: Int = 0,
  dataBits: Int = 64,
  attr:String = "",
  lenBits: Int = 8,
  sizeBits: Int = 3,
  burstBits: Int = 2,
  cacheBits: Int = 4,
  lockBits: Int = 2,
  qosBits:Int = 4,
  regionBits:Int = 4
)

class AxiLiteParams(addrBits:Int, idBits:Int, dataBits:Int, attr:String = "") extends AxiParams(
  addrBits = addrBits,
  idBits = idBits,
  userBits = 0,
  dataBits = dataBits,
  lenBits = 0,
  sizeBits = 0,
  burstBits = 0,
  cacheBits = 0,
  lockBits = 0,
  qosBits = 0,
  regionBits = 0
)

class AWFlit(params: AxiParams) extends Bundle {
  val id = UInt(params.idBits.W)
  val addr = UInt(params.addrBits.W)
  val len = UInt(params.lenBits.W)
  val size = UInt(params.sizeBits.W)
  val burst = UInt(params.burstBits.W)
//  val lock = UInt(params.lockBits.W)
  val cache = UInt(params.cacheBits.W)
//  val prot = UInt(3.W)
//  val qos = UInt(params.qosBits.W)
//  val region = UInt(params.regionBits.W)
  val user = UInt(params.userBits.W)
}

class ARFlit(params: AxiParams) extends Bundle {
  val id = UInt(params.idBits.W)
  val addr = UInt(params.addrBits.W)
  val len = UInt(params.lenBits.W)
  val size = UInt(params.sizeBits.W)
  val burst = UInt(params.burstBits.W)
//  val lock = UInt(params.lockBits.W)
  val cache = UInt(params.cacheBits.W)
//  val prot = UInt(3.W)
//  val qos = UInt(params.qosBits.W)
//  val region = UInt(params.regionBits.W)
  val user = UInt(params.userBits.W)
}

class WFlit(params: AxiParams) extends Bundle {
  val data = UInt(params.dataBits.W)
  val strb = UInt((params.dataBits / 8).W)
  val last = Bool()
  val user = UInt(params.userBits.W)
}

class RFlit(params: AxiParams) extends Bundle {
  val id = UInt(params.idBits.W)
  val data = UInt(params.dataBits.W)
  val resp = UInt(2.W)
  val last = Bool()
  val user = UInt(params.userBits.W)
}

class BFlit(params: AxiParams) extends Bundle {
  val id = UInt(params.idBits.W)
  val resp = UInt(2.W)
  val user = UInt(params.userBits.W)
}

object AxiUtils {
  def extConn(extnl:ExtAxiBundle, intnl: AxiBundle):Unit = {
    for((chn, bd) <- intnl.elements) {
      val dcp = bd.asInstanceOf[DecoupledIO[Bundle]]
      extnl.elements(s"${chn}valid") <> dcp.valid
      extnl.elements(s"${chn}ready") <> dcp.ready
      for((field, sig) <- dcp.bits.elements) {
        extnl.elements(s"$chn$field") <> sig
      }
    }
  }

  def getExtnl(intnl: AxiBundle):ExtAxiBundle = {
    val extnl = Wire(new ExtAxiBundle(intnl.params))
    extnl <> intnl
    extnl
  }

  def getIntnl(extnl: ExtAxiBundle):AxiBundle = {
    val intnl = Wire(new AxiBundle(extnl.params))
    intnl <> extnl
    intnl
  }
}

class AxiBundle(val params: AxiParams) extends Bundle {
  val aw = Decoupled(new AWFlit(params))
  val ar = Decoupled(new ARFlit(params))
  val w = Decoupled(new WFlit(params))
  val b = Flipped(Decoupled(new BFlit(params)))
  val r = Flipped(Decoupled(new RFlit(params)))

  def <>(that: ExtAxiBundle):Unit = AxiUtils.extConn(that, this)
}

class ExtAxiBundle(val params: AxiParams) extends Bundle {
  val awvalid = Output(Bool())
  val awready = Input(Bool())
  val awid = Output(UInt(params.idBits.W))
  val awaddr = Output(UInt(params.addrBits.W))
  val awlen = Output(UInt(params.lenBits.W))
  val awsize = Output(UInt(params.sizeBits.W))
  val awburst = Output(UInt(params.burstBits.W))
  //  val awlock = Output(UInt(params.lockBits.W))
  val awcache = Output(UInt(params.cacheBits.W))
  //  val awprot = Output(UInt(3.W))
  //  val awqos = Output(UInt(params.qosBits.W))
  //  val awregion = Output(UInt(params.regionBits.W))
  val awuser = Output(UInt(params.userBits.W))

  val arvalid = Output(Bool())
  val arready = Input(Bool())
  val arid = Output(UInt(params.idBits.W))
  val araddr = Output(UInt(params.addrBits.W))
  val arlen = Output(UInt(params.lenBits.W))
  val arsize = Output(UInt(params.sizeBits.W))
  val arburst = Output(UInt(params.burstBits.W))
  //  val arlock = Output(UInt(params.lockBits.W))
  val arcache = Output(UInt(params.cacheBits.W))
  //  val arprot = Output(UInt(3.W))
  //  val arqos = Output(UInt(params.qosBits.W))
  //  val arregion = Output(UInt(params.regionBits.W))
  val aruser = Output(UInt(params.userBits.W))

  val wvalid = Output(Bool())
  val wready = Input(Bool())
  val wdata = Output(UInt(params.dataBits.W))
  val wstrb = Output(UInt((params.dataBits / 8).W))
  val wlast = Output(Bool())
  val wuser = Output(UInt(params.userBits.W))

  val bvalid = Input(Bool())
  val bready = Output(Bool())
  val bid = Input(UInt(params.idBits.W))
  val bresp = Input(UInt(2.W))
  val buser = Input(UInt(params.userBits.W))

  val rvalid = Input(Bool())
  val rready = Output(Bool())
  val rid = Input(UInt(params.idBits.W))
  val rdata = Input(UInt(params.dataBits.W))
  val rresp = Input(UInt(2.W))
  val rlast = Input(Bool())
  val ruser = Input(UInt(params.userBits.W))

  def <>(that: AxiBundle):Unit = AxiUtils.extConn(this, that)
}