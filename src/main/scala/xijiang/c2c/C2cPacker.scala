package xijiang.c2c
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xs.utils.ResetRRArbiter
import zhujiang.{ZJBundle, ZJModule, ZJParametersKey}

class C2cIcnBundle(implicit p:Parameters) extends ZJBundle {
  val req = Decoupled(UInt(reqFlitBits.W))
  val rsp = Decoupled(UInt(respFlitBits.W))
  val dat = Decoupled(UInt(dataFlitBits.W))
  val snp = Decoupled(UInt(snoopFlitBits.W))
}

class C2cTxDispatcher(implicit p:Parameters) extends ZJModule {
  private val (nrReqSlots, nrReqDeq) = C2cUtils.getSlotsAndDeq(reqFlitBits)
  private val (nrRspSlots, nrRspDeq) = C2cUtils.getSlotsAndDeq(respFlitBits)
  private val (nrDatSlots, nrDatDeq) = C2cUtils.getSlotsAndDeq(dataFlitBits)
  private val (nrSnpSlots, nrSnpDeq) = C2cUtils.getSlotsAndDeq(snoopFlitBits)

  val io = IO(new Bundle {
    val rx = new Bundle {
      val reqSlots = Flipped(Vec(nrReqDeq, Decoupled(new C2cSlot)))
      val rspSlots = Flipped(Vec(nrRspDeq, Decoupled(new C2cSlot)))
      val snpSlots = Flipped(Vec(nrSnpDeq, Decoupled(new C2cSlot)))
      val datSlots = Flipped(Vec(nrDatDeq, Decoupled(new C2cSlot)))
      val reqGrant = Flipped(Decoupled(UInt(3.W)))
      val rspGrant = Flipped(Decoupled(UInt(3.W)))
      val datGrant = Flipped(Decoupled(UInt(3.W)))
      val snpGrant = Flipped(Decoupled(UInt(3.W)))
    }
    val tx = Decoupled(new C2cPayload)
  })
  require(nrReqSlots == 2)
  require(nrRspSlots == 1)
  require(nrDatSlots == 5)
  require(nrSnpSlots == 2)


  private val flitAllSlotsType = Vec(C2cUtils.nrSlotsPerFrame, Valid(new C2cSlot))

  private val oPipe = Module(new Queue(new C2cPayload, entries = 2))
  private val payloadArb = Module(new ResetRRArbiter(flitAllSlotsType, 2))

  private val datPayload = Wire(Decoupled(flitAllSlotsType))
  datPayload.bits.zip(io.rx.datSlots).foreach({ case(a, b) =>
    a.valid := b.valid
    a.bits := b.bits
    b.ready := datPayload.ready
  })
  datPayload.valid := datPayload.bits.map(_.valid).reduce(_ | _)

  private val combPayload = Wire(Decoupled(flitAllSlotsType))
  private val combArb = Module(new ResetRRArbiter(Bool(), 2))
  combArb.io.in.head.valid := io.rx.reqSlots.head.valid
  combArb.io.in.last.valid := io.rx.snpSlots.head.valid
  combArb.io.out.ready := combPayload.ready
  combArb.io.in.foreach(_.bits := DontCare)
  io.rx.reqSlots.foreach(_.ready := combArb.io.in.head.ready)
  io.rx.snpSlots.foreach(_.ready := combArb.io.in.last.ready)

  for(i <- 0 until nrReqSlots) {
    combPayload.bits(i).valid := Mux(combArb.io.chosen === 0.U, io.rx.reqSlots(i).valid, io.rx.snpSlots(i).valid)
    combPayload.bits(i).bits := Mux(combArb.io.chosen === 0.U, io.rx.reqSlots(i).bits, io.rx.snpSlots(i).bits)
  }
  combPayload.bits.last.valid := io.rx.rspSlots.head.valid
  combPayload.bits.last.bits := io.rx.rspSlots.head.bits
  io.rx.rspSlots.head.ready := combPayload.ready

  combPayload.valid := combPayload.bits.map(_.valid).reduce(_ | _)

  payloadArb.io.in.head <> datPayload
  payloadArb.io.in.last <> combPayload

  private val grantSeq = Seq(io.rx.reqGrant, io.rx.rspGrant, io.rx.datGrant, io.rx.snpGrant)
  private val txGrantSeq = Seq(oPipe.io.enq.bits.reqGrants, oPipe.io.enq.bits.rspGrants, oPipe.io.enq.bits.datGrants, oPipe.io.enq.bits.snpGrants)
  private val grantReq = Cat(grantSeq.map(_.valid)).orR
  oPipe.io.enq.valid := grantReq || payloadArb.io.out.valid
  oPipe.io.enq.bits.slots := payloadArb.io.out.bits
  payloadArb.io.out.ready := oPipe.io.enq.ready
  txGrantSeq.zip(grantSeq).foreach({ case(a, b) =>
    a.valid := b.valid
    a.bits := b.bits
    b.ready := oPipe.io.enq.ready
  })
  io.tx <> oPipe.io.deq
}

class C2cBundle extends Bundle {
  val tx = Decoupled(UInt(256.W))
  val rx = Input(Valid(UInt(256.W)))
  def <=>(that:C2cBundle):Unit = {
    this.rx.valid := that.tx.valid
    this.rx.bits := that.tx.bits
    that.tx.ready := true.B

    that.rx.valid := this.tx.valid
    that.rx.bits := this.tx.bits
    this.tx.ready := true.B
  }
}

class C2cPacker(implicit p:Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val icn = new Bundle {
      val tx = new C2cIcnBundle
      val rx = Flipped(new C2cIcnBundle)
    }
    val c2c = new C2cBundle
  })
  private val dispatcher = Module(new C2cTxDispatcher)
  private val txreq = Module(new TxQueue(UInt(reqFlitBits.W), C2cUtils.reqId))
  private val txrsp = Module(new TxQueue(UInt(respFlitBits.W), C2cUtils.rspId))
  private val txdat = Module(new TxQueue(UInt(dataFlitBits.W), C2cUtils.datId))
  private val txsnp = Module(new TxQueue(UInt(snoopFlitBits.W), C2cUtils.snpId))

  private val rxreq = Module(new RxQueue(UInt(reqFlitBits.W), C2cUtils.reqId, zjParams.c2cParams.reqRxSize))
  private val rxrsp = Module(new RxQueue(UInt(respFlitBits.W), C2cUtils.rspId, zjParams.c2cParams.rspRxSize))
  private val rxdat = Module(new RxQueue(UInt(dataFlitBits.W), C2cUtils.datId, zjParams.c2cParams.datRxSize))
  private val rxsnp = Module(new RxQueue(UInt(snoopFlitBits.W), C2cUtils.snpId, zjParams.c2cParams.snpRxSize))

  //tx connections
  txreq.io.enq <> io.icn.rx.req
  txrsp.io.enq <> io.icn.rx.rsp
  txdat.io.enq <> io.icn.rx.dat
  txsnp.io.enq <> io.icn.rx.snp

  dispatcher.io.rx.reqSlots <> txreq.io.deq
  dispatcher.io.rx.rspSlots <> txrsp.io.deq
  dispatcher.io.rx.datSlots <> txdat.io.deq
  dispatcher.io.rx.snpSlots <> txsnp.io.deq

  io.c2c.tx.valid := dispatcher.io.tx.valid
  io.c2c.tx.bits := dispatcher.io.tx.bits.asUInt
  dispatcher.io.tx.ready := io.c2c.tx.ready

  //rx connections
  private val rxPayload  = Wire(Valid(new C2cPayload))
  rxPayload.valid := io.c2c.rx.valid
  rxPayload.bits := io.c2c.rx.bits.asTypeOf(new C2cPayload)
  private val rxPipe = Pipe(rxPayload)

  rxreq.io.enq := rxPipe
  rxrsp.io.enq := rxPipe
  rxdat.io.enq := rxPipe
  rxsnp.io.enq := rxPipe

  io.icn.tx.req <> rxreq.io.deq
  io.icn.tx.rsp <> rxrsp.io.deq
  io.icn.tx.dat <> rxdat.io.deq
  io.icn.tx.snp <> rxsnp.io.deq

  //grant connections
  dispatcher.io.rx.reqGrant <> rxreq.io.grant
  dispatcher.io.rx.rspGrant <> rxrsp.io.grant
  dispatcher.io.rx.datGrant <> rxdat.io.grant
  dispatcher.io.rx.snpGrant <> rxsnp.io.grant

  txreq.io.grant.valid := rxPipe.valid && rxPipe.bits.reqGrants.valid
  txreq.io.grant.bits := rxPipe.bits.reqGrants.bits
  txrsp.io.grant.valid := rxPipe.valid && rxPipe.bits.rspGrants.valid
  txrsp.io.grant.bits := rxPipe.bits.rspGrants.bits
  txdat.io.grant.valid := rxPipe.valid && rxPipe.bits.datGrants.valid
  txdat.io.grant.bits := rxPipe.bits.datGrants.bits
  txsnp.io.grant.valid := rxPipe.valid && rxPipe.bits.snpGrants.valid
  txsnp.io.grant.bits := rxPipe.bits.snpGrants.bits
}

class C2cLoopBack(implicit p:Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val enq = Flipped(new C2cIcnBundle)
    val deq = new C2cIcnBundle
  })
  private def addCounter[T <: Data](port:DecoupledIO[T], name:String):Unit = {
    val cnt = RegInit(0.U(64.W))
    when(port.fire) {
      cnt := cnt + 1.U
    }
    dontTouch(cnt)
    cnt.suggestName(s"${name}_cnt")
  }

  private val q = p.alterPartial({
    case ZJParametersKey => zjParams.copy(c2cParams = zjParams.c2cParams.copy(reqRxSize = 31, rspRxSize = 15, datRxSize = 7, snpRxSize = 7))
  })
  private val linkLatency = 32
  private val p0 = Module(new C2cPacker()(q))
  private val p1 = Module(new C2cPacker()(q))
  private val link0 = Wire(Valid(UInt(C2cUtils.payloadBits.W)))
  private val link1 = Wire(Valid(UInt(C2cUtils.payloadBits.W)))
  private val link0V = RegInit(VecInit(Seq.fill(linkLatency)(false.B)))
  private val link0B = Reg(Vec(linkLatency, UInt(256.W)))
  private val link1V = RegInit(VecInit(Seq.fill(linkLatency)(false.B)))
  private val link1B = Reg(Vec(linkLatency, UInt(256.W)))

  private val link0VSeq = link0V :+ link0.valid
  private val link0BSeq = link0B :+ link0.bits
  private val link1VSeq = link1V :+ link1.valid
  private val link1BSeq = link1B :+ link1.bits

  link0.valid := p0.io.c2c.tx.valid
  link0.bits := p0.io.c2c.tx.bits
  p0.io.c2c.tx.ready := true.B
  link1.valid := p1.io.c2c.tx.valid
  link1.bits := p1.io.c2c.tx.bits
  p1.io.c2c.tx.ready := true.B
  for(i <- 0 until linkLatency) {
    link0VSeq(i) := link0VSeq(i + 1)
    link1VSeq(i) := link1VSeq(i + 1)
    when(link0VSeq(i + 1)) {
      link0BSeq(i) := link0BSeq(i + 1)
    }
    when(link1VSeq(i + 1)) {
      link1BSeq(i) := link1BSeq(i + 1)
    }
  }

  p1.io.c2c.rx.valid := link0VSeq.head
  p1.io.c2c.rx.bits := link0BSeq.head

  p0.io.c2c.rx.valid := link1VSeq.head
  p0.io.c2c.rx.bits := link1BSeq.head

  p0.io.icn.rx <> io.enq
  p1.io.icn.rx <> p1.io.icn.tx
  io.deq <> p0.io.icn.tx

  addCounter(p0.io.icn.rx.req, "enq_req")
  addCounter(p0.io.icn.rx.rsp, "enq_rsp")
  addCounter(p0.io.icn.rx.dat, "enq_dat")
  addCounter(p0.io.icn.rx.snp, "enq_snp")

  addCounter(p1.io.icn.tx.req, "tx_req")
  addCounter(p1.io.icn.tx.rsp, "tx_rsp")
  addCounter(p1.io.icn.tx.dat, "tx_dat")
  addCounter(p1.io.icn.tx.snp, "tx_snp")

  addCounter(p1.io.icn.rx.req, "rx_req")
  addCounter(p1.io.icn.rx.rsp, "rx_rsp")
  addCounter(p1.io.icn.rx.dat, "rx_dat")
  addCounter(p1.io.icn.rx.snp, "rx_snp")

  addCounter(p0.io.icn.tx.req, "deq_req")
  addCounter(p0.io.icn.tx.rsp, "deq_rsp")
  addCounter(p0.io.icn.tx.dat, "deq_dat")
  addCounter(p0.io.icn.tx.snp, "deq_snp")
}
