package xijiang.c2c
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.NodeType
import xs.utils.ResetRRArbiter
import xs.utils.debug.HardwareAssertionKey
import zhujiang.chi.RingFlit
import zhujiang.{ZJBundle, ZJModule, ZJParametersKey}

class C2cChiBundle(implicit p:Parameters) extends ZJBundle {
  val req = Decoupled(UInt(rreqFlitBits.W))
  val rsp = Decoupled(UInt(respFlitBits.W))
  val dat = Decoupled(UInt(dataFlitBits.W))
  val snp = Decoupled(UInt(snoopFlitBits.W))
  val dbg = Option.when(p(HardwareAssertionKey).enable)(Decoupled(UInt(debugFlitBits.W)))
}

class C2cTxDispatcher(implicit p:Parameters) extends ZJModule {
  private val chnInfoSeq = Seq(
    C2cUtils.getSlotsAndDeq(rreqFlitBits),
    C2cUtils.getSlotsAndDeq(respFlitBits),
    C2cUtils.getSlotsAndDeq(dataFlitBits),
    C2cUtils.getSlotsAndDeq(snoopFlitBits),
  )

  val io = IO(new Bundle {
    val rx = new Bundle {
      val slots = MixedVec(chnInfoSeq.map(info => Flipped(Vec(info._2, Decoupled(new C2cSlot)))))
      val grants = Vec(C2cUtils.nrChn, Flipped(Decoupled(UInt(C2cUtils.grantDataBits.W))))
    }
    val tx = Decoupled(new C2cPayload)
  })
  private val reqSlots = io.rx.slots(C2cUtils.reqId)
  private val rspSlots = io.rx.slots(C2cUtils.rspId)
  private val datSlots = io.rx.slots(C2cUtils.datId)
  private val snpSlots = io.rx.slots(C2cUtils.snpId)
  require(chnInfoSeq(C2cUtils.reqId)._1 == 2)
  require(chnInfoSeq(C2cUtils.rspId)._1 == 1)
  require(chnInfoSeq(C2cUtils.datId)._1 == 5)
  require(chnInfoSeq(C2cUtils.snpId)._1 == 2)

  private val flitAllSlotsType = Vec(C2cUtils.nrSlotsPerFrame, Valid(new C2cSlot))

  private val oPipe = Module(new Queue(new C2cPayload, entries = 2))
  private val payloadArb = Module(new ResetRRArbiter(flitAllSlotsType, 2))

  private val datPayload = Wire(Decoupled(flitAllSlotsType))
  datPayload.bits.zip(datSlots).foreach({ case(a, b) =>
    a.valid := b.valid
    a.bits := b.bits
    b.ready := datPayload.ready
  })
  datPayload.valid := datPayload.bits.map(_.valid).reduce(_ | _)

  private val combPayload = Wire(Decoupled(flitAllSlotsType))
  private val combArb = Module(new ResetRRArbiter(Bool(), 2)) // Dont care about Data of this arbiter, use it arbitrary logics only.
  combArb.io.in.head.valid := reqSlots.head.valid
  combArb.io.in.last.valid := snpSlots.head.valid
  combArb.io.out.ready := combPayload.ready
  combArb.io.in.foreach(_.bits := DontCare)
  reqSlots.foreach(_.ready := combArb.io.in.head.ready)
  snpSlots.foreach(_.ready := combArb.io.in.last.ready)

  for(i <- reqSlots.indices) {
    combPayload.bits(i).valid := Mux(combArb.io.chosen === 0.U, reqSlots(i).valid, snpSlots(i).valid)
    combPayload.bits(i).bits := Mux(combArb.io.chosen === 0.U, reqSlots(i).bits, snpSlots(i).bits)
  }
  combPayload.bits.last.valid := rspSlots.head.valid
  combPayload.bits.last.bits := rspSlots.head.bits
  rspSlots.head.ready := combPayload.ready

  combPayload.valid := combPayload.bits.map(_.valid).reduce(_ | _)

  payloadArb.io.in.head <> datPayload
  payloadArb.io.in.last <> combPayload

  oPipe.io.enq.valid := Cat(io.rx.grants.map(_.valid)).orR || payloadArb.io.out.valid
  oPipe.io.enq.bits.slots := payloadArb.io.out.bits
  payloadArb.io.out.ready := oPipe.io.enq.ready

  oPipe.io.enq.bits.grants.zip(io.rx.grants).foreach({ case(a, b) =>
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

class C2cInitializer extends Module {
  val io = IO(new Bundle {
    val link = new C2cBundle
    val userTx = Input(UInt(C2cUtils.slotDataBits.W))
    val userRx = Output(Valid(UInt(C2cUtils.slotDataBits.W)))
  })
  private val recvPayload = io.link.rx.bits.asTypeOf(new C2cPayload)
  private val initFrame = io.link.rx.valid && Cat(recvPayload.slots.map(_.valid) ++ recvPayload.grants.map(_.valid)) === 0.U

  private val initialized = RegInit(false.B)
  private val ack = initFrame && recvPayload.slots(1).bits.data === 1.U
  private val lb = initFrame && recvPayload.slots(1).bits.data === 0.U
  private val loopBackPayload = WireInit(io.link.rx.bits.asTypeOf(new C2cPayload))

  initialized := Mux(ack, true.B, initialized)
  loopBackPayload.slots(0).bits.data := io.userTx
  loopBackPayload.slots(1).bits.data := 1.U

  io.link.tx.valid := lb | !initialized
  io.link.tx.bits := Mux(lb, loopBackPayload.asUInt, 0.U)

  io.userRx.valid := initialized
  io.userRx.bits := RegEnable(recvPayload.slots.head.bits.data, ack)
}

class C2cPacker(implicit p:Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val chi = new Bundle {
      val tx = new C2cChiBundle
      val rx = Flipped(new C2cChiBundle)
    }
    val userTx = Input(UInt(C2cUtils.slotDataBits.W))
    val userRx = Output(Valid(UInt(C2cUtils.slotDataBits.W)))
    val c2c = new C2cBundle
  })
  private val dispatcher = Module(new C2cTxDispatcher)
  private val txreq = Module(new TxQueue(UInt(rreqFlitBits.W), C2cUtils.reqId))
  private val txrsp = Module(new TxQueue(UInt(respFlitBits.W), C2cUtils.rspId))
  private val txdat = Module(new TxQueue(UInt(dataFlitBits.W), C2cUtils.datId))
  private val txsnp = Module(new TxQueue(UInt(snoopFlitBits.W), C2cUtils.snpId))

  private val rxreq = Module(new RxQueue(UInt(rreqFlitBits.W), C2cUtils.reqId, zjParams.c2cParams.reqRxSize))
  private val rxrsp = Module(new RxQueue(UInt(respFlitBits.W), C2cUtils.rspId, zjParams.c2cParams.rspRxSize))
  private val rxdat = Module(new RxQueue(UInt(dataFlitBits.W), C2cUtils.datId, zjParams.c2cParams.datRxSize))
  private val rxsnp = Module(new RxQueue(UInt(snoopFlitBits.W), C2cUtils.snpId, zjParams.c2cParams.snpRxSize))

  private val initializer = Module(new C2cInitializer)
  private val rxPayload  = Wire(Valid(new C2cPayload))
  rxPayload.valid := io.c2c.rx.valid
  rxPayload.bits := io.c2c.rx.bits.asTypeOf(new C2cPayload)
  private val rxPipe = Pipe(rxPayload)

  //init link
  initializer.io.userTx := io.userTx
  io.userRx := initializer.io.userRx
  initializer.io.link.rx.valid := rxPipe.valid
  initializer.io.link.rx.bits := rxPipe.bits.asUInt
  //tx connections
  txreq.io.enq <> io.chi.rx.req
  txdat.io.enq <> io.chi.rx.dat
  txsnp.io.enq <> io.chi.rx.snp

  if(p(HardwareAssertionKey).enable) {
    val rspArb = Module(new ResetRRArbiter(UInt(respFlitBits.W), 2))
    rspArb.io.in.head <> io.chi.rx.rsp
    rspArb.io.in.last <> io.chi.rx.dbg.get
    txrsp.io.enq <> rspArb.io.out
  } else {
    txrsp.io.enq <> io.chi.rx.rsp
  }

  private val txqSeq = Seq(txreq, txrsp, txdat, txsnp)
  for(txq <- txqSeq) {
    dispatcher.io.rx.slots(txq.chnId) <> txq.io.deq
    txq.io.grant.valid := rxPipe.valid && rxPipe.bits.grants(txq.chnId).valid
    txq.io.grant.bits := rxPipe.bits.grants(txq.chnId).bits
  }

  io.c2c.tx.valid := initializer.io.link.tx.valid || dispatcher.io.tx.valid
  io.c2c.tx.bits := Mux(initializer.io.link.tx.valid, initializer.io.link.tx.bits, dispatcher.io.tx.bits.asUInt)
  dispatcher.io.tx.ready := !initializer.io.link.tx.valid && io.c2c.tx.ready
  initializer.io.link.tx.ready := io.c2c.tx.ready

  //rx connections
  private val rxqSeq = Seq(rxreq, rxrsp, rxdat, rxsnp)
  for(rxq <- rxqSeq) {
    rxq.io.enq <> rxPipe
    dispatcher.io.rx.grants(rxq.chnId) <> rxq.io.grant
  }

  io.chi.tx.req <> rxreq.io.deq
  io.chi.tx.rsp <> rxrsp.io.deq
  io.chi.tx.dat <> rxdat.io.deq
  io.chi.tx.snp <> rxsnp.io.deq

  if(p(HardwareAssertionKey).enable) {
    val rxRspFlit = rxrsp.io.deq.bits.asTypeOf(new RingFlit(respFlitBits))
    val mnid = zjParams.island.filter(_.nodeType == NodeType.M).head.nodeId.U
    val isDebug = rxRspFlit.TgtID === mnid
    io.chi.tx.rsp.valid := rxrsp.io.deq.valid && !isDebug
    rxrsp.io.deq.ready := io.chi.tx.rsp.ready && !isDebug
    io.chi.tx.rsp.bits := rxrsp.io.deq.bits

    io.chi.tx.dbg.get.valid := rxrsp.io.deq.valid && isDebug
    rxrsp.io.deq.ready := io.chi.tx.dbg.get.ready && isDebug
    io.chi.tx.dbg.get.bits := rxrsp.io.deq.bits
  } else {
    io.chi.tx.rsp <> rxrsp.io.deq
  }
}

class C2cLoopBack(implicit p:Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val enq = Flipped(new C2cChiBundle)
    val deq = new C2cChiBundle
    val p0UserIn = Input(UInt(C2cUtils.slotDataBits.W))
    val p0UserOut = Output(Valid(UInt(C2cUtils.slotDataBits.W)))
    val p1UserIn = Input(UInt(C2cUtils.slotDataBits.W))
    val p1UserOut = Output(Valid(UInt(C2cUtils.slotDataBits.W)))
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
  p0.io.userTx := io.p0UserIn
  io.p0UserOut := p0.io.userRx
  p1.io.userTx := io.p1UserIn
  io.p1UserOut := p1.io.userRx

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

  p0.io.chi.rx <> io.enq
  p1.io.chi.rx <> p1.io.chi.tx
  io.deq <> p0.io.chi.tx

  addCounter(p0.io.chi.rx.req, "enq_req")
  addCounter(p0.io.chi.rx.rsp, "enq_rsp")
  addCounter(p0.io.chi.rx.dat, "enq_dat")
  addCounter(p0.io.chi.rx.snp, "enq_snp")

  addCounter(p1.io.chi.tx.req, "tx_req")
  addCounter(p1.io.chi.tx.rsp, "tx_rsp")
  addCounter(p1.io.chi.tx.dat, "tx_dat")
  addCounter(p1.io.chi.tx.snp, "tx_snp")

  addCounter(p1.io.chi.rx.req, "rx_req")
  addCounter(p1.io.chi.rx.rsp, "rx_rsp")
  addCounter(p1.io.chi.rx.dat, "rx_dat")
  addCounter(p1.io.chi.rx.snp, "rx_snp")

  addCounter(p0.io.chi.tx.req, "deq_req")
  addCounter(p0.io.chi.tx.rsp, "deq_rsp")
  addCounter(p0.io.chi.tx.dat, "deq_dat")
  addCounter(p0.io.chi.tx.snp, "deq_snp")
}
