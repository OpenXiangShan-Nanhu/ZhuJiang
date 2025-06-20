package zhujiang.axi

import chisel3._
import chisel3.util._
import xs.utils.ResetRRArbiter

abstract class BaseAxiXbar(mstParams:Seq[AxiParams]) extends Module {
  def slvMatchersSeq: Seq[UInt => Bool]
  private lazy val _slvP = AxiSlvParamsCalc(mstParams)
  private lazy val dataBits = _slvP.dataBits
  private lazy val mstMaxIdBits = mstParams.map(_.idBits).max
  private lazy val extraIdBits = log2Ceil(mstParams.length)
  private lazy val slvIdBits = mstMaxIdBits + extraIdBits
  private lazy val slvParams = Seq.fill(slvMatchersSeq.length)(_slvP.copy())
  private lazy val extraBitsRange = (slvIdBits - 1, mstMaxIdBits)
  private lazy val mstSize = mstParams.length
  private lazy val slvSize = slvParams.length

  lazy val io = IO(new Bundle {
    val upstream = MixedVec(mstParams.map(params => Flipped(new AxiBundle(params))))
    val downstream = MixedVec(slvParams.map(params => new AxiBundle(params)))
  })

  private def writeReqConn(): Unit = {
    val awDnStrmRdyMat = Wire(Vec(mstSize, Vec(slvSize, Bool())))
    val wDnStrmRdyMat = Wire(Vec(mstSize, Vec(slvSize, Bool())))
    dontTouch(awDnStrmRdyMat)
    dontTouch(wDnStrmRdyMat)
    awDnStrmRdyMat.suggestName("awDnStrmRdyMat")
    wDnStrmRdyMat.suggestName("wDnStrmRdyMat")
    for(sidx <- slvParams.indices) {
      val recordQueue = Module(new Queue(UInt(mstSize.W), entries = 16))
      val awQueue = Module(new Queue(new AWFlit(slvParams(sidx)), entries = 2))
      val wQueue = Module(new Queue(new WFlit(slvParams(sidx)), entries = 2))
      val arb = Module(new ResetRRArbiter(new AWFlit(slvParams(sidx)), mstParams.length))
      recordQueue.suggestName(s"aw_rq_$sidx")
      arb.suggestName(s"aw_arb_$sidx")
      awQueue.suggestName(s"aw_q_$sidx")
      wQueue.suggestName(s"w_q_$sidx")
      io.downstream(sidx).aw <> awQueue.io.deq
      io.downstream(sidx).w <> wQueue.io.deq

      arb.io.out.ready := awQueue.io.enq.ready && recordQueue.io.enq.ready
      recordQueue.io.enq.valid := arb.io.out.valid && awQueue.io.enq.ready
      awQueue.io.enq.valid := arb.io.out.valid && recordQueue.io.enq.ready

      val record = if(mstSize > 1) UIntToOH(arb.io.out.bits.id(extraBitsRange._1, extraBitsRange._2)) else 1.U
      recordQueue.io.enq.bits := record
      awQueue.io.enq.bits := arb.io.out.bits
      val wSelV = WireInit(Mux1H(recordQueue.io.deq.bits, io.upstream.map(_.w.valid)))
      wSelV.suggestName(s"w_sel_vld_$sidx")
      dontTouch(wSelV)
      wQueue.io.enq.valid := recordQueue.io.deq.valid && wSelV
      wQueue.io.enq.bits := Mux1H(recordQueue.io.deq.bits, io.upstream.map(_.w.bits))
      recordQueue.io.deq.ready := wQueue.io.enq.fire && wQueue.io.enq.bits._last

      for(midx <- mstParams.indices) {
        val wreq = io.upstream(midx).aw
        val ain = arb.io.in(midx)
        ain.valid := wreq.valid && slvMatchersSeq(sidx)(wreq.bits.addr)
        ain.bits := wreq.bits
        if(mstSize > 1) ain.bits.id := Cat(midx.U(extraIdBits.W), wreq.bits.id.asTypeOf(UInt(mstMaxIdBits.W)))
        awDnStrmRdyMat(midx)(sidx) := ain.ready && slvMatchersSeq(sidx)(wreq.bits.addr)
        wDnStrmRdyMat(midx)(sidx) := wQueue.io.enq.ready && recordQueue.io.deq.valid && recordQueue.io.deq.bits(midx)
        when(wreq.valid) {
          assert(PopCount(awDnStrmRdyMat(midx)) <= 1.U)
        }
        when(io.upstream(midx).w.valid) {
          assert(PopCount(wDnStrmRdyMat(midx)) <= 1.U)
        }
      }
    }
    for(midx <- mstParams.indices) {
      io.upstream(midx).aw.ready := io.upstream(midx).aw.valid && Cat(awDnStrmRdyMat(midx)).orR
      io.upstream(midx).w.ready := io.upstream(midx).w.valid && Cat(wDnStrmRdyMat(midx)).orR
    }
  }

  private def readReqConn(): Unit = {
    val arDnStrmRdyMat = Wire(Vec(mstSize, Vec(slvSize, Bool())))
    dontTouch(arDnStrmRdyMat)
    arDnStrmRdyMat.suggestName("arDnStrmRdyMat")
    for(sidx <- slvParams.indices) {
      val arQueue = Module(new Queue(new ARFlit(slvParams(sidx)), entries = 2))
      val arb = Module(new ResetRRArbiter(new ARFlit(slvParams(sidx)), mstParams.length))
      arb.suggestName(s"ar_arb_$sidx")
      arQueue.suggestName(s"ar_q_$sidx")
      io.downstream(sidx).ar <> arQueue.io.deq
      arQueue.io.enq <> arb.io.out

      for(midx <- mstParams.indices) {
        val rreq = io.upstream(midx).ar
        val ain = arb.io.in(midx)
        ain.valid := rreq.valid && slvMatchersSeq(sidx)(rreq.bits.addr)
        ain.bits := rreq.bits
        if(mstSize > 1) ain.bits.id := Cat(midx.U(extraIdBits.W), rreq.bits.id.asTypeOf(UInt(mstMaxIdBits.W)))
        arDnStrmRdyMat(midx)(sidx) := ain.ready && slvMatchersSeq(sidx)(rreq.bits.addr)
        when(rreq.valid) {
          assert(PopCount(arDnStrmRdyMat(midx)) <= 1.U)
        }
      }
    }
    for(midx <- mstParams.indices) {
      io.upstream(midx).ar.ready := io.upstream(midx).ar.valid && Cat(arDnStrmRdyMat(midx)).orR
    }
  }

  private def writeRespConn(): Unit = {
    val bUpStrmRdyMat = Wire(Vec(slvSize, Vec(mstSize, Bool())))
    dontTouch(bUpStrmRdyMat)
    bUpStrmRdyMat.suggestName("bUpStrmRdyMat")
    for(midx <- mstParams.indices) {
      val bQueue = Module(new Queue(new BFlit(mstParams(midx)), entries = 2))
      val arb = Module(new ResetRRArbiter(new BFlit(mstParams(midx)), slvParams.length))
      bQueue.suggestName(s"b_q_$midx")
      arb.suggestName(s"b_arb_$midx")
      io.upstream(midx).b <> bQueue.io.deq
      bQueue.io.enq <> arb.io.out

      for(sidx <- slvParams.indices) {
        val wresp = io.downstream(sidx).b
        val ain = arb.io.in(sidx)
        val correctMst = if(mstSize > 1) wresp.bits.id(extraBitsRange._1, extraBitsRange._2) === midx.U else true.B
        ain.valid := wresp.valid && correctMst
        ain.bits := wresp.bits
        ain.bits.id := wresp.bits.id(mstParams(midx).idBits - 1, 0)
        bUpStrmRdyMat(sidx)(midx) := ain.ready && correctMst
        when(wresp.valid) {
          assert(PopCount(bUpStrmRdyMat(sidx)) <= 1.U)
        }
      }
    }
    for(sidx <- slvParams.indices) {
      io.downstream(sidx).b.ready := io.downstream(sidx).b.valid && Cat(bUpStrmRdyMat(sidx)).orR
    }
  }

  private def readRespConn(): Unit = {
    val rUpStrmRdyMat = Wire(Vec(slvSize, Vec(mstSize, Bool())))
    dontTouch(rUpStrmRdyMat)
    rUpStrmRdyMat.suggestName("rUpStrmRdyMat")
    for(midx <- mstParams.indices) {
      val rQueue = Module(new Queue(new RFlit(mstParams(midx)), entries = 2))
      val arb = Module(new ResetRRArbiter(new RFlit(mstParams(midx)), slvParams.length))
      rQueue.suggestName(s"r_queue_$midx")
      arb.suggestName(s"r_arb_$midx")
      io.upstream(midx).r <> rQueue.io.deq
      rQueue.io.enq <> arb.io.out

      for(sidx <- slvParams.indices) {
        val rdata = io.downstream(sidx).r
        val ain = arb.io.in(sidx)
        val correctMst = if(mstSize > 1) rdata.bits.id(extraBitsRange._1, extraBitsRange._2) === midx.U else true.B
        ain.valid := rdata.valid && correctMst
        ain.bits := rdata.bits
        ain.bits.data := rdata.bits.data(mstParams(midx).dataBits - 1, 0)
        ain.bits.id := rdata.bits.id(mstParams(midx).idBits - 1, 0)
        rUpStrmRdyMat(sidx)(midx) := ain.ready && correctMst
        when(rdata.valid) {
          assert(PopCount(rUpStrmRdyMat(sidx)) <= 1.U)
        }
      }
    }
    for(sidx <- slvParams.indices) {
      io.downstream(sidx).r.ready := io.downstream(sidx).r.valid && Cat(rUpStrmRdyMat(sidx)).orR
    }
  }

  def initialize(): Unit = {
    mstParams.foreach(m => require(m.dataBits == dataBits))
    io
    writeReqConn()
    readReqConn()
    writeRespConn()
    readRespConn()
  }
}