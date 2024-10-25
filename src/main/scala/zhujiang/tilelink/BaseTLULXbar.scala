package zhujiang.tilelink

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xs.utils.ResetRRArbiter
import zhujiang.ZJModule

abstract class BaseTLULXbar(implicit p: Parameters) extends ZJModule {
  def mstParams: Seq[TilelinkParams]
  def slvAddrBits: Int
  def slvMatchersSeq: Seq[UInt => Bool]

  private lazy val dataBits = mstParams.head.dataBits
  private lazy val mstMaxIdBits = mstParams.map(_.sourceBits).max
  private lazy val mstMaxAddrBits = mstParams.map(_.addrBits).max
  private lazy val extraIdBits = log2Ceil(mstParams.length)
  private lazy val slvIdBits = mstMaxIdBits + extraIdBits
  private lazy val slvParams = Seq.fill(slvMatchersSeq.length)(TilelinkParams(addrBits = mstMaxAddrBits, dataBits = dataBits, sourceBits = slvIdBits))
  private lazy val extraBitsRange = (slvIdBits - 1, mstMaxIdBits)
  private lazy val mstSize = mstParams.length
  private lazy val slvSize = slvParams.length

  lazy val io = IO(new Bundle {
    val upstream = MixedVec(mstParams.map(params => Flipped(new TLULBundle(params))))
    val downstream = MixedVec(slvParams.map(params => new TLULBundle(params)))
  })

  private def ReqConn(): Unit = {
    val aDnStrmRdyMat = Wire(Vec(mstSize, Vec(slvSize, Bool())))
    dontTouch(aDnStrmRdyMat)
    aDnStrmRdyMat.suggestName("aDnStrmRdyMat")
    for(sidx <- slvParams.indices) {
      val aQueue = Module(new Queue(new AFlit(slvParams(sidx)), entries = 2))
      val arb = Module(new ResetRRArbiter(new AFlit(slvParams(sidx)), mstParams.length))
      arb.suggestName(s"a_arb_$sidx")
      aQueue.suggestName(s"a_q_$sidx")
      io.downstream(sidx).a <> aQueue.io.deq
      aQueue.io.enq <> arb.io.out

      for(midx <- mstParams.indices) {
        val req = io.upstream(midx).a
        val ain = arb.io.in(midx)
        ain.valid := req.valid && slvMatchersSeq(sidx)(req.bits.address)
        ain.bits := req.bits
        if(mstSize > 1) ain.bits.source := Cat(midx.U(extraIdBits.W), req.bits.source.asTypeOf(UInt(mstMaxIdBits.W)))
        aDnStrmRdyMat(midx)(sidx) := ain.ready
      }
    }
    for(midx <- mstParams.indices) {
      io.upstream(midx).a.ready := io.upstream(midx).a.valid && Cat(aDnStrmRdyMat(midx)).orR
    }
  }

  private def RespConn(): Unit = {
    val dUpStrmRdyMat = Wire(Vec(slvSize, Vec(mstSize, Bool())))
    dontTouch(dUpStrmRdyMat)
    dUpStrmRdyMat.suggestName("dUpStrmRdyMat")
    for(midx <- mstParams.indices) {
      val dQueue = Module(new Queue(new DFlit(mstParams(midx)), entries = 2))
      val arb = Module(new ResetRRArbiter(new DFlit(mstParams(midx)), slvParams.length))
      dQueue.suggestName(s"d_q_$midx")
      arb.suggestName(s"d_arb_$midx")
      io.upstream(midx).d <> dQueue.io.deq
      dQueue.io.enq <> arb.io.out

      for(sidx <- slvParams.indices) {
        val resp = io.downstream(sidx).d
        val ain = arb.io.in(sidx)
        val correctMst = if(mstSize > 1) resp.bits.source(extraBitsRange._1, extraBitsRange._2) === midx.U else true.B
        ain.valid := resp.valid && correctMst
        ain.bits := resp.bits
        ain.bits.source := resp.bits.source(mstParams(midx).sourceBits - 1, 0)
        dUpStrmRdyMat(sidx)(midx) := ain.ready && correctMst
      }
    }
    for(sidx <- slvParams.indices) {
      io.downstream(sidx).d.ready := io.downstream(sidx).d.valid && Cat(dUpStrmRdyMat(sidx)).orR
    }
  }

  def initialize(): Unit = {
    mstParams.foreach(m => require(m.dataBits == dataBits))
    io
    ReqConn()
    RespConn()
  }
}
