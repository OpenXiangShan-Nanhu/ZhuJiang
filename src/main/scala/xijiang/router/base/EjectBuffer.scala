package xijiang.router.base

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import zhujiang.ZJModule
import zhujiang.chi.{NodeIdBundle, RingFlit}

class VipTable[T <: Data](gen:T, size: Int) extends Module {
  val io = IO(new Bundle {
    val update = Input(Valid(new Bundle {
      val tag = gen.cloneType
      val rel = Bool()
    }))
    val vip = Output(Valid(gen))
  })
  private val valids = RegInit(VecInit(Seq.fill(size)(false.B)))
  private val table = Reg(Vec(size, gen))
  private val vipPtrOH = RegInit(1.U(size.W))
  private val enqIdxOH = PriorityEncoderOH(valids.map(!_))

  private val tagHitExisted = valids.zip(table).map(e => e._1 && e._2 === io.update.bits.tag)
  private val tagExisted = Cat(tagHitExisted).orR
  private val doAlloc = io.update.valid && !tagExisted && !io.update.bits.rel
  when(io.update.valid) {
    assert(PopCount(tagHitExisted) <= 1.U)
  }
  when(doAlloc) {
    assert(Cat(valids.map(!_)).orR)
  }
  for(idx <- table.indices) {
    val enq = doAlloc && enqIdxOH(idx)
    val rel = io.update.valid && io.update.bits.rel && tagHitExisted(idx)
    valids(idx) := Mux(enq, true.B, Mux(rel, false.B, valids(idx)))
    when(enq) {
      table(idx) := io.update.bits.tag
    }
  }
  io.vip.valid := Mux1H(vipPtrOH, valids)
  io.vip.bits := Mux1H(vipPtrOH, table)

  private val highMaskVec = Wire(Vec(size, Bool()))
  private val lowMaskVec = Wire(Vec(size, Bool()))
  dontTouch(highMaskVec)
  dontTouch(lowMaskVec)
  private val highMask = highMaskVec.asUInt
  private val lowMask = lowMaskVec.asUInt
  private val validMask = valids.asUInt
  for(idx <- 0 until size) {
    if(idx == 0) {
      highMaskVec(idx) := false.B
    } else {
      highMaskVec(idx) := vipPtrOH(idx - 1, 0).orR
    }
    if(idx == size - 1) {
      lowMaskVec(idx) := false.B
    } else {
      lowMaskVec(idx) := vipPtrOH(size - 1, idx + 1).orR
    }
    assert(PopCount(Seq(highMaskVec(idx), lowMaskVec(idx), vipPtrOH(idx))) === 1.U, s"bit $idx is not covered by any mask!")
  }

  private val highValidMask = highMask & validMask
  private val lowValidMask = lowMask & validMask
  private val highPtrNext = PriorityEncoderOH(highValidMask)
  private val lowPtrNext = PriorityEncoderOH(lowValidMask)
  private val ptrMove = !io.vip.valid && validMask.orR
  private val ptrNext = Mux(highValidMask.orR, highPtrNext, lowPtrNext)
  when(ptrMove) {
    vipPtrOH := ptrNext
    assert((ptrNext & validMask).orR)
  }
}

class EjectBuffer[T <: RingFlit](gen: T, size: Int, chn: String)(implicit p: Parameters) extends ZJModule {
  private def getTag(flit: RingFlit): UInt = {
    val tgtAid = flit.tgt.asTypeOf(new NodeIdBundle).aid
    if(chn == "DAT") Cat(flit.src, flit.txn, tgtAid, flit.did) else Cat(flit.src, flit.txn, tgtAid)
  }
  require(size >= 3)
  val io = IO(new Bundle {
    val enq = Flipped(Decoupled(gen))
    val deq = Decoupled(gen)
  })
  override val desiredName = s"EjectBuffer$chn"
  private val flitTag = getTag(io.enq.bits)
  private val oqueue = Module(new Queue(gen, size - 1))
  private val ipipe = Module(new Queue(gen, 1, pipe = true))
  private val vipTable = Module(new VipTable(UInt(flitTag.getWidth.W), zjParams.island.size))
  private val empties = RegInit(size.U(log2Ceil(size + 1).W))

  oqueue.io.enq <> ipipe.io.deq
  io.deq <> oqueue.io.deq
  private val enqFire = ipipe.io.enq.fire
  private val deqFire = oqueue.io.deq.fire
  when(enqFire && !deqFire) {
    assert(empties > 0.U)
    empties := empties - 1.U
  }.elsewhen(!enqFire && deqFire) {
    assert(empties < size.U)
    empties := empties + 1.U
  }
  when(empties.orR) {
    assert(ipipe.io.enq.ready)
  }

  vipTable.io.update.valid := RegNext(io.enq.valid, false.B)
  vipTable.io.update.bits.rel := RegNext(io.enq.ready, true.B)
  vipTable.io.update.bits.tag := RegEnable(flitTag, io.enq.valid)

  private val allowEnq = Mux(empties === 1.U, flitTag === vipTable.io.vip.bits && vipTable.io.vip.valid, true.B)
  ipipe.io.enq.valid := io.enq.valid & allowEnq
  ipipe.io.enq.bits := io.enq.bits
  io.enq.ready := empties.orR & allowEnq
}
