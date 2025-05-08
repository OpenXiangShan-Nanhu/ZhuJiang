package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import xs.utils.queue.FastQueue
import chisel3.experimental.BundleLiterals._

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- DBID Pool ---------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class DBIDPool(implicit p: Parameters) extends DJModule {
  val io = IO(new Bundle {
    val enq0    = Flipped(Valid(UInt(dbIdBits.W)))
    val enq1    = Flipped(Valid(UInt(dbIdBits.W)))
    val deq0    = Decoupled(UInt(dbIdBits.W))
    val deq1    = Decoupled(UInt(dbIdBits.W))
  })
  dontTouch(io)
  require(djparam.nrBeat == 2)

  val q0 = Module(new FastQueue(UInt(dbIdBits.W), djparam.nrDataBuf/2, false))
  val q1 = Module(new FastQueue(UInt(dbIdBits.W), djparam.nrDataBuf/2, false))
  dontTouch(q0.io)
  dontTouch(q1.io)

//  q0.io.enq <> DontCare
//  q0.io.deq <> DontCare

  // reset
  val rstCounter = Counter(djparam.nrDataBuf/2)
  val rstDoneReg = RegEnable(true.B, false.B, rstCounter.inc())

  // Enq
  val enqOne    = io.enq0.valid ^ io.enq1.valid
  val enqTwo    = io.enq0.valid & io.enq1.valid
  val enqSel0   = io.enq0.valid
  val enqSelQ0  = q0.io.count <= q1.io.count
  dontTouch(enqOne)
  dontTouch(enqTwo)
  dontTouch(enqSel0)
  dontTouch(enqSelQ0)

  // Reset
  when(!rstDoneReg) {
    q0.io.enq.valid := true.B
    q1.io.enq.valid := true.B
    q0.io.enq.bits  := Cat(0.U, rstCounter.value)
    q1.io.enq.bits  := Cat(1.U, rstCounter.value)
  // Enq one
  }.elsewhen(enqOne) {
    q0.io.enq.valid :=  enqSelQ0
    q1.io.enq.valid := !enqSelQ0
    q0.io.enq.bits  := Mux(enqSel0, io.enq0.bits, io.enq1.bits)
    q1.io.enq.bits  := Mux(enqSel0, io.enq0.bits, io.enq1.bits)
  // Enq two
  }.otherwise {
    q0.io.enq.valid := enqTwo
    q1.io.enq.valid := enqTwo
    q0.io.enq.bits  := io.enq0.bits
    q1.io.enq.bits  := io.enq1.bits
  }
  HAssert.withEn(q0.io.enq.ready, q0.io.enq.valid)
  HAssert.withEn(q1.io.enq.ready, q1.io.enq.valid)

  // Deq
  val deqOne    = io.deq0.ready ^ io.deq1.ready
  val deqTwo    = io.deq0.ready & io.deq1.ready
  val deqSel0   = io.deq0.ready
  val deqSelQ0  = q0.io.count >= q1.io.count
  dontTouch(deqOne)
  dontTouch(deqTwo)
  dontTouch(deqSel0)
  dontTouch(deqSelQ0)

  // Deq one
  when(deqOne) {
    q0.io.deq.ready :=  deqSelQ0
    q1.io.deq.ready := !deqSelQ0
    io.deq0.bits    := Mux(deqSelQ0, q0.io.deq.bits, q1.io.deq.bits)
    io.deq1.bits    := Mux(deqSelQ0, q0.io.deq.bits, q1.io.deq.bits)
  // Deq one
  }.otherwise {
    q0.io.deq.ready := deqTwo
    q1.io.deq.ready := deqTwo
    io.deq0.bits    := q0.io.deq.bits
    io.deq1.bits    := q1.io.deq.bits
  }
  io.deq0.valid := q0.io.deq.valid & rstDoneReg
  io.deq1.valid := q1.io.deq.valid & rstDoneReg
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------------- DBID Ctrl ---------------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //
class DBIDCtrl(implicit p: Parameters) extends DJModule {
  val io = IO(new Bundle {
    val req         = Flipped(Decoupled(new HnTxnID with HasDataVec))
    val release     = Flipped(Valid(new HnTxnID with HasDataVec))
    val getDBIDVec  = Vec(5, Flipped(new GetDBID))
  })

  /*
   * Module and Reg declaration
   */
  val pool          = Module(new DBIDPool)
  val dbidRegVec2   = Reg(Vec(djparam.nrPoS, Vec(2, UInt(dbIdBits.W))))
  val validRegVec2  = RegInit(VecInit(Seq.fill(djparam.nrPoS) { VecInit(false.B, false.B) })) // valid is use for assert
  val reqId         = io.req.bits.hnTxnID
  val relId         = io.release.bits.hnTxnID

  /*
   * Receive req
   */
  // Set req ready
  val hasTwo          = pool.io.deq0.valid & pool.io.deq1.valid
  io.req.ready        := hasTwo // Waste a DB to simplify ready judgment condition
  // Set pool deq ready
  pool.io.deq0.ready  := io.req.valid & io.req.bits.dataVec(0) & hasTwo
  pool.io.deq1.ready  := io.req.valid & io.req.bits.dataVec(1) & hasTwo
  // HAssert
  HAssert.withEn(!validRegVec2(reqId)(0), io.req.valid & io.req.bits.dataVec(0))
  HAssert.withEn(!validRegVec2(reqId)(1), io.req.valid & io.req.bits.dataVec(1))
  HAssert.withEn(!io.req.bits.isZero, io.req.valid)

  /*
   * Receive release
   */
  // Set pool enq valid
  pool.io.enq0.valid  := io.release.valid & io.release.bits.dataVec(0)
  pool.io.enq1.valid  := io.release.valid & io.release.bits.dataVec(1)
  // Set pool enq bits
  pool.io.enq0.bits   := dbidRegVec2(relId)(0)
  pool.io.enq1.bits   := dbidRegVec2(relId)(1)
  // HAssert
  HAssert.withEn(validRegVec2(reqId)(0), io.release.valid & io.release.bits.dataVec(0))
  HAssert.withEn(validRegVec2(reqId)(1), io.release.valid & io.release.bits.dataVec(1))

  /*
   * Save dbid in reg
   */
  // dbid valid
  validRegVec2(relId)(0)  := io.req.valid & io.req.bits.dataVec(0)
  validRegVec2(relId)(1)  := io.req.valid & io.req.bits.dataVec(1)
  // dbid
  dbidRegVec2(relId)(0)   := pool.io.deq0.bits
  dbidRegVec2(relId)(1)   := pool.io.deq1.bits
  // HAssert
  HAssert.withEn(pool.io.deq0.fire, io.req.fire & io.req.bits.dataVec(0))
  HAssert.withEn(pool.io.deq1.fire, io.req.fire & io.req.bits.dataVec(1))

  /*
   * Get dbid from reg
   */
  io.getDBIDVec.foreach { get =>
    get.dbidVec(0) := dbidRegVec2(get.hnTxnID)(0)
    get.dbidVec(1) := dbidRegVec2(get.hnTxnID)(1)
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}