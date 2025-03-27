package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{DomainInfo, HardwareAssertion}
import xs.utils.queue.FastQueue

class DBIDPool(implicit p: Parameters) extends DJModule {
  val io = IO(new Bundle {
    val enq0    = Flipped(Valid(UInt(dbIdBits.W)))
    val enq1    = Flipped(Valid(UInt(dbIdBits.W)))
    val deq0    = Decoupled(UInt(dbIdBits.W))
    val deq1    = Decoupled(UInt(dbIdBits.W))
    val hasOne  = Output(Bool())
    val hasTwo  = Output(Bool())
  })
  val q0 = Module(new FastQueue(UInt(dbIdBits.W), djparam.nrDataBuf/2, true))
  val q1 = Module(new FastQueue(UInt(dbIdBits.W), djparam.nrDataBuf/2, true))
  // TODO
  q0.io <> DontCare
  q1.io <> DontCare

  // reset
  val rstCounter = Counter(djparam.nrDataBuf/2)
  val rstDoneReg = RegEnable(true.B, false.B, rstCounter.inc)

  // enq
  val enqOne  = io.enq0.valid ^ io.enq1.valid
  val enqTwo  = io.enq0.valid & io.enq1.valid
  val enqSel0 = q0.io.count === q1.io.count
  HardwareAssertion.withEn(!io.enq1.valid, enqOne)
  HardwareAssertion(q0.io.count >= q1.io.count)
  HardwareAssertion(q0.io.count - q1.io.count <= 1.U)

  // Reset
  when(!rstDoneReg) {
    q0.io.enq.valid := true.B
    q1.io.enq.valid := true.B
    q0.io.enq.bits  := Cat(0.U, rstCounter.value)
    q1.io.enq.bits  := Cat(1.U, rstCounter.value)
  // Enq
  }.elsewhen(enqOne & enqSel0) {
    q0.io.enq.valid := io.enq0.valid
    q0.io.enq.bits  := io.enq0.bits
  }.elsewhen(enqOne & !enqSel0) {
    q1.io.enq.valid := io.enq0.valid
    q1.io.enq.bits  := io.enq0.bits
  }.elsewhen(enqTwo) {
    q0.io.enq.valid := io.enq0.valid
    q0.io.enq.bits  := io.enq0.bits
    q1.io.enq.valid := io.enq1.valid
    q1.io.enq.bits  := io.enq1.bits
  }.otherwise {
    q0.io.enq <> DontCare
    q1.io.enq <> DontCare
  }
  HardwareAssertion.withEn(q0.io.enq.ready, q0.io.enq.valid)
  HardwareAssertion.withEn(q0.io.enq.ready, q0.io.enq.valid)

  // Deq
  io.deq0 <> q0.io.deq
  io.deq1 <> q1.io.deq
  io.hasOne := q0.io.deq.ready
  io.hasTwo := q1.io.deq.ready
}


class DataCtrl(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // CHI TX/RX DAT
    val txDat       = Decoupled(new DataFlit())
    val rxDat       = Flipped(Decoupled(new DataFlit())) // Only use rxDat.Data/DataID/BE in DataCtrl
    // Task From Frontend or Backend
    val reqDB       = Flipped(Decoupled(new PackLLCTxnID with HasChiSize))
    val task        = Flipped(Decoupled(new DataTask()))
    val resp        = Valid(new PackLLCTxnID())
    // To/From DataStorage
    val readDsVec   = Vec(djparam.nrDSBank, Vec(2, Decoupled(new DsRead())))
    val writeDsVec  = Vec(djparam.nrDSBank, Vec(2, Decoupled(new DsWrite())))
    val respDsVec   = Vec(djparam.nrDSBank, Vec(2, Flipped(Valid(new DsResp()))))
  })
  io <> DontCare

  /*
   * Modudle, Reg and Wire declaration
   */
  val cmVec   = RegInit(VecInit(Seq.fill(djparam.nrDataCM) { 0.U.asTypeOf(new DJBundle {
    val valid = Bool()
    val dbid0 = Valid(UInt(dbIdBits.W))
    val dbid1 = Valid(UInt(dbIdBits.W))
  }) }))
  val msgVec  = Reg(Vec(djparam.nrDataCM, new DataTask()))
  val dataBuf = Reg(Vec(djparam.nrDataBuf, new DJBundle {
    val beat  = UInt(BeatBits.W)
    val mask  = UInt(MaskBits.W)
  }))
//  val dbidPool = Module(new DBIDPool())
  val cmValVec = cmVec.map(_.valid)


  /*
   * Receive Req or Task
   */
  val taskHitVec_rec  = msgVec.map(_.llcTxnID.get === io.task.bits.llcTxnID.get).zip(cmValVec).map { case(a, b) => a & b }
  val taskHit_rec     = taskHitVec_rec.reduce(_ | _)
  val cmHasFree_rec   = cmValVec.map(!_).reduce(_ | _)
  val cmFreeId_rec    = PriorityEncoder(cmValVec.map(!_))
  val taskNeedCM_rec  = io.task.valid & !taskHit_rec

  // Set read
  io.task.ready  := cmHasFree_rec | taskHit_rec
//  io.reqDB.ready := cmHasFree_rec & Mux(io.reqDB.bits.isFullSize, dbidPool.io.hasTwo, dbidPool.io.hasOne) & !taskNeedCM_rec

  // Store Message
  msgVec.zipWithIndex.foreach {
    case(msg, i) =>
      val taskHit = io.task.fire & Mux(taskNeedCM_rec, cmFreeId_rec === i.U, io.task.bits.llcTxnID.get === msg.llcTxnID.get)
      val reqHit  = io.reqDB.fire & cmFreeId_rec === i.U
      when(taskHit) {
        msg := io.task.bits
      }.elsewhen(reqHit) {
        msg := 0.U.asTypeOf(msg)
        msg.llcTxnID := io.reqDB.bits.llcTxnID
      }
  }



}