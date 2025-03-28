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

class DataCtrl(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // CHI TX/RX DAT
    val txDat       = Decoupled(new DataFlit())
    val rxDat       = Flipped(Decoupled(new DataFlit())) // Only use rxDat.Data/DataID/BE in DataCtrl
    // Task From Frontend or Backend
    val reqDB       = Flipped(Decoupled(new PackLLCTxnID with HasDataVec))
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
  val dbidPool = Module(new DBIDPool())
  val cmValVec = cmVec.map(_.valid)
  dbidPool.io <> DontCare

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
  io.reqDB.ready := cmHasFree_rec & Mux(io.reqDB.bits.isFullSize, dbidPool.io.hasTwo, dbidPool.io.hasOne) & !taskNeedCM_rec

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

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}