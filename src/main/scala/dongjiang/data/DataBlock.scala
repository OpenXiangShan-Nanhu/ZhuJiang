package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.HardwareAssertion
import xs.utils.sram.SinglePortSramTemplate

class DataBlock(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // CHI TX/RX DAT
    val txDat = Decoupled(new DataFlit())
    val rxDat = Flipped(Decoupled(new DataFlit())) // Only use rxDat.Data/DataID/BE in DataCtrl
    // Task From Frontend or Backend
    val reqDB = Flipped(Decoupled(new PackLLCTxnID with HasDataVec))
    val task  = Flipped(Decoupled(new DataTask()))
    val resp  = Valid(new PackLLCTxnID())
  })

  /*
   * Modudle declaration
   */
  val dataCtrl    = Module(new DataCtrl())
  val dataStorage = Seq.fill(djparam.nrDSBank) { Seq.fill(2) { Module(new BeatStorage) } }

  /*
   * Connect
   */
  // io
  io.txDat <> dataCtrl.io.txDat
  io.resp := dataCtrl.io.resp

  // dataCtrl
  dataCtrl.io.rxDat   <> io.rxDat
  dataCtrl.io.reqDB   <> io.reqDB
  dataCtrl.io.task    <> io.task
  dataCtrl.io.respDs  := fastArb(dataStorage.flatMap(_.map(_.io.resp)))
  HardwareAssertion(PopCount(dataStorage.flatMap(_.map(_.io.resp.valid))) <= 1.U)

  // dataStorage
  dataCtrl.io.readDs.ready  := false.B
  dataCtrl.io.writeDs.ready := false.B
  dataStorage.zipWithIndex.foreach { case (vec, i) =>
    vec.zipWithIndex.foreach { case (v, j) =>
      val rHit = dataCtrl.io.readDs.valid  & dataCtrl.io.readDs.bits.ds.bank === i.U  & dataCtrl.io.readDs.bits.beatNum === j.U
      val wHit = dataCtrl.io.writeDs.valid & dataCtrl.io.writeDs.bits.ds.bank === i.U & dataCtrl.io.writeDs.bits.beatNum === j.U
      v.io.read.valid   := rHit
      v.io.write.valid  := wHit
      v.io.read.bits    := dataCtrl.io.readDs.bits
      v.io.write.bits   := dataCtrl.io.writeDs.bits
      when(rHit) { dataCtrl.io.readDs.ready   := v.io.read.ready  }
      when(wHit) { dataCtrl.io.writeDs.ready  := v.io.write.ready  }
    }
  }

  
  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 1)
}