package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{DomainInfo, HardwareAssertion}
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
    val reqDB = Flipped(Decoupled(new PackLLCTxnID with HasChiSize))
    val task  = Flipped(Decoupled(new DataTask()))
    val resp  = Valid(new PackLLCTxnID())
  })
  io <> DontCare

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
  dataCtrl.io.rxDat <> io.rxDat
  dataCtrl.io.reqDB <> io.reqDB
  dataCtrl.io.task <> io.task
  dataCtrl.io.respDsVec.zipWithIndex.foreach { case(vec, i) =>
    vec.zipWithIndex.foreach { case(v, j) =>
      v := dataStorage(i)(j).io.resp
    }
  }

  // dataStorage
  dataStorage.zipWithIndex.foreach { case (vec, i) =>
    vec.zipWithIndex.foreach { case (v, j) =>
      v.io.read  <> dataCtrl.io.readDsVec(i)(j)
      v.io.write <> dataCtrl.io.writeDsVec(i)(j)
    }
  }
  
  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 1)
}