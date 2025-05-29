package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.CutHnTxnID
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._

class DataBlock(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // CHI TX/RX DAT
    val txDat       = Decoupled(new DataFlit)
    val rxDat       = Flipped(Decoupled(new DataFlit)) // Only use rxDat.Data/DataID/BE/TxnID in DataBlock
    // Task From Frontend or Backend
    val cutHnTxnID  = Flipped(Valid(new CutHnTxnID)) // broadcast signal
    val reqDB       = Flipped(Decoupled(new HnTxnID with HasDataVec))
    val task        = Flipped(Valid(new DataTask)) // broadcast signal
    val resp        = Valid(new HnTxnID)
    val cleanDB     = Flipped(Valid(new HnTxnID with HasDataVec)) // broadcast signal
  })

  /*
   * Module declaration
   */
  val dataStorage = Seq.fill(djparam.nrDSBank) { Seq.fill(djparam.nrBeat) { Module(new BeatStorage) } }
  val dataCM      = Module(new DataCM)
  val dbidCtrl    = Module(new DBIDCtrl)
  val datBuf      = Module(new DataBuffer)

  /*
   * Connect IO
   */
  // txDat
  io.txDat.valid        := datBuf.io.toCHI.valid
  io.txDat.bits         := dataCM.io.txDatBits
  io.txDat.bits.Data    := datBuf.io.toCHI.bits.dat.Data
  io.txDat.bits.BE      := datBuf.io.toCHI.bits.dat.BE
  io.rxDat.ready        := datBuf.io.fromCHI.ready
  // resp
  io.resp               := dataCM.io.resp

  /*
   * Connect dataStorage
   */
  val readDs      = dataCM.io.readToDB.bits
  val writeDs     = datBuf.io.writeDS.bits
  val rReadyVec2  = Wire(Vec(djparam.nrDSBank, Vec(djparam.nrBeat, Bool())))
  val wReadyVec2  = Wire(Vec(djparam.nrDSBank, Vec(djparam.nrBeat, Bool())))
  dataStorage.zipWithIndex.foreach { case(ds, i) =>
    ds.zipWithIndex.foreach { case(bs, j) =>
      // read // TODO: dataCM dont ReadDS
      bs.io.read.valid      := dataCM.io.readToDB.valid & readDs.ds.bank === i.U & readDs.beatNum === j.U
      bs.io.read.bits       := readDs
      rReadyVec2(i)(j)      := bs.io.read.ready
      // write
      bs.io.write.valid     := datBuf.io.writeDS.valid & writeDs.ds.bank === i.U & writeDs.beatNum === j.U
      bs.io.write.bits      := writeDs
      wReadyVec2(i)(j)      := bs.io.write.ready
    }
  }
  HAssert(PopCount(dataStorage.flatMap(_.map(_.io.resp.valid))) <= 1.U)
  // ready
  dataCM.io.readToDB.ready  := rReadyVec2(readDs.ds.bank)(readDs.beatNum)
  datBuf.io.writeDS.ready   := wReadyVec2(writeDs.ds.bank)(writeDs.beatNum)

  /*
   * Connect dataCM
   */
  dataCM.io.cutHnTxnID          := io.cutHnTxnID
  dataCM.io.reqDBIn             <> io.reqDB
  dataCM.io.task                := io.task
  dataCM.io.clean               := io.cleanDB
  dataCM.io.txDataDCID          := datBuf.io.toCHI.bits.dcid
  dataCM.io.dsWriDB.valid       := datBuf.io.respDS.valid
  dataCM.io.dsWriDB.bits.dcid   := datBuf.io.respDS.bits.dcid
  dataCM.io.txDatFire.valid     := io.txDat.fire
  dataCM.io.txDatFire.bits.dcid := datBuf.io.toCHI.bits.dcid // TODO

  /*
   * Connect dbidCtrl
   */
  dbidCtrl.io.req               <> dataCM.io.reqDBOut
  dbidCtrl.io.release           := io.cleanDB
  dbidCtrl.io.cutHnTxnID        := io.cutHnTxnID

  /*
   * Connect datBuf
   */
  datBuf.io.readToCHI           <> dataCM.io.readToCHI
  datBuf.io.readToDS            <> dataCM.io.readToDS
  datBuf.io.respDS              := fastArb(dataStorage.flatMap(_.map(_.io.resp)))
  datBuf.io.fromCHI.valid       := io.rxDat.valid
  datBuf.io.fromCHI.bits.dat    := io.rxDat.bits
  datBuf.io.toCHI.ready         := io.txDat.ready
  datBuf.io.cleanMaskVec.zipWithIndex.foreach { case (c, i) =>
    c.valid := io.cleanDB.valid & io.cleanDB.bits.dataVec(i) & dbidCtrl.io.getDBIDVec.last.dbidVec(i).valid
    c.bits.dbid := dbidCtrl.io.getDBIDVec.last.dbidVec(i).bits // Set dbid
  }

  /*
   * Connect getDBIDVec
   */
  // connect dataCM <> dbidCtrl
  dbidCtrl.io.getDBIDVec.slice(0, dbidCtrl.io.getDBIDVec.size).zip(dataCM.io.getDBIDVec).foreach { case(a, b) => a <> b }
  // remap data from CHI TxnID and DataID to DBID
  dbidCtrl.io.getDBIDVec(3).hnTxnID := io.rxDat.bits.TxnID
  datBuf.io.fromCHI.bits.dbid       := dbidCtrl.io.getDBIDVec(3).dbidVec(Mux(io.rxDat.bits.DataID === "b00".U, 0.U, 1.U)).bits; require(djparam.nrBeat == 2)
  HAssert.withEn(dbidCtrl.io.getDBIDVec(3).dbidVec(Mux(io.rxDat.bits.DataID === "b00".U, 0.U, 1.U)).valid, io.rxDat.valid)
  // remap cleanMaskVec dbid
  dbidCtrl.io.getDBIDVec(4).hnTxnID := io.cleanDB.bits.hnTxnID
  // require
  require(dataCM.io.getDBIDVec.size + 2 == dbidCtrl.io.getDBIDVec.size)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}