package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.backend.UpdHnTxnID
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug._
import xs.utils.mbist.MbistPipeline

class DataBlock(isTop: Boolean = false)(implicit p: Parameters) extends DJModule {
  override def isTopModule: Boolean = isTop
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // CHI TX/RX DAT
    val txDat       = Decoupled(new DataFlit)
    val rxDat       = Flipped(Decoupled(new DataFlit)) // Only use rxDat.Data/DataID/BE/TxnID in DataBlock
    // Task From Frontend or Backend
    val updHnTxnID  = Flipped(Valid(new UpdHnTxnID)) // broadcast signal
    val reqDB       = Flipped(Decoupled(new HnTxnID with HasDataVec))
    val task        = Flipped(Valid(new DataTask)) // broadcast signal
    val resp        = Valid(new HnTxnID)
    val cleanDB     = Flipped(Valid(new HnTxnID with HasDataVec)) // broadcast signal
  })

  /*
   * Module declaration
   */
  val beatStorage = Seq.fill(djparam.nrDSBank) { Seq.fill(djparam.nrBeat) { Module(new BeatStorage(!isTop)) } }
  val dataCM      = Module(new DataCM)
  val dbidCtrl    = Module(new DBIDCtrl)
  val datBuf      = Module(new DataBuffer(!isTop))
  MbistPipeline.PlaceMbistPipeline(1, "HomeDataBlock", hasMbist)

  /*
   * Connect IO
   */
  // txDat
  val dbToCHI           = datBuf.io.toCHI.valid
  val dsToCHI           = datBuf.io.dsResp.valid & datBuf.io.dsResp.bits.toCHI
  io.txDat.valid        := dbToCHI | dsToCHI
  io.txDat.bits         := dataCM.io.getChiDat.bits
  io.txDat.bits.DataID  := Mux(dbToCHI, datBuf.io.toCHI.bits.dat.DataID, datBuf.io.dsResp.bits.getDataID)
  io.txDat.bits.Data    := Mux(dbToCHI, datBuf.io.toCHI.bits.dat.Data,   datBuf.io.dsResp.bits.beat)
  io.txDat.bits.BE      := Mux(dbToCHI, datBuf.io.toCHI.bits.dat.BE,     Fill(djparam.BeatByte, 1.U))
  // rxDat
  io.rxDat.ready        := datBuf.io.fromCHI.ready
  // resp
  io.resp               := dataCM.io.resp

  /*
   * Connect beatStorage
   */
  val readDs      = dataCM.io.readToDB.bits
  val writeDs     = datBuf.io.writeDS.bits
  val rReadyVec2  = Wire(Vec(djparam.nrDSBank, Vec(djparam.nrBeat, Bool())))
  val wReadyVec2  = Wire(Vec(djparam.nrDSBank, Vec(djparam.nrBeat, Bool())))
  beatStorage.zipWithIndex.foreach { case(ds, i) =>
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
  HAssert(PopCount(beatStorage.flatMap(_.map(_.io.resp.valid))) <= 1.U)
  // ready
  dataCM.io.readToDB.ready  := rReadyVec2(readDs.ds.bank)(readDs.beatNum)
  datBuf.io.writeDS.ready   := wReadyVec2(writeDs.ds.bank)(writeDs.beatNum)

  /*
   * Connect dataCM
   */
  dataCM.io.updHnTxnID              := io.updHnTxnID
  dataCM.io.reqDBIn                 <> io.reqDB
  dataCM.io.task                    := io.task
  dataCM.io.clean                   := io.cleanDB
  dataCM.io.dbidResp                := dbidCtrl.io.resp
  // dsWriDB
  dataCM.io.dsWriDB.valid           := datBuf.io.dsResp.fire
  dataCM.io.dsWriDB.bits.dcid       := datBuf.io.dsResp.bits.dcid
  dataCM.io.dsWriDB.bits.beatNum    := datBuf.io.dsResp.bits.beatNum
  // txDatFire
  dataCM.io.txDatFire.valid         := io.txDat.fire
  dataCM.io.txDatFire.bits.dcid     := Mux(dbToCHI, datBuf.io.toCHI.bits.dcid,    datBuf.io.dsResp.bits.dcid)
  dataCM.io.txDatFire.bits.beatNum  := Mux(dbToCHI, datBuf.io.toCHI.bits.beatNum, datBuf.io.dsResp.bits.beatNum)
  // dbWriDS
  dataCM.io.dbWriDS.valid           := datBuf.io.writeDS.fire
  dataCM.io.dbWriDS.bits.dcid       := datBuf.io.writeDS.bits.dcid
  dataCM.io.dbWriDS.bits.beatNum    := datBuf.io.writeDS.bits.beatNum
  // Get CHI Data bits
  dataCM.io.getChiDat.valid         := io.txDat.valid
  dataCM.io.getChiDat.dcid          := Mux(dbToCHI, datBuf.io.toCHI.bits.dcid,    datBuf.io.dsResp.bits.dcid)
  // Get dbid
  dataCM.io.getDBID.valid           := io.rxDat.valid
  dataCM.io.getDBID.TxnID           := io.rxDat.bits.TxnID
  dataCM.io.getDBID.DataID          := io.rxDat.bits.DataID

  /*
   * Connect dbidCtrl
   */
  dbidCtrl.io.req               <> dataCM.io.reqDBOut
  dbidCtrl.io.release           := dataCM.io.release

  /*
   * Connect datBuf
   */
  datBuf.io.readToCHI           <> dataCM.io.readToCHI
  datBuf.io.readToDS            <> dataCM.io.readToDS
  datBuf.io.dsResp              := fastArb(beatStorage.map(bs => Pipe(fastArb(bs.map(_.io.resp)))))
  datBuf.io.fromCHI.valid       := io.rxDat.valid
  datBuf.io.fromCHI.bits.dat    := io.rxDat.bits
  datBuf.io.fromCHI.bits.dbid   := dataCM.io.getDBID.dbid
  datBuf.io.toCHI.ready         := io.txDat.ready
  datBuf.io.clean.valid         := dbidCtrl.io.req.fire
  datBuf.io.clean.bits.dataVec  := dbidCtrl.io.req.bits
  datBuf.io.clean.bits.dbidVec  := dbidCtrl.io.resp

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}