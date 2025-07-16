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
  val dataStorage = Seq.fill(djparam.nrDSBank) { Seq.fill(djparam.nrBeat) { Module(new BeatStorage(!isTop)) } }
  val dataCM      = Module(new DataCM)
  val dbidCtrl    = Module(new DBIDCtrl)
  val datBuf      = Module(new DataBuffer(!isTop))
  val rxDatQ      = Module(new Queue(new DataFlit, entries = 1, flow = false, pipe = true))
  val txDatQ      = Module(new Queue(new DataFlit, entries = 1, flow = false, pipe = true))

  /*
   * Connect IO
   */
  // txDat
  io.txDat              <> txDatQ.io.deq
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
  dataCM.io.updHnTxnID          := io.updHnTxnID
  dataCM.io.reqDBIn             <> io.reqDB
  dataCM.io.task                := io.task
  dataCM.io.clean               := io.cleanDB
  dataCM.io.dbidResp            := dbidCtrl.io.resp
  dataCM.io.dsWriDB.valid       := datBuf.io.respDS.valid
  dataCM.io.dsWriDB.bits.dcid   := datBuf.io.respDS.bits.dcid
  dataCM.io.txDatFire.valid     := txDatQ.io.enq.fire
  dataCM.io.txDatFire.bits.dcid := datBuf.io.toCHI.bits.dcid // TODO
  // Get CHI Data bits
  dataCM.io.getChiDat.valid     := txDatQ.io.enq.valid
  dataCM.io.getChiDat.dcid      := datBuf.io.toCHI.bits.dcid // TODO
  // Get dbid
  dataCM.io.getDBID.valid       := rxDatQ.io.deq.valid
  dataCM.io.getDBID.TxnID       := rxDatQ.io.deq.bits.TxnID
  dataCM.io.getDBID.DataID      := rxDatQ.io.deq.bits.DataID

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
  datBuf.io.respDS              := fastArb(dataStorage.map(bs => Pipe(fastArb(bs.map(_.io.resp)))))
  datBuf.io.fromCHI.valid       := rxDatQ.io.deq.valid
  datBuf.io.fromCHI.bits.dat    := rxDatQ.io.deq.bits
  datBuf.io.fromCHI.bits.dbid   := dataCM.io.getDBID.dbid
  datBuf.io.toCHI.ready         := txDatQ.io.enq.ready
  datBuf.io.release             := dataCM.io.release

  /*
   * Connect RxDatQ and TxDatQ
   */
  // RxDatQ
  rxDatQ.io.enq                 <> io.rxDat
  rxDatQ.io.deq.ready           := datBuf.io.fromCHI.ready
  // TxDatQ
  txDatQ.io.enq.valid           := datBuf.io.toCHI.valid
  txDatQ.io.enq.bits            := dataCM.io.getChiDat.bits
  txDatQ.io.enq.bits.Data       := datBuf.io.toCHI.bits.dat.Data
  txDatQ.io.enq.bits.BE         := datBuf.io.toCHI.bits.dat.BE

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(2)
}