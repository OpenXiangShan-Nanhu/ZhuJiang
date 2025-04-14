package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.HardwareAssertion
import xs.utils.queue.FastQueue
import dongjiang.data.State._
import xs.utils.sram.DualPortSramTemplate

class DataCtrl(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // CHI TX/RX DAT
    val txDat   = Decoupled(new DataFlit())
    val rxDat   = Flipped(Decoupled(new DataFlit())) // Only use rxDat.Data/DataID/BE in DataCtrl
    // Task From Frontend or Backend
    val reqDB   = Flipped(Decoupled(new PackLLCTxnID with HasDataVec))
    val task    = Flipped(Decoupled(new DataTask()))
    val resp    = Valid(new PackLLCTxnID())
    // To/From DataStorage
    val readDs  = Decoupled(new DsRead() with HasBeatNum)
    val writeDs = Decoupled(new DsWrite() with HasBeatNum)
    val respDs  = Flipped(Valid(new DsResp()))
  })

  /*
   * SRAM, Module, Reg and Wire declaration
   */
  // DataCtrl -> dc
  val datCM   = RegInit(VecInit(Seq.fill(djparam.nrDataCM) { 0.U.asTypeOf(new DCState()) }))
  val datMsg  = Reg(Vec(djparam.nrDataBuf, new DJBundle {
    // dbid
    val dbid0 = UInt(dbIdBits.W)
    val dbid1 = UInt(dbIdBits.W)
    // message
    val msg   = new DataTask()
    def getDBID(dataId: UInt): UInt = Mux(dataId === 0.U, dbid0, dbid1)
  }))
  // DataBuffer -> db
  val datBuf  = Module(new DualPortSramTemplate(
    gen       = new DJBundle {
      val b   = UInt(8.W) // byte
      val m   = Bool()    // mask
    },
    set       = djparam.nrDataBuf,
    way       = djparam.BeatByte,
    suffix    = "_llc_db"
  ))
  val dbidPool      = Module(new DBIDPool())
  val wDsQ          = Module(new FastQueue(new DsWrite() with HasBeatNum with HasDCID, 2, false))
  val txDatQ        = Module(new FastQueue(new DataFlit(), 2, false))
  val replQ         = Module(new FastQueue(new DataFlit(), 4, false)) // TODO
  val replNumReg    = RegInit(0.U(2.W))
  val datBuf_rData  = Wire(Vec(djparam.BeatByte, UInt(8.W)))
  val datBuf_rMask  = Wire(Vec(djparam.BeatByte, Bool()))

  /*
   * [FREE] Receive Req or Task
   */
  // CM
  val dcFreeVec_rec     = datCM.map(_.isFree)
  val cmHasFree_rec     = dcFreeVec_rec.reduce(_ | _)
  val cmFreeId_rec      = PriorityEncoder(dcFreeVec_rec)
  // DB
  val taskBlockByDB_rec = (io.task.bits.dataVec(0)  ^ dbidPool.io.deq0.valid) | (io.task.bits.dataVec(1)  ^ dbidPool.io.deq1.valid)
  val reqBlockByDB_rec  = (io.reqDB.bits.dataVec(0) ^ dbidPool.io.deq0.valid) | (io.reqDB.bits.dataVec(1) ^ dbidPool.io.deq1.valid)
  // Set ready
  io.task.ready  := (cmHasFree_rec & !taskBlockByDB_rec) | !io.task.bits.dataOp.reqs
  io.reqDB.ready := cmHasFree_rec & !reqBlockByDB_rec & (!io.task.valid | !io.task.bits.dataOp.reqs)
  // HardwareAssertion
  HardwareAssertion.withEn(io.task.bits.dataVec.asUInt =/= 0.U,  io.task.valid)
  HardwareAssertion.withEn(io.reqDB.bits.dataVec.asUInt =/= 0.U, io.reqDB.valid)
  val cmValVec = datCM.map(_.valid)
  // Task
  val taskIdHitVec = datMsg.map(_.msg.llcTxnID.get === io.task.bits.llcTxnID.get)
  HardwareAssertion.withEn(PopCount(cmValVec.zip(taskIdHitVec).map(a => a._1 & a._2)) === 0.U, io.task.valid &  io.task.bits.dataOp.reqs)
  HardwareAssertion.withEn(PopCount(cmValVec.zip(taskIdHitVec).map(a => a._1 & a._2)) === 1.U, io.task.valid & !io.task.bits.dataOp.reqs)
  // Req
  val reqidHitVec = datMsg.map(_.msg.llcTxnID.get === io.reqDB.bits.llcTxnID.get)
  HardwareAssertion.withEn(PopCount(cmValVec.zip(reqidHitVec).map(a => a._1 & a._2)) === 0.U, io.reqDB.valid)


  /*
   * [WAIT] Receive Data from CHI/DS
   */
  // rxDat
  val dcVec_rxDat = datCM.zip(datMsg).map(dc => dc._1.valid & dc._2.msg.llcTxnID.get === io.rxDat.bits.TxnID)
  val dcid_rxDat  = PriorityEncoder(dcVec_rxDat)
  val dbid_rxDat  = datMsg(dcid_rxDat).getDBID(io.rxDat.bits.DataID)
  io.rxDat.ready  := !io.respDs.valid
  // rxDS
  val dcid_rxDs   = io.respDs.bits.dcid
  val dbid_rxDs   = datMsg(dcid_rxDs).getDBID(datCM(dcid_rxDs).getDataId)
  val repl_rxDs   = datMsg(dcid_rxDs).msg.dataOp.repl
  // save
  val dcid_save   = Mux(io.respDs.valid & !repl_rxDs, dbid_rxDs, dbid_rxDat)
  val beat_save   = Mux(io.respDs.valid & !repl_rxDs, io.respDs.bits.beat, io.rxDat.bits.Data).asTypeOf(Vec(djparam.BeatByte, UInt(8.W)))
  val mask_save   = Mux(io.respDs.valid & !repl_rxDs, Fill(djparam.BeatByte, 1.U), io.rxDat.bits.BE)
  // sram
  datBuf.io.wreq.valid      := (io.respDs.valid & !repl_rxDs) | io.rxDat.valid
  datBuf.io.wreq.bits.addr  := dcid_save
  datBuf.io.wreq.bits.data.zipWithIndex.foreach { case(data, i) =>
    data.b := beat_save(i)
    data.m := mask_save(i)
  }
  datBuf.io.wreq.bits.mask.get := mask_save
  // HardwareAssertion
  HardwareAssertion.withEn(PopCount(dcVec_rxDat) === 1.U, io.rxDat.valid)
  HardwareAssertion.withEn(io.rxDat.bits.DataID === "b00".U | io.rxDat.bits.DataID === "b10".U, io.rxDat.valid)
  HardwareAssertion.withEn(datCM(dcid_rxDs).valid, io.respDs.valid)
  // replQ
  replQ.io.enq.valid        := io.respDs.valid & repl_rxDs
  replQ.io.enq.bits         := datMsg(dcid_rxDs).msg.txDat
  replQ.io.enq.bits.Data    := io.respDs.bits.beat
  replQ.io.enq.bits.BE      := Fill(djparam.BeatByte, 1.U)
  replQ.io.enq.bits.DataID  := datCM(dcid_rxDs).getDataId
  HardwareAssertion.withEn(replQ.io.enq.ready, replQ.io.enq.valid)
  // replNum
  val replNumNext = replNumReg - replQ.io.enq.fire +& (io.readDs.fire & datMsg(io.readDs.bits.dcid).msg.dataOp.repl)
  replNumReg      := replNumNext
  HardwareAssertion(replNumNext <= 4.U )


  /*
   * [READ] Read DS
   */
  // read
  val vec0_rDS    = datCM.zip(datMsg).map(dc => dc._1.valid & dc._1.isRead0 & !dc._2.msg.dataOp.repl)
  val vec1_rDS    = datCM.zip(datMsg).map(dc => dc._1.valid & dc._1.isRead1 & !dc._2.msg.dataOp.repl)
  val v0_rDS      = vec0_rDS.reduce(_ | _)
  val v1_rDS      = vec1_rDS.reduce(_ | _)
  val dcid_rDS    = Mux(v1_rDS, PriorityEncoder(vec1_rDS), PriorityEncoder(vec0_rDS))
  val dsidx_rDS   = datMsg(dcid_rDS).msg.ds
  val v_rDS       = v0_rDS | v1_rDS
  // repl
  val vec0_repl   = datCM.zip(datMsg).map(dc => dc._1.valid & dc._1.isRead0 & dc._2.msg.dataOp.repl)
  val vec1_repl   = datCM.zip(datMsg).map(dc => dc._1.valid & dc._1.isRead1 & dc._2.msg.dataOp.repl)
  val v0_repl     = vec0_repl.reduce(_ | _)
  val v1_repl     = vec1_repl.reduce(_ | _)
  val dcid_repl   = Mux(v1_repl, PriorityEncoder(vec1_repl), PriorityEncoder(vec0_repl))
  val dsidx_repl  = datMsg(dcid_repl).msg.ds
  val v_repl      = (v0_repl | v1_repl) & (replQ.io.count > replNumReg)
  // readDs
  io.readDs.valid         := v_rDS | v_repl
  io.readDs.bits.ds       := Mux(v_rDS, dsidx_rDS,      dsidx_repl)
  io.readDs.bits.beatNum  := Mux(v_rDS, v1_rDS.asUInt, v1_repl.asUInt)
  io.readDs.bits.dcid     := Mux(v_rDS, dcid_rDS,      dcid_repl)


  /*
   * [SEND] Send TxDat To CHI
   */
  val vec0_txDat  = datCM.map(cm => cm.valid & cm.isSend0)
  val vec1_txDat  = datCM.map(cm => cm.valid & cm.isSend1)
  val v0_txDat    = vec0_txDat.reduce(_ | _)
  val v1_txDat    = vec1_txDat.reduce(_ | _)
  val v_txDat     = (v0_txDat | v1_txDat) & Mux(txDatQ.io.enq.valid, txDatQ.io.freeNum > 1.U, txDatQ.io.freeNum > 0.U)
  val dcid_txDat  = Mux(v1_txDat, PriorityEncoder(vec1_txDat), PriorityEncoder(vec0_txDat))
  // txDatQ
  txDatQ.io.enq.valid       := RegNext(v_txDat)
  txDatQ.io.enq.bits        := datMsg(RegNext(dcid_txDat)).msg.txDat
  txDatQ.io.enq.bits.Data   := datBuf_rData.asUInt
  txDatQ.io.enq.bits.BE     := datBuf_rMask.asUInt
  txDatQ.io.enq.bits.DataID := Mux(RegNext(v1_txDat), "b10".U, "b00".U)
  HardwareAssertion.withEn(txDatQ.io.enq.ready, txDatQ.io.enq.valid)
  // io.txDat
  io.txDat <> fastArb(Seq(txDatQ.io.deq, replQ.io.deq))


  /*
   * [SAVE] Write DS
   */
  val vec0_wDS  = datCM.map(cm => cm.valid & cm.isSave0)
  val vec1_wDS  = datCM.map(cm => cm.valid & cm.isSave1)
  val v0_wDS    = vec0_wDS.reduce(_ | _)
  val v1_wDS    = vec1_wDS.reduce(_ | _)
  val v_wDs     = (v0_wDS | v1_wDS) & Mux(wDsQ.io.enq.valid, wDsQ.io.freeNum > 1.U, wDsQ.io.freeNum > 0.U) & !v_txDat
  val dcid_wDS  = Mux(v1_wDS, PriorityEncoder(vec1_wDS), PriorityEncoder(vec0_wDS))
  val dc_wDS    = datMsg(dcid_wDS)
  // wDsQ
  wDsQ.io.enq.valid         := RegNext(v_wDs)
  wDsQ.io.enq.bits.beat     := datBuf_rData.asUInt
  wDsQ.io.enq.bits.mask     := datBuf_rMask.asUInt
  wDsQ.io.enq.bits.ds       := RegNext(dc_wDS.msg.ds)
  wDsQ.io.enq.bits.beatNum  := RegNext(v1_wDS.asUInt)
  wDsQ.io.enq.bits.dcid     := RegNext(dcid_wDS)
  HardwareAssertion.withEn(wDsQ.io.enq.ready, wDsQ.io.enq.valid)
  // io.writeDs
  io.writeDs.valid  := wDsQ.io.deq.valid
  io.writeDs.bits   := wDsQ.io.deq.bits
  wDsQ.io.deq.ready := io.writeDs.ready


  /*
   * [SEND/SAVE] Read DatBuf
   */
  val dbid_txDat  = datMsg(dcid_txDat).getDBID(datCM(dcid_txDat).getDataId)
  val dbid_wDS    = datMsg(dcid_wDS).getDBID(datCM(dcid_wDS).getDataId)
  // read
  datBuf.io.rreq.valid  := v_txDat | v_wDs
  datBuf.io.rreq.bits   := Mux(v_txDat, dbid_txDat, dbid_wDS)
  // resp
  datBuf_rData.zip(datBuf.io.rresp.bits.map(_.b)).foreach { case (a, b) => a := b }
  datBuf_rMask.zip(datBuf.io.rresp.bits.map(_.m)).foreach { case (a, b) => a := b }

  /*
   * [RESP] Send Resp To Commit
   */
  val vec_resp  = datCM.map(cm => cm.valid & cm.isResp)
  val dcid_resp = PriorityEncoder(vec_resp)

  io.resp.valid := vec_resp.reduce(_ | _)
  io.resp.bits.llcTxnID := datMsg(dcid_resp).msg.llcTxnID


  /*
   * Modify DataCtrl
   */
  val dcVec_rel = datCM.map(cm => cm.valid & cm.isClean)
  val dcId_rel  = PriorityEncoder(dcVec_rel)
  datCM.zip(datMsg).zipWithIndex.foreach {
    case(dc, i) =>
      val taskHit       = io.task.fire  & !io.task.bits.dataOp.reqs & dc._1.valid & io.task.bits.llcTxnID.get === dc._2.msg.llcTxnID.get
      val taskAllocHit  = io.task.fire  & cmFreeId_rec === i.U & io.task.bits.dataOp.reqs
      val reqAllocHit   = io.reqDB.fire & cmFreeId_rec === i.U
      val respHit       = io.resp.fire  & dcid_resp  === i.U
      val release       = respHit       & dc._2.msg.dataOp.clean

      // Valid and DBID
      when(taskAllocHit | reqAllocHit) {
        dc._1.valid  := true.B
        dc._2.dbid0  := dbidPool.io.deq0.bits
        dc._2.dbid1  := dbidPool.io.deq1.bits
      }.elsewhen(release) {
        dc._1.valid  := false.B
      }

      // Message
      when(taskAllocHit | taskHit) {
        dc._2.msg := io.task.bits
      }.elsewhen(reqAllocHit) {
        dc._2.msg.llcTxnID := io.reqDB.bits.llcTxnID
      }.elsewhen(release) {
        dc._2.msg := 0.U.asTypeOf(dc._2.msg)
      }

      // Transfer State
      val nxs = dc._1.getNXS(dc._2.msg.dataOp, dc._2.msg.dataVec)
      val readHit = io.readDs.fire      & io.readDs.bits.dcid === i.U
      val waitHit = io.respDs.fire      & io.respDs.bits.dcid === i.U
      val sendHit = datBuf.io.rreq.fire & dcid_txDat === i.U & v_txDat
      val saveHit = datBuf.io.rreq.fire & dcid_wDS   === i.U & v_wDs & !v_txDat
      val releaseHit = dcVec_rel(i)
      dc._1.state   := PriorityMux(Seq(
        (taskAllocHit | taskHit & io.task.bits.dataOp.read)  -> Mux(io.task.bits.dataVec(0), READ0, READ1),
        (taskAllocHit | taskHit & io.task.bits.dataOp.send)  -> Mux(io.task.bits.dataVec(0), SAVE0, SAVE1),
        (taskAllocHit | taskHit & io.task.bits.dataOp.save)  -> Mux(io.task.bits.dataVec(0), SAVE0, SAVE1),
        (taskAllocHit | taskHit & io.task.bits.dataOp.clean) -> CLEAN,
        (taskAllocHit | taskHit & io.task.bits.dataOp.reqs)  -> RESP, // Nothing need to do
        (readHit | waitHit | sendHit | saveHit | releaseHit | respHit) -> nxs,
        true.B -> dc._1.state
      ))

      // Hardware Assertion
      HardwareAssertion.withEn(!dc._1.valid & dc._1.noTask,  taskAllocHit | reqAllocHit, cf"Data Ctrl Index[${i}]")
      HardwareAssertion.withEn(dc._1.noTask,  taskHit, cf"Data Ctrl Index[${i}]")
      HardwareAssertion.withEn(dc._1.valid & (dc._1.isRead0 | dc._1.isRead1) & dc._2.msg.dataOp.read,  readHit, cf"Data Ctrl Index[${i}]")
      HardwareAssertion.withEn(dc._1.valid & (dc._1.isWait0 | dc._1.isWait1) & dc._2.msg.dataOp.read,  waitHit, cf"Data Ctrl Index[${i}]")
      HardwareAssertion.withEn(dc._1.valid & (dc._1.isSend0 | dc._1.isSend1) & dc._2.msg.dataOp.send,  sendHit, cf"Data Ctrl Index[${i}]")
      HardwareAssertion.withEn(dc._1.valid & (dc._1.isSave0 | dc._1.isSave1) & dc._2.msg.dataOp.save,  saveHit, cf"Data Ctrl Index[${i}]")
      HardwareAssertion.withEn(dc._1.valid & dc._1.isClean                   & dc._2.msg.dataOp.clean, releaseHit, cf"Data Ctrl Index[${i}]")
      HardwareAssertion.withEn(dc._1.valid & dc._1.isResp,  respHit, cf"Data Ctrl Index[${i}]")
      HardwareAssertion.checkTimeout(!dc._1.valid, TIMEOUT_DATACM, cf"TIMEOUT: Data Ctrl Index[${i}]")
  }


  /*
   * DBIDPool enq and deq
   */
  // enq
  dbidPool.io.enq0.valid := dcVec_rel.reduce(_ | _) & datMsg(dcId_rel).msg.dataVec(0)
  dbidPool.io.enq1.valid := dcVec_rel.reduce(_ | _) & datMsg(dcId_rel).msg.dataVec(1)
  dbidPool.io.enq0.bits  := datMsg(dcId_rel).dbid0
  dbidPool.io.enq1.bits  := datMsg(dcId_rel).dbid1
  // deq
  dbidPool.io.deq0.ready := (io.task.fire & io.task.bits.dataOp.reqs & io.task.bits.dataVec(0)) | (io.reqDB.fire & io.reqDB.bits.dataVec(0))
  dbidPool.io.deq1.ready := (io.task.fire & io.task.bits.dataOp.reqs & io.task.bits.dataVec(1)) | (io.reqDB.fire & io.reqDB.bits.dataVec(1))

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}