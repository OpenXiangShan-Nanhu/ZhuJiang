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
import xs.utils.mbist.MbistPipeline
import xs.utils.queue.FastQueue
import xs.utils.sram.DualPortSramTemplate
import zhujiang.utils.SramPwrCtlBoring

class DataBuffer(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Req In
    val readToCHI     = Flipped(Decoupled(new ReadDB))
    val readToDS      = Flipped(Decoupled(new ReadDB))
    val cleanMaskVec  = Vec(djparam.nrBeat, Flipped(Valid(new DBID)))
    // Data In
    val respDS        = Flipped(Valid(new DsResp))
    val fromCHI       = Flipped(Decoupled(new PackDataFilt with HasDBID)) // Only use rxDat.Data/DataID/BE in DataBlock
    // Data Out
    val writeDS       = Decoupled(new DsWrite with HasBeatNum)
    val toCHI         = Decoupled(new PackDataFilt with HasDCID)
  })

  /*
   * Module and Reg declaration
   */
  // SRAM
  val datBuf      = Module(new DualPortSramTemplate(
    gen           = UInt(8.W),
    set           = djparam.nrDataBuf,
    way           = djparam.BeatByte,
    hasMbist      = hasMbist,
    suffix        = "_llc_db",
    powerCtl      = true
  ))
  SramPwrCtlBoring.addSink(datBuf.io.pwctl)
  // Mask and replace flag
  val maskRegVec  = RegInit(VecInit(Seq.fill(djparam.nrDataBuf) { 0.U(djparam.BeatByte.W) }))
  val replRegVec  = RegInit(VecInit(Seq.fill(djparam.nrDataBuf) { false.B }))
  // Reading Reg
  val rToCHIReg   = RegNext(io.readToCHI.fire, false.B) // Reading datBuf to CHI
  val rToDSReg    = RegNext(io.readToDS.fire,  false.B) // Reading datBuf to DS
  HAssert(!(io.readToCHI.fire & io.readToDS.fire))
  // Output Queue
  val toDSQ       = Module(new FastQueue(new DsWrite with HasBeatNum, 2, false))
  val toCHIQ      = Module(new FastQueue(new PackDataFilt with HasDCID, 2, false))

  /*
   * Receive read req
   */
  // Set ready ready
  val hasFreetoCHI      = toCHIQ.io.freeNum > rToCHIReg.asUInt
  val hasFreetoDS       = toDSQ.io.freeNum > rToDSReg.asUInt
  io.readToCHI.ready    := hasFreetoCHI & !io.readToDS.valid // TODO: has risk here
  io.readToDS.ready     := hasFreetoDS

  // Set datBuf read io
  datBuf.io.rreq.valid  := io.readToCHI.fire | io.readToDS.fire
  datBuf.io.rreq.bits   := Mux(io.readToDS.valid, io.readToDS.bits.dbid, io.readToCHI.bits.dbid)

  // Set replace flag
  replRegVec.zipWithIndex.foreach { case(r, i) =>
    val replHit   = io.readToDS.fire & io.readToDS.bits.repl  & io.readToDS.bits.dbid === i.U
    val cleanHit  = Cat(io.cleanMaskVec.map(c => c.valid & c.bits.dbid === i.U)).orR
    when(replHit) {
      r := true.B
      HAssert(!r)
    }.elsewhen(cleanHit) {
      r := false.B
    }
    HAssert(!(replHit & cleanHit))
  }

  /*
   * Receive data in
   */
  // Set ready
  io.fromCHI.ready  := !io.respDS.valid

  // Get write db message
  val dbid      = Mux(io.respDS.valid, io.respDS.bits.dbid, io.fromCHI.bits.dbid)
  val FullMask  = Fill(djparam.BeatByte, 1.U)
  val mask      = maskRegVec(dbid)
  val repl      = replRegVec(dbid)

  // Save Data
  val wDataVec                  = Mux(io.respDS.valid, io.respDS.bits.beat, io.fromCHI.bits.dat.Data).asTypeOf(Vec(djparam.BeatByte, UInt(8.W)))
  datBuf.io.wreq.valid          := io.respDS.valid | io.fromCHI.valid
  datBuf.io.wreq.bits.addr      := dbid
  datBuf.io.wreq.bits.mask.get  := Mux(io.respDS.valid, Mux(repl, FullMask, ~mask), io.fromCHI.bits.dat.BE)
  datBuf.io.wreq.bits.data.zip(wDataVec).foreach { case(a, b) => a := b }

  /*
   * Modify mask
   * TODO: optimize clock gating
   */
  maskRegVec.zipWithIndex.foreach { case(m, i) =>
    val cleanVec  = VecInit(io.cleanMaskVec.map(c => c.valid & c.bits.dbid === i.U))
    val cleanHit  = cleanVec.asUInt.orR
    val dsHit     = io.respDS.fire  & io.respDS.bits.dbid  === i.U
    val chiHit    = io.fromCHI.fire & io.fromCHI.bits.dbid === i.U
    when(cleanHit) {
      m := 0.U
    }.elsewhen(dsHit) {
      m := FullMask
    }.elsewhen(chiHit) {
      m := m | io.fromCHI.bits.dat.BE
    }
    HAssert(PopCount(cleanVec ++ Seq(dsHit, chiHit)) <= 1.U)
  }

  /*
   * Output data
   */
  // toCHIQ
  val toCHIDBID                 = RegEnable(io.readToCHI.bits.dbid, io.readToCHI.fire)
  toCHIQ.io.enq.valid           := rToCHIReg
  toCHIQ.io.enq.bits.dcid       := RegEnable(io.readToCHI.bits.dcid, io.readToCHI.fire)
  toCHIQ.io.enq.bits.dat        := DontCare
  toCHIQ.io.enq.bits.dat.BE     := maskRegVec(toCHIDBID)
  toCHIQ.io.enq.bits.dat.Data   := datBuf.io.rresp.bits.asUInt
  HAssert.withEn(toCHIQ.io.enq.ready,   toCHIQ.io.enq.valid)
  HAssert.withEn(datBuf.io.rresp.valid, toCHIQ.io.enq.valid)

  // ToDSQ
  toDSQ.io.enq.valid            := rToDSReg
  toDSQ.io.enq.bits.ds          := RegEnable(io.readToDS.bits.ds,      io.readToDS.fire)
  toDSQ.io.enq.bits.beatNum     := RegEnable(io.readToDS.bits.beatNum, io.readToDS.fire)
  toDSQ.io.enq.bits.beat        := datBuf.io.rresp.bits.asUInt
  HAssert.withEn(toDSQ.io.enq.ready,    toDSQ.io.enq.valid)
  HAssert.withEn(datBuf.io.rresp.valid, toDSQ.io.enq.valid)

  // Output to IO
  io.toCHI    <> toCHIQ.io.deq
  io.writeDS  <> toDSQ.io.deq


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}