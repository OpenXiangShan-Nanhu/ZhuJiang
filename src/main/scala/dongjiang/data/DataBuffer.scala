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
import zhujiang.chi.DatOpcode._
import zhujiang.chi.RspOpcode.Comp

class DataBuffer(powerCtl: Boolean)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Req In
    val readToCHI     = Flipped(Decoupled(new ReadDB))
    val readToDS      = Flipped(Decoupled(new ReadDB))
    val rstFlagVec    = Vec(djparam.nrBeat, Flipped(Valid(new DBID)))
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
    powerCtl      = powerCtl
  ))
  SramPwrCtlBoring.addSink(datBuf.io.pwctl)
  // Mask and replace flag
  val maskRegVec  = Reg(Vec(djparam.nrDataBuf, UInt(djparam.BeatByte.W)))
  val replRegVec  = Reg(Vec(djparam.nrDataBuf, Bool()))
  // Reading Reg
  val rToCHISftReg= RegInit(0.U(2.W))
  val rToDSSftReg = RegInit(0.U(2.W))
  rToCHISftReg    := Cat(io.readToCHI.fire, rToCHISftReg(1)) // Reading datBuf to CHI
  rToDSSftReg     := Cat(io.readToDS.fire,  rToDSSftReg(1))  // Reading datBuf to DS
  HAssert(!(io.readToCHI.fire & io.readToDS.fire))
  // Output Queue
  val toDSQ       = Module(new FastQueue(new DsWrite with HasBeatNum, 2, false))
  val toCHIQ      = Module(new FastQueue(new PackDataFilt with HasDCID, 2, false))

  /*
   * Receive read req
   */
  // Set ready ready
  val hasFreetoCHI      = toCHIQ.io.freeNum > PopCount(rToCHISftReg)
  val hasFreetoDS       = toDSQ.io.freeNum  > PopCount(rToDSSftReg)
  io.readToCHI.ready    := hasFreetoCHI & !io.readToDS.valid // TODO: has risk here
  io.readToDS.ready     := hasFreetoDS

  // Set datBuf read io
  datBuf.io.rreq.valid  := RegNext(io.readToCHI.fire | io.readToDS.fire, false.B)
  datBuf.io.rreq.bits   := RegEnable(Mux(io.readToDS.valid, io.readToDS.bits.dbid, io.readToCHI.bits.dbid), io.readToCHI.fire | io.readToDS.fire)

  // Set replace flag
  replRegVec.zipWithIndex.foreach { case(r, i) =>
    val replHit = io.readToDS.fire & io.readToDS.bits.repl  & io.readToDS.bits.dbid === i.U
    val rstHit  = Cat(io.rstFlagVec.map(c => c.valid & c.bits.dbid === i.U)).orR
    when(replHit) {
      r := true.B
      HAssert(!r)
    }.elsewhen(rstHit) {
      r := false.B
    }
    HAssert(!(replHit & rstHit))
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
  datBuf.io.wreq.bits.mask.get  := Mux(io.respDS.valid, Mux(repl, FullMask, ~mask), PriorityMux(Seq(
  (io.fromCHI.bits.dat.Opcode === CompData             | io.fromCHI.bits.dat.Opcode === SnpRespData       | io.fromCHI.bits.dat.Opcode === SnpRespDataFwded) -> ~mask,
  (io.fromCHI.bits.dat.Opcode === NonCopyBackWriteData | io.fromCHI.bits.dat.Opcode === CopyBackWriteData | io.fromCHI.bits.dat.Opcode === NCBWrDataCompAck) -> io.fromCHI.bits.dat.BE)))
  datBuf.io.wreq.bits.data.zip(wDataVec).foreach { case(a, b) => a := b }

  // HAssert
  HAssert.withEn(io.fromCHI.bits.dat.Opcode === CompData | 
                 io.fromCHI.bits.dat.Opcode === SnpRespData | 
                 io.fromCHI.bits.dat.Opcode === SnpRespDataFwded | 
                 io.fromCHI.bits.dat.Opcode === NonCopyBackWriteData | 
                 io.fromCHI.bits.dat.Opcode === CopyBackWriteData |
                 io.fromCHI.bits.dat.Opcode === NCBWrDataCompAck, io.fromCHI.fire)
  HAssert.withEn(io.fromCHI.bits.dat.BE === FullMask, io.fromCHI.fire & io.fromCHI.bits.dat.Opcode === SnpRespData)
  HAssert.withEn(io.fromCHI.bits.dat.BE === FullMask, io.fromCHI.fire & io.fromCHI.bits.dat.Opcode === SnpRespDataFwded)
  /*
   * Modify mask
   * TODO: optimize clock gating
   */
  maskRegVec.zipWithIndex.foreach { case(m, i) =>
    val rstVec  = VecInit(io.rstFlagVec.map(c => c.valid & c.bits.dbid === i.U))
    val rstHit  = rstVec.asUInt.orR
    val dsHit   = io.respDS.fire  & io.respDS.bits.dbid  === i.U
    val chiHit  = io.fromCHI.fire & io.fromCHI.bits.dbid === i.U
    when(rstHit) {
      m := 0.U
    }.elsewhen(dsHit) {
      m := FullMask
    }.elsewhen(chiHit) {
      m :=  Mux(io.fromCHI.bits.dat.Opcode === CompData | io.fromCHI.bits.dat.Opcode === SnpRespData | io.fromCHI.bits.dat.Opcode === SnpRespDataFwded, FullMask, m | io.fromCHI.bits.dat.BE)
    }
    HAssert(PopCount(rstVec ++ Seq(dsHit, chiHit)) <= 1.U)
  }

  /*
   * Output data
   */
  // toCHIQ
  val toCHIDBID                 =  RegEnable(RegEnable(io.readToCHI.bits.dbid, io.readToCHI.fire), rToCHISftReg(1))
  toCHIQ.io.enq.valid           := rToCHISftReg(0)
  toCHIQ.io.enq.bits.dcid       := RegEnable(RegEnable(io.readToCHI.bits.dcid, io.readToCHI.fire), rToCHISftReg(1))
  toCHIQ.io.enq.bits.dat        := DontCare
  toCHIQ.io.enq.bits.dat.BE     := maskRegVec(toCHIDBID)
  toCHIQ.io.enq.bits.dat.Data   := datBuf.io.rresp.bits.asUInt
  HAssert.withEn(toCHIQ.io.enq.ready,   toCHIQ.io.enq.valid)
  HAssert.withEn(datBuf.io.rresp.valid, toCHIQ.io.enq.valid)

  // ToDSQ
  toDSQ.io.enq.valid            := rToDSSftReg(0)
  toDSQ.io.enq.bits.ds          := RegEnable(RegEnable(io.readToDS.bits.ds,      io.readToDS.fire), rToDSSftReg(1))
  toDSQ.io.enq.bits.beatNum     := RegEnable(RegEnable(io.readToDS.bits.beatNum, io.readToDS.fire), rToDSSftReg(1))
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