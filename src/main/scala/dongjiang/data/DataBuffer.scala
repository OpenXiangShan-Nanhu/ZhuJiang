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
import xs.utils.queue.FastQueue
import xs.utils.sram.DualPortSramTemplate
import zhujiang.utils.SramPwrCtlBoring
import zhujiang.chi.DatOpcode._
import zhujiang.chi.RspOpcode.Comp
import chisel3.experimental.BundleLiterals._

class DataBuffer(powerCtl: Boolean)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Req In
    val readToCHI     = Flipped(Decoupled(new ReadDB))
    val readToDS      = Flipped(Decoupled(new ReadDB))
    val clean         = Flipped(Valid(new DBIDVec with HasDataVec))
    // Data In
    val dsResp        = Flipped(Valid(new DsResp))
    val fromCHI       = Flipped(Decoupled(new PackDataFilt with HasDBID)) // Only use rxDat.Data/DataID/BE in DataBlock
    // Data Out
    val writeDS       = Decoupled(new WriteDS with HasBeatNum)
    val toCHI         = Decoupled(new PackDataFilt with HasDCID with HasBeatNum)
  })
  val FullMask        = Fill(djparam.BeatByte, 1.U)

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
  val toDSQ       = Module(new FastQueue(new WriteDS, 2, false))
  val toCHIQ      = Module(new FastQueue(new PackDataFilt with HasDCID with HasBeatNum, 2, false))

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
    val replHit   = io.readToDS.fire & io.readToDS.bits.repl & io.readToDS.bits.dbid === i.U
    val cleanHit  = io.clean.valid & (io.clean.bits.dataVec.asUInt & VecInit(io.clean.bits.dbidVec.map(_ === i.U)).asUInt) =/= 0.U
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
   *
   * Set Mask logic:
   *  1. Data from DS  and it is replacing          -> FullMask
   *  2. Data from DS  and it is not replacing      -> ~(mask already save)
   *  3. Data from CHI and it is Read or Snoop Data -> ~(mask already save)
   *  4. Data from CHI and it is Write Data         -> BE from CHI RxDat
   */
  // Set ready
  io.fromCHI.ready  := !io.dsResp.valid

  // Get write db message
  val wriVal        = io.dsResp.valid | io.fromCHI.valid
  val wDataVec      = Mux(io.dsResp.valid, io.dsResp.bits.beat, io.fromCHI.bits.dat.Data).asTypeOf(Vec(djparam.BeatByte, UInt(8.W)))
  val dbid          = Mux(io.dsResp.valid, io.dsResp.bits.dbid, io.fromCHI.bits.dbid)
  val dsWriReg      = RegNext(io.dsResp.valid)
  val replReg       = RegNext(replRegVec(dbid))
  val maskReg       = RegEnable(maskRegVec(dbid), wriVal)
  val fromChiBEReg  = RegEnable(io.fromCHI.bits.dat.BE, io.fromCHI.valid)
  val fromChiOp     = io.fromCHI.bits.dat.Opcode
  val readOrSnpReg  = RegNext(fromChiOp === SnpRespDataFwded | fromChiOp === SnpRespData       | fromChiOp === CompData)
  // val writeReg   = RegNext(fromChiOp === NCBWrDataCompAck | fromChiOp === CopyBackWriteData | fromChiOp === NonCopyBackWriteData)

  // Save data
  datBuf.io.wreq.valid          := RegNext(wriVal)
  datBuf.io.wreq.bits.addr      := RegEnable(dbid, wriVal)
  datBuf.io.wreq.bits.mask.get  := Mux(dsWriReg, Mux(replReg, FullMask, ~maskReg), Mux(readOrSnpReg, ~maskReg, fromChiBEReg))
  datBuf.io.wreq.bits.data.zip(wDataVec).foreach { case (a, b) => a := RegEnable(b, wriVal) }

  // HAssert
  HAssert.withEn(fromChiOp === CompData |
                 fromChiOp === SnpRespData |
                 fromChiOp === SnpRespDataFwded |
                 fromChiOp === NonCopyBackWriteData |
                 fromChiOp === CopyBackWriteData |
                 fromChiOp === NCBWrDataCompAck, io.fromCHI.fire)
  HAssert.withEn(io.fromCHI.bits.dat.BE === FullMask, io.fromCHI.fire & fromChiOp === SnpRespData)
  HAssert.withEn(io.fromCHI.bits.dat.BE === FullMask, io.fromCHI.fire & fromChiOp === SnpRespDataFwded)
  /*
   * Modify mask
   * TODO: optimize clock gating
   */
  maskRegVec.zipWithIndex.foreach { case(m, i) =>
    val cleanHit  = io.clean.valid & (io.clean.bits.dataVec.asUInt & VecInit(io.clean.bits.dbidVec.map(_ === i.U)).asUInt) =/= 0.U
    val dsHit     = io.dsResp.fire  & io.dsResp.bits.dbid  === i.U
    val chiHit    = io.fromCHI.fire & io.fromCHI.bits.dbid === i.U
    when(cleanHit) {
      m := 0.U
    }.elsewhen(dsHit) {
      m := FullMask
    }.elsewhen(chiHit) {
      m := Mux(fromChiOp === CompData | fromChiOp === SnpRespData | fromChiOp === SnpRespDataFwded, FullMask, m | io.fromCHI.bits.dat.BE)
    }
    HAssert(PopCount(Seq(cleanHit, dsHit, chiHit)) <= 1.U)
  }

  /*
   * Output data
   */
  // toCHIQ
  toCHIQ.io.enq.valid           := rToCHISftReg(0)
  val toChiDBID                 =  RegEnable(RegEnable(io.readToCHI.bits.dbid,    io.readToCHI.fire), rToCHISftReg(1))
  val toChiBeatNum              =  RegEnable(RegEnable(io.readToCHI.bits.beatNum, io.readToCHI.fire), rToCHISftReg(1))
  toCHIQ.io.enq.bits.dcid       := RegEnable(RegEnable(io.readToCHI.bits.dcid,    io.readToCHI.fire), rToCHISftReg(1))
  toCHIQ.io.enq.bits.beatNum    := toChiBeatNum
  toCHIQ.io.enq.bits.dat        := DontCare
  toCHIQ.io.enq.bits.dat.BE     := maskRegVec(toChiDBID)
  toCHIQ.io.enq.bits.dat.Data   := datBuf.io.rresp.bits.asUInt
  toCHIQ.io.enq.bits.dat.DataID := Cat(toChiBeatNum, 0.U(1.W)); require(djparam.nrBeat == 2)
  HAssert.withEn(toCHIQ.io.enq.ready,   toCHIQ.io.enq.valid)
  HAssert.withEn(datBuf.io.rresp.valid, toCHIQ.io.enq.valid)

  // ToDSQ
  toDSQ.io.enq.valid            := rToDSSftReg(0)
  toDSQ.io.enq.bits.dcid        := RegEnable(RegEnable(io.readToDS.bits.dcid,    io.readToDS.fire), rToDSSftReg(1))
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