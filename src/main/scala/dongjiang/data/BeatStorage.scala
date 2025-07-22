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
import zhujiang.utils.SramPwrCtlBoring


class Shift(implicit p: Parameters) extends DJBundle {
  // setup + hold + latency
  val read  = UInt(readDsLatency.W)
  val write = UInt(readDsLatency.W)

  def recRead(fire: Bool) = this.read   := Cat(fire, read >> 1)
  def recWri (fire: Bool) = this.write  := Cat(fire, write >> 1)

  private val hi = readDsLatency - 1
  private val lo = readDsLatency - (dsMuticycle - 1)
  def req        = read | write
  def reqReady   = if(dsMuticycle > 1) !req(hi, lo).orR else true.B
  def outResp    = read(0).asBool
}

class BeatStorage(powerCtl: Boolean)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val read  = Flipped(Decoupled(new ReadDS()))
    val write = Flipped(Decoupled(new WriteDS()))
    val resp  = Valid(new DsResp())
  })

  /*
   * SRAM, Reg and Wire declaration
   */
  val array     = Module(new SinglePortSramTemplate(
    gen         = UInt(BeatBits.W),
    set         = nrDsSet,
    setup       = djparam.dataRamSetup,
    latency     = djparam.dataRamLatency,
    extraHold   = djparam.dataRamExtraHold,
    hasMbist    = hasMbist,
    outputReg   = true,
    suffix      = "_llc_dat",
    powerCtl    = powerCtl,
    moduleName  = Some("HomeDatRam")
  ))
  SramPwrCtlBoring.addSink(array.io.pwctl)
  val respPipe    = Module(new Pipe(new DCID with HasDBID with HasBeatNum { val toCHI = Bool() }, readDsLatency))
  val shiftReg    = RegInit(0.U.asTypeOf(new Shift))
  val rstDoneReg  = RegEnable(true.B, false.B, array.io.req.ready)
  HardwareAssertion.withEn(!(array.io.req.ready ^ io.write.ready), rstDoneReg) // Check Shift Reg logic

// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------- Receive Req and Read/Write SRAM ------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  // Read / Write SRAM
  val readHit  = io.read.valid
  val writeHit = io.write.valid

  array.io.req.valid          := (readHit | writeHit) & shiftReg.reqReady & rstDoneReg
  array.io.req.bits.addr      := Mux(writeHit, io.write.bits.ds.idx, io.read.bits.ds.idx)
  array.io.req.bits.write     := writeHit
  array.io.req.bits.data.head := io.write.bits.beat
  HardwareAssertion.withEn(array.io.req.ready, array.io.req.valid)

  // Set Req Ready
  io.write.ready := rstDoneReg & shiftReg.reqReady
  io.read.ready  := rstDoneReg & shiftReg.reqReady & !io.write.valid
  shiftReg.recWri(io.write.fire)
  shiftReg.recRead(io.read.fire)

  // respPipe
  respPipe.io.enq.valid         := io.read.fire
  respPipe.io.enq.bits.dcid     := io.read.bits.dcid
  respPipe.io.enq.bits.dbid     := io.read.bits.dbid
  respPipe.io.enq.bits.beatNum  := io.read.bits.beatNum
  respPipe.io.enq.bits.toCHI    := io.read.bits.toCHI


// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Get SRAM Resp and Output Resp ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //

  // Output
  io.resp.valid         := shiftReg.outResp
  io.resp.bits.beat     := array.io.resp.bits.data.head
  io.resp.bits.dcid     := respPipe.io.deq.bits.dcid
  io.resp.bits.dbid     := respPipe.io.deq.bits.dbid
  io.resp.bits.beatNum  := respPipe.io.deq.bits.beatNum
  io.resp.bits.toCHI    := respPipe.io.deq.bits.toCHI

  HardwareAssertion.withEn(respPipe.io.deq.valid, shiftReg.outResp)
  HardwareAssertion.withEn(array.io.resp.valid, shiftReg.outResp)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}