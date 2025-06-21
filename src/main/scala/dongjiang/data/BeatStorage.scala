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

class DsRead(implicit p: Parameters) extends DJBundle with HasDsIdx with HasDBID with HasDCID

class DsWrite(implicit p: Parameters) extends DJBundle with HasDsIdx { val beat  = UInt(BeatBits.W) }

class DsResp(implicit p: Parameters) extends DJBundle with HasDBID with HasDCID { val beat = UInt(BeatBits.W) }

class BeatStorage(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val read  = Flipped(Decoupled(new DsRead()))
    val write = Flipped(Decoupled(new DsWrite()))
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
    powerCtl    = true,
    moduleName  = Some("HomeDatRam")
  ))
  SramPwrCtlBoring.addSink(array.io.pwctl)
  val dcidPipe    = Module(new Pipe(UInt(dcIdBits.W), readDsLatency))
  val dbidPipe    = Module(new Pipe(UInt(dbIdBits.W), readDsLatency))
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

  // dcidPipe
  dcidPipe.io.enq.valid := io.read.fire
  dcidPipe.io.enq.bits  := io.read.bits.dcid

  // dbidPipe
  dbidPipe.io.enq.valid := io.read.fire
  dbidPipe.io.enq.bits  := io.read.bits.dbid


// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Get SRAM Resp and Output Resp ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //

  // Output
  io.resp.valid     := shiftReg.outResp
  io.resp.bits.beat := array.io.resp.bits.data.head
  io.resp.bits.dcid := dcidPipe.io.deq.bits
  io.resp.bits.dbid := dbidPipe.io.deq.bits

  HardwareAssertion.withEn(dcidPipe.io.deq.valid, shiftReg.outResp)
  HardwareAssertion.withEn(dbidPipe.io.deq.valid, shiftReg.outResp)
  HardwareAssertion.withEn(array.io.resp.valid, shiftReg.outResp)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}