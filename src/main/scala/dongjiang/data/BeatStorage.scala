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


class Shift(implicit p: Parameters) extends DJBundle {
  // setup + hold + latency
  val read  = UInt(readDsLatency.W)
  val write = UInt(readDsLatency.W)

  def recRead(fire: Bool) = this.read   := Cat(fire, read >> 1)
  def recWri (fire: Bool) = this.write  := Cat(fire, write >> 1)

  private val hi = readDsLatency - 1
  private val lo = readDsLatency - dsMuticycle
  def req        = read | write
  def reqReady   = !req(hi, lo).orR
  def getData    = read(1).asBool
  def outResp    = read(0).asBool
}

class DsRead(implicit p: Parameters) extends DJBundle with HasDsIdx with HasDCID

class DsWrite(implicit p: Parameters) extends DJBundle with HasDsIdx {
  val beat  = UInt(BeatBits.W)
  val mask  = UInt(MaskBits.W)
}

class DsResp(implicit p: Parameters) extends DJBundle with HasDCID {
  val beat    = UInt(BeatBits.W)
}

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
    gen         = UInt(8.W),
    set         = nrDsSet,
    way         = djparam.BeatByte,
    setup       = djparam.dataSetup,
    latency     = djparam.dataLatency,
    extraHold   = djparam.dataExtraHold,
    suffix      = "_llc_dat"
  ))
  val dcidPipe    = Module(new Pipe(UInt(dcIdBits.W), readDsLatency))
  val shiftReg    = RegInit(0.U.asTypeOf(new Shift))
  val rstDoneReg  = RegEnable(true.B, false.B, array.io.req.ready)
  val datRespVec  = Wire(Vec(djparam.BeatByte, UInt(8.W)))
  val dataReg     = Reg(UInt(BeatBits.W))

// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------- Receive Req and Read/Write SRAM ------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  // Read / Write SRAM
  val readHit  = io.read.valid
  val writeHit = io.write.valid

  array.io.req.valid          := (readHit | writeHit) & shiftReg.reqReady & rstDoneReg
  array.io.req.bits.addr      := Mux(writeHit, io.write.bits.ds.idx, io.read.bits.ds.idx)
  array.io.req.bits.write     := writeHit
  array.io.req.bits.mask.get  := io.write.bits.mask
  array.io.req.bits.data.zipWithIndex.foreach {
    case(data, j) =>
      data := io.write.bits.beat.asTypeOf(Vec(djparam.BeatByte, UInt(8.W)))(j)
  }
  HardwareAssertion.withEn(array.io.req.ready, array.io.req.valid)

  // Set Req Ready
  io.write.ready := rstDoneReg & shiftReg.reqReady
  io.read.ready  := rstDoneReg & shiftReg.reqReady & !io.write.valid
  shiftReg.recWri(io.write.fire)
  shiftReg.recRead(io.read.fire)

  // dcidPipe
  dcidPipe.io.enq.valid := io.read.fire
  dcidPipe.io.enq.bits  := io.read.bits.dcid


// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Get SRAM Resp and Output Resp ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  // pre-processing: reverse
  datRespVec  := array.io.resp.bits.data.reverse
  dataReg     := Mux(shiftReg.getData, datRespVec.asUInt, dataReg)

  // Output
  io.resp.valid     := shiftReg.outResp
  io.resp.bits.beat := dataReg
  io.resp.bits.dcid := dcidPipe.io.deq.bits

  HardwareAssertion.withEn(dcidPipe.io.deq.valid, shiftReg.outResp)
  HardwareAssertion.withEn(array.io.resp.valid, shiftReg.outResp)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}