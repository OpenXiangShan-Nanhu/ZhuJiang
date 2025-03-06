package dongjiang.directory

import math._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{DomainInfo, HardwareAssertion}
import xs.utils.sram.{SinglePortSramTemplate, DualPortSramTemplate}
import freechips.rocketchip.util.ReplacementPolicy

class Shift(implicit p: Parameters) extends DJBundle {
  // setup + 1 + hold + 1
  val req = UInt((readDirLatency+1).W)
  val wri = UInt((readDirLatency+1).W)

  val multicycle  = max(djparam.dirSetup, djparam.dirLatency)
  def canReceive  = if(multicycle == 1) true.B else !req(readDirLatency-1, readDirLatency+1-multicycle).orR
  def getSRAMResp = req(1).asBool & !wri(1).asBool
  def getDirResp  = req(0).asBool & !wri(1).asBool
}

class DirectoryBase(dirType: String, dirBank: Int)(implicit p: Parameters) extends DJModule {
  val param = new DirParam(dirType)
  val repl  = ReplacementPolicy.fromString("plru", param.ways)

  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val read  = Flipped(Decoupled(new Addr(dirType)))
    val write = Flipped(Decoupled(new DirEntry(dirType)))
    val resp  = Output(new DirEntry(dirType))
  })

  io <> DontCare
  io.read.ready := true.B
  HardwareAssertion(!io.read.valid)

  /*
   * SRAM, Reg and Wire declaration
   */
  val metaArray = Module(new SinglePortSramTemplate(
    gen         = Vec(param.nrMetas, new ChiState(dirType)),
    set         = param.sets,
    way         = param.ways,
    shouldReset = true,
    setup       = djparam.dirSetup,
    latency     = djparam.dirLatency,
    extraHold   = djparam.dirExtraHold,
  ))
  val tagArray  = Module(new SinglePortSramTemplate(
    gen         = UInt(param.tagBits.W),
    set         = param.sets,
    way         = param.ways,
    shouldReset = false,
    setup       = djparam.dirSetup,
    latency     = djparam.dirLatency,
    extraHold   = djparam.dirExtraHold,
  ))
  val replArray = Module(new DualPortSramTemplate(
    gen         = UInt(repl.nBits.W),
    set         = param.sets,
    way         = param.ways,
    shouldReset = true,
  ))
  val lockTable = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { 0.U.asTypeOf(new DJBundle {
    val lock    = Bool()
    val set     = UInt(param.setBits.W)
    val way     = UInt(param.wayBits.W)
  }) }) }))
  val shiftReg  = RegInit(0.U.asTypeOf(new Shift))

  metaArray.io <> DontCare
  tagArray.io <> DontCare
  replArray.io <> DontCare

  /*
   * [S0]: Receive Read/Write Req
   */
  val canRec_s0 = shiftReg.canReceive
  val hasReq_s0 = io.write.valid | io.read.valid
  val isHit_s0  = io.write.valid & io.write.bits.hit
  val reqSet_s0 = Mux(io.write.valid, io.write.bits.set, io.read.bits.set)

  // metaArray
  metaArray.io.req.valid          := hasReq_s0 & canRec_s0 & !isHit_s0
  metaArray.io.req.bits.addr      := reqSet_s0
  metaArray.io.req.bits.write     := isHit_s0
  metaArray.io.req.bits.mask.get  := UIntToOH(io.write.bits.way)
  metaArray.io.req.bits.data.foreach(_ := io.write.bits.metaVec)

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue - 2)
}