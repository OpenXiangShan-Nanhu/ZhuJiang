package dongjiang.utils

import chisel3._
import chisel3.util._
import xs.utils.sram.SRAMTemplate

abstract class SramWrapperBaseIO[T <: Data](gen: T) extends Module {
  def sets: Int
  def ways: Int
  val setBits = log2Up(sets)
  require(ways > 0)
  require(setBits > 0)

  val io = IO(new Bundle {
    val req     = Flipped(Decoupled(new Bundle {
      val index = UInt(setBits.W)
      val wayMaskOpt = if(ways > 1) Some(UInt(ways.W)) else None
      val wmode = Bool()
    }))
    val data    = Flipped(Decoupled(gen))
    val resp    = Valid(Vec(ways, gen))
  })
}


class SramHmcSaveDataWrapper[T <: Data](
                                         gen:          T,
                                         set:          Int,
                                         way:          Int = 1,
                                         singlePort:   Boolean = false,
                                         shouldReset:  Boolean = false,
                                         extraReset:   Boolean = false,
                                         holdRead:     Boolean = false,
                                         bypassWrite:  Boolean = false,
                                         multicycle:   Int = 1,
                                         holdMcp:      Boolean = false,
                                         hasMbist:     Boolean = false,
                                         suffix:       String = "",
                                         powerCtl:     Boolean = false,
                                         val foundry:  String = "Unknown",
                                         val sramInst: String = "STANDARD")
  extends SramWrapperBaseIO(gen) {

  override def sets: Int = set
  override def ways: Int = way
  def mcp = multicycle

  require(multicycle >= 2)
  require(holdMcp)


  private val sram = Module(new SRAMTemplate(gen, set, way, singlePort, shouldReset, extraReset, holdRead, bypassWrite, multicycle, holdMcp, hasMbist, suffix, powerCtl, foundry, sramInst))

  private val reqFire       = io.req.fire
  private val readShiftReg  = RegInit(0.U((mcp + 1).W))
  private val reqBitsReg    = RegEnable(io.req.bits, io.req.fire)
  private val reqDataReg    = RegEnable(io.data.bits, io.req.fire)
  private val reqValidReg   = RegInit(0.U((mcp - 1).W))

  reqValidReg               := Cat(reqFire, reqValidReg) >> 1.U
  io.req.ready              := !reqValidReg.orR

  sram.io.earlyWen.get      := reqFire && io.req.bits.wmode
  sram.io.earlyRen.get      := reqFire && !io.req.bits.wmode

  sram.io.w.req.valid       := reqValidReg(mcp - 2) && reqBitsReg.wmode
  sram.io.w.req.bits.setIdx := reqBitsReg.index
  sram.io.w.req.bits.data.foreach(_ := reqDataReg)
  if(ways > 1) sram.io.w.req.bits.waymask.get := io.req.bits.wayMaskOpt.get

  sram.io.r.req.valid       := reqValidReg(mcp - 2) && !reqBitsReg.wmode
  sram.io.r.req.bits.setIdx := reqBitsReg.index

  readShiftReg  := Cat(sram.io.earlyRen.get, readShiftReg) >> 1.U
  io.resp.valid := readShiftReg(0)
  io.resp.bits  := sram.io.r.resp.data

  io.data.ready := io.req.ready

  assert(Mux(reqFire & io.req.bits.wmode, io.data.valid, true.B))
  assert(PopCount(Cat(sram.io.earlyWen.get, sram.io.w.req.valid)) <= 1.U)
  assert(PopCount(Cat(sram.io.earlyRen.get, sram.io.r.req.valid)) <= 1.U)
  assert(RegNext(sram.io.earlyWen.get, false.B) === sram.io.w.req.valid)
  assert(RegNext(sram.io.earlyRen.get, false.B) === sram.io.r.req.valid)
}


class SramHmcWrapper[T <: Data](
                                 gen:          T,
                                 set:          Int,
                                 way:          Int = 1,
                                 singlePort:   Boolean = false,
                                 shouldReset:  Boolean = false,
                                 extraReset:   Boolean = false,
                                 holdRead:     Boolean = false,
                                 bypassWrite:  Boolean = false,
                                 multicycle:   Int = 1,
                                 holdMcp:      Boolean = false,
                                 hasMbist:     Boolean = false,
                                 suffix:       String = "",
                                 powerCtl:     Boolean = false,
                                 val foundry:  String = "Unknown",
                                 val sramInst: String = "STANDARD")
  extends SramWrapperBaseIO(gen) {

  override def sets: Int = set
  override def ways: Int = way
  def mcp = multicycle

  require(multicycle >= 2)
  require(holdMcp)

  private val sram = Module(new SRAMTemplate(gen, set, way, singlePort, shouldReset, extraReset, holdRead, bypassWrite, multicycle, holdMcp, hasMbist, suffix, powerCtl, foundry, sramInst))

  private val reqFire       = io.req.fire
  private val readShiftReg  = RegInit(0.U((mcp + 1).W))
  private val reqBitsReg    = RegEnable(io.req.bits, io.req.fire)
  private val reqValidReg   = RegInit(0.U((mcp - 1).W))

  reqValidReg               := Cat(reqFire, reqValidReg) >> 1.U
  io.req.ready              := !reqValidReg.orR

  sram.io.earlyWen.get      := reqFire && io.req.bits.wmode
  sram.io.earlyRen.get      := reqFire && !io.req.bits.wmode

  sram.io.w.req.valid       := reqValidReg(mcp - 2) && reqBitsReg.wmode
  sram.io.w.req.bits.setIdx := reqBitsReg.index
  sram.io.w.req.bits.data.foreach(_ := io.data.bits)
  if(ways > 1) sram.io.w.req.bits.waymask.get := io.req.bits.wayMaskOpt.get

  sram.io.r.req.valid       := reqValidReg(mcp - 2) && !reqBitsReg.wmode
  sram.io.r.req.bits.setIdx := reqBitsReg.index

  readShiftReg  := Cat(sram.io.earlyRen.get, readShiftReg) >> 1.U
  io.resp.valid := readShiftReg(0)
  io.resp.bits  := sram.io.r.resp.data

  io.data.ready := RegNext(sram.io.w.req.valid)

  assert(Mux(sram.io.w.req.valid, io.data.valid, true.B))
  assert(Mux(RegNext(sram.io.w.req.valid), io.data.valid, true.B))
  assert(PopCount(Cat(sram.io.earlyWen.get, sram.io.w.req.valid)) <= 1.U)
  assert(PopCount(Cat(sram.io.earlyRen.get, sram.io.r.req.valid)) <= 1.U)
  assert(RegNext(sram.io.earlyWen.get, false.B) === sram.io.w.req.valid)
  assert(RegNext(sram.io.earlyRen.get, false.B) === sram.io.r.req.valid)
}



class SramMcpWrapper[T <: Data](
                                 gen:          T,
                                 set:          Int,
                                 way:          Int = 1,
                                 singlePort:   Boolean = false,
                                 shouldReset:  Boolean = false,
                                 extraReset:   Boolean = false,
                                 holdRead:     Boolean = false,
                                 bypassWrite:  Boolean = false,
                                 multicycle:   Int = 1,
                                 holdMcp:      Boolean = false,
                                 hasMbist:     Boolean = false,
                                 suffix:       String = "",
                                 powerCtl:     Boolean = false,
                                 val foundry:  String = "Unknown",
                                 val sramInst: String = "STANDARD")
  extends SramWrapperBaseIO(gen) {

  override def sets: Int = set
  override def ways: Int = way
  def mcp = multicycle

  require(multicycle >= 2)
  require(!holdMcp)

  private val sram = Module(new SRAMTemplate(gen, set, way, singlePort, shouldReset, extraReset, holdRead, bypassWrite, multicycle, holdMcp, hasMbist, suffix, powerCtl, foundry, sramInst))

  private val reqFire       = io.req.fire
  private val readShiftReg  = RegInit(0.U(mcp.W))
  private val reqValidReg   = RegInit(0.U((mcp - 1).W))

  reqValidReg               := Cat(reqFire, reqValidReg) >> 1.U
  io.req.ready              := !reqValidReg.orR

  sram.io.w.req.valid       := reqFire && io.req.bits.wmode
  sram.io.w.req.bits.setIdx := io.req.bits.index
  sram.io.w.req.bits.data.foreach(_ := io.data.bits)
  if(ways > 1) sram.io.w.req.bits.waymask.get := io.req.bits.wayMaskOpt.get

  sram.io.r.req.valid       := reqFire && !io.req.bits.wmode
  sram.io.r.req.bits.setIdx := io.req.bits.index

  readShiftReg  := Cat(sram.io.r.req.valid, readShiftReg) >> 1.U
  io.resp.valid := readShiftReg(0)
  io.resp.bits  := sram.io.r.resp.data

  io.data.ready := !reqValidReg.orR

  assert(Mux(io.req.fire & io.req.bits.wmode, io.data.valid, true.B))
}


class SramWithoutMcpWrapper[T <: Data](
                                        gen:          T,
                                        set:          Int,
                                        way:          Int = 1,
                                        singlePort:   Boolean = false,
                                        shouldReset:  Boolean = false,
                                        extraReset:   Boolean = false,
                                        holdRead:     Boolean = false,
                                        bypassWrite:  Boolean = false,
                                        multicycle:   Int = 1,
                                        holdMcp:      Boolean = false,
                                        hasMbist:     Boolean = false,
                                        suffix:       String = "",
                                        powerCtl:     Boolean = false,
                                        val foundry:  String = "Unknown",
                                        val sramInst: String = "STANDARD")
  extends SramWrapperBaseIO(gen) {

  override def sets: Int = set
  override def ways: Int = way

  require(multicycle == 1)
  require(!holdMcp)

  private val sram = Module(new SRAMTemplate(gen, set, way, singlePort, shouldReset, extraReset, holdRead, bypassWrite, multicycle, holdMcp, hasMbist, suffix, powerCtl, foundry, sramInst))

  private val reqFire       = io.req.fire

  io.req.ready              := true.B

  sram.io.w.req.valid       := reqFire && io.req.bits.wmode
  sram.io.w.req.bits.setIdx := io.req.bits.index
  sram.io.w.req.bits.data.foreach(_ := io.data.bits)
  if(ways > 1) sram.io.w.req.bits.waymask.get := io.req.bits.wayMaskOpt.get

  sram.io.r.req.valid       := reqFire && !io.req.bits.wmode
  sram.io.r.req.bits.setIdx := io.req.bits.index

  io.resp.valid := RegNext(sram.io.r.req.valid)
  io.resp.bits  := sram.io.r.resp.data

  io.data.ready := true.B

  assert(Mux(io.req.fire & io.req.bits.wmode, io.data.valid, true.B))
}


class SramWrapper[T <: Data](
                              gen:          T,
                              set:          Int,
                              way:          Int = 1,
                              singlePort:   Boolean = false,
                              shouldReset:  Boolean = false,
                              extraReset:   Boolean = false,
                              holdRead:     Boolean = false,
                              bypassWrite:  Boolean = false,
                              multicycle:   Int = 1,
                              holdMcp:      Boolean = false,
                              hmcSaveData:  Boolean = false,
                              hasMbist:     Boolean = false,
                              suffix:       String = "",
                              powerCtl:     Boolean = false,
                              val foundry:  String = "Unknown",
                              val sramInst: String = "STANDARD")
  extends SramWrapperBaseIO(gen) {

  override def sets: Int = set
  override def ways: Int = way

  val sramOpt = if (multicycle >= 2 & holdMcp & hmcSaveData) {
    Some(Module(new SramHmcSaveDataWrapper(gen, set, way, singlePort, shouldReset, extraReset, holdRead, bypassWrite, multicycle, holdMcp, hasMbist, suffix, powerCtl, foundry, sramInst)))
  } else if (multicycle >= 2 & holdMcp & !hmcSaveData) {
    Some(Module(new SramHmcWrapper(gen, set, way, singlePort, shouldReset, extraReset, holdRead, bypassWrite, multicycle, holdMcp, hasMbist, suffix, powerCtl, foundry, sramInst)))
  } else if (multicycle >= 2 & !holdMcp) {
    Some(Module(new SramMcpWrapper(gen, set, way, singlePort, shouldReset, extraReset, holdRead, bypassWrite, multicycle, holdMcp, hasMbist, suffix, powerCtl, foundry, sramInst)))
  } else if (multicycle == 1) {
    Some(Module(new SramWithoutMcpWrapper(gen, set, way, singlePort, shouldReset, extraReset, holdRead, bypassWrite, multicycle, holdMcp, hasMbist, suffix, powerCtl, foundry, sramInst)))
  } else {
    None
  }

  io <> sramOpt.get.io

}