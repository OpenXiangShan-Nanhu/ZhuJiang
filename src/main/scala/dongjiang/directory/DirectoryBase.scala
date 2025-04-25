package dongjiang.directory

import math._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{HAssert, HardwareAssertion}
import xs.utils.sram.{DualPortSramTemplate, SinglePortSramTemplate}
import freechips.rocketchip.util.ReplacementPolicy
import xs.utils.mbist.MbistPipeline
import chisel3.experimental.BundleLiterals._

class Shift(implicit p: Parameters) extends DJBundle {
  val read  = UInt(readDirLatency.W)
  val write = UInt(readDirLatency.W)
  val repl  = UInt(readDirLatency.W)

  def recRead_d0(fire: Bool) = this.read   := Cat(fire, read >> 1)
  def recRepl_d0(fire: Bool) = this.repl   := Cat(fire, repl >> 1)
  def recWri_d0 (fire: Bool) = this.write  := Cat(fire, write >> 1)

  def outDirResp_d2 = read(0).asBool
  def updTagMeta_d2 = read(0).asBool  &  repl(0).asBool
  def wriUpdRepl_d2 = write(0).asBool & !repl(0).asBool

  private val hi    = readDirLatency - 1
  private val lo    = readDirLatency - (dirMuticycle - 1)
  def req           = read | write
  def tagMetaReady  = if(dirMuticycle > 1) !req(hi, lo).orR else true.B
  def replWillWrite = (repl & read).orR // when it is repl read, cant receive new req
}

class DirectoryBase(dirType: String)(implicit p: Parameters) extends DJModule {
  override val desiredName = s"Directory${dirType.toUpperCase}"
  val param = new DirParam(dirType)
  val repl  = ReplacementPolicy.fromString("plru", param.ways)

  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config    = Input(new DJConfigIO())
    val dirBank   = Input(UInt(dirBankBits.W))
    val read      = Flipped(Decoupled(new Addr(dirType) with HasPackPosIndex))
    val write     = Flipped(Decoupled(new DirEntry(dirType) with HasPackPosIndex))
    val resp      = Valid(new DirEntry(dirType))
    val unlock    = Flipped(Valid(new PosIndex()))
  })
  dontTouch(io)

  /*
  * SRAM, Reg and Wire declaration
  */
  val metaArray = Module(new SinglePortSramTemplate(
    gen         = Vec(param.nrMetas, new ChiState(dirType)),
    set         = param.sets,
    way         = param.ways,
    shouldReset = true,
    setup       = djparam.dirRamSetup,
    latency     = djparam.dirRamLatency,
    extraHold   = djparam.dirRamExtraHold,
    outputReg   = true,
    suffix      = s"_${dirType}_meta",
    hasMbist    = hasMbist
  ))

  val tagArray  = Module(new SinglePortSramTemplate(
    gen         = UInt(param.tagBits.W),
    set         = param.sets,
    way         = param.ways,
    shouldReset = false,
    setup       = djparam.dirRamSetup,
    latency     = djparam.dirRamLatency,
    extraHold   = djparam.dirRamExtraHold,
    suffix      = s"_${dirType}_tag",
    outputReg   = true,
    hasMbist    = hasMbist
  ))

  val replArray = Module(new DualPortSramTemplate(
    gen         = UInt(repl.nBits.W),
    set         = param.sets,
    way         = 1,
    shouldReset = true,
    bypassWrite = true,
    suffix      = s"_${dirType}_repl",
    hasMbist    = hasMbist
  ))
  MbistPipeline.PlaceMbistPipeline(1, desiredName, hasMbist)

  dontTouch(metaArray.io)
  dontTouch(tagArray.io)
  dontTouch(replArray.io)

  val lockTable = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { new DJBundle {
    val valid   = Bool()
    val set     = UInt(param.setBits.W)
    val way     = UInt(param.wayBits.W)
  }.Lit(_.valid -> false.B) }) }))

  val shiftReg      = RegInit(0.U.asTypeOf(new Shift))

  val resetDoneReg  = RegEnable(true.B, false.B, metaArray.io.req.ready & replArray.io.rreq.ready & replArray.io.wreq.ready)
  HardwareAssertion.withEn(!(metaArray.io.req.ready ^ io.write.ready), resetDoneReg & !shiftReg.replWillWrite) // Check Shift Reg logic

  // [D0]: Receive Req and Read/Write SRAM

  // [D1]: Get SRAM Resp
  val reqSftReg_d1    = RegInit(VecInit(Seq.fill(readDirLatency) { 0.U.asTypeOf(new DJBundle with HasAddr with HasPackPosIndex {
    override def addrType: String = dirType
    val metaVec       = Vec(param.nrMetas, new ChiState(dirType))
    val wriWayOH      = UInt(param.ways.W)
  }) }))
  val replSftReg_d1   = RegInit(VecInit(Seq.fill(readDirLatency-1) { 0.U(repl.nBits.W) }))

  // [D2]: Select Way and Output DIR Resp
  val tagResp_d2      = tagArray.io.resp.bits.data
  val metaResp_d2     = metaArray.io.resp.bits.data
  // from d1
  val req_d2          = WireInit(0.U.asTypeOf(reqSftReg_d1.head))
  val replMes_d2      = WireInit(0.U(repl.nBits.W))
  val addrVec_d2      = WireInit(VecInit(Seq.fill(param.ways) { 0.U.asTypeOf(new DJBundle with HasAddr {
    override def addrType: String = dirType
  }) }))
  val reqTag_d2       = Wire(UInt(param.tagBits.W))  ; dontTouch(reqTag_d2)
  val reqSet_d2       = Wire(UInt(param.setBits.W))  ; dontTouch(reqSet_d2)
  val tagHitVec_d2    = Wire(Vec(param.ways, Bool())); dontTouch(tagHitVec_d2)
  val metaValVec_d2   = Wire(Vec(param.ways, Bool())); dontTouch(metaValVec_d2)
  // create in d2
  val readHit_d2      = WireInit(false.B)
  val useWayVec_d2    = WireInit(0.U(param.ways.W))
  val selWayOH_d2     = WireInit(0.U(param.ways.W))
  val newReplMes_d2   = WireInit(0.U(repl.nBits.W))


  // ---------------------------------------------------------------------------------------------------------------------- //
  // ---------------------------------------- [D0]: Receive Req and Read/Write SRAM --------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // sram read/write type                                                                                 // (read, write, repl)
  val writeHit_d0 = io.write.valid & io.write.bits.hit  & shiftReg.tagMetaReady & !shiftReg.replWillWrite // (0,    1,     0   ) -> read  repl / write repl (wriUpdRepl_d2)
  val wriNoHit_d0 = io.write.valid & !io.write.bits.hit & shiftReg.tagMetaReady & !shiftReg.replWillWrite // (1,    0,     1   ) -> read  repl / write repl (updTagMeta_d2)
  val read_d0     = io.read.valid                       & shiftReg.tagMetaReady & !shiftReg.replWillWrite // (1,    0,     0   ) -> read  repl / write repl when hit
  val repl_d0     = shiftReg.updTagMeta_d2                                                                // (0,    1,     1   )

  // common
  val reqSet_d0 = Mux(repl_d0, reqSet_d2, Mux(io.write.valid, io.write.bits.Addr.set, io.read.bits.Addr.set))

  // write message
  val wriMask_d0    = Mux(repl_d0, selWayOH_d2, io.write.bits.wayOH)
  val wriMetaVec_d0 = Mux(repl_d0, req_d2.metaVec, io.write.bits.metaVec)

  // metaArray
  metaArray.io.req.valid          := (writeHit_d0 | wriNoHit_d0 | read_d0 | repl_d0) & resetDoneReg
  metaArray.io.req.bits.addr      := reqSet_d0
  metaArray.io.req.bits.write     := writeHit_d0 | repl_d0
  metaArray.io.req.bits.mask.get  := wriMask_d0
  metaArray.io.req.bits.data.foreach(_ := wriMetaVec_d0)
  HardwareAssertion.withEn(metaArray.io.req.ready, metaArray.io.req.valid)
  HardwareAssertion.withEn(metaArray.io.req.bits.mask.get =/= 0.U, metaArray.io.req.valid & metaArray.io.req.bits.write)

  // tagArray
  tagArray.io.req.valid           := (wriNoHit_d0 | read_d0 | repl_d0) & resetDoneReg
  tagArray.io.req.bits.addr       := reqSet_d0
  tagArray.io.req.bits.write      := repl_d0
  tagArray.io.req.bits.mask.get   := selWayOH_d2
  tagArray.io.req.bits.data.foreach(_ := req_d2.Addr.tag)
  HardwareAssertion.withEn(tagArray.io.req.ready, tagArray.io.req.valid)
  HardwareAssertion.withEn(tagArray.io.req.bits.mask.get =/= 0.U, tagArray.io.req.valid & tagArray.io.req.bits.write)

  // shiftReg
  // The meta is used because all actions trigger reads or writes to the meta
  shiftReg.recRead_d0(metaArray.io.req.fire & !metaArray.io.req.bits.write)
  shiftReg.recWri_d0 (metaArray.io.req.fire & metaArray.io.req.bits.write)
  shiftReg.recRepl_d0(metaArray.io.req.fire & (wriNoHit_d0 | repl_d0))
  HardwareAssertion(!(shiftReg.read & shiftReg.write).orR)
  HardwareAssertion.withEn((shiftReg.repl & shiftReg.req).orR, shiftReg.repl.orR)

  // read/write ready
  io.read.ready   := resetDoneReg & shiftReg.tagMetaReady & !shiftReg.replWillWrite & !io.write.valid
  io.write.ready  := resetDoneReg & shiftReg.tagMetaReady & !shiftReg.replWillWrite
  HardwareAssertion.withEn(metaArray.io.req.ready, shiftReg.updTagMeta_d2)
  HardwareAssertion.withEn(tagArray.io.req.ready,  shiftReg.updTagMeta_d2)

  // replArray
  // read
  replArray.io.rreq.valid         := (wriNoHit_d0 | wriNoHit_d0 | read_d0) & resetDoneReg
  replArray.io.rreq.bits          := Mux(io.write.valid, io.write.bits.Addr.set, io.read.bits.Addr.set)
  // write
  replArray.io.wreq.valid         := shiftReg.wriUpdRepl_d2 | shiftReg.updTagMeta_d2 | (shiftReg.outDirResp_d2 & readHit_d2)
  replArray.io.wreq.bits.addr     := reqSet_d2
  replArray.io.wreq.bits.data(0)  := newReplMes_d2
  HardwareAssertion.withEn(replArray.io.rreq.ready, replArray.io.rreq.valid)
  HardwareAssertion.withEn(replArray.io.wreq.ready, replArray.io.wreq.valid)


  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------- [D1]: Get SRAM Resp and Update repl resp ------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // reqSftReg_d1
  when(io.write.fire | io.read.fire) {
    reqSftReg_d1.last.addr      := Mux(io.write.valid, io.write.bits.addr,    io.read.bits.addr)
    reqSftReg_d1.last.pos       := Mux(io.write.valid, io.write.bits.pos,     io.read.bits.pos)
    reqSftReg_d1.last.wriWayOH  := Mux(io.write.valid, io.write.bits.wayOH,   0.U)
    reqSftReg_d1.last.metaVec   := Mux(io.write.valid, io.write.bits.metaVec, 0.U.asTypeOf(reqSftReg_d1.last.metaVec))
  }
  reqSftReg_d1.zipWithIndex.foreach {  case (sft, i) =>
    if (i > 0) { reqSftReg_d1(i - 1) := sft }
  }

  when(shiftReg.outDirResp_d2) {
    HAssert(tagArray.io.resp.valid)
    HAssert(metaArray.io.resp.valid)
  }

  // Get Repl Resp and Update Repl Resp
  replSftReg_d1.last  := Mux(reqSet_d2 === reqSftReg_d1.last.Addr.set, newReplMes_d2, replArray.io.rresp.bits(0))
  replSftReg_d1.zipWithIndex.foreach {
    case (sft, i) =>
      if(i > 0) {
        replSftReg_d1(i-1) := Mux(reqSet_d2 === reqSftReg_d1(i).Addr.set, newReplMes_d2, sft)
      }
  }


  // ---------------------------------------------------------------------------------------------------------------------- //
  // ---------------------------------------- [D2]: Select Way and Output DIR Resp ---------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // Get from D1
  req_d2      := reqSftReg_d1.head
  replMes_d2  := replSftReg_d1.head
  addrVec_d2.zip(tagResp_d2).foreach { case(addr, tag) => addr.Addr.cat(io.config.bankId, tag, reqSet_d2, io.dirBank) }
  reqTag_d2   := req_d2.Addr.tag
  reqSet_d2   := req_d2.Addr.set

  // Get Hit Vec
  tagHitVec_d2      := addrVec_d2.map(_.Addr.tag === reqTag_d2)
  metaValVec_d2     := metaResp_d2.map(meta => Cat(meta.map(_.isValid)).orR)
  val hasInvalid_d2 = metaValVec_d2.map(!_).reduce(_ | _)
  val hitVec_d2     = tagHitVec_d2.zip(metaValVec_d2).map { case(a, b) => a & b }
  val hit_d2        = hitVec_d2.reduce(_ | _)
  readHit_d2        := shiftReg.read(0) & hit_d2
  HardwareAssertion.withEn(!hit_d2, shiftReg.updTagMeta_d2)
  HardwareAssertion.withEn(PopCount(hitVec_d2) <= 1.U, shiftReg.read(0))

  // Select Way
  useWayVec_d2      := lockTable(req_d2.pos.set).map(lock => Mux(lock.valid & lock.set === reqSet_d2, UIntToOH(lock.way), 0.U)).reduce(_ | _)
  val unuseWay_d2   = PriorityEncoder(~useWayVec_d2.asUInt)
  val replWay_d2    = repl.get_replace_way(replMes_d2)
  val hitWay_d2     = PriorityEncoder(hitVec_d2)
  val invWay_d2     = PriorityEncoder(metaValVec_d2.map(!_))
  val selIsUsing_d2 = useWayVec_d2(replWay_d2)
  val selWay_d2     = PriorityMux(Seq(
    hit_d2          -> hitWay_d2,
    hasInvalid_d2   -> invWay_d2,
    selIsUsing_d2   -> unuseWay_d2,
    true.B          -> replWay_d2
  ))
  dontTouch(hit_d2)
  dontTouch(hasInvalid_d2)
  dontTouch(selIsUsing_d2)
  dontTouch(hitWay_d2)
  dontTouch(invWay_d2)
  dontTouch(unuseWay_d2)
  dontTouch(replWay_d2)
  HardwareAssertion.withEn(PopCount(useWayVec_d2) < param.ways.U, !hit_d2 & shiftReg.read(0))

  // Output Directory Resp
  selWayOH_d2          := UIntToOH(selWay_d2)
  io.resp.valid        := shiftReg.outDirResp_d2
  io.resp.bits.addr    := addrVec_d2(selWay_d2).addr
  io.resp.bits.wayOH   := selWayOH_d2
  io.resp.bits.hit     := hit_d2
  io.resp.bits.metaVec := metaResp_d2(selWay_d2)

  // Get New replace message
  newReplMes_d2 := repl.get_next_state(replMes_d2,  OHToUInt(Mux(shiftReg.wriUpdRepl_d2, req_d2.wriWayOH, selWayOH_d2)))

  // Update Lock Table
  lockTable.zipWithIndex.foreach {
    case(lockSet, i) =>
      lockSet.zipWithIndex.foreach {
        case(lock, j) =>
          val hit         = req_d2.pos.idxMatch(i, j)
          val write       = !shiftReg.read(0) &  shiftReg.write(0) & !shiftReg.repl(0) & hit
          val readRepl    =  shiftReg.read(0) & !shiftReg.write(0) &  shiftReg.repl(0) & hit
          val read        =  shiftReg.read(0) & !shiftReg.write(0) & !shiftReg.repl(0) & hit
          val wriRepl     = !shiftReg.read(0) &  shiftReg.write(0) &  shiftReg.repl(0) & hit
          val clean       = io.unlock.valid & io.unlock.bits.idxMatch(i, j)

          when((read & hit_d2) | readRepl) {
            lock.valid  := true.B
            lock.way    := selWay_d2
          }.elsewhen(clean) {
            lock.valid  := false.B
            lock.way    := selWay_d2
          }

          // read_hit -> write -> clean
          //     ^                  ^
          //    lock              unlock
          //
          // read_miss -> readRepl -> wriRepl -> clean
          //                 ^                     ^
          //                lock                 unlock
          HardwareAssertion(PopCount(Seq(read & hit_d2, readRepl, clean)) <= 1.U, cf"Lock Table Index[${i.U}][${j.U}]")
          HardwareAssertion.withEn(!lock.valid, read, cf"Lock Table Index[${i.U}][${j.U}]")
          HardwareAssertion.withEn( lock.valid, write, cf"Lock Table Index[${i.U}][${j.U}]")
          HardwareAssertion.withEn(!lock.valid, readRepl, cf"Lock Table Index[${i.U}][${j.U}]")
          // HardwareAssertion.withEn( lock.valid, wriRepl, cf"Lock Table Index[${i.U}][${j.U}]") // clean may occur before wriRepl
          // HardwareAssertion.withEn( lock.valid, clean, cf"Lock Table Index[${i.U}][${j.U}]")   // It is possible to execute clean without lock
          HardwareAssertion.checkTimeout(!lock.valid, TIMEOUT_LOCK, cf"TIMEOUT: Directory Lock Index[${i.U}][${j.U}]")
      }
  }


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}