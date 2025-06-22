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
import zhujiang.utils.SramPwrCtlBoring
import dongjiang.directory.DIR._

object DIR {
  val D0 = 4
  val D1 = 3
  val D2 = 2
  val D3 = 1
  val D4 = 0
}

class Shift(implicit p: Parameters) extends DJBundle {
  // value
  val read  = UInt(readDirLatency.W)
  val write = UInt(readDirLatency.W)
  val repl  = UInt(readDirLatency.W)

  // shift
  def recRead_d0(fire: Bool) = this.read   := Cat(fire, read >> 1)
  def recRepl_d0(fire: Bool) = this.repl   := Cat(fire, repl >> 1)
  def recWri_d0 (fire: Bool) = this.write  := Cat(fire, write >> 1)

  // D3
  def wriUpdRepl_d3 = write(D3).asBool & !repl(D3).asBool
  def updTagMeta_d3 = read(D3).asBool  &  repl(D3).asBool
  // D4
  def outDirResp_d4 = read(D4).asBool
  def updTagMeta_d4 = read(D4).asBool  &  repl(D4).asBool
  def wriUpdRepl_d4 = write(D4).asBool & !repl(D4).asBool

  // other
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
    val read      = Flipped(Decoupled(new Addr(dirType) with HasPackHnIdx))
    val write     = Flipped(Decoupled(new DirEntry(dirType) with HasPackHnIdx))
    val resp      = Valid(new DirEntry(dirType) with HasHnTxnID { val toRepl = Bool() })
    val unlock    = Flipped(Valid(new PackHnIdx))
  })
  dontTouch(io)

  require(readDirLatency == 4)
  require(dirMuticycle == 2)

  /*
  * SRAM, Reg and Wire declaration
  */
  // metaArray
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
    hasMbist    = hasMbist,
    powerCtl    = true
  ))
  SramPwrCtlBoring.addSink(metaArray.io.pwctl)

  // tagArray
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
    hasMbist    = hasMbist,
    powerCtl    = true
  ))
  SramPwrCtlBoring.addSink(tagArray.io.pwctl)

  // replArray
  val replArray = Module(new DualPortSramTemplate(
    gen         = UInt(repl.nBits.W),
    set         = param.sets,
    way         = 1,
    shouldReset = true,
    bypassWrite = true,
    suffix      = s"_${dirType}_repl",
    outputReg   = true,
    hasMbist    = hasMbist,
    powerCtl    = true
  ))
  SramPwrCtlBoring.addSink(replArray.io.pwctl)
  MbistPipeline.PlaceMbistPipeline(1, desiredName, hasMbist)

  dontTouch(metaArray.io)
  dontTouch(tagArray.io)
  dontTouch(replArray.io)

  // lockTable
  val lockTable = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { new DJBundle {
    val valid   = Bool()
    val set     = UInt(param.setBits.W)
    val way     = UInt(param.wayBits.W)
  }.Lit(_.valid -> false.B) }) }))
  val lockNext  = WireInit(lockTable)

  // shiftReg
  val shiftReg        = RegInit(0.U.asTypeOf(new Shift))

  // resetDoneReg
  val resetDoneReg    = RegEnable(true.B, false.B, metaArray.io.req.ready & replArray.io.rreq.ready & replArray.io.wreq.ready)
  HardwareAssertion.withEn(!(metaArray.io.req.ready ^ io.write.ready), resetDoneReg & !shiftReg.replWillWrite) // Check Shift Reg logic

  // reqSftReg
  val reqSftReg       = Reg(Vec(readDirLatency ,new DJBundle with HasAddr with HasPackHnIdx {
    override def addrType: String = dirType
    val metaVec       = Vec(param.nrMetas, new ChiState(dirType))
    val wriWayOH      = UInt(param.ways.W)
  }))
  when(io.write.fire | io.read.fire) {
    reqSftReg.last.addr      := Mux(io.write.valid, io.write.bits.addr,    io.read.bits.addr)
    reqSftReg.last.hnIdx     := Mux(io.write.valid, io.write.bits.hnIdx,   io.read.bits.hnIdx)
    reqSftReg.last.wriWayOH  := Mux(io.write.valid, io.write.bits.wayOH,   0.U)
    reqSftReg.last.metaVec   := Mux(io.write.valid, io.write.bits.metaVec, 0.U.asTypeOf(reqSftReg.last.metaVec))
  }
  reqSftReg.zipWithIndex.foreach { case(sft, i) =>
    if (i > 0) { when(shiftReg.req.orR | io.read.fire | io.write.fire) { reqSftReg(i - 1) := sft } }
  }

  // [D0]: Receive req and read/write SRAM

  // [D1]: Wait SRAM resp

  // [D2]: Update repl resp and select repl/unuse way
  val replMes_d2      = Wire(UInt(repl.nBits.W))
  val useWayVec_d2    = Wire(UInt(param.ways.W))
  dontTouch(useWayVec_d2)

  // [D3]: Determine hit
  val tagResp_d3      = tagArray.io.resp.bits.data
  val metaResp_d3     = metaArray.io.resp.bits.data
  // from d2
  val req_d3          = reqSftReg(D3)
  val replMesReg_d3   = RegEnable(replMes_d2,     shiftReg.req(D2))
  val addrVec_d3      = WireInit(VecInit(Seq.fill(param.ways) { 0.U.asTypeOf(new DJBundle with HasAddr {
    override def addrType: String = dirType
  }) }))
  val reqTag_d3       = Wire(UInt(param.tagBits.W))  ; dontTouch(reqTag_d3)
  val reqSet_d3       = Wire(UInt(param.setBits.W))  ; dontTouch(reqSet_d3)
  val tagHitVec_d3    = Wire(Vec(param.ways, Bool())); dontTouch(tagHitVec_d3)
  val metaValVec_d3   = Wire(Vec(param.ways, Bool())); dontTouch(metaValVec_d3)
  // create in d2
  val readHit_d3      = WireInit(false.B)
  val selWayOH_d3     = WireInit(0.U(param.ways.W))
  val newReplMes_d3   = WireInit(0.U(repl.nBits.W))
  val resp_d3         = Wire(chiselTypeOf(io.resp.bits))

  // [D4] Output DIR resp and update SRAM
  val req_d4          = reqSftReg.head
  val readHitReg_d4   = RegEnable(readHit_d3,     shiftReg.req(D3))
  val selWayOHReg_d4  = RegEnable(selWayOH_d3,    shiftReg.req(D3))
  val newReplMesReg_d4= RegEnable(newReplMes_d3,  shiftReg.req(D3))
  val respReg_d4      = RegEnable(resp_d3,        shiftReg.req(D3))


  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------- [D0]: Receive req and read/write SRAM / [D4]: Write SRAM  -------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // sram read/write type                                                                                 // (read, write, repl)
  val writeHit_d0 = io.write.fire &  io.write.bits.hit & shiftReg.tagMetaReady & !shiftReg.replWillWrite  // (0,    1,     0   ) -> read  repl / write repl (wriUpdRepl_d4)
  val wriNoHit_d0 = io.write.fire & !io.write.bits.hit & shiftReg.tagMetaReady & !shiftReg.replWillWrite  // (1,    0,     1   ) -> read  repl / write repl (updTagMeta_d4)
  val read_d0     = io.read.fire                       & shiftReg.tagMetaReady & !shiftReg.replWillWrite  // (1,    0,     0   ) -> read  repl / write repl when hit
  val repl_d0     = shiftReg.updTagMeta_d4                                                                // (0,    1,     1   )

  // common
  val reqSet_d0 = Mux(repl_d0, req_d4.Addr.set, Mux(io.write.valid, io.write.bits.Addr.set, io.read.bits.Addr.set))

  // write message
  val wriMask_d0    = Mux(repl_d0, selWayOHReg_d4, io.write.bits.wayOH)
  val wriMetaVec_d0 = Mux(repl_d0, req_d4.metaVec, io.write.bits.metaVec)

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
  tagArray.io.req.bits.mask.get   := selWayOHReg_d4
  tagArray.io.req.bits.data.foreach(_ := req_d4.Addr.tag)
  HardwareAssertion.withEn(tagArray.io.req.ready, tagArray.io.req.valid)
  HardwareAssertion.withEn(tagArray.io.req.bits.mask.get =/= 0.U, tagArray.io.req.valid & tagArray.io.req.bits.write)

  // shiftReg
  // The meta is used because all actions trigger reads or writes to the meta
  shiftReg.recRead_d0(metaArray.io.req.fire & !metaArray.io.req.bits.write)
  shiftReg.recWri_d0 (metaArray.io.req.fire &  metaArray.io.req.bits.write)
  shiftReg.recRepl_d0(metaArray.io.req.fire & (wriNoHit_d0 | repl_d0))
  HardwareAssertion(!(shiftReg.read & shiftReg.write).orR)
  HardwareAssertion.withEn((shiftReg.repl & shiftReg.req).orR, shiftReg.repl.orR)

  // read/write ready
  io.read.ready   := resetDoneReg & shiftReg.tagMetaReady & !shiftReg.replWillWrite & !io.write.valid
  io.write.ready  := resetDoneReg & shiftReg.tagMetaReady & !shiftReg.replWillWrite
  HardwareAssertion.withEn(metaArray.io.req.ready, shiftReg.updTagMeta_d4)
  HardwareAssertion.withEn(tagArray.io.req.ready,  shiftReg.updTagMeta_d4)

  // replArray
  // read
  replArray.io.rreq.valid         := (writeHit_d0 | wriNoHit_d0 | read_d0) & resetDoneReg
  replArray.io.rreq.bits          := Mux(io.write.valid, io.write.bits.Addr.set, io.read.bits.Addr.set)
  // write
  replArray.io.wreq.valid         := shiftReg.wriUpdRepl_d4 | shiftReg.updTagMeta_d4 | (shiftReg.outDirResp_d4 & readHitReg_d4)
  replArray.io.wreq.bits.addr     := req_d4.Addr.set
  replArray.io.wreq.bits.data(0)  := newReplMesReg_d4
  HardwareAssertion.withEn(replArray.io.rreq.ready, replArray.io.rreq.valid)
  HardwareAssertion.withEn(replArray.io.wreq.ready, replArray.io.wreq.valid)


  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------------------- [D1]: Wait SRAM resp --------------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // Nothing to do

  // ---------------------------------------------------------------------------------------------------------------------- //
  // ----------------------------------- [D2]: Update repl resp and select repl/unuse way --------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // Get Replace Message
  replMes_d2          := Mux(req_d4.Addr.set === reqSftReg(D2).Addr.set, newReplMesReg_d4, replArray.io.rresp.bits(0))
  val replRespNeedVal = shiftReg.read(D2) | (shiftReg.write(D2) & !shiftReg.repl(D2))
  HAssert(!(replRespNeedVal ^ replArray.io.rresp.valid))

  // Selet Replace/Unuse Way
  useWayVec_d2        := lockTable(reqSftReg(D2).hnIdx.pos.set).map(lock => Mux(lock.valid & lock.set === reqSftReg(D2).Addr.set, UIntToOH(lock.way), 0.U)).reduce(_ | _)
  val replWay_d2      = repl.get_replace_way(replMes_d2)
  val unuseWay_d2     = PriorityEncoder(~useWayVec_d2.asUInt)
  val selIsUsing_d2   = useWayVec_d2(replWay_d2)
  val hasUnuseWay_d2  = PopCount(useWayVec_d2) < param.ways.U

  // ---------------------------------------------------------------------------------------------------------------------- //
  // ------------------------------------------------- [D3]: Determine hit ------------------------------------------------ //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // Get from D2
  addrVec_d3.zip(tagResp_d3).foreach { case(addr, tag) => addr.Addr.cat(io.config.bankId, tag, reqSet_d3, io.dirBank) }
  reqTag_d3   := req_d3.Addr.tag
  reqSet_d3   := req_d3.Addr.set
  HAssert(!(shiftReg.read(D3) ^ tagArray.io.resp.valid))
  HAssert(!(shiftReg.read(D3) ^ metaArray.io.resp.valid))

  // Get Hit Vec
  tagHitVec_d3      := addrVec_d3.map(_.Addr.tag === reqTag_d3)
  metaValVec_d3     := metaResp_d3.map(meta => Cat(meta.map(_.isValid)).orR)
  val hasInvalid_d3 = metaValVec_d3.map(!_).reduce(_ | _)
  val hitVec_d3     = tagHitVec_d3.zip(metaValVec_d3).map { case(a, b) => a & b }
  val hit_d3        = hitVec_d3.reduce(_ | _)
  readHit_d3        := shiftReg.read(D3) & hit_d3
  HardwareAssertion.withEn(!hit_d3, shiftReg.updTagMeta_d3)
  HardwareAssertion.withEn(PopCount(hitVec_d3) <= 1.U, shiftReg.read(D3))

  // Select Way
  val selIsUsingReg_d3  = RegEnable(selIsUsing_d2,  shiftReg.req(D2))
  val unuseWayReg_d3    = RegEnable(unuseWay_d2,    shiftReg.req(D2))
  val replWayReg_d3     = RegEnable(replWay_d2,     shiftReg.req(D2))
  val hitWay_d3         = PriorityEncoder(hitVec_d3)
  val invWay_d3         = PriorityEncoder(metaValVec_d3.map(!_))
  val selWay_d3         = PriorityMux(Seq(
    hit_d3              -> hitWay_d3,
    hasInvalid_d3       -> invWay_d3,
    selIsUsingReg_d3    -> unuseWayReg_d3,
    true.B              -> replWayReg_d3
  ))
  dontTouch(selIsUsingReg_d3)
  dontTouch(unuseWayReg_d3)
  dontTouch(replWayReg_d3)
  dontTouch(hit_d3)
  dontTouch(hasInvalid_d3)
  dontTouch(hitWay_d3)
  dontTouch(invWay_d3)
  HardwareAssertion.withEn(RegNext(hasUnuseWay_d2), !hit_d3 & shiftReg.read(D3))

  // Output Directory Resp
  selWayOH_d3     := UIntToOH(selWay_d3)
  resp_d3.addr    := addrVec_d3(selWay_d3).addr
  resp_d3.wayOH   := selWayOH_d3
  resp_d3.hit     := hit_d3
  resp_d3.metaVec := metaResp_d3(selWay_d3)
  resp_d3.hnTxnID := req_d3.hnIdx.getTxnID
  resp_d3.toRepl  := shiftReg.repl(D3)

  // Get New replace message
  newReplMes_d3 := repl.get_next_state(replMesReg_d3,  OHToUInt(Mux(shiftReg.wriUpdRepl_d3, req_d3.wriWayOH, selWayOH_d3)))

  // Update Lock Table
  // ------------------------------------------------------------------------------
  // | operation | read | write | repl | hit | old | new |         node           |
  // |----------------------------------------------------------------------------|
  // | none      |   0  |   0   |   0  |  -  |  -  |  -  | no operation           |
  // | read      |   1  |   0   |   0  |  F  |  F  |  F  | read  dir without hit  |
  // | read      |   1  |   0   |   0  |  T  |  F  |  T  | read  dir with hit     |
  // | write     |   0  |   1   |   0  |  -  |  -  |  -  | write dir with hit     |
  // | readRepl  |   1  |   0   |   1  |  F  |  F  |  T  | read  dir for write    |
  // | wriRepl   |   0  |   1   |   1  |  -  |  -  |  -  | write dir without hit  |
  // ------------------------------------------------------------------------------
  // Node:
  //  1. '-' indicate dont care
  //  2. 'old' and 'new' indicate 'lock' state
  val read_d3     =  shiftReg.read(D3) & !shiftReg.write(D3) & !shiftReg.repl(D3)
  val write_d3    = !shiftReg.read(D3) &  shiftReg.write(D3) & !shiftReg.repl(D3)
  val readRepl_d3 =  shiftReg.read(D3) & !shiftReg.write(D3) &  shiftReg.repl(D3)
  val wriRepl_d3  = !shiftReg.read(D3) &  shiftReg.write(D3) &  shiftReg.repl(D3)
  lockTable.zipWithIndex.foreach { case(lockSet, i) =>
    lockSet.zipWithIndex.foreach { case(lock, j) =>
      // hnIdx
      val hnIdx       = Wire(new HnIndex)
      hnIdx.dirBank   := io.dirBank
      hnIdx.pos.set   := i.U
      hnIdx.pos.way   := j.U

      // hit message
      val unLockHit   = io.unlock.valid  & io.unlock.bits.hnIdx.asUInt === hnIdx.asUInt
      val reqHit      = shiftReg.req(D3) & req_d3.hnIdx.asUInt === hnIdx.asUInt
      val readMiss    = read_d3     & reqHit & !hit_d3
      val readHit     = read_d3     & reqHit &  hit_d3
      val write       = write_d3    & reqHit
      val readRepl    = readRepl_d3 & reqHit
      val wriRepl     = wriRepl_d3  & reqHit

      reqHit.suggestName  (f"reqHit_${i}_${j}")
      readMiss.suggestName(f"write_${i}_${j}")
      readHit.suggestName (f"readRepl_${i}_${j}")
      write.suggestName   (f"read_${i}_${j}")
      readRepl.suggestName(f"wriRepl_${i}_${j}")
      wriRepl.suggestName (f"unLockHit_${i}_${j}")

      // get state
      val dontCare  = Cat(lock.valid, lock.valid)
      val state     = PriorityMux(Seq(
        unLockHit   -> Cat(lock.valid, false.B),
        readMiss    -> "b00".U,
        readHit     -> "b01".U,
        write       -> dontCare,
        readRepl    -> "b01".U,
        wriRepl     -> dontCare,
        true.B      -> dontCare
      ))

      // modify lock state
      val (oldLock, newLock) = (state(1), state(0))
      oldLock.suggestName(f"oldLock_${i}_${j}")
      newLock.suggestName(f"newLock_${i}_${j}")
      when(!oldLock & newLock) {
        lockNext(i)(j).valid  := true.B
        lockNext(i)(j).set    := reqSet_d3
        lockNext(i)(j).way    := selWay_d3
      }.elsewhen(!newLock) {
        lockNext(i)(j).valid  := false.B
      }

      // HAssert
      HAssert(!(readMiss & readHit & readRepl & unLockHit), cf"Lock Table Index[${i.U}][${j.U}]")
      HAssert(lock.valid === oldLock, cf"Lock Table Index[${i.U}][${j.U}]")
      HAssert.checkTimeout(!lock.valid, TIMEOUT_LOCK, cf"TIMEOUT: Directory Lock Index[${i.U}][${j.U}]")
    }
  }

  when(shiftReg.req(D3) | io.unlock.valid) {
    lockTable := lockNext
  }

  // ---------------------------------------------------------------------------------------------------------------------- //
  // -------------------------------------------- [D4]: Output DIR Resp --------------------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  io.resp.valid := shiftReg.outDirResp_d4
  io.resp.bits  := respReg_d4


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}