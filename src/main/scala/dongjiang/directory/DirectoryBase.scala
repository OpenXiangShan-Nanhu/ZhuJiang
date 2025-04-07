package dongjiang.directory

import math._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.HardwareAssertion
import xs.utils.sram.{SinglePortSramTemplate, DualPortSramTemplate}
import freechips.rocketchip.util.ReplacementPolicy

class Shift(implicit p: Parameters) extends DJBundle {
  val read  = UInt(readDirLatency.W)
  val write = UInt(readDirLatency.W)
  val repl  = UInt(readDirLatency.W)

  def recRead_d0(fire: Bool) = this.read   := Cat(fire, read >> 1)
  def recRepl_d0(fire: Bool) = this.repl   := Cat(fire, repl >> 1)
  def recWri_d0 (fire: Bool) = this.write  := Cat(fire, write >> 1)

  def getTagMeta_d1 = read(1).asBool
  def outDirResp_d2 = read(0).asBool
  def updTagMeta_d2 = read(0).asBool  & repl(0).asBool
  def wriUpdRepl_d2 = write(0).asBool & !repl(0).asBool

  private val hi    = readDirLatency - 1
  private val lo    = readDirLatency - dirMuticycle
  def req           = read | write
  def tagMetaReady  = !req(hi, lo).orR
  def replWillWrite = (repl & read)(lo-1, 0).orR // when it is repl read, cant receive new req
}

class DirectoryBase(dirType: String, dirBank: Int)(implicit p: Parameters) extends DJModule {
  val param = new DirParam(dirType)
  val repl  = ReplacementPolicy.fromString("plru", param.ways)

  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config    = Input(new DJConfigIO())
    val read      = Flipped(Decoupled(new Addr(dirType) with HasPackPosIndex))
    val write     = Flipped(Decoupled(new DirEntry(dirType) with HasPackPosIndex))
    val resp      = Valid(new DirEntry(dirType) with HasPackPosIndex)
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
    setup       = djparam.dirSetup,
    latency     = djparam.dirLatency,
    extraHold   = djparam.dirExtraHold,
    suffix      = s"_${dirType}_meta"
  ))

  val tagArray  = Module(new SinglePortSramTemplate(
    gen         = UInt(param.tagBits.W),
    set         = param.sets,
    way         = param.ways,
    shouldReset = false,
    setup       = djparam.dirSetup,
    latency     = djparam.dirLatency,
    extraHold   = djparam.dirExtraHold,
    suffix      = s"_${dirType}_tag"
  ))

  val replArray = Module(new DualPortSramTemplate(
    gen         = UInt(repl.nBits.W),
    set         = param.sets,
    way         = 1,
    shouldReset = true,
    bypassWrite = true,
    suffix      = s"_${dirType}_repl"
  ))

  dontTouch(metaArray.io)
  dontTouch(tagArray.io)
  dontTouch(replArray.io)

  val lockTable = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { 0.U.asTypeOf(new DJBundle {
    val valid   = Bool()
    val set     = UInt(param.setBits.W)
    val way     = UInt(param.wayBits.W)
  }) }) }))

  val shiftReg      = RegInit(0.U.asTypeOf(new Shift))

  val resetDoneReg  = RegEnable(true.B, false.B, metaArray.io.req.ready & replArray.io.rreq.ready & replArray.io.wreq.ready)

  // [D0]: Receive Req and Read/Write SRAM

  // [D1]: Get SRAM Resp
  val reqSftReg_d1    = RegInit(VecInit(Seq.fill(readDirLatency) { 0.U.asTypeOf(new DJBundle with HasAddr with HasPackPosIndex {
    override def addrType: String = dirType
    val metaVec       = Vec(param.nrMetas, new ChiState(dirType))
    val wriWayOH      = UInt(param.ways.W)
  }) }))
  val replSftReg_d1   = RegInit(VecInit(Seq.fill(readDirLatency-1) { 0.U(repl.nBits.W) }))
  val tagRespReg_d1   = RegInit(VecInit(Seq.fill(param.ways) { 0.U.asTypeOf(UInt(param.tagBits.W)) }))
  val metaRespReg_d1  = RegInit(VecInit(Seq.fill(param.ways) { VecInit(Seq.fill(param.nrMetas) { 0.U.asTypeOf(new ChiState(dirType)) }) }))

  // [D2]: Select Way and Output DIR Resp
  // from d1
  val req_d2          = WireInit(0.U.asTypeOf(reqSftReg_d1.head))
  val metaVec_d2      = WireInit(0.U.asTypeOf(metaRespReg_d1))
  val replMes_d2      = WireInit(0.U(repl.nBits.W))
  val addrVec_d2      = WireInit(VecInit(Seq.fill(param.ways) { 0.U.asTypeOf(new DJBundle with HasAddr {
    override def addrType: String = dirType
  }) }))
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
  val reqSet_d0 = Mux(repl_d0, req_d2.Addr.set, Mux(io.write.valid, io.write.bits.Addr.set, io.read.bits.Addr.set))

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

  // tagArray
  tagArray.io.req.valid           := (wriNoHit_d0 | read_d0 | repl_d0) & resetDoneReg
  tagArray.io.req.bits.addr       := reqSet_d0
  tagArray.io.req.bits.write      := repl_d0
  tagArray.io.req.bits.mask.get   := selWayOH_d2
  tagArray.io.req.bits.data.foreach(_ := req_d2.Addr.tag)
  HardwareAssertion.withEn(tagArray.io.req.ready, tagArray.io.req.valid)

  // shiftReg
  // The meta is used because all actions trigger reads or writes to the meta
  shiftReg.recRead_d0(metaArray.io.req.fire & !metaArray.io.req.bits.write)
  shiftReg.recWri_d0 (metaArray.io.req.fire & metaArray.io.req.bits.write)
  shiftReg.recRepl_d0(metaArray.io.req.fire & repl_d0)
  HardwareAssertion(!(shiftReg.read & shiftReg.write).orR)
  HardwareAssertion.withEn(!(shiftReg.repl ^ shiftReg.req).orR, shiftReg.repl.orR)

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
  replArray.io.wreq.bits.addr     := req_d2.Addr.set
  replArray.io.wreq.bits.data(0)  := repl.get_next_state(replMes_d2,  OHToUInt(Mux(shiftReg.wriUpdRepl_d2, req_d2.wriWayOH, selWayOH_d2)))
  HardwareAssertion.withEn(replArray.io.rreq.ready, replArray.io.rreq.valid)
  HardwareAssertion.withEn(replArray.io.wreq.ready, replArray.io.wreq.valid)


  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------- [D1]: Get SRAM Resp and Update repl resp ------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // reqSftReg_d1
  reqSftReg_d1.last.addr      := Mux(io.write.valid, io.write.bits.addr, io.read.bits.addr)
  reqSftReg_d1.last.pos       := Mux(io.write.valid, io.write.bits.pos,  io.read.bits.pos)
  reqSftReg_d1.last.wriWayOH  := io.write.bits.wayOH
  reqSftReg_d1.last.metaVec   := io.write.bits.metaVec
  reqSftReg_d1.zipWithIndex.foreach {
    case(sft, i) =>
      if(i > 0) {
        reqSftReg_d1(i-1) := sft
      }
  }

  // tagReg_d1 and metaVecReg_d1
  when(shiftReg.getTagMeta_d1) {
    tagRespReg_d1  := tagArray.io.resp.bits.data
    metaRespReg_d1 := metaArray.io.resp.bits.data
  }
  HardwareAssertion(!(tagArray.io.resp.valid  ^ shiftReg.getTagMeta_d1))
  HardwareAssertion(!(metaArray.io.resp.valid ^ shiftReg.getTagMeta_d1))

  // Get Repl Resp and Update Repl Resp
  replSftReg_d1.last  := Mux(req_d2.Addr.set === reqSftReg_d1.last.Addr.set, newReplMes_d2, replArray.io.rresp.bits(0))
  replSftReg_d1.zipWithIndex.foreach {
    case (sft, i) =>
      if(i > 0) {
        replSftReg_d1(i-1) := Mux(req_d2.Addr.set === reqSftReg_d1(i).Addr.set, newReplMes_d2, sft)
      }
  }


  // ---------------------------------------------------------------------------------------------------------------------- //
  // ---------------------------------------- [D2]: Select Way and Output DIR Resp ---------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  // Get from D1
  req_d2      := reqSftReg_d1.head
  metaVec_d2  := metaRespReg_d1
  replMes_d2  := replSftReg_d1.head
  addrVec_d2.zip(tagRespReg_d1).foreach { case(addr, tag) => addr.Addr.cat(io.config.bankId, tag, req_d2.Addr.set, dirBank.U(dirBankBits.W)) }

  // Get Hit Vec
  val tagHitVec_d2  = addrVec_d2.map(_.Addr.tag === req_d2.Addr.tag)
  val metaHitVec_d2 = metaVec_d2.map(_.map(_.isValid).reduce(_ | _))
  val hasInvalid    = metaHitVec_d2.map(!_).reduce(_ | _)
  val hitVec_d2     = tagHitVec_d2.zip(metaHitVec_d2).map { case(a, b) => a & b }
  val hit_d2        = hitVec_d2.reduce(_ | _)
  readHit_d2        := shiftReg.read(0) & hit_d2
  HardwareAssertion.withEn(!hit_d2, shiftReg.updTagMeta_d2)

  // Select Way
  useWayVec_d2    := lockTable(req_d2.pos.set).map(lock => Mux(lock.valid & lock.set === req_d2.Addr.set, UIntToOH(lock.way), 0.U)).reduce(_ | _)
  val unuseWay_d2 = PriorityEncoder(useWayVec_d2)
  val replWay_d2  = repl.get_replace_way(replMes_d2)
  val hitWay_d2   = PriorityEncoder(hitVec_d2)
  val invWay_d2   = PriorityEncoder(metaHitVec_d2.map(!_))
  val selIsUsing  = useWayVec_d2(replWay_d2)
  selWayOH_d2     := PriorityMux(Seq(
    hit_d2        -> hitWay_d2,
    hasInvalid    -> invWay_d2,
    selIsUsing    -> unuseWay_d2,
    true.B        -> replWay_d2
  ))

  // Output Directory Resp
  val selWay            = OHToUInt(selWayOH_d2)
  io.resp.valid        := shiftReg.outDirResp_d2
  io.resp.bits.addr    := addrVec_d2(selWay).addr
  io.resp.bits.wayOH   := selWayOH_d2
  io.resp.bits.hit     := hit_d2
  io.resp.bits.metaVec := metaVec_d2(selWay)
  io.resp.bits.pos     := req_d2.pos
  HardwareAssertion.placePipe(Int.MaxValue-3)

  // Update Lock Table
  lockTable.zipWithIndex.foreach {
    case(lockSet, i) =>
      lockSet.zipWithIndex.foreach {
        case(lock, j) =>
          val readHitLock = readHit_d2             & req_d2.pos.idxMatch(i, j)
          val replLock    = shiftReg.updTagMeta_d2 & req_d2.pos.idxMatch(i, j)
          val writeLock   = shiftReg.wriUpdRepl_d2 & req_d2.pos.idxMatch(i, j)
          val cleanLock   = io.unlock.valid        & io.unlock.bits.idxMatch(i, j)
          when(readHitLock | replLock | writeLock) {
            lock.valid  := true.B
            lock.way    := selWay
          }.elsewhen(cleanLock) {
            lock.valid  := false.B
            lock.way    := selWay
          }
          HardwareAssertion(PopCount(Seq(readHitLock, replLock, writeLock, cleanLock)) <= 1.U, cf"Lock Table Index[$i][$j]")
          HardwareAssertion.withEn(!lock.valid, readHitLock | replLock | writeLock, cf"Lock Table Index[$i][$j]")
          HardwareAssertion.checkTimeout(!lock.valid, TIMEOUT_LOCK, cf"TIMEOUT: Directory Lock Index[${i}][${j}]")
      }
  }


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}