package dongjiang.pcu

import dongjiang._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.DecoupledQueue
import xs.utils.perf.HasPerfLogging
import xs.utils.sram.DualPortSramTemplate
import xs.utils.debug.{DomainInfo, HardwareAssertion, awhen}


object DBState {
  val width       = 2
  // FREE -> ALLOC -> FREE
  // FREE -> ALLOC -> READ(needClean)  -> READING(needClean)  -> FREE
  // FREE -> ALLOC -> READ(!needClean) -> READING(!needClean) -> ALLOC -> READ(needClean) -> READING(needClean) -> FREE
  val FREE        = "b00".U
  val ALLOC       = "b01".U
  val READ        = "b10".U // Ready to read
  val READING     = "b11".U // Already partially read
}


class DBEntry(implicit p: Parameters) extends DJBundle with HasToIncoID {
  val state       = UInt(DBState.width.W)
  val rBeatNum    = UInt(log2Ceil(nrBeat).W)
  val rBeatOH     = UInt(2.W)
  val needClean   = Bool()
  val exAtomic    = Bool()
  val doneAtomic  = Bool()

  def getDataID   = toDataID(rBeatNum)
  def getRNum     = Mux(rBeatOH === "b10".U, 1.U(1.W), rBeatNum)
  def isLast      = Mux(rBeatOH === "b11".U, rBeatNum === 1.U, rBeatNum === 0.U)
  def isFree      = state === DBState.FREE
  def isAlloc     = state === DBState.ALLOC
  def isRead      = state === DBState.READ
  def isReading   = state === DBState.READING
  def canRecReq   = isAlloc & (doneAtomic | !exAtomic)
}

trait HasBeatBum extends DJBundle { this: Bundle => val beatNum = UInt(1.W) }

class BeatBundle (implicit p: Parameters) extends DJBundle {
  val beat        = UInt(beatBits.W)
  val mask        = UInt(maskBits.W)
}


class DataBuffer()(implicit p: Parameters) extends DJModule with HasPerfLogging {
// --------------------- IO declaration ------------------------//
  val io = IO(Flipped(new DBBundle(hasDBRCReq = true)))

  val apu           = Module(new AtomicProcessUnit())

// --------------------- Reg and Wire declaration ------------------------//
  val beatEnrtys    = Module(new DualPortSramTemplate(new BeatBundle(), set = djparam.nrDatBuf * nrBeat, way = 1, shouldReset = false))
  val ctrlEntrys    = RegInit(VecInit(Seq.fill(djparam.nrDatBuf) { 0.U.asTypeOf(new DBEntry()) }))
  // beat queue
  val beatInQ       = Module(new DecoupledQueue(new BeatBundle with HasDBID with HasBeatBum))
  val beatOutQ      = Module(new DecoupledQueue(new NodeFDBData()))
  // apu
  val apuEntryInit  = WireInit(0.U.asTypeOf(new APUEntry())); apuEntryInit.op := AtomicOp.NONE.U
  val apuEntrys     = RegInit(VecInit(Seq.fill(djparam.nrAPU) { apuEntryInit }))
  // dbid resp queue
  val dbidRespQ     = Module(new Queue(new DBIDResp(), entries = 1, flow = false, pipe = true))
  val rBeatCanGo    = WireInit(false.B); dontTouch(rBeatCanGo)
  val readingReg    = RegInit(false.B)
  // Ensure that the order of dataFDB Out is equal to the order of dbRCReq In
  val readIdQ       = Module(new DecoupledQueue(UInt(dbIdBits.W)))

// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- GetDBID & DBIDResp ------------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
/*
 * DataBuffer Entry Relationship(nrDatBuf = 16, nrAPU = 4, nrDatBufRsvd = 2)
 *                            ---------------------------------------------------------------------------------
 *                            | 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 0A | 0B | 0C | 0D | 0E | 0F |
 * beatEntry(sram):           ---------------------------------------------------------------------------------
 *                            | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 1A | 1B | 1C | 1D | 1E | 1F |
 *                            ---------------------------------------------------------------------------------
 * ctrlEntry(state machine):  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 |  A |  B |  C |  D | *E | *F | (* expressed as reservations)
 *                            ---------------------------------------------------------------------------------
 * apuEntrys(state machine):                                                              |  3 |  2 |  1 |  0 |
 *                                                                                        ---------------------
 *
 * Req expect Atomic select logic: left to right (---->)
 * Atomic req select logic:        right to left (<----)
 * Reserved entries cant be used by Write(include CopyBack)
 */
  val dbFreeVec                 = ctrlEntrys.map(_.isFree)
  val dbWithApuFreeVec          = dbFreeVec.reverse.slice(0, djparam.nrAPU)
  val reqIsAtomic               = io.getDBID.bits.reqIsAtomic
  val reqIsWrite                = io.getDBID.bits.reqIsWrite
  val getDBIDId                 = Mux(reqIsAtomic, Fill(dbIdBits, 1.U(1.W)) - PriorityEncoder(dbWithApuFreeVec), PriorityEncoder(dbFreeVec))
  // receive
  io.getDBID.ready              := dbidRespQ.io.enq.ready
  dbidRespQ.io.enq.valid        := io.getDBID.valid
  dbidRespQ.io.enq.bits.retry   := Mux(reqIsAtomic, !dbWithApuFreeVec.reduce(_ | _), Mux(reqIsWrite, !dbFreeVec.reduce(_ | _) & getDBIDId < (djparam.nrDCURBuf - djparam.nrDatBufRsvd).U, !dbFreeVec.reduce(_ | _)))
  dbidRespQ.io.enq.bits.dbID    := getDBIDId
  dbidRespQ.io.enq.bits.to      := io.getDBID.bits.from
  dbidRespQ.io.enq.bits.entryID := io.getDBID.bits.entryID
  // output
  io.dbidResp                   <> dbidRespQ.io.deq

  val hwaFlags = Array.fill(5)(Wire(Bool()))
  for (i <- 0 until 5) {
    hwaFlags(i) := true.B
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- DATA TO DB ----------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Atomic Data
   */
  when(io.dataTDB.valid & io.dataTDB.bits.atomicVal) {
    // HardwareAssertion(io.dataTDB.bits.dbID >= nrDBWithoutAPUs.U)
    hwaFlags(0) := io.dataTDB.bits.dbID >= nrDBWithoutAPUs.U
    apuEntrys(io.dataTDB.bits.apuEID).atomic.data := io.dataTDB.bits.data
    apuEntrys(io.dataTDB.bits.apuEID).atomic.mask := io.dataTDB.bits.mask
    apuEntrys(io.dataTDB.bits.apuEID).initOff     := io.dataTDB.bits.dataID(1)
  }
  HardwareAssertion(hwaFlags(0))

  /*
   * Cache Line
   */
  val fromNodeVal             = io.dataTDB.valid & !io.dataTDB.bits.atomicVal
  val fromAPUVal              = apu.io.out.valid
  beatInQ.io.enq.valid        := fromNodeVal | fromAPUVal
  beatInQ.io.enq.bits.dbID    := Mux(fromAPUVal, apu.io.out.bits.dbID,                              io.dataTDB.bits.dbID)
  beatInQ.io.enq.bits.beatNum := Mux(fromAPUVal, apuEntrys(apu.io.out.bits.apuEID).initOff.asUInt,  toBeatNum(io.dataTDB.bits.dataID))
  beatInQ.io.enq.bits.beat    := Mux(fromAPUVal, apu.io.out.bits.data,                              io.dataTDB.bits.data)
  beatInQ.io.enq.bits.mask    := Mux(fromAPUVal, Fill(maskBits, 1.U),                               io.dataTDB.bits.mask)

  io.dataTDB.ready            := beatInQ.io.enq.ready & !apu.io.out.valid
  when(fromAPUVal) { ctrlEntrys(apu.io.out.bits.dbID).doneAtomic := true.B }

  /*
   * Write Data To SRAM
   */
  beatEnrtys.io.wreq.valid              := beatInQ.io.deq.valid
  beatEnrtys.io.wreq.bits.addr          := Cat(beatInQ.io.deq.bits.beatNum ,beatInQ.io.deq.bits.dbID); require(isPow2(djparam.nrDatBuf))
  beatEnrtys.io.wreq.bits.data(0).beat  := beatInQ.io.deq.bits.beat
  beatEnrtys.io.wreq.bits.data(0).mask  := beatInQ.io.deq.bits.mask

  beatInQ.io.deq.ready                  := beatEnrtys.io.wreq.ready

// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------- RC REQ TO DB ---------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  when(io.dbRCReqOpt.get.fire) {
    ctrlEntrys(io.dbRCReqOpt.get.bits.dbID).needClean := io.dbRCReqOpt.get.bits.isClean
    ctrlEntrys(io.dbRCReqOpt.get.bits.dbID).to        := io.dbRCReqOpt.get.bits.to
    ctrlEntrys(io.dbRCReqOpt.get.bits.dbID).rBeatOH   := io.dbRCReqOpt.get.bits.rBeatOH
    ctrlEntrys(io.dbRCReqOpt.get.bits.dbID).exAtomic  := io.dbRCReqOpt.get.bits.exAtomic
    // HardwareAssertion(Mux(io.dbRCReqOpt.get.bits.isRead, io.dbRCReqOpt.get.bits.rBeatOH =/= 0.U, true.B))
    // HardwareAssertion(Mux(io.dbRCReqOpt.get.bits.exAtomic, !ctrlEntrys(io.dbRCReqOpt.get.bits.dbID).doneAtomic, true.B))
    hwaFlags(1) := Mux(io.dbRCReqOpt.get.bits.isRead, io.dbRCReqOpt.get.bits.rBeatOH =/= 0.U, true.B)
    hwaFlags(2) := Mux(io.dbRCReqOpt.get.bits.exAtomic, !ctrlEntrys(io.dbRCReqOpt.get.bits.dbID).doneAtomic, true.B)
  }

  HardwareAssertion(hwaFlags(1))
  HardwareAssertion(hwaFlags(2))

  val canRecReq           = ctrlEntrys(io.dbRCReqOpt.get.bits.dbID).canRecReq
  readIdQ.io.enq.valid    := canRecReq & io.dbRCReqOpt.get.valid & io.dbRCReqOpt.get.bits.isRead
  readIdQ.io.enq.bits     := io.dbRCReqOpt.get.bits.dbID
  io.dbRCReqOpt.get.ready := canRecReq & readIdQ.io.enq.ready
  // assert
  HardwareAssertion(Mux(io.dbRCReqOpt.get.bits.isRead, !(io.dbRCReqOpt.get.fire ^ readIdQ.io.enq.fire), !readIdQ.io.enq.valid))
  awhen(io.dbRCReqOpt.get.valid) {
    HardwareAssertion(!ctrlEntrys(io.dbRCReqOpt.get.bits.dbID).isFree)
    HardwareAssertion(!(io.dbRCReqOpt.get.bits.exAtomic & io.dbRCReqOpt.get.bits.isClean))
  }

// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------- DATA TO NODE ---------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  // Reading
  val readingVec  = ctrlEntrys.map(_.isReading)
  val readingId   = PriorityEncoder(readingVec)
  val hasReading  = readingVec.reduce(_ | _)
  // Read
  val readId      = Mux(readIdQ.io.deq.valid, readIdQ.io.deq.bits, 0.U)
  // Select
  val selReadId   = Mux(hasReading, readingId, readId)
  val readVal     = readIdQ.io.deq.valid | readingVec.reduce(_ | _)
  // readIdQ deq ready
  readIdQ.io.deq.ready  := !hasReading & rBeatCanGo
  // assert
  HardwareAssertion(PopCount(readingVec) <= 1.U)
  HardwareAssertion(Mux(readIdQ.io.deq.valid, ctrlEntrys(readId).isRead, true.B))


  /*
   * Add RBeatNum
   */

  val rBeatFire                   = beatEnrtys.io.rreq.fire
  readingReg                      := rBeatFire
  ctrlEntrys(selReadId).rBeatNum  := Mux(rBeatFire, ctrlEntrys(selReadId).rBeatNum + 1.U, ctrlEntrys(selReadId).rBeatNum)

  /*
   * Read Data From SRAM
   */
  val rBeatNum                  = ctrlEntrys(selReadId).getRNum; require(rBeatNum.getWidth == log2Ceil(nrBeat))
  val canRSram                  = (beatOutQ.io.count + readingReg.asUInt - beatOutQ.io.deq.fire.asUInt) < 2.U; dontTouch(canRSram)
  rBeatCanGo                    := beatEnrtys.io.rreq.ready & canRSram
  // sram rreq
  beatEnrtys.io.rreq.valid      := readVal & canRSram
  beatEnrtys.io.rreq.bits       := Cat(rBeatNum, selReadId)
  // beatOutQ
  beatOutQ.io.enq.valid         := readingReg
  beatOutQ.io.enq.bits.dbID     := RegEnable(selReadId,                       rBeatFire)
  beatOutQ.io.enq.bits.dataID   := RegEnable(ctrlEntrys(selReadId).getDataID, rBeatFire)
  beatOutQ.io.enq.bits.to       := RegEnable(ctrlEntrys(selReadId).to,        rBeatFire)
  beatOutQ.io.enq.bits.data     := beatEnrtys.io.rresp.bits(0).beat
  beatOutQ.io.enq.bits.mask     := beatEnrtys.io.rresp.bits(0).mask

  HardwareAssertion(Mux(beatOutQ.io.enq.valid, beatEnrtys.io.rresp.valid, true.B))
  HardwareAssertion(Mux(beatOutQ.io.enq.valid, beatOutQ.io.enq.ready, true.B))

  /*
   * Send Data To Node
   */
  io.dataFDB.valid              := beatOutQ.io.deq.valid
  io.dataFDB.bits               := Mux(beatOutQ.io.deq.valid, beatOutQ.io.deq.bits, 0.U.asTypeOf(io.dataFDB.bits))
  beatOutQ.io.deq.ready         := io.dataFDB.ready

// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------------------------- READ DATA TO APU ------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val outDBID           = beatOutQ.io.deq.bits.dbID
  val outApuEID         = io.dataFDB.bits.apuEID
  val beatHit           = toBeatNum(io.dataFDB.bits.dataID) === apuEntrys(outApuEID).initOff.asUInt
  val exAtomic          = ctrlEntrys(outDBID).exAtomic

  apu.io.in.valid       := io.dataFDB.fire & beatHit & exAtomic;
  apu.io.in.bits.dbID   := outDBID
  apu.io.in.bits.op     := apuEntrys(outApuEID).op
  apu.io.in.bits.atomic := apuEntrys(outApuEID).atomic
  apu.io.in.bits.data   := io.dataFDB.bits.data

  HardwareAssertion(Mux(apu.io.in.valid, apuEntrys(outApuEID).op =/= AtomicOp.NONE.U, true.B))
  HardwareAssertion(Mux(apu.io.in.valid, outDBID >= nrDBWithoutAPUs.U, true.B))

  val hwa_flag1 = Wire(Bool())
  hwa_flag1 := true.B

  val hwa_flag2 = Wire(Bool())
  hwa_flag2 := true.B

// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- State Transfer ------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  ctrlEntrys.zipWithIndex.foreach {
    case (e, i) =>
      val apuEID      = (djparam.nrDatBuf - 1 - i).U(apuIdBits - 1, 0)
      switch(e.state) {
        // FREE
        is(DBState.FREE) {
          val hit     = io.getDBID.fire & getDBIDId === i.U
          e           := 0.U.asTypeOf(e)
          e.state     := Mux(hit, DBState.ALLOC, e.state)
          // assert(e.isFree | !hit)
          hwa_flag1 := e.isFree | !hit
          when(reqIsAtomic & hit) {
            // assert(i.U >= nrDBWithoutAPUs.U)
            hwa_flag2 := i.U >= nrDBWithoutAPUs.U
            apuEntrys(apuEID).op              := io.getDBID.bits.atomicOp
            apuEntrys(apuEID).atomic.swapFst  := io.getDBID.bits.swapFst
          }
        }
        // ALLOC
        is(DBState.ALLOC) {
          val hit     = io.dbRCReqOpt.get.fire & io.dbRCReqOpt.get.bits.dbID === i.U
          val read    = io.dbRCReqOpt.get.bits.isRead & hit
          val clean   = io.dbRCReqOpt.get.bits.isClean & hit
          e.state     := Mux(read, DBState.READ, Mux(clean, DBState.FREE, e.state))
          e.rBeatNum  := 0.U
        }
        // READ
        is(DBState.READ) {
          val hit     = rBeatFire & readId === i.U & !hasReading
          val clean   = ctrlEntrys(i).needClean
          e.state     := Mux(hit, Mux(e.isLast, Mux(clean, DBState.FREE, DBState.ALLOC), DBState.READING), e.state)
        }
        // READING
        is(DBState.READING) {
          val hit     = rBeatFire & readingId === i.U
          val clean   = ctrlEntrys(i).needClean
          e.state     := Mux(hit, Mux(clean, DBState.FREE, DBState.ALLOC), e.state)
        }
      }
      HardwareAssertion(hwa_flag1)
      HardwareAssertion(hwa_flag2)
  }


// ----------------------------------------------------- Assertion ---------------------------------------------------------- //
  awhen(io.dataTDB.valid){ HardwareAssertion(ctrlEntrys(io.dataTDB.bits.dbID).isAlloc) }

  val cntReg = RegInit(VecInit(Seq.fill(djparam.nrDatBuf) { 0.U(64.W) }))
  // cntReg.zip(ctrlEntrys).foreach { case (c, e) => c := Mux(e.isFree, 0.U, c + 1.U) }
  // cntReg.zipWithIndex.foreach { case (c, i) => assert(c < TIMEOUT_DB.U, "DataBuffer ENTRY[0x%x] STATE[0x%x] TIMEOUT", i.U, ctrlEntrys(i).state) }

  cntReg.zip(ctrlEntrys).zipWithIndex.foreach {case ((c, e),i) => HardwareAssertion.checkTimeout(e.isFree, TIMEOUT_DB, cf"DataBuffer ENTRY[0x${i.U}] STATE[0x${ctrlEntrys(i).state}] TIMEOUT", i.U, ctrlEntrys(i).state)}


// ----------------------------------------------------- Perf Counter ------------------------------------------------------- //
  require(djparam.nrDatBuf >= 4 & djparam.nrDatBuf % 4 == 0)
  for (i <- 0 until (djparam.nrDatBuf / 4)) {
    XSPerfAccumulate(s"pcu_dataBuf_group[${i}]_deal_req_cnt", io.getDBID.fire & (i * 4).U <= getDBIDId & getDBIDId <= (i * 4 + 3).U)
  }
  XSPerfAccumulate("pcu_dataBuf_deal_req_cnt", io.getDBID.fire)
  XSPerfAccumulate("pcu_dataBuf_req_retry_cnt", dbidRespQ.io.enq.fire & dbidRespQ.io.enq.bits.retry)

  HardwareAssertion.placePipe(3)
}
















