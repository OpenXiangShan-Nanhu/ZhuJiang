package dongjiang.pcu

import dongjiang._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.Encoder.RREncoder
import xs.utils.perf.HasPerfLogging

/*
 * ID Transfer:
 *
 *
 * { dbRCReq  } Read / Clean Req To DataBuffer      { to }                { dbID }
 * { getDBID  } Get DBID Req To DataBuffer          { from }  { entryID }
 * { dbidResp } Resp With DBID From DataBuffer      { to }    { entryID } { dbID }
 * { dataTDB  } Send Data To DataBufer                                    { dbID }
 * { dataFDB  } Send Data From DataBuffer           { to }                { dbID }
 *
 */

object DBState {
  val width       = 3
  // FREE -> ALLOC -> FREE
  // FREE -> ALLOC -> READING(needClean) -> READ(needClean) -> FREE
  // FREE -> ALLOC -> READING(!needClean) -> READ(!needClean) -> READ_DONE -> READING(needClean) -> READ(needClean) -> FREE
  val FREE        = "b000".U
  val ALLOC       = "b001".U
  val READ        = "b010".U // Ready to read
  val READING     = "b011".U // Already partially read
  val READ_DONE   = "b100".U // Has been read all beat
}


class DBEntry(implicit p: Parameters) extends DJBundle with HasToIncoID {
  val state       = UInt(DBState.width.W)
  val beatRNum    = UInt(log2Ceil(nrBeat).W)
  val needClean   = Bool()
  val beats       = Vec(nrBeat, UInt(beatBits.W)) // TODO: Reg -> SRAM
  val beatVals    = Vec(nrBeat, Bool())

  def getBeat     = beats(beatRNum)
  def isFree      = state === DBState.FREE
  def isAlloc     = state === DBState.ALLOC
  def isRead      = state === DBState.READ
  def isReading   = state === DBState.READING
  def isReadDone  = state === DBState.READ_DONE
  def canRecReq   = isAlloc | isReadDone
}


class DataBuffer()(implicit p: Parameters) extends DJModule with HasPerfLogging {
// --------------------- IO declaration ------------------------//
  val io = IO(Flipped(new DBBundle(hasDBRCReq = true)))

  // Del it
  dontTouch(io)


// --------------------- Reg and Wire declaration ------------------------//
  val entrys    = RegInit(VecInit(Seq.fill(djparam.nrDatBuf) { 0.U.asTypeOf(new DBEntry()) }))
  // dbidResp
  val dbidRespQ = Module(new Queue(new DBIDResp(), 1, flow = false, pipe = true))




// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- WREQ & WRESP --------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val dbFreeVec                 = entrys.map(_.isFree)
  val getDBIDId                 = PriorityEncoder(dbFreeVec)
  // receive
  io.getDBID.ready              := dbidRespQ.io.enq.ready & dbFreeVec.reduce(_ | _)
  dbidRespQ.io.enq.valid        := io.getDBID.valid & dbFreeVec.reduce(_ | _)
  dbidRespQ.io.enq.bits.dbID    := getDBIDId
  dbidRespQ.io.enq.bits.to      := io.getDBID.bits.from
  dbidRespQ.io.enq.bits.entryID := io.getDBID.bits.entryID
  // output
  io.dbidResp                   <> dbidRespQ.io.deq


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- DATA TO DB ----------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  when(io.dataTDB.valid){
    entrys(io.dataTDB.bits.dbID).beatVals(toBeatNum(io.dataTDB.bits.dataID))  := true.B
    entrys(io.dataTDB.bits.dbID).beats(toBeatNum(io.dataTDB.bits.dataID))     := io.dataTDB.bits.data
    assert(!entrys(io.dataTDB.bits.dbID).beatVals(toBeatNum(io.dataTDB.bits.dataID)))
  }
  io.dataTDB.ready := true.B


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------- RC REQ TO DB ---------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  when(io.dbRCReqOpt.get.fire) {
    entrys(io.dbRCReqOpt.get.bits.dbID).needClean := io.dbRCReqOpt.get.bits.isClean
    entrys(io.dbRCReqOpt.get.bits.dbID).to        := io.dbRCReqOpt.get.bits.to

  }
  io.dbRCReqOpt.get.ready := entrys(io.dbRCReqOpt.get.bits.dbID).canRecReq
  when(io.dbRCReqOpt.get.valid) {
    assert(!entrys(io.dbRCReqOpt.get.bits.dbID).isFree)
    assert(entrys(io.dbRCReqOpt.get.bits.dbID).beatVals.reduce(_ & _))
  }

// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------- DATA TO NODE ---------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  // TODO: Ensure that the order of dataFDB Out is equal to the order of dbRCReq In
  val readVec     = entrys.map(_.isRead)
  val readingVec  = entrys.map(_.isReading)
  val hasReading  = readingVec.reduce(_ | _)
  val readId      = RREncoder(readVec)
  val readingId   = PriorityEncoder(readingVec)
  val selReadId   = Mux(hasReading, readingId, readId)
  assert(PopCount(readingVec) <= 1.U)

  io.dataFDB.valid            := readVec.reduce(_ | _) | readingVec.reduce(_ | _)
  io.dataFDB.bits.data        := entrys(selReadId).getBeat
  io.dataFDB.bits.dataID      := toDataID(entrys(selReadId).beatRNum)
  io.dataFDB.bits.dbID        := selReadId
  io.dataFDB.bits.to          := entrys(selReadId).to
  entrys(selReadId).beatRNum  := entrys(selReadId).beatRNum + io.dataFDB.fire.asUInt

  when(io.dataFDB.valid) {
    assert(entrys(io.dataFDB.bits.dbID).beatVals(toBeatNum(io.dataTDB.bits.dataID)))
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- State Transfer ------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.zipWithIndex.foreach {
    case (e, i) =>
      switch(e.state) {
        // FREE
        is(DBState.FREE) {
          val hit     = dbidRespQ.io.enq.fire & getDBIDId === i.U
          e           := 0.U.asTypeOf(e)
          e.state     := Mux(hit, DBState.ALLOC, e.state)
        }
        // ALLOC
        is(DBState.ALLOC) {
          val hit     = io.dbRCReqOpt.get.fire & io.dbRCReqOpt.get.bits.dbID === i.U
          val read    = io.dbRCReqOpt.get.bits.isRead & hit
          val clean   = io.dbRCReqOpt.get.bits.isClean & hit
          e.state     := Mux(read, DBState.READ, Mux(clean, DBState.FREE, e.state))
        }
        // READ
        is(DBState.READ) {
          val hit     = io.dataFDB.fire & !hasReading & io.dataFDB.bits.dbID === i.U
          if(nrBeat > 1) {
            e.state   := Mux(hit, DBState.READING, e.state)
          } else {
            e.state   := Mux(hit, DBState.READ_DONE, e.state)
          }
        }
        // READING
        is(DBState.READING) {
          val hit     = io.dataFDB.fire & io.dataFDB.bits.dbID === i.U
          val isLast  = entrys(i).beatRNum === (nrBeat - 1).U
          val clean   = entrys(i).needClean
          e.state     := Mux(hit & isLast, Mux(clean, DBState.FREE, DBState.READ_DONE), e.state)
        }
        // READ_DONE
        is(DBState.READ_DONE) {
          val hit     = io.dbRCReqOpt.get.fire & io.dbRCReqOpt.get.bits.dbID === i.U
          val read    = io.dbRCReqOpt.get.bits.isRead & hit
          val clean   = io.dbRCReqOpt.get.bits.isClean & hit
          e.state     := Mux(read, DBState.READ, Mux(clean, DBState.FREE, e.state))
        }
      }
  }


// ----------------------------------------------------- Assertion ---------------------------------------------------------- //
  when(io.dataTDB.valid){ assert(entrys(io.dataTDB.bits.dbID).isAlloc) }

  val cntReg = RegInit(VecInit(Seq.fill(djparam.nrDatBuf) { 0.U(64.W) }))
  cntReg.zip(entrys).foreach { case (c, e) => c := Mux(e.isFree, 0.U, c + 1.U) }
  cntReg.zipWithIndex.foreach { case (c, i) => assert(c < TIMEOUT_DB.U, "DataBuffer ENTRY[0x%x] STATE[0x%x] TIMEOUT", i.U, entrys(i).state) }


// ----------------------------------------------------- Perf Counter ------------------------------------------------------- //
  require(djparam.nrDatBuf >= 4 & djparam.nrDatBuf % 4 == 0)
  for (i <- 0 until (djparam.nrDatBuf / 4)) {
    XSPerfAccumulate(s"pcu_dataBuf_group[${i}]_deal_req_cnt", io.getDBID.fire & (i * 4).U <= getDBIDId & getDBIDId <= (i * 4 + 3).U)
  }
  XSPerfAccumulate("pcu_dataBuf_deal_req_cnt", io.getDBID.fire)
  XSPerfAccumulate("pcu_dataBuf_req_block_cnt", io.getDBID.valid & PopCount(dbFreeVec) === 0.U)

}
















