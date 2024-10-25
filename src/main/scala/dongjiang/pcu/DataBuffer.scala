package dongjiang.pcu

import dongjiang._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.Encoder.RREncoder

/*
 * ID Transfer:
 *
 * pcuIdx: PCU Index
 * { from(incoID) | to(incoID) | entryID | mshrIdx(mshrWay | mshrSet) | dbid | dcuIdx }
 *
 * { dbRCReq  } Read / Clean Req To DataBuffer      { to }                { dbid }
 * { getDBID  } Get DBID Req To DataBuffer          { from }  { entryID }
 * { DBIDResp } Resp With DBID From DataBuffer      { to }    { entryID } { dbid }
 * { dataTDB  } Send Data To DataBufer                                    { dbid }
 * { dataFDB  } Send Data From DataBuffer           { to }                { dbid }
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


class DataBuffer()(implicit p: Parameters) extends DJModule {
// --------------------- IO declaration ------------------------//
  val io = IO(Flipped(new DBBundle(hasDBRCReq = true)))

  // Del it
  dontTouch(io)


// --------------------- Reg and Wire declaration ------------------------//
  val entrys    = RegInit(VecInit(Seq.fill(djparam.nrDatBuf) { 0.U.asTypeOf(new DBEntry()) }))
  // wResp
  val wRespQ    = Module(new Queue(new DBWResp, 1, flow = false, pipe = true))




// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- WREQ & WRESP --------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val dbFreeVec             = entrys.map(_.isFree)
  val wReqId                = PriorityEncoder(dbFreeVec)
  // receive
  io.wReq.ready             := wRespQ.io.enq.ready & dbFreeVec.reduce(_ | _)
  wRespQ.io.enq.valid       := io.wReq.valid & dbFreeVec.reduce(_ | _)
  wRespQ.io.enq.bits.dbid   := wReqId
  wRespQ.io.enq.bits.to     := io.wReq.bits.from
  wRespQ.io.enq.bits.pcuId  := io.wReq.bits.pcuId
  // output
  io.wResp                  <> wRespQ.io.deq


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- DATA TO DB ----------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  when(io.dataTDB.valid){
    entrys(io.dataTDB.bits.dbid).beatVals(toBeatNum(io.dataTDB.bits.dataID))  := true.B
    entrys(io.dataTDB.bits.dbid).beats(toBeatNum(io.dataTDB.bits.dataID))     := io.dataTDB.bits.data
    assert(!entrys(io.dataTDB.bits.dbid).beatVals(toBeatNum(io.dataTDB.bits.dataID)))
  }
  io.dataTDB.ready := true.B


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------- RC REQ TO DB ---------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  when(io.dbRCReqOpt.get.fire) {
    entrys(io.dbRCReqOpt.get.bits.dbid).needClean := io.dbRCReqOpt.get.bits.isClean
    entrys(io.dbRCReqOpt.get.bits.dbid).to        := io.dbRCReqOpt.get.bits.to

  }
  io.dbRCReqOpt.get.ready := entrys(io.dbRCReqOpt.get.bits.dbid).canRecReq
  when(io.dbRCReqOpt.get.valid) {
    assert(!entrys(io.dbRCReqOpt.get.bits.dbid).isFree)
    assert(entrys(io.dbRCReqOpt.get.bits.dbid).beatVals.reduce(_ & _))
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
  io.dataFDB.bits.dbid        := selReadId
  io.dataFDB.bits.to          := entrys(selReadId).to
  entrys(selReadId).beatRNum  := entrys(selReadId).beatRNum + io.dataFDB.fire.asUInt

  when(io.dataFDB.valid) {
    assert(entrys(io.dataFDB.bits.dbid).beatVals(toBeatNum(io.dataTDB.bits.dataID)))
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- State Transfer ------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.zipWithIndex.foreach {
    case (e, i) =>
      switch(e.state) {
        // FREE
        is(DBState.FREE) {
          val hit     = wRespQ.io.enq.fire & wReqId === i.U
          e           := 0.U.asTypeOf(e)
          e.state     := Mux(hit, DBState.ALLOC, e.state)
        }
        // ALLOC
        is(DBState.ALLOC) {
          val hit     = io.dbRCReqOpt.get.fire & io.dbRCReqOpt.get.bits.dbid === i.U
          val read    = io.dbRCReqOpt.get.bits.isRead & hit
          val clean   = io.dbRCReqOpt.get.bits.isClean & hit
          e.state     := Mux(read, DBState.READ, Mux(clean, DBState.FREE, e.state))
        }
        // READ
        is(DBState.READ) {
          val hit     = io.dataFDB.fire & !hasReading & io.dataFDB.bits.dbid === i.U
          if(nrBeat > 1) {
            e.state   := Mux(hit, DBState.READING, e.state)
          } else {
            e.state   := Mux(hit, DBState.READ_DONE, e.state)
          }
        }
        // READING
        is(DBState.READING) {
          val hit     = io.dataFDB.fire & io.dataFDB.bits.dbid === i.U
          val isLast  = entrys(i).beatRNum === (nrBeat - 1).U
          val clean   = entrys(i).needClean
          e.state     := Mux(hit & isLast, Mux(clean, DBState.FREE, DBState.READ_DONE), e.state)
        }
        // READ_DONE
        is(DBState.READ_DONE) {
          val hit     = io.dbRCReqOpt.get.fire & io.dbRCReqOpt.get.bits.dbid === i.U
          val read    = io.dbRCReqOpt.get.bits.isRead & hit
          val clean   = io.dbRCReqOpt.get.bits.isClean & hit
          e.state     := Mux(read, DBState.READ, Mux(clean, DBState.FREE, e.state))
        }
      }
  }


// ----------------------------------------------------- Assertion ---------------------------------------------------------- //
  when(io.dataTDB.valid){ assert(entrys(io.dataTDB.bits.dbid).isAlloc) }

  val cntReg = RegInit(VecInit(Seq.fill(djparam.nrDatBuf) { 0.U(64.W) }))
  cntReg.zip(entrys).foreach { case (c, e) => c := Mux(e.isFree, 0.U, c + 1.U) }
  cntReg.zipWithIndex.foreach { case (c, i) => assert(c < TIMEOUT_DB.U, "DataBuffer ENTRY[0x%x] STATE[0x%x] TIMEOUT", i.U, entrys(i).state) }

}
















