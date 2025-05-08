package dongjiang.data

import chisel3._
import chisel3.experimental.SourceInfo
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import dongjiang.bundle._
import dongjiang.frontend.decode._
import zhujiang.chi.DataFlit
import dongjiang.data.CTRLSTATE._
import xs.utils.debug._

/*
 * HasDataOp -> DataOp -> HasPackDataOp -> PackDataOp
 */
// Optional Combination
trait HasDataOp { this: Bundle =>
  // flag
  // if set repl, dont care other flag or operation
  val reqs      = Bool() // Request DataCM and DataBuffer
  val repl      = Bool() // Replace, exchanging data between ata storage and data buffer
  // operation (need resp to Backend when all done)
  // If both read and save are true, they will be executed concurrently to complete the cache line replacement operation.
  val read      = Bool() // data storage  -> data buffer
  val send      = Bool() // data buffer   -> chi tx data
  val save      = Bool() // data buffer   -> data storage
  val clean     = Bool() // Release DataBuf

  def data0     =   reqs | read | send
  def data1     =   save | clean
  def valid     =   reqs | read | send | save  | clean
  def onlyClean = !(reqs | read | send | save) & clean

  // Use in DataCM
  def getNextState(state: UInt)(implicit p: Parameters, s: SourceInfo) = {
    HAssert(state === ALLOC | state === REPL | state === READ | state === WAIT | state === SEND | state === SAVE)
    HAssert(valid | repl)
    // Without replace
    val next0 = WireInit(0.U(CTRLSTATE.width.W))
    switch(state) {
      is(ALLOC) {
        next0 := PriorityMux(Seq(
          read    -> READ,
          send    -> SEND,
          save    -> SAVE,
          clean   -> CLEAN,
          reqs    -> RESP
        ))
      }
      is(READ)  { next0 := WAIT }
      is(WAIT)  {
        next0 := PriorityMux(Seq(
          send    -> SEND,
          save    -> SAVE,
          clean   -> CLEAN,
          true.B  -> RESP
        ))
      }
      is(SEND)  {
        next0 := PriorityMux(Seq(
          save    -> SAVE,
          clean   -> CLEAN,
          true.B  -> RESP
        ))
      }
      is(SAVE)  { next0 := Mux(clean, CLEAN, RESP) }
      is(CLEAN) { next0 := RESP }
      is(RESP)  { next0 := Mux(clean, TOFREE, ALLOC) }
    }
    // With replace
    val next1 = WireInit(0.U(CTRLSTATE.width.W))
    switch(state) {
      is(ALLOC) { next1 := REPL   }
      is(REPL)  { next1 := WAIT   }
      is(SEND)  { next1 := CLEAN  }
      is(CLEAN) { next1 := RESP   }
      is(RESP)  { next1 := TOFREE }
    }
    Mux(repl, next1, next0)
  }

}

class DataOp extends Bundle with HasDataOp

trait HasPackDataOp { this: Bundle => val dataOp = new DataOp }

class PackDataOp(implicit p: Parameters) extends DJBundle with HasPackDataOp

/*
 * HasDsIdx
 */
class DsIdx(implicit p: Parameters) extends DJBundle {
  val bank = UInt(dsBankBits.W)
  val idx  = UInt(dsIdxBits.W)

  def set(a: UInt, way: UInt): Unit = {
    require(a.getWidth   == addrBits)
    require(way.getWidth == llcWayBits)
    val temp  = Cat(getLlcSet(a), way, getDirBank(a))
    this.bank := temp(dsBank_ds_hi, dsBank_ds_lo)
    this.idx  := temp(dsIdx_ds_hi, dsIdx_ds_lo)
  }
}

trait HasDsIdx { this: DJBundle =>
  val ds = new DsIdx()
}

class PackDsIdx(implicit p: Parameters) extends DJBundle with HasDsIdx

/*
 * DataTask -> DataTaskBundle
 */
class DataTask(implicit p: Parameters) extends DJBundle with HasHnTxnID
  with HasPackDataOp with HasDsIdx with HasDataVec {
  val txDat = new DataFlit
}

class PackDataTask(implicit p: Parameters) extends DJBundle { val task = new DataTask }

/*
 * HasOpBeat
 */
trait HasBeatNum { this: DJBundle =>
  val beatNum = UInt(log2Ceil(djparam.nrBeat).W)
}

/*
 * HasDBID -> DBID
 */
trait HasDBID { this: DJBundle =>
  val dbid = UInt(dbIdBits.W)
}

class DBID(implicit p: Parameters) extends DJBundle with HasDBID

/*
 * HasDCID -> DCID
 */
trait HasDCID { this: DJBundle =>
  val dcid = UInt(dcIdBits.W)
}

class DCID(implicit p: Parameters) extends DJBundle with HasDCID

/*
 * HasCritical -> Critical
 */
trait HasCritical { this: DJBundle =>
  val critical = Bool()
}

class Critical(implicit p: Parameters) extends DJBundle with HasDCID

/*
 * ReadDB / ReadDS
 */
class ReadDB(implicit p: Parameters) extends DJBundle with HasDsIdx with HasDCID with HasDBID with HasBeatNum with HasCritical { val repl = Bool() }
class ReadDS(implicit p: Parameters) extends DJBundle with HasDsIdx with HasDCID with HasDBID with HasBeatNum with HasCritical

/*
 * GetDBID
 */
class GetDBID(implicit p: Parameters) extends DJBundle {
  val hnTxnID   = Output(UInt(hnTxnIDBits.W))
  val dbidVec   = Input(Vec(djparam.nrBeat, UInt(dbIdBits.W)))
}

/*
 * PackDataFilt
 */
class PackDataFilt(implicit p: Parameters) extends DJBundle {
  val dat = new DataFlit
}