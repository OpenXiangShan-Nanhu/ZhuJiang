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
  // operation (need resp to Backend when all done)
  val repl      = Bool() // Replace, exchanging data between data storage and data buffer
  val read      = Bool() // data storage  -> data buffer
  val send      = Bool() // data buffer   -> chi tx data
  val save      = Bool() // data buffer   -> data storage
  def onlySave  = !repl & !read & !send & save
  def isValid   =  repl |  read |  send | save
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
  val dbidVec   = Vec(djparam.nrBeat, Flipped(Valid(UInt(dbIdBits.W))))
}

/*
 * PackDataFilt
 */
class PackDataFilt(implicit p: Parameters) extends DJBundle {
  val dat = new DataFlit
}