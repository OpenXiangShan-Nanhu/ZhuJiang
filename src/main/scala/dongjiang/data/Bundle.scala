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
import dongjiang.utils.HasQoS
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
  val merge     = Bool() // cant use fast data resp channel(DS to CHI)
  def readToDB  =  repl |  read
  def readToDS  =  repl |  save
  def readToCHI =  repl |  send
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
  with HasPackDataOp with HasDsIdx with HasDataVec with HasQoS {
  val txDat = new DataFlit
}

class PackDataTask(implicit p: Parameters) extends DJBundle { val task = new DataTask }

/*
 * HasOpBeat
 */
trait HasBeatNum { this: DJBundle =>
  val beatNum   = UInt(log2Ceil(djparam.nrBeat).W)
  def getDataID = Cat(beatNum, 0.U(1.W))
  require(djparam.nrBeat == 2)
}

/*
 * HasDBID -> DBID
 */
trait HasDBID { this: DJBundle =>
  val dbid = UInt(dbIdBits.W)
}

class DBID(implicit p: Parameters) extends DJBundle with HasDBID

trait HasDBIDVec { this: DJBundle =>
  val dbidVec = Vec(djparam.nrBeat, UInt(dbIdBits.W))
}

class DBIDVec(implicit p: Parameters) extends DJBundle with HasDBIDVec

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
 * ReadDB / ReadDS / WriteDS / DsResp
 */
class ReadDB (implicit p: Parameters) extends DJBundle with HasDsIdx with HasDCID with HasDBID with HasBeatNum with HasQoS with HasCritical { val repl  = Bool() }
class ReadDS (implicit p: Parameters) extends DJBundle with HasDsIdx with HasDCID with HasDBID with HasBeatNum with HasQoS with HasCritical { val toCHI = Bool() }
class WriteDS(implicit p: Parameters) extends DJBundle with HasDsIdx with HasDCID with HasBeatNum { val beat = UInt(BeatBits.W) }
class DsResp (implicit p: Parameters) extends DJBundle with HasDBID  with HasDCID with HasBeatNum { val beat = UInt(BeatBits.W); val toCHI = Bool() }

/*
 * PackDataFilt
 */
class PackDataFilt(implicit p: Parameters) extends DJBundle {
  val dat = new DataFlit
}