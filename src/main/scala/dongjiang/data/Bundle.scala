package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import dongjiang.bundle._
import dongjiang.frontend.decode._
import zhujiang.chi.DataFlit

/*
 * HasAlrDB
 */
trait HasAlrDB { this: DJBundle =>
  val alrDB   = new DJBundle {
    val reqs  = Bool()
    val fast  = Bool()
  }
}

/*
 * HasDataOp -> DataOp -> HasPackDataOp -> PackDataOp
 */
trait HasDataOp { this: DJBundle =>
  val reqs = Bool() // request buffer
  val read = Bool() // sram -> buffer
  val save = Bool() // buffer -> sram
  val send = Bool() // buffer -> chi
  val repl = Bool() // read -> buffer0; buffer0 -> chi / buffer1 -> sram
}

class DataOp(implicit p: Parameters) extends DJBundle with HasDataOp

trait HasPackDataOp { this: DJBundle => val dataOp = new DataOp }

class PackDataOp(implicit p: Parameters) extends DJBundle with HasPackDataOp

/*
 * HasLLCIndex
 */
trait HasLLCIndex { this: DJBundle =>
  val llc   = new DJBundle() {
    val set = UInt(llcSetBits.W)
    val way = UInt(llcWayBits.W)
  }
}

/*
 * DataTask -> DataTaskBundle
 */
class DataTask(implicit p: Parameters) extends DJBundle with HasPackDataOp with HasPackLLCTxnID with HasChiSize with HasLLCIndex {
  val txDat = new DataFlit
}