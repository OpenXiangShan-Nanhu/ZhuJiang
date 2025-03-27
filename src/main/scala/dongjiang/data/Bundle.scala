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
// Optional Combination
// 1. read SRAM:                    read
// 2. save in SRAM:                 save (must without reqs)
// 3. send to CHI:                  send (must without reqs)
// 4. read SRAM and send to CHI:    read -> send
// 5. replace:                      read -> save -> send (must without reqs,  with repl and clean)
// 6. send to CHI and save in SRAM: send -> save (must without reqs)
trait HasDataOp { this: DJBundle =>
  // flag
  val reqs  = Bool() // Request DataBuffer
  val repl  = Bool() // Replace
  val clean = Bool() // Release DataBuffer
  // operation (need resp to CommitCM)
  val read  = Bool() // sram -> buffer
  val save  = Bool() // buffer -> sram
  val send  = Bool() // buffer -> chi
}

class DataOp(implicit p: Parameters) extends DJBundle with HasDataOp

trait HasPackDataOp { this: DJBundle => val dataOp = new DataOp }

class PackDataOp(implicit p: Parameters) extends DJBundle with HasPackDataOp

/*
 * HasDsIdx
 */
trait HasDsIdx { this: DJBundle =>
  val ds = new DJBundle {
    val bank = UInt(dsBankBits.W)
    val idx  = UInt(dsIdxBits.W)
  }
}

/*
 * DataTask -> DataTaskBundle
 */
class DataTask(implicit p: Parameters) extends DJBundle with HasPackDataOp with HasPackLLCTxnID with HasDsIdx {
  val txDat   = new DataFlit
  val useVec  = Vec(2, Bool())
}

/*
 * HasDCID -> DCID
 */
trait HasDCID { this: DJBundle =>
  val dcid = UInt(dcIdBits.W)
}

class DCID(implicit p: Parameters) extends DJBundle with HasDCID


/*
 * HasDBID -> DBID
 */
trait HasDBID { this: DJBundle =>
  val dbid = UInt(dbIdBits.W)
}

class DBID(implicit p: Parameters) extends DJBundle with HasDBID