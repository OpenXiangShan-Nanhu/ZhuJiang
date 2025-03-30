package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import dongjiang.bundle._
import dongjiang.frontend.decode._
import zhujiang.chi.DataFlit
import dongjiang.data.State._

/*
 * HasAlrDB
 */
trait HasAlrDB { this: DJBundle =>
  val alrDB   = new DJBundle {
    val reqs  = Bool() // Already request DataCM and DataBuf. If set it, commit will clean when done all
    val fast  = Bool() // Already Send Task[reqs+read+send+clean] to DataBlock
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
// 5. replace:                      read -> save (must without reqs,  with repl and clean)
// 6. send to CHI and save in SRAM: send -> save (must without reqs)
trait HasDataOp { this: DJBundle =>
  // flag
  val reqs  = Bool() // Request DataCM and DataBuf
  val repl  = Bool() // Replace
  val clean = Bool() // Release DataCM and DataBuf; Node: need set dataVec
  // operation (need resp to CommitCM)
  val read  = Bool() // sram -> buffer
  val send  = Bool() // buffer -> chi
  val save  = Bool() // buffer -> sram
}

class DataOp(implicit p: Parameters) extends DJBundle with HasDataOp

trait HasPackDataOp { this: DJBundle => val dataOp = new DataOp }

class PackDataOp(implicit p: Parameters) extends DJBundle with HasPackDataOp

/*
 * HasDsIdx
 */
trait HasDsIdx { this: DJBundle =>
  val ds = new DJBundle {
    val idx  = UInt(dsIdxBits.W)
    val bank = UInt(dsBankBits.W)
  }
}

/*
 * DataTask -> DataTaskBundle
 */
class DataTask(implicit p: Parameters) extends DJBundle with HasPackDataOp with HasPackLLCTxnID with HasDsIdx with HasDataVec {
  val txDat   = new DataFlit
}

/*
 * HasDCID -> DCID
 */
trait HasDCID { this: DJBundle =>
  val dcid = UInt(dcIdBits.W)
}

class DCID(implicit p: Parameters) extends DJBundle with HasDCID


/*
 * HasBeatNum
 */
trait HasBeatNum { this: DJBundle =>
  val beatNum = UInt(1.W)
}


/*
 * HasDBID -> DBID
 */
trait HasDBID { this: DJBundle =>
  val dbid = UInt(dbIdBits.W)
}

class DBID(implicit p: Parameters) extends DJBundle with HasDBID


/*
 * Data Ctrl Machine State
 */
object State {
  // READ -> WAIT -> SEND -> SAVE
  val width = 4
  val NOTASK= 0.U
  val READ0 = 1.U
  val READ1 = 2.U
  val WAIT0 = 3.U
  val WAIT1 = 4.U
  val SAVE0 = 5.U
  val SAVE1 = 6.U
  val SEND0 = 7.U
  val SEND1 = 8.U
  val RESP  = 9.U
}

class DCState(implicit p: Parameters) extends DJBundle {
  val valid   = Bool()
  val state   = UInt(State.width.W)

  // isFree
  def isFree  = !valid

  // isStateX
  def noTask  = state === NOTASK
  def isRead0 = state === READ0
  def isRead1 = state === READ1
  def isWait0 = state === WAIT0
  def isWait1 = state === WAIT1
  def isSave0 = state === SAVE0
  def isSave1 = state === SAVE1
  def isSend0 = state === SEND0
  def isSend1 = state === SEND1
  def isResp  = state === RESP

  // getDataId
  def getDataId: UInt = Mux(isWait0 | isSave0 | isSend0, "b00".U, "b10".U)

  // getNXS
  def getNXS(dataOp: DataOp, dataVec: Vec[Bool])  = {
    PriorityMux(Seq(
      // READ
      isRead0 -> Mux(dataVec(1), READ1, WAIT0),
      isRead1 -> Mux(dataVec(0), WAIT0, WAIT1),
      // WAIT0
      isWait0 -> PriorityMux(Seq(
        dataVec(1)  -> WAIT1,
        dataOp.save -> SAVE0,
        dataOp.send -> SAVE0,
        true.B      -> RESP)),
      // WAIT1
      isWait1 -> PriorityMux(Seq(
        dataOp.send -> Mux(dataVec(0), SEND0, SEND1),
        dataOp.save -> Mux(dataVec(0), SAVE0, SAVE1),
        true.B      -> RESP)),
      // SEND
      isSend0 -> Mux(dataVec(1), SEND1, Mux(dataOp.save, SAVE0, RESP)),
      isSave1 -> Mux(dataOp.save, Mux(dataVec(0), SAVE0, SAVE1), RESP),
      // SAVE
      isSave0 -> Mux(dataVec(1), SAVE1, RESP),
      isSave1 -> RESP,
      // RESP
      isResp  -> NOTASK,
    ))
  }
}