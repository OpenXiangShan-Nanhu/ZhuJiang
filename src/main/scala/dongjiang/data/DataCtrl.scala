package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{DomainInfo, HardwareAssertion}

class DataCtrl(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // CHI TX/RX DAT
    val txDatVec    = Vec(nrIcn, Decoupled(new DataFlit()))
    val rxDat       = Flipped(Decoupled(new DataFlit())) // Only use rxDat.Data/DataID/BE in DataCtrl
    // DB Req from frontends
    val reqDBVec    = Vec(djparam.nrDirBank, Flipped(Decoupled(new DJBundle with HasLLCTxnID {
      val double    = Bool()
    })))
    // DB Resp to frontends
    val respDBVec   = Output(Vec(djparam.nrDirBank, new DCID()))
    // Read LLC Hit Message
    val hitMesVec   = Vec(djparam.nrDirBank, Flipped(Valid(new DJBundle with HasDCID {
      val set       = UInt(llcSetBits.W)
      val way       = UInt(llcWayBits.W)
    })))
  })

  io <> DontCare
  io.reqDBVec.foreach(_.ready := true.B)


}