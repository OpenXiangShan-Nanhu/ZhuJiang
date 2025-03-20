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
    val txDat       = Decoupled(new DataFlit())
    val rxDat       = Flipped(Decoupled(new DataFlit())) // Only use rxDat.Data/DataID/BE in DataCtrl
    // Read LLC Hit Message
    val hitMesVec   = Vec(djparam.nrDirBank, Flipped(Decoupled(new DJBundle with HasPosIndex {
      val set       = UInt(llcSetBits.W)
      val way       = UInt(llcWayBits.W)
    })))
  })

  io <> DontCare

}