package dongjiang.data

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{DomainInfo, HardwareAssertion}

class DataCtrl(nrIcn: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // chi tx/rx Dat
    val txDatVec = Vec(nrIcn, Decoupled(new DataFlit()))
    val rxDat    = Flipped(Decoupled(new DataFlit())) // Only use rxDat.Data/DataID/BE in Backend
  })
  io <> DontCare


}