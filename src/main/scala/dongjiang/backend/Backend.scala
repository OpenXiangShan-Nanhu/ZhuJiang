package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xijiang.Node
import xs.utils.debug.{DomainInfo, HardwareAssertion}

class Backend(nrIcn: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // chi tx
    val txReqVec = Vec(nrIcn, Decoupled(new ReqFlit(true)))
    val txSnpVec = Vec(nrIcn, Decoupled(new SnoopFlit()))
    val txRspVec = Vec(nrIcn, Decoupled(new RespFlit()))
    // chi rx
    val rxRspVec = Vec(nrIcn, Flipped(Decoupled(new RespFlit())))
    val rxDat    = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
  })
  io <> DontCare


}