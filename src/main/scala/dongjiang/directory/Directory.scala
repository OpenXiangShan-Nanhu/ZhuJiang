package dongjiang.directory

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import xs.utils.debug.{DomainInfo, HardwareAssertion}

class Directory(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Read from frontends
    val fReadDirVec = Vec(djparam.nrDirBank, Flipped(Decoupled(new DJBundle with HasAddr with HasDCID {
      val early     = Bool() // Early access to data
    })))
    // Resp to frontends
    val fRespDirVec = Output(Vec(djparam.nrDirBank, Bool())) // TODO
  })
  io <> DontCare
  io.fReadDirVec.foreach(_.ready := true.B)




}