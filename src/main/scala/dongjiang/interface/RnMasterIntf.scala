package dongjiang.pcu.intf

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import chisel3._
import org.chipsalliance.cde.config._
import chisel3.util.{Cat, Decoupled, PopCount, RegEnable, Valid, ValidIO, log2Ceil}


class RnMasterIntf(rnMasId: Int, param: InterfaceParam)(implicit p: Parameters) extends IntfBaseIO(isSlv = false, hasFree = true, hasReq2Slice = true, hasDBRCReq = true) {
  // Del it
  io <> DontCare
  dontTouch(io)

}