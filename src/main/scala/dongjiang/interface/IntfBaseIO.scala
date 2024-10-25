package dongjiang.pcu.intf

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils._
import dongjiang.utils.FastArb._

abstract class IntfBaseIO(isSlv:Boolean, hasFree: Boolean = false, hasReq2Slice: Boolean = false, hasDBRCReq: Boolean = false)(implicit p: Parameters) extends DJModule {
// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val hnfID           = Input(UInt(chiNodeIdBits.W))
    val freeOpt         = if(hasFree) Some(Output(Bool())) else None
    // CHI
    val chiOpt          = if(isSlv) Some(Flipped(new CHIBundleDecoupled)) else Some(new CHIBundleDecoupled)
    // slice ctrl signals
    val req2SliceOpt    = if(hasReq2Slice) Some(Decoupled(new Req2SliceBundle())) else None
    val reqAck2NodeOpt  = if(hasReq2Slice) Some(Flipped(Decoupled(new ReqAck2NodeBundle()))) else None
    val resp2NodeOpt    = if(hasReq2Slice) Some(Flipped(Decoupled(new Resp2NodeBundle()))) else None
    val req2Node        = Flipped(Decoupled(new Req2NodeBundle()))
    val resp2Slice      = Decoupled(new Resp2SliceBundle())
    // slice DataBuffer signals
    val dbSigs          = new DBBundle(hasDBRCReq)

    def free            = freeOpt.get
    def chi             = chiOpt.get
    def req2Slice       = req2SliceOpt.get
    def reqAck2Node     = reqAck2NodeOpt.get
    def resp2Node       = resp2NodeOpt.get
  })
}