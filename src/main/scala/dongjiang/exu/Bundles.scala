package dongjiang.pcu.exu

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import scala.collection.immutable.ListMap
import scala.math.{max, min}


// ---------------------------------------------------------------- EXU Base Bundle ----------------------------------------------------------------------------- //
class ExuChiMesBundle(implicit p: Parameters) extends DJBundle with HasCHIChannel {
    // REQ Mes(Use In Req)
    val expCompAck      = Bool()
    // Common
    val opcode          = UInt(7.W)
}

class ExuRespMesBundle(implicit p: Parameters) extends DJBundle {
    val slvResp         = Valid(UInt(ChiResp.width.W))
    val mstResp         = Valid(UInt(ChiResp.width.W))
    val fwdState        = Valid(UInt(ChiResp.width.W))
    // TODO
    val slvDBID         = Valid(UInt(dbIdBits.W))
    val mstDBID         = Valid(UInt(dbIdBits.W))

    def noRespValid     = !slvResp.valid & !mstResp.valid
}

class PipeTaskBundle(implicit p: Parameters) extends DJBundle  {
    val chiMes          = new ExuChiMesBundle()
    val chiIndex        = new ChiIndexBundle()
    val respMes         = new ExuRespMesBundle()
    // other
    val taskMes         = new DJBundle with HasUseAddr with HasPipeID with HasMSHRWay {
        val readDir     = Bool()
    }

    def fullAddr(d: UInt, p:UInt) = getFullAddr(taskMes.useAddr, d, p, chiIndex.offset)
    def mshrMatch(set: Int, way: Int): Bool = taskMes.mSet === set.U & taskMes.mshrWay === way.U
}

object UpdMSHRType { val width = 2; val RETRY = "b0".U ; val UPD = "b1".U }

class UpdateMSHRReqBundle(implicit p: Parameters) extends DJBundle with HasMHSRIndex with HasCHIChannel {
    val updType     = UInt(UpdMSHRType.width.W)
    val waitIntfVec = Vec(nrIntf, Bool())
    val hasNewReq   = Bool()
    val mTag        = UInt(mshrTagBits.W)
    val opcode      = UInt(7.W)
    val unlock      = Bool()

    def isRetry     = updType === UpdMSHRType.RETRY
    def isUpdate    = updType === UpdMSHRType.UPD & waitIntfVec.reduce(_ | _)
    def isClean     = updType === UpdMSHRType.UPD & !waitIntfVec.reduce(_ | _)
}

class DirReadBundle(implicit p: Parameters) extends DJBundle with HasUseAddr with HasMSHRWay with HasPipeID

class DirRespBaseBundle(nrWays: Int, nrMetas: Int, replWayBits: Int)(implicit p: Parameters) extends DJBundle with HasUseAddr with HasPipeID {
    val hit         = Bool()
    val wayOH       = UInt(nrWays.W)
    val metaVec     = Vec(nrMetas, new CHIStateBundle())
    val replMes     = UInt(replWayBits.W)
    val replRetry   = Bool()
}

class DirRespBundle(implicit p: Parameters) extends DJBundle with HasPipeID {
    val s           = new DirRespBaseBundle(djparam.selfWays, 1, sReplWayBits) // self
    val sf          = new DirRespBaseBundle(djparam.sfDirWays, nrCcNode, sfReplWayBits) // snoop filter
}

class DirWriteBaseBundle(nrWays: Int, nrMetas: Int, replWayBits: Int)(implicit p: Parameters) extends DJBundle with HasUseAddr {
    val wayOH       = UInt(nrWays.W)
    val metaVec     = Vec(nrMetas, new CHIStateBundle())
    val replMes     = UInt(replWayBits.W)
}

class DirWriteBundle(implicit p: Parameters) extends DJBundle {
    val s           = Decoupled(new DirWriteBaseBundle(djparam.selfWays, 1, sReplWayBits)) // self
    val sf          = Decoupled(new DirWriteBaseBundle(djparam.sfDirWays, nrCcNode, sfReplWayBits)) // snoop filter
}

trait HasDirBank extends DJBundle { val dirBank = UInt(dirBankBits.W) }

class DirReadMSHRBundle(implicit p: Parameters) extends DJBundle with HasPipeID with HasDirBank with HasMSHRSet

class MSHRRespDirBundle(implicit p: Parameters) extends DJBundle with HasPipeID with HasDirBank {
    val addrs       = Vec(djparam.nrMSHRWays, Valid(UInt(useAddrBits.W)))
}




