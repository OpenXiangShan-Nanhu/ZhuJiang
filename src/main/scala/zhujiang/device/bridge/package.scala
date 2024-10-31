package zhujiang.device

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.{IcnRxBundle, IcnTxBundle}
import zhujiang.chi._
import zhujiang.{HasZJParams, ZJBundle, ZJModule}

package object bridge {
  class ChiUpstreamOpVec(implicit p: Parameters) extends ZJBundle {
    val receiptResp = Bool()
    val dbidResp = Bool()
    val wdata = Bool()
    val rdata = Bool()
    val compAck = Bool()
    val comp = Bool()
    val compCmo = Bool()
    def readReq(order: UInt, expCompAck: Bool): Unit = {
      receiptResp := order === 0.U
      dbidResp := true.B
      wdata := true.B
      rdata := false.B
      compAck := !expCompAck
      comp := true.B
      compCmo := true.B
    }
    def writeReq(expCompAck: Bool, cmo:Bool): Unit = {
      receiptResp := true.B
      dbidResp := false.B
      wdata := false.B
      rdata := true.B
      compAck := !expCompAck
      comp := false.B
      compCmo := !cmo
    }
    def completed: Bool = this.asUInt.andR
    def decode(req: ReqFlit, check: Bool): Unit = {
      when(check) {
        assert(req.Opcode === ReqOpcode.ReadNoSnp || req.Opcode === ReqOpcode.WriteNoSnpPtl || req.Opcode === ReqOpcode.WriteNoSnpFullCleanInv)
        assert(req.Size <= 6.U)
      }
      when(req.Opcode === ReqOpcode.ReadNoSnp) {
        readReq(req.Order, req.ExpCompAck)
      }.otherwise {
        writeReq(req.ExpCompAck, req.Opcode === ReqOpcode.WriteNoSnpFullCleanInv)
      }
    }
  }

  abstract class DownstreamOpVec(implicit p: Parameters) extends ZJBundle {
    def completed:Bool
    def decode(req:ReqFlit, check:Bool):Unit
  }

  abstract class IcnIoDevCtrlOpVecCommon(implicit p: Parameters) extends ZJBundle {
    val u = new ChiUpstreamOpVec
    def d: DownstreamOpVec
    def icnReadReceipt:Bool
    def icnDBID:Bool
    def icnComp:Bool
    def icnCompCmo:Bool = d.completed && !u.compCmo

    def needIssue:Bool
  }

  class IcnIoDevCtrlInfoCommon(ioDataBits:Int, val withData:Boolean, val mem:Boolean)(implicit p: Parameters) extends ZJBundle {
    val data = if(withData) Some(UInt(ioDataBits.W)) else None
    val mask = if(withData) Some(UInt((ioDataBits / 8).W)) else None
    val size = UInt(3.W)
    val addr = UInt(raw.W)
    val txnId = UInt(12.W)
    val srcId = UInt(niw.W)
    val returnNid = if(mem) Some(UInt(niw.W)) else None
    val returnTxnId = if(mem) Some(UInt(12.W)) else None
    val dwt = if(mem) Some(Bool()) else None
    val readCnt = UInt(8.W)
  }

  abstract class IcnIoDevRsEntryCommon[
    T <: IcnIoDevCtrlOpVecCommon,
    K <: IcnIoDevCtrlInfoCommon
  ](implicit p: Parameters) extends ZJBundle {
    def state: T
    def info: K
    def enq(req:ReqFlit, valid:Bool):Unit
  }
}
