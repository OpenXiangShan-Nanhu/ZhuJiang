package zhujiang.device.bridge

import chisel3._
import org.chipsalliance.cde.config.Parameters
import zhujiang.{ZJBundle, device}
import zhujiang.chi.{ReqFlit, ReqOpcode}
import zhujiang.device.bridge.{DownstreamOpVec, IcnIoDevCtrlInfoCommon, IcnIoDevCtrlOpVecCommon, IcnIoDevRsEntryCommon}

package object tlul {
  class TlulDownstreamOpVec(implicit p: Parameters) extends DownstreamOpVec {
    val wreq = Bool()
    val rreq = Bool()
    val wresp = Bool()
    val rdata = Bool()
    private def readReq(): Unit = {
      wreq := true.B
      rreq := false.B
      wresp := true.B
      rdata := false.B
    }
    private def writeReq(): Unit = {
      wreq := false.B
      rreq := true.B
      wresp := false.B
      rdata := true.B
    }
    def completed: Bool = this.asUInt.andR
    def decode(req: ReqFlit, check: Bool): Unit = {
      when(check) {
        assert(req.Opcode === ReqOpcode.ReadNoSnp || req.Opcode === ReqOpcode.WriteNoSnpPtl)
        assert(req.Size <= 3.U)
      }
      when(req.Opcode === ReqOpcode.ReadNoSnp) {
        readReq()
      }.otherwise {
        writeReq()
      }
    }
  }

  class TLULBridgeCtrlOpVec(implicit p: Parameters) extends IcnIoDevCtrlOpVecCommon {
    val d = new TlulDownstreamOpVec
    def icnReadReceipt: Bool = !u.receiptResp
    def icnDBID: Bool = !u.dbidResp
    def icnComp: Bool = !u.comp

    def tlaPut: Bool = !d.wreq & u.wdata
    def tlaGet: Bool = !d.rreq

    def needIssue: Bool = icnReadReceipt || icnDBID || icnComp || tlaPut || tlaGet
  }

  class TLULCtrlInfo(ioDataBits: Int)(implicit p: Parameters) extends IcnIoDevCtrlInfoCommon(ioDataBits = ioDataBits, withData = true, mem = false)

  class TLULRsEntry(dataBits: Int)(implicit p: Parameters) extends IcnIoDevRsEntryCommon[TLULBridgeCtrlOpVec, TLULCtrlInfo] {
    val state = new TLULBridgeCtrlOpVec
    val info = new TLULCtrlInfo(dataBits)
    def enq(req:ReqFlit, valid:Bool):Unit = {
      info.addr := req.Addr
      info.size := req.Size
      info.txnId := req.TxnID
      info.srcId := req.SrcID
      info.readCnt := 0.U
      state.u.decode(req, valid)
      state.d.decode(req, valid)
    }
  }
}
