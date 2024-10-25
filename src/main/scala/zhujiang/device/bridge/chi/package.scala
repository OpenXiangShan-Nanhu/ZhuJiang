package zhujiang.device.bridge

import chisel3._
import org.chipsalliance.cde.config.Parameters
import zhujiang.{ZJBundle, device}
import zhujiang.chi._

package object chi {
  class ChiDownstreamOpVec(implicit p: Parameters) extends DownstreamOpVec {
    val wreq = Bool()
    val rreq = Bool()
    val receiptResp = Bool()
    val dbidResp = Bool()
    val wdata = Bool()
    val rdata = Bool()
    val comp = Bool()
    private def readReq(order: UInt): Unit = {
      wreq := true.B
      rreq := false.B
      receiptResp := order === 0.U
      dbidResp := true.B
      wdata := true.B
      rdata := false.B
      comp := false.B
    }
    private def writeReq(): Unit = {
      wreq := false.B
      rreq := true.B
      receiptResp := true.B
      dbidResp := false.B
      wdata := false.B
      rdata := true.B
      comp := false.B
    }
    def completed: Bool = this.asUInt.andR
    def decode(req: ReqFlit, check: Bool): Unit = {
      when(check) {
        assert(req.Opcode === ReqOpcode.ReadNoSnp || req.Opcode === ReqOpcode.WriteNoSnpPtl)
        assert(req.Size <= 5.U)
      }
      when(req.Opcode === ReqOpcode.ReadNoSnp) {
        readReq(req.Order)
      }.otherwise {
        writeReq()
      }
    }
  }

  class ChiSnBridgeCtrlOpVec(implicit p: Parameters) extends IcnIoDevCtrlOpVecCommon {
    val d = new ChiDownstreamOpVec
    def icnReadReceipt: Bool = !u.receiptResp
    def icnDBID: Bool = !u.dbidResp
    def icnComp: Bool = !u.comp

    def snWriteNoSnpPtl: Bool = !d.wreq
    def snReadNoSnp: Bool = !d.rreq
    def snNCBWrDataCompAck: Bool = d.wreq && d.dbidResp && u.wdata && !d.wdata

    def needIssue: Bool = icnReadReceipt || icnDBID || icnComp || snWriteNoSnpPtl || snReadNoSnp || snNCBWrDataCompAck
  }

  class ChiSnBridgeCtrlInfo(ioDataBits: Int)(implicit p: Parameters)
    extends IcnIoDevCtrlInfoCommon(ioDataBits = ioDataBits, withData = true, dxt = false ){
    val order = UInt(2.W)
    val dbid = UInt(12.W)
    val isSnooped = Bool()
  }

  class ChiSnBridgeRsEntry(dataBits: Int)(implicit p: Parameters) extends IcnIoDevRsEntryCommon[ChiSnBridgeCtrlOpVec, ChiSnBridgeCtrlInfo] {
    val state = new ChiSnBridgeCtrlOpVec
    val info = new ChiSnBridgeCtrlInfo(dataBits)
    def enq(req:ReqFlit, valid:Bool) = {
      info.addr := req.Addr
      info.size := req.Size
      info.txnId := req.TxnID
      info.srcId := req.SrcID
      info.order := req.Order
      info.readCnt := 0.U
      state.u.decode(req, valid)
      state.d.decode(req, valid)
      info.isSnooped := true.B
    }
  }
}
