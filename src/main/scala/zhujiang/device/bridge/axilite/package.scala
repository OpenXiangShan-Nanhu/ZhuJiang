package zhujiang.device.bridge

import chisel3._
import org.chipsalliance.cde.config.Parameters
import zhujiang.{ZJBundle, device}
import zhujiang.chi.{ReqFlit, ReqOpcode}
import zhujiang.device.bridge.tlul.TlulDownstreamOpVec
import zhujiang.device.bridge.{DownstreamOpVec, IcnIoDevCtrlInfoCommon, IcnIoDevCtrlOpVecCommon, IcnIoDevRsEntryCommon}

package object axilite {
  class AxiLiteDownstreamOpVec(implicit p: Parameters) extends DownstreamOpVec {
    val waddr = Bool()
    val raddr = Bool()
    val wdata = Bool()
    val wresp = Bool()
    val rdata = Bool()
    private def readReq(): Unit = {
      waddr := true.B
      raddr := false.B
      wdata := true.B
      wresp := true.B
      rdata := false.B
    }
    private def writeReq(): Unit = {
      waddr := false.B
      raddr := true.B
      wdata := false.B
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

  class AxiLiteBridgeCtrlOpVec(implicit p: Parameters) extends IcnIoDevCtrlOpVecCommon {
    val d = new AxiLiteDownstreamOpVec
    def icnReadReceipt: Bool = !u.receiptResp
    def icnDBID: Bool = !u.dbidResp
    def icnComp: Bool = !u.comp

    def axiWaddr: Bool = !d.waddr && u.wdata
    def axiRaddr: Bool = !d.raddr
    def axiWdata: Bool = d.waddr && !d.wdata && u.wdata

    def needIssue: Bool = icnReadReceipt || icnDBID || icnComp || axiWaddr || axiRaddr || axiWdata
  }

  class AxiLiteCtrlInfo(ioDataBits: Int)(implicit p: Parameters) extends IcnIoDevCtrlInfoCommon(ioDataBits = ioDataBits, withData = true, dxt = false) {
    val isSnooped = Bool()
    val write = Bool()
  }

  class AxiLiteRsEntry(dataBits: Int)(implicit p: Parameters) extends IcnIoDevRsEntryCommon[AxiLiteBridgeCtrlOpVec, AxiLiteCtrlInfo] {
    val state = new AxiLiteBridgeCtrlOpVec
    val info = new AxiLiteCtrlInfo(dataBits)
    def enq(req:ReqFlit, valid:Bool):Unit = {
      info.addr := req.Addr
      info.size := req.Size
      info.txnId := req.TxnID
      info.srcId := req.SrcID
      info.readCnt := 0.U
      state.u.decode(req, valid)
      state.d.decode(req, valid)
      info.isSnooped := true.B
      info.write := req.Opcode =/= ReqOpcode.ReadNoSnp
    }
  }
}
