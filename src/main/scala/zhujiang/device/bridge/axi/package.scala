package zhujiang.device.bridge

import chisel3._
import org.chipsalliance.cde.config.Parameters
import zhujiang.chi.{ReqFlit, ReqOpcode}

package object axi {
  class AxiDownstreamOpVec(implicit p: Parameters) extends DownstreamOpVec {
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
        assert(req.Size <= 6.U)
      }
      when(req.Opcode === ReqOpcode.ReadNoSnp) {
        readReq()
      }.otherwise {
        writeReq()
      }
    }
  }

  class AxiBridgeCtrlOpVec(implicit p: Parameters) extends IcnIoDevCtrlOpVecCommon {
    val d = new AxiDownstreamOpVec
    val bufferAllocated = Bool()
    def icnReadReceipt: Bool = !u.receiptResp
    def icnDBID: Bool = bufferAllocated && !u.dbidResp
    def icnComp: Bool = bufferAllocated && !u.comp

    def axiWaddr: Bool = !d.waddr && u.wdata
    def axiRaddr: Bool = !d.raddr
    def axiWdata: Bool = d.waddr && !d.wdata && u.wdata

    def needIssue: Bool = icnReadReceipt || icnDBID || icnComp || axiWaddr || axiRaddr || axiWdata
  }

  class AxiCtrlInfo(implicit p: Parameters) extends IcnIoDevCtrlInfoCommon(ioDataBits = 0, withData = false, dxt = true) {
    val isSnooped = Bool()
  }

  class AxiRsEntry(implicit p: Parameters) extends IcnIoDevRsEntryCommon[AxiBridgeCtrlOpVec, AxiCtrlInfo] {
    val state = new AxiBridgeCtrlOpVec
    val info = new AxiCtrlInfo
    def enq(req:ReqFlit, valid:Bool):Unit = {
      info.addr := req.Addr
      info.size := req.Size
      info.txnId := req.TxnID
      info.srcId := req.SrcID
      info.returnTxnId.get := req.ReturnTxnID
      info.returnNid.get := req.ReturnNID
      info.dwt.get := req.Opcode =/= ReqOpcode.ReadNoSnp && req.DoDWT
      info.readCnt := 0.U
      state.u.decode(req, valid)
      state.d.decode(req, valid)
      state.bufferAllocated := req.Opcode === ReqOpcode.ReadNoSnp
      info.isSnooped := true.B
    }
  }
}
