package zhujiang.device.bridge

import chisel3._
import chisel3.util.Cat
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
    def issued:Bool = waddr & raddr
    def decode(req: ReqFlit, check: Bool): Unit = {
      when(check) {
        val legalCode = Seq(ReqOpcode.ReadNoSnp, ReqOpcode.WriteNoSnpPtl, ReqOpcode.WriteNoSnpFull, ReqOpcode.WriteNoSnpFullCleanInv)
        val legal = Cat(legalCode.map(_ === req.Opcode)).orR
        assert(legal, "opcode: 0x%x", req.Opcode)
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
    def wakeup: Bool = d.wresp && d.rdata
  }

  class AxiCtrlInfo(implicit p: Parameters) extends IcnIoDevCtrlInfoCommon(ioDataBits = 0, withData = false, mem = true)

  class AxiRsEntry(implicit p: Parameters) extends IcnIoDevRsEntryCommon[AxiBridgeCtrlOpVec, AxiCtrlInfo] {
    val state = new AxiBridgeCtrlOpVec
    val info = new AxiCtrlInfo
    override def enq(req:ReqFlit, valid:Bool):Unit = {
      super.enq(req, valid)
      state.bufferAllocated := req.Opcode === ReqOpcode.ReadNoSnp
    }
  }
}
