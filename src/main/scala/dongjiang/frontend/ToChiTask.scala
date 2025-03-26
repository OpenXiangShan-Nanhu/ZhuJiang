package dongjiang.frontend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle.ChiChannel._
import xs.utils.debug._
import zhujiang.chi.ReqOpcode._
import dongjiang.bundle._

class ReqToChiTask(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config = new DJConfigIO()
    // CHI REQ IN
    val rxReq   = Flipped(Decoupled(new ReqFlit(false)))
    // CHI TASK OUT
    val chiTask = Decoupled(new Chi with HasAddr)
  })

  // Connect Valid Ready
  io.chiTask.valid := io.rxReq.valid
  io.rxReq.ready   := io.chiTask.ready
  

  // Connect Bits
  val req       = io.rxReq.bits
  val task      = io.chiTask.bits
  task.addr     := req.Addr
  task.toLAN    := task.Addr.isToLAN(io.config.ci)
  task.fromLAN  := NocType.rxIs(req, LAN)
  task.nodeId   := req.SrcID
  task.channel  := REQ
  task.opcode   := req.Opcode
  task.txnID    := req.TxnID
  // Use in req
  task.dataId   := task.Addr.getDataId
  task.order    := req.Order
  task.snpAttr  := req.SnpAttr
  task.snoopMe  := req.SnoopMe
  task.size     := req.Size
  task.memAttr  := req.MemAttr.asTypeOf(task.memAttr)
  task.expCompAck := req.ExpCompAck
  // Use in snoop
  task.fwdNID   := DontCare
  task.fwdTxnID := DontCare
  task.retToSrc := DontCare

  /*
   * HardwareAssertion
   */
  awhen(io.rxReq.valid) {
    // TODO
    // QoS
    // TgtID
    // SrcID
    HardwareAssertion.withEn(task.fromCcRnf | task.fromCcRni | task.fromRniDma, task.fromLAN, cf"SrcID => [${task.nodeId}]")
    HardwareAssertion.withEn(task.toLAN(io.config.ci), task.fromBBN, cf"SrcID => [${task.nodeId}]")
    // TxnID
    // ReturnNID
    // StashNID
    // DataTarget
    // StashNIDValid
    // Endian
    // Deep
    // PrefetchTgtHint
    // ReturnTxnID
    // StashLPIDValid
    // StashLPID
    // Opcode
    HardwareAssertion(task.reqIsLegal)
    // Size
    HardwareAssertion.withEn(task.isFullSize, task.isAllocatingRead | task.isDataless | task.isWriteFull)
    HardwareAssertion.withEn(task.isNotFullSize, task.isAtomic)
    // Addr
    HardwareAssertion(task.Addr.bankId === io.config.bankId)
    HardwareAssertion.withEn(task.Addr.offset === 0.U, task.isAllocatingRead | task.isDataless | task.isWriteFull)
    // NS
    // NSE
    // LikelyShared
    // AllowRetry
    // Order
    HardwareAssertion.withEn(task.noOrder, task.fromCcRnf)
    HardwareAssertion.withEn(task.isEO,    task.fromCcRni)
    HardwareAssertion.withEn(task.isEO,    task.fromRni & task.isRead)
    HardwareAssertion.withEn(task.isOWO,   task.fromRni & task.isWrite)
    HardwareAssertion.withEn(task.noOrder, task.fromBBN)
    // PCrdType
    // MemAttr
    HardwareAssertion.withEn(task.memAttr.ewa,  task.isCopyBackWrite)
    HardwareAssertion(!task.memAttr.device)
    HardwareAssertion.withEn(!task.memAttr.allocate, !task.memAttr.cacheable)
    HardwareAssertion.placePipe(Int.MaxValue-3)
    // SnpAttr
    HardwareAssertion.withEn(task.snpAttr,  task.fromCcRnf)
    HardwareAssertion.withEn(!task.snpAttr, task.fromCcRni)
    HardwareAssertion.withEn(!task.snpAttr, task.fromRni)
    // DoDWT
    // PGroupID
    // StashGroupID
    // TagGroupID
    // LPID
    // Excl
    // SnoopMe
    HardwareAssertion.withEn(!task.snoopMe, task.fromCcRni)
    HardwareAssertion.withEn(!task.snoopMe, task.fromRni)
    // CAH
    // ExpCompAck
    HardwareAssertion.withEn(task.expCompAck,  task.fromCcRnf & task.isRead)
    HardwareAssertion.withEn(!task.expCompAck, task.fromCcRnf & !task.isRead)
    HardwareAssertion.withEn(!task.expCompAck, task.fromCcRni)
    HardwareAssertion.withEn(!task.expCompAck, task.fromRni)
    // TagOp
    // TraceTag
    // MPAM
    // PBHA
    // MECID
    // StreamID
    // SecSID1
    // RSVDC
    HardwareAssertion.placePipe(Int.MaxValue-3)
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}


class SnpToChiTask(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config = new DJConfigIO()
    // CHI REQ IN
    val rxSnp   = Flipped(Decoupled(new SnoopFlit()))
    // CHI TASK OUT
    val chiTask = Decoupled(new Chi with HasAddr)
  })

  HardwareAssertion(!io.rxSnp.valid)

  // Connect Valid Ready
  io.chiTask.valid := io.rxSnp.valid
  io.rxSnp.ready   := io.chiTask.ready


  // Connect Bits
  val snp       = io.rxSnp.bits
  val task      = io.chiTask.bits
  task.addr     := Cat(snp.Addr, 0.U(3.W))
  task.toLAN    := true.B
  task.fromLAN  := false.B
  task.nodeId   := snp.SrcID
  task.channel  := SNP
  task.opcode   := snp.Opcode
  task.txnID    := snp.TxnID
  // Use in req
  task.dataId   := DontCare
  task.order    := DontCare
  task.snpAttr  := DontCare
  task.snoopMe  := DontCare
  task.size     := DontCare
  task.memAttr  := DontCare
  task.expCompAck := DontCare
  // Use in snoop
  task.fwdNID   := snp.FwdNID
  task.fwdTxnID := snp.FwdTxnID
  task.retToSrc := snp.RetToSrc

  /*
   * HardwareAssertion
   */
awhen(io.rxSnp.valid) {
  // TODO
  // QoS
  // TgtID
  // SrcID
  HardwareAssertion(task.fromBBN)
  HardwareAssertion(task.bbnCI =/= io.config.ci)
  HardwareAssertion(task.bbnBId === io.config.bankId)
  // TxnID
  // FwdNID
  // PBHA
  // FwdTxnID
  // StashLPIDValid
  // StashLPID
  // VMIDExt
  // Opcode
  HardwareAssertion(task.snpIsLegal)
  // Addr
  HardwareAssertion(task.Addr.ci === io.config.ci)
  // NS
  // NSE
  // DoNotGoToSD
  // RetToSrc
  // TraceTag
  // MPAM
  // MECID
  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}