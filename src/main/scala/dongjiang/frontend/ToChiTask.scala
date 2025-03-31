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
  task.order    := req.Order
  task.snpAttr  := req.SnpAttr
  task.snoopMe  := req.SnoopMe
  task.memAttr  := req.MemAttr.asTypeOf(task.memAttr)
  task.expCompAck := req.ExpCompAck
  // Use in snoop
  task.fwdNID   := DontCare
  task.fwdTxnID := DontCare
  task.retToSrc := DontCare
  // Use Data Vec
  task.dataVec(1) := req.Size === 6.U | req.Addr(5)
  task.dataVec(0) := req.Size === 6.U | !req.Addr(5)

  /*
   * HardwareAssertion
   */
  when(io.rxReq.valid) {
    // TODO
    // QoS
    // TgtID
    // SrcID
    HardwareAssertion.withEn(task.fromCcRnf | task.fromCcRni | task.fromRniDma, task.fromLAN, cf"Invalid NodeID, fromCcRnF: ${task.fromCcRnf}, fromCcRni: ${task.fromCcRni}, fromRniDma: ${task.fromRniDma}, SrcID: [${task.nodeId}]\t\nccNodeIdSeq: ${ccNodeIdSeq}\t\nrniNodeIdSeq: ${rniNodeIdSeq}\t\nsnNodeIdSeq:${snNodeIdSeq}")
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
    HardwareAssertion.withEn(task.isHalfSize, task.isAtomic)
    // Addr
    HardwareAssertion(task.Addr.bankId === io.config.bankId, cf"BankId is not matched, bankdId = addr[${bankId_hi}:${bankId_lo}], task.Addr.bankID = ${task.Addr.bankId}, io.config.bankId = ${io.config.bankId}")
    HardwareAssertion.withEn(task.Addr.offset === 0.U, task.isAllocatingRead | task.isDataless | task.isWriteFull, "Offset should be 0 in allocatingRead/dataless/write transactions")
    // NS
    // NSE
    // LikelyShared
    // AllowRetry
    // Order
    HardwareAssertion.withEn(task.noOrder, task.fromCcRnf, "Requests from CC-RNF should not have any order(i.e. order = None)")
    HardwareAssertion.withEn(task.isEO,    task.fromCcRni, "Requests from CC-RNI should use EndpointOrder(EO)")
    HardwareAssertion.withEn(task.isEO,    task.fromRni & task.isRead, "Requests from RNI should use EndpointOrder(EO) in read transactions")
    HardwareAssertion.withEn(task.isOWO,   task.fromRni & task.isWrite, "Requests from RNI should use OrderedWriteOrder(OWO) in write transactions")
    HardwareAssertion.withEn(task.noOrder, task.fromBBN)
    // PCrdType
    // MemAttr
    HardwareAssertion.withEn(task.memAttr.ewa,  task.isCopyBackWrite)
    HardwareAssertion(!task.memAttr.device)
    HardwareAssertion.withEn(!task.memAttr.allocate, !task.memAttr.cacheable)
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
    HardwareAssertion.withEn(!task.snoopMe, task.fromCcRni, "Requests from CC-RNI shoule not expect to be snooped")
    HardwareAssertion.withEn(!task.snoopMe, task.fromRni, "Requests from RNI shoule not expect to be snooped")
    // CAH
    // ExpCompAck
    HardwareAssertion.withEn(task.expCompAck,  task.fromCcRnf & task.isRead, "Reuqests from CC-RNF should assert ExpCompAck in read transactions")
    HardwareAssertion.withEn(!task.expCompAck, task.fromCcRnf & !task.isRead, "Reuqests from CC-RNF should not assert ExpCompAck while transaction is not read")
    HardwareAssertion.withEn(!task.expCompAck, task.fromCcRni, "Reuqests from CC-RNI should not assert ExpCompAck")
    HardwareAssertion.withEn(!task.expCompAck, task.fromRni, "Reuqests from RNI should not assert ExpCompAck")
    // TagOp
    // TraceTag
    // MPAM
    // PBHA
    // MECID
    // StreamID
    // SecSID1
    // RSVDC
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
  task.order    := DontCare
  task.snpAttr  := DontCare
  task.snoopMe  := DontCare
  task.memAttr  := DontCare
  task.expCompAck := DontCare
  // Use in snoop
  task.fwdNID   := snp.FwdNID
  task.fwdTxnID := snp.FwdTxnID
  task.retToSrc := snp.RetToSrc
  // Use Data Vec
  task.dataVec(1) := true.B
  task.dataVec(0) := true.B

  /*
   * HardwareAssertion
   */
  when(io.rxSnp.valid) {
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