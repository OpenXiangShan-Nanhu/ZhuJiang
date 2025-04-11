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
    val shouldBeFull = task.isAllocatingRead | task.isDataless | task.isWriteFull
    when(task.fromRni) {
      HAssert( task.expCompAck, "Requests form RNI should assert ExpCompAck"                                                                                                                          )
      HAssert(!task.snoopMe   , "Requests from RNI should not assert SnoopMe"                                                                                                                         )
      HAssert.withEn( task.snpAttr   , (task.opcode === ReqOpcode.ReadOnce  || task.opcode === ReqOpcode.WriteUniquePtl) , "Request from RNI access Cacheable mem should be assert SnpAttr"           )
      HAssert.withEn(!task.snpAttr   , (task.opcode === ReqOpcode.ReadNoSnp || task.opcode === ReqOpcode.WriteNoSnpPtl ) , "Request from RNI access Non-Cacheable mem should not assert SnpAttr"      )
      HAssert.withEn( task.noOrder   , task.isRead                                                                       , "Requests from RNI should use EndpointOrder(EO) in read transactions"      )
      HAssert.withEn( task.isOWO     , task.isWrite                                                                      , "Requests from RNI should use OrderedWriteOrder(OWO) in write transactions")
    }
    when(task.fromCcRni & (task.reqIs(ReadNoSnp) | task.reqIs(WriteNoSnpPtl))) {
      HAssert(!task.snpAttr                           , "CC-RNI Request should not assert SnpAttr"                                 )
      HAssert(!task.snoopMe                           , "CC-RNI Request should not assert SnoopMe"                                 )
      HAssert.withEn( task.isEO  && !task.expCompAck  , task.isRead   , "Requests of read from CC-RNI should use EndpointOrder(EO)")
      HAssert.withEn( task.isOWO &&  task.expCompAck  , task.isWrite  , "Request of write form CC-RNI should use OWO"              )
    }
    when(task.fromCcRnf & !(task.reqIs(ReadNoSnp) | task.reqIs(WriteNoSnpPtl))) {
      HAssert( task.snpAttr                           , "Requests from CC-RNF must assert SnpAttr"                                                                                                 )
      HAssert( task.noOrder                           , "Requests from CC-RNF should not assert Order"                                                                                             )
      HAssert.withEn( task.expCompAck                 , task.isRead                                                , "Reuqests from CC-RNF should assert ExpCompAck in read transactions"          )
      HAssert.withEn( task.expCompAck                 , task.opcode === ReqOpcode.WriteEvictOrEvict                , cf"Requests from CC-RNF which Opcode is ${task.opcode} must assert ExpCompAck")
      HAssert.withEn(!task.expCompAck                 , task.isWrite & task.opcode =/= ReqOpcode.WriteEvictOrEvict , "Request from CC-RNF which ExpCompAck should not be asserted"                 )
    }
    when(task.fromBBN) {
      HAssert( task.noOrder            , "Requests from BBN should not assert Order")
      HAssert( task.toLAN(io.config.ci), cf"SrcID => [${task.nodeId}]"              )
    }
    when(shouldBeFull) {
      HAssert( task.isFullSize                                                                             )
      HAssert( task.Addr.offset === 0.U, "Offset should be 0 in allocatingRead/dataless/write transactions")
    }
    // Common
    HAssert(!task.memAttr.device                                                                                                                                                                         )
    HAssert( task.reqIsLegal                                                                                                                                                                             )
    HAssert( task.Addr.bankId === io.config.bankId, cf"BankId is not matched, bankdId = addr[${bankId_hi}:${bankId_lo}], task.Addr.bankID = ${task.Addr.bankId}, io.config.bankId = ${io.config.bankId}" )
    HAssert.withEn(!task.memAttr.allocate         ,!task.memAttr.cacheable                                                                                                                               )
    HAssert.withEn( task.memAttr.ewa              , task.isCopyBackWrite                                                                                                                                 )
    HAssert.withEn(task.fromCcRnf | task.fromCcRni | task.fromRniDma, task.fromLAN, cf"Invalid NodeID, fromCcRnF: ${task.fromCcRnf}, fromCcRni: ${task.fromCcRni}, fromRniDma: ${task.fromRniDma}, SrcID: [${task.nodeId}]\t\nccNodeIdSeq: ${ccNodeIdSeq}\t\nrniNodeIdSeq: ${rniNodeIdSeq}\t\nsnNodeIdSeq:${snNodeIdSeq}")

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