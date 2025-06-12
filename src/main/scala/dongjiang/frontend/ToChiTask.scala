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
    val config  = new DJConfigIO()
    // CHI REQ IN
    val rxReq   = Flipped(Decoupled(new ReqFlit(false)))
    // CHI TASK OUT
    val chiTask = Decoupled(new PackChi with HasAddr)
  })

  // Connect Valid Ready
  io.chiTask.valid    := io.rxReq.valid
  io.rxReq.ready      := io.chiTask.ready
  

  // Connect Bits
  val req             = io.rxReq.bits
  val task            = io.chiTask.bits
  task.addr           := req.Addr
  task.chi.toLAN      := task.Addr.isToLAN(io.config.ci)
  task.chi.fromLAN    := NocType.rxIs(req, LAN)
  task.chi.nodeId     := req.SrcID
  task.chi.channel    := REQ
  task.chi.opcode     := req.Opcode
  task.chi.txnID      := req.TxnID
  // Use in req
  task.chi.order      := req.Order
  task.chi.snpAttr    := req.SnpAttr
  task.chi.snoopMe    := req.SnoopMe
  task.chi.memAttr    := req.MemAttr.asTypeOf(task.chi.memAttr)
  task.chi.expCompAck := req.ExpCompAck
  task.chi.qos        := DontCare // TODO: req.QoS
  // Use in snoop
  task.chi.fwdNID     := DontCare
  task.chi.fwdTxnID   := DontCare
  task.chi.retToSrc   := DontCare
  // Use Data Vec
  // Only applicable when dw == 256, the number of dataVec dimensions is the number of beats
  task.chi.dataVec(1) := req.Size === 6.U | req.Addr(5)
  task.chi.dataVec(0) := req.Size === 6.U | !req.Addr(5)
  require(djparam.nrBeat == 2)

  /*
   * HardwareAssertion
   */
  when(io.rxReq.valid) {
    val shouldBeFull = task.chi.isAllocatingRead | task.chi.isDataless | task.chi.isWriteFull
    when(task.chi.fromRni) {
      HAssert(!task.chi.snoopMe   , "Requests from RNI should not assert SnoopMe"                                                                                                                         )
      HAssert.withEn( task.chi.snpAttr   , (task.chi.opcode === ReqOpcode.ReadOnce  || task.chi.opcode === ReqOpcode.WriteUniquePtl)  , "Request from RNI access Cacheable mem should be assert SnpAttr"           )
      HAssert.withEn(!task.chi.snpAttr   , (task.chi.opcode === ReqOpcode.ReadNoSnp || task.chi.opcode === ReqOpcode.WriteNoSnpPtl )  , "Request from RNI access Non-Cacheable mem should not assert SnpAttr"      )
      HAssert.withEn( task.chi.isEO                        , task.chi.isRead  , "Requests from RNI should use EndpointOrder(EO) in read transactions"      )
      HAssert.withEn( task.chi.isOWO && task.chi.expCompAck, task.chi.isWrite , "Requests from RNI should use OrderedWriteOrder(OWO) and ExpCompAck in write transactions")
    }
    when(task.chi.fromCcRni & (task.chi.reqIs(ReadNoSnp) | task.chi.reqIs(WriteNoSnpPtl))) {
      HAssert(!task.chi.snpAttr                               , "CC-RNI Request should not assert SnpAttr"                                 )
      HAssert(!task.chi.snoopMe                               , "CC-RNI Request should not assert SnoopMe"                                 )
      HAssert.withEn( task.chi.isEO  && !task.chi.expCompAck  , task.chi.isRead   , "Requests of read from CC-RNI should use EndpointOrder(EO)")
      HAssert.withEn( task.chi.isOWO &&  task.chi.expCompAck  , task.chi.isWrite  , "Request of write form CC-RNI should use OWO"              )
    }
    when(task.chi.fromCcRnf & !(task.chi.reqIs(ReadNoSnp) | task.chi.reqIs(WriteNoSnpPtl))) {
      HAssert( task.chi.snpAttr                           , "Requests from CC-RNF must assert SnpAttr"                                                                                                 )
      HAssert( task.chi.noOrder                           , "Requests from CC-RNF should not assert Order"                                                                                             )
      HAssert.withEn( task.chi.expCompAck                 , task.chi.isRead                                                     , "Reuqests from CC-RNF should assert ExpCompAck in read transactions"          )
      HAssert.withEn( task.chi.expCompAck                 , task.chi.opcode === ReqOpcode.WriteEvictOrEvict                     , cf"Requests from CC-RNF which Opcode is ${task.chi.opcode} must assert ExpCompAck")
      HAssert.withEn(!task.chi.expCompAck                 , task.chi.isWrite & task.chi.opcode =/= ReqOpcode.WriteEvictOrEvict  , "Request from CC-RNF which ExpCompAck should not be asserted"                 )
    }
    when(task.chi.fromBBN) {
      HAssert( task.chi.noOrder , "Requests from BBN should not assert Order")
      HAssert( task.chi.toLAN   , cf"SrcID => [${task.chi.nodeId}]"              )
    }
    when(shouldBeFull) {
      HAssert( task.chi.isFullSize                                                                             )
      HAssert( task.Addr.offset === 0.U, "Offset should be 0 in allocatingRead/dataless/write transactions")
    }
    // Common
    HAssert(!task.chi.memAttr.device                                                                                                                                                                         )
    HAssert( task.chi.reqIsLegal                                                                                                                                                                             )
    HAssert( task.Addr.bankId === io.config.bankId, cf"BankId is not matched, bankdId = addr[${bankId_hi}:${bankId_lo}], task.chi.Addr.bankID = ${task.Addr.bankId}, io.config.bankId = ${io.config.bankId}" )
    HAssert.withEn(!task.chi.memAttr.allocate         ,!task.chi.memAttr.cacheable                                                                                                                               )
    HAssert.withEn( task.chi.memAttr.ewa              , task.chi.isCopyBackWrite                                                                                                                                 )
    HAssert.withEn(task.chi.fromCcRnf | task.chi.fromCcRni | task.chi.fromRniDma, task.chi.fromLAN, cf"Invalid NodeID, fromCcRnF: ${task.chi.fromCcRnf}, fromCcRni: ${task.chi.fromCcRni}, fromRniDma: ${task.chi.fromRniDma}, SrcID: [${task.chi.nodeId}]\t\nccNodeIdSeq: ${ccNodeIdSeq}\t\nrniNodeIdSeq: ${rniNodeIdSeq}\t\nsnNodeIdSeq:${snNodeIdSeq}")

  }

  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(1)
}


class SnpToChiTask(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config  = new DJConfigIO()
    // CHI REQ IN
    val rxSnp   = Flipped(Decoupled(new SnoopFlit()))
    // CHI TASK OUT
    val chiTask = Decoupled(new PackChi with HasAddr)
  })

  HardwareAssertion(!io.rxSnp.valid)

  // Connect Valid Ready
  io.chiTask.valid    := io.rxSnp.valid
  io.rxSnp.ready      := io.chiTask.ready


  // Connect Bits
  val snp             = io.rxSnp.bits
  val task            = io.chiTask.bits
  task.addr           := Cat(snp.Addr, 0.U(3.W))
  task.chi.toLAN      := true.B
  task.chi.fromLAN    := false.B
  task.chi.nodeId     := snp.SrcID
  task.chi.channel    := SNP
  task.chi.opcode     := snp.Opcode
  task.chi.txnID      := snp.TxnID
  // Use in req
  task.chi.order      := DontCare
  task.chi.snpAttr    := DontCare
  task.chi.snoopMe    := DontCare
  task.chi.memAttr    := DontCare
  task.chi.expCompAck := DontCare
  // Use in snoop
  task.chi.fwdNID     := snp.FwdNID
  task.chi.fwdTxnID   := snp.FwdTxnID
  task.chi.retToSrc   := snp.RetToSrc
  task.chi.qos        := DontCare // TODO: snp.QoS
  // Use Data Vec
  task.chi.dataVec(1) := true.B
  task.chi.dataVec(0) := true.B

  /*
   * HardwareAssertion
   */
  when(io.rxSnp.valid) {
  // TODO
  // QoS
  // TgtID
  // SrcID
  HardwareAssertion(task.chi.fromBBN)
  HardwareAssertion(task.chi.bbnCI =/= io.config.ci)
  HardwareAssertion(task.chi.bbnBId === io.config.bankId)
  // TxnID
  // FwdNID
  // PBHA
  // FwdTxnID
  // StashLPIDValid
  // StashLPID
  // VMIDExt
  // Opcode
  HardwareAssertion(task.chi.snpIsLegal)
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
  HardwareAssertion.placePipe(1)
}