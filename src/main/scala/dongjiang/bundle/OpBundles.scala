package dongjiang.bundle

import chisel3._
import chisel3.util._
import dongjiang.DJBundle
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.SnpOpcode._

trait HasChiOp { this: DJBundle with HasChiChannel =>
  val opcode = UInt(ChiOpcodeBits.W)

  def reqIs(op: UInt*): Bool = {
    isReq & Cat(op.map(_ === opcode)).orR
  }

  def snpIs(op: UInt*): Bool = {
    isSnp & Cat(op.map(_ === opcode)).orR
  }

  def rspIs(op: UInt*): Bool = {
    isRsp& Cat(op.map(_ === opcode)).orR
  }

  def datIs(op: UInt*): Bool = {
    isDat & Cat(op.map(_ === opcode)).orR
  }

  // REQ
  def isAllocatingRead: Bool = reqIs(
    ReadClean,
    ReadNotSharedDirty,
    ReadShared,
    ReadUnique,
    ReadPreferUnique,
    MakeReadUnique,
  )

  // REQ
  def isNonAllocatingRead: Bool = reqIs(
    ReadNoSnp,
    ReadNoSnpSep, // Only used in Home Node to Subordinate Node, requesting the Completer to send only a data response.
    ReadOnce,
    ReadOnceCleanInvalid,
    ReadOnceMakeInvalid,
  )

  // REQ
  def isRead: Bool = isAllocatingRead | isNonAllocatingRead

  // REQ
  def isDataless: Bool = reqIs(
    CleanUnique,
    MakeUnique,
    Evict,
    StashOnceUnique,
    StashOnceSepUnique,
    StashOnceShared,
    StashOnceSepShared,
    CleanShared,
    CleanSharedPersist,
    CleanSharedPersistSep,
    CleanInvalid,
    CleanInvalidPoPA,
    MakeInvalid,
  )

  // REQ
  def isWriteFull: Bool = reqIs(
    WriteNoSnpFull,
    WriteUniqueFull,
    WriteUniqueFullStash,
    WriteBackFull,
    WriteCleanFull,
    WriteEvictFull,
    WriteEvictOrEvict,
  )

  def isWritePtl: Bool = reqIs(
    WriteNoSnpPtl,
    WriteUniquePtl,
    WriteUniquePtlStash,
    WriteBackPtl,
  )

  def isImmediateWrite: Bool = reqIs(
    WriteNoSnpPtl,
    WriteNoSnpFull,
    WriteNoSnpDef,
    WriteUniquePtl,
    WriteUniqueFull,
    WriteUniquePtlStash,
    WriteUniqueFullStash,
  )

  def isCopyBackWrite: Bool = reqIs(
    WriteBackFull,
    WriteBackPtl,
    WriteCleanFull,
    WriteEvictOrEvict,
    WriteEvictFull,
  )

  def isWrite: Bool = reqIs(
    WriteNoSnpPtl,
    WriteNoSnpFull,
    WriteNoSnpZero,
    WriteNoSnpDef,
    WriteUniquePtl,
    WriteUniqueFull,
    WriteUniqueZero,
    WriteUniquePtlStash,
    WriteUniqueFullStash,
    WriteBackPtl,
    WriteBackFull,
    WriteCleanFull,
    WriteEvictFull,
    WriteEvictOrEvict,
  )

  // REQ
  def isCombinedWrite: Bool = reqIs(
    WriteNoSnpPtlCleanInv,
    WriteNoSnpPtlCleanSh,
    WriteNoSnpPtlCleanShPerSep,
    WriteNoSnpPtlCleanInvPoPA,
    WriteNoSnpFullCleanInv,
    WriteNoSnpFullCleanSh,
    WriteNoSnpFullCleanShPerSep,
    WriteNoSnpFullCleanInvPoPA,
    WriteUniquePtlCleanSh,
    WriteUniquePtlCleanShPerSep,
    WriteUniqueFullCleanSh,
    WriteUniqueFullCleanShPerSep,
    WriteBackFullCleanInv,
    WriteBackFullCleanSh,
    WriteBackFullCleanShPerSep,
    WriteBackFullCleanInvPoPA,
    WriteCleanFullCleanSh,
    WriteCleanFullCleanShPerSep,
  )

  // REQ
  def isAtomicStore: Bool = reqIs(
    AtomicStoreADD,
    AtomicStoreCLR,
    AtomicStoreEOR,
    AtomicStoreSET,
    AtomicStoreSMAX,
    AtomicStoreSMIN,
    AtomicStoreUMAX,
    AtomicStoreUMIN,
  )

  // REQ
  def isAtomicLoad: Bool = reqIs(
    AtomicLoadADD,
    AtomicLoadCLR,
    AtomicLoadEOR,
    AtomicLoadSET,
    AtomicLoadSMAX,
    AtomicLoadSMIN,
    AtomicLoadUMAX,
    AtomicLoadUMIN,
  )

  // REQ
  def isAtomicSwap: Bool = reqIs(
    AtomicSwap,
  )

  // REQ
  def isAtomicCompare: Bool = reqIs(
    AtomicCompare,
  )

  // REQ
  def isAtomic: Bool = isAtomicStore | isAtomicLoad | isAtomicSwap | isAtomicCompare

  // REQ
  def isOther: Bool = reqIs(
    DVMOp,
    PrefetchTgt,
    PCrdReturn,
  )

  // REQ: Get Atomic type expect AmoticStoreX
  def getAtomicOp: UInt = opcode(3, 0)

  // REQ: legal
  // Note: If it is modified, you need to check if reqNeedData needs to be modified accordingly.
  def reqIsLegal: Bool = reqIs(
    // Read
    ReadNoSnp,
    ReadOnce,
    ReadNotSharedDirty,
    ReadUnique,
    // Dataless
    MakeUnique,
    Evict,
    CleanShared,
    CleanInvalid,
    MakeInvalid,
    // Write
    WriteNoSnpPtl,
    WriteUniquePtl,
    WriteUniqueFull,
    WriteBackFull,
    WriteEvictOrEvict,
  ) | isAtomicLoad | isAtomicSwap | isAtomicCompare

  // RSP
  def isSnpResp: Bool = rspIs(
    SnpResp,
    SnpRespFwded,
  )

  // DAT
  def isSnpRespData: Bool = datIs(
    SnpRespData,
    SnpRespDataPtl,
    SnpRespDataFwded,
  )

  // SNP
  def isSnpFwd: Bool = snpIs(
    SnpSharedFwd,
    SnpCleanFwd,
    SnpOnceFwd,
    SnpNotSharedDirtyFwd,
    SnpPreferUniqueFwd,
    SnpUniqueFwd,
  )

  // SNP: legal
  // Note: If it is modified, you need to check if snpNeedData needs to be modified accordingly.
  def snpIsLegal: Bool = snpIs(
    // SnpLCrdReturn,
    // SnpShared,
    // SnpClean,
    SnpOnce,
    SnpNotSharedDirty,
    // SnpUniqueStash,
    // SnpMakeInvalidStash,
    SnpUnique,
    // SnpCleanShared,
    // SnpCleanInvalid,
    SnpMakeInvalid,
    // SnpStashUnique,
    // SnpStashShared,
    // SnpDVMOp,
    // SnpQuery,
    // SnpSharedFwd,
    // SnpCleanFwd,
    SnpOnceFwd,
    SnpNotSharedDirtyFwd,
    // SnpPreferUnique,
    // SnpPreferUniqueFwd,
    SnpUniqueFwd,
  )

  // transfer SnpFwd to Snp
  def getNoFwdSnpOp: UInt = {
    val snpOp = WireInit(0.U(ChiOpcodeBits.W))
    when(isSnpFwd) {
      switch(opcode) {
        is(SnpSharedFwd) {
          snpOp := SnpShared
        }
        is(SnpCleanFwd) {
          snpOp := SnpClean
        }
        is(SnpOnceFwd) {
          snpOp := SnpOnce
        }
        is(SnpNotSharedDirtyFwd) {
          snpOp := SnpNotSharedDirty
        }
        is(SnpPreferUniqueFwd) {
          snpOp := SnpPreferUnique
        }
        is(SnpUniqueFwd) {
          snpOp := SnpUnique
        }
      }
    }.otherwise {
      snpOp := opcode
    }
    snpOp
  }
}
