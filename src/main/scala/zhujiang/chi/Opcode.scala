package zhujiang.chi

import chisel3._
import chisel3.util._

object ReqOpcode {
  val width = 7
  val ReqLCrdReturn = 0x00.U(width.W)
  val ReadShared = 0x01.U(width.W)
  val ReadClean = 0x02.U(width.W)
  val ReadOnce = 0x03.U(width.W)
  val ReadNoSnp = 0x04.U(width.W)
  val PCrdReturn = 0x05.U(width.W)
  val ReadUnique = 0x07.U(width.W)
  val CleanShared = 0x08.U(width.W)
  val CleanInvalid = 0x09.U(width.W)
  val MakeInvalid = 0x0A.U(width.W)
  val CleanUnique = 0x0B.U(width.W)
  val MakeUnique = 0x0C.U(width.W)
  val Evict = 0x0D.U(width.W)

  val ReadNoSnpSep = 0x11.U(width.W)
  val CleanSharedPersistSep = 0x13.U(width.W)
  val DVMOp = 0x14.U(width.W)
  val WriteEvictFull = 0x15.U(width.W)
  val WriteCleanFull = 0x17.U(width.W)
  val WriteUniquePtl = 0x18.U(width.W)
  val WriteUniqueFull = 0x19.U(width.W)
  val WriteBackPtl = 0x1A.U(width.W)
  val WriteBackFull = 0x1B.U(width.W)
  val WriteNoSnpPtl = 0x1C.U(width.W)
  val WriteNoSnpFull = 0x1D.U(width.W)

  val WriteUniqueFullStash = 0x20.U(width.W)
  val WriteUniquePtlStash = 0x21.U(width.W)
  val StashOnceShared = 0x22.U(width.W)
  val StashOnceUnique = 0x23.U(width.W)
  val ReadOnceCleanInvalid = 0x24.U(width.W)
  val ReadOnceMakeInvalid = 0x25.U(width.W)
  val ReadNotSharedDirty = 0x26.U(width.W)
  val CleanSharedPersist = 0x27.U(width.W)
  val AtomicStoreADD = 0x28.U(width.W)
  val AtomicStoreCLR = 0x29.U(width.W)
  val AtomicStoreEOR = 0x2A.U(width.W)
  val AtomicStoreSET = 0x2B.U(width.W)
  val AtomicStoreSMAX = 0x2C.U(width.W)
  val AtomicStoreSMIN = 0x2D.U(width.W)
  val AtomicStoreUMAX = 0x2E.U(width.W)
  val AtomicStoreUMIN = 0x2F.U(width.W)

  val AtomicLoadADD = 0x30.U(width.W)
  val AtomicLoadCLR = 0x31.U(width.W)
  val AtomicLoadEOR = 0x32.U(width.W)
  val AtomicLoadSET = 0x33.U(width.W)
  val AtomicLoadSMAX = 0x34.U(width.W)
  val AtomicLoadSMIN = 0x35.U(width.W)
  val AtomicLoadUMAX = 0x36.U(width.W)
  val AtomicLoadUMIN = 0x37.U(width.W)
  val AtomicSwap = 0x38.U(width.W)
  val AtomicCompare = 0x39.U(width.W)
  val PrefetchTgt = 0x3A.U(width.W)

  val MakeReadUnique = 0x41.U(width.W)
  val WriteEvictOrEvict = 0x42.U(width.W)
  val WriteUniqueZero = 0x43.U(width.W)
  val WriteNoSnpZero = 0x44.U(width.W)
  val StashOnceSepShared = 0x47.U(width.W)
  val StashOnceSepUnique = 0x48.U(width.W)
  val ReadPreferUnique = 0x4C.U(width.W)
  val CleanInvalidPoPAa = 0x4D.U(width.W)
  val WriteNoSnpDef = 0x4E.U(width.W)

  val WriteNoSnpFullCleanSh = 0x50.U(width.W)
  val WriteNoSnpFullCleanInv = 0x51.U(width.W)
  val WriteNoSnpFullCleanShPerSep = 0x52.U(width.W)
  val WriteUniqueFullCleanSh = 0x54.U(width.W)
  val WriteUniqueFullCleanShPerSep = 0x56.U(width.W)
  val WriteBackFullCleanSh = 0x58.U(width.W)
  val WriteBackFullCleanInv = 0x59.U(width.W)
  val WriteBackFullCleanShPerSep = 0x5A.U(width.W)
  val WriteCleanFullCleanSh = 0x5C.U(width.W)
  val WriteCleanFullCleanShPerSep = 0x5E.U(width.W)

  val WriteNoSnpPtlCleanSh = 0x60.U(width.W)
  val WriteNoSnpPtlCleanInv = 0x61.U(width.W)
  val WriteNoSnpPtlCleanShPerSep = 0x62.U(width.W)
  val WriteUniquePtlCleanSh = 0x64.U(width.W)
  val WriteUniquePtlCleanShPerSep = 0x66.U(width.W)

  val WriteNoSnpPtlCleanInvPoPA = 0x70.U(width.W)
  val WriteNoSnpFullCleanInvPoPA = 0x71.U(width.W)
  val WriteBackFullCleanInvPoPA = 0x79.U(width.W)

  // Self Define
  val Replace = 0x7A.U(width.W)
  val FlushDCU = 0x7B.U(width.W)

  // Judge req type
  def isReadX         (x: UInt): Bool = (ReadShared <= x & x <= ReadNoSnp) | x === ReadUnique | x === ReadNoSnpSep |
                                        (ReadOnceCleanInvalid <= x & x <= ReadNotSharedDirty) | x === ReadPreferUnique
  def isDatalessX     (x: UInt): Bool = (CleanShared <= x & x <= Evict) | x === CleanSharedPersistSep | (StashOnceShared <= x & x <= StashOnceUnique) | x === CleanSharedPersist | (StashOnceSepShared <= x & x <= StashOnceSepUnique) | x === CleanInvalidPoPAa
  def isWriteX        (x: UInt): Bool = (WriteEvictFull <= x & x <= WriteUniquePtlStash) | (WriteEvictOrEvict <= x & x <= WriteNoSnpZero) |
                                        (WriteNoSnpDef <= x & x <= WriteBackFullCleanInvPoPA)
  def isCBX           (x: UInt): Bool = x === WriteBackFull | x === WriteBackPtl | x === WriteCleanFull | x === WriteEvictFull | x === WriteEvictOrEvict // is CopyBack
  def isWriUniX       (x: UInt): Bool = x === WriteUniqueFull | x === WriteUniquePtl
  def isWriXPtl       (x: UInt): Bool = x === WriteUniquePtl | x === WriteNoSnpPtl
  def isWriXFull      (x: UInt): Bool = x === WriteUniqueFull | x === WriteBackFull
  def isCMO           (x: UInt): Bool = x === CleanShared | x === CleanInvalid | x === MakeInvalid
  def isAtomicX       (x: UInt): Bool = AtomicStoreADD <= x & x <= AtomicCompare
  def isAtomicStoreX  (x: UInt): Bool = AtomicStoreADD <= x & x <= AtomicStoreUMIN
  def isReplace       (x: UInt): Bool = x === Replace
  def isFlush         (x: UInt): Bool = x === FlushDCU
  def isCombinedWrite (x: UInt): Bool = WriteNoSnpFullCleanSh <= x & x <= WriteBackFullCleanInvPoPA

  // Get Atomic type expect AmoticStoreX
  def getAtomicOp     (x: UInt): UInt = x(3, 0)
}

object RspOpcode {
  val width = 5
  val RespLCrdReturn = 0x00.U(width.W)
  val SnpResp = 0x01.U(width.W)
  val CompAck = 0x02.U(width.W)
  val RetryAck = 0x03.U(width.W)
  val Comp = 0x04.U(width.W)
  val CompDBIDResp = 0x05.U(width.W)
  val DBIDResp = 0x06.U(width.W)
  val PCrdGrant = 0x07.U(width.W)
  val ReadReceipt = 0x08.U(width.W)
  val SnpRespFwded = 0x09.U(width.W)
  val TagMatch = 0x0A.U(width.W)
  val RespSepData = 0x0B.U(width.W)
  val Persist = 0x0C.U(width.W)
  val CompPersist = 0x0D.U(width.W)
  val DBIDRespOrd = 0x0E.U(width.W)

  val CompStashDone = 0x11.U(width.W)
  val CompCMO = 0x14.U(width.W)

  def isSnpRespX(opcode: UInt): Bool = opcode === SnpResp || opcode === SnpRespFwded
}

object DatOpcode {
  val width = 4
  val DataLCrdReturn = 0x00.U(width.W)
  val SnpRespData = 0x01.U(width.W)
  val CopyBackWriteData = 0x02.U(width.W)
  val NonCopyBackWriteData = 0x03.U(width.W)
  val CompData = 0x04.U(width.W)
  val SnpRespDataPtl = 0x05.U(width.W)
  val SnpRespDataFwded = 0x06.U(width.W)
  val WriteDataCancel = 0x07.U(width.W)
  val DataSepResp = 0x0B.U(width.W)
  val NCBWrDataCompAck = 0x0C.U(width.W)

  def isSnpRespDataX(opcode: UInt): Bool = opcode === SnpRespData || opcode === SnpRespDataPtl || opcode === SnpRespDataFwded
}

object SnpOpcode {
  val width = 5
  val SnpLCrdReturn = 0x00.U(width.W)
  val SnpShared = 0x01.U(width.W)
  val SnpClean = 0x02.U(width.W)
  val SnpOnce = 0x03.U(width.W)
  val SnpNotSharedDirty = 0x04.U(width.W)
  val SnpUniqueStash = 0x05.U(width.W)
  val SnpMakeInvalidStash = 0x06.U(width.W)
  val SnpUnique = 0x07.U(width.W)
  val SnpCleanShared = 0x08.U(width.W)
  val SnpCleanInvalid = 0x09.U(width.W)
  val SnpMakeInvalid = 0x0A.U(width.W)
  val SnpStashUnique = 0x0B.U(width.W)
  val SnpStashShared = 0x0C.U(width.W)
  val SnpDVMOp = 0x0D.U(width.W)

  val SnpQuery = 0x10.U(width.W)
  val SnpSharedFwd = 0x11.U(width.W)
  val SnpCleanFwd = 0x12.U(width.W)
  val SnpOnceFwd = 0x13.U(width.W)
  val SnpNotSharedDirtyFwd = 0x14.U(width.W)
  val SnpPreferUnique = 0x15.U(width.W)
  val SnpPreferUniqueFwd = 0x16.U(width.W)
  val SnpUniqueFwd = 0x17.U(width.W)

  // Self define, only use in DongJiang internal
  val SnpUniqueEvict = 0x18.U(width.W)

  def isSnpXFwd        (x: UInt): Bool = x >= SnpSharedFwd & x =/= SnpPreferUnique

  def getNoFwdSnpOp(x: UInt): UInt = {
    val snpOp = WireInit(0.U(width.W))
    when(isSnpXFwd(x)) {
      switch(x) {
        is(SnpSharedFwd)         { snpOp := SnpShared }
        is(SnpCleanFwd)          { snpOp := SnpClean }
        is(SnpOnceFwd)           { snpOp := SnpOnce }
        is(SnpNotSharedDirtyFwd) { snpOp := SnpNotSharedDirty }
        is(SnpPreferUniqueFwd)   { snpOp := SnpPreferUnique }
        is(SnpUniqueFwd)         { snpOp := SnpUnique }
      }
    }.otherwise {
      snpOp := x
    }
    snpOp
  }
}