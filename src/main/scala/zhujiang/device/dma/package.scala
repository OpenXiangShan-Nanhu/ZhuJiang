package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import zhujiang.{ZJBundle, ZJModule}
import xs.utils.FastArbiter
import zhujiang.axi._
import zhujiang.chi._


object Burst {
  def FIX   = "b00".U
  def INCR  = "b01".U
  def WRAP  = "b10".U
  def RSV   = "b11".U

  def isFix(burst: UInt): Bool = {
    !(burst.orR)
  }
  def isIncr(burst: UInt): Bool = {
    burst(0).asBool
  }
  def isWrap(burst: UInt): Bool = {
    burst(1).asBool
  }
}

case class DmaParams(
  pageBits  : Int = 12,
  offset    : Int = 6,
  idBits    : Int = 12,
  nrBeats   : Int = 2,
  chiEntrySize  : Int = 32,
  axiEntrySize  : Int = 8,
  dbEntrySize   : Int = 32,
  readDMT       : Boolean = true,
  rniID         : Int = 1,
) 


class AxiRdEntry(isPipe: Boolean)(implicit P: Parameters) extends ZJBundle {
  private val rni = zjParams.dmaParams
  val preAddr  = UInt((raw - rni.pageBits).W)
  val exAddr   = UInt(rni.pageBits.W)
  val endAddr  = UInt(rni.pageBits.W)
  val id       = UInt(rni.idBits.W)
  val byteMask = UInt(rni.pageBits.W)
  val len      = UInt(8.W)
  val firstTxn = if(!isPipe) Some(UInt(log2Ceil(dw/8).W)) else None
  val lastTxn  = if(!isPipe) Some(UInt(log2Ceil(dw/8).W)) else None
  val cnt      = if(!isPipe) Some(UInt(8.W))              else None
  val range    = if(isPipe)  Some(UInt(rni.pageBits.W))   else None
  val num      = if(!isPipe) Some(UInt(8.W))              else None
  val size     = UInt(3.W)
  val cache    = UInt(4.W)
  val burst    = UInt(2.W)

  def wrapMask(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }
  def pipeInit[T <: ARFlit](ar: T): AxiRdEntry = {
    this.preAddr    := ar.addr(raw - 1, rni.pageBits)
    this.exAddr     := ar.addr(rni.pageBits - 1, 0) >> ar.size << ar.size
    this.endAddr    := ((ar.addr(rni.pageBits - 1, 0) >> ar.size) + (ar.len + 1.U)) << ar.size
    this.id         := ar.id
    this.byteMask   := Mux(Burst.isWrap(ar.burst), wrapMask(ar.len, ar.size), 0.U)
    this.len        := ar.len + 1.U
    this.range.get  := (ar.len + 1.U) << ar.size
    this.size       := ar.size
    this.cache      := ar.cache
    this.burst      := ar.burst
    this
  }
  def getNum(modify: Bool, range: UInt, endAddr: UInt, exAddr: UInt, len: UInt, burst: UInt): UInt = {
    val canNotMerge   = !modify
     val fixMerge      = Burst.isFix(burst)  & modify
     val wrapMerge     = Burst.isWrap(burst) & modify
     val incrMerge     = Burst.isIncr(burst) & modify
     //number compute
     val wrapMergeNum  = Mux(range(rni.pageBits - 1, rni.offset).orR, range(rni.pageBits - 1, rni.offset), 1.U)
     val incrMergeNum  = endAddr(rni.pageBits - 1, rni.offset) + endAddr(rni.offset - 1, 0).orR - exAddr(rni.pageBits - 1, rni.offset)
     val fixMergeNum   = 1.U
     PriorityMux(Seq(
      canNotMerge     -> len,
      fixMerge        -> fixMergeNum,
      wrapMerge       -> wrapMergeNum,
      incrMerge       -> incrMergeNum
     ))
  }  
  def entryInit[T <: AxiRdEntry](info: T): AxiRdEntry = {
    this.preAddr      := info.preAddr
    this.exAddr       := info.exAddr
    this.endAddr      := info.endAddr
    this.id           := info.id
    this.byteMask     := info.byteMask
    this.cnt.get      := 0.U
    this.num.get      := getNum(info.cache(1).asBool, info.range.get, info.endAddr, info.exAddr, info.len, info.burst)
    this.size         := info.size
    this.len          := info.len
    this.cache        := info.cache
    this.burst        := info.burst
    this.firstTxn.get := ("b100000".U - info.exAddr(rni.offset - 1, 0)) >> info.size
    this.lastTxn.get  := info.endAddr(rni.offset - 1, 0) >> info.size
    this
  }
}

class AxiWrEntry(isPipe : Boolean)(implicit p: Parameters) extends ZJBundle {
  private val rni = zjParams.dmaParams
  val preAddr    = UInt((raw - rni.pageBits).W)
  val exAddr     = UInt(rni.pageBits.W)
  val endAddr    = UInt(rni.pageBits.W)
  val byteMask   = UInt(rni.pageBits.W)
  val burst      = UInt(2.W)
  val cnt        = if(!isPipe) Some(UInt(8.W))             else None
  val num        = if(!isPipe) Some(UInt(8.W))             else None
  val len        = if(isPipe)  Some(UInt(8.W))             else None
  val range      = if(isPipe)  Some(UInt(rni.pageBits.W))  else None
  val size       = UInt(3.W)
  val cache      = UInt(4.W)
  val id         = UInt(rni.idBits.W)

  def byteComp(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }

  def pipeInit[T <: AWFlit](aw : T): AxiWrEntry = {
    this.preAddr        := aw.addr(raw - 1, rni.pageBits)
    this.exAddr         := aw.addr(rni.pageBits - 1, 0) >> aw.size << aw.size
    this.endAddr        := ((aw.addr(rni.pageBits - 1, 0) >> aw.size) + (aw.len + 1.U)) << aw.size
    this.range.get      := (aw.len + 1.U) << aw.size
    this.burst          := aw.burst
    this.byteMask       := Mux(Burst.isWrap(aw.burst), byteComp(aw.len, aw.size), 0.U)
    this.id             := aw.id
    this.size           := aw.size
    this.len.get        := aw.len + 1.U
    this.cache          := aw.cache
    this
  }
  def getNum(modify: Bool, exAddr: UInt, len: UInt, burst: UInt, range: UInt, endAddr: UInt): UInt = {
     val canNotMerge   = !modify
     val fixMerge      = Burst.isFix(burst)  & modify
     val wrapMerge     = Burst.isWrap(burst) & modify
     val incrMerge     = Burst.isIncr(burst) & modify
     //number compute
     val wrapMergeNum  = Mux(range(rni.pageBits - 1, rni.offset).orR, range(rni.pageBits - 1, rni.offset), 1.U)
     val incrMergeNum  = endAddr(rni.pageBits - 1, rni.offset) + endAddr(rni.offset - 1, 0).orR - exAddr(rni.pageBits - 1, rni.offset)
     val fixMergeNum   = 1.U
     PriorityMux(Seq(
      canNotMerge     -> len,
      fixMerge        -> fixMergeNum,
      wrapMerge       -> wrapMergeNum,
      incrMerge       -> incrMergeNum
     ))
  }
  def entryInit[T <: AxiWrEntry](info : T): AxiWrEntry = {
    this.preAddr        := info.preAddr
    this.exAddr         := info.exAddr
    this.endAddr        := info.endAddr
    this.num.get        := getNum(info.cache(1), info.exAddr, info.len.get, info.burst, info.range.get, info.endAddr)
    this.burst          := info.burst
    this.cnt.get        := 0.U
    this.byteMask       := info.byteMask
    this.size           := info.size
    this.cache          := info.cache
    this.id             := info.id
    this
  }
}


class AxiRMstEntry(implicit p: Parameters) extends ZJBundle {
  private val rni = zjParams.dmaParams
  val id          = UInt(rni.idBits.W)
  val last        = Bool()
  val shift       = UInt(rni.offset.W)
  val size        = UInt(6.W)
  val txns        = UInt(log2Ceil(dw/8).W)
}

class DataCtrl(implicit p: Parameters) extends ZJBundle {
  private val rni = zjParams.dmaParams
  val data   = UInt(dw.W)
  val id     = UInt(rni.idBits.W)
  val idx    = UInt(log2Ceil(rni.chiEntrySize).W)
  val resp   = UInt(2.W)
}

class CHIREntry(dmt : Boolean)(implicit p : Parameters) extends ZJBundle {
  private val rni    = zjParams.dmaParams
  val arId           = UInt(rni.idBits.W)
  val idx            = UInt(log2Ceil(rni.chiEntrySize).W)
  val double         = Bool()
  val addr           = UInt(raw.W)
  val size           = UInt(3.W)
  val nid            = UInt(log2Ceil(rni.chiEntrySize).W)
  val dbSite1        = UInt(log2Ceil(rni.dbEntrySize).W)
  val dbSite2        = UInt(log2Ceil(rni.dbEntrySize).W)
  val memAttr        = UInt(4.W)
  val haveRcvRct     = Bool()
  val haveWrDB1      = Bool()
  val haveWrDB2      = Bool()
  val sendComp       = Bool()
  val fromDCT        = Bool()
  val rcvDatComp     = Bool()
  val haveSendAck    = if(dmt) Some(Bool())      else None
  val homeNid        = if(dmt) Some(UInt(niw.W)) else None
  val dbid           = if(dmt) Some(UInt(12.W))  else None

  def ARMesInit[T <: ARFlit](b: T): CHIREntry = {
    this                  := 0.U.asTypeOf(this)
    this.double           := b.len(0).asBool
    this.arId             := b.user
    this.idx              := b.id
    this.addr             := b.addr
    this.size             := b.size
    this.haveRcvRct       := Mux(b.cache(1), 1.U, 0.U)
    this.memAttr          := Cat(b.cache(2), b.cache(3) | b.cache(2), !b.cache(1) & !b.cache(2) & !b.cache(3), b.cache(0) | b.cache(2) | b.cache(3))
    this.haveSendAck.get  := Mux(!b.cache(1) & !b.cache(2) & !b.cache(3), true.B, false.B)
    this
  }
}


class readRdDataBuffer(bufferSize: Int, axiParams: AxiParams)(implicit p: Parameters) extends ZJBundle {
  val set      = UInt(log2Ceil(bufferSize).W)
  val id       = UInt(log2Ceil(zjParams.dmaParams.chiEntrySize).W)
  val resp     = UInt(2.W)
  val originId = UInt(axiParams.idBits.W)
  val last     = Bool()

  def SetBdl[T <: RdDBEntry](c: T, i: UInt): readRdDataBuffer = {
    this.id       := c.idx
    this.originId := c.arID
    this.resp     := 0.U
    this.set      := Mux(i === 1.U, c.dbSite2, c.dbSite1)
    this.last     := Mux(c.double & i === 0.U, false.B, true.B)
    this
  }
}

class DataBufferAlloc(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  private val rni  = zjParams.dmaParams
  val buf          = Vec(2, UInt(log2Ceil(bufferSize).W))
  val num          = UInt(2.W)
}

class DataBufferTxReq(ctrlSize: Int)(implicit p: Parameters) extends ZJBundle {
  val idxOH = UInt(ctrlSize.W)
  val flit  = new DataFlit
}

class writeRdDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val set    = UInt(log2Ceil(bufferSize).W)
  val data   = UInt(dw.W)
}

class writeWrDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val set   = UInt(log2Ceil(bufferSize).W)
  val data  = UInt(dw.W)
  val mask  = UInt(bew.W)
}
class respDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val data     = UInt(dw.W)
  val id       = UInt(log2Ceil(zjParams.dmaParams.chiEntrySize).W)
  val resp     = UInt(2.W)
  val last     = Bool()
}
class readWrDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val set     = UInt(log2Ceil(bufferSize).W)
  val tgtId   = UInt(niw.W)
  val txnID   = UInt(12.W)
  val dataID  = UInt(2.W)
  val withAck = Bool()
}

class DmaReqFlit(implicit p : Parameters) extends ReqFlit {
  def RReqInit[T <: CHIREntry](c : T, i : UInt): ReqFlit = {
    this          := 0.U.asTypeOf(this)
    this.Addr     := c.addr
    this.Opcode   := Mux(c.memAttr(1), ReqOpcode.ReadNoSnp, ReqOpcode.ReadOnce)
    this.SrcID    := zjParams.dmaParams.rniID.U
    this.Order    := Mux(c.memAttr(1), "b11".U, "b00".U)
    this.TxnID    := i
    this.Size     := Mux(c.double, 6.U, c.size)
    this.MemAttr  := c.memAttr
    this.SnpAttr  := Mux(c.memAttr(1), 0.U, 1.U)
    if(zjParams.dmaParams.readDMT){
      this.ExpCompAck := Mux(c.memAttr(1), false.B, true.B)
    } else {this.ExpCompAck := false.B}
    this
  }
  def wReqInit[T <: CHIWEntry](c : T, i : UInt): ReqFlit = {
    this            := 0.U.asTypeOf(this)
    this.Addr       := c.addr
    this.Opcode     := Mux(c.memAttr(1), ReqOpcode.WriteNoSnpPtl, ReqOpcode.WriteUniquePtl)
    this.Size       := Mux(c.double, 6.U, c.size)
    this.TxnID      := i
    this.MemAttr    := c.memAttr
    this.ExpCompAck := true.B
    this.Order      := "b10".U
    this.SrcID      := zjParams.dmaParams.rniID.U
    this.SnpAttr    := Mux(c.memAttr(1), 0.U, 1.U)
    this
  }
}
class DmaRspFlit(implicit p: Parameters) extends RespFlit {
  def compAckInit[T <: CHIREntry](c : T) : RespFlit = {
    this.Opcode     := RspOpcode.CompAck
    this.TxnID      := c.dbid.get
    this.TgtID      := c.homeNid.get
    this.SrcID      := zjParams.dmaParams.rniID.U
    this
  }
}

class AxiWMstEntry(implicit p: Parameters) extends ZJBundle {
  val shift     = UInt(6.W)
  val nextShift = UInt(6.W)
  val size      = UInt(6.W)
  val burst     = UInt(2.W)
  val id        = UInt(zjParams.dmaParams.idBits.W)
  val last      = Bool()
  val dontMerge = Bool()
}

class CHIWEntry(implicit p: Parameters) extends ZJBundle {
  private val rni    = zjParams.dmaParams
  val awId           = UInt(log2Ceil(rni.axiEntrySize).W)
  val double         = Bool()
  val size           = UInt(3.W)
  val memAttr        = UInt(4.W)
  val tgtid          = UInt(niw.W)
  val addr           = UInt(raw.W)
  val dbid           = UInt(12.W)
  val dbSite1        = UInt(log2Ceil(rni.dbEntrySize).W)
  val dbSite2        = UInt(log2Ceil(rni.dbEntrySize).W)
  val haveRcvComp    = Bool()

  def awMesInit[T <: AWFlit](aw : T): CHIWEntry = {
    this           := 0.U.asTypeOf(this)
    this.double    := aw.len(0).asBool
    this.memAttr   := Cat(aw.cache(3), aw.cache(2) | aw.cache(3), !aw.cache(1) & !aw.cache(2) & !aw.cache(3), aw.cache(0) | aw.cache(2) | aw.cache(3))
    this.awId      := aw.id
    this.size      := aw.size
    this.addr      := aw.addr
    this
  }
}

class RdDBEntry(implicit p: Parameters) extends ZJBundle {
  private val rni   = zjParams.dmaParams
  val arID          = UInt(rni.idBits.W)
  val idx           = UInt(rni.chiEntrySize.W)
  val double        = Bool()
  val dbSite1       = UInt(log2Ceil(rni.dbEntrySize).W)
  val dbSite2       = UInt(log2Ceil(rni.dbEntrySize).W)

  def rdDBInit[T <: CHIREntry](b: T): RdDBEntry = {
    this.arID    := b.arId
    this.idx     := b.idx
    this.double  := b.double
    this.dbSite1 := b.dbSite1
    this.dbSite2 := b.dbSite2
    this
  }
}