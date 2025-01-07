package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.cde.config.Parameters
import chisel3.util.RRArbiter
import xs.utils.ResetRRArbiter
import zhujiang._
import zhujiang.chi._
import zhujiang.axi._
import xijiang.{Node, NodeType}
import zhujiang.chi.ReqOpcode.AtomicLoadUMIN

object DmaDef {
    def RREncoder(in: Seq[Bool]): UInt = {
        val arb = Module(new ResetRRArbiter(UInt(log2Ceil(in.size).W), in.size))
        arb.io.in.zipWithIndex.foreach {
            case(a, i) =>
                a.valid := in(i)
                a.bits  := i.U
        }
        arb.io.out.ready := true.B
        arb.io.out.bits
    }
}


object BurstMode {
  val width        = 2
  val Fix          = "b00".U
  val Incr         = "b01".U
  val Wrap         = "b10".U
  val Reserve      = "b11".U
}

class AxiRdEntry(implicit p: Parameters) extends ZJBundle {
  val prefixAddr   = UInt(36.W)
  val shiftAddr    = UInt(12.W)
  val addrMask     = UInt(12.W)
  val arId         = UInt(zjParams.dmaParams.idBits.W)
  val cnt          = UInt(8.W)
  val len          = UInt(8.W)
  val size         = UInt(3.W)
  val burst        = UInt(2.W)

  def byteComp(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }

  def assignArVal[T <: ARFlit](ar : T): AxiRdEntry = {
    this.arId   := ar.id
    this.len    := ar.len + 1.U
    this.cnt    := 0.U
    this.size   := ar.size
    this.prefixAddr := ar.addr(raw - 1, 12)
    this.shiftAddr  := ar.addr(11, 0)
    this.addrMask   := byteComp(ar.len, ar.size)
    this.burst      := ar.burst
    this
  }
}
class lenValue(implicit p: Parameters) extends ZJBundle {
  val len    = UInt(8.W)
  val burst  = UInt(2.W)
}

class SpiltValue(implicit p: Parameters) extends ZJBundle {
  val shift   = UInt(6.W)
  val size    = UInt(6.W)
}
class DataValue(implicit p: Parameters) extends ZJBundle {
  val data   = UInt(dw.W)
  val rid    = UInt(zjParams.dmaParams.idBits.W)
}


class AllocWrChiReq(implicit p: Parameters) extends ZJBundle {
  val addr         = UInt(raw.W)
  val sendDataCnts = UInt(4.W)
  val sendDataCnt  = UInt(4.W)
  val size         = UInt(3.W)
  val double       = Bool()  //alloc databuffer 
  val id           = UInt(zjParams.dmaParams.idBits.W)
  val last         = Bool()

  def chiWrReqAssign[T <: AXIWEntry](a : T): AllocWrChiReq = {
    this.addr     := a.addr
    this.double   := Mux(!a.addr(5) & a.burst === BurstMode.Incr & a.cnt =/= a.num, true.B, false.B)
    this.last     := Mux(this.double, a.cnt + 1.U === a.num, a.cnt === a.num)
    this.sendDataCnts := Mux(a.addr(raw - 1), 1.U, 1.U << log2Ceil(dw / 8) >> a.size)
    this.sendDataCnt  := a.shift >> a.size
    this.id           := a.awid
    this.size         := a.size
    this
  }
}



class AXIWEntry(implicit p: Parameters) extends ZJBundle {
  val addr       = UInt(raw.W)
  val shift      = UInt(5.W)
  val burst      = UInt(BurstMode.width.W)
  val num        = UInt(8.W)
  val cnt        = UInt(8.W)
  val byteMask   = UInt(9.W)
  val size       = UInt(3.W)
  val awid       = UInt(zjParams.dmaParams.idBits.W)

  def byteComp(len:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << log2Ceil(bew).U) >> maxShift
  }

  def axiWEntryInit[T <: AWFlit](aw : T): AXIWEntry = {
    this.addr    := aw.addr //& (~31.U(raw.W))
    this.shift   := aw.addr(4, 0)
    this.burst   := aw.burst
    this.byteMask := byteComp(aw.len)
    this.num      := aw.len << aw.size >> log2Ceil(dw / 8)
    this.awid     := aw.id
    this.cnt      := 0.U
    this.size     := aw.size
    this
  }
}

class ChiDBPtr(chiEntrySize: Int) extends Bundle {
  val set    = UInt(log2Ceil(chiEntrySize).W)
  val poi    = UInt(1.W)
  def PtrRdAdd[T <: CHIREntry](c: T): ChiDBPtr = {
    this.poi := Mux(c.double & this.poi === 0.U, 1.U, 0.U)
    this.set := Mux(!c.double | c.double & this.poi === 1.U, this.set + 1.U, this.set)
    this
  }
  def PtrWrAdd[T <: CHIWEntry](c: T): ChiDBPtr = {
    this.poi := Mux(c.double & this.poi === 0.U, 1.U, 0.U)
    this.set := Mux(!c.double | c.double & this.poi === 1.U, this.set + 1.U, this.set)
    this
  }
}

class readRdDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val set      = UInt(log2Ceil(bufferSize).W)
  val id       = UInt(zjParams.dmaParams.idBits.W)
  val resp     = UInt(2.W)
  val last     = Bool()

  def setBdl[T <: CHIREntry, R <: ChiDBPtr](c: T, i: R): readRdDataBuffer = {
    this.id       := c.arId
    this.resp     := 0.U
    this.set      := Mux(i.poi === 1.U, c.dbSite2, c.dbSite1)
    this.last     := Mux(c.double & i.poi === 0.U, false.B, true.B)
    this
  }
}

class readWrDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val set    = UInt(log2Ceil(bufferSize).W)
  val tgtId  = UInt(niw.W)
  val txnID  = UInt(12.W)
  val dataID = UInt(2.W)
}

class respDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val data = UInt(dw.W)
  val id       = UInt(zjParams.dmaParams.idBits.W)
  val resp     = UInt(2.W)
  val last     = Bool()
}

class CHIREntry(implicit p : Parameters) extends ZJBundle {
  val arId           = UInt(log2Ceil(zjParams.dmaParams.axiEntrySize).W)
  val double         = Bool()
  val addr           = UInt(raw.W)
  val size           = UInt(3.W)
  val dbSite1        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val dbSite2        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val haveAllocDB    = Bool()
  val haveSendReq    = Bool()
  val haveWrDB1      = Bool()
  val haveWrDB2      = Bool()

  def chiREntryInit[T <: ARFlit](b: T): CHIREntry = {
    this.haveAllocDB      := false.B
    this.haveSendReq      := false.B
    this.haveWrDB1        := false.B
    this.haveWrDB2        := false.B
    this.dbSite1          := 0.U
    this.dbSite2          := 0.U
    this.double           := b.len(0).asBool
    this.arId             := b.id
    this.addr             := b.addr
    this.size             := b.size
    this
  }
  def chiRAllocDB[T <: ChiDataBufferCtrlEntry](b: T): CHIREntry = {
    this.haveAllocDB := true.B
    this.dbSite1     := b.buf(0)
    this.dbSite2     := b.buf(1)
    this
  }
}

class DmaReqFlit(implicit p : Parameters) extends ReqFlit {
  def RReqInit[T <: CHIREntry](c : T, i : UInt): ReqFlit = {
    this          := 0.U.asTypeOf(this)
    this.Addr     := c.addr
    this.Opcode   := Mux(c.addr(raw - 1), ReqOpcode.ReadNoSnp, ReqOpcode.ReadOnce)
    this.SrcID    := 1.U
    this.Order    := "b11".U
    this.TxnID    := i
    this.Size     := Mux(c.double, 6.U, c.size)
    this
  }
  def WReqInit[T <: CHIWEntry](c : T, i : UInt): ReqFlit = {
    this          := 0.U.asTypeOf(this)
    this.Addr     := c.addr
    this.Opcode   := Mux(c.addr(raw - 1), ReqOpcode.WriteNoSnpPtl, ReqOpcode.WriteUniquePtl)
    this.Size     := Mux(c.rcvDataCnts === 1.U, c.size, Mux(c.double, 6.U, 5.U))
    this.TxnID    := i
    this.ExpCompAck := true.B
    this.Order      := "b10".U
    this.SrcID      := 1.U
    this
  }
}

class CHIWEntry(implicit p: Parameters) extends ZJBundle {
  val awId           = UInt(log2Ceil(zjParams.dmaParams.axiEntrySize).W)
  val double         = Bool()
  val rcvDataCnts    = UInt(4.W)
  val cnt            = UInt(4.W)
  val size           = UInt(3.W)
  val tgtid          = UInt(niw.W)
  val addr           = UInt(raw.W)
  val dbid           = UInt(12.W)
  val dbSite1        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val dbSite2        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val last           = Bool()
  val haveAllocDB    = Bool()
  val haveSendReq    = Bool()
  val haveRcvComp    = Bool()

  def chiWEntryInit[T <: AllocWrChiReq](b: T): CHIWEntry = {
    this.haveAllocDB    := false.B
    this.haveSendReq    := false.B
    this.haveRcvComp    := false.B
    this.cnt            := b.sendDataCnt
    this.size           := b.size
    this.dbSite1        := 0.U
    this.dbSite2        := 0.U
    this.last           := b.last
    this.rcvDataCnts    := b.sendDataCnts
    this.double         := b.double
    this.awId           := b.id
    this.addr           := b.addr
    this
  }
  def wPtrAdd[T <: WFlit, R <: ChiDBPtr](w : T, r : R) : CHIWEntry = {
    this.cnt            := Mux(this.cnt + 1.U === this.rcvDataCnts, 0.U, this.cnt + 1.U)
    r.set               := Mux(this.cnt + 1.U === this.rcvDataCnts & r.poi === 1.U | this.cnt + 1.U === this.rcvDataCnts & !this.double | w.last, r.set + 1.U, r.set)
    r.poi               := Mux(this.cnt + 1.U === this.rcvDataCnts & r.poi === 1.U | w.last, 0.U, Mux(this.cnt + 1.U === this.rcvDataCnts & this.double & r.poi === 0.U, 1.U, r.poi))
    this
  }
}
