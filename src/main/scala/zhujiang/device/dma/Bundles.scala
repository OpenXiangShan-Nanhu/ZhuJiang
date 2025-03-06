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
import zhujiang.device.dma.BurstMode.{Wrap => Wrap}

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

class AxiRdEntry(isPipe: Boolean)(implicit p: Parameters) extends ZJBundle {
  val prefixAddr   = UInt(36.W)
  val shiftAddr    = UInt(12.W)
  val endAddr      = UInt(12.W)
  val id           = UInt(zjParams.dmaParams.idBits.W)
  val byteMask     = UInt(12.W)
  val cnt          = if(!isPipe) Some(UInt(8.W)) else None
  val len          = if(isPipe) Some(UInt(8.W)) else None
  val num          = if(!isPipe) Some(UInt(8.W)) else None
  val size         = UInt(3.W)
  val burst        = UInt(2.W)
  val error        = if(!isPipe) Some(Bool()) else None

  def byteComp(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }
  def pipeInit[T <: ARFlit](ar: T): AxiRdEntry = {
    this.prefixAddr  := ar.addr(raw - 1, 12)
    this.shiftAddr   := ar.addr(11, 0)
    this.endAddr     := ((ar.addr(11, 0) >> ar.size) + (ar.len + 1.U)) << ar.size
    this.id          := ar.id
    this.len.get     := ar.len + 1.U
    this.byteMask    := Mux(ar.burst === BurstMode.Wrap, byteComp(ar.len, ar.size), 0.U) 
    this.size        := ar.size
    this.burst       := ar.burst
    this
  }

  def entryInit[T <: AxiRdEntry](info : T): AxiRdEntry = {
    this.prefixAddr   := info.prefixAddr
    this.shiftAddr    := info.shiftAddr
    this.endAddr      := info.endAddr
    this.id           := info.id
    this.byteMask     := info.byteMask
    this.cnt.get      := 0.U
    this.num.get      := Mux(info.burst === BurstMode.Incr, info.endAddr(11, 6) + info.endAddr(5, 0).orR - info.shiftAddr(11, 6), info.len.get)
    this.size         := info.size
    this.burst        := info.burst
    this.error.get    := (info.endAddr < info.shiftAddr) & info.burst === BurstMode.Incr
    this
  }
}

class dataWithid(implicit p: Parameters) extends ZJBundle {
  val data   = UInt(dw.W)
  val id     = UInt(zjParams.dmaParams.idBits.W)
  val entry  = UInt(log2Ceil(zjParams.dmaParams.chiEntrySize).W)
}
class dArEntry(implicit p: Parameters) extends ZJBundle {
  val id      = UInt(zjParams.dmaParams.idBits.W)
  val num     = UInt(6.W)
  val last    = Bool()
  val shift   = UInt(6.W)
  val size    = UInt(6.W)
  val error   = Bool()
}

class AxiWrEntry(isPipe : Boolean)(implicit p: Parameters) extends ZJBundle {
  val prefixAddr = UInt((raw - 12).W)
  val shiftAddr  = UInt(12.W)
  val endAddr    = UInt(12.W)
  val burst      = UInt(BurstMode.width.W)
  val cnt        = if(!isPipe) Some(UInt(8.W)) else None
  val byteMask   = UInt(12.W)
  val size       = UInt(3.W)
  val awid       = UInt(zjParams.dmaParams.idBits.W)
  val num        = if(!isPipe) Some(UInt(8.W)) else None
  val len        = if(isPipe) Some(UInt(8.W)) else None

  def byteComp(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }

  def pipeInit[T <: AWFlit](aw : T): AxiWrEntry = {
    this.prefixAddr     := aw.addr(raw - 1, 12)
    this.shiftAddr      := aw.addr(11, 0) >> aw.size << aw.size
    this.endAddr        := ((aw.addr(11, 0) >> aw.size) + (aw.len + 1.U)) << aw.size
    this.burst          := aw.burst
    this.byteMask       := Mux(aw.burst =/= BurstMode.Wrap, 0.U, byteComp(aw.len, aw.size))
    this.awid           := aw.id
    this.size           := aw.size
    this.len.get        := aw.len
    this
  }
  def entryInit[T <: AxiWrEntry](info : T): AxiWrEntry = {
    this.prefixAddr     := info.prefixAddr
    this.shiftAddr      := info.shiftAddr
    this.endAddr        := info.endAddr
    this.num.get        := Mux(info.burst === BurstMode.Incr, info.endAddr(11, 6) + info.endAddr(5, 0).orR - info.shiftAddr(11, 6), info.len.get + 1.U)
    this.burst          := info.burst
    this.cnt.get        := 0.U
    this.byteMask       := info.byteMask
    this.size           := info.size
    this.awid           := info.awid
    this
  }
}

class dAwEntry(implicit p: Parameters) extends ZJBundle {
  val shift     = UInt(6.W)
  val nextShift = UInt(6.W)
  val size      = UInt(6.W)
  val burst     = UInt(2.W)
  val id        = UInt(zjParams.dmaParams.idBits.W)
  val last      = Bool()
  val error     = Bool()
}

class ChiDBPtr(chiEntrySize: Int) extends Bundle {
  val set    = UInt(log2Ceil(chiEntrySize).W)
  val poi    = UInt(1.W)
  val flag   = Bool()
  def PtrRdAdd[T <: CHIREntry](c: T): ChiDBPtr = {
    this.poi  := Mux(c.double & this.poi === 0.U, 1.U, 0.U)
    this.flag := Mux((this.set + 1.U)(log2Ceil(chiEntrySize) - 1, 0) === 0.U & (this.poi === 1.U | this.poi === 0.U & !c.double), !this.flag, this.flag)
    this.set  := Mux(!c.double | c.double & this.poi === 1.U, this.set + 1.U, this.set)
    this
  }
  def PtrWrAdd[T <: CHIWEntry](c: T): ChiDBPtr = {
    this.poi  := Mux(c.double & this.poi === 0.U, 1.U, 0.U)
    this.flag := Mux((this.set + 1.U)(log2Ceil(chiEntrySize) - 1, 0) === 0.U & (this.poi === 1.U | this.poi === 0.U & !c.double), !this.flag, this.flag)
    this.set  := Mux(!c.double | c.double & this.poi === 1.U, this.set + 1.U, this.set)
    this
  }
}

class readRdDataBuffer(bufferSize: Int, axiParams: AxiParams)(implicit p: Parameters) extends ZJBundle {
  val set      = UInt(log2Ceil(bufferSize).W)
  val id       = UInt(log2Ceil(zjParams.dmaParams.chiEntrySize).W)
  val resp     = UInt(2.W)
  val originId = UInt(axiParams.idBits.W)
  val last     = Bool()

  def SetBdl[T <: CHIREntry](c: T, i: UInt): readRdDataBuffer = {
    this.id       := c.idx
    this.originId := c.arId
    this.resp     := 0.U
    this.set      := Mux(i === 1.U, c.dbSite2, c.dbSite1)
    this.last     := Mux(c.double & i === 0.U, false.B, true.B)
    this
  }
}

class readWrDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val set     = UInt(log2Ceil(bufferSize).W)
  val tgtId   = UInt(niw.W)
  val txnID   = UInt(12.W)
  val dataID  = UInt(2.W)
  val withAck = Bool()
}

class respDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val data     = UInt(dw.W)
  val id       = UInt(log2Ceil(zjParams.dmaParams.chiEntrySize).W)
  val resp     = UInt(2.W)
  val last     = Bool()
}

class CHIREntry(implicit p : Parameters) extends ZJBundle {
  val arId           = UInt(zjParams.dmaParams.idBits.W)
  val idx            = UInt(log2Ceil(zjParams.dmaParams.chiEntrySize).W)
  val double         = Bool()
  val addr           = UInt(raw.W)
  val size           = UInt(3.W)
  val nid            = UInt(log2Ceil(zjParams.dmaParams.chiEntrySize).W)
  val dbSite1        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val dbSite2        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val haveWrDB1      = Bool()
  val haveWrDB2      = Bool()
  val sendComp       = Bool()
  val fromDCT        = Bool()
  val rcvDatComp     = Bool()

  def ARMesInit[T <: ARFlit](b: T): CHIREntry = {
    this           := 0.U.asTypeOf(this)
    this.double    := b.len(0).asBool
    this.arId      := b.user
    this.idx       := b.id
    this.addr      := b.addr
    this.size      := b.size
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
    this.Size     := Mux(c.double, 6.U, c.size)
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
  val size           = UInt(3.W)
  val tgtid          = UInt(niw.W)
  val addr           = UInt(raw.W)
  val dbid           = UInt(12.W)
  val dbSite1        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val dbSite2        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val haveRcvComp    = Bool()

  def AWMesInit[T <: AWFlit](aw : T): CHIWEntry = {
    this           := 0.U.asTypeOf(this)
    this.double    := aw.len === 1.U
    this.awId      := aw.id
    this.size      := aw.size
    this.addr      := aw.addr
    this
  }
}
