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
    this.shiftAddr  := ar.addr(11, 0) >> ar.size << ar.size
    this.addrMask   := byteComp(ar.len, ar.size)
    this.burst      := ar.burst
    this
  }
}
class lenValue(implicit p: Parameters) extends ZJBundle {
  val len    = UInt(8.W)
  val burst  = UInt(2.W)
}

class SpiltArValue(implicit p: Parameters) extends ZJBundle {
  val shift   = UInt(6.W)
  val size    = UInt(6.W)
}
class DataValue(implicit p: Parameters) extends ZJBundle {
  val data   = UInt(dw.W)
  val rid    = UInt(zjParams.dmaParams.idBits.W)
}

class AxiWrEntry(implicit p: Parameters) extends ZJBundle {
  val prefixAddr = UInt(36.W)
  val shiftAddr  = UInt(12.W)
  val burst      = UInt(BurstMode.width.W)
  val cnt        = UInt(8.W)
  val byteMask   = UInt(9.W)
  val size       = UInt(3.W)
  val awid       = UInt(zjParams.dmaParams.idBits.W)
  val len        = UInt(8.W)

  def byteComp(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }

  def assignAwVal[T <: AWFlit](aw : T): AxiWrEntry = {
    this.prefixAddr := aw.addr(raw - 1, 12)
    this.shiftAddr  := aw.addr(11, 0) >> aw.size << aw.size
    this.len      := aw.len + 1.U
    this.burst    := aw.burst
    this.byteMask := byteComp(aw.len, aw.size)
    this.awid     := aw.id
    this.cnt      := 0.U
    this.size     := aw.size
    this
  }
}

class SpiltAwValue(implicit p: Parameters) extends ZJBundle {
  val shift = UInt(6.W)
  val size  = UInt(6.W)
  val burst = UInt(2.W)
  val id    = UInt(zjParams.dmaParams.idBits.W)
  val last  = Bool()
}

class ChiDBPtr(chiEntrySize: Int) extends Bundle {
  val set    = UInt(log2Ceil(chiEntrySize).W)
  val poi    = UInt(1.W)
  val flag   = Bool()
  def PtrRdAdd[T <: CHIREntry](c: T): ChiDBPtr = {
    this.poi := Mux(c.double & this.poi === 0.U, 1.U, 0.U)
    this.set := Mux(!c.double | c.double & this.poi === 1.U, this.set + 1.U, this.set)
    this
  }
  def PtrWrAdd[T <: CHIWEntry](c: T): ChiDBPtr = {
    this.poi  := Mux(c.double & this.poi === 0.U, 1.U, 0.U)
    this.flag := Mux((this.set + 1.U)(log2Ceil(chiEntrySize) - 1, 0) === 0.U & this.poi === 0.U, !this.flag, this.flag)
    this.set  := Mux(!c.double | c.double & this.poi === 1.U, this.set + 1.U, this.set)
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
  val arId           = UInt(zjParams.dmaParams.idBits.W)
  val double         = Bool()
  val addr           = UInt(raw.W)
  val size           = UInt(3.W)
  val dbSite1        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val dbSite2        = UInt(log2Ceil(zjParams.dmaParams.bufferSize).W)
  val haveAllocDB    = Bool()
  val haveSendReq    = Bool()
  val haveWrDB1      = Bool()
  val haveWrDB2      = Bool()

  def ARMesInit[T <: ARFlit](b: T): CHIREntry = {
    this           := 0.U.asTypeOf(this)
    this.double    := b.len(0).asBool
    this.arId      := b.id
    this.addr      := b.addr
    this.size      := b.size
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
  val haveAllocDB    = Bool()
  val haveSendReq    = Bool()
  val haveRcvComp    = Bool()
  val haveRdData     = Bool()
  val haveRcvDBID    = Bool()

  def AWMesInit[T <: AWFlit](aw : T): CHIWEntry = {
    this           := 0.U.asTypeOf(this)
    this.double    := aw.len === 1.U
    this.awId      := aw.id
    this.size      := aw.size
    this.addr      := aw.addr
    this
  }
}
