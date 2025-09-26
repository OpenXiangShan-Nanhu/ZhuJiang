package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.router.base.DeviceIcnBundle
import zhujiang.{ZJBundle, ZJModule, ZJParametersKey}
import xs.utils.FastArbiter
import zhujiang.axi._
import zhujiang.chi._
import xijiang.Node
import freechips.rocketchip.amba.ahb.AHBImpMaster.bundle


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
  axiEntrySize  : Int = 2,
  readDMT       : Boolean = true,
  rniID         : Int = 1,
  node          : Node,
  offset        : Int = 6,
  matchSet      : Int = 32,
  cacheBytes    : Int = 64
){
  lazy val pageBits  = 12
  lazy val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  lazy val idBits    = axiParams.idBits
  lazy val cacheBits = axiParams.cacheBits
  lazy val qosBits   = axiParams.qosBits
  lazy val sizeBits  = axiParams.sizeBits
  lazy val lenBits   = axiParams.lenBits
  lazy val cntBits   = log2Ceil(4096 / cacheBytes) + 1
  lazy val bufferSize = node.outstanding * 2

}
class Finish(node : Node)(implicit P: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val streamID = UInt(log2Ceil(node.outstanding).W)
  val idx      = UInt(log2Ceil(node.outstanding).W)
}


class AxiRdEntry(isPipe: Boolean, node : Node)(implicit P: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val preAddr  = UInt((raw - rni.pageBits).W)
  val exAddr   = UInt(rni.pageBits.W)
  val endAddr  = UInt(rni.pageBits.W)
  val qos      = UInt(4.W)
  val id       = UInt(rni.idBits.W)
  val byteMask = UInt(rni.pageBits.W)
  val len      = UInt(9.W)
  val cnt      = if(!isPipe) Some(UInt(rni.cntBits.W))                else None
  val range    = if(isPipe)  Some(UInt(rni.pageBits.W))               else None
  val num      = if(!isPipe) Some(UInt(rni.cntBits.W))                else None
  val streamID = if(!isPipe) Some(UInt(log2Ceil(node.outstanding).W)) else None
  val size     = UInt(3.W)
  val cache    = UInt(4.W)
  val burst    = UInt(2.W)
  val spLast   = Bool()

  def wrapMask(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }
  def pipeInit[T <: ARFlit](ar: T): AxiRdEntry = {
    this.preAddr    := ar.addr(raw - 1, rni.pageBits)
    this.exAddr     := ar.addr(rni.pageBits - 1, 0)
    this.endAddr    := this.exAddr + this.range.get
    this.qos        := ar.qos
    this.id         := ar.id
    this.byteMask   := wrapMask(ar.len, ar.size)
    this.len        := ar.len +& 1.U(1.W)
    this.range.get  := (ar.len +& 1.U(1.W)) << ar.size
    this.size       := ar.size
    this.cache      := ar.cache
    this.burst      := ar.burst
    this.spLast     := ar.user.asBool
    this
  }
  def getNum(range: UInt, endAddr: UInt, exAddr: UInt, len: UInt, burst: UInt): UInt = {
     val fixMerge      = Burst.isFix(burst)
     val wrapMerge     = Burst.isWrap(burst)
     val incrMerge     = Burst.isIncr(burst)
     //number compute
     val wrapMergeNum  = Mux(range(rni.pageBits - 1, rni.offset + 1).orR, Mux(exAddr(rni.offset - 1, 0).orR, range(rni.pageBits - 1, rni.offset) + 1.U, range(rni.pageBits - 1, rni.offset)), 1.U)
     val incrMergeNum  = endAddr(rni.pageBits - 1, rni.offset) - exAddr(rni.pageBits - 1, rni.offset) + endAddr(rni.offset - 1, 0).orR 
     val fixMergeNum   = 1.U
     PriorityMux(Seq(
      fixMerge        -> fixMergeNum,
      wrapMerge       -> wrapMergeNum,
      incrMerge       -> incrMergeNum
     ))
  }  
  def entryInit[T <: AxiRdEntry](info: T, streamId: UInt): AxiRdEntry = {
    this.preAddr      := info.preAddr
    this.exAddr       := Mux(Burst.isFix(info.burst) & info.cache(1), Cat(info.exAddr(rni.pageBits - 1, rni.offset - 1), 0.U((rni.offset - 1).W)), info.exAddr)
    this.endAddr      := Mux(Burst.isWrap(info.burst), info.exAddr, Mux(Burst.isFix(info.burst) & info.cache(1), Cat(info.exAddr(rni.pageBits - 1, rni.offset - 1), info.len(rni.offset - 2, 0)), info.endAddr))
    this.qos          := info.qos
    this.id           := info.id
    this.byteMask     := Mux(Burst.isWrap(info.burst), info.byteMask, Mux(Burst.isIncr(info.burst), 0xFFF.U, Mux(Burst.isFix(info.burst) & info.cache(1), 0x1F.U, 0.U)))
    this.cnt.get      := 0.U
    this.num.get      := Mux(info.cache(1), getNum(info.range.get, info.endAddr, info.exAddr, info.len, info.burst), 1.U)
    this.size         := Mux(Burst.isFix(info.burst) & info.cache(1), 0.U, info.size)
    this.len          := info.len
    this.cache        := info.cache
    this.burst        := info.burst
    this.spLast       := info.spLast
    this.streamID.get := streamId
    this
  }
}

class AxiWrEntry(isPipe : Boolean, node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val preAddr     = UInt((raw - rni.pageBits).W)
  val exAddr      = UInt(rni.pageBits.W)
  val endAddr     = UInt(rni.pageBits.W)
  val byteMask    = UInt(rni.pageBits.W)
  val qos         = UInt(4.W)
  val burst       = UInt(2.W)
  val cnt         = if(!isPipe) Some(UInt(rni.cntBits.W))   else None
  val num         = if(!isPipe) Some(UInt(rni.cntBits.W))   else None
  val len         = if(isPipe)  Some(UInt(9.W))             else None
  val range       = if(isPipe)  Some(UInt(rni.pageBits.W))  else None
  val size        = UInt(3.W)
  val cache       = UInt(4.W)
  val id          = UInt(rni.idBits.W)
  val streamId    = UInt(log2Ceil(node.outstanding).W)
  val spLast      = Bool()

  def byteComp(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }

  def pipeInit[T <: AWFlit](aw : T): AxiWrEntry = {
    this.preAddr        := aw.addr(raw - 1, rni.pageBits)
    this.exAddr         := aw.addr(rni.pageBits - 1, 0)
    this.endAddr        := this.range.get + this.exAddr
    this.range.get      := (aw.len +& 1.U(1.W)) << aw.size
    this.burst          := aw.burst
    this.qos            := aw.qos
    this.byteMask       := byteComp(aw.len, aw.size)
    this.id             := aw.id
    this.size           := aw.size
    this.len.get        := aw.len +& 1.U(1.W)
    this.cache          := aw.cache
    this.spLast         := aw.user.asBool
    this
  }
  def getNum(exAddr: UInt, len: UInt, burst: UInt, range: UInt, endAddr: UInt): UInt = {
     val fixMerge      = Burst.isFix(burst)
     val wrapMerge     = Burst.isWrap(burst)
     val incrMerge     = Burst.isIncr(burst)
     //number compute
     val wrapMergeNum  = Mux(range(rni.pageBits - 1, rni.offset + 1).orR, Mux(exAddr(rni.offset - 1, 0).orR, range(rni.pageBits - 1, rni.offset) + 1.U, range(rni.pageBits - 1, rni.offset)), 1.U)
     val incrMergeNum  = endAddr(rni.pageBits - 1, rni.offset) + endAddr(rni.offset - 1, 0).orR - exAddr(rni.pageBits - 1, rni.offset)
     val fixMergeNum   = 1.U
     PriorityMux(Seq(
      fixMerge        -> fixMergeNum,
      wrapMerge       -> wrapMergeNum,
      incrMerge       -> incrMergeNum
     ))
  }
  def entryInit[T <: AxiWrEntry](info : T, streamId: UInt): AxiWrEntry = {
    this.preAddr        := info.preAddr
    this.exAddr         := info.exAddr
    this.endAddr        := Mux(Burst.isWrap(info.burst), info.exAddr, info.endAddr)
    this.qos            := info.qos
    this.num.get        := Mux(info.cache(1), getNum(info.exAddr, info.len.get, info.burst, info.range.get, info.endAddr), 1.U)
    this.burst          := info.burst
    this.cnt.get        := 0.U
    this.byteMask       := Mux(Burst.isIncr(info.burst), 0xFFF.U, Mux(Burst.isWrap(info.burst), info.byteMask, 0.U))
    this.size           := info.size
    this.cache          := info.cache
    this.id             := info.id
    this.spLast         := info.spLast
    this.streamId       := streamId
    this
  }
}

class AxiRMstEntry(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val id          = UInt(rni.idBits.W)
  val last        = Bool()
  val nextShift   = UInt(rni.offset.W)
  val endShift    = UInt(rni.offset.W)
  val byteMask    = UInt(rni.offset.W)
  val size        = UInt(6.W)
  val beat        = UInt(1.W)
  val outBeat     = UInt(1.W)
  val noMerge     = Bool()
  val valid       = Bool()
  val subValid    = Bool()
}

class RdState extends Bundle {
  val sendReq       = Bool()
  val rcvData0      = Bool()
  val rcvData1      = Bool()
  val rcvDataCmp    = Bool()
  val rcvRct        = Bool()
  val sendData      = Bool()
  val sendAck       = Bool()
}

class CHIRdEntry(node: Node)(implicit p : Parameters) extends ZJBundle {
  private val rni   = DmaParams(node = node)
  val arId          = UInt(rni.idBits.W)
  val eId           = UInt(log2Ceil(node.outstanding).W)
  val size          = UInt(3.W)
  val qos           = UInt(4.W)
  val fullSize      = Bool()
  val fromDCT       = Bool()
  val addr          = UInt(raw.W)
  val memAttr       = new MemAttr
  val homeNid       = UInt(niw.W)
  val dbid          = UInt(12.W)
  val dbSite1       = UInt(log2Ceil(rni.bufferSize).W)
  val dbSite2       = UInt(log2Ceil(rni.bufferSize).W)
  val respErr       = UInt(2.W)
  val reqNid        = UInt(log2Ceil(node.outstanding).W)
  val ackNid        = UInt(log2Ceil(node.outstanding).W)
  val finishNid     = UInt(log2Ceil(node.outstanding).W)
  val state         = new RdState
  val valid         = Bool()
  val complete      = Bool()


  def isToOrder     = valid && !state.rcvRct
  def shodSendReq   = valid && !state.sendReq
  def shodSendData  = valid && state.rcvDataCmp && !state.sendData
  def shodSendAck   = valid && !state.sendAck && state.rcvDataCmp
  def allTaskComp   = valid && state.rcvRct & state.sendAck & state.sendData

  def shodBlockReq(arIdIn: UInt, eIdIn : UInt): Bool =  isToOrder && (arId === arIdIn) && (eId =/= eIdIn)
  def shodBlockRResp(arIdIn: UInt): Bool             =  !state.sendData && (arId === arIdIn) && valid
  def shodBlockFinish(eIdIn: UInt): Bool             =  valid && (eIdIn === eId)

  def arToChi[T <: ARFlit, U <: DataBufferAlloc](b: T, reqNid: UInt, ackNid: UInt, finishNid: UInt, resp: U): CHIRdEntry = {
    this                   := 0.U.asTypeOf(this)
    this.arId              := b.user(b.user.getWidth - 1, log2Ceil(node.outstanding))
    this.eId               := b.user(log2Ceil(node.outstanding) - 1, 0)
    this.size              := b.size
    this.qos               := b.qos
    this.fullSize          := b.len(0).asBool
    this.addr              := b.addr
    this.memAttr.allocate  := b.cache(2)
    this.memAttr.cacheable := b.cache(3) | b.cache(2)
    this.memAttr.device    := !b.cache(1)
    this.memAttr.ewa       := b.cache(0)
    this.dbSite1           := resp.buf(0)
    this.dbSite2           := resp.buf(1)
    this.reqNid            := reqNid
    this.ackNid            := ackNid
    this.finishNid         := finishNid
    this.state.sendAck     := Mux(!b.cache(1), true.B, false.B)
    this.valid             := true.B
    this
  }
}




class readRdDataBuffer(bufferSize: Int, outstanding: Int, axiParams: AxiParams)(implicit p: Parameters) extends ZJBundle {
  val set      = UInt(log2Ceil(bufferSize).W)
  val id       = UInt(log2Ceil(outstanding).W)
  val resp     = UInt(2.W)
  val originId = UInt(axiParams.idBits.W)
  val last     = Bool()

  def SetBdl[T <: RdDBEntry](c: T, i: UInt): readRdDataBuffer = {
    this.id       := c.idx
    this.originId := c.arID
    this.resp     := c.respErr
    this.set      := Mux(i === 1.U, c.dbSite2, c.dbSite1)
    this.last     := Mux(c.double & i === 0.U, false.B, true.B)
    this
  }
}

class DataBufferAlloc(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val buf          = Vec(2, UInt(log2Ceil(bufferSize).W))
  val num          = UInt(2.W)
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
class respDataBuffer(outstanding: Int)(implicit p: Parameters) extends ZJBundle {
  val data     = UInt(dw.W)
  val id       = UInt(log2Ceil(outstanding).W)
  val resp     = UInt(2.W)
  val last     = Bool()
}
class readWrDataBuffer(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val set     = UInt(log2Ceil(bufferSize).W)
  val tgtId   = UInt(niw.W)
  val txnID   = UInt(12.W)
  val dataID  = UInt(2.W)
}


class DmaReqFlit(implicit p : Parameters) extends ReqFlit {
  def rdReqInit[T <: CHIRdEntry](c : T, i : UInt, node: Node): ReqFlit = {
    val rni        = DmaParams(node = node)
    this          := 0.U.asTypeOf(this)
    this.QoS      := c.qos
    this.Addr     := Mux(c.fullSize, Cat(c.addr(raw - 1, rni.offset), 0.U(rni.offset.W)), c.addr)
    this.Opcode   := Mux(!c.memAttr.allocate & !c.memAttr.cacheable, ReqOpcode.ReadNoSnp, ReqOpcode.ReadOnce)
    this.SrcID    := rni.rniID.U
    this.Order    := "b11".U
    this.TxnID    := i
    this.Size     := Mux(c.fullSize, 6.U, c.size)
    this.MemAttr  := c.memAttr.asUInt
    this.SnpAttr  := Mux(c.memAttr.device | !c.memAttr.cacheable, 0.U, 1.U)
    this.ExpCompAck := Mux(c.memAttr.device, false.B, true.B)
    this
  }
  def wReqInit[T <: CHIWEntry](c : T, i : UInt, node: Node): ReqFlit = {
    val rni          = DmaParams(node = node)
    this            := 0.U.asTypeOf(this)
    this.Addr       := Mux(c.fullSize, Cat(c.addr(raw - 1, rni.offset), 0.U(rni.offset.W)), c.addr)
    this.QoS        := c.qos
    this.Opcode     := Mux(!c.memAttr.allocate & !c.memAttr.cacheable, ReqOpcode.WriteNoSnpPtl, ReqOpcode.WriteUniquePtl)
    this.Size       := Mux(c.fullSize, 6.U, c.size)
    this.TxnID      := (1.U << (this.TxnID.getWidth - 1)) | i
    this.MemAttr    := c.memAttr.asUInt
    this.ExpCompAck := true.B
    this.Order      := "b10".U
    this.SrcID      := rni.rniID.U
    this.SnpAttr    := Mux(c.memAttr.device | !c.memAttr.cacheable, 0.U, 1.U)
    this
  }
}
class DmaRspFlit(implicit p: Parameters) extends RespFlit {
  def compAckInit[T <: CHIRdEntry](c : T, node: Node) : RespFlit = {
    val rni          = DmaParams(node = node)
    this.Opcode     := RspOpcode.CompAck
    this.TxnID      := c.dbid
    this.TgtID      := c.homeNid
    this.SrcID      := rni.rniID.U
    this
  }
}

class AxiWMstEntry(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val shift     = UInt(6.W)
  val nextShift = UInt(6.W)
  val mask      = UInt(6.W)
  val size      = UInt(6.W)
  val burst     = UInt(2.W)
  val id        = UInt(rni.idBits.W)
  val last      = Bool()
  val dontMerge = Bool()
  val specWrap  = Bool()
  val fullWrap  = Bool()
  val valid     = Bool()
}


class AxiOriEntry(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni    = DmaParams(node = node)
  val id             = UInt(rni.idBits.W)
  val cache          = UInt(rni.cacheBits.W)
  val addrPrefix     = UInt((raw - rni.pageBits).W)
  val exAddr         = UInt(rni.pageBits.W)
  val size           = UInt(rni.sizeBits.W)
  val qos            = UInt(rni.qosBits.W)
  val burst          = UInt(2.W)
  val len            = UInt(rni.lenBits.W)
  val byteMask       = UInt(rni.pageBits.W)

  def wrapMask(len:UInt, size:UInt) = {
    val maxShift = 1 << 3
    val tail = ((BigInt(1) << maxShift) - 1).U
    (Cat(len, tail) << size) >> maxShift
  }

  def entryInit[T <: AXFlit](ax: T): AxiOriEntry = {
    this.addrPrefix := ax.addr(raw - 1, rni.pageBits)
    this.exAddr     := ax.addr(rni.pageBits - 1, 0) >> ax.size << ax.size
    this.qos        := ax.qos
    this.id         := ax.id
    this.byteMask   := Mux(Burst.isFix(ax.burst), 0.U, Mux(Burst.isIncr(ax.burst), 0xFFF.U, wrapMask(ax.len, ax.size)))
    this.len        := ax.len
    this.size       := ax.size
    this.cache      := ax.cache
    this.burst      := ax.burst
    this
  }
}

class WrState extends Bundle {
  val sendReq     = Bool()
  val rcvDBID     = Bool()
  val rcvComp     = Bool()
  val rcvDataCmp  = Bool()
  val sendData    = Bool()
  val sendAck     = Bool()
  val sendBResp   = Bool()
}

class CHIWEntry(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni    = DmaParams(node = node)
  val awId           = UInt(log2Ceil(node.outstanding).W)
  val eId            = UInt(log2Ceil(node.outstanding).W)
  val qos            = UInt(4.W)
  val fullSize       = Bool()
  val size           = UInt(3.W)
  val reqNid         = UInt(log2Ceil(node.outstanding).W)
  val ackNid         = UInt(log2Ceil(node.outstanding).W)
  val bNid           = UInt(log2Ceil(node.outstanding).W)
  val finishNid      = UInt(log2Ceil(node.outstanding).W)
  val memAttr        = new MemAttr
  val tgtid          = UInt(niw.W)
  val addr           = UInt(raw.W)
  val dbid           = UInt(12.W)
  val dbSite1        = UInt(log2Ceil(rni.bufferSize).W)
  val dbSite2        = UInt(log2Ceil(rni.bufferSize).W)
  val state          = new WrState
  val valid          = Bool()
  val complete       = Bool()

  def isToOrder      = valid && !state.rcvDBID
  def isWaitData     = valid && !state.rcvDataCmp
  def isSendAck      = valid && (state.rcvComp || state.rcvDBID) && !state.sendAck
  def isSendData     = valid && state.rcvDataCmp && !state.sendData && state.rcvDBID
  def isSendBResp    = valid && state.rcvComp && state.rcvDataCmp && !state.sendBResp
  def allTaskComp    = state.sendAck && state.sendData && state.sendBResp

  def shodBlockReq(awIdIn: UInt, eIdIn: UInt): Bool   = isToOrder && (awId === awIdIn) && (eId =/= eIdIn)
  def shodBlockAck(awIdIn: UInt, eIdIn: UInt): Bool   = !state.rcvComp && valid && (awId === awIdIn) && (eId =/= eIdIn)
  def shodBlockBResp(awIdIn: UInt): Bool              = !state.sendBResp && valid && (awId === awIdIn)
  def shodBlockFinish(eIdIn: UInt): Bool              = (eId === eIdIn) && valid

  def awMesInit[T <: AWFlit, U <: DataBufferAlloc](aw : T, reqNid: UInt, ackNid: UInt, bNid: UInt, finishNid: UInt, resp: U): CHIWEntry = {
    this           := 0.U.asTypeOf(this)
    this.fullSize  := aw.len(0).asBool
    this.qos       := aw.qos
    this.awId      := aw.user(aw.user.getWidth - 1, log2Ceil(node.outstanding))
    this.eId       := aw.user(log2Ceil(node.outstanding) - 1, 0)
    this.size      := aw.size
    this.reqNid    := reqNid
    this.ackNid    := ackNid
    this.bNid      := bNid
    this.finishNid := finishNid
    this.addr      := aw.addr
    this.dbSite1   := resp.buf(0)
    this.dbSite2   := resp.buf(1)
    this.memAttr.allocate  := aw.cache(3)
    this.memAttr.cacheable := aw.cache(3) | aw.cache(2)
    this.memAttr.device    := !aw.cache(1)
    this.memAttr.ewa       := aw.cache(0)
    this.valid             := true.B
    this
  }
}

class RdDBEntry(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni   = DmaParams(node = node)
  val arID          = UInt(rni.idBits.W)
  val idx           = UInt(log2Ceil(node.outstanding).W)
  val double        = Bool()
  val dbSite1       = UInt(log2Ceil(rni.bufferSize).W)
  val dbSite2       = UInt(log2Ceil(rni.bufferSize).W)
  val respErr       = UInt(2.W)
  val streamId      = UInt(log2Ceil(node.outstanding).W)

  def rdDBInit[T <: CHIRdEntry](b: T, i: UInt): RdDBEntry = {
    this.arID     := b.arId
    this.idx      := i
    this.double   := b.fullSize
    this.dbSite1  := b.dbSite1
    this.dbSite2  := b.dbSite2
    this.respErr  := b.respErr
    this.streamId := b.eId
    this
  }
}

class RdDBWrEntry(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val tgtId    = UInt(niw.W)
  val txnID    = UInt(12.W)
  val fullSize = Bool()
  val dbSite1  = UInt(log2Ceil(rni.bufferSize).W)
  val dbSite2  = UInt(log2Ceil(rni.bufferSize).W)
  val shift    = UInt(1.W)

  def rdWrDBInit[T <: CHIWEntry](b: T): RdDBWrEntry = {
    this.tgtId     := b.tgtid
    this.dbSite1   := b.dbSite1
    this.dbSite2   := b.dbSite2
    this.fullSize  := b.fullSize
    this.txnID     := b.dbid
    this.shift     := b.addr(rni.offset - 1)
    this
  }
}

class DMADebug(node: Node)(implicit p: Parameters) extends ZJBundle {
  private val rni = DmaParams(node = node)
  val wTxnID      = UInt(log2Ceil(node.outstanding).W)
  val wAWID       = UInt(rni.idBits.W)
  val wValid      = Bool()
  val rTxnID      = UInt(log2Ceil(node.outstanding).W)
  val rARID       = UInt(rni.idBits.W)
  val rValid      = Bool()
}