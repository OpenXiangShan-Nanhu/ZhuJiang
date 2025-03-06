package dongjiang.bundle

import chisel3._
import dongjiang.DJBundle
import zhujiang.chi._

object ChiResp {
  val width = 3

  def I  = "b000".U(width.W)
  def SC = "b001".U(width.W)
  def UC = "b010".U(width.W)
  def UD = "b010".U(width.W)
  def SD = "b011".U(width.W)

  def PassDirty = "b100".U(width.W)

  def I_PD  = "b100".U(width.W)
  def SC_PD = "b101".U(width.W)
  def UC_PD = "b110".U(width.W)
  def UD_PD = "b110".U(width.W)
  def SD_PD = "b111".U(width.W)

  def setPD(state: UInt, pd: Bool = true.B): UInt = {
    require(state.getWidth == width)
    state | Mux(pd, PassDirty, 0.U)
  }
}

trait HasChiResp { this: Bundle =>
  val state = UInt(ChiResp.width.W)

  val baseWidth = ChiResp.width-2

  def isInvalid = state(baseWidth, 0) === ChiResp.I(baseWidth, 0)
  def isShared  = state(baseWidth, 0) === ChiResp.SC(baseWidth, 0) | state(baseWidth, 0) === ChiResp.SD(baseWidth, 0)
  def isUnique  = state(baseWidth, 0) === ChiResp.UC(baseWidth, 0) | state(baseWidth, 0) === ChiResp.UD(baseWidth, 0)
  def isClean   = state(baseWidth, 0) === ChiResp.SC(baseWidth, 0) | state(baseWidth, 0) === ChiResp.UC(baseWidth, 0)
  def isDirty   = state(baseWidth, 0) === ChiResp.UD(baseWidth, 0) | state(baseWidth, 0) === ChiResp.SD(baseWidth, 0)
  def passDirty = state(ChiResp.width-1)
}

class ChiResp extends Bundle with HasChiResp

object ChiState {
  val width = 3

  def I = "b00".U(width.W)
  def SC = "b01".U(width.W)
  def UC = "b11".U(width.W)
  def UD = "b10".U(width.W)
}

trait HasChiState { this: Bundle =>
  def stateType: String = "llc"
  val sw    = if(stateType == "llc") ChiState.width else 1
  val state = UInt(sw.W)

  /*
    * Exclusive Coherence State
    * RN(isInvalid) -> HN(isInvalid / isShared / isUnique)
    * RN(isShared)  -> HN(isInvalid)
    * RN(isUnique)  -> HN(isInvalid)
    */
  def isInvalid: Bool = if(stateType == "llc") state(0).asBool else state === ChiState.I
  def isValid  : Bool = if(stateType == "llc") state(0).asBool else state =/= ChiState.I
  def isShared : Bool = if(stateType == "llc") false.B         else state === ChiState.SC
  def isUnique : Bool = if(stateType == "llc") false.B         else state(1).asBool
  def isClean  : Bool = if(stateType == "llc") false.B         else state(0).asBool
  def isDirty  : Bool = if(stateType == "llc") false.B         else state === ChiState.UD
}

class ChiState(dirType: String = "llc") extends Bundle with HasChiState {
  override def stateType: String = dirType
}

object ChiChannel {
  val width = 2

  val REQ = "b00".U
  val DAT = "b01".U
  val RSP = "b10".U
  val SNP = "b11".U

}

trait HasChiChannel { this: Bundle =>
  val channel = UInt(ChiChannel.width.W)

  def isReq = channel === ChiChannel.REQ
  def isDat = channel === ChiChannel.DAT
  def isRsp = channel === ChiChannel.RSP
  def isSnp = channel === ChiChannel.SNP
}

trait HasChiOrder { this: Bundle =>
  val order = UInt(Order.width.W)

  def noOrder = order === Order.None
  def isRO    = order === Order.RequestOrder
  def isEO    = order === Order.EndpointOrder
  def isOWO   = order === Order.OWO
  def isRA    = order === Order.RequestAccepted
}

trait HasChiSnpField { this: Bundle =>
  val snpAttr = Bool()
  val snoopMe = Bool()

  def mustSnp = snoopMe
  def canSnp  = snpAttr
  def cantSnp = !snpAttr
  def illegalSnpField = snoopMe & !snpAttr
}

trait HasChiSize { this: DJBundle =>
  val size = UInt(ChiSizeBits.W)

  def isFullSize    = size === ChiFullSize.U
  def isHalfSize    = size === ChiHalfSize.U
  def isNotFullSize = size <= ChiHalfSize.U
}


