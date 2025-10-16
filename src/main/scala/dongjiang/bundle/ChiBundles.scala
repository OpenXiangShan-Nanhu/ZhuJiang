package dongjiang.bundle

import chisel3._
import chisel3.util.PriorityMux
import dongjiang.DJBundle
import zhujiang.chi._

/*
 * Chi Resp
 */
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
  val resp      = UInt(ChiResp.width.W)

  val baseWidth = ChiResp.width-2

  def isInvalid = resp(baseWidth, 0) === ChiResp.I(baseWidth, 0)
  def isValid   = !isInvalid
  def isShared  = resp(baseWidth, 0) === ChiResp.SC(baseWidth, 0) | resp(baseWidth, 0) === ChiResp.SD(baseWidth, 0)
  def isUnique  = resp(baseWidth, 0) === ChiResp.UC(baseWidth, 0) | resp(baseWidth, 0) === ChiResp.UD(baseWidth, 0)
  def isClean   = resp(baseWidth, 0) === ChiResp.SC(baseWidth, 0) | resp(baseWidth, 0) === ChiResp.UC(baseWidth, 0)
  def isDirty   = resp(baseWidth, 0) === ChiResp.UD(baseWidth, 0) | resp(baseWidth, 0) === ChiResp.SD(baseWidth, 0)
  def passDirty = resp(ChiResp.width-1)
}

class ChiResp extends Bundle with HasChiResp


/*
 * Chi State
 */
object ChiState {
  val width = 2

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
  def isInvalid : Bool = if(stateType == "sf") !state(0).asBool else state === ChiState.I
  def isValid   : Bool = if(stateType == "sf")  state(0).asBool else state =/= ChiState.I
  def isShared  : Bool = if(stateType == "sf") false.B          else state === ChiState.SC
  def isUnique  : Bool = if(stateType == "sf") false.B          else state(1).asBool
  def isClean   : Bool = if(stateType == "sf") false.B          else state(0).asBool
  def isDirty   : Bool = if(stateType == "sf") false.B          else state === ChiState.UD

  def cbResp  : UInt = {
    val resp = PriorityMux(Seq(
      (state === ChiState.I)  -> ChiResp.I,
      (state === ChiState.SC) -> ChiResp.SC,
      (state === ChiState.UC) -> ChiResp.UC,
      (state === ChiState.UD) -> ChiResp.UD_PD,
    ))
    resp
  }
}

class ChiState(dirType: String = "llc") extends Bundle with HasChiState {
  override def stateType: String = dirType
}


/*
 * Chi Channel
 */
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

class ChiChnlBundle extends Bundle with HasChiChannel


/*
 * Chi Order and ExpCompAck
 */
trait HasChiOrderAndExpCompAck { this: Bundle =>
  val order       = UInt(Order.width.W)
  val expCompAck  = Bool()

  // In our HNF, only can deal None, EO and OWO

  def noOrder = order === Order.None
  // Permitted between: HN-I to SN-I
  def isRO    = order === Order.RequestOrder & !expCompAck
  // Permitted between: RNto HN, and HN-I to SN-I
  def isEO    = order === Order.EndpointOrder
  // Permitted between: RN to HN
  def isOWO   = order === Order.OWO & expCompAck
  // Permitted between: HN-F to SN-F, and HN-I to SN-I
  def isRA    = order === Order.RequestAccepted
}

/*
 * Chi SnpField
 */
trait HasChiSnpField { this: Bundle =>
  val snpAttr = Bool()
  val snoopMe = Bool()

  def mustSnp = snoopMe
  def canSnp  = snpAttr
  def cantSnp = !snpAttr
  def illegalSnpField = snoopMe & !snpAttr
}

/*
 * Chi RespErr
 */
trait HasRespErr { this: Bundle =>
  val respErr = Bool()

  def isOK    = respErr === RespErr.NormalOkay
  def isEXOK  = respErr === RespErr.ExclusiveOkay
  def isDERR  = respErr === RespErr.DataError
  def isNDERR = respErr === RespErr.NonDataError
  def isERR   = isDERR | isNDERR
}