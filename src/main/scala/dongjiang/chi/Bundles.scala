package dongjiang.chi

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import scala.collection.immutable.ListMap
import zhujiang._
import zhujiang.chi._


class CHIBundleDecoupled(implicit p: Parameters) extends ZJBundle {
    val txreq = Decoupled(new ReqFlit)
    val txdat = Decoupled(new DataFlit)
    val txrsp = Decoupled(new RespFlit)

    val rxrsp = Flipped(Decoupled(new RespFlit))
    val rxdat = Flipped(Decoupled(new DataFlit))
    val rxsnp = Flipped(Decoupled(new SnoopFlit))
}


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
    def isShared = state(baseWidth, 0) === ChiResp.SC(baseWidth, 0) | state(baseWidth, 0) === ChiResp.SD(baseWidth, 0)
    def isUnique = state(baseWidth, 0) === ChiResp.UC(baseWidth, 0) | state(baseWidth, 0) === ChiResp.UD(baseWidth, 0)
    def isClean = state(baseWidth, 0) === ChiResp.SC(baseWidth, 0) | state(baseWidth, 0) === ChiResp.UC(baseWidth, 0)
    def isDirty = state(baseWidth, 0) === ChiResp.UD(baseWidth, 0) | state(baseWidth, 0) === ChiResp.SD(baseWidth, 0)
    def passDirty = state(ChiResp.width-1)
}

class CHIRespBundle extends Bundle with HasChiResp

object ChiState {
    val width = 3

    // [U/S] + [D/C] + [V/I]
    def I = "b000".U(width.W)
    def SC = "b001".U(width.W)
    def UC = "b101".U(width.W)
    def SD = "b011".U(width.W)
    def UD = "b111".U(width.W)
}

trait HasChiStates { this: Bundle =>
    val state = UInt(ChiState.width.W)

    /*
     * Coherence State
     * RN(isInvalid) -> HN(isInvalid / isShared / isUnique)
     * RN(isShared)  -> HN(isInvalid / isShared)
     * RN(isUnique)  -> HN(isInvalid)
     *
     * Dirty / Clean State
     * RN(isClean)   -> HN(isInvalid / isClean)
     * RN(isDirty)   -> HN(isInvalid / isDirty)
     */
    def isInvalid   = state(0) === ChiState.I(0)
    def isisShared  = state(ChiState.width-1) === 0.U & !isInvalid
    def isUnique    = state(ChiState.width-1) === 1.U & !isInvalid
    def isClean     = state(ChiState.width-2) === 0.U & !isInvalid
    def isDirty     = state(ChiState.width-2) === 1.U & !isInvalid
}

class CHIStateBundle extends Bundle with HasChiStates

object CHIChannel {
    val width = 2

    val REQ = "b00".U
    val DAT = "b01".U
    val RSP = "b10".U
    val SNP = "b11".U

}

trait HasCHIChannel {
    this: Bundle =>
    val channel = UInt(CHIChannel.width.W) // TODO: Del it because unuse

    def isReq = channel === CHIChannel.REQ
    def isDat = channel === CHIChannel.DAT
    def isRsp = channel === CHIChannel.RSP
    def isSnp = channel === CHIChannel.SNP
}
