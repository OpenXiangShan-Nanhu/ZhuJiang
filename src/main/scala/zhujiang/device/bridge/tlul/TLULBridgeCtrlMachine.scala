package zhujiang.device.bridge.tlul

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import zhujiang.device.bridge.BaseCtrlMachine
import zhujiang.tilelink.{AFlit, AOpcode, TilelinkParams}

class TLULBridgeCtrlMachine(
  icnNode: Node,
  tlparams:TilelinkParams,
  outstanding:Int,
  ioDataBits: Int,
  compareTag: (UInt, UInt) => Bool
)(implicit p: Parameters) extends BaseCtrlMachine (
  genOpVec = new TLULBridgeCtrlOpVec,
  genInfo = new TLULCtrlInfo(ioDataBits),
  genRsEntry = new TLULRsEntry(ioDataBits),
  node = icnNode,
  outstanding = outstanding,
  ioDataBits = ioDataBits,
  compareTag = compareTag
){
  val tla = IO(Decoupled(new AFlit(tlparams)))

  wakeupOutCond := allDone && valid
  private val plmnd = payloadMiscNext.state.d
  private val pld = payload.state.d

  when(io.readDataFire) {
    plmnd.rdata := io.readDataLast || pld.rdata
    plmnd.wresp := io.readDataLast || pld.wresp
  }

  private val tlaPut = payload.state.tlaPut
  private val tlaGet = payload.state.tlaGet
  private val tlaB = Wire(new AFlit(tlparams))
  private val partial = payload.info.size < log2Ceil(ioDataBits / 8).U

  tla.valid := valid && (tlaPut || tlaGet) && !waiting.orR
  tlaB := DontCare
  tlaB.opcode := MuxCase(0.U, Seq(
    tlaPut -> Mux(partial, AOpcode.PutPartialData, AOpcode.PutFullData),
    tlaGet -> AOpcode.Get
  ))
  tlaB.size := payload.info.size
  tlaB.source := io.idx
  tlaB.address := payload.info.addr
  tlaB.mask := payload.info.mask.get
  tlaB.data := payload.info.data.get
  tla.bits := tlaB

  when(tla.fire) {
    plmnd.wreq := tlaB.opcode === AOpcode.PutPartialData || tlaB.opcode === AOpcode.PutFullData || pld.wreq
    plmnd.rreq := tlaB.opcode === AOpcode.Get || pld.rreq
  }
}
