package zhujiang.device.bridge.tlul

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import zhujiang.chi.DatOpcode
import zhujiang.device.bridge.BaseCtrlMachine
import zhujiang.tilelink.{AFlit, AOpcode, TilelinkParams}

class TLULBridgeCtrlMachine(
  icnNode: Node,
  tlparams: TilelinkParams,
  outstanding: Int,
  ioDataBits: Int,
  compareTag: (UInt, UInt) => Bool
)(implicit p: Parameters) extends BaseCtrlMachine(
  genOpVec = new TLULBridgeCtrlOpVec,
  genInfo = new TLULCtrlInfo(ioDataBits, icnNode.nodeType == NodeType.S),
  genRsEntry = new TLULRsEntry(ioDataBits, icnNode.nodeType == NodeType.S),
  node = icnNode,
  outstanding = outstanding,
  ioDataBits = ioDataBits,
  slvBusDataBits = tlparams.dataBits,
  compareTag = compareTag
) {
  val tla = IO(Decoupled(new AFlit(tlparams)))

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
  tlaB.mask := slvMask
  tlaB.data := slvData
  tla.bits := tlaB

  when(tla.fire) {
    plmnd.wreq := tlaB.opcode === AOpcode.PutPartialData || tlaB.opcode === AOpcode.PutFullData || pld.wreq
    plmnd.rreq := tlaB.opcode === AOpcode.Get || pld.rreq
  }
  when(icn.rx.data.valid && icn.rx.data.bits.Opcode === DatOpcode.WriteDataCancel) {
    plmnd.wreq := true.B || pld.wreq
    plmnd.wresp := true.B || pld.wresp
  }
}
