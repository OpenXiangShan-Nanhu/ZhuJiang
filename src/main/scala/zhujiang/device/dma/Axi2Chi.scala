package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import zhujiang.ZJModule
import zhujiang.axi._
import zhujiang.chi._
import freechips.rocketchip.rocket.CSR.W
import freechips.rocketchip.diplomacy.BufferParams.pipe

case class DmaParams(
  bufferSize: Int = 32,
  idBits: Int = 12,
  entrySize: Int = 4,
  nidBits: Int = 2,
  sendReqNumBits: Int = 6,
  nrBeats: Int = 2
)

class Axi2Chi(node: Node)(implicit p: Parameters) extends ZJModule {
  require(node.nodeType == NodeType.RI)
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits)
  val axi = IO(Flipped(new AxiBundle(axiParams)))
  val icn = IO(new DeviceIcnBundle(node))

  dontTouch(icn)
  dontTouch(axi)
  icn.tx := DontCare
  axi := DontCare

  //SubModule
  private val readHandle  = Module(new ReadHandle)
  private val writeHandle = Module(new WriteHandle)

  private val readReqQ    = Module(new Queue(new ReqFlit, entries = dmaParams.entrySize, flow = false, pipe = true))
  private val writeReqQ   = Module(new Queue(new ReqFlit, entries = 4, flow = false, pipe =  true))


  //Connect logic
  readReqQ.io.enq.bits           := readHandle.io.chi_txreq.bits
  readReqQ.io.enq.valid          := readHandle.io.chi_txreq.valid
  readHandle.io.chi_txreq.ready  := readReqQ.io.enq.ready

  writeReqQ.io.enq.bits          := writeHandle.io.chi_txreq.bits
  writeReqQ.io.enq.valid         := writeHandle.io.chi_txreq.valid
  writeHandle.io.chi_txreq.ready := writeReqQ.io.enq.ready

  writeReqQ.io.deq.ready         := icn.tx.req.get.ready
  readReqQ.io.deq.ready          := icn.tx.req.get.ready & !writeReqQ.io.deq.fire
  
  
  axi.ar <> readHandle.io.axi_ar
  axi.aw <> writeHandle.io.axi_aw
  axi.r  <> readHandle.io.axi_r
  axi.w  <> writeHandle.io.axi_w
  axi.b  <> writeHandle.io.axi_b

  icn.rx.data.get                <> readHandle.io.chi_rxdat
  writeHandle.io.chi_rxrsp.valid <> icn.rx.resp.get.valid
  writeHandle.io.chi_rxrsp.bits  <> icn.rx.resp.get.bits

  readHandle.io.chi_rxrsp.valid  <> icn.rx.resp.get.valid
  readHandle.io.chi_rxrsp.bits   <> icn.rx.resp.get.bits

  writeHandle.io.chi_txrsp.ready := icn.tx.resp.get.ready
  readHandle.io.chi_txrsp.ready  := icn.tx.resp.get.ready && !writeHandle.io.chi_txrsp.valid

  icn.rx.resp.get.ready := writeHandle.io.chi_rxrsp.ready && readHandle.io.chi_rxrsp.ready
  icn.tx.data.get       <> writeHandle.io.chi_txdat
  icn.tx.req.get.valid  := writeReqQ.io.deq.valid | readReqQ.io.deq.valid
  icn.tx.req.get.bits   := Mux(writeReqQ.io.deq.valid, writeReqQ.io.deq.bits, Mux(readReqQ.io.deq.valid, readReqQ.io.deq.bits, 0.U.asTypeOf(readReqQ.io.deq.bits)))
  icn.tx.resp.get.valid := writeHandle.io.chi_txrsp.valid || readHandle.io.chi_txrsp.valid
  icn.tx.resp.get.bits  := Mux(writeHandle.io.chi_txrsp.valid, writeHandle.io.chi_txrsp.bits, readHandle.io.chi_txrsp.bits)
}
