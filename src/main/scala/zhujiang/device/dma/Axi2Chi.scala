package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xijiang.router.base.DeviceIcnBundle
import zhujiang.{ZJBundle, ZJModule}
import xs.utils.FastArbiter
import zhujiang.axi._
import zhujiang.chi._
import xs.utils.perf.{DebugOptions, DebugOptionsKey}

case class DmaParams(
  chiEntrySize: Int = 16,
  idBits: Int = 12,
  axiEntrySize: Int = 4,
  bufferSize: Int = 16,
  nrBeats: Int = 2,
  axiSpiltSize: Int = 4,
  shiftAddrBits: Int = 6,
)

class Axi2Chi(node: Node)(implicit p: Parameters) extends ZJModule {
  require(node.nodeType == NodeType.RI)
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits, attr = node.attr)
  val axi = IO(Flipped(new AxiBundle(axiParams)))
  val icn = IO(new DeviceIcnBundle(node))

  // For Debug
  if (p(DebugOptionsKey).EnableDebug) {
    dontTouch(axi)
    dontTouch(icn)
  }

  // //SubModule
  private val axiSpilt    = Module(new AxiSpilt)
  private val chiRdE      = Module(new ChiRdCtrl)
  private val rdDB        = Module(new DataBufferForRead(axiParams, dmaParams.bufferSize, dmaParams.chiEntrySize))
//Write Submodule 
  private val chiWrE      = Module(new ChiWrCtrl)
  private val wrDB        = Module(new DataBufferForWrite(dmaParams.bufferSize, dmaParams.chiEntrySize))

  axiSpilt.txAxi.ar <> chiRdE.io.axiAr
  axiSpilt.txAxi.r  <> rdDB.io.axiR

  chiRdE.io.reqDB  <> rdDB.io.alloc
  chiRdE.io.respDB <> rdDB.io.allocRsp
  chiRdE.io.chiRsp <> icn.rx.resp.get
  chiRdE.io.chiDat <> icn.rx.data.get
  chiRdE.io.wrDB   <> rdDB.io.wrDB
  chiRdE.io.rdDB   <> rdDB.io.rdDB

  axi.ar <> axiSpilt.rxAxi.ar
  axi.r  <> axiSpilt.rxAxi.r

  chiRdE.io.chiRsp.bits    := icn.rx.resp.get.bits
  chiWrE.io.chiRxRsp.bits  := icn.rx.resp.get.bits
  chiRdE.io.chiRsp.valid   := icn.rx.resp.get.valid
  chiWrE.io.chiRxRsp.valid := icn.rx.resp.get.valid
  
  icn.rx.resp.get.ready    := chiRdE.io.chiRsp.ready & chiWrE.io.chiRxRsp.ready

// Write Logic Connection
  FastArbiter(VecInit(Seq(chiRdE.io.chiReq, chiWrE.io.chiReq)), icn.tx.req.get)
  axiSpilt.txAxi.aw <> chiWrE.io.axiAw
  axiSpilt.txAxi.w  <> chiWrE.io.axiW
  axiSpilt.txAxi.b  <> chiWrE.io.axiB

  chiWrE.io.chiTxRsp <> icn.tx.resp.get
  chiWrE.io.reqDB    <> wrDB.io.alloc
  chiWrE.io.respDB   <> wrDB.io.allocRsp
  chiWrE.io.wrDB     <> wrDB.io.wrDB
  chiWrE.io.rdDB     <> wrDB.io.rdDB


  axi.aw <> axiSpilt.rxAxi.aw
  axi.w  <> axiSpilt.rxAxi.w
  axi.b  <> axiSpilt.rxAxi.b

  icn.tx.data.get <> wrDB.io.dData
}
