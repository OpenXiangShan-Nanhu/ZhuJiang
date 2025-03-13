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
  private val axiParamsUser = AxiParams(dataBits = dw, addrBits = raw, idBits = log2Ceil(dmaParams.chiEntrySize), userBits = axiParams.idBits)
  require(axiParams.idBits >= log2Ceil(dmaParams.chiEntrySize))
  val axi = IO(Flipped(new AxiBundle(axiParams)))
  val icn = IO(new DeviceIcnBundle(node))

  // For Debug
  if (p(DebugOptionsKey).EnableDebug) {
    dontTouch(axi)
    dontTouch(icn)
  }

//SubModule
  private val axiRSpilt   = Module(new AxiRSpilt)
  private val axiWSpilt   = Module(new AxiWSpilt)
  private val chiRdE      = Module(new ChiRdCtrl)
  private val rdDB        = Module(new DataBufferForRead(axiParamsUser, dmaParams.bufferSize, dmaParams.chiEntrySize))
  private val chiWrE      = Module(new ChiWrCtrl)
  private val wrDB        = Module(new DataBufferForWrite(dmaParams.bufferSize, dmaParams.chiEntrySize))

  axiRSpilt.io.dAxiAr <> chiRdE.io.axiAr
  axiRSpilt.io.dAxiR  <> rdDB.io.axiR

  chiRdE.io.reqDB  <> rdDB.io.alloc
  chiRdE.io.respDB <> rdDB.io.allocRsp
  chiRdE.io.chiRsp <> icn.rx.resp.get
  chiRdE.io.chiDat <> icn.rx.data.get
  chiRdE.io.wrDB   <> rdDB.io.wrDB
  chiRdE.io.rdDB   <> rdDB.io.rdDB

  axi.ar <> axiRSpilt.io.uAxiAr
  axi.r  <> axiRSpilt.io.uAxiR

  chiRdE.io.chiRsp.bits    := icn.rx.resp.get.bits
  chiWrE.io.chiRxRsp.bits  := icn.rx.resp.get.bits
  chiRdE.io.chiRsp.valid   := icn.rx.resp.get.valid
  chiWrE.io.chiRxRsp.valid := icn.rx.resp.get.valid
  
  icn.rx.resp.get.ready    := chiRdE.io.chiRsp.ready & chiWrE.io.chiRxRsp.ready

// Write Logic Connection
  FastArbiter(VecInit(Seq(chiRdE.io.chiReq, chiWrE.io.chiReq)), icn.tx.req.get)
  axiWSpilt.io.dAxiAw <> chiWrE.io.axiAw
  axiWSpilt.io.dAxiW  <> chiWrE.io.axiW
  axiWSpilt.io.dAxiB  <> chiWrE.io.axiB

  chiWrE.io.chiTxRsp <> icn.tx.resp.get
  chiWrE.io.reqDB    <> wrDB.io.alloc
  chiWrE.io.respDB   <> wrDB.io.allocRsp
  chiWrE.io.wrDB     <> wrDB.io.wrDB
  chiWrE.io.rdDB     <> wrDB.io.rdDB


  axi.aw <> axiWSpilt.io.uAxiAw
  axi.w  <> axiWSpilt.io.uAxiW
  axi.b  <> axiWSpilt.io.uAxiB

  icn.tx.data.get <> wrDB.io.dData
}
