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

case class DmaParams(
  chiEntrySize: Int = 16,
  idBits: Int = 12,
  axiEntrySize: Int = 4,
  bufferSize: Int = 16,
  nrBeats: Int = 2,
  axiSpiltSize: Int = 4,
  axiMergeSize: Int = 8,
  shiftAddrBits: Int = 6,
)

class Axi2Chi(node: Node)(implicit p: Parameters) extends ZJModule {
  require(node.nodeType == NodeType.RI)
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits)
  val axi = IO(Flipped(new AxiBundle(axiParams)))
  val icn = IO(new DeviceIcnBundle(node))

  axi.aw := DontCare
  axi.w := DontCare
  axi.b := DontCare
  icn := DontCare

  // //SubModule
  private val axiSpilt    = Module(new AxiSpilt)
  private val chiRdE      = Module(new ChiREntrys)
  private val rdDB        = Module(new DataBufferForRead(axiParams, dmaParams.bufferSize, dmaParams.chiEntrySize))

  axiSpilt.rxAxi := DontCare
  axiSpilt.txAxi := DontCare

  axiSpilt.txAxi.ar <> chiRdE.io.axiAr
  axiSpilt.txAxi.r  <> rdDB.io.axiR

  chiRdE.io.reqDB  <> rdDB.io.alloc
  chiRdE.io.respDB <> rdDB.io.allocRsp
  chiRdE.io.chiReq <> icn.tx.req.get
  chiRdE.io.chiRsp <> icn.rx.resp.get
  chiRdE.io.chiDat <> icn.rx.data.get
  chiRdE.io.wrDB   <> rdDB.io.wrDB
  chiRdE.io.rdDB   <> rdDB.io.rdDB

  axi.ar <> axiSpilt.rxAxi.ar
  axi.r  <> axiSpilt.rxAxi.r

  // private val axiWrE = Module(new AXIWEntrys)
  // private val chiWrE = Module(new ChiWEntrys)
  // private val wrDB   = Module(new DataBufferForWrite(dmaParams.bufferSize, dmaParams.chiEntrySize))


  // def isWrRcvOpcode(op : UInt) : Bool = {
  //   val canRcv   = WireInit(false.B)
  //    canRcv := op === RspOpcode.Comp | op === RspOpcode.DBIDResp | op === RspOpcode.CompDBIDResp
  //    canRcv
  // }

  // // //Connect logic
  // axi.ar <> axiRdE.io.ar
  // axi.aw <> axiWrE.io.aw
  // axi.b  <> chiWrE.io.uB
  // axi.r  <> rdDB.io.axiR
  // axi.w  <> chiWrE.io.uW

  // FastArbiter(VecInit(Seq(chiRdE.io.chiReq, chiWrE.io.dTxReq)), icn.tx.req.get)

  // axiRdE.io.alloc <> chiRdE.io.alloc
  // axiWrE.io.alloc <> chiWrE.io.alloc

  // chiWrE.io.reqDB  <> wrDB.io.alloc
  // chiWrE.io.respDB <> wrDB.io.allocRsp
  // chiWrE.io.rdDB   <> wrDB.io.rdDB
  // chiWrE.io.wrDB   <> wrDB.io.wrDB

  // chiWrE.io.dTxRsp <> icn.tx.resp.get
  // wrDB.io.dData    <> icn.tx.data.get

  // icn.rx.resp.get.ready     := chiWrE.io.dRxRsp.ready & chiRdE.io.chiRsp.ready
  
  // chiWrE.io.dRxRsp.valid    := icn.rx.resp.get.valid & isWrRcvOpcode(icn.rx.resp.get.bits.asTypeOf(new RespFlit).Opcode)
  // chiRdE.io.chiRsp.valid    := icn.rx.resp.get.valid & icn.rx.resp.get.bits.asTypeOf(new RespFlit).Opcode === RspOpcode.ReadReceipt
  // chiWrE.io.dRxRsp.bits     := icn.rx.resp.get.bits
  // chiRdE.io.chiRsp.bits     := icn.rx.resp.get.bits
  
  // chiRdE.io.reqDB  <> rdDB.io.alloc
  // chiRdE.io.chiDat <> icn.rx.data.get
  // chiRdE.io.wrDB   <> rdDB.io.wrDB
  // chiRdE.io.rdDB   <> rdDB.io.rdDB
  // chiRdE.io.respDB <> rdDB.io.allocRsp
  
}
