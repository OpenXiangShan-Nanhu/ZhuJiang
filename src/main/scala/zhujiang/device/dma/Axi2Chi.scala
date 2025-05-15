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
import zhujiang.chi.FlitHelper.connIcn
import dongjiang.utils.fastArb
import xs.utils.mbist.MbistPipeline

class Axi2Chi(node: Node)(implicit p: Parameters) extends ZJModule {
  private val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val rni = DmaParams(node = node, offset = log2Ceil(zjParams.cachelineBytes))
  require(node.nodeType == NodeType.RI || node.nodeType == NodeType.RH)
  require(rni.idBits >= log2Ceil(node.outstanding))
  require(axiParams.dataBits == dw)

  val axi     = IO(Flipped(new AxiBundle(axiParams)))
  val icn     = IO(new DeviceIcnBundle(node))
  val working = IO(Output(Bool()))



  if(p(DebugOptionsKey).EnableDebug) {
    dontTouch(axi)
    dontTouch(icn)
    dontTouch(working)
  }
  //SubModule
  private val axiRdSlave  = Module(new AxiRdSlave(node))
  private val axiWrSlave  = Module(new AxiWrSlave(node))
  private val chiRdMaster = Module(new ChiRdMaster(node))
  private val chiWrMaster = Module(new ChiWrMaster(node))
  private val rdDB        = Module(new DataBufferForRead(node))
  private val wrDB        = Module(new DataBufferForWrite(bufferSize = node.outstanding, ctrlSize = node.outstanding))

  private val arbReqOut   = Wire(chiWrMaster.io.chiReq.cloneType)
  private val arbRspOut   = Wire(chiWrMaster.io.chiTxRsp.cloneType)


  axiRdSlave.io.dAxiAr <> chiRdMaster.io.axiAr
  axiRdSlave.io.dAxiR  <> rdDB.io.axiR
  axiRdSlave.io.uAxiAr <> axi.ar
  axiRdSlave.io.uAxiR  <> axi.r

  axiWrSlave.io.uAxiAw <> axi.aw
  axiWrSlave.io.uAxiW  <> axi.w
  axiWrSlave.io.uAxiB  <> axi.b
  axiWrSlave.io.dAxiAw <> chiWrMaster.io.axiAw
  axiWrSlave.io.dAxiW  <> chiWrMaster.io.axiW
  axiWrSlave.io.dAxiB  <> chiWrMaster.io.axiB
  

  rdDB.io.alloc        <> chiRdMaster.io.reqDB
  rdDB.io.allocRsp     <> chiRdMaster.io.respDB
  rdDB.io.rdDB         <> chiRdMaster.io.rdDB
  rdDB.io.wrDB         <> chiRdMaster.io.wrDB

  wrDB.io.alloc        <> chiWrMaster.io.reqDB
  wrDB.io.allocRsp     <> chiWrMaster.io.respDB
  wrDB.io.rdDB         <> chiWrMaster.io.rdDB
  wrDB.io.wrDB         <> chiWrMaster.io.wrDB

  FastArbiter(Seq(chiRdMaster.io.chiReq      , chiWrMaster.io.chiReq  ), arbReqOut)
  FastArbiter(Seq(chiRdMaster.io.chiTxRsp.get, chiWrMaster.io.chiTxRsp), arbRspOut)

  working  := axiRdSlave.io.working || axiWrSlave.io.working || chiRdMaster.io.working || chiWrMaster.io.working

  if(icn.tx.req.isDefined) {
    connIcn(icn.tx.req.get         , arbReqOut      )
  } else {
    connIcn(icn.tx.hpr.get         , arbReqOut      )
  }
  connIcn(icn.tx.resp.get        , arbRspOut      )
  connIcn(icn.tx.data.get        , wrDB.io.dData  )
  connIcn(chiRdMaster.io.chiDat  , icn.rx.data.get)
  connIcn(chiRdMaster.io.chiRxRsp, icn.rx.resp.get)
  connIcn(chiWrMaster.io.chiRxRsp, icn.rx.resp.get)
  MbistPipeline.PlaceMbistPipeline(1, "MbistPipelineRni", hasMbist)
}
