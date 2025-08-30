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
  private val axiArSplit  = Module(new AxiDevSpilt(node = node))
  private val axiAwSplit  = Module(new AxiDevSpilt(node = node))
  private val rdDB        = Module(new DataBufferForRead(node))
  private val wrDB        = Module(new DataBufferForWrite(bufferSize = node.outstanding, ctrlSize = node.outstanding))

  private val arbReqOut   = Wire(chiWrMaster.io.chiReq.cloneType)
  private val arbRspOut   = Wire(chiWrMaster.io.chiTxRsp.cloneType)
  private val rxArQueue   = Module(new Queue(gen = new AxiBundle(axiParams).ar.bits, entries = 2, pipe = false, flow = false))
  private val rxAwQueue   = Module(new Queue(gen = new AxiBundle(axiParams).aw.bits, entries = 2, pipe = false, flow = false))
  private val rxDatQueue  = Module(new Queue(gen = chiRdMaster.io.chiDat.bits.cloneType, entries = 2, pipe = false, flow = false))
  private val rxRspPipe   = Pipe(icn.rx.resp.get.fire, icn.rx.resp.get.bits)
  private val rcvIsRct    = rxRspPipe.valid & rxRspPipe.bits.asTypeOf(chiRdMaster.io.chiRxRsp.bits).Opcode === RspOpcode.ReadReceipt

  axi.ar                <> rxArQueue.io.enq
  axi.aw                <> rxAwQueue.io.enq
  icn.rx.resp.get.ready := true.B

  axiRdSlave.io.dAxiAr <> chiRdMaster.io.axiAr
  axiRdSlave.io.finish <> chiRdMaster.io.finish
  axiRdSlave.io.dAxiR  <> rdDB.io.axiR
  axiRdSlave.io.uAxiAr <> axiArSplit.io.dAxiAx
  axiRdSlave.io.uAxiR  <> axi.r

  axiArSplit.io.dAxiAx <> axiRdSlave.io.uAxiAr
  axiArSplit.io.uAxiAx <> rxArQueue.io.deq
  axiAwSplit.io.dAxiAx <> axiWrSlave.io.uAxiAw
  axiAwSplit.io.uAxiAx <> rxAwQueue.io.deq



  axiWrSlave.io.uAxiW  <> axi.w
  axiWrSlave.io.uAxiB  <> axi.b
  axiWrSlave.io.dAxiAw <> chiWrMaster.io.axiAw
  axiWrSlave.io.dAxiW  <> chiWrMaster.io.axiW
  axiWrSlave.io.dAxiB  <> chiWrMaster.io.axiB
  axiWrSlave.io.finish <> chiWrMaster.io.finish
  

  rdDB.io.alloc        <> chiRdMaster.io.reqDB
  rdDB.io.allocRsp     <> chiRdMaster.io.respDB
  rdDB.io.rdDB         <> chiRdMaster.io.rdDB
  rdDB.io.wrDB         <> chiRdMaster.io.wrDB

  wrDB.io.alloc        <> chiWrMaster.io.reqDB
  wrDB.io.allocRsp     <> chiWrMaster.io.respDB
  wrDB.io.rdDB         <> chiWrMaster.io.rdDB
  wrDB.io.wrDB         <> chiWrMaster.io.wrDB

  chiRdMaster.io.chiRxRsp.valid := rxRspPipe.valid
  chiRdMaster.io.chiRxRsp.bits  := rxRspPipe.bits
  chiRdMaster.io.chiDat.valid   := rxDatQueue.io.deq.valid
  chiRdMaster.io.chiDat.bits    := rxDatQueue.io.deq.bits

  chiWrMaster.io.chiRxRsp.valid := rxRspPipe.valid
  chiWrMaster.io.chiRxRsp.bits  := rxRspPipe.bits
  assert(chiRdMaster.io.chiRxRsp.ready, "if ready is false, please use Queue, Do not use Pipe")
  assert(chiWrMaster.io.chiRxRsp.ready, "if ready is false, please use Queue, Do not use Pipe")

  rxDatQueue.io.deq.ready       := chiRdMaster.io.chiDat.ready


  FastArbiter(Seq(chiRdMaster.io.chiReq  , chiWrMaster.io.chiReq  ), arbReqOut)
  FastArbiter(Seq(chiRdMaster.io.chiTxRsp, chiWrMaster.io.chiTxRsp), arbRspOut)

  working  := axiRdSlave.io.working || axiWrSlave.io.working || axiArSplit.io.working || axiAwSplit.io.working

  if(icn.tx.req.isDefined) {
    connIcn(icn.tx.req.get         , arbReqOut      )
  } else {
    connIcn(icn.tx.hpr.get         , arbReqOut      )
  }
  connIcn(icn.tx.resp.get        , arbRspOut      )
  connIcn(icn.tx.data.get        , wrDB.io.dData  )
  connIcn(rxDatQueue.io.enq      , icn.rx.data.get)
  MbistPipeline.PlaceMbistPipeline(1, "MbistPipelineRni", hasMbist)
}
