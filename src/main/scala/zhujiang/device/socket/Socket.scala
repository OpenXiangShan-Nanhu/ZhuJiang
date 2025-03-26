package zhujiang.device.socket

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.Node
import xijiang.c2c.{C2cBundle, C2cPacker, C2cUtils}
import xijiang.router.base.{DeviceIcnBundle, IcnBundle}
import xs.utils.ResetGen
import zhujiang.ZJParametersKey

object SocketType {
  val sync:Int = 0
  val async:Int = 1
  val c2c:Int = 2

  def getType(t:String):Int = {
    t.toLowerCase match {
      case "sync" => sync
      case "async" => async
      case "c2c" => c2c
    }
  }

  def testType(tgtStr:String, refStr:String): Boolean = {
    getType(tgtStr) == getType(refStr)
  }
}

trait SocketCommon {
  def connChn[T <: Data, K <: Data](sink:Option[DecoupledIO[T]], src:Option[DecoupledIO[K]]):Unit = {
    if(src.isDefined && sink.isDefined) require(src.get.bits.getWidth == sink.get.bits.getWidth)
    sink.foreach(s => s.valid := src.map(_.valid).getOrElse(false.B))
    sink.foreach(s => s.bits := src.map(_.bits).getOrElse(0.U).asTypeOf(s.bits))
    src.foreach(s => s.ready := sink.map(_.ready).getOrElse(false.B))
  }
  def socketType: String
  def testType(refStr:String): Boolean = SocketType.testType(socketType, refStr)
}

class SocketIcnSideBundle(val node:Node)(implicit p:Parameters) extends Bundle with SocketCommon {
  val socketType = node.socket
  val sync = if(testType("sync")) Some(new IcnPdcBundle(node)) else None
  val async = if(testType("async")) Some(new IcnAsyncBundle(node)) else None
  val c2c = if(testType("c2c")) Some(new C2cBundle) else None
  val resetTx = Output(AsyncReset())
  val c2cClock = if(testType("c2c")) Some(Input(Clock())) else None

  def <>(that: SocketDevSideBundle):Unit = {
    this.sync.foreach(_ <> that.sync.get)
    this.async.foreach(_ <> that.async.get)
    this.c2c.foreach(_ <=> that.c2c.get)
    that.resetRx := this.resetTx
  }
}

class SocketDevSideBundle(val node:Node)(implicit p:Parameters) extends Bundle with SocketCommon {
  val socketType = node.socket
  val sync = if(testType("sync")) Some(new DevPdcBundle(node)) else None
  val async = if(testType("async")) Some(new DeviceIcnAsyncBundle(node)) else None
  val c2c = if(testType("c2c")) Some(new C2cBundle) else None
  val resetRx = Input(AsyncReset())
  val c2cClock = if(testType("c2c")) Some(Input(Clock())) else None

  def <>(that: SocketIcnSideBundle):Unit = {
    this.sync.foreach(_ <> that.sync.get)
    this.async.foreach(_ <> that.async.get)
    this.c2c.foreach(_ <=> that.c2c.get)
    this.resetRx := that.resetTx
  }
}

class SocketIcnSide(node:Node)(implicit p:Parameters) extends Module with SocketCommon {
  val socketType = node.socket
  val io = IO(new Bundle {
    val dev = new DeviceIcnBundle(node)
    val socket = new SocketIcnSideBundle(node)
  })
  io.socket.resetTx := reset
  if(testType("sync")) {
    val pdcModule = Module(new ChiPdcIcnSide(node))
    pdcModule.io.dev <> io.dev
    io.socket.sync.get <> pdcModule.io.icn
  }
  if(testType("async")) {
    val asyncModule = Module(new IcnSideAsyncModule(node))
    asyncModule.io.dev <> io.dev
    io.socket.async.get <> asyncModule.io.async
  }
  if(testType("c2c")) {
    val resetGen = Module(new ResetGen())
    val asyncIcnModule = Module(new IcnSideAsyncModule(node))
    val asyncDevModule = Module(new DeviceSideAsyncModule(node))
    val c2c = Module(new C2cPacker)
    asyncIcnModule.io.dev <> io.dev
    asyncDevModule.io.async <> asyncIcnModule.io.async
    c2c.io.userTx := DontCare
    connChn(Some(c2c.io.chi.rx.req), asyncDevModule.io.icn.tx.req)
    connChn(Some(c2c.io.chi.rx.rsp), asyncDevModule.io.icn.tx.resp)
    connChn(Some(c2c.io.chi.rx.dat), asyncDevModule.io.icn.tx.data)
    connChn(Some(c2c.io.chi.rx.snp), asyncDevModule.io.icn.tx.snoop)
    connChn(c2c.io.chi.rx.dbg, asyncDevModule.io.icn.tx.debug)

    connChn(asyncDevModule.io.icn.rx.req, Some(c2c.io.chi.tx.req))
    connChn(asyncDevModule.io.icn.rx.resp, Some(c2c.io.chi.tx.rsp))
    connChn(asyncDevModule.io.icn.rx.data, Some(c2c.io.chi.tx.dat))
    connChn(asyncDevModule.io.icn.rx.snoop, Some(c2c.io.chi.tx.snp))
    connChn(asyncDevModule.io.icn.rx.debug, c2c.io.chi.tx.dbg)

    asyncDevModule.clock := io.socket.c2cClock.get
    asyncDevModule.reset := resetGen.o_reset
    c2c.clock := io.socket.c2cClock.get
    c2c.reset := resetGen.o_reset
    resetGen.clock := io.socket.c2cClock.get
    resetGen.dft := DontCare

    io.socket.c2c.get <> c2c.io.c2c
  }
}

class SocketDevSide(node:Node)(implicit p:Parameters) extends Module with SocketCommon {
  val socketType = node.socket
  val io = IO(new Bundle {
    val icn = new IcnBundle(node)
    val socket = new SocketDevSideBundle(node)
    val resetOut = Output(AsyncReset())
  })
  io.resetOut := io.socket.resetRx
  if(testType("sync")) {
    val pdcModule = Module(new ChiPdcDevSide(node))
    pdcModule.io.dev <> io.socket.sync.get
    io.icn <> pdcModule.io.icn
  }
  if(testType("async")) {
    val asyncModule = Module(new DeviceSideAsyncModule(node))
    asyncModule.io.icn <> io.icn
    io.socket.async.get <> asyncModule.io.async
  }
  if(testType("c2c")) {
    val resetGen = Module(new ResetGen())
    val asyncDevModule = Module(new DeviceSideAsyncModule(node))
    val asyncIcnModule = Module(new IcnSideAsyncModule(node))
    val c2c = Module(new C2cPacker)
    asyncDevModule.io.icn <> io.icn
    asyncIcnModule.io.async <> asyncDevModule.io.async

    c2c.io.userTx := DontCare
    connChn(Some(c2c.io.chi.rx.req), asyncIcnModule.io.dev.tx.req)
    connChn(Some(c2c.io.chi.rx.rsp), asyncIcnModule.io.dev.tx.resp)
    connChn(Some(c2c.io.chi.rx.dat), asyncIcnModule.io.dev.tx.data)
    connChn(Some(c2c.io.chi.rx.snp), asyncIcnModule.io.dev.tx.snoop)
    connChn(c2c.io.chi.rx.dbg, asyncIcnModule.io.dev.tx.debug)

    connChn(asyncIcnModule.io.dev.rx.req, Some(c2c.io.chi.tx.req))
    connChn(asyncIcnModule.io.dev.rx.resp, Some(c2c.io.chi.tx.rsp))
    connChn(asyncIcnModule.io.dev.rx.data, Some(c2c.io.chi.tx.dat))
    connChn(asyncIcnModule.io.dev.rx.snoop, Some(c2c.io.chi.tx.snp))
    connChn(asyncIcnModule.io.dev.rx.debug, c2c.io.chi.tx.dbg)

    asyncIcnModule.clock := io.socket.c2cClock.get
    asyncIcnModule.reset := resetGen.o_reset
    c2c.clock := io.socket.c2cClock.get
    c2c.reset := resetGen.o_reset
    resetGen.clock := io.socket.c2cClock.get
    resetGen.dft := DontCare

    io.socket.c2c.get <> c2c.io.c2c
  }
}
