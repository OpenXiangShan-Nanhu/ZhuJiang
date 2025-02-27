package dongjiang


import chisel3._
import chisel3.experimental.hierarchy.{instantiable, public}
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.frontend._
import dongjiang.backend._
import dongjiang.directory._
import dongjiang.data._
import xijiang._
import xijiang.router.base.DeviceIcnBundle
import xs.utils.ResetRRArbiter
import xs.utils.debug.{DomainInfo, HardwareAssertion}


class DJConfigIO(implicit p: Parameters) extends DJBundle {
  val closeLLC      = Input(Bool())
  val ci            = Input(UInt(ciBits.W))
  val bankId        = Input(UInt(bankBits.W))
  val lanFrinedsVec = Input(Vec(nrLanIcn, Vec(nrFriendsNodeMax, UInt(nodeIdBits.W))))
  val hnIdVec       = Input(Vec(nrIcn, UInt(nodeIdBits.W)))
}

@instantiable
class DongJiang(hnNodeSeq: Seq[Node])(implicit p: Parameters) extends DJRawModule
  with ImplicitClock with ImplicitReset {
  /*
   * IO declaration
   */
  @public val io  = IO(new Bundle {
    // Flush LLC
    val flushCache = new DJBundle {
      val req = Input(Valid(UInt(nrCcNode.W)))
      val ack = Output(UInt(nrBank.W))
    }
    // Configuration
    val config = new DJConfigIO()
    // ICN
    val icnVec = MixedVec(hnNodeSeq.map(n => new DeviceIcnBundle(n)))
  })
  @public val reset = IO(Input(AsyncReset()))
  @public val clock = IO(Input(Clock()))
  val implicitClock = clock
  val implicitReset = reset

  io.flushCache.ack := DontCare

  require(hnNodeSeq.length == nrLanIcn)
  if(hasHnx) {
    hnNodeSeq.init.foreach(a => require(a.nodeType == NodeType.HF))
    require(hnNodeSeq.last.nodeType == NodeType.HX)
  } else {
    hnNodeSeq.foreach(a => require(a.nodeType == NodeType.HF))
  }

  /*
   * Print message
   */
  print(
    s"""
       |DongJiang Info: {
       |  Support Protocol: CHI-G
       |  lanPortNum: ${nrLanIcn}
       |  bbnPortNum: ${nrBbnIcn}
       |  llcSize: ${djparam.llcSizeInKiB} KiB
       |  sfSize: ${djparam.sfSizeInKiB} KiB
       |  llcWays: ${djparam.llcWays}
       |  sfWays: ${djparam.sfWays}
       |  openDCT: ${djparam.openDCT}
       |  nrPoS: ${djparam.nrPoS}
       |  dataBufSize: ${djparam.dataBufSizeInByte} Byte
       |  dataSetup: ${djparam.dataSetup}
       |  dataLatency: ${djparam.dataSetup}
       |  dataExtraHold: ${djparam.dataExtraHold}
       |  dirSetup: ${djparam.dirSetup}
       |  dirLatency: ${djparam.dirLatency}
       |  dirExtraHold: ${djparam.dirExtraHold}
       |}
       |""".stripMargin)

  /*
   * Module declaration
   */
  val level     = 0
  val frontends = Seq.fill(djparam.nrDirBank) { Module(new Frontend()) }
  val backend   = Module(new Backend())
  val directory = Module(new Directory())
  val dataCtrl  = Module(new DataCtrl())
  val chiXbar   = Module(new ChiXbar())


  /*
   * Connect Config
   */
  frontends.foreach(_.io.config := io.config)

  /*
   * Connect IO CHI
   */
  // [frontends].rxReq <-> [ChiXbar] <-> io.chi.rxReq
  chiXbar.io.rxReq.inVec.zip(io.icnVec.map(_.rx.req.get)).foreach { case(a, b) => a <> b }
  chiXbar.io.rxReq.outVec.zip(frontends.map(_.io.rxReq)).foreach { case(a, b) => a <> b }

  // [frontends].rxSnp <-> [ChiXbar] <-> io.chi.rxSnp
  if(hasHnx) {
    chiXbar.io.rxSnp.in <> io.icnVec.last.rx.snoop.get
  } else {
    chiXbar.io.rxSnp.in <> DontCare
  }
  chiXbar.io.rxSnp.outVec.zip(frontends.map(_.io.rxSnp)).foreach { case(a, b) => a <> b }

  // [backend].rxRsp <-> io.chi.rxRsp
  backend.io.rxRspVec.zip(io.icnVec.map(_.rx.resp.get)).foreach { case(a, b) => a <> b }

  // [dataCtrl].rxDat <-> io.chi.rxDat
  // [backend].rxDat  <-- io.chi.rxDat
  val rxDat = fastArb(io.icnVec.map(_.rx.data.get))
  dataCtrl.io.rxDat      <> rxDat
  backend.io.rxDat.valid := rxDat.fire
  backend.io.rxDat.bits  := rxDat.bits

  // [backend].txReq <-> [ChiXbar] <-> io.chi.txReq
  chiXbar.io.txReq.inVec.zip(backend.io.txReqVec).foreach { case (a, b) => a <> b }
  chiXbar.io.txReq.outVec.zip(io.icnVec.map(_.tx.req.get)).foreach { case (a, b) => a <> b }

  // [backend].txSnp <-> [ChiXbar] <-> io.chi.txSnp
  chiXbar.io.txSnp.inVec.zip(backend.io.txSnpVec).foreach { case (a, b) => a <> b }
  chiXbar.io.txSnp.outVec.zip(io.icnVec.map(_.tx.snoop.get)).foreach { case (a, b) => a <> b }

  // [backend].txRsp <-> [ChiXbar] <-> io.chi.txRsp
  chiXbar.io.txRsp.inVec.zip(backend.io.txRspVec).foreach { case (a, b) => a <> b }
  chiXbar.io.txRsp.outVec.zip(io.icnVec.map(_.tx.resp.get)).foreach { case (a, b) => a <> b }

  // [dataCtrl].txDat <-> [ChiXbar] <-> io.chi.txDat
  chiXbar.io.txDat.inVec.zip(dataCtrl.io.txDatVec).foreach { case (a, b) => a <> b }
  chiXbar.io.txDat.outVec.zip(io.icnVec.map(_.tx.data.get)).foreach { case (a, b) => a <> b }


  /*
   * Hardware Assertion Node And IO
   */
  HardwareAssertion(true.B)
  private val assertionNode = HardwareAssertion.placePipe(Int.MaxValue, true)
  @public
  val assertionOut = IO(assertionNode.assertion.cloneType)
  @public
  val assertionInfo = DomainInfo(assertionNode.desc)
  assertionOut <> assertionNode.assertion

}
