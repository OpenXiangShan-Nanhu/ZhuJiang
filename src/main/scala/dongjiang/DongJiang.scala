package dongjiang


import chisel3._
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
import zhujiang.chi.FlitHelper.connIcn
import xs.utils.ResetRRArbiter
import xs.utils.debug.{DomainInfo, HardwareAssertion}
import dongjiang.frontend.decode.Decode._
import zhujiang.chi


class DJConfigIO(implicit p: Parameters) extends DJBundle {
  val ci            = Input(UInt(ciBits.W))
  val closeLLC      = Input(Bool())
  val bankId        = Input(UInt(bankBits.W))
}

class DongJiang(lanNode: Node, bbnNode: Option[Node] = None)(implicit p: Parameters) extends DJModule
  with ImplicitClock with ImplicitReset {
  /*
   * IO declaration
   */
  val io  = IO(new Bundle {
    // Flush LLC
    val flushCache  = new DJBundle {
      val req       = Input(Valid(UInt(nrCcNode.W)))
      val ack       = Output(UInt(nrBank.W))
    }
    val working     = Output(Bool())
    // Configuration
    val config      = new DJConfigIO()
    // ICN
    val lan         = new DeviceIcnBundle(lanNode)
    val bbnOpt      = if(hasBBN) Some(new DeviceIcnBundle(bbnNode.get)) else None
  })
  dontTouch(io)

  // TODO
  io.flushCache.ack := DontCare
  io.working        := true.B

  /*
   * Requirement LAN and BBN
   */
  // Get hnNodeSeq
  var hnNodeSeq = Seq(lanNode)
  if(hasBBN) {
    require(bbnNode.nonEmpty)
    require(bbnNode.get.nodeType == NodeType.HX)
    hnNodeSeq = hnNodeSeq ++ Seq(bbnNode.get)
  } else {
    require(bbnNode.isEmpty)
  }
  require(lanNode.nodeType == NodeType.HF)
  // Get icnVec
  val icnVec = Wire(MixedVec(hnNodeSeq.map(n => new DeviceIcnBundle(n))))
  icnVec.head <> io.lan
  icnVec.foreach(_.tx.debug.foreach(_ := DontCare))
  NocType.setRx(icnVec.head.rx.req.get.bits.asTypeOf(new ReqFlit(false)), LAN)
  NocType.setRx(icnVec.head.rx.resp.get.bits.asTypeOf(new RespFlit()), LAN)
  NocType.setRx(icnVec.head.rx.data.get.bits.asTypeOf(new DataFlit()), LAN)
  if(hasBBN) {
    icnVec.last <> io.bbnOpt.get
    NocType.setRx(icnVec.last.rx.req.get.bits.asTypeOf(new ReqFlit(false)), BBN)
    NocType.setRx(icnVec.last.rx.snoop.get.bits.asTypeOf(new SnoopFlit()), BBN)
    NocType.setRx(icnVec.last.rx.resp.get.bits.asTypeOf(new RespFlit()), BBN)
    NocType.setRx(icnVec.last.rx.data.get.bits.asTypeOf(new DataFlit()), BBN)
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
       |  llcSize: ${djparam.llcSizeInB} B
       |  sfSize: ${djparam.sfSizeInB} B
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
       |  ccNodeIdSeq: $ccNodeIdSeq
       |  rniNodeIdSeq: $rniNodeIdSeq
       |  snNodeIdSeq: $snNodeIdSeq
       |  address slice:
       |    [fullAddr(${djparam.addressBits-1}:0)] = [useAddr1(${useAddr_hi}:${bankId_hi+1})] + [bankId(${bankId_hi}:${bankId_lo})] + [useAddr0(${bankId_lo-1}:${useAddr_lo})] + [offset(${offset_hi}:${offset_lo})]
       |    [useAddr(${useAddrBits-1}:0)]  = [useAddr1(${useAddrBits-1}:${bankId_lo-offsetBits})] + [useAddr0(${bankId_lo-offsetBits-1}:0)]
       |                     = [llcTag(${llcTag_ua_hi}:${llcTag_ua_lo})] + [llcSet(${llcSet_ua_hi}:${llcSet_ua_lo})] + [dirBank(${dirBank_ua_hi}:${dirBank_ua_lo})]
       |                     = [sfTag(${sfTag_ua_hi}:${sfTag_ua_lo})] + [sfSet(${sfSet_ua_hi}:${sfSet_ua_lo})] + [dirBank(${dirBank_ua_hi}:${dirBank_ua_lo})]
       |                     = [posTag(${posTag_ua_hi}:${posTag_ua_lo})] + [posSet(${posSet_ua_hi}:${posSet_ua_lo})] + [dirBank(${dirBank_ua_hi}:${dirBank_ua_lo})]
       |                     = [ci(${ci_ua_hi}:${ci_ua_lo})] + [unUse(${ci_ua_lo-1}:0)]
       | decodeTableSize: ${l_ci*l_si*l_ti} = ChiInst($l_ci) x StateInst($l_si) x TaskInst($l_ti) x SecTaskInst($l_sti)
       |}
       |""".stripMargin)

  /*
   * Module declaration
   */
  val level     = 0
  val frontends = Seq.tabulate(djparam.nrDirBank)(i => Module(new Frontend(i)))
  val backend   = Module(new Backend())
  val directory = Module(new Directory())
  val dataBlock = Module(new DataBlock())
  val chiXbar   = Module(new ChiXbar())


  /*
   * Connect Config
   */
  frontends.foreach(_.io.config := io.config)
  directory.io.config := io.config
  backend.io.config := io.config

  /*
   * Connect IO CHI
   */
  // [frontends].rxReq <-> [ChiXbar] <-> io.chi.rxReq
  chiXbar.io.rxReq.inVec.zip(icnVec.map(_.rx.req.get)).foreach { case(a, b) => connIcn(a, b) }
  chiXbar.io.rxReq.outVec.zip(frontends.map(_.io.rxReq)).foreach { case(a, b) => a <> b }

  // [frontends].rxSnp <-> [ChiXbar] <-> io.chi.rxSnp
  if(hasBBN) {
    connIcn(chiXbar.io.rxSnp.in, icnVec.last.rx.snoop.get)
  } else {
    chiXbar.io.rxSnp.in <> DontCare
  }
  chiXbar.io.rxSnp.outVec.zip(frontends.map(_.io.rxSnp)).foreach { case(a, b) => a <> b }

  // [backend].rxRsp <-> io.chi.rxRsp
  connIcn(backend.io.rxRsp, fastArb(icnVec.map(_.rx.resp.get)))

  // [dataBlock].rxDat <-> io.chi.rxDat
  // [backend].rxDat  <-- io.chi.rxDat
  val rxDat = fastArb(icnVec.map(_.rx.data.get))
  connIcn(dataBlock.io.rxDat, rxDat)
  backend.io.rxDat.valid := rxDat.fire
  backend.io.rxDat.bits  := rxDat.bits.asTypeOf(backend.io.rxDat.bits)

  // [backend].txReq <-> [ChiXbar] <-> io.chi.txReq
  chiXbar.io.txReq.in <> backend.io.txReq
  chiXbar.io.txReq.outVec.zip(icnVec.map(_.tx.req.get)).foreach { case (a, b) => connIcn(b, a) }

  // [backend].txSnp <-> [ChiXbar] <-> io.chi.txSnp
  chiXbar.io.txSnp.in <> backend.io.txSnp
  chiXbar.io.txSnp.outVec.zip(icnVec.map(_.tx.snoop.get)).foreach { case (a, b) => connIcn(b, a) }

  // [backend].txRsp <-> [ChiXbar] <-> io.chi.txRsp
  chiXbar.io.txRsp.in <> backend.io.txRsp
  chiXbar.io.txRsp.outVec.zip(icnVec.map(_.tx.resp.get)).foreach { case (a, b) => connIcn(b, a) }

  // [dataBlock].txDat <-> [ChiXbar] <-> io.chi.txDat
  chiXbar.io.txDat.in <> dataBlock.io.txDat
  chiXbar.io.txDat.outVec.zip(icnVec.map(_.tx.data.get)).foreach { case (a, b) => connIcn(b, a) }

  // Set CBusy in CHIXbar
  // TODO: Need to argue reasonableness
  chiXbar.io.cBusy := Cat(backend.io.multicore, frontends.map(_.io.posBusy).reduce(_ | _))


  /*
   * Connect frontends
   */
  frontends.zipWithIndex.foreach {
    case(f, i) =>
      f.io.respDir_s3 := directory.io.rRespVec(i)
      f.io.updPosNest <> backend.io.updPosNestVec(i)
      f.io.updPosTag  := backend.io.updPosTagVec(i)
      f.io.cleanPos   := backend.io.cleanPosVec(i)
      f.io.lockPosSet := backend.io.lockPosSetVec(i)
      f.io.getPosAddr := backend.io.getPosAddrVec(i)
  }

  /*
   * Connect Directory
   */
  directory.io.readVec.zip(frontends.map(_.io.readDir_s1)).foreach { case(a, b) => a <> b }
  directory.io.write <> backend.io.writeDir
  directory.io.unlockVec.zipWithIndex.foreach { case(vec, i) => vec := backend.io.unlockVec(i) }

  /*
   * Connect backend
   */
  backend.io.fastResp <> fastRRArb(frontends.map(_.io.fastResp))
  backend.io.respDir  := directory.io.wResp
  backend.io.cmtAllocVec.zip(frontends.map(_.io.cmtAlloc_s3)).foreach    { case(a, b) => a <> b }
  backend.io.cmAllocVec2.zip(frontends.map(_.io.cmAllocVec_s4)).foreach  { case(a, b) => a <> b }
  backend.io.posRespAddrVec.zip(frontends.map(_.io.posRespAddr)).foreach { case(a, b) => a := b }
  backend.io.dataResp := dataBlock.io.resp

  /*
   * Connect dataBlock
   */
  dataBlock.io.reqDB <> fastArb(Seq(backend.io.reqDB) ++ frontends.map(_.io.reqDB_s1))
  dataBlock.io.task  <> fastArb(Seq(backend.io.dataTask) ++ frontends.map(_.io.fastData_s3))
}
