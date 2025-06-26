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
import dongjiang.frontend.decode.Decode._
import xs.utils.debug.HardwareAssertion
import xs.utils.sram.SramPowerCtl
import zhujiang.ZJParametersKey
import zhujiang.utils.SramPwrCtlBoring


class DJConfigIO(implicit p: Parameters) extends DJBundle {
  val ci            = Input(UInt(ciBits.W))
  val closeLLC      = Input(Bool()) // TODO
  val bankId        = Input(UInt(bankBits.W))
}

class DongJiang(lanNode: Node, bbnNode: Option[Node] = None)(implicit p: Parameters) extends DJModule
  with ImplicitClock with ImplicitReset {
  /*
   * IO declaration
   */
  val io  = IO(new Bundle {
    // Flush LLC  // TODO
    val flushCache  = new DJBundle {
      val req       = Input(Valid(UInt(nrBank.W)))
      val ack       = Output(UInt(nrBank.W))
    }
    val working     = Output(Bool())
    // Configuration
    val config      = new DJConfigIO()
    // ICN
    val lan         = new DeviceIcnBundle(lanNode)
    val bbnOpt      = if(hasBBN) Some(new DeviceIcnBundle(bbnNode.get)) else None
    val ramPwrCtl   = new SramPowerCtl
  })
  dontTouch(io)

  // TODO
  io.flushCache.ack := DontCare

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
  def setRx(flit: Flit, t: Int): Flit = { val temp = WireInit(flit); temp.tgt := t.U; temp }
  val icnVec = Wire(MixedVec(hnNodeSeq.map(n => new DeviceIcnBundle(n))))
  // LAN
  icnVec.head <> io.lan
  icnVec.foreach(_.tx.debug.foreach(_ := DontCare))
  icnVec.head.rx.req.get.bits     := setRx(io.lan.rx.req.get.bits.asTypeOf(new ReqFlit(false)), LAN)
  icnVec.head.rx.resp.get.bits    := setRx(io.lan.rx.resp.get.bits.asTypeOf(new RespFlit()), LAN)
  icnVec.head.rx.data.get.bits    := setRx(io.lan.rx.data.get.bits.asTypeOf(new DataFlit()), LAN)
  if(hasHPR) {
    icnVec.head.rx.hpr.get.bits   := setRx(io.lan.rx.hpr.get.bits.asTypeOf(new ReqFlit(false)), LAN)
  }
  // BBN
  if(hasBBN) {
    icnVec.last <> io.bbnOpt.get
    icnVec.last.rx.req.get.bits   := setRx(io.bbnOpt.get.rx.req.get.bits.asTypeOf(new ReqFlit(false)), BBN)
    icnVec.last.rx.snoop.get.bits := setRx(io.bbnOpt.get.rx.snoop.get.bits.asTypeOf(new SnoopFlit()), BBN)
    icnVec.last.rx.resp.get.bits  := setRx(io.bbnOpt.get.rx.resp.get.bits.asTypeOf(new RespFlit()), BBN)
    icnVec.last.rx.data.get.bits  := setRx(io.bbnOpt.get.rx.data.get.bits.asTypeOf(new DataFlit()), BBN)
    if(hasHPR) {
      icnVec.last.rx.hpr.get.bits := setRx(io.bbnOpt.get.rx.hpr.get.bits.asTypeOf(new ReqFlit(false)), BBN)
    }
  }

  /*
   * Print message
   */
  // full addr
  val sUseAddr1   = if(hnxBankOff != 6) s"[useAddr1(${useAddr_hi}:${bankId_hi+1})] + " else s"[useAddr(${useAddr_hi}:${bankId_hi+1})] + "
  val sUseAddr0   = if(hnxBankOff != 6) s"[useAddr0(${bankId_lo-1}:${useAddr_lo})] + " else s""
  val sBankId     = if(nrBank != 0) s"[bankId(${bankId_hi}:${bankId_lo})] + " else s""
  val sFullAddr   = s"[fullAddr(${djparam.addressBits-1}:0)] = " + sUseAddr1 + sBankId + sUseAddr0 + s"[offset(${offset_hi}:${offset_lo})]"
  // dir bank id
  val sDirBankId  = if(djparam.nrDirBank > 1) s"+ [dirBank(${dirBank_ua_hi}:${dirBank_ua_lo})]" else s""
  // pos set
  val sPosSet     = if(posSets > 1) s"+ [posSet(${posSet_hn_hi}:${posSet_hn_lo})]" else s""
  //
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
       |  sfMetas: ${nrSfMetas}
       |  openDCT: ${djparam.openDCT}
       |  nrReqBuf: ${nrReqTaskBuf * djparam.nrDirBank}
       |  nrHprBuf: ${nrHprTaskBuf * djparam.nrDirBank}
       |  nrSnpBuf: ${nrSnpTaskBuf * djparam.nrDirBank}
       |  nrPoS: ${djparam.nrPoS} = dirBank[${djparam.nrDirBank}] x posSets[${posSets}] x posWays[${posWays}]
       |  dataBufSize: ${djparam.dataBufSizeInByte} B
       |  dataSetup: ${djparam.dataRamSetup}
       |  dataLatency: ${djparam.dataRamSetup}
       |  dataExtraHold: ${djparam.dataRamExtraHold}
       |  dirSetup: ${djparam.dirRamSetup}
       |  dirLatency: ${djparam.dirRamLatency}
       |  dirExtraHold: ${djparam.dirRamExtraHold}
       |  ccNodeIdSeq: $ccNodeIdSeq
       |  rniNodeIdSeq: $rniNodeIdSeq
       |  snNodeIdSeq: $snNodeIdSeq
       |  address slice:
       |    $sFullAddr
       |                     = [ci(${ci_hi}:${ci_lo})]
       |    [useAddr(${useAddrBits-1}:0)]  = [llcTag(${llcTag_ua_hi}:${llcTag_ua_lo})] + [llcSet(${llcSet_ua_hi}:${llcSet_ua_lo})] $sDirBankId
       |                     = [sfTag (${sfTag_ua_hi}:${sfTag_ua_lo})] + [sfSet (${sfSet_ua_hi}:${sfSet_ua_lo})] $sDirBankId
       |                     = [posTag(${posTag_ua_hi}:${posTag_ua_lo})] $sPosSet $sDirBankId
       |  HnTxnID slice:
       |     [hnTxnID(${hnTxnIDBits-1}:0)]  = [dirBank(${dirBank_hn_hi}:${dirBank_hn_lo})] $sPosSet + [posWay(${posWay_hn_hi}:${posWay_hn_lo})]
       |  decodeTableSize: ${l_ci*l_si*l_ti} = ChiInst($l_ci) x StateInst($l_si) x TaskInst($l_ti) x SecTaskInst($l_sti)
       |}
       |""".stripMargin)

  /*
   * Module declaration
   */
  val level     = 0
  val frontends = Seq.fill(djparam.nrDirBank)(Module(new Frontend()))
  val backend   = Module(new Backend())
  val directory = Module(new Directory())
  val dataBlock = Module(new DataBlock())
  val chiXbar   = Module(new ChiXbar())

  /*
   * System is Working
   */
  val workSftReg = RegInit(0.U((readDirLatency.max(readDsLatency) * 2).W))
  workSftReg := Cat(frontends.map(_.io.working).reduce(_ | _), workSftReg(workSftReg.getWidth-1, 1))
  io.working := workSftReg.orR

  /*
   * Connect DirBank and Config
   */
  frontends.zipWithIndex.foreach { case(a, b) => a.io.dirBank := b.U }
  frontends.foreach(_.io.config := io.config)
  directory.io.config := io.config
  backend.io.config   := io.config

  /*
   * Connect IO CHI
   */
  // [frontends].rxReq <-> [ChiXbar] <-> io.chi.rxReq
  chiXbar.io.rxReq.inVec.zip(icnVec.map(_.rx.req.get)).foreach   { case(a, b) => connIcn(a, b) }
  chiXbar.io.rxReq.outVec.zip(frontends.map(_.io.rxReq)).foreach { case(a, b) => a <> b }

  // [frontends].rxHpr <-> [ChiXbar] <-> io.chi.rxHpr
  if(hasHPR) {
    chiXbar.io.rxHpr.inVec.get.zip(icnVec.map(_.rx.hpr.get)).foreach { case (a, b) => connIcn(a, b) }
  }
  chiXbar.io.rxHpr.outVec.zip(frontends.map(_.io.rxHpr)).foreach { case (a, b) => a <> b }

  // [frontends].rxSnp <-> [ChiXbar] <-> io.chi.rxSnp
  if(hasBBN) {
    connIcn(chiXbar.io.rxSnp.in, icnVec.last.rx.snoop.get)
  } else {
    chiXbar.io.rxSnp.in <> DontCare
  }
  chiXbar.io.rxSnp.outVec.zip(frontends.map(_.io.rxSnp)).foreach { case(a, b) => a <> b }

  // [backend].rxRsp <-> io.chi.rxRsp
  val rxRspArb = Module(new Arbiter(new RespFlit, nrIcn))
  rxRspArb.io.in.zip(icnVec.map(_.rx.resp)).foreach { case(a, b) => a <> b.get }
  connIcn(backend.io.rxRsp, rxRspArb.io.out)

  // [dataBlock].rxDat <-> io.chi.rxDat
  // [backend].rxDat  <-- io.chi.rxDat
  val rxDatArb = Module(new Arbiter(new DataFlit, nrIcn))
  rxDatArb.io.in.zip(icnVec.map(_.rx.data)).foreach { case (a, b) => a <> b.get }
  connIcn(dataBlock.io.rxDat, rxDatArb.io.out)
  backend.io.rxDat.valid := rxDatArb.io.out.fire
  backend.io.rxDat.bits  := rxDatArb.io.out.bits

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
  val alrUsePos = frontends.map(_.io.alrUsePoS).reduce(_ +& _); dontTouch(alrUsePos)
  val posBusy   = PriorityMux(Seq(
    (alrUsePos < (djparam.nrPoS * 0.5 ).toInt.U)  -> "b00".U,
    (alrUsePos < (djparam.nrPoS * 0.75).toInt.U)  -> "b01".U,
    (alrUsePos < (djparam.nrPoS * 0.9 ).toInt.U)  -> "b10".U,
    true.B -> "b11".U,
  ))
  chiXbar.io.cBusy := RegNext(Cat(0.U(1.W), posBusy))

  /*
   * Connect frontends
   */
  frontends.zipWithIndex.foreach {
    case(f, i) =>
      f.io.respDir          := directory.io.rRespVec(i)
      f.io.reqPoS           <> backend.io.reqPosVec(i)
      f.io.updPosTag        := backend.io.updPosTag
      f.io.updPosNest       := backend.io.updPosNest
      f.io.cleanPos         := backend.io.cleanPos
      f.io.getAddrVec.zip(backend.io.getAddrVec).foreach { case(a, b) => a.hnIdx := b.hnIdx }
  }

  /*
   * Connect Directory
   */
  directory.io.readVec.zip(frontends.map(_.io.readDir)).foreach { case(a, b) => a <> b }
  directory.io.write        <> backend.io.writeDir
  directory.io.unlock       := backend.io.unlock

  /*
   * Connect backend
   */
  backend.io.respDir        := directory.io.wResp
  backend.io.fastResp       <> fastRRArb(frontends.map(_.io.fastResp))
  backend.io.recRespType    <> fastRRArb(frontends.map(_.io.recRespType))
  backend.io.dataResp       := dataBlock.io.resp
  backend.io.cmtTaskVec.zip(frontends.map(_.io.cmtTask)).foreach { case(a, b) => a <> b }
  backend.io.getAddrVec.zip(frontends.map(_.io.getAddrVec).transpose).foreach { case(a, b) =>
    a.result := VecInit(b.map(_.result))(a.hnIdx.dirBank)
  }

  /*
   * Connect dataBlock
   */
  dataBlock.io.cutHnTxnID   := backend.io.cutHnTxnID
  dataBlock.io.cleanDB      <> fastArb.validOut(Seq(backend.io.cleanDB, fastRRArb(frontends.map(_.io.cleanDB))))
  dataBlock.io.reqDB        <> fastArb(Seq(backend.io.reqDB, fastRRArb(frontends.map(_.io.reqDB_s3)), fastRRArb(frontends.map(_.io.reqDB_s1))))
  dataBlock.io.task         := fastArb.validOut(Seq(backend.io.dataTask, fastRRArb(frontends.map(_.io.fastData))))

  /*
   * HardwareAssertion placePipe
   */
  SramPwrCtlBoring.getSrc() := io.ramPwrCtl
  HardwareAssertion.placePipe(3)
}

// Top module for unit test only
class DongJiangTop()(implicit p: Parameters) extends DJModule {
  val nrPosWays = posWays - 2
  require(nrSnoopCM == 4, f"nrSnoopCM should be 4, but is ${nrSnoopCM}")
  require(nrPosWays == 2, f"nrPosWays should be 2, but is ${nrPosWays}")
  require(llcSets == 4, f"llcSets should be 4, but is ${llcSets}")
  require(sfSets == 8, f"sfSets should be 8, but is ${sfSets}")
  require(nrDataCM == 8, f"nrDataCM should be 8, but is ${nrDataCM}")
  require(nrReadCM == 4, f"nrReadCM should be 8, but is ${nrReadCM}")

  val hnfNode = Node(nodeType = NodeType.HF)
  val rnfNode = Node(nodeType = NodeType.RF)
  val snNode = Node(nodeType = NodeType.S)
  val io = IO(new Bundle {
    val rnf = Flipped(new DeviceIcnBundle(rnfNode))
    val sn = Flipped(new DeviceIcnBundle(snNode))
  })

  dontTouch(io)

  val rnfNodes = zjParams.island.filter(_.nodeType == NodeType.CC)
  val rniNodes = zjParams.island.filter(_.nodeType == NodeType.RI)
  val hnfNodes = zjParams.island.filter(_.nodeType == NodeType.HF)
  val snNodes  = zjParams.island.filter(_.nodeType == NodeType.S)
  val rnfIdSeq = rnfNodes.map(_.nodeId)
  val rniIdSeq = rniNodes.map(_.nodeId)
  val hnfIdSeq = hnfNodes.map(_.nodeId)
  val snIdSeq  = snNodes.map(_.nodeId)

  def isToHNF(tgtID: UInt) = {
    Cat(hnfIdSeq.map( id => id.U >> zjParams.nodeAidBits.U === tgtID >> zjParams.nodeAidBits.U)).orR
  }

  def isToRNF(tgtID: UInt) = {
    Cat(rnfIdSeq.map( id => id.U >> zjParams.nodeAidBits.U === tgtID >> zjParams.nodeAidBits)).orR ||
      Cat(rniIdSeq.map( id => id.U >> zjParams.nodeAidBits.U === tgtID >> zjParams.nodeAidBits.U)).orR
  }

  def isToSN(tgtID: UInt) = {
    Cat(snIdSeq.map( id => id.U >> zjParams.nodeAidBits.U === tgtID >> zjParams.nodeAidBits.U)).orR
  }

  val djNodeId = hnfNodes.head.nodeId
  val snNodeId = snNodes.head.nodeId
  val dj = Module(new DongJiang(hnfNode))
  dj.io <> DontCare

  dj.io.ramPwrCtl := 0.U.asTypeOf(dj.io.ramPwrCtl)
  dj.io.lan.rx.req.get <> io.rnf.tx.req.get
  dj.io.lan.rx.resp.get <> io.rnf.tx.resp.get
  dj.io.lan.rx.data.get <> io.rnf.tx.data.get
  val rnfTxDatFlit = io.rnf.tx.data.get.bits.asTypeOf(new DataFlit)
  val snTxDatFlit = io.sn.tx.data.get.bits.asTypeOf(new DataFlit)
  val snTxDatFlitWire = WireInit(snTxDatFlit)
  val djRxDat = dj.io.lan.rx.data.get
  val rnfTxDatToHNF = io.rnf.tx.data.get.valid && isToHNF(rnfTxDatFlit.TgtID)
  val snTxDatToHNF = io.sn.tx.data.get.valid && isToHNF(snTxDatFlit.TgtID)
  dontTouch(rnfTxDatToHNF)
  dontTouch(snTxDatToHNF)
  djRxDat.valid := rnfTxDatToHNF || snTxDatToHNF
  djRxDat.bits := Mux(rnfTxDatToHNF, io.rnf.tx.data.get.bits, snTxDatFlitWire)
  io.rnf.tx.data.get.ready := djRxDat.ready
  io.sn.tx.data.get.ready := djRxDat.ready && !rnfTxDatToHNF
  snTxDatFlitWire.HomeNID := djNodeId.U

  val djTxRespFlit = dj.io.lan.tx.resp.get.bits.asTypeOf(new RespFlit)
  val djTxRespFlitWire = WireInit(djTxRespFlit)
  io.rnf.rx.resp.get <> dj.io.lan.tx.resp.get
  djTxRespFlitWire.SrcID := djNodeId.U
  io.rnf.rx.resp.get.bits := djTxRespFlitWire

  io.rnf.rx.data.get <> io.sn.tx.data.get

  val djTxDatFlit = dj.io.lan.tx.data.get.bits.asTypeOf(new DataFlit)
  val djTxDatFlitWire = WireInit(djTxDatFlit) 
  val rnfRxDat = io.rnf.rx.data.get
  val snRxDat = io.sn.rx.data.get
  val djTxDatToRNF = dj.io.lan.tx.data.get.valid && isToRNF(djTxDatFlit.TgtID)
  val djTxDatToSN = dj.io.lan.tx.data.get.valid && !djTxDatToRNF
  val snTxDatToRNF = io.sn.tx.data.get.valid && isToRNF(snTxDatFlit.TgtID)
  dontTouch(djTxDatToRNF)
  dontTouch(snTxDatToRNF)
  rnfRxDat.valid := djTxDatToRNF || snTxDatToRNF
  rnfRxDat.bits := Mux(djTxDatToRNF, djTxDatFlitWire, snTxDatFlitWire)
  snRxDat.valid := djTxDatToSN
  snRxDat.bits := dj.io.lan.tx.data.get.bits
  djTxDatFlitWire.SrcID := djNodeId.U
  djTxDatFlitWire.HomeNID := djNodeId.U
  dj.io.lan.tx.data.get.ready := Mux(djTxDatToRNF, rnfRxDat.ready, snRxDat.ready)
  io.sn.tx.data.get.ready := Mux(snTxDatToRNF, rnfRxDat.ready && !djTxDatToRNF, djRxDat.ready && !rnfTxDatToHNF)  
  when(dj.io.lan.tx.data.get.valid) {
    assert(djTxDatToRNF || djTxDatToSN, "djTxDatToRNF or djTxDatToSN must be true")
  }

  val djTxSnpFlit = dj.io.lan.tx.snoop.get.bits.asTypeOf(new SnoopFlit)
  val djTxSnpFlitWire = WireInit(djTxSnpFlit)
  io.rnf.rx.snoop.get <> dj.io.lan.tx.snoop.get
  djTxSnpFlitWire.SrcID := djNodeId.U
  io.rnf.rx.snoop.get.bits := djTxSnpFlitWire

  val djTxReq = dj.io.lan.tx.req.get
  val djTxReqFlit = djTxReq.asTypeOf(new HReqFlit)
  val djTxReqFlitWire = WireInit(djTxReq.asTypeOf(new HReqFlit))
  val snRxReqFlit = io.sn.rx.req.get.bits.asTypeOf(new HReqFlit)
  dontTouch(djTxReqFlitWire)
  io.sn.rx.req.get.valid := djTxReq.valid
  djTxReq.ready := io.sn.rx.req.get.ready
  djTxReqFlitWire.SrcID := djNodeId.U
  djTxReqFlitWire.TgtID := snNodeId.U
  djTxReqFlitWire.ReturnNID.get := Mux(djTxReqFlit.ReturnNID.get === 0xFF.U, djNodeId.U, djTxReqFlit.ReturnNID.get)
  io.sn.rx.req.get.bits := djTxReqFlitWire

  val rnfTxRspFlit = io.rnf.tx.resp.get.bits.asTypeOf(new RespFlit)
  val snTxRspFlit = io.sn.tx.resp.get.bits.asTypeOf(new RespFlit)
  val djRxRsp = dj.io.lan.rx.resp.get
  val rnfTxRspToHNF = io.rnf.tx.resp.get.valid && isToHNF(rnfTxRspFlit.TgtID)
  val snTxRspToHNF = io.sn.tx.resp.get.valid && isToHNF(snTxRspFlit.TgtID)
  djRxRsp.valid := rnfTxRspToHNF || snTxRspToHNF
  djRxRsp.bits := Mux(rnfTxRspToHNF, io.rnf.tx.resp.get.bits, io.sn.tx.resp.get.bits)
  io.rnf.tx.resp.get.ready := djRxRsp.ready
  io.sn.tx.resp.get.ready := djRxRsp.ready && !rnfTxRspToHNF

  dontTouch(dj.io)
}