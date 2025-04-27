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
import zhujiang.ZJParametersKey


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
  icnVec.head <> io.lan
  icnVec.foreach(_.tx.debug.foreach(_ := DontCare))
  icnVec.head.rx.req.get.bits  := setRx(io.lan.rx.req.get.bits.asTypeOf(new ReqFlit(false)), LAN)
  icnVec.head.rx.resp.get.bits := setRx(io.lan.rx.resp.get.bits.asTypeOf(new RespFlit()), LAN)
  icnVec.head.rx.data.get.bits := setRx(io.lan.rx.data.get.bits.asTypeOf(new DataFlit()), LAN)
  if(hasBBN) {
    icnVec.last <> io.bbnOpt.get
    icnVec.last.rx.req.get.bits   := setRx(io.bbnOpt.get.rx.req.get.bits.asTypeOf(new ReqFlit(false)), BBN)
    icnVec.last.rx.snoop.get.bits := setRx(io.bbnOpt.get.rx.snoop.get.bits.asTypeOf(new SnoopFlit()), BBN)
    icnVec.last.rx.resp.get.bits  := setRx(io.bbnOpt.get.rx.resp.get.bits.asTypeOf(new RespFlit()), BBN)
    icnVec.last.rx.data.get.bits  := setRx(io.bbnOpt.get.rx.data.get.bits.asTypeOf(new DataFlit()), BBN)
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
       |  sfMetas: ${nrSfMetas}
       |  openDCT: ${djparam.openDCT}
       |  nrPoS: ${djparam.nrPoS}
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
  val alrUsePos = frontends.map(_.io.alrUsePoS).reduce(_ +& _); dontTouch(alrUsePos)
  val posBusy   = PriorityMux(Seq(
    (alrUsePos < (djparam.nrPoS * 0.5 ).toInt.U)  -> "b00".U,
    (alrUsePos < (djparam.nrPoS * 0.75).toInt.U)  -> "b01".U,
    (alrUsePos < (djparam.nrPoS * 0.9 ).toInt.U)  -> "b10".U,
    true.B -> "b11".U,
  ))
  chiXbar.io.cBusy := RegNext(Cat(0.U(1.W), posBusy))

  /*
   * Get addr from PoS logic
   */
  // wriDirGetVec
  val wriDirGetVec = Wire(Vec(2, new GetAddr))
  // sf/llc dirBank
  wriDirGetVec.head.llcTxnID.dirBank := backend.io.writeDir.sf.bits.Addr.dirBank
  wriDirGetVec.last.llcTxnID.dirBank := backend.io.writeDir.llc.bits.Addr.dirBank
  // sf/llc pos
  wriDirGetVec.head.llcTxnID.pos := backend.io.writeDir.sf.bits.pos
  wriDirGetVec.last.llcTxnID.pos := backend.io.writeDir.llc.bits.pos
  // modsGetVec
  val modsGetVec = chiXbar.io.addrVec ++ wriDirGetVec
  // getAddrVec2
  val getAddrVec2 = Wire(Vec(djparam.nrDirBank, Vec(nrGetAddr, new GetAddr(true))))
  getAddrVec2.transpose.zip(modsGetVec).foreach { case (get, mods) =>
    val addrVec = Wire(Vec(djparam.nrDirBank, new Addr()))
    addrVec.zip(get.map(_.result)).foreach { case (a, b) => a.addr := b.addr }
    // get pos index
    get.foreach(_.pos := mods.llcTxnID.pos)
    // get result
    mods.result := addrVec(mods.llcTxnID.dirBank)
  }

  /*
   * Connect frontends
   */
  frontends.zipWithIndex.foreach {
    case(f, i) =>
      f.io.respDir_s3   := directory.io.rRespVec(i)
      f.io.updPosNest   <> backend.io.updPosNestVec(i)
      f.io.updPosTag    := backend.io.updPosTagVec(i)
      f.io.cleanPos     := backend.io.cleanPosVec(i)
      f.io.getAddrVec.zip(getAddrVec2(i)).foreach { case(a, b) => a <> b }
  }

  /*
   * Connect Directory
   */
  directory.io.readVec.zip(frontends.map(_.io.readDir_s1)).foreach { case(a, b) => a <> b }
  directory.io.unlockVec.zipWithIndex.foreach { case(vec, i) => vec := backend.io.unlockVec(i) }
  directory.io.write.sf  <> backend.io.writeDir.sf
  directory.io.write.llc <> backend.io.writeDir.llc
  directory.io.write.sf.bits.addr  := wriDirGetVec.head.result.addr
  directory.io.write.llc.bits.addr := wriDirGetVec.last.result.addr

  /*
   * Connect backend
   */
  backend.io.fastResp <> fastRRArb(frontends.map(_.io.fastResp))
  backend.io.respDir  := directory.io.wResp
  backend.io.cmtAllocVec.zip(frontends.map(_.io.cmtAlloc_s3)).foreach    { case(a, b) => a <> b }
  backend.io.cmAllocVec2.zip(frontends.map(_.io.cmAllocVec_s4)).foreach  { case(a, b) => a <> b }
  backend.io.dataResp := dataBlock.io.resp

  /*
   * Connect dataBlock
   */
  dataBlock.io.reqDB <> fastArb(Seq(backend.io.reqDB) ++ frontends.map(_.io.reqDB_s1))
  dataBlock.io.task  <> fastArb(Seq(backend.io.dataTask) ++ frontends.map(_.io.fastData_s3))
}

// Top module for unit test only
class DongJiangTop()(implicit p: Parameters) extends DJModule {
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
  val dj = Module(new DongJiang(hnfNode))
  dj.io <> DontCare
  
  dj.io.lan.rx.req.get <> io.rnf.tx.req.get
  dj.io.lan.rx.resp.get <> io.rnf.tx.resp.get
  dj.io.lan.rx.data.get <> io.rnf.tx.data.get
  val rnfTxDatFlit = io.rnf.tx.data.get.bits.asTypeOf(new DataFlit)
  val snTxDatFlit = io.sn.tx.data.get.bits.asTypeOf(new DataFlit)
  val djRxDat = dj.io.lan.rx.data.get
  val rnfTxDatToHNF = io.rnf.tx.data.get.valid && isToHNF(rnfTxDatFlit.TgtID)
  val snTxDatToHNF = io.sn.tx.data.get.valid && isToHNF(snTxDatFlit.TgtID)
  dontTouch(rnfTxDatToHNF)
  dontTouch(snTxDatToHNF)
  djRxDat.valid := rnfTxDatToHNF || snTxDatToHNF
  djRxDat.bits := Mux(rnfTxDatToHNF, io.rnf.tx.data.get.bits, io.sn.tx.data.get.bits)
  io.rnf.tx.data.get.ready := djRxDat.ready
  io.sn.tx.data.get.ready := djRxDat.ready && !rnfTxDatToHNF

  io.rnf.rx.resp.get <> dj.io.lan.tx.resp.get
  io.rnf.rx.data.get <> io.sn.tx.data.get
  val djTxDatFlit = dj.io.lan.tx.data.get.bits.asTypeOf(new DataFlit)
  val rnfRxDat = io.rnf.rx.data.get
  val snRxDat = io.sn.rx.data.get
  val djTxDatToRNF = dj.io.lan.tx.data.get.valid && isToRNF(djTxDatFlit.TgtID)
  val djTxDatToSN = dj.io.lan.tx.data.get.valid && !djTxDatToRNF
  val snTxDatToRNF = io.sn.tx.data.get.valid && isToRNF(snTxDatFlit.TgtID)
  dontTouch(djTxDatToRNF)
  dontTouch(snTxDatToRNF)
  rnfRxDat.valid := djTxDatToRNF || snTxDatToRNF
  rnfRxDat.bits := Mux(djTxDatToRNF, dj.io.lan.tx.data.get.bits, io.sn.tx.data.get.bits)
  snRxDat.valid := djTxDatToSN
  snRxDat.bits := dj.io.lan.tx.data.get.bits
  dj.io.lan.tx.data.get.ready := Mux(djTxDatToRNF, rnfRxDat.ready, snRxDat.ready)
  io.sn.tx.data.get.ready := rnfRxDat.ready && !djTxDatToRNF

  io.rnf.rx.snoop.get <> dj.io.lan.tx.snoop.get

  val djTxReq = dj.io.lan.tx.req.get
  val djTxReqFlit = djTxReq.asTypeOf(new HReqFlit)
  val djTxReqFlitWire = WireInit(djTxReq.asTypeOf(new HReqFlit))
  val snRxReqFlit = io.sn.rx.req.get.bits.asTypeOf(new HReqFlit)
  dontTouch(djTxReqFlitWire)
  io.sn.rx.req.get.valid := djTxReq.valid
  djTxReq.ready := io.sn.rx.req.get.ready
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