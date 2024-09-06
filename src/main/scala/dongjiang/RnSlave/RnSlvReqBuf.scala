package DONGJIANG.RNSLAVE

import DONGJIANG._
import DONGJIANG.IdL0._
import DONGJIANG.CHI._
import chisel3._
import org.chipsalliance.cde.config._
import chisel3.util.{Cat, Decoupled, PopCount, RegEnable, Valid, ValidIO, log2Ceil}

class RBFSMState(implicit p: Parameters) extends Bundle {
  // schedule
  val s_snp       = Bool()
  val s_snpResp   = Bool()
  val s_req2mshr  = Bool()
  val s_resp      = Bool()
  val s_udpMSHR   = Bool()
  val s_getDBID   = Bool()
  val s_dbidResp  = Bool()

  // wait
  val w_snpResp   = Bool()
  val w_mpResp    = Bool()
  val w_dbid      = Bool()
  val w_dbData    = Bool()
  val w_rnData    = Bool()
  val w_compAck   = Bool()
}


class RnSlvReqBuf(rnSlvId: Int, reqBufId: Int, param: InterfaceParam)(implicit p: Parameters) extends PCUBaseIO(isSlv = true, hasFree = true, hasReq2Slice = true, hasDBRCReq = false) {
// --------------------- Reg and Wire declaration ------------------------//
  // req reg
  val reqReg            = RegInit(0.U.asTypeOf(new DJBundle with HasFromIDBits with HasMSHRWay {
    val addr            = UInt(addressBits.W)
    val opcode          = UInt(6.W)
    val txnId           = UInt(djparam.txnidBits.W)
    val srcId           = UInt(djparam.nodeIdBits.W)
    // Snp Mes
    val tgtId           = UInt(djparam.nodeIdBits.W)
    val retToSrc        = Bool()
    val doNotGoToSD     = Bool()
  }))
  // req from slice or txreq
  val req2Node          = WireInit(0.U.asTypeOf(reqReg))
  val reqFTxReq         = WireInit(0.U.asTypeOf(reqReg))
  val reqIsWrite        = WireInit(false.B)
  // reqBuf Ctrl
  val freeReg           = RegInit(true.B)
  val fsmReg            = RegInit(0.U.asTypeOf(new RBFSMState))
  // data crtl
  val getDBNumReg       = RegInit(0.U(log2Ceil(nrBeat + 1).W))
  val getTxDatNumReg    = RegInit(0.U(log2Ceil(nrBeat + 1).W))
  val getAllData        = WireInit(false.B) // get all Data from DataBuffer or TxDat
  val dbidReg           = RegInit(0.U(dbIdBits.W))
  val dbidBankIdReg     = RegInit(0.U(dbIdBits.W)) // dbid from which bank
  // snoop resp reg
  val snpRespReg        = RegInit(0.U(3.W))
  val snpRespHasDataReg = RegInit(false.B)
  val snpFwdStateReg    = RegInit(0.U(3.W))
  // slice resp reg
  val mpRespReg         = RegInit(0.U.asTypeOf(new Resp2NodeBundle()))



// ---------------------------  ReqBuf State release/alloc/set logic --------------------------------//
  /*
   * ReqBuf release logic
   */
  val alloc   = io.req2Node.fire | io.chi.txreq.fire
  val release = fsmReg.asUInt === 0.U // all s_task / w_task done
  freeReg     := Mux(release & !alloc, true.B, Mux(alloc, false.B, freeReg))
  io.free     := freeReg


  /*
  * Alloc or Set state
  */
  when(io.chi.txreq.fire & reqIsWrite) {
    // send
    fsmReg.s_req2mshr := true.B
    fsmReg.s_getDBID  := true.B
    fsmReg.s_dbidResp := true.B
    // wait
    fsmReg.w_dbid     := true.B
    fsmReg.w_rnData   := true.B
  }.elsewhen(io.chi.txreq.fire) {
    // send
    fsmReg.s_req2mshr := true.B
    fsmReg.s_resp     := true.B
    fsmReg.s_udpMSHR  := true.B
    // wait
    fsmReg.w_mpResp   := true.B
    fsmReg.w_compAck  := io.chi.txreq.bits.ExpCompAck
  }.elsewhen(io.req2Node.fire) {
    // send
    fsmReg.s_snp      := true.B
    fsmReg.s_snpResp  := true.B
    fsmReg.s_getDBID  := io.req2Node.bits.retToSrc
    // wait
    fsmReg.w_snpResp  := true.B
    fsmReg.w_rnData   := io.req2Node.bits.retToSrc
    fsmReg.w_dbid     := io.req2Node.bits.retToSrc
  }.otherwise {
    /*
     * Common
     */
    // send
    fsmReg.s_req2mshr := Mux(io.req2Slice.fire, false.B, fsmReg.s_req2mshr)
    fsmReg.s_getDBID  := Mux(io.dbSigs.wReq.fire, false.B, fsmReg.s_getDBID)
    // wait
    fsmReg.w_dbid     := Mux(io.dbSigs.wResp.fire, false.B, fsmReg.w_dbid)
    fsmReg.w_rnData   := Mux((io.chi.rxdat.fire & getAllData) | (io.chi.txrsp.fire & fsmReg.w_snpResp), false.B, fsmReg.w_rnData) // when need wait snp resp data but only get resp, cancel w_rnData

    /*
     * Deal CHI TxReq (except Write)
     */
    // send
    fsmReg.s_resp     := Mux(io.chi.rxrsp.fire | (io.chi.rxdat.fire & getAllData), false.B, fsmReg.s_resp)
    fsmReg.s_udpMSHR  := Mux(io.resp2Slice.fire, false.B, fsmReg.s_udpMSHR)
    // wait
    fsmReg.w_mpResp   := Mux(io.resp2Node.fire, false.B, fsmReg.w_mpResp)
    fsmReg.w_dbData   := Mux(io.resp2Node.fire & io.resp2Node.bits.isRxDat, true.B, Mux(getAllData, false.B, fsmReg.w_dbData))
    fsmReg.w_compAck  := Mux(io.chi.txrsp.fire & io.chi.txrsp.bits.Opcode === CHIOp.RSP.CompAck, false.B, fsmReg.w_compAck)

    /*
     * Deal CHI TxReq (Write)
     */
    // send
    fsmReg.s_dbidResp := Mux(io.chi.rxrsp.fire, false.B, fsmReg.s_dbidResp)

    /*
     * Deal Req from Slice (Snoop)
     */
    // send
    fsmReg.s_snp      := Mux(io.chi.rxsnp.fire, false.B ,fsmReg.s_snp)
    fsmReg.s_snpResp  := Mux(io.resp2Slice.fire, false.B ,fsmReg.s_snpResp)
    // wait
    fsmReg.w_snpResp  := Mux(io.chi.txrsp.fire | (io.chi.txdat.fire & getAllData), false.B, fsmReg.w_snpResp)
  }


// ---------------------------  Receive Req(TxReq and req2Node) Logic --------------------------------//
  /*
   * Receive req2Node(Snoop)
   */
  req2Node.addr      := io.req2Node.bits.addr
  req2Node.opcode    := io.req2Node.bits.opcode
  req2Node.from      := io.req2Node.bits.from
  req2Node.tgtId     := io.req2Node.bits.tgtId
  req2Node.txnId     := io.req2Node.bits.txnId
  req2Node.srcId     := io.req2Node.bits.srcId
  req2Node.retToSrc  := io.req2Node.bits.retToSrc
  req2Node.doNotGoToSD := io.req2Node.bits.doNotGoToSD

  /*
   * Receive chiTxReq(Read / Dataless / Atomic / CMO)
   */
  reqFTxReq.addr      := io.chi.txreq.bits.Addr
  reqFTxReq.opcode    := io.chi.txreq.bits.Opcode
  reqFTxReq.txnId     := io.chi.txreq.bits.TxnID
  reqFTxReq.srcId     := io.chi.txreq.bits.SrcID
  reqIsWrite          := CHIOp.REQ.isWriteX(io.chi.txreq.bits.Opcode)

  /*
   * Save req2Node or reqFTxReq
   */
  reqReg := Mux(io.req2Node.fire, req2Node, Mux(io.chi.txreq.fire, reqFTxReq, reqReg))


  /*
   * Save MshrWay
   */
  when(io.req2Node.fire)        { reqReg.mshrWay := io.req2Node.bits.mshrWay; reqReg.useEvict := io.req2Node.bits.useEvict  }
  .elsewhen(io.resp2Slice.fire) { reqReg.mshrWay := io.resp2Slice.bits.mshrWay; reqReg.useEvict := io.resp2Slice.bits.useEvict  }


// ---------------------------  Receive CHI Resp(TxRsp and TxDat) Logic --------------------------------//
  /*
   * Receive Snoop TxRsp
   */
  when(fsmReg.w_snpResp & (io.chi.txrsp.fire | io.chi.txdat.fire)) {
    val rsp = io.chi.txrsp
    val dat = io.chi.txdat

    snpRespReg        := Mux(rsp.fire, rsp.bits.Resp, dat.bits.Resp)
    snpRespHasDataReg := dat.fire
    snpFwdStateReg    := Mux(rsp.fire, rsp.bits.FwdState, dat.bits.FwdState)
  }

  /*
   * Count data get from TxDat number
   */
  getTxDatNumReg := Mux(release, 0.U, getTxDatNumReg + io.chi.txdat.fire.asUInt)

// ---------------------------  Send CHI Req or Resp(RxSnp, RxRsp and RxDat) Logic --------------------------------//
  /*
   * Send RxSnp
   */
  io.chi.rxsnp.valid          := fsmReg.s_snp & !fsmReg.w_dbid
  io.chi.rxsnp.bits           := DontCare
  io.chi.rxsnp.bits.TgtID     := reqReg.tgtId
  io.chi.rxsnp.bits.SrcID     := rnSlvId.U
  io.chi.rxsnp.bits.TxnID     := reqBufId.U
  io.chi.rxsnp.bits.FwdNID    := reqReg.srcId
  io.chi.rxsnp.bits.FwdTxnID  := reqReg.txnId
  io.chi.rxsnp.bits.Addr      := reqReg.addr(addressBits - 1, addressBits - io.chi.rxsnp.bits.Addr.getWidth)
  io.chi.rxsnp.bits.Opcode    := reqReg.opcode
  io.chi.rxsnp.bits.RetToSrc  := reqReg.retToSrc
  io.chi.rxsnp.bits.DoNotGoToSD := reqReg.doNotGoToSD

  /*
   * Send RxRsp
   */
  val compVal               = fsmReg.s_resp & !fsmReg.w_mpResp & !fsmReg.w_dbData
  val dbdidRespVal          = fsmReg.s_dbidResp & !fsmReg.w_dbid
  io.chi.rxrsp.valid        := compVal | dbdidRespVal
  io.chi.rxrsp.bits         := DontCare
  io.chi.rxrsp.bits.Opcode  := Mux(compVal, mpRespReg.opcode, CHIOp.RSP.CompDBIDResp)
  io.chi.rxrsp.bits.TgtID   := reqReg.srcId
  io.chi.rxrsp.bits.SrcID   := rnSlvId.U
  io.chi.rxrsp.bits.TxnID   := reqReg.txnId
  io.chi.rxrsp.bits.DBID    := reqBufId.U
  io.chi.rxrsp.bits.Resp    := Mux(compVal, mpRespReg.resp, 0.U)
  io.chi.rxrsp.bits.PCrdType := 0.U // This system dont support Transaction Retry


  /*
   * Send RxDat
   */
  io.chi.rxdat.valid        := fsmReg.s_resp & fsmReg.w_dbData & io.dbSigs.dataFDB.valid & !fsmReg.w_mpResp
  io.chi.rxdat.bits         := DontCare
  io.chi.rxdat.bits.Opcode  := mpRespReg.opcode
  io.chi.rxdat.bits.TgtID   := reqReg.srcId
  io.chi.rxdat.bits.SrcID   := rnSlvId.U
  io.chi.rxdat.bits.TxnID   := reqBufId.U
  io.chi.rxdat.bits.DBID    := reqReg.txnId
  io.chi.rxdat.bits.HomeNID := rnSlvId.U
  io.chi.rxdat.bits.Resp    := mpRespReg.resp
  io.chi.rxdat.bits.DataID  := DontCare
  io.chi.rxdat.bits.Data    := DontCare


// ---------------------------  Receive resp2Node / Send req2Slice and resp2Slice  --------------------------------//
  /*
   * Receive Resp From Slice
   */
  mpRespReg := Mux(io.resp2Node.fire, io.resp2Node.bits, mpRespReg)


  /*
   * Send Req To Slice
   */
  io.req2Slice.valid            := fsmReg.s_req2mshr & !fsmReg.w_rnData
  io.req2Slice.bits.opcode      := reqReg.opcode
  io.req2Slice.bits.addr        := reqReg.addr
  io.req2Slice.bits.isSnp       := false.B
  io.req2Slice.bits.srcID       := reqReg.srcId
  io.req2Slice.bits.txnID       := reqReg.txnId
  // IdMap
  io.req2Slice.bits.to.idL0     := IdL0.SLICE.U
  io.req2Slice.bits.to.idL1     := parseAddress(reqReg.addr)._4 // Remap in Xbar
  io.req2Slice.bits.to.idL2     := DontCare
  io.req2Slice.bits.from.idL0   := INTF.U
  io.req2Slice.bits.from.idL1   := rnSlvId.U
  io.req2Slice.bits.from.idL2   := reqBufId.U
  // Use in RnMaster
  io.req2Slice.bits.retToSrc    := DontCare
  io.req2Slice.bits.doNotGoToSD := DontCare




  /*
   * Send Resp To Slice
   * send update MSHR and send snoop resp also use resp2Slice
   */
  io.resp2Slice.valid           := (fsmReg.s_udpMSHR | fsmReg.s_snpResp) & PopCount(fsmReg.asUInt) === 1.U // only udpMSHR or snpResp need to do
  io.resp2Slice.bits.resp       := snpRespReg
  io.resp2Slice.bits.isSnpResp  := fsmReg.s_snpResp
  io.resp2Slice.bits.hasData    := snpRespHasDataReg
  io.resp2Slice.bits.dbid       := dbidReg
  io.resp2Slice.bits.mshrSet    := parseMSHRAddress(reqReg.addr)._2
  io.resp2Slice.bits.mshrWay    := reqReg.mshrWay
  io.resp2Slice.bits.useEvict   := reqReg.useEvict
  io.resp2Slice.bits.fwdState.valid := CHIOp.SNP.isSnpXFwd(reqReg.opcode)
  io.resp2Slice.bits.fwdState.bits  := snpFwdStateReg
  // IdMap
  io.resp2Slice.bits.from.idL0  := INTF.U
  io.resp2Slice.bits.from.idL1  := rnSlvId.U
  io.resp2Slice.bits.from.idL2  := reqBufId.U
  io.resp2Slice.bits.to         := Mux(fsmReg.s_udpMSHR, mpRespReg.from, reqReg.from)


// ---------------------------  DataBuffer Ctrl Signals  --------------------------------//
  /*
   * Send Data To DataBuffer
   */
  io.dbSigs.dataTDB.valid           := io.chi.txdat.valid
  io.dbSigs.dataTDB.bits.data       := DontCare
  io.dbSigs.dataTDB.bits.dataID     := DontCare
  io.dbSigs.dataTDB.bits.dbid       := dbidReg
  io.dbSigs.dataTDB.bits.to.idL0    := IdL0.SLICE.U
  io.dbSigs.dataTDB.bits.to.idL1    := dbidBankIdReg
  io.dbSigs.dataTDB.bits.to.idL2    := DontCare


  /*
   * Send wReq to get dbid
   */
  io.dbSigs.wReq.valid            := fsmReg.s_getDBID
  // IdMap
  io.dbSigs.wReq.bits.to.idL0     := IdL0.SLICE.U
  io.dbSigs.wReq.bits.to.idL1     := parseAddress(reqReg.addr)._4 // Remap in Xbar
  io.dbSigs.wReq.bits.to.idL2     := DontCare
  io.dbSigs.wReq.bits.from.idL0   := INTF.U
  io.dbSigs.wReq.bits.from.idL1   := rnSlvId.U
  io.dbSigs.wReq.bits.from.idL2   := reqBufId.U

  /*
   * Receive dbid from wResp
   */
  dbidReg       := Mux(io.dbSigs.wResp.fire, io.dbSigs.wResp.bits.dbid, dbidReg)
  dbidBankIdReg := Mux(io.dbSigs.wResp.fire, io.dbSigs.wResp.bits.from.idL1, dbidBankIdReg)

  /*
   * Count data get from DataBuffer number
   */
  getDBNumReg   := Mux(release, 0.U, getDBNumReg + io.dbSigs.dataFDB.valid.asUInt)



// ---------------------------  Other Signals  --------------------------------//
  /*
   * getAllData logic
   */
  getAllData  := getTxDatNumReg === nrBeat.U | (getTxDatNumReg === (nrBeat - 1).U & io.chi.txdat.fire) |
                 getDBNumReg === nrBeat.U    | (getDBNumReg === (nrBeat - 1).U & io.dbSigs.dataFDB.valid)


  /*
   * Set io ready value
   */
  io.chi.txreq.ready      := true.B
  io.chi.txrsp.ready      := true.B
  io.chi.txdat.ready      := io.dbSigs.dataTDB.ready
  io.req2Node.ready       := true.B
  io.resp2Node.ready      := true.B
  io.dbSigs.wResp.ready   := true.B
  io.dbSigs.dataFDB.ready := io.chi.rxdat.ready


// ---------------------------  Assertion  --------------------------------//
  // when it is free, it can receive or send mes
  assert(Mux(io.free, !io.chi.txrsp.valid, true.B))
  assert(Mux(io.free, !io.chi.txdat.valid, true.B))
  assert(Mux(io.free, !io.chi.rxdat.valid, true.B))
  assert(Mux(io.free, !io.chi.rxrsp.valid, true.B))
  assert(Mux(io.free, !io.chi.rxsnp.valid, true.B))
  assert(Mux(io.free, !io.req2Slice.valid, true.B))
  assert(Mux(io.free, !io.resp2Node.valid, true.B))
  assert(Mux(io.free, !io.resp2Slice.valid, true.B))
  assert(Mux(io.free, !io.dbSigs.wReq.valid, true.B))
  assert(Mux(io.free, !io.dbSigs.wResp.valid, true.B))
  assert(Mux(io.free, !io.dbSigs.dataFDB.valid, true.B))
  assert(Mux(io.free, !io.dbSigs.dataTDB.valid, true.B))

  assert(Mux(!freeReg, !(io.chi.txreq.valid | io.req2Node.valid), true.B), "When ReqBuf valid, it cant input new req")
  assert(Mux(io.chi.txreq.valid | io.req2Node.valid, io.free, true.B), "Reqbuf cant block req input")
  assert(!(io.chi.txreq.valid & io.req2Node.valid), "Reqbuf cant receive txreq and snpTask at the same time")
  when(release) {
    assert(fsmReg.asUInt === 0.U, "when ReqBuf release, all task should be done")
  }
  assert(Mux(getDBNumReg === nrBeat.U, !io.dbSigs.dataFDB.valid, true.B), "ReqBuf get data from DataBuf overflow")
  assert(Mux(io.dbSigs.dataFDB.valid, fsmReg.s_resp & fsmReg.w_dbData, true.B), "When dbDataValid, ReqBuf should set s_resp and w_dbData")
  assert(Mux(io.dbSigs.dataFDB.valid, !fsmReg.w_mpResp, true.B), "When dataFDBVal, ReqBuf should has been receive mpResp")

  assert(Mux(fsmReg.w_snpResp & io.chi.txrsp.fire, !io.chi.txrsp.bits.Resp(2), true.B))

  val cntReg = RegInit(0.U(64.W))
  cntReg := Mux(io.free, 0.U, cntReg + 1.U)
  assert(cntReg < TIMEOUT_RB.U, "REQBUF[0x%x] ADDR[0x%x] OP[0x%x] SNP[0x%x] TIMEOUT", reqBufId.U, reqReg.addr, reqReg.opcode, reqReg.from.isSLICE.asUInt)




}