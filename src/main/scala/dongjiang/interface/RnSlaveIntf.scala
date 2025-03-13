package dongjiang.pcu.intf

import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.SnpOpcode._
import zhujiang.chi._
import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.StepRREncoder
import xijiang.Node
import xs.utils._
import xs.utils.perf.{DebugOptions, DebugOptionsKey, HasPerfLogging}
import xs.utils.debug.{DomainInfo, HardwareAssertion, awhen}

/*
 * ************************************************************** State transfer ***********************************************************************************
 * Req Expect Write And Atomic
 * Req Retry:             [Free]  -----> [Req2Exu] -----> [WaitExuAck] ---retry---> [Req2Exu]
 * Req Receive:           [Free]  -----> [Req2Exu] -----> [WaitExuAck] --receive--> ([waitSendRRec]) -----> [Free]
 *                                           ^
 *
 * Amotic Retry:          [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] ---retry---> [Req2Exu]
 * Atomic:                [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] --receive--> [Free]
 *                                           ^
 *
 * Write Retry:           [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] ---retry---> [Req2Exu]
 * Write:                 [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] --receive--> [WaitCompAck] -----> [Free]
 *                                                                                                                       ^
 *
 * Read / Dataless / Atomic Resp:  [Free]  -----> [Resp2Node] -----> ([WaitCompAck]) -----> [Free]
 * CopyBack Resp:                  [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Resp2Node] -----> [Free]
 *
 *
 * Snoop Need Data:       [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [Snp2Node] -----> ([Snp2NodeIng]) -----> [WaitSnpResp] -----> [Resp2Exu]
 * Snoop No Need Data:    [Free]                   ----->                   [Snp2Node] -----> ([Snp2NodeIng]) -----> [WaitSnpResp] -----> [Resp2Exu]
 *                                                                              ^
 *
 * ( ^ is NID block point)
 *
 * ************************************************************** Entry Transfer ********************************************************************************
 * Req / Write / Atomic:
 * [------] <- Req0x0 | [Req0x0] -> Req to EXU   | [------] <- Resp from EXU    | [Rsp0x0] -> resp to node
 * [------]           | [------]                 | [------]                     | [------]
 * [------]           | [------]                 | [------]                     | [------]
 * [------]           | [------]                 | [------]                     | [------]
 *
 *
 * Write:
 * [------] <- Wri0x0 | [Wri0x0] -> Wri to EXU   | [Wri0x0] <- CompAck from rni | [------]
 * [------]           | [------]                 | [------]                     | [------]
 * [------]           | [------]                 | [------]                     | [------]
 * [------]           | [------]                 | [------]                     | [------]
 *
 * Snoop:
 * [------] <- Snp0x0 | [Snp0x0] -> Snp to node  | [Snp0x0] <- Resp from node   | [Snp0x0] -> resp to EXU
 * [------]           | [------]                 | [------]                     | [------]
 * [------]           | [------]                 | [------]                     | [------]
 * [------]           | [------]                 | [------]                     | [------]
 *
 *
 * Req / Write / Atomic Nest By Snp:
 * [------] <- Req0x0 | [Req0x0]                 | [Req0x0] need to wait snoop done
 * [------]           | [------] <-Snp0x0        | [Snp0x0]
 * [------]           | [------]                 | [------]
 * [------]           | [------]                 | [------]
 *
 * Resp Nest By Snp:
 * [------] <- Snp0x0 | [Snp0x0]                 | [Snp0x0] need to wait Resp Get CompAck
 * [------]           | [------] <-Resp0x0       | [Rsp0x0]
 * [------]           | [------]                 | [------]
 * [------]           | [------]                 | [------]
 *
 */

object RSState {
  val width = 4
  // commom
  val Free            = "b0000".U // 0x0
  val Req2Exu         = "b0001".U // 0x1
  val WaitExuAck      = "b0010".U // 0x2
  val WaitSendRRec    = "b0011".U // 0x3
  val Resp2Node       = "b0101".U // 0x5
  val WaitCompAck     = "b0110".U // 0x6
  val Resp2Exu        = "b0111".U // 0x7
  val GetDBID         = "b1000".U // 0x8
  val WaitDBID        = "b1001".U // 0x9
  val DBIDResp2Node   = "b1010".U // 0xa
  val WaitData        = "b1011".U // 0xb
  val Snp2Node        = "b1100".U // 0xc
  val Snp2NodeIng     = "b1101".U // 0xd
  val WaitSnpResp     = "b1110".U // 0xe
}

class RSEntry(param: InterfaceParam)(implicit p: Parameters) extends DJBundle  {
  val chiIndex        = new ChiIndexBundle()
  val chiMes          = new ChiMesBundle()
  val pcuIndex        = new PcuIndexBundle()
  val entryMes        = new DJBundle with HasUseAddr with HasDcuID {
    val state         = UInt(RSState.width.W)
    val nID           = UInt(param.entryIdBits.W)
    val hasData       = Bool()
    val getBeatNum    = UInt(1.W)
    val getSnpRespVec = Vec(nrCcNode, Bool())
    val snpFwdWaitAck = Bool() // CompAck
    val needSendRRec  = Bool() // Need send ReadReceipt
    val swapFst       = Bool() // Only use in atomic
    val snpFwdSVal    = Bool() // Snoop get SnpRespXFwded
    val snpTgtVec     = Vec(nrCcNode, Bool())
    val reqIsWrite    = Bool() // Adding Reg for timing considerations
    val reqIsAtomic   = Bool() // Adding Reg for timing considerations
  }

  def state         = entryMes.state
  def isFree        = entryMes.state === RSState.Free
  def isReqBeSend   = entryMes.state === RSState.Req2Exu    & entryMes.nID === 0.U
  def isRspBeSend   = (entryMes.state === RSState.Resp2Node & (chiMes.isRsp | (chiMes.isReq & isWriUniX(chiMes.opcode)))) | entryMes.state === RSState.DBIDResp2Node | entryMes.needSendRRec
  def isDatBeSend   = entryMes.state === RSState.Resp2Node  & chiMes.isDat
  def isGetDBID     = entryMes.state === RSState.GetDBID    & (entryMes.nID === 0.U | !isWriUniX(chiMes.opcode))
  def isSendSnp     = entryMes.state === RSState.Snp2Node   & entryMes.nID === 0.U
  def isSendSnpIng  = entryMes.state === RSState.Snp2NodeIng
  def isLastBeat    = Mux(chiIndex.fullSize, entryMes.getBeatNum === 1.U, entryMes.getBeatNum === 0.U)
  def fullAddr(p: UInt) = entryMes.fullAddr(entryMes.dcuID, p, chiIndex.offset)
  def snpAddr (p: UInt) = entryMes.snpAddr(entryMes.dcuID, p)
  def addrWithDcuID     = Cat(entryMes.useAddr, entryMes.dcuID)
}




class RnSlaveIntf(param: InterfaceParam, node: Node)(implicit p: Parameters) extends IntfBaseIO(param, node) with HasPerfLogging  {
// --------------------- Reg and Wire declaration ------------------------//
  val entrys            = RegInit(VecInit(Seq.fill(param.nrEntry) { 0.U.asTypeOf(new RSEntry(param)) }))
  // ENTRY Receive Task ID
  val entryFreeID       = Wire(UInt(param.entryIdBits.W))
  // ENTRY Send Req To EXU
  val entrySendReqID    = Wire(UInt(param.entryIdBits.W))
  // ENTRY Get DBID ID
  val entryGetDBID      = Wire(UInt(param.entryIdBits.W))
  // ENTRY Receive DBID ID
  val entryRecDBID      = Wire(UInt(param.entryIdBits.W))
  // ENTRY Send Snp ID
  val entrySendSnpID    = Wire(UInt(param.entryIdBits.W))
  // ENTRY Receive TxRsp ID
  val entryRecChiRspID  = Wire(UInt(param.entryIdBits.W))
  // ENTRY Receive TxDat ID
  val entryRecChiDatID  = Wire(UInt(param.entryIdBits.W))
  // ENTRY Send Resp To EXU ID
  val entryResp2ExuID   = Wire(UInt(param.entryIdBits.W))
  // ENTRY Send Resp To Node ID
  val entrySendRspID    = Wire(UInt(param.entryIdBits.W))
  // ENTRY Rec Data From DB ID
  val entryRecDataFDBID = Wire(UInt(param.entryIdBits.W))
  // req/resp from EXU or rxReq
  val entrySave         = WireInit(0.U.asTypeOf(new RSEntry(param)))
  // snp to node
  val snpIsLast         = Wire(Bool())
  val snpAlrSendOHReg   = RegInit(0.U(nrCcNode.W))
  // CompAck is generated by DMT
  val rspIsDMTComp      = Wire(Bool())
  val dmtCompVal        = Wire(Bool())
  // HitVec
  val snp2IntfHitVec_g    = Reg(Vec(param.nrEntry, Bool())); dontTouch(snp2IntfHitVec_g)
  val snpResp2ExuHitVec_g = Reg(Vec(param.nrEntry, Bool())); dontTouch(snpResp2ExuHitVec_g)
  val reqAckHitVec_g      = Reg(Vec(param.nrEntry, Bool())); dontTouch(reqAckHitVec_g)
  val compAckHitVec_g     = Reg(Vec(param.nrEntry, Bool())); dontTouch(compAckHitVec_g)
  // MatchVec
  val reqMatchVec         = Wire(Vec(param.nrEntry, Bool())); dontTouch(reqMatchVec)
  val snpMatchVec         = Wire(Vec(param.nrEntry, Bool())); dontTouch(snpMatchVec)

  val entrysValVec_g      = Reg(Vec(param.nrEntry, Bool()));
  val entrysIsSnpVec_g    = Reg(Vec(param.nrEntry, Bool()));
  val entrysIsReqVec_g    = Reg(Vec(param.nrEntry, Bool()));

  // CHI
  val rxReq             = Wire(new DecoupledIO(new ReqFlit()))
  val rxDat             = Wire(new DecoupledIO(new DataFlit()))
  val rxRsp             = Wire(new DecoupledIO(new RespFlit()))
  val txSnp             = WireInit(0.U.asTypeOf(Decoupled(new SnoopFlit())))
  val txDat             = WireInit(0.U.asTypeOf(Decoupled(new DataFlit())))
  val txRsp             = WireInit(0.U.asTypeOf(Decoupled(new RespFlit())))
  io.chi                <> DontCare
  io.chi.rx.req.get     <> rxReq
  io.chi.rx.data.get    <> rxDat
  io.chi.rx.resp.get    <> rxRsp
  io.chi.tx.snoop.get   <> txSnp
  io.chi.tx.data.get    <> txDat
  io.chi.tx.resp.get    <> txRsp


  /*
   * for Debug
   */
  val entrys_dbg_addr = Wire(Vec(param.nrEntry, UInt(fullAddrBits.W)))
  entrys_dbg_addr.zipWithIndex.foreach { case (addr, i) => addr := entrys(i).fullAddr(io.pcuID) }
  if (p(DebugOptionsKey).EnableDebug) {
    dontTouch(entrys_dbg_addr)
  }

val hwaFlags = Array.fill(33)(Wire(Bool()))
for (i <- 0 until 33) {
  hwaFlags(i) := true.B
}

// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------  Update Entry Value --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Update chiIdex
   */
  entrys.map(_.chiIndex).zipWithIndex.foreach {
    case (chiIdx, i) =>
      // Receive New Req
      when((rxReq.fire | io.req2Intf.fire | io.resp2Intf.fire) & entryFreeID === i.U) {
        chiIdx          := entrySave.chiIndex
        hwaFlags(0) := entrys(i).state === RSState.Free
      }

      HardwareAssertion(hwaFlags(0), cf"RNSLV ENTRY[0x${i}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)

  }
  HardwareAssertion.placePipe(2)

  /*
   * Update chiMes
   */
  entrys.map(_.chiMes).zipWithIndex.foreach {
    case (chiMes, i) =>
      val hitRespDat    = rxDat.fire & entryRecChiDatID === i.U
      val hitRespRsp    = rxRsp.fire & entryRecChiRspID === i.U & !rspIsDMTComp
      // Receive New Req
      when((rxReq.fire | io.req2Intf.fire | io.resp2Intf.fire) & entryFreeID === i.U) {
        chiMes          := entrySave.chiMes
      // Receive CHI TX Dat or CHI TX Rsp
      }.elsewhen(hitRespDat | hitRespRsp) {
        // get resp
        chiMes.resp     := Mux(hitRespDat, rxDat.bits.Resp, Mux(hitRespRsp & !chiMes.retToSrc & rxRsp.bits.Opcode =/= CompAck, rxRsp.bits.Resp, chiMes.resp))
        // get fwd resp
        val isFwdResp   = Mux(hitRespDat, rxDat.bits.Opcode === SnpRespDataFwded, rxRsp.bits.Opcode === SnpRespFwded)
        val fwdState    = Mux(hitRespDat, rxDat.bits.FwdState, rxRsp.bits.FwdState)
        chiMes.fwdState := Mux(isFwdResp, fwdState, chiMes.fwdState)
        // assert
        when(hitRespDat) {
          hwaFlags(1) := rxDat.bits.Opcode === SnpRespData | rxDat.bits.Opcode === SnpRespDataFwded | rxDat.bits.Opcode === CopyBackWriteData | rxDat.bits.Opcode === NonCopyBackWriteData
          hwaFlags(2) := Mux(chiMes.isSnp & chiMes.retToSrc, entrys(i).state === RSState.Snp2NodeIng | entrys(i).state === RSState.WaitSnpResp, entrys(i).state === RSState.WaitData)
        }
        when(!hitRespDat & hitRespRsp & (rxRsp.bits.Opcode === CompAck)) { 
          hwaFlags(3) := entrys(i).state === RSState.WaitCompAck | entrys(i).entryMes.snpFwdWaitAck
        }
        when(!hitRespDat & hitRespRsp & (!(rxRsp.bits.Opcode === CompAck))) { 
          hwaFlags(4) := entrys(i).state === RSState.Snp2NodeIng | entrys(i).state === RSState.WaitSnpResp
        }
      }

      HardwareAssertion(hwaFlags(1), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(2), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)

      HardwareAssertion(hwaFlags(3))
      HardwareAssertion(hwaFlags(4))
  }
  HardwareAssertion.placePipe(2)

  /*
   * Update pcuIndex
   */
  entrys.map(_.pcuIndex).zipWithIndex.foreach {
    case (pcuIdx, i) =>
      // Receive New Req
      when((rxReq.fire | io.req2Intf.fire | io.resp2Intf.fire) & entryFreeID === i.U) {
        pcuIdx          := entrySave.pcuIndex
      // Receive DBID From DataBuffer
      }.elsewhen(io.dbSigs.dbidResp.fire & io.dbSigs.dbidResp.bits.receive & entryRecDBID === i.U) {
        pcuIdx.dbID     := io.dbSigs.dbidResp.bits.dbID
        hwaFlags(5) := entrys(i).state === RSState.WaitDBID
      }

      HardwareAssertion(hwaFlags(5), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)

  }
  HardwareAssertion.placePipe(2)

  /*
   * Update entryMes
   */
  entrys.map(_.entryMes).zipWithIndex.foreach {
    case (entryMes, i) =>
      val hitRespDat            = rxDat.fire & entryRecChiDatID === i.U
      val hitRespRsp            = rxRsp.fire & entryRecChiRspID === i.U & !rspIsDMTComp
      val hitFDB                = io.dbSigs.dataFDB.fire & entryRecDataFDBID === i.U
      // ------------------------------------------- Update Base Values -------------------------------------------------- //
      // Receive New Req
      when((rxReq.fire | io.req2Intf.fire | io.resp2Intf.fire) & entryFreeID === i.U) {
        entryMes := entrySave.entryMes
      // Modify getBeatNum / snpFwdSVal / snpFwdWaitAck
      }.elsewhen(hitRespDat | hitRespRsp | hitFDB) {
        // getBeatNum
        entryMes.getBeatNum := entryMes.getBeatNum + hitRespDat.asUInt + hitFDB.asUInt
        // snpFwdSVal
        entryMes.snpFwdSVal := entryMes.snpFwdSVal | Mux(hitRespDat, rxDat.bits.Opcode === SnpRespDataFwded, rxRsp.bits.Opcode === SnpRespFwded)
        // snpFwdWaitAck
        entryMes.snpFwdWaitAck := Mux(hitRespRsp & rxRsp.bits.Opcode === CompAck, false.B, entryMes.snpFwdWaitAck)
        // assert
        hwaFlags(6) := !(hitRespDat & hitFDB)
        when(hitRespDat) {
          hwaFlags(7) := isWriteX(entrys(i).chiMes.opcode) | (entrys(i).chiMes.opcode === CompDBIDResp & entrys(i).chiMes.isRsp) | (entrys(i).chiMes.isReq & isAtomicX(entrys(i).chiMes.opcode)) | (entrys(i).chiMes.isSnp & entrys(i).chiMes.retToSrc)
        }.elsewhen(hitFDB) {
          hwaFlags(8) := entrys(i).isDatBeSend
        }.elsewhen(entrys(i).chiMes.isSnp & hitRespRsp & rxRsp.bits.Opcode === CompAck) {
          hwaFlags(9) := entryMes.snpFwdWaitAck
          hwaFlags(10) := getUseNodeID(rxRsp.bits.SrcID) === entrys(i).chiIndex.nodeID
        }
        // Receive DBID From DataBuffer
      }.elsewhen(io.dbSigs.dbidResp.fire & io.dbSigs.dbidResp.bits.receive & entryRecDBID === i.U) {
        entryMes.hasData := true.B
        hwaFlags(11) := !entryMes.hasData
        // Send ReadReceipt
      }.elsewhen(txRsp.fire & entrySendRspID === i.U & txRsp.bits.Opcode === ReadReceipt) {
        entryMes.needSendRRec := false.B
      }

      HardwareAssertion(hwaFlags(6), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(7), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(8), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(9), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(10), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(11), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)



      // ---------------------------------------------- Record Snp Resp --------------------------------------------------- //
      when(entrys(i).chiMes.isSnp & (entrys(i).state === RSState.Snp2NodeIng | entrys(i).state === RSState.WaitSnpResp)) {
        val rspHit        = rxRsp.fire & (rxRsp.bits.Opcode === SnpResp     | rxRsp.bits.Opcode === SnpRespFwded)     & entryRecChiRspID === i.U & !rspIsDMTComp
        val datHit        = rxDat.fire & (rxDat.bits.Opcode === SnpRespData | rxDat.bits.Opcode === SnpRespDataFwded) & entryRecChiDatID === i.U & entrys(i).isLastBeat
        val rspId         = getMetaIDByNodeID(rxRsp.bits.SrcID) 
        hwaFlags(12) := fromCcNode(rxRsp.bits.SrcID) | !rxRsp.valid
        val datId         = getMetaIDByNodeID(rxDat.bits.SrcID) 
        hwaFlags(13) := fromCcNode(rxDat.bits.SrcID) | !rxDat.valid
        val rspIdOH       = Mux(rspHit, UIntToOH(rspId), 0.U)
        val datIdOH       = Mux(datHit, UIntToOH(datId), 0.U)
        // Record GetSnpRespOH
        entryMes.getSnpRespVec := (entryMes.getSnpRespVec.asUInt | rspIdOH | datIdOH).asBools
        // assert
        hwaFlags(14) := Mux(rspHit, !entryMes.getSnpRespVec(rspId).asBool, true.B)
        hwaFlags(15) := Mux(datHit, !entryMes.getSnpRespVec(datId).asBool, true.B)
        hwaFlags(16) := Mux(rspHit, entryMes.snpTgtVec(rspId).asBool,      true.B)
        hwaFlags(17) := Mux(datHit, entryMes.snpTgtVec(datId).asBool,      true.B)
      }

      HardwareAssertion(hwaFlags(14), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(15), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(16),cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(17),cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)

  }
  HardwareAssertion.placePipe(2)




// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------  State Transfer -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.map(_.state).zipWithIndex.foreach {
    case(state, i) =>
      switch(state) {
        // State: Free
        is(RSState.Free) {
          val hit         = entryFreeID === i.U
          val isOthResp   = io.resp2Intf.fire & !(io.resp2Intf.bits.chiMes.opcode === CompDBIDResp & io.resp2Intf.bits.chiMes.isRsp) // Expect of CompDBIDResp
          val isDBIDResp  = io.resp2Intf.fire &   io.resp2Intf.bits.chiMes.opcode === CompDBIDResp & io.resp2Intf.bits.chiMes.isRsp; assert(Mux(io.resp2Intf.fire & io.resp2Intf.bits.chiMes.isRsp, io.resp2Intf.bits.chiMes.opcode =/= DBIDResp, true.B))
          val isSnp       = io.req2Intf.fire
          val snpGetDBID  = io.req2Intf.bits.pcuMes.snpNeedDB; 
          val isRead      = rxReq.fire & isReadX(rxReq.bits.Opcode)
          val isDataLess  = rxReq.fire & isDatalessX(rxReq.bits.Opcode)
          val isCB        = rxReq.fire & isCBX(rxReq.bits.Opcode)
          val isWrite     = rxReq.fire & isWriteX(rxReq.bits.Opcode) & !isCBX(rxReq.bits.Opcode)
          val isAtomic    = rxReq.fire & isAtomicX(rxReq.bits.Opcode)
          when(hit) {
            state := Mux(isOthResp, RSState.Resp2Node,
                      Mux(isDBIDResp, RSState.GetDBID,
                        Mux(isSnp & snpGetDBID, RSState.GetDBID,
                          Mux(isSnp & !snpGetDBID, RSState.Snp2Node,
                            Mux(isRead | isDataLess | isCB, RSState.Req2Exu,
                              Mux(isWrite, RSState.GetDBID,
                                Mux(isAtomic, RSState.GetDBID, state)))))))
          }
          hwaFlags(18) := PopCount(Seq(isOthResp, isDBIDResp, isSnp, isRead, isDataLess, isCB, isWrite, isAtomic)) <= 1.U
          hwaFlags(19) := Mux(io.resp2Intf.fire, isOthResp | isDBIDResp, true.B)
          hwaFlags(20) := Mux(rxReq.fire, isRead | isDataLess | isCB | isWrite | isAtomic, true.B)

        }
        // State: Req2Exu
        is(RSState.Req2Exu) {
          val hit       = io.req2Exu.fire & entrySendReqID === i.U
          state         := Mux(hit, RSState.WaitExuAck, state)
        }
        // State: WaitExuAck
        is(RSState.WaitExuAck) {
          val hit       = io.reqAck2Intf.fire & io.reqAck2Intf.bits.entryID === i.U
          val isWriUni  = entrys(i).chiMes.isReq & isWriUniX(entrys(i).chiMes.opcode)
          state         := Mux(hit, Mux(io.reqAck2Intf.bits.retry, RSState.Req2Exu, // Retry
                                      Mux(entrys(i).entryMes.needSendRRec, RSState.WaitSendRRec, // ReadOnce
                                        Mux(isWriUni, RSState.Resp2Node, // WriteUniqueX
                                          RSState.Free))), state) // Other
        }
        // State: WaitSendRRec
        is(RSState.WaitSendRRec) {
          val hit = !entrys(i).entryMes.needSendRRec | (txRsp.fire & entrySendRspID === i.U)
          hwaFlags(21) := Mux(txRsp.fire & entrySendRspID === i.U, txRsp.bits.Opcode === ReadReceipt, true.B)
          state := Mux(hit, RSState.Free, state)
        }
        // State: Resp2Node
        is(RSState.Resp2Node) {
          val txDatHit  = txDat.fire & entryRecDataFDBID === i.U & entrys(i).isLastBeat
          val txRspHit  = txRsp.fire & entrySendRspID === i.U
          hwaFlags(22) := txRsp.bits.Opcode === Comp | !txRspHit
          val expAck    = entrys(i).chiMes.expCompAck
          state         := Mux(txDatHit | txRspHit, Mux(expAck, RSState.WaitCompAck, RSState.Free), state)
        }
        // State: WaitCompAck
        is(RSState.WaitCompAck) {
          val hit       = rxRsp.fire & rxRsp.bits.TxnID === i.U
          hwaFlags(23) := Mux(hit, rxRsp.bits.Opcode === CompAck, true.B)
          state         := Mux(hit, RSState.Free, state)
        }
        // State: GetDBID
        is(RSState.GetDBID) {
          val hit       = io.dbSigs.getDBID.fire & entryGetDBID === i.U
          hwaFlags(24) := Mux(hit, entrys(i).entryMes.nID === 0.U, true.B)
          state         := Mux(hit, RSState.WaitDBID, state)
        }
        // State: WaitDBID
        is(RSState.WaitDBID) {
          val hit       = io.dbSigs.dbidResp.fire & entryRecDBID === i.U
          val rec       = io.dbSigs.dbidResp.bits.receive
          state         := Mux(hit, Mux(rec, Mux(entrys(i).chiMes.isSnp, RSState.Snp2Node, RSState.DBIDResp2Node), RSState.GetDBID), state)
        }
        // State: DBIDResp2Node
        is(RSState.DBIDResp2Node) {
          val hit       = txRsp.fire & entrySendRspID === i.U
          // assert(txRsp.bits.Opcode === CompDBIDResp | txRsp.bits.Opcode === DBIDResp | !hit)
          hwaFlags(25) := txRsp.bits.Opcode === CompDBIDResp | txRsp.bits.Opcode === DBIDResp | !hit
          state         := Mux(hit, RSState.WaitData, state)
        }
        // State: WaitData
        is(RSState.WaitData) {
          val hit       = rxDat.fire & rxDat.bits.TxnID === i.U
          state         := Mux(hit & entrys(i).isLastBeat, Mux(entrys(i).chiMes.isReq, RSState.Req2Exu, RSState.Resp2Exu), state)
        }
        // State: Snp2Node
        is(RSState.Snp2Node) {
          val hit       = txSnp.fire & entrySendSnpID === i.U
          state         := Mux(hit, Mux(snpIsLast, RSState.WaitSnpResp, RSState.Snp2NodeIng), state)
        }
        // State: Snp2NodeIng
        is(RSState.Snp2NodeIng) {
          val hit       = txSnp.fire & entrySendSnpID === i.U
          state         := Mux(hit & snpIsLast, RSState.WaitSnpResp, state)
        }
        // State: WaitSnpResp
        is(RSState.WaitSnpResp) {
          val rspHit    = rxRsp.fire & entryRecChiRspID === i.U & (rxRsp.bits.Opcode === SnpResp     | rxRsp.bits.Opcode === SnpRespFwded) & !rspIsDMTComp
          val datHit    = rxDat.fire & entryRecChiDatID === i.U & (rxDat.bits.Opcode === SnpRespData | rxDat.bits.Opcode === SnpRespDataFwded)
          val shlGetNum = PopCount(entrys(i).entryMes.getSnpRespVec.asUInt ^ entrys(i).entryMes.snpTgtVec.asUInt)
          val nowGetNum = rspHit.asTypeOf(UInt(2.W)) + (datHit & entrys(i).isLastBeat).asTypeOf(UInt(2.W))
          state         := Mux(shlGetNum === nowGetNum, RSState.Resp2Exu, state)
        }
        // State: Resp2Exu
        is(RSState.Resp2Exu) {
          val hit       = io.resp2Exu.fire & entryResp2ExuID === i.U & !dmtCompVal
          state         := Mux(hit, RSState.Free, state)
        }
      }

      HardwareAssertion(hwaFlags(21), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(22), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(23), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(24))
      HardwareAssertion(hwaFlags(25))
      HardwareAssertion(hwaFlags(26), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)
      HardwareAssertion(hwaFlags(27), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}]", i.U, entrys(i).state)

      HardwareAssertion(hwaFlags(28))
    }
    HardwareAssertion.placePipe(2)


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- Get Entry NID -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  // Get Addr With DcuID
  val snp2IntfAWD         = io.req2Intf.bits.addrWithDcuID
  val resp2ExuAWD         = entrys(entryResp2ExuID).addrWithDcuID
  val reqAckAWD           = entrys(io.reqAck2Intf.bits.entryID).addrWithDcuID
  val compAckAWD          = entrys(entryRecChiRspID).addrWithDcuID

  // [Set New NID] Match Vec
  snpMatchVec             := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === entrySave.addrWithDcuID & (e.chiMes.isRsp | e.chiMes.isDat) & e.chiMes.expCompAck }
  reqMatchVec             := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === entrySave.addrWithDcuID & (e.chiMes.isReq | e.chiMes.isSnp) }
  val reqMatchNum_g       = RegNext(PopCount(reqMatchVec))
  val snpMatchNum_g       = RegNext(PopCount(snpMatchVec))

  // [Set New NID] Match Req        | Addr With DcuID                                  | Fire                         | other
  val reqAckMatchReq_g    = RegNext(reqAckAWD   === entrySave.addrWithDcuID) & RegNext(io.reqAck2Intf.fire) & RegNext(!io.reqAck2Intf.bits.retry)
  val compAckMatchReq_g   = RegNext(compAckAWD  === entrySave.addrWithDcuID) & RegNext(rxRsp.fire)          & RegNext(rxRsp.bits.Opcode === CompAck & !dmtCompVal)
  val snpRespMatchReq_g   = RegNext(resp2ExuAWD === entrySave.addrWithDcuID) & RegNext(io.resp2Exu.fire)    & RegNext(!dmtCompVal & io.resp2Exu.bits.pcuMes.isSnpResp)

  // [Set New NID] Judgement
  val entryFreeID_g       = RegNext(entryFreeID)
  val setRespNID_g        = RegNext(io.resp2Intf.fire)
  val setSnpNID_g         = RegNext(io.req2Intf.fire)
  val setReqNID_g         = RegNext(rxReq.fire)

  // [Modify NID] Modify Vec
  snp2IntfHitVec_g        := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === snp2IntfAWD  & io.req2Intf.fire }
  snpResp2ExuHitVec_g     := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === resp2ExuAWD  & io.resp2Exu.fire    & !dmtCompVal                   & io.resp2Exu.bits.pcuMes.isSnpResp }
  reqAckHitVec_g          := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === reqAckAWD    & io.reqAck2Intf.fire & !io.reqAck2Intf.bits.retry    & io.reqAck2Intf.bits.entryID =/= i.U }
  compAckHitVec_g         := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === compAckAWD   & rxRsp.fire          & rxRsp.bits.Opcode === CompAck & !dmtCompVal & entryRecChiRspID =/= i.U }

  // [Modify NID] Judgement
  entrysValVec_g          := entrys.map(!_.isFree)
  entrysIsSnpVec_g        := entrys.map(_.chiMes.isSnp)
  entrysIsReqVec_g        := entrys.map(_.chiMes.isReq)


  entrys.map(_.entryMes.nID).zipWithIndex.foreach {
    case(nID, i) =>
      /*
      * Set New NID
      */
      when(entryFreeID_g === i.U) {
        // Resp always takes priority
        when(setRespNID_g)  { 
          nID := 0.U 
        }
        // Snp
        .elsewhen(setSnpNID_g)  { 
          nID := snpMatchNum_g - compAckMatchReq_g                                              
          hwaFlags(29) := snpMatchNum_g >= compAckMatchReq_g.asUInt
          }
        // Req
        .elsewhen(setReqNID_g)  { 
          nID := reqMatchNum_g - snpRespMatchReq_g - reqAckMatchReq_g                            
          hwaFlags(30) := reqMatchNum_g >= snpRespMatchReq_g.asTypeOf(UInt(2.W)) + reqAckMatchReq_g
          }
        }
      /*
      * Modify NID
      */
      .elsewhen(entrysValVec_g(i)) {
        // Snp
        when(entrysIsSnpVec_g(i)){ 
          nID := nID - compAckHitVec_g(i)
          hwaFlags(31) := nID >= compAckHitVec_g(i)
        }
        // Req
        .elsewhen(entrysIsReqVec_g(i))  { 
          nID := nID + snp2IntfHitVec_g(i) - snpResp2ExuHitVec_g(i) - reqAckHitVec_g(i)
          hwaFlags(32) := nID + snp2IntfHitVec_g(i) >= snpResp2ExuHitVec_g(i).asTypeOf(UInt(2.W))  + reqAckHitVec_g(i)
        }
      }

      HardwareAssertion(hwaFlags(29), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}] NID[0x${nID}]", i.U, entrys(i).state, nID)
      HardwareAssertion(hwaFlags(30), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}] NID[0x${nID}]", i.U, entrys(i).state, nID)
      HardwareAssertion(hwaFlags(31), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}] NID[0x${nID}]", i.U, entrys(i).state, nID)
      HardwareAssertion(hwaFlags(32), cf"RNSLV ENTRY[0x${i.U}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] STATE[0x${entrys(i).state}] NID[0x${nID}]", i.U, entrys(i).state, nID)

  }
  HardwareAssertion.placePipe(2)



// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------- Receive Req From CHITXREQ, Req2Node From EXU or Resp From EXU---------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get ENTRY Free ID
   */
  val entryFreeVec  = entrys.map(_.isFree)
  val entryFreeNum  = PopCount(entryFreeVec)
  entryFreeID       := PriorityEncoder(entryFreeVec)

  /*
   * Receive req2Intf(Snoop)
   */
  val reqVal        = rxReq.valid
  val snpVal        = io.req2Intf.valid; HardwareAssertion(Mux(snpVal, io.req2Intf.bits.chiMes.isSnp, true.B))
  val respVal       = io.resp2Intf.valid
  //                                              | RESP(resp2Intf)                                 | SNP(req2Intf)                             | REQ(rxReq)
  entrySave.entryMes.useAddr      := Mux(respVal, io.resp2Intf.bits.pcuMes.useAddr,     Mux(snpVal, io.req2Intf.bits.pcuMes.useAddr,            parseFullAddr(rxReq.bits.Addr)._6))
  entrySave.entryMes.dcuID        := Mux(respVal, io.resp2Intf.bits.from,               Mux(snpVal, io.req2Intf.bits.from,                      parseFullAddr(rxReq.bits.Addr)._3))
  entrySave.entryMes.snpFwdWaitAck:= Mux(respVal, false.B,                              Mux(snpVal, isSnpXFwd(io.req2Intf.bits.chiMes.opcode),  false.B))
  entrySave.entryMes.needSendRRec := Mux(respVal, false.B,                              Mux(snpVal, false.B,                                    rxReq.bits.Opcode === ReadOnce))
  entrySave.entryMes.swapFst      := Mux(respVal, false.B,                              Mux(snpVal, false.B,                                    rxReq.bits.Addr(offsetBits -1 , 0)(rxReq.bits.Size).asBool))
  entrySave.entryMes.hasData      := Mux(respVal, io.resp2Intf.bits.chiMes.isDat,       Mux(snpVal, io.req2Intf.bits.pcuMes.hasPcuDBID,         false.B))
  entrySave.entryMes.snpTgtVec    := Mux(respVal, DontCare,                             Mux(snpVal, io.req2Intf.bits.pcuMes.snpTgtVec,          DontCare))
  entrySave.entryMes.reqIsWrite   := Mux(respVal, false.B,                              Mux(snpVal, false.B,                                    isWriteX(rxReq.bits.Opcode)))
  entrySave.entryMes.reqIsAtomic  := Mux(respVal, false.B,                              Mux(snpVal, false.B,                                    isAtomicX(rxReq.bits.Opcode)))
  entrySave.pcuIndex.mshrWay      := Mux(respVal, io.resp2Intf.bits.pcuIndex.mshrWay,   Mux(snpVal, io.req2Intf.bits.pcuIndex.mshrWay,          DontCare))
  entrySave.pcuIndex.dbID         := Mux(respVal, io.resp2Intf.bits.pcuIndex.dbID,      Mux(snpVal, io.req2Intf.bits.pcuIndex.dbID,             0.U))
  entrySave.chiIndex.txnID        := Mux(respVal, io.resp2Intf.bits.chiIndex.txnID,     Mux(snpVal, io.req2Intf.bits.chiIndex.txnID,            rxReq.bits.TxnID))
  entrySave.chiIndex.nodeID       := Mux(respVal, io.resp2Intf.bits.chiIndex.nodeID,    Mux(snpVal, io.req2Intf.bits.chiIndex.nodeID,           getUseNodeID(rxReq.bits.SrcID)))
  entrySave.chiIndex.size         := Mux(respVal, io.resp2Intf.bits.chiIndex.size,      Mux(snpVal, chiFullSize.U,                              rxReq.bits.Size))
  entrySave.chiIndex.offset       := Mux(respVal, io.resp2Intf.bits.chiIndex.offset,    Mux(snpVal, 0.U,                                        parseFullAddr(rxReq.bits.Addr)._5))
  entrySave.chiMes.opcode         := Mux(respVal, io.resp2Intf.bits.chiMes.opcode,      Mux(snpVal, io.req2Intf.bits.chiMes.opcode,             rxReq.bits.Opcode))
  entrySave.chiMes.retToSrc       := Mux(respVal, DontCare,                             Mux(snpVal, io.req2Intf.bits.chiMes.retToSrc,           DontCare))
  entrySave.chiMes.doNotGoToSD    := Mux(respVal, DontCare,                             Mux(snpVal, io.req2Intf.bits.chiMes.doNotGoToSD,        DontCare))
  entrySave.chiMes.channel        := Mux(respVal, io.resp2Intf.bits.chiMes.channel,     Mux(snpVal, CHIChannel.SNP,                             CHIChannel.REQ))
  entrySave.chiMes.resp           := Mux(respVal, io.resp2Intf.bits.chiMes.resp,        Mux(snpVal, 0.U,                                        0.U))
  entrySave.chiMes.expCompAck     := Mux(respVal, io.resp2Intf.bits.chiMes.expCompAck,  Mux(snpVal, false.B,                                    rxReq.bits.ExpCompAck))
  HardwareAssertion(Mux(snpVal, io.req2Intf.bits.chiIndex.fullSize, true.B))
  HardwareAssertion(Mux(snpVal, io.req2Intf.bits.chiMes.isSnp, true.B))
  HardwareAssertion(Mux(snpVal, io.req2Intf.bits.chiIndex.offset === 0.U, true.B))


  /*
   * Set Ready Value
   */
  rxReq.ready           := entryFreeNum >= param.nrEvictEntry.U & !io.req2Intf.valid & !io.resp2Intf.valid
  io.req2Intf.ready     := entryFreeNum > 0.U & !io.resp2Intf.valid
  io.resp2Intf.ready    := entryFreeNum > 0.U
  io.reqAck2Intf.ready  := true.B


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- Select Entry send Req to EXU --------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Select one Entry
   */
  val reqBeSendVec  = entrys.map(_.isReqBeSend)
  entrySendReqID    := StepRREncoder(reqBeSendVec, io.req2Exu.ready)

  /*
   * Send Req To Node
   */
  io.req2Exu.valid                      := reqBeSendVec.reduce(_ | _)
  io.req2Exu.bits                       := DontCare
  io.req2Exu.bits.chiMes.channel        := entrys(entrySendReqID).chiMes.channel
  io.req2Exu.bits.chiMes.opcode         := entrys(entrySendReqID).chiMes.opcode
  io.req2Exu.bits.chiMes.expCompAck     := entrys(entrySendReqID).chiMes.expCompAck
  io.req2Exu.bits.chiMes.resp           := entrys(entrySendReqID).chiMes.resp
  io.req2Exu.bits.chiIndex.nodeID       := entrys(entrySendReqID).chiIndex.nodeID
  io.req2Exu.bits.chiIndex.txnID        := entrys(entrySendReqID).chiIndex.txnID
  io.req2Exu.bits.chiIndex.size         := entrys(entrySendReqID).chiIndex.size
  io.req2Exu.bits.chiIndex.offset       := entrys(entrySendReqID).chiIndex.offset
  io.req2Exu.bits.pcuMes.useAddr        := entrys(entrySendReqID).entryMes.useAddr
  // pcuIndex
  io.req2Exu.bits.to                    := entrys(entrySendReqID).entryMes.dcuID
  io.req2Exu.bits.from                  := param.intfID.U
  io.req2Exu.bits.pcuIndex.entryID      := entrySendReqID
  io.req2Exu.bits.pcuIndex.dbID         := entrys(entrySendReqID).pcuIndex.dbID



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- Select Entry send Resp to Node ------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Select one Entry to Send RxDat
   */
  val datBeSendVec   = entrys.map { case p => p.isDatBeSend & io.dbSigs.dataFDB.valid & p.pcuIndex.dbID === io.dbSigs.dataFDB.bits.dbID }
  entryRecDataFDBID  := PriorityEncoder(datBeSendVec)
  HardwareAssertion(PopCount(datBeSendVec) <= 1.U)

  txDat.valid        := datBeSendVec.reduce(_ | _)
  txDat.bits         := DontCare
  txDat.bits.Opcode  := entrys(entryRecDataFDBID).chiMes.opcode
  txDat.bits.TgtID   := getFullNodeID(entrys(entryRecDataFDBID).chiIndex.nodeID)
  txDat.bits.SrcID   := io.hnfID
  txDat.bits.TxnID   := entrys(entryRecDataFDBID).chiIndex.txnID
  txDat.bits.HomeNID := io.hnfID
  txDat.bits.DBID    := entryRecDataFDBID
  txDat.bits.Resp    := entrys(entryRecDataFDBID).chiMes.resp

  io.dbSigs.dataFDB.ready := txDat.ready & datBeSendVec.reduce(_ | _)


  /*
   * Select one Entry to Send RxRsp
   */
  val rspBeSendVec   = entrys.map { case p => p.isRspBeSend }
  entrySendRspID     := PriorityEncoder(rspBeSendVec)

  txRsp.valid        := rspBeSendVec.reduce(_ | _)
  txRsp.bits         := DontCare
  txRsp.bits.Opcode  := Mux(entrys(entrySendRspID).chiMes.isRsp, entrys(entrySendRspID).chiMes.opcode,  // Resp from exu
                          Mux(isReadX(entrys(entrySendRspID).chiMes.opcode), ReadReceipt,               // ReadOnce
                            Mux(entrys(entrySendRspID).state === RSState.DBIDResp2Node, Mux(isWriUniX(entrys(entrySendRspID).chiMes.opcode) | isAtomicX(entrys(entrySendRspID).chiMes.opcode), DBIDResp, CompDBIDResp), // Write or CopyBack Send DBIDResp
                              Comp))) // Write Send Comp
  txRsp.bits.TgtID   := getFullNodeID(entrys(entrySendRspID).chiIndex.nodeID)
  txRsp.bits.SrcID   := io.hnfID
  txRsp.bits.TxnID   := entrys(entrySendRspID).chiIndex.txnID
  txRsp.bits.DBID    := entrySendRspID
  txRsp.bits.Resp    := entrys(entrySendRspID).chiMes.resp



// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------------------- Receive Rsp Or Dat From Node ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Entry ID
   */
  entryRecChiRspID                := rxRsp.bits.TxnID(param.entryIdBits - 1, 0)
  entryRecChiDatID                := rxDat.bits.TxnID(param.entryIdBits - 1, 0)

  /*
   * Send Data To DataBuffer
   */
  io.dbSigs.dataTDB.valid         := rxDat.valid
  io.dbSigs.dataTDB.bits.dbID     := entrys(entryRecChiDatID).pcuIndex.dbID
  io.dbSigs.dataTDB.bits.data     := rxDat.bits.Data
  io.dbSigs.dataTDB.bits.dataID   := Mux(entrys(entryRecChiDatID).chiIndex.secBeat, "b10".U, rxDat.bits.DataID)
  io.dbSigs.dataTDB.bits.mask     := rxDat.bits.BE
  io.dbSigs.dataTDB.bits.atomicVal:= entrys(entryRecChiDatID).entryMes.reqIsAtomic

  /*
   * Set ready value
   */
  rspIsDMTComp              := rxRsp.bits.TxnID(chiTxnIdBits - 1).asBool; HardwareAssertion(Mux(rxRsp.valid & rspIsDMTComp, rxRsp.bits.Opcode === CompAck, true.B))
  dmtCompVal                := rxRsp.valid & rspIsDMTComp
  rxRsp.ready               := Mux(rspIsDMTComp, io.resp2Exu.ready, true.B)
  rxDat.ready               := io.dbSigs.dataTDB.ready


// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- Get DBID From DataBuffer and Wait DataBuffer Resp ---------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Send Get DBID Req To From DataBuffer
   */
  val entryGetDBIDVec         = entrys.map(_.isGetDBID)
  entryGetDBID                := StepRREncoder(entryGetDBIDVec, io.dbSigs.getDBID.ready)

  /*
   * Set DataBuffer Req Value
   */
  io.dbSigs.getDBID.valid             := entryGetDBIDVec.reduce(_ | _)
  io.dbSigs.getDBID.bits.from         := param.intfID.U
  io.dbSigs.getDBID.bits.entryID      := entryGetDBID
  io.dbSigs.getDBID.bits.reqIsWrite   := entrys(entryGetDBID).entryMes.reqIsWrite
  io.dbSigs.getDBID.bits.reqIsAtomic  := entrys(entryGetDBID).entryMes.reqIsAtomic
  io.dbSigs.getDBID.bits.atomicOp     := getAtomicOp(entrys(entryGetDBID).chiMes.opcode)
  io.dbSigs.getDBID.bits.swapFst      := entrys(entryGetDBID).entryMes.swapFst

  /*
   * Receive DBID From DataBuffer
   */
  entryRecDBID              := io.dbSigs.dbidResp.bits.entryID
  io.dbSigs.dbidResp.ready  := true.B



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- Select Entry send Snp to Node -------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Entry ID
   */
  val entrySendSnpVec     = entrys.map(_.isSendSnp)
  val entrySendSnpIngVec  = entrys.map(_.isSendSnpIng)
  entrySendSnpID          := Mux(entrySendSnpIngVec.reduce(_ | _), PriorityEncoder(entrySendSnpIngVec), PriorityEncoder(entrySendSnpVec))

  /*
   * Get Tgt ID
   */
  val snpShouldSendOH     = entrys(entrySendSnpID).entryMes.snpTgtVec.asUInt
  val snpBeSendOH         = snpShouldSendOH ^ snpAlrSendOHReg
  val snpTgtID            = getNodeIDByMetaID(PriorityEncoder(snpBeSendOH)); HardwareAssertion(PriorityEncoder(snpBeSendOH) < nrCcNode.U | !txSnp.valid)
  snpIsLast               := PopCount(snpBeSendOH) === 1.U; dontTouch(snpIsLast)
  snpAlrSendOHReg         := Mux(txSnp.fire, Mux(snpIsLast, 0.U, snpAlrSendOHReg | UIntToOH(getMetaIDByNodeID(snpTgtID))), snpAlrSendOHReg); HardwareAssertion(fromCcNode(snpTgtID) | !txSnp.valid)

  /*
   * Send Snp to Node
   */
  txSnp.valid             := entrySendSnpVec.reduce(_ | _) | entrySendSnpIngVec.reduce(_ | _)
  txSnp.bits              := DontCare
  txSnp.bits.Addr         := entrys(entrySendSnpID).snpAddr(io.pcuID)
  txSnp.bits.Opcode       := Mux(snpAlrSendOHReg === 0.U, entrys(entrySendSnpID).chiMes.opcode, getNoFwdSnpOp(entrys(entrySendSnpID).chiMes.opcode))
  txSnp.bits.TgtID        := snpTgtID
  txSnp.bits.SrcID        := io.hnfID
  txSnp.bits.TxnID        := entrySendSnpID
  txSnp.bits.FwdNID       := getFullNodeID(entrys(entrySendSnpID).chiIndex.nodeID)
  txSnp.bits.FwdTxnID     := entrys(entrySendSnpID).chiIndex.txnID
  txSnp.bits.RetToSrc     := entrys(entrySendSnpID).chiMes.retToSrc & snpAlrSendOHReg === 0.U
  txSnp.bits.DoNotGoToSD  := entrys(entrySendSnpID).chiMes.doNotGoToSD


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- Select Entry send Resp to EXU ----------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Select one Entry to Send RxDat
   */
  val respBeSendVec                     = entrys.map { case p => p.state === RSState.Resp2Exu & (!p.entryMes.snpFwdWaitAck | !p.entryMes.snpFwdSVal) }
  entryResp2ExuID                       := PriorityEncoder(respBeSendVec)


  val dmtMshrWay                        = rxRsp.bits.TxnID(mshrWayBits - 1, 0)
  val dmtMshrSet                        = rxRsp.bits.TxnID(mshrSetBits + mshrWayBits - 1, mshrWayBits)
  val dmtDcuID                          = rxRsp.bits.TxnID(dcuBankBits + mshrSetBits + mshrWayBits - 1, mshrSetBits + mshrWayBits)

  io.resp2Exu.valid                     := respBeSendVec.reduce(_ | _) | dmtCompVal
  io.resp2Exu.bits                      := DontCare
  io.resp2Exu.bits.from                 := param.intfID.U
  io.resp2Exu.bits.to                   := Mux(dmtCompVal, dmtDcuID,    entrys(entryResp2ExuID).entryMes.dcuID)
  io.resp2Exu.bits.pcuIndex.mshrSet     := Mux(dmtCompVal, dmtMshrSet,  entrys(entryResp2ExuID).entryMes.mSet)
  io.resp2Exu.bits.pcuIndex.mshrWay     := Mux(dmtCompVal, dmtMshrWay,  entrys(entryResp2ExuID).pcuIndex.mshrWay)
  io.resp2Exu.bits.pcuIndex.dbID        := Mux(dmtCompVal, 0.U,         entrys(entryResp2ExuID).pcuIndex.dbID)
  io.resp2Exu.bits.chiMes.resp          := Mux(dmtCompVal, 0.U,         entrys(entryResp2ExuID).chiMes.resp)
  io.resp2Exu.bits.chiMes.fwdState      := Mux(dmtCompVal, 0.U,         entrys(entryResp2ExuID).chiMes.fwdState)
  io.resp2Exu.bits.pcuMes.hasData       := Mux(dmtCompVal, false.B,     entrys(entryResp2ExuID).entryMes.hasData)
  io.resp2Exu.bits.pcuMes.fwdSVald      := Mux(dmtCompVal, false.B,     entrys(entryResp2ExuID).entryMes.snpFwdSVal)
  io.resp2Exu.bits.pcuMes.isSnpResp     := !dmtCompVal & entrys(entryResp2ExuID).chiMes.isSnp
  io.resp2Exu.bits.pcuMes.isWriResp     := !dmtCompVal & entrys(entryResp2ExuID).chiMes.isRsp
  io.resp2Exu.bits.pcuMes.isCompAck     := dmtCompVal

  HardwareAssertion(Mux(io.resp2Exu.fire & isSnpXFwd(entrys(entryResp2ExuID).chiMes.opcode) & entrys(entryResp2ExuID).chiMes.isSnp, entrys(entryResp2ExuID).entryMes.snpFwdSVal, true.B),
    cf"RNSLV ENTRY[0x${entryResp2ExuID}] STATE[0x${entrys(entryResp2ExuID).entryMes.state}] ADDR[0x${entrys(entryResp2ExuID).fullAddr(io.pcuID)}] CHANNEL[0x${entrys(entryResp2ExuID).chiMes.channel}] OP[0x${entrys(entryResp2ExuID).chiMes.opcode}] TIMEOUT", 

    entryResp2ExuID, entrys(entryResp2ExuID).entryMes.state, entrys(entryResp2ExuID).chiMes.opcode)



// ---------------------------  Assertion  -------------------------------- //
  val cntReg = RegInit(VecInit(Seq.fill(param.nrEntry) { 0.U(64.W) }))
  
  cntReg.zip(entrys).zipWithIndex.foreach { case((c, p), i) =>
  awhen(entrys(i).chiMes.isReq) {
    HardwareAssertion.checkTimeout(p.isFree | (io.reqAck2Intf.fire & io.reqAck2Intf.bits.entryID === i.U), TIMEOUT_RSINTF_REQ, cf"RNSLV ENTRY[0x${i.U}] STATE[0x${entrys(i).entryMes.state}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] CHANNEL[0x${entrys(i).chiMes.channel}] OP[0x${entrys(i).chiMes.opcode}] TIMEOUT")
  }
  .elsewhen(entrys(i).chiMes.isSnp) {
    HardwareAssertion.checkTimeout(p.isFree | (io.reqAck2Intf.fire & io.reqAck2Intf.bits.entryID === i.U), TIMEOUT_RSINTF_SNP, cf"RNSLV ENTRY[0x${i.U}] STATE[0x${entrys(i).entryMes.state}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] CHANNEL[0x${entrys(i).chiMes.channel}] OP[0x${entrys(i).chiMes.opcode}] TIMEOUT")
  }
  .otherwise {
    HardwareAssertion.checkTimeout(p.isFree | (io.reqAck2Intf.fire & io.reqAck2Intf.bits.entryID === i.U), TIMEOUT_RSINTF_RSP, cf"RNSLV ENTRY[0x${i.U}] STATE[0x${entrys(i).entryMes.state}] ADDR[0x${entrys(i).fullAddr(io.pcuID)}] CHANNEL[0x${entrys(i).chiMes.channel}] OP[0x${entrys(i).chiMes.opcode}] TIMEOUT")
  }
}
HardwareAssertion.placePipe(2)

  awhen(rxReq.valid) {
    // [1:cacheable] [2:ccxChipID] [3:dcuBank] [4:pcuBank] [5:offset] [6:useAddr]
    // cacheable
    HardwareAssertion(parseFullAddr(rxReq.bits.Addr)._1 === 0.U)
    // ccxChipID
    HardwareAssertion(parseFullAddr(rxReq.bits.Addr)._2 === 0.U)
    // pcuBank
    HardwareAssertion(parseFullAddr(rxReq.bits.Addr)._4 === io.pcuID)
    // offset
    HardwareAssertion(Mux(isAtomicX(rxReq.bits.Opcode), true.B, // Atomic
           Mux(isWriUniX(rxReq.bits.Opcode) | rxReq.bits.Opcode === ReadOnce, parseFullAddr(rxReq.bits.Addr)._5 === 0.U | parseFullAddr(rxReq.bits.Addr)._5 === 0x20.U, // WriteUnique or ReadOnce
               parseFullAddr(rxReq.bits.Addr)._5 === 0.U))) // Other
    // TgtID
    HardwareAssertion(rxReq.bits.TgtID === io.hnfID)
    // ExpCompAck
    HardwareAssertion(Mux(isReadX(rxReq.bits.Opcode) | isWriUniX(rxReq.bits.Opcode), rxReq.bits.ExpCompAck, true.B))
    // Order
    HardwareAssertion(Mux(isWriUniX(rxReq.bits.Opcode),    rxReq.bits.Order === Order.OWO & rxReq.bits.ExpCompAck,
           Mux(rxReq.bits.Opcode === ReadOnce,  rxReq.bits.Order === Order.EndpointOrder, rxReq.bits.Order === Order.None)))
    // Size
    HardwareAssertion(Mux(isWriXFull(rxReq.bits.Opcode),   Mux(rxReq.bits.Opcode === WriteBackFull, rxReq.bits.Size === chiFullSize.U, true.B), true.B))
    HardwareAssertion(Mux(isAtomicX(rxReq.bits.Opcode),    rxReq.bits.Size <= chiHalfSize.U, // Atomic
           Mux(rxReq.bits.Opcode === WriteBackPtl,    rxReq.bits.Size < chiFullSize.U, // WriteXPtl
           Mux(rxReq.bits.Opcode === ReadOnce,  rxReq.bits.Size <= chiFullSize.U, // ReadOnce
           Mux(rxReq.bits.Opcode === WriteUniquePtl, rxReq.bits.Size <= chiFullSize.U,
           Mux(rxReq.bits.Opcode === WriteUniqueFull, rxReq.bits.Size <= chiFullSize.U,
                                                rxReq.bits.Size === chiFullSize.U)))))) // Other
    // Endian
//    HardwareAssertion(rxReq.bits.Endian.asUInt === 0.U) // Must be Little Endian
  }

  awhen(rxDat.valid) {
    HardwareAssertion(
      Mux(
        rxDat.bits.Opcode === NonCopyBackWriteData,
        rxDat.bits.BE.asUInt > 0.U, // Atomic
        Mux(
          rxDat.bits.Opcode === CopyBackWriteData,
          Mux(
            rxDat.bits.Resp === ChiState.I,
            rxDat.bits.BE.asUInt === 0.U,
            PopCount(rxDat.bits.BE).asUInt === 32.U
          ),
          true.B // default
        )
      )
    )
  }


// -------------------------------------------------- Perf Counter ------------------------------------------------------ //
  val reqFire = rxReq.fire | io.req2Intf.fire | io.resp2Intf.fire
  require(param.nrEntry >= 4 & param.nrEntry % 4 == 0)
  for(i <- 0 until  (param.nrEntry/4)) {
    XSPerfAccumulate(s"pcu_localRnSlave_entry_group[${i}]_deal_req_cnt", reqFire & (i*4).U <= entryFreeID & entryFreeID <= (i*4+3).U)
  }
  XSPerfAccumulate("pcu_localRnSlave_req_cnt", reqFire)
  XSPerfAccumulate("pcu_localRnSlave_req_block_cnt", (rxReq.valid | io.req2Intf.valid | io.resp2Intf.valid) & entryFreeNum === 0.U)

  HardwareAssertion.placePipe(3)
}