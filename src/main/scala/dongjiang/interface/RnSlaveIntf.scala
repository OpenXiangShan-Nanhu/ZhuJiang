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
import dongjiang.utils.Encoder._
import xijiang.Node
import xs.utils._
import xs.utils.perf.{DebugOptions, DebugOptionsKey, HasPerfLogging}

/*
 * ************************************************************** State transfer ***********************************************************************************
 * Req Contain Read and Dataless
 * Req Retry:             [Free]  -----> [Req2Exu] -----> [WaitExuAck] ---retry---> [Req2Exu]
 * Req Receive:           [Free]  -----> [Req2Exu] -----> [WaitExuAck] --receive--> ([waitSendRRec]) -----> [Free]
 *
 *
 * Resp:                  [Free]  -----> [Resp2Node] -----> [WaitCompAck] -----> [Free]
 *
 *
 * Write Retry:           [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] ---retry---> [Req2Exu]
 * Write:                 [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] --receive--> [Resp2Node] -----> [WaitCompAck] -----> [Free]
 *
 *
 * Amotic Retry:          [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] ---retry---> [Req2Exu]
 * Amotic:                [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] --receive--> [Free]
 *
 *
 * CopyBack Retry:        [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] ---retry---> [Req2Exu]
 * CopyBack:              [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Exu] -----> [WaitExuAck] --receive--> [Free]
 * CopyBack Nest By Snp case:       ^                ^                 ^                      ^                 ^                ^
 *                              waitWBDone       waitWBDone        waitWBDone             waitWBDone  -----> trans2Snp        trans2Snp
 *
 *
 * Snoop Need Data:       [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [Snp2Node] -----> ([Snp2NodeIng]) -----> [WaitSnpResp] -----> [Resp2Exu]
 * Snoop No Need Data:    [Free]                   ----->                   [Snp2Node] -----> ([Snp2NodeIng]) -----> [WaitSnpResp] -----> [Resp2Exu]
 * Snoop Nest CopyBack 0: [XXXX]                   ----->                   [Snp2Node] -----> ([Snp2NodeIng]) -----> [WaitSnpResp] -----> [Resp2Exu]
 * Snoop Nest CopyBack 1: [XXXX]                   ----->                                                                                 [Resp2Exu] * only need to snp one node
 *                                                   ^
 *                                                trans2Snp
 *
 *
 *
 * ************************************************************** Entry Transfer ********************************************************************************
 * Req:
 * [------] <- Req0x0 | [Req0x0] -> Req to EXU   | [------] <- Resp from EXU   | [Rsp0x0] -> resp to node
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 *
 *
 * CopyBack:
 * [------] <- CB 0x0 | [CB 0x0] -> Wri to EXU   | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 *
 *
 * Write:
 * [------] <- Wri0x0 | [Wri0x0] -> Wri to EXU   | [------] <- CompAck from node | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 *
 * Snoop:
 * [Snp0x0] <- Snp0x0 | [Snp0x0] -> Snp to node  | [Snp0x0] <- Resp from node  | [------] -> resp to EXU
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 *
 *
 * Req Nest By Snp:
 * [------] <- Req0x0 | [Req0x0]                 | [Req0x0] need to wait snoop done
 * [------]           | [------] <-Snp0x0        | [Snp0x0]
 * [------]           | [------]                 | [------]
 * [------]           | [------]                 | [------]
 *
 *
 * CopyBack Nest By Snp:
 * [------] <- CB 0x0 | [CB 0x0] <-Snp0x0        | [CB 0x0] need to wait write done and transfer write to snoop
 * [------]           | [------]                 | [------]
 * [------]           | [------]                 | [------]
 * [------]           | [------]                 | [------]
 *
 *
 * Write Nest By Snp:
 * [------] <- Wri0x0 | [Wri0x0]                 | [Wri0x0] need to wait snoop done
 * [------]           | [------] <-Snp0x0        | [Snp0x0]
 * [------]           | [------]                 | [------]
 * [------]           | [------]                 | [------]
 *
 *
 *
 * ************************************************************** ID Transfer ********************************************************************************
 *
 * CHI:
 * { TgtID | SrcID | TxnID | DBID | FwdNID | FwdTxnID }
 *
 * chiIdx: CHI Index
 * { nodeID | txnID }
 *
 * pcuIdx: PCU Index
 * { from(incoID) | to(incoID) | entryID | mshrIdx(mshrWay | mshrSet) | dbID | dcuID }
 *
 *
 * Read / Dataless:
 * { Read / Dataless } TxReq  From CHI And Store In Intf          { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        |
 * { Req2Exu         } Req    From Intf And Send To EXU           { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALSLV } { pcuIdx.entryID = entryID }
 * { ReqAck          } ReqAck From EXU and Match With Entry ID                                                                                              | { pcuIdx.entryID == entryID }
 * { Resp2Intf       } Resp   From EXU And Store In Intf          { chiIdx.nodeID = tgtID } { chiIdx.txnID = txnID }                                        | { pcuIdx.dbID = pcuIdx.dbID }
 * { Comp(Data)      } RxRsp  Send To CHI                         { TgtID = chiIdx.nodeID } { TxnID = chiIdx.txnID } { HomeNID = hnfID } { DBID = entryID } |
 * { CompAck         } TxRsp  From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 *
 *
 * Read with DMT: TODO
 * { Read            } TxReq  From CHI And Store In Intf          { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        |
 * { Req2Exu         } Req    From Intf And Send To EXU           { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALSLV } { pcuIdx.entryID = entryID }
 * { ReqAck          } ReqAck From EXU and Match With Entry ID                                                                                              | { pcuIdx.entryID == entryID }
 * { CompAck         } TxRsp  From CHI And Send Resp To EXU                                                                                                 | { pcuIdx.to = TxnID.head } { pcuIdx.mshrIdx = TxnID.tail }
 *
 *
 * Write:
 * { Write           } TxReq  From CHI And Store In Intf          { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        |
 * { CompDBIDResp    } RxRsp  Send To CHI                         { TgtID = chiIdx.nodeID } { TxnID = chiIdx.txnID } { DBID = entryID }                     |
 * { CBWriteData     } TxRat  From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Req2Exu         } Req    From Intf And Send To EXU                                                                                                     | { pcuIdx.to = chiMes.dcuID } { pcuIdx.incfrom = LOCALSLV } { pcuIdx.entryID = entryID } { pcuIdx.dbID = pcuIdx.dbID }
 * { ReqAck          } ReqAck From EXU and Match With Entry ID                                                                                              | { pcuIdx.entryID == entryID }
 *
 *
 * Write with DWT: Not implemented in the system
 * { Write           } TxReq  From CHI And Store In Intf          { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        |
 * { Req2Exu         } Req    From Intf And Send To EXU           { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALSLV } { pcuIdx.entryID = entryID }
 * { ReqAck          } ReqAck From EXU and Match With Entry ID                                                                                              | { pcuIdx.entryID == entryID }
 *
 *
 * Snoop:
 * { Req2Node        } Req    From EXU And Store In Intf                                                                                                    | { pcuIdx.snpVec = idx.snpVec } { pcuIdx.mshrIdx = pcuIdx.mshrIdx }
 * { Snoop           } Req    Send To CHI                         { TgtID = snpID } { TxnID = entryID }                                                     |
 * { SnResp(Data)    } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Exu        } Resp   Send To EXU                                                                                                                   | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALSLV } { pcuIdx.mshrIdx = mshrIdx }
 *
 *
 * Snoop with DCT: TODO
 * { Req2Node        } Req    From EXU And Store In Intf        { chiIdx.nodeID =  chiIdx.nodeID } { chiIdx.txnID =  chiIdx.txnID }                         | { pcuIdx.snpVec = idx.snpVec } { pcuIdx.mshrIdx = pcuIdx.mshrIdx }
 * { Snoop           } Req    Send To CHI                         { TgtID = snpID } { TxnID = entryID }                                                     |
 * { SnResp          } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { SnoopFwd        } Req    Send To CHI                         { TgtID = snpID } { TxnID = entryID } { FwdNID = chiIdx.nodeID } { FwdTxnID = chiIdx.txnID }
 * { SnpResp(Data)Fwded } Resp From CHI And Match With Entry ID   { TxnID == entryID }                                                                      |
 * { Resp2Exu        } Resp   Send To EXU                                                                                                                   | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALSLV } { pcuIdx.mshrIdx = mshrIdx } { pcuIdx.dbID = pcuIdx.dbID }
 *
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
    val getSnpRespOH  = UInt(nrCcNode.W)
    val snpFwdWaitAck = Bool() // CompAck
    val needSendRRec  = Bool() // Need send ReadReceipt
    val swapFst       = Bool() // Only use in atomic
    val nestMes       = new Bundle {
      val waitWBDone  = Bool()
      val trans2Snp   = Bool()
    }
  }

  def state         = entryMes.state
  def isFree        = entryMes.state === RSState.Free
  def isReqBeSend   = entryMes.state === RSState.Req2Exu    & entryMes.nID === 0.U
  def isRspBeSend   = (entryMes.state === RSState.Resp2Node & (chiMes.isRsp | (chiMes.isReq & isWriUniX(chiMes.opcode)))) | entryMes.state === RSState.DBIDResp2Node | entryMes.needSendRRec
  def isDatBeSend   = entryMes.state === RSState.Resp2Node  & chiMes.isDat
  def isGetDBID     = entryMes.state === RSState.GetDBID    & entryMes.nID === 0.U
  def isSendSnp     = entryMes.state === RSState.Snp2Node   & entryMes.nID === 0.U
  def isSendSnpIng  = entryMes.state === RSState.Snp2NodeIng
  def isLastBeat    = Mux(chiIndex.fullSize, entryMes.getBeatNum === 1.U, entryMes.getBeatNum === 0.U)
  def fullAddr(p: UInt) = entryMes.fullAddr(entryMes.dcuID, p)
  def snpAddr (p: UInt) = entryMes.snpAddr(entryMes.dcuID, p)
  def addrWithDcuID     = Cat(entryMes.useAddr, entryMes.dcuID)
}




class RnSlaveIntf(param: InterfaceParam, node: Node)(implicit p: Parameters) extends IntfBaseIO(param, node) with HasPerfLogging  {
  // Del it
  io <> DontCare
  dontTouch(io)
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
  // req/resp from EXU or rxReq
  val entrySave         = WireInit(0.U.asTypeOf(new RSEntry(param)))
  // snp to node
  val snpIsLast         = Wire(Bool())
  val snpAlreadySendVecReg = RegInit(0.U(nrCcNode.W))
  // CompAck is generated by DMT
  val rspIsDMTComp      = Wire(Bool())
  val dmtCompVal        = Wire(Bool())
  // HitVec
  val snp2IntfHitVec    = Wire(Vec(param.nrEntry, Bool())); dontTouch(snp2IntfHitVec)
  val resp2ExuHitVec    = Wire(Vec(param.nrEntry, Bool())); dontTouch(resp2ExuHitVec)
  val reqAckHitVec      = Wire(Vec(param.nrEntry, Bool())); dontTouch(reqAckHitVec)
  val compAckHitVec     = Wire(Vec(param.nrEntry, Bool())); dontTouch(compAckHitVec)
  // MatchVec
  val reqMatchVec       = Wire(Vec(param.nrEntry, Bool())); dontTouch(reqMatchVec)
  val snpMatchVec       = Wire(Vec(param.nrEntry, Bool())); dontTouch(snpMatchVec)

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
        assert(entrys(i).state === RSState.Free, "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
      }
  }

  /*
   * Update chiMes
   */
  entrys.map(_.chiMes).zipWithIndex.foreach {
    case (chiMes, i) =>
      // Receive New Req
      when((rxReq.fire | io.req2Intf.fire | io.resp2Intf.fire) & entryFreeID === i.U) {
        chiMes          := entrySave.chiMes
      // Receive CHI TX Dat or CHI TX Rsp
      }.elsewhen((rxDat.fire & entryRecChiDatID === i.U) | (rxRsp.fire & entryRecChiRspID === i.U & !rspIsDMTComp)) {
        val hitRespDat  = rxDat.fire & entryRecChiDatID === i.U
        val hitRespRsp  = rxRsp.fire & entryRecChiRspID === i.U & !rspIsDMTComp
        val fwdState     = Mux(hitRespDat, rxDat.bits.FwdState, rxRsp.bits.FwdState)
        chiMes.resp     := Mux(hitRespDat, rxDat.bits.Resp, Mux(hitRespRsp & !chiMes.retToSrc & rxRsp.bits.Opcode =/= CompAck, rxRsp.bits.Resp, chiMes.resp))
        chiMes.fwdState := Mux(fwdState > ChiResp.I, fwdState, chiMes.fwdState)
        when(hitRespDat) {
          assert(rxDat.bits.Opcode === SnpRespData | rxDat.bits.Opcode === CopyBackWriteData, "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
          assert(Mux(chiMes.isSnp & chiMes.retToSrc, entrys(i).state === RSState.Snp2NodeIng | entrys(i).state === RSState.WaitSnpResp, entrys(i).state === RSState.WaitData), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
        }.elsewhen(hitRespRsp) {
          when(rxRsp.bits.Opcode === CompAck) { assert(entrys(i).state === RSState.WaitCompAck | entrys(i).entryMes.snpFwdWaitAck, "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state) }
          .otherwise { assert(entrys(i).state === RSState.Snp2NodeIng | entrys(i).state === RSState.WaitSnpResp, "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state) }
        }
      }
  }

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
        assert(entrys(i).state === RSState.WaitDBID, "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
      }
  }


  /*
   * Update entryMes
   */
  entrys.map(_.entryMes).zipWithIndex.foreach {
    case (entryMes, i) =>
      // ------------------------------------------- Update Base Values -------------------------------------------------- //
      // Receive New Req
      when((rxReq.fire | io.req2Intf.fire | io.resp2Intf.fire) & entryFreeID === i.U) {
        entryMes                := entrySave.entryMes
      // Count CHI TX Dat
      }.elsewhen(rxDat.fire & entryRecChiDatID === i.U) {
        val hitRespDat = rxDat.fire & entryRecChiDatID === i.U
        entryMes.getBeatNum     := entryMes.getBeatNum + hitRespDat.asUInt
        entryMes.hasData        := true.B
        assert(isWriteX(entrys(i).chiMes.opcode) | (entrys(i).chiMes.isSnp & entrys(i).chiMes.retToSrc))
      // Get CompAck
      }.elsewhen(rxRsp.fire & entryRecChiRspID === i.U & !rspIsDMTComp & rxRsp.bits.Opcode === CompAck) {
        entryMes.snpFwdWaitAck  := false.B
      // Send ReadReceipt
      }.elsewhen(txRsp.fire & entrySendRspID === i.U & txRsp.bits.Opcode === ReadReceipt) {
        entryMes.needSendRRec   := false.B
      }


      // ---------------------------------------------- Record Snp Resp --------------------------------------------------- //
      when(entrys(i).chiMes.isSnp) {
        when(entrys(i).state === RSState.Snp2NodeIng | entrys(i).state === RSState.WaitSnpResp) {
          val rspHit        = rxRsp.fire & entryRecChiRspID === i.U & !rspIsDMTComp
          val datHit        = rxDat.fire & entryRecChiDatID === i.U & entrys(i).isLastBeat
          val rspId         = getMetaIDByNodeID(rxRsp.bits.SrcID); assert(fromCcNode(rxRsp.bits.SrcID) | !rxRsp.valid)
          val datId         = getMetaIDByNodeID(rxDat.bits.SrcID); assert(fromCcNode(rxDat.bits.SrcID) | !rxDat.valid)
          val rspIdOH       = Mux(rspHit, UIntToOH(rspId), 0.U)
          val datIdOH       = Mux(datHit, UIntToOH(datId), 0.U)
          entryMes.getSnpRespOH := entryMes.getSnpRespOH | rspIdOH | datIdOH
          // assert
          val getSnpRespVec = Wire(Vec(nrCcNode, Bool()))
          val tgtSnpVec     = Wire(Vec(nrCcNode, Bool()))
          getSnpRespVec     := entryMes.getSnpRespOH.asBools
          tgtSnpVec         := entrys(i).chiIndex.snpCcMetaVec(nrCcNode - 1, 0).asBools
          assert(Mux(rspHit, !getSnpRespVec(rspId), true.B), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
          assert(Mux(datHit, !getSnpRespVec(datId), true.B), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
          assert(Mux(rspHit, tgtSnpVec(rspId),      true.B), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
          assert(Mux(datHit, tgtSnpVec(datId),      true.B), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
        }
      }
  }



// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------  State Transfer -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.map(_.state).zipWithIndex.foreach {
    case(state, i) =>
      switch(state) {
        // State: Free
        is(RSState.Free) {
          val hit       = entryFreeID === i.U
          val reqHit    = rxReq.fire & !isWriteX(rxReq.bits.Opcode) & hit
          val cbOrWriHit= rxReq.fire & isWriteX(rxReq.bits.Opcode) & hit
          val atomicHit = rxReq.fire & isAtomicX(rxReq.bits.Opcode) & hit
          val snpHit    = io.req2Intf.fire & hit
          val respHit   = io.resp2Intf.fire & hit
          val snpHasDBID= io.req2Intf.bits.pcuMes.hasPcuDBID
          val ret2Src   = io.req2Intf.bits.chiMes.retToSrc
          state         := Mux(reqHit, RSState.Req2Exu,
                            Mux(atomicHit, RSState.GetDBID,
                              Mux(snpHit & snpHasDBID, RSState.Snp2Node,
                                Mux(snpHit & ret2Src, RSState.GetDBID,
                                  Mux(snpHit & !ret2Src, RSState.Snp2Node,
                                    Mux(cbOrWriHit, RSState.GetDBID,
                                        Mux(respHit, RSState.Resp2Node, state)))))))
          assert(PopCount(Seq(reqHit, cbOrWriHit, snpHit, respHit)) <= 1.U, "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
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
          val hit = !entrys(i).entryMes.needSendRRec | (txRsp.fire & entrySendRspID === i.U); assert(Mux(txRsp.fire & entrySendRspID === i.U, txRsp.bits.Opcode === ReadReceipt, true.B))
          state := Mux(hit, RSState.Free, state)
        }
        // State: Resp2Node
        is(RSState.Resp2Node) {
          val txDatHit  = txDat.fire & txDat.bits.DBID === i.U & toBeatNum(txDat.bits.DataID) === (nrBeat - 1).U
          val txRspHit  = txRsp.fire & entrySendRspID === i.U; assert(txRsp.bits.Opcode === Comp | !txRspHit)
          val expAck    = entrys(i).chiMes.expCompAck
          state         := Mux(txDatHit | txRspHit, Mux(expAck, RSState.WaitCompAck, RSState.Free), state)
        }
        // State: WaitCompAck
        is(RSState.WaitCompAck) {
          val hit       = rxRsp.fire & rxRsp.bits.TxnID === i.U
          assert(Mux(hit, rxRsp.bits.Opcode === CompAck, true.B), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state)
          state         := Mux(hit, RSState.Free, state)
        }
        // State: GetDBID
        is(RSState.GetDBID) {
          val hit       = io.dbSigs.getDBID.fire & entryGetDBID === i.U; assert(Mux(hit, entrys(i).entryMes.nID === 0.U, true.B), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state) // TODO: Consider Write can go no sorting required
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
          val hit       = txRsp.fire & entrySendRspID === i.U; assert(txRsp.bits.Opcode === CompDBIDResp | txRsp.bits.Opcode === DBIDResp | !hit)
          state         := Mux(hit, RSState.WaitData, state)
        }
        // State: WaitData
        is(RSState.WaitData) {
          val hit       = rxDat.fire & rxDat.bits.TxnID === i.U
          state         := Mux(hit & entrys(i).isLastBeat, RSState.Req2Exu, state)
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
          val rspHit    = rxRsp.fire & entryRecChiRspID === i.U & !rspIsDMTComp
          val datHit    = rxDat.fire & entryRecChiDatID === i.U
          val shlGetNum = PopCount(entrys(i).entryMes.getSnpRespOH ^ entrys(i).chiIndex.snpCcMetaVec)
          val nowGetNum = rspHit.asTypeOf(UInt(2.W)) + (datHit & entrys(i).isLastBeat).asTypeOf(UInt(2.W))
          state         := Mux(shlGetNum === nowGetNum, RSState.Resp2Exu, state)
        }
        // State: Resp2Exu
        is(RSState.Resp2Exu) {
          val hit       = io.resp2Exu.fire & entryResp2ExuID === i.U & !dmtCompVal
          state         := Mux(hit, RSState.Free, state)
        }
      }
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------- Get Task NID --------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Req NID
   */
  val snp2IntfAWD     = io.req2Intf.bits.addrWithDcuID
  val resp2ExuAWD     = entrys(entryResp2ExuID).addrWithDcuID
  val reqAckAWD       = entrys(io.reqAck2Intf.bits.entryID).addrWithDcuID
  val compAckAWD      = entrys(rxRsp.bits.TxnID).addrWithDcuID

  snp2IntfHitVec      := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === snp2IntfAWD             & io.req2Intf.fire }
  resp2ExuHitVec      := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === resp2ExuAWD             & io.resp2Exu.fire    & !dmtCompVal }
  reqAckHitVec        := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === reqAckAWD               & io.reqAck2Intf.fire & !io.reqAck2Intf.bits.retry & io.reqAck2Intf.bits.entryID =/= i.U }
  compAckHitVec       := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === compAckAWD              & rxRsp.fire          & rxRsp.bits.Opcode === CompAck & !dmtCompVal }

  reqMatchVec         := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === entrySave.addrWithDcuID & (e.chiMes.isReq | e.chiMes.isSnp) }
  snpMatchVec         := entrys.zipWithIndex.map { case(e, i) => !e.isFree & e.addrWithDcuID === entrySave.addrWithDcuID & (e.chiMes.isRsp | e.chiMes.isDat) & e.chiMes.expCompAck }

  val reqMatchNum     = PopCount(reqMatchVec)
  val snpMatchNum     = PopCount(snpMatchVec)

  entrys.map(_.entryMes.nID).zipWithIndex.foreach {
    case(nID, i) =>
      /*
       * Set New NID
       */
      when(entryFreeID === i.U) {
        // Resp always takes priority
        when(io.resp2Intf.fire)     { nID := 0.U }
        // Snp
        .elsewhen(io.req2Intf.fire) { nID := snpMatchNum - compAckHitVec(i) ;                                     assert(snpMatchNum >= compAckHitVec(i), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x] NID[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state, nID) }
        // Req
        .elsewhen(rxReq.fire)       { nID := reqMatchNum - resp2ExuHitVec(i) - reqAckHitVec(i);                   assert(reqMatchNum >= resp2ExuHitVec(i) + reqAckHitVec(i), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x] NID[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state, nID) }
      }
      /*
       * Modify NID
       */
      .elsewhen(!entrys(i).isFree) {
        // Snp
        when(entrys(i).chiMes.isSnp)      { nID := nID - compAckHitVec(i);                                        assert(nID >= compAckHitVec(i), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x] NID[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state, nID) }
        // Req
        .elsewhen(entrys(i).chiMes.isReq) { nID := nID + snp2IntfHitVec(i) - resp2ExuHitVec(i) - reqAckHitVec(i); assert(nID + snp2IntfHitVec(i) >= resp2ExuHitVec(i) + reqAckHitVec(i), "RNSLV ENTRY[0x%x] ADDR[0x%x] STATE[0x%x] NID[0x%x]", i.U, entrys(i).fullAddr(io.pcuID), entrys(i).state, nID) }
      }
  }


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
  val snpVal        = io.req2Intf.valid; assert(Mux(snpVal, io.req2Intf.bits.chiMes.isSnp, true.B))
  val respVal       = io.resp2Intf.valid
  //                                              | RESP(resp2Intf)                                 | SNP(req2Intf)                       | REQ(rxReq)
  entrySave.entryMes.useAddr      := Mux(respVal, io.resp2Intf.bits.pcuMes.useAddr,     Mux(snpVal, io.req2Intf.bits.pcuMes.useAddr,      parseFullAddr(rxReq.bits.Addr)._6))
  entrySave.entryMes.dcuID        := Mux(respVal, io.resp2Intf.bits.from,               Mux(snpVal, io.req2Intf.bits.from,                parseFullAddr(rxReq.bits.Addr)._3))
  entrySave.entryMes.snpFwdWaitAck:= Mux(respVal, false.B,                              Mux(snpVal, isSnpXFwd(io.req2Intf.bits.chiMes.opcode), false.B))
  entrySave.entryMes.needSendRRec := Mux(respVal, false.B,                              Mux(snpVal, false.B,                              rxReq.bits.Opcode === ReadOnce))
  entrySave.entryMes.swapFst      := Mux(respVal, false.B,                              Mux(snpVal, false.B,                              rxReq.bits.Addr(offsetBits -1 , 0)(rxReq.bits.Size).asBool))
  entrySave.pcuIndex.mshrWay      := Mux(respVal, DontCare,                             Mux(snpVal, io.req2Intf.bits.pcuIndex.mshrWay,    DontCare))
  entrySave.pcuIndex.dbID         := Mux(respVal, io.resp2Intf.bits.pcuIndex.dbID,      Mux(snpVal, io.req2Intf.bits.pcuIndex.dbID,       0.U))
  entrySave.chiIndex.txnID        := Mux(respVal, io.resp2Intf.bits.chiIndex.txnID,     Mux(snpVal, io.req2Intf.bits.chiIndex.txnID,      rxReq.bits.TxnID))
  entrySave.chiIndex.nodeID       := Mux(respVal, io.resp2Intf.bits.chiIndex.nodeID,    Mux(snpVal, io.req2Intf.bits.chiIndex.nodeID,     getUseNodeID(rxReq.bits.SrcID)))
  entrySave.chiIndex.beatOH       := Mux(respVal, io.resp2Intf.bits.chiIndex.beatOH,    Mux(snpVal, "b11".U,                              rxReq.bits.beatOH))
  entrySave.chiMes.opcode         := Mux(respVal, io.resp2Intf.bits.chiMes.opcode,      Mux(snpVal, io.req2Intf.bits.chiMes.opcode,       rxReq.bits.Opcode))
  entrySave.chiMes.retToSrc       := Mux(respVal, DontCare,                             Mux(snpVal, io.req2Intf.bits.chiMes.retToSrc,     DontCare))
  entrySave.chiMes.doNotGoToSD    := Mux(respVal, DontCare,                             Mux(snpVal, io.req2Intf.bits.chiMes.doNotGoToSD,  DontCare))
  entrySave.chiMes.channel        := Mux(respVal, io.resp2Intf.bits.chiMes.channel,     Mux(snpVal, CHIChannel.SNP,                       CHIChannel.REQ))
  entrySave.chiMes.resp           := Mux(respVal, io.resp2Intf.bits.chiMes.resp,        Mux(snpVal, 0.U,                                  0.U))
  entrySave.chiMes.expCompAck     := Mux(respVal, io.resp2Intf.bits.chiMes.expCompAck,  Mux(snpVal, false.B,                              rxReq.bits.ExpCompAck))
  assert(Mux(snpVal, io.req2Intf.bits.chiIndex.fullSize, true.B))
  assert(Mux(snpVal, io.req2Intf.bits.chiMes.isSnp, true.B))


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
  entrySendReqID    := RREncoder(reqBeSendVec)

  /*
   * Send Req To Node
   */
  io.req2Exu.valid                      := reqBeSendVec.reduce(_ | _)
  io.req2Exu.bits.chiMes.channel        := entrys(entrySendReqID).chiMes.channel
  io.req2Exu.bits.chiMes.opcode         := entrys(entrySendReqID).chiMes.opcode
  io.req2Exu.bits.chiMes.expCompAck     := entrys(entrySendReqID).chiMes.expCompAck
  io.req2Exu.bits.chiMes.resp           := entrys(entrySendReqID).chiMes.resp
  io.req2Exu.bits.chiIndex.nodeID       := entrys(entrySendReqID).chiIndex.nodeID
  io.req2Exu.bits.chiIndex.txnID        := entrys(entrySendReqID).chiIndex.txnID
  io.req2Exu.bits.chiIndex.beatOH       := entrys(entrySendReqID).chiIndex.beatOH
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
  val datBeSendVec  = entrys.map { case p => p.isDatBeSend & p.pcuIndex.dbID === io.dbSigs.dataFDB.bits.dbID }
  val datSelId      = PriorityEncoder(datBeSendVec)

  txDat.valid        := datBeSendVec.reduce(_ | _)
  txDat.bits         := DontCare
  txDat.bits.Opcode  := entrys(datSelId).chiMes.opcode
  txDat.bits.TgtID   := getFullNodeID(entrys(datSelId).chiIndex.nodeID)
  txDat.bits.SrcID   := io.hnfID
  txDat.bits.TxnID   := entrys(datSelId).chiIndex.txnID
  txDat.bits.HomeNID := io.hnfID
  txDat.bits.DBID    := datSelId
  txDat.bits.Resp    := entrys(datSelId).chiMes.resp
  txDat.bits.DataID  := io.dbSigs.dataFDB.bits.dataID
  txDat.bits.Data    := io.dbSigs.dataFDB.bits.data
  txDat.bits.BE      := Fill(txDat.bits.BE.getWidth, 1.U(1.W))

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
                            Mux(entrys(entrySendRspID).state === RSState.DBIDResp2Node, Mux(isWriUniX(entrys(entrySendRspID).chiMes.opcode), DBIDResp, CompDBIDResp), // Write or CopyBack Send DBIDResp
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
  io.dbSigs.dataTDB.bits.atomicVal:= isAtomicX(entrys(entryRecChiDatID).chiMes.opcode)

  /*
   * Set ready value
   */
  rspIsDMTComp              := rxRsp.bits.TxnID(chiTxnIdBits - 1).asBool; assert(Mux(rxRsp.valid & rspIsDMTComp, rxRsp.bits.Opcode === CompAck, true.B))
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
  entryGetDBID                := RREncoder(entryGetDBIDVec)

  /*
   * Set DataBuffer Req Value
   */
  io.dbSigs.getDBID.valid           := entryGetDBIDVec.reduce(_ | _)
  io.dbSigs.getDBID.bits.from       := param.intfID.U
  io.dbSigs.getDBID.bits.entryID    := entryGetDBID
  io.dbSigs.getDBID.bits.atomicVal  := isAtomicX(entrys(entryGetDBID).chiMes.opcode)
  io.dbSigs.getDBID.bits.atomicOp   := getAtomicOp(entrys(entryGetDBID).chiMes.opcode)
  io.dbSigs.getDBID.bits.swapFst    := entrys(entryGetDBID).entryMes.swapFst

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
  val snpShouldSendVec  = entrys(entrySendSnpID).chiIndex.snpCcMetaVec
  val snpBeSendVec      = snpShouldSendVec ^ snpAlreadySendVecReg
  val snpTgtID          = getNodeIDByMetaID(PriorityEncoder(snpBeSendVec)); assert(PriorityEncoder(snpBeSendVec) < nrCcNode.U | !txSnp.valid)
  snpIsLast             := PopCount(snpBeSendVec.asBools) === 1.U; dontTouch(snpIsLast)
  snpAlreadySendVecReg  := Mux(txSnp.fire, Mux(snpIsLast, 0.U, snpAlreadySendVecReg | UIntToOH(getMetaIDByNodeID(snpTgtID))), snpAlreadySendVecReg); assert(fromCcNode(snpTgtID) | !txSnp.valid)

  /*
   * Send Snp to Node
   */
  txSnp.valid          := entrySendSnpVec.reduce(_ | _) | entrySendSnpIngVec.reduce(_ | _)
  txSnp.bits           := DontCare
  txSnp.bits.Addr      := entrys(entrySendSnpID).snpAddr(io.pcuID)
  txSnp.bits.Opcode    := Mux(snpAlreadySendVecReg === 0.U, entrys(entrySendSnpID).chiMes.opcode, getNoFwdSnpOp(entrys(entrySendSnpID).chiMes.opcode))
  txSnp.bits.TgtID     := snpTgtID
  txSnp.bits.SrcID     := io.hnfID
  txSnp.bits.TxnID     := entrySendSnpID
  txSnp.bits.FwdNID    := entrys(entrySendSnpID).chiIndex.nodeID
  txSnp.bits.FwdTxnID  := entrys(entrySendSnpID).chiIndex.txnID
  txSnp.bits.RetToSrc  := entrys(entrySendSnpID).chiMes.retToSrc & snpAlreadySendVecReg === 0.U
  txSnp.bits.DoNotGoToSD := entrys(entrySendSnpID).chiMes.doNotGoToSD


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- Select Entry send Resp to EXU ----------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Select one Entry to Send RxDat
   */
  val respBeSendVec                     = entrys.map { case p => p.state === RSState.Resp2Exu & !p.entryMes.snpFwdWaitAck }
  entryResp2ExuID                       := PriorityEncoder(respBeSendVec)


  val dmtMshrWay                        = rxRsp.bits.TxnID(mshrWayBits - 1, 0)
  val dmtMshrSet                        = rxRsp.bits.TxnID(mshrSetBits + mshrWayBits - 1, mshrWayBits)
  val dmtDcuID                          = rxRsp.bits.TxnID(dcuBankBits + mshrSetBits + mshrWayBits - 1, mshrSetBits + mshrWayBits)

  io.resp2Exu.valid                     := respBeSendVec.reduce(_ | _) | dmtCompVal
  io.resp2Exu.bits.from                 := param.intfID.U
  io.resp2Exu.bits.to                   := Mux(dmtCompVal, dmtDcuID,    entrys(entryResp2ExuID).entryMes.dcuID)
  io.resp2Exu.bits.pcuIndex.mshrSet     := Mux(dmtCompVal, dmtMshrSet,  entrys(entryResp2ExuID).entryMes.mSet)
  io.resp2Exu.bits.pcuIndex.mshrWay     := Mux(dmtCompVal, dmtMshrWay,  entrys(entryResp2ExuID).pcuIndex.mshrWay)
  io.resp2Exu.bits.pcuIndex.dbID        := Mux(dmtCompVal, 0.U,         entrys(entryResp2ExuID).pcuIndex.dbID)
  io.resp2Exu.bits.chiMes.resp          := Mux(dmtCompVal, 0.U,         entrys(entryResp2ExuID).chiMes.resp)
  io.resp2Exu.bits.chiMes.fwdState      := Mux(dmtCompVal, 0.U,         entrys(entryResp2ExuID).chiMes.fwdState)
  io.resp2Exu.bits.pcuMes.hasData       := Mux(dmtCompVal, false.B,     entrys(entryResp2ExuID).entryMes.hasData)
  io.resp2Exu.bits.pcuMes.fwdSVald      := Mux(dmtCompVal, false.B,     isSnpXFwd(entrys(entryResp2ExuID).chiMes.opcode))
  io.resp2Exu.bits.pcuMes.isSnpResp     := !dmtCompVal
  io.resp2Exu.bits.pcuMes.isCompAck     := dmtCompVal


// ---------------------------  Assertion  -------------------------------- //
  val cntReg = RegInit(VecInit(Seq.fill(param.nrEntry) { 0.U(64.W) }))
  cntReg.zip(entrys).zipWithIndex.foreach { case((c, p), i) => c := Mux(p.isFree | (io.reqAck2Intf.fire & io.reqAck2Intf.bits.entryID === i.U) , 0.U, c + 1.U) }
  cntReg.zipWithIndex.foreach { case(c, i) => assert(c < TIMEOUT_RSINTF.U, "RNSLV ENTRY[0x%x] STATE[0x%x] ADDR[0x%x] CHANNEL[%x] OP[0x%x] TIMEOUT", i.U, entrys(i).entryMes.state, entrys(i).fullAddr(io.pcuID), entrys(i).chiMes.channel, entrys(i).chiMes.opcode) }

  when(rxReq.valid) {
    // [1:cacheable] [2:ccxChipID] [3:dcuBank] [4:pcuBank] [5:offset] [6:useAddr]
    // cacheable
    assert(parseFullAddr(rxReq.bits.Addr)._1 === 0.U)
    // ccxChipID
    assert(parseFullAddr(rxReq.bits.Addr)._2 === 0.U)
    // pcuBank
    assert(parseFullAddr(rxReq.bits.Addr)._4 === io.pcuID)
    // offset
    assert(Mux(isAtomicX(rxReq.bits.Opcode), true.B, // Atomic
           Mux(isWriUniX(rxReq.bits.Opcode) | rxReq.bits.Opcode === ReadOnce, parseFullAddr(rxReq.bits.Addr)._5 === 0.U | parseFullAddr(rxReq.bits.Addr)._5 === 0x20.U, // WriteUnique or ReadOnce
               parseFullAddr(rxReq.bits.Addr)._5 === 0.U))) // Other
    // TgtID
    assert(rxReq.bits.TgtID === io.hnfID)
    // ExpCompAck
    assert(Mux(isReadX(rxReq.bits.Opcode) & rxReq.bits.Opcode =/= ReadOnce, rxReq.bits.ExpCompAck, true.B)) // TODO: need more power check
    assert(Mux(isWriUniX(rxReq.bits.Opcode),    rxReq.bits.ExpCompAck, true.B)) // TODO: need more power check
    // Order
    assert(Mux(isWriUniX(rxReq.bits.Opcode),    rxReq.bits.Order === Order.OWO & rxReq.bits.ExpCompAck,
           Mux(rxReq.bits.Opcode === ReadOnce,  rxReq.bits.Order === Order.EndpointOrder, rxReq.bits.Order === Order.None)))
    // Size
    assert(Mux(isWriXFull(rxReq.bits.Opcode),   rxReq.bits.Size === chiFullSize.U, true.B))
    assert(Mux(isAtomicX(rxReq.bits.Opcode),    rxReq.bits.Size <= chiHalfSize.U, // Atomic
           Mux(isWriXPtl(rxReq.bits.Opcode),    rxReq.bits.Size < chiFullSize.U, // WriteXPtl
           Mux(rxReq.bits.Opcode === ReadOnce,  rxReq.bits.Size <= chiFullSize.U, // ReadOnce
                                                rxReq.bits.Size === chiFullSize.U)))) // Other
    // Endian
    assert(rxReq.bits.Endian.asUInt === 0.U) // Must be Little Endian
  }

// -------------------------------------------------- Perf Counter ------------------------------------------------------ //
  val reqFire = rxReq.fire | io.req2Intf.fire | io.resp2Intf.fire
  require(param.nrEntry >= 4 & param.nrEntry % 4 == 0)
  for(i <- 0 until  (param.nrEntry/4)) {
    XSPerfAccumulate(s"pcu_localRnSlave_entry_group[${i}]_deal_req_cnt", reqFire & (i*4).U <= entryFreeID & entryFreeID <= (i*4+3).U)
  }
  XSPerfAccumulate("pcu_localRnSlave_req_cnt", reqFire)
  XSPerfAccumulate("pcu_localRnSlave_req_block_cnt", (rxReq.valid | io.req2Intf.valid | io.resp2Intf.valid) & entryFreeNum === 0.U)
}