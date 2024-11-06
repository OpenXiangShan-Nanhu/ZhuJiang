package dongjiang.pcu.intf

import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi._
import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.FastArb._
import xijiang.Node
import xs.utils.perf.{DebugOptions, DebugOptionsKey, HasPerfLogging}


/*
 * ************************************************************** State transfer ***********************************************************************************
 *
 * Read Req Without DMT:  [Free] -----> [GetDBID] -----> [WaitDBID] -----> [Req2Node] -----> [WaitNodeData] -----> [Resp2Exu]
 *
 * Read Req With DMT:     [Free] -----> [Req2Node] -----> [Resp2Exu]
 *
 * Write Req:             [Free] -----> [Req2Node] -----> [WaitNodeDBID] -----> [RCDB] -----> [WriteData2Node] -----> ([WaitNodeComp]) -----> [Resp2Exu]
 *
 * Replace:               [Free] -----> [Req2Node] -----> [WaitNodeDBID] -----> [Replace2Node] -----> [WaitReplDBID] -----> [RCDB] -----> [WriteData2Node] -----> ([WaitNodeComp]) -----> [Resp2Exu]
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
 * Read: TODO: When using Read without DMT need to make sure that the RnSlave does not fill up the DataBuffer.
 * { Req2Intf        } Req    From Exu And Store In Intf                                                                                                    | { pcuIdx.mshrIdx = pcuIdx.mshrIdx } { pcuIdx.dcuIdx = pcuIdx.dcuIdx }
 * { Read            } Req    Send To CHI                         { TgtID = tgtID } { ReturnNID = hnfID } { ReturnTxnID = entryID }                         |
 * { CompData        } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Exu        } Resp   Send To Exu                                                                                                                   | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
 *
 *
 * Read With DMT: Not implemented in the system
 * { Req2Intf        } Req    From Exu And Store In Intf          { chiIdx.nodeID = chiIdx.nodeID } { chiIdx.txnID =  chiIdx.txnID }                        | { pcuIdx.mshrIdx = pcuIdx.mshrIdx }
 * { Read            } Req    Send To CHI                         { TgtID = chiMes.tgtID } { TxnID = Cat(chiMes.dcuID, pcuIdx.mshrIdx) } { ReturnNID = chiIdx.nodeID } { ReturnTxnID = chiIdx.txnID }
 * { Resp2Exu        } Resp   Send To Exu                                                                                                                   | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
 *
 *
 * Write:
 * { Req2Intf        } Req    From Exu And Store In Intf                                                                                                    | { pcuIdx.mshrIdx = pcuIdx.mshrIdx } { pcuIdx.dbID = pcuIdx.dbID } { pcuIdx.dcuIdx = pcuIdx.dcuIdx }
 * { Write           } Req    Send To CHI                         { TgtID = tgtID } { TxnID = entryID } { ReturnNID = hnfID } { ReturnTxnID = entryID }     |
 * { DBIDResp        } Resp   From CHI And Match With Entry ID    { TxnID == entryID } { chiIdx.txnID = DBID } (Store DBID In chiIdx.txnID)                 |
 * { NCBWrData       } Data   Send To CHI                         { TgtID = tgtID } { TxnID = chiIdx.txnID }                                                |
 * { Comp            } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Exu        } Resp   Send To Exu                                                                                                                   | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
 *
 *
 * Write With DWT: TODO
 * { Req2Intf        } Req    From Exu And Store In Intf          { chiIdx.nodeID = chiIdx.nodeID } { chiIdx.txnID =  chiIdx.txnID }                        | { pcuIdx.mshrIdx = pcuIdx.mshrIdx } { pcuIdx.dcuIdx = pcuIdx.dcuIdx }
 * { Write           } Req    Send To CHI                         { TgtID = tgtID } { TxnID = entryID } { ReturnNID = chiIdx.nodeID } { ReturnTxnID = chiIdx.txnID } |
 * { Comp            } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Exu        } Resp   Send To Exu                                                                                                                   | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
 *
 * Replace:
 * { Req2Intf        } Req    From Exu And Store In Intf                                                                                                    | { pcuIdx.mshrIdx = pcuIdx.mshrIdx } { pcuIdx.dbID = pcuIdx.dbID } { pcuIdx.dcuIdx = pcuIdx.dcuIdx }
 * { Write           } Req    Send To CHI                         { TgtID = ddrcID } { TxnID = entryID } { ReturnNID = hnfID } { ReturnTxnID = entryID }    |
 * { DBIDResp        } Resp   From CHI And Match With Entry ID    { TxnID == entryID } { chiIdx.txnID = DBID } (Store DBID In chiIdx.txnID)                 |
 * { Replace         } Req    Send To CHI                         { TgtID = tgtID } { TxnID = entryID } { ReturnTxnID = chiIdx.txnID }                      |
 * { NCBWrData       } Data   Send To CHI                         { TgtID = tgtID } { TxnID = chiIdx.txnID }                                                |
 * { Comp            } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Exu        } Resp   Send To Exu                                                                                                                   | { pcuIdx.to = chiMes.dcuID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
 *
 *
 */

object SMState {
  val width = 4
  // commom
  val Free            = "b0000".U // 0x0
  val GetDBID         = "b0001".U // 0x1
  val WaitDBID        = "b0010".U // 0x2
  val Req2Node        = "b0011".U // 0x3
  val WaitNodeData    = "b0100".U // 0x4
  val Resp2Exu        = "b0101".U // 0x5
  val WaitNodeDBID    = "b0110".U // 0x6
  val RCDB            = "b0111".U // 0x7
  val WriteData2Node  = "b1000".U // 0x8
  val WaitNodeComp    = "b1001".U // 0x9
  val Replace2Node    = "b1010".U // 0xa
  val WaitReplDBID    = "b1011".U // 0xb
}

object SMType {
  val width = 2
  val RDCU  = "b00".U
  val RDDR  = "b01".U
  val WRITE = "b10".U // Write DCU
  val REPL  = "b11".U // Read DCU And Write DDRC
}

class SMEntry(param: InterfaceParam)(implicit p: Parameters) extends DJBundle {
  val chiIndex        = new ChiIndexBundle()
  val chiMes          = new ChiMesBundle()
  val pcuIndex        = new PcuIndexBundle()
  val entryMes        = new DJBundle with HasUseAddr with HasDcuID {
    val state         = UInt(RSState.width.W)
    val doDMT         = Bool()
    val toDCU         = Bool()
    val hasData       = Bool()
    val getDataNum    = UInt(beatNumBits.W)
    val alrGetCompNum = UInt(2.W) // Already Get Comp, only use in write or replace
    val selfWay       = UInt(sWayBits.W)
  }

  def state           = entryMes.state
  def isFree          = entryMes.state === SMState.Free
  def isGetDBID       = entryMes.state === SMState.GetDBID
  def isReq2Node      = entryMes.state === SMState.Req2Node
  def isRepl2Node     = entryMes.state === SMState.Replace2Node
  def isResp2Exu      = entryMes.state === SMState.Resp2Exu
  def isWaitDBData    = entryMes.state === SMState.WriteData2Node
  def isRCDB          = entryMes.state === SMState.RCDB
  def isLastBeat      = entryMes.getDataNum === (nrBeat - 1).U
  def isReadReq       = isReadX(chiMes.opcode)
  def isWriteReq      = isWriteX(chiMes.opcode)
  def isReplReq       = isReplace(chiMes.opcode)
  def mshrIndexTxnID  = Cat(entryMes.dcuID, entryMes.mSet, pcuIndex.mshrWay) | (1 << fullNodeIdBits).U
  def fullAddr (p: UInt) = entryMes.fullAddr(entryMes.dcuID, p)

  def getAllComp      = Mux(isReplReq, entryMes.alrGetCompNum === 2.U, entryMes.alrGetCompNum === 1.U)

  def fullTgtID(fIDSeq: Seq[UInt]): UInt = { // friendIdSeq
    val nodeID      = WireInit(0.U(fullNodeIdBits.W))
    when(entryMes.toDCU) { nodeID := getFriendDcuIDByDcuBankID(entryMes.dcuID, fIDSeq); assert(entryMes.dcuID <= nrBankPerPCU.U) }
    .otherwise           { nodeID := ddrcNodeId.U }
    nodeID
  }
  def reqAddr(p: UInt) : UInt = {
    val addr        = WireInit(0.U(fullAddrBits.W))
    when(entryMes.toDCU) { addr := getDCUAddress(entryMes.sSet, entryMes.dirBank, entryMes.selfWay) }
    .otherwise           { addr := fullAddr(p) }
    addr
  }
  def reqOp: UInt = {
    val op          = WireInit(0.U(7.W))
    when(!isReplReq)   { op := chiMes.opcode }
    .elsewhen(entryMes.toDCU) { op := chiMes.opcode }
    .otherwise      { op := WriteNoSnpFull }
    op
  }
}

class SnMasterIntf(param: InterfaceParam, node: Node)(implicit p: Parameters) extends IntfBaseIO(param, node) with HasPerfLogging {
  // Del it
  io <> DontCare
  dontTouch(io)
// --------------------- Reg and Wire declaration ------------------------//
  val entrys          = RegInit(VecInit(Seq.fill(param.nrEntry) { 0.U.asTypeOf(new SMEntry(param)) }))
  // Intf Receive Req ID
  val entryGetReqID   = Wire(UInt(param.entryIdBits.W))
  // Intf Get DBID ID
  val entryGetDBID    = Wire(UInt(param.entryIdBits.W))
  // Intf Receive DBID ID
  val entryRecDBID    = Wire(UInt(param.entryIdBits.W))
  // Intf Req To Node ID
  val entryReq2NodeID = Wire(UInt(param.entryIdBits.W))
  // Intf Send Resp To Exu ID
  val entryResp2ExuID = Wire(UInt(param.entryIdBits.W))
  // Intf Send RC Req To DataBuufer ID
  val entryRCDBID     = Wire(UInt(param.entryIdBits.W))
  // req from EXU
  val entrySave       = WireInit(0.U.asTypeOf(new SMEntry(param)))


  // CHI
  val rxDat           = Wire(new DecoupledIO(new DataFlit()))
  val rxRsp           = Wire(new DecoupledIO(new RespFlit()))
  val txDat           = WireInit(0.U.asTypeOf(Decoupled(new DataFlit())))
  val txReq           = WireInit(0.U.asTypeOf(Decoupled(new ReqFlit())))
  io.chi              <> DontCare
  io.chi.rx.data.get  <> rxDat
  io.chi.rx.resp.get  <> rxRsp
  io.chi.tx.data.get  <> txDat
  io.chi.tx.req.get   <> txReq

  /*
   * for Debug
   */
  val entrys_dbg_addr = Wire(Vec(param.nrEntry, UInt(fullAddrBits.W)))
  entrys_dbg_addr.zipWithIndex.foreach { case(addr, i) => addr := entrys(i).fullAddr(io.pcuID) }
  if (p(DebugOptionsKey).EnableDebug) {
    dontTouch(entrys_dbg_addr)
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------  Update Entry Value  --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.zipWithIndex.foreach {
    case (entry, i) =>
      /*
       * Receive New Req
       */
      when(io.req2Intf.fire & entryGetReqID === i.U) {
        entry                       := entrySave
        assert(entry.state === SMState.Free, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
      /*
       * Receive DBID From DataBuffer
       */
      }.elsewhen(io.dbSigs.dbidResp.fire & entryRecDBID === i.U) {
        entry.entryMes.hasData      := true.B
        entry.pcuIndex.dbID         := io.dbSigs.dbidResp.bits.dbID
        assert(entry.state === SMState.WaitDBID, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
      /*
       * Receive Data And Resp From CHI RxDat
       */
      }.elsewhen(rxDat.fire & rxDat.bits.TxnID === i.U) {
        entry.entryMes.getDataNum   := entry.entryMes.getDataNum + 1.U
        entry.chiMes.resp           := rxDat.bits.Resp
        assert(rxDat.bits.Opcode === CompData, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
        assert(entry.isReadReq, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
        assert(entry.state === SMState.WaitNodeData, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
      /*
       * Receive DBID or Comp From CHI RxRsp
       */
      }.elsewhen(rxRsp.fire & rxRsp.bits.TxnID === i.U) {
        when(rxRsp.bits.Opcode === DBIDResp | rxRsp.bits.Opcode === CompDBIDResp) {
          entry.chiIndex.txnID      := rxRsp.bits.DBID
          entry.entryMes.toDCU      := true.B // for send Replace
          entry.entryMes.alrGetCompNum  := entry.entryMes.alrGetCompNum + (rxRsp.bits.Opcode === CompDBIDResp).asUInt
          assert(entry.state === SMState.WaitReplDBID | entry.state === SMState.WaitNodeDBID, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
          assert(Mux(entry.isReplReq, Mux(entry.state === SMState.WaitReplDBID, entry.entryMes.toDCU, !entry.entryMes.toDCU), entry.entryMes.toDCU), "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
        }
        when(rxRsp.bits.Opcode === Comp) {
          entry.entryMes.alrGetCompNum := entry.entryMes.alrGetCompNum + 1.U
            assert(Mux(!entry.isReplReq | entry.entryMes.alrGetCompNum === 1.U, entry.state === SMState.WaitNodeComp,
            entry.state === SMState.WaitReplDBID | entry.state === SMState.RCDB | entry.state === SMState.WriteData2Node |  entry.state === SMState.WaitNodeComp),
            "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
        }
      /*
       * Receive Data From DataBuffer
       */
      }.elsewhen(io.dbSigs.dataFDB.fire & entry.isWaitDBData & io.dbSigs.dataFDB.bits.dbID === entry.pcuIndex.dbID) {
        entry.entryMes.getDataNum   := entry.entryMes.getDataNum + 1.U
        assert(entry.state === SMState.WriteData2Node, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
      /*
       * Clean Intf Entry When Its Free
       */
      }.elsewhen(entry.isFree) {
        entry               := 0.U.asTypeOf(entry)
      }
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- Entry State Transfer ---------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.zipWithIndex.foreach {
    case (entry, i) =>
      switch(entry.state) {
        // State: Free
        is(SMState.Free) {
          val reqHit    = io.req2Intf.fire & isReadX(io.req2Intf.bits.chiMes.opcode) & entryGetReqID === i.U; assert(!reqHit | io.req2Intf.bits.chiMes.opcode === ReadNoSnp)
          val writeHit  = io.req2Intf.fire & isWriteX(io.req2Intf.bits.chiMes.opcode) & entryGetReqID === i.U; assert(!writeHit | io.req2Intf.bits.chiMes.opcode === WriteNoSnpFull)
          val replHit   = io.req2Intf.fire & isReplace(io.req2Intf.bits.chiMes.opcode) & entryGetReqID === i.U; assert(!replHit | io.req2Intf.bits.chiMes.opcode === Replace)
          entry.state     := Mux(reqHit, Mux(io.req2Intf.bits.pcuMes.doDMT, SMState.Req2Node, SMState.GetDBID),
                              Mux(writeHit, SMState.Req2Node,
                                Mux(replHit, SMState.Req2Node, entry.state)))
        }
        // State: GetDBID
        is(SMState.GetDBID) {
          val hit       = io.dbSigs.getDBID.fire & entryGetDBID === i.U
          entry.state   := Mux(hit, SMState.WaitDBID, entry.state)
          assert(entry.isReadReq | !hit)
        }
        // State: WaitDBID
        is(SMState.WaitDBID) {
          val hit       = io.dbSigs.dbidResp.fire & entryRecDBID === i.U
          entry.state   := Mux(hit, SMState.Req2Node, entry.state)
        }
        // State: Req2Node
        is(SMState.Req2Node) {
          val hit       = txReq.fire & entryReq2NodeID === i.U
          entry.state   := Mux(hit, Mux(entry.isReadReq, Mux(entry.entryMes.doDMT, SMState.Resp2Exu, SMState.WaitNodeData), SMState.WaitNodeDBID), entry.state)
        }
        // State: WaitNodeResp
        is(SMState.WaitNodeData) {
          val rxDatHit  = rxDat.fire & rxDat.bits.TxnID === i.U
          entry.state   := Mux(rxDatHit & entry.isLastBeat, SMState.Resp2Exu, entry.state)
        }
        // State: Resp2Exu
        is(SMState.Resp2Exu) {
          val hit       = io.resp2Exu.fire & entryResp2ExuID === i.U
          entry.state   := Mux(hit, SMState.Free, entry.state)
        }
        // State: WaitNodeDBID
        is(SMState.WaitNodeDBID) {
          val hit       = rxRsp.fire & rxRsp.bits.TxnID === i.U
          entry.state   := Mux(hit, Mux(entry.isReplReq, SMState.Replace2Node, SMState.RCDB), entry.state)
        }
        // State: RCDB
        is(SMState.RCDB) {
          val hit       = io.dbSigs.dbRCReq.fire & entryRCDBID === i.U
          entry.state   := Mux(hit, SMState.WriteData2Node, entry.state)
        }
        // State: WriteData2Node
        is(SMState.WriteData2Node) {
          val hit       = txDat.fire & entry.isLastBeat & io.dbSigs.dataFDB.bits.dbID === entry.pcuIndex.dbID
          entry.state   := Mux(hit, Mux(entry.getAllComp, SMState.Resp2Exu, SMState.WaitNodeComp), entry.state)
        }
        // State: WaitNodeComp
        is(SMState.WaitNodeComp) {
          val hit       = rxRsp.fire & rxRsp.bits.TxnID === i.U
          val canGo     = !entry.isReplReq | entry.entryMes.alrGetCompNum === 1.U
          entry.state   := Mux(hit & canGo,  SMState.Resp2Exu, entry.state)
        }
        // State: Replace2Node
        is(SMState.Replace2Node) {
          val hit       = txReq.fire & entryReq2NodeID === i.U; assert(!hit | entry.isReplReq, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.fullAddr(io.pcuID))
          entry.state     := Mux(hit, SMState.WaitReplDBID, entry.state)
        }
        // State: WaitReplDBID
        is(SMState.WaitReplDBID) {
          val hit       = rxRsp.fire & rxRsp.bits.TxnID === i.U
          entry.state     := Mux(hit, SMState.RCDB, entry.state)
        }
      }
  }


  // -------------------------------------------------------------------------------------------------------------------- //
// ------------------------------ Receive Req From CHITXREQ or Req2Node and Save In Intf -------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive req2Node(Snoop)
   */
  entrySave.entryMes.useAddr  := io.req2Intf.bits.pcuMes.useAddr
  entrySave.entryMes.dcuID    := io.req2Intf.bits.from
  entrySave.entryMes.doDMT    := io.req2Intf.bits.pcuMes.doDMT
  entrySave.entryMes.toDCU    := io.req2Intf.bits.pcuMes.toDCU
  entrySave.entryMes.selfWay  := io.req2Intf.bits.pcuMes.selfWay
  entrySave.pcuIndex.mshrWay  := io.req2Intf.bits.pcuIndex.mshrWay
  entrySave.pcuIndex.dbID     := io.req2Intf.bits.pcuIndex.dbID
  entrySave.chiMes.resp       := io.req2Intf.bits.chiMes.resp
  entrySave.chiMes.opcode     := io.req2Intf.bits.chiMes.opcode
  entrySave.chiIndex.nodeID   := io.req2Intf.bits.chiIndex.nodeID
  entrySave.chiIndex.txnID    := io.req2Intf.bits.chiIndex.txnID
  assert(Mux(io.req2Intf.valid, !io.req2Intf.bits.chiMes.expCompAck, true.B))

  /*
   * Set Intf Value
   */
  val entryFreeVec  = entrys.map(_.isFree)
  val entryFreeNum  = PopCount(entryFreeVec)
  entryGetReqID     := PriorityEncoder(entryFreeVec)

  /*
   * Set Ready Value
   */
  io.req2Intf.ready := entryFreeNum > 0.U



// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- Get DBID From DataBuffer and Wait DataBuffer Resp ---------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Send Get DBID Req To From DataBuffer
   */
  val entryGetDBIDVec                 = entrys.map(_.isGetDBID)
  entryGetDBID                        := PriorityEncoder(entryGetDBIDVec)

  /*
   * Set DataBuffer Req Value
   */
  io.dbSigs.getDBID.valid             := entryGetDBIDVec.reduce(_ | _)
  io.dbSigs.getDBID.bits.from         := param.intfID.U
  io.dbSigs.getDBID.bits.entryID      := entryGetDBID

  /*
   * Receive DBID From DataBuffer
   */
  entryRecDBID                        := io.dbSigs.dbidResp.bits.entryID
  io.dbSigs.dbidResp.ready            := true.B



// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- Send Req To Node -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val entryReq2NodeVec    = entrys.map { case p => p.isReq2Node | p.isRepl2Node }
  entryReq2NodeID         := PriorityEncoder(entryReq2NodeVec)

  txReq.valid             := entryReq2NodeVec.reduce(_ | _)
  txReq.bits.Addr         := entrys(entryReq2NodeID).reqAddr(io.pcuID)
  txReq.bits.Opcode       := entrys(entryReq2NodeID).reqOp
  txReq.bits.TgtID        := entrys(entryReq2NodeID).fullTgtID(io.fIDVec)
  txReq.bits.TxnID        := Mux(entrys(entryReq2NodeID).entryMes.doDMT, entrys(entryReq2NodeID).mshrIndexTxnID, entryReq2NodeID)
  txReq.bits.SrcID        := io.hnfID
  txReq.bits.Size         := log2Ceil(djparam.blockBytes).U
  txReq.bits.MemAttr      := entrys(entryReq2NodeID).chiMes.resp // Multiplex MemAttr to transfer CHI State // Use in Read Req
  //                                                                     Read With DMT                                                                                    Replcae                                 Read Without DMT
  txReq.bits.ReturnNID    := Mux(entrys(entryReq2NodeID).entryMes.doDMT, getFullNodeID(entrys(entryReq2NodeID).chiIndex.nodeID), Mux(entrys(entryReq2NodeID).isRepl2Node, ddrcNodeId.U,                           io.hnfID))
  txReq.bits.ReturnTxnID  := Mux(entrys(entryReq2NodeID).entryMes.doDMT, entrys(entryReq2NodeID).chiIndex.txnID,                 Mux(entrys(entryReq2NodeID).isRepl2Node, entrys(entryReq2NodeID).chiIndex.txnID, entryReq2NodeID))
  assert(Mux(entrys(entryReq2NodeID).isReadReq, true.B, !entrys(entryReq2NodeID).entryMes.doDMT))
  if (p(DebugOptionsKey).EnableDebug) {
    io.chi_tx_req_bits_DbgAddr.get  := entrys_dbg_addr(entryReq2NodeID)
    dontTouch(io.chi_tx_req_bits_DbgAddr.get)
  }

// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------- Receive CHI DBID From From Node ------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  rxRsp.ready             := true.B
  assert(Mux(rxRsp.fire, rxRsp.bits.Opcode === CompDBIDResp | rxRsp.bits.Opcode === DBIDResp | rxRsp.bits.Opcode === Comp, true.B))


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- Read And Clean DataBuffer --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val entryRCDBIDVec                = entrys.map(_.isRCDB)
  entryRCDBID                       := PriorityEncoder(entryRCDBIDVec)

  io.dbSigs.dbRCReq.valid           := entryRCDBIDVec.reduce(_ | _)
  io.dbSigs.dbRCReq.bits.isRead     := true.B
  io.dbSigs.dbRCReq.bits.isClean    := true.B
  io.dbSigs.dbRCReq.bits.dbID       := entrys(entryRCDBID).pcuIndex.dbID
  io.dbSigs.dbRCReq.bits.to         := param.intfID.U


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------- Receive Data From DataBuffer And Send Data To Node ----------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val entrySendDatVec           = entrys.map { case p => p.state === SMState.WriteData2Node & p.pcuIndex.dbID === io.dbSigs.dataFDB.bits.dbID }
  val entrySendDatID            = PriorityEncoder(entrySendDatVec)
  assert(Mux(txDat.valid, PopCount(entrySendDatVec) === 1.U, true.B))

  txDat.valid            := io.dbSigs.dataFDB.valid
  txDat.bits             := DontCare
  txDat.bits.Opcode      := NonCopyBackWriteData
  txDat.bits.TgtID       := entrys(entrySendDatID).fullTgtID(io.fIDVec)
  txDat.bits.SrcID       := io.hnfID
  txDat.bits.TxnID       := entrys(entrySendDatID).chiIndex.txnID
  txDat.bits.DataID      := io.dbSigs.dataFDB.bits.dataID
  txDat.bits.Data        := io.dbSigs.dataFDB.bits.data
  txDat.bits.BE          := Fill(rxDat.bits.BE.getWidth, 1.U(1.W))

  io.dbSigs.dataFDB.ready       := txDat.ready

// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- Receive Data From CHI DAT And Send It To DataBuffer -------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  io.dbSigs.dataTDB.valid       := rxDat.valid
  io.dbSigs.dataTDB.bits.dbID   := entrys(rxDat.bits.TxnID(param.entryIdBits-1, 0)).pcuIndex.dbID
  io.dbSigs.dataTDB.bits.data   := rxDat.bits.Data
  io.dbSigs.dataTDB.bits.dataID := rxDat.bits.DataID
  rxDat.ready                   := io.dbSigs.dataTDB.ready
  assert(Mux(rxDat.valid, rxDat.bits.TxnID <= param.nrEntry.U, true.B))



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------ Send Resp To Exu -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val entryResp2ExuVec                  = entrys.map(_.isResp2Exu)
  entryResp2ExuID                       := PriorityEncoder(entryResp2ExuVec)

  io.resp2Exu.valid                     := entryResp2ExuVec.reduce(_ | _)
  io.resp2Exu.bits                      := DontCare
  io.resp2Exu.bits.pcuMes.isReqResp     := entrys(entryResp2ExuID).isReadReq
  io.resp2Exu.bits.pcuMes.isWriResp     := entrys(entryResp2ExuID).isWriteReq | entrys(entryResp2ExuID).isReplReq
  io.resp2Exu.bits.pcuMes.hasData       := entrys(entryResp2ExuID).entryMes.hasData
  io.resp2Exu.bits.chiMes.resp          := entrys(entryResp2ExuID).chiMes.resp
  io.resp2Exu.bits.pcuIndex.mshrSet     := entrys(entryResp2ExuID).entryMes.mSet
  io.resp2Exu.bits.pcuIndex.mshrWay     := entrys(entryResp2ExuID).pcuIndex.mshrWay
  io.resp2Exu.bits.pcuIndex.dbID        := entrys(entryResp2ExuID).pcuIndex.dbID
  io.resp2Exu.bits.from                 := param.intfID.U
  io.resp2Exu.bits.to                   := entrys(entryResp2ExuID).entryMes.dcuID





// -------------------------------------------------  Assertion  -------------------------------------------------------- //
  val cntReg = RegInit(VecInit(Seq.fill(param.nrEntry) { 0.U(64.W) }))
  cntReg.zip(entrys).foreach { case (c, p) => c := Mux(p.isFree, 0.U, c + 1.U) }
  cntReg.zipWithIndex.foreach { case (c, i) => assert(c < TIMEOUT_SMINTF.U, "SNMAS Intf[0x%x] STATE[0x%x] ADDR[0x%x] OP[0x%x] TIMEOUT", i.U, entrys(i).state, entrys(i).fullAddr(io.pcuID), entrys(i).chiMes.opcode) }

// -------------------------------------------------- Perf Counter ------------------------------------------------------ //
  require(param.nrEntry >= 4 & param.nrEntry % 4 == 0)
  for (i <- 0 until (param.nrEntry / 4)) {
    XSPerfAccumulate(s"pcu_localSnMaster_entry_group[${i}]_deal_req_cnt", io.req2Intf.fire & (i * 4).U <= entryGetReqID & entryGetReqID <= (i * 4 + 3).U)
  }
  XSPerfAccumulate("pcu_localSnMaster_req_cnt", io.req2Intf.fire)
  XSPerfAccumulate("pcu_localRnSlave_req_block_cnt", io.req2Intf.valid & entryFreeNum === 0.U)
}