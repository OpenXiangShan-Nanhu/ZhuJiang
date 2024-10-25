package dongjiang.pcu.intf

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import dongjiang.chi.CHIOp.REQ._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.FastArb._

/*
 * ************************************************************** State transfer ***********************************************************************************
 *
 * Read Req:  [Free] -----> [GetDBID] -----> [WaitDBID] -----> [Req2Node] -----> [WaitNodeData] -----> [Resp2Slice]
 *
 * Write Req: [Free] -----> [Req2Node] -----> [WaitNodeDBID] -----> [RCDB] -----> [WriteData2Node] -----> [WaitNodeComp] -----> [Resp2Slice]
 *
 * Replace:   [Free] -----> [Req2Node] -----> [WaitNodeDBID] -----> [Replace2Node] -----> [WaitReplDBID] -----> [RCDB] -----> [WriteData2Node] -----> [WaitNodeComp] -----> [Resp2Slice]
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
 * { from(incoID) | to(incoID) | entryID | mshrIdx(mshrWay | mshrSet) | dbid | dcuIdx }
 *
 *
 * Read: TODO: When using Read without DMT need to make sure that the RnSlave does not fill up the DataBuffer.
 * { Req2Node        } Req    From Slice And Store In Intf                                                                                                  | { pcuIdx.mshrIdx = pcuIdx.mshrIdx } { pcuIdx.dcuIdx = pcuIdx.dcuIdx }
 * { Read            } Req    Send To CHI                         { TgtID = tgtID } { ReturnNID = hnfID } { ReturnTxnID = entryID }                         |
 * { CompData        } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Slice      } Resp   Send To Slice                                                                                                                 | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
 *
 *
 * Read With DMT: Not implemented in the system
 * { Req2Node        } Req    From Slice And Store In Intf        { chiIdx.nodeID = chiIdx.nodeID } { chiIdx.txnID =  chiIdx.txnID }                        | { pcuIdx.mshrIdx = pcuIdx.mshrIdx }
 * { Read            } Req    Send To CHI                         { TgtID = chiMes.tgtID } { TxnID = Cat(chiMes.bankID, pcuIdx.mshrIdx) } { ReturnNID = chiIdx.nodeID } { ReturnTxnID = chiIdx.txnID }
 *
 *
 * Write:
 * { Req2Node        } Req    From Slice And Store In Intf                                                                                                  | { pcuIdx.mshrIdx = pcuIdx.mshrIdx } { pcuIdx.dbid = pcuIdx.dbid } { pcuIdx.dcuIdx = pcuIdx.dcuIdx }
 * { Write           } Req    Send To CHI                         { TgtID = tgtID } { TxnID = entryID } { ReturnNID = hnfID } { ReturnTxnID = entryID }     |
 * { DBIDResp        } Resp   From CHI And Match With Entry ID    { TxnID == entryID } { chiIdx.txnID = DBID } (Store DBID In chiIdx.txnID)                 |
 * { NCBWrData       } Data   Send To CHI                         { TgtID = tgtID } { TxnID = chiIdx.txnID }                                                |
 * { Comp            } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Slice      } Resp   Send To Slice                                                                                                                 | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
 *
 *
 * Write With DWT: TODO
 * { Req2Node        } Req    From Slice And Store In Intf        { chiIdx.nodeID = chiIdx.nodeID } { chiIdx.txnID =  chiIdx.txnID }                        | { pcuIdx.mshrIdx = pcuIdx.mshrIdx } { pcuIdx.dcuIdx = pcuIdx.dcuIdx }
 * { Write           } Req    Send To CHI                         { TgtID = tgtID } { TxnID = entryID } { ReturnNID = chiIdx.nodeID } { ReturnTxnID = chiIdx.txnID } |
 * { Comp            } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Slice      } Resp   Send To Slice                                                                                                                 | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
 *
 * Replace:
 * { Req2Node        } Req    From Slice And Store In Intf                                                                                                  | { pcuIdx.mshrIdx = pcuIdx.mshrIdx } { pcuIdx.dbid = pcuIdx.dbid } { pcuIdx.dcuIdx = pcuIdx.dcuIdx }
 * { Write           } Req    Send To CHI                         { TgtID = ddrcID } { TxnID = entryID } { ReturnNID = hnfID } { ReturnTxnID = entryID }    |
 * { DBIDResp        } Resp   From CHI And Match With Entry ID    { TxnID == entryID } { chiIdx.txnID = DBID } (Store DBID In chiIdx.txnID)                 |
 * { Replace         } Req    Send To CHI                         { TgtID = tgtID } { TxnID = entryID } { ReturnNID = ddrcID } { ReturnTxnID = chiIdx.txnID } |
 * { NCBWrData       } Data   Send To CHI                         { TgtID = tgtID } { TxnID = chiIdx.txnID }                                                |
 * { Comp            } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Slice      } Resp   Send To Slice                                                                                                                 | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALMAS } { pcuIdx.mshrIdx = mshrIdx }
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
  val Resp2Slice      = "b0101".U // 0x5
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
  val state         = UInt(SMState.width.W)
  val indexMes      = new DJBundle with HasAddr with HasFromIncoID with HasMSHRWay with HasDBID {
    val selfWay     = UInt(sWayBits.W)
  }
  val chiMes        = new SNMASCHIMesBundle()
  val hasData       = Bool()
  val getDataNum    = UInt(beatNumBits.W)
  val alrGetComp    = Bool() // Already Get Comp

  def reqAddr(tgtId: UInt): UInt = {
    val addr        = Wire(UInt(addressBits.W))
    when(tgtId === ddrcNodeId.U) { addr := indexMes.addr
    }.otherwise { addr := getDCUAddress(indexMes.addr, indexMes.selfWay) }
    addr
  }

  def isFree        = state === SMState.Free
  def isGetDBID     = state === SMState.GetDBID
  def isReq2Node    = state === SMState.Req2Node
  def isRepl2Node   = state === SMState.Replace2Node
  def isResp2Slice  = state === SMState.Resp2Slice
  def isWaitDBData  = state === SMState.WriteData2Node
  def isRCDB        = state === SMState.RCDB
  def isLastBeat    = getDataNum === (nrBeat - 1).U
  def isRead        = isReadX(chiMes.opcode)
  def isWrite       = isWriteX(chiMes.opcode)
  def isRepl        = isReplace(chiMes.opcode)
}

class SnMasterIntf(snMasId: Int, param: InterfaceParam)(implicit p: Parameters) extends IntfBaseIO(isSlv = false, hasReq2Slice = false, hasDBRCReq = true) {
  // Del it
  io <> DontCare
  dontTouch(io)
// --------------------- Reg and Wire declaration ------------------------//
  val entrys          = RegInit(VecInit(Seq.fill(param.nrEntry) { 0.U.asTypeOf(new SMEntry(param)) })); dontTouch(entrys)
  // Intf Receive Req ID
  val entryGetReqID   = Wire(UInt(param.entryIdBits.W))
  // Intf Get DBID ID
  val entryGetDBID    = Wire(UInt(param.entryIdBits.W))
  // Intf Receive DBID ID
  val entryRecDBID    = Wire(UInt(param.entryIdBits.W))
  // Intf Req To Node ID
  val entryReq2NodeID = Wire(UInt(param.entryIdBits.W))
  // Intf Send Resp To Slice ID
  val entryResp2SliceID = Wire(UInt(param.entryIdBits.W))
  // Intf Send RC Req To DataBuufer ID
  val entryRCDBID     = Wire(UInt(param.entryIdBits.W))
  // req from slice
  val indexSaveInIntf = WireInit(0.U.asTypeOf(entrys(0).indexMes))
  val reqSaveInIntf   = WireInit(0.U.asTypeOf(entrys(0).chiMes))


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------  Update Entry Value  --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.zipWithIndex.foreach {
    case (entry, i) =>
      /*
       * Receive New Req
       */
      when(io.req2Node.fire & entryGetReqID === i.U) {
        entry               := 0.U.asTypeOf(entry) // Clean Intf Entry When ReAlloc
        entry.indexMes      := indexSaveInIntf
        entry.chiMes        := reqSaveInIntf
        assert(entry.state === SMState.Free, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
      /*
       * Receive DBID From DataBuffer
       */
      }.elsewhen(io.dbSigs.wResp.fire & entryRecDBID === i.U) {
        entry.hasData       := true.B
        entry.indexMes.dbid := io.dbSigs.wResp.bits.dbid
        assert(entry.state === SMState.WaitDBID, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
      /*
       * Receive Data And Resp From CHI RxDat
       */
      }.elsewhen(io.chi.rxdat.fire & io.chi.rxdat.bits.TxnID === i.U) {
        entry.getDataNum    := entry.getDataNum + 1.U
        entry.chiMes.resp   := io.chi.rxdat.bits.Resp
        assert(io.chi.rxdat.bits.Opcode === CHIOp.DAT.CompData, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
        assert(entry.isRead, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
        assert(entry.state === SMState.WaitNodeData, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
      /*
       * Receive DBID or Comp From CHI RxRsp
       */
      }.elsewhen(io.chi.rxrsp.fire & io.chi.rxrsp.bits.TxnID === i.U) {
        when(io.chi.rxrsp.bits.Opcode === CHIOp.RSP.DBIDResp) {
          entry.chiMes.chiDBID := io.chi.rxrsp.bits.DBID
          assert(entry.state === SMState.WaitReplDBID | entry.state === SMState.WaitNodeDBID, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
        }
        when(io.chi.rxrsp.bits.Opcode === CHIOp.RSP.Comp) {
          entry.alrGetComp  := true.B
          assert(Mux(!entry.isRepl | entry.alrGetComp, entry.state === SMState.WaitNodeComp,
            entry.state === SMState.WaitReplDBID | entry.state === SMState.RCDB | entry.state === SMState.WriteData2Node |  entry.state === SMState.WaitNodeComp),
            "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
        }

      /*
       * Receive Data From DataBuffer
       */
      }.elsewhen(io.dbSigs.dataFDB.fire & entry.isWaitDBData & io.dbSigs.dataFDB.bits.dbid === entry.indexMes.dbid) {
        entry.getDataNum    := entry.getDataNum + 1.U
        assert(entry.state === SMState.WriteData2Node, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
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
          val reqHit    = io.req2Node.fire & isReadX(io.req2Node.bits.opcode) & entryGetReqID === i.U; assert(!reqHit | io.req2Node.bits.opcode === ReadNoSnp)
          val writeHit  = io.req2Node.fire & isWriteX(io.req2Node.bits.opcode) & entryGetReqID === i.U; assert(!writeHit | io.req2Node.bits.opcode === WriteNoSnpFull)
          val replHit   = io.req2Node.fire & isReplace(io.req2Node.bits.opcode) & entryGetReqID === i.U; assert(!replHit | io.req2Node.bits.opcode === Replace)
          entry.state     := Mux(reqHit, SMState.GetDBID,
                            Mux(writeHit, SMState.Req2Node,
                              Mux(replHit, SMState.Req2Node, entry.state)))
        }
        // State: GetDBID
        is(SMState.GetDBID) {
          val hit       = io.dbSigs.wReq.fire & entryGetDBID === i.U
          entry.state     := Mux(hit, SMState.WaitDBID, entry.state)
          assert(entry.isRead | !hit)
        }
        // State: WaitDBID
        is(SMState.WaitDBID) {
          val hit       = io.dbSigs.wResp.fire & entryRecDBID === i.U
          entry.state     := Mux(hit, SMState.Req2Node, entry.state)
        }
        // State: Req2Node
        is(SMState.Req2Node) {
          val hit       = io.chi.txreq.fire & entryReq2NodeID === i.U
          entry.state     := Mux(hit, Mux(entry.isRead, SMState.WaitNodeData, SMState.WaitNodeDBID), entry.state)
        }
        // State: WaitNodeResp
        is(SMState.WaitNodeData) {
          val rxDatHit  = io.chi.rxdat.fire & io.chi.rxdat.bits.TxnID === i.U
          entry.state     := Mux(rxDatHit & entry.isLastBeat, SMState.Resp2Slice, entry.state)
        }
        // State: Resp2Slice
        is(SMState.Resp2Slice) {
          val hit       = io.resp2Slice.fire & entryResp2SliceID === i.U
          entry.state     := Mux(hit, SMState.Free, entry.state)
        }
        // State: WaitNodeDBID
        is(SMState.WaitNodeDBID) {
          val hit       = io.chi.rxrsp.fire & io.chi.rxrsp.bits.TxnID === i.U
          entry.state     := Mux(hit, Mux(entry.isRepl, SMState.Replace2Node, SMState.RCDB), entry.state)
        }
        // State: RCDB
        is(SMState.RCDB) {
          val hit       = io.dbSigs.dbRCReq.fire & entryRCDBID === i.U
          entry.state     := Mux(hit, SMState.WriteData2Node, entry.state)
        }
        // State: WriteData2Node
        is(SMState.WriteData2Node) {
          val hit       = io.chi.txdat.fire & entry.isLastBeat & io.dbSigs.dataFDB.bits.dbid === entry.indexMes.dbid
          entry.state     := Mux(hit, SMState.WaitNodeComp, entry.state)
        }
        // State: WaitNodeComp
        is(SMState.WaitNodeComp) {
          val hit       = io.chi.rxrsp.fire & io.chi.rxrsp.bits.TxnID === i.U
          val canGo     = !entry.isRepl | entry.alrGetComp
          entry.state     := Mux(hit & canGo,  SMState.Resp2Slice, entry.state)
        }
        // State: Replace2Node
        is(SMState.Replace2Node) {
          val hit       = io.chi.txreq.fire & entryReq2NodeID === i.U; assert(!hit | entry.isRepl, "SNMAS Intf[0x%x] STATE[0x%x] OP[0x%x] ADDR[0x%x] TGTID[0x%x]", i.U, entry.state, entry.chiMes.opcode, entry.indexMes.addr, entry.chiMes.tgtID)
          entry.state     := Mux(hit, SMState.WaitReplDBID, entry.state)
        }
        // State: WaitReplDBID
        is(SMState.WaitReplDBID) {
          val hit       = io.chi.rxrsp.fire & io.chi.rxrsp.bits.TxnID === i.U
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
  indexSaveInIntf.addr     := io.req2Node.bits.addr
  indexSaveInIntf.selfWay  := io.req2Node.bits.selfWay
  indexSaveInIntf.mshrWay  := io.req2Node.bits.mshrWay
  indexSaveInIntf.from     := io.req2Node.bits.from
  indexSaveInIntf.dbid     := io.req2Node.bits.dbid
  reqSaveInIntf.resp       := io.req2Node.bits.resp
  reqSaveInIntf.opcode     := io.req2Node.bits.opcode
  reqSaveInIntf.tgtID      := io.req2Node.bits.tgtID
  reqSaveInIntf.expCompAck := io.req2Node.bits.expCompAck
  assert(Mux(io.req2Node.valid, !io.req2Node.bits.expCompAck, true.B))

  /*
   * Set Intf Value
   */
  val entryFreeVec        = entrys.map(_.isFree)
  val entryFreeNum        = PopCount(entryFreeVec)
  entryGetReqID           := PriorityEncoder(entryFreeVec)

  /*
   * Set Ready Value
   */
  io.req2Node.ready       := entryFreeNum > 0.U



// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- Get DBID From DataBuffer and Wait DataBuffer Resp ---------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Send Get DBID Req To From DataBuffer
   */
  val entryGetDBIDVec     = entrys.map(_.isGetDBID)
  entryGetDBID            := PriorityEncoder(entryGetDBIDVec)

  /*
   * Set DataBuffer Req Value
   */
  io.dbSigs.wReq.valid            := entryGetDBIDVec.reduce(_ | _)
  io.dbSigs.wReq.bits.from.IncoId := snMasId.U
  io.dbSigs.wReq.bits.pcuId     := entryGetDBID

  /*
   * Receive DBID From DataBuffer
   */
  entryRecDBID := io.dbSigs.wResp.bits.pcuId
  io.dbSigs.wResp.ready := true.B



// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- Send Req To Node -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val entryReq2NodeVec          = entrys.map { case p => p.isReq2Node | p.isRepl2Node }
  entryReq2NodeID               := PriorityEncoder(entryReq2NodeVec)

  val writeDDR                  = entrys(entryReq2NodeID).isReq2Node & entrys(entryReq2NodeID).isRepl

  io.chi.txreq.valid            := entryReq2NodeVec.reduce(_ | _)
  io.chi.txreq.bits.Addr        := entrys(entryReq2NodeID).reqAddr(io.chi.txreq.bits.TgtID)
  //                                             Send Write To DDR(When its Repl)     Send Req To DDR / DCU
  io.chi.txreq.bits.Opcode      := Mux(writeDDR, WriteNoSnpFull,                      entrys(entryReq2NodeID).chiMes.opcode)
  io.chi.txreq.bits.TgtID       := Mux(writeDDR, ddrcNodeId.U,                        entrys(entryReq2NodeID).chiMes.tgtID)
  io.chi.txreq.bits.TxnID       := entryReq2NodeID
  io.chi.txreq.bits.SrcID       := io.hnfID
  io.chi.txreq.bits.Size        := log2Ceil(djparam.blockBytes).U
  io.chi.txreq.bits.MemAttr     := entrys(entryReq2NodeID).chiMes.resp // Multiplex MemAttr to transfer CHI State // Use in Read Req
  //                                                                   Send Replace To DCU                      Send Req To DDR / DCU
  io.chi.txreq.bits.ReturnNID   := Mux(entrys(entryReq2NodeID).isRepl, ddrcNodeId.U,                            io.hnfID)
  io.chi.txreq.bits.ReturnTxnID := Mux(entrys(entryReq2NodeID).isRepl, entrys(entryReq2NodeID).chiMes.chiDBID,  entryReq2NodeID)


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------- Receive CHI DBID From From Node ------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  io.chi.rxrsp.ready            := true.B
  assert(Mux(io.chi.rxrsp.fire, io.chi.rxrsp.bits.Opcode === CHIOp.RSP.DBIDResp | io.chi.rxrsp.bits.Opcode === CHIOp.RSP.Comp, true.B))


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- Read And Clean DataBuffer --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val entryRCDBIDVec                = entrys.map(_.isRCDB)
  entryRCDBID                       := PriorityEncoder(entryRCDBIDVec)

  io.dbSigs.dbRCReq.valid           := entryRCDBIDVec.reduce(_ | _)
  io.dbSigs.dbRCReq.bits.isRead     := true.B
  io.dbSigs.dbRCReq.bits.isClean    := true.B
  io.dbSigs.dbRCReq.bits.dbid       := entrys(entryRCDBID).indexMes.dbid
  io.dbSigs.dbRCReq.bits.to.IncoId  := snMasId.U


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------- Receive Data From DataBuffer And Send Data To Node ----------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val entrySendDatVec           = entrys.map { case p => p.state === SMState.WriteData2Node & p.indexMes.dbid === io.dbSigs.dataFDB.bits.dbid }
  val entrySendDatID            = PriorityEncoder(entrySendDatVec)
  assert(Mux(io.chi.txdat.valid, PopCount(entrySendDatVec) === 1.U, true.B))

  io.chi.txdat.valid            := io.dbSigs.dataFDB.valid
  io.chi.txdat.bits             := DontCare
  io.chi.txdat.bits.Opcode      := CHIOp.DAT.NonCopyBackWrData
  io.chi.txdat.bits.TgtID       := entrys(entrySendDatID).chiMes.tgtID
  io.chi.txdat.bits.SrcID       := io.hnfID
  io.chi.txdat.bits.TxnID       := entrys(entrySendDatID).chiMes.chiDBID
  io.chi.txdat.bits.DataID      := io.dbSigs.dataFDB.bits.dataID
  io.chi.txdat.bits.Data        := io.dbSigs.dataFDB.bits.data
  io.chi.txdat.bits.BE          := Fill(io.chi.rxdat.bits.BE.getWidth, 1.U(1.W))

  io.dbSigs.dataFDB.ready       := io.chi.txdat.ready

// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- Receive Data From CHI DAT And Send It To DataBuffer -------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  io.dbSigs.dataTDB.valid       := io.chi.rxdat.valid
  io.dbSigs.dataTDB.bits.dbid   := entrys(io.chi.rxdat.bits.TxnID(param.entryIdBits-1, 0)).indexMes.dbid
  io.dbSigs.dataTDB.bits.data   := io.chi.rxdat.bits.Data
  io.dbSigs.dataTDB.bits.dataID := io.chi.rxdat.bits.DataID
  io.chi.rxdat.ready            := io.dbSigs.dataTDB.ready
  assert(Mux(io.chi.rxdat.valid, io.chi.rxdat.bits.TxnID <= param.nrEntry.U, true.B))



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------ Send Resp To Slice -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val entryResp2SliceVec          = entrys.map(_.isResp2Slice)
  entryResp2SliceID               := PriorityEncoder(entryResp2SliceVec)

  io.resp2Slice.valid             := entryResp2SliceVec.reduce(_ | _)
  io.resp2Slice.bits              := DontCare
  io.resp2Slice.bits.isReqResp    := entrys(entryResp2SliceID).isRead
  io.resp2Slice.bits.isWriResp    := entrys(entryResp2SliceID).isWrite | entrys(entryResp2SliceID).isRepl
  io.resp2Slice.bits.mshrSet      := entrys(entryResp2SliceID).indexMes.mSet
  io.resp2Slice.bits.mshrWay      := entrys(entryResp2SliceID).indexMes.mshrWay
  io.resp2Slice.bits.from.IncoId  := snMasId.U
  io.resp2Slice.bits.to           := entrys(entryResp2SliceID).indexMes.from
  io.resp2Slice.bits.hasData      := entrys(entryResp2SliceID).hasData
  io.resp2Slice.bits.dbid         := entrys(entryResp2SliceID).indexMes.dbid
  io.resp2Slice.bits.resp         := entrys(entryResp2SliceID).chiMes.resp



// ---------------------------  Assertion  --------------------------------//
  val cntReg = RegInit(VecInit(Seq.fill(param.nrEntry) {
    0.U(64.W)
  }))
  cntReg.zip(entrys).foreach { case (c, p) => c := Mux(p.isFree, 0.U, c + 1.U) }
  cntReg.zipWithIndex.foreach { case (c, i) => assert(c < TIMEOUT_SMINTF.U, "SNMAS Intf[0x%x] STATE[0x%x] ADDR[0x%x] OP[0x%x] TIMEOUT", i.U, entrys(i).state, entrys(i).indexMes.addr, entrys(i).chiMes.opcode) }
}