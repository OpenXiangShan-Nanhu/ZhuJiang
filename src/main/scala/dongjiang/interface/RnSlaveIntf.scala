package dongjiang.pcu.intf

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import dongjiang.chi.CHIOp.REQ._
import dongjiang.chi.CHIOp.RSP._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.Encoder._
import xs.utils._

/*
 * ************************************************************** State transfer ***********************************************************************************
 * Req Contain Read and Dataless
 * Req Retry:             [Free]  -----> [Req2Slice] -----> [WaitSliceAck] ---retry---> [Req2Slice]
 * Req Receive:           [Free]  -----> [Req2Slice] -----> [WaitSliceAck] --receive--> [Free]
 *
 *
 * Resp Need Read DB:     [Free]  -----> [RCDB] -----> [Resp2Node] -----> [WaitCompAck]
 * Resp Not Need Read DB: [Free]         ----->        [Resp2Node] -----> [WaitCompAck]
 *
 *
 * Write Retry:           [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Slice] -----> [WaitSliceAck] ---retry---> [Req2Slice]
 * Write:                 [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [DBIDResp2Node] -----> [WaitData] -----> [Req2Slice] -----> [WaitSliceAck] --receive--> [Free]
 * Write Nest By Snp case:          ^                 ^                     ^                 ^                  ^                 ^
 *                              waitWBDone       waitWBDone             waitWBDone        waitWBDone  ----->  trans2Snp        trans2Snp
 *
 *
 * Snoop Need Data:       [Free]  -----> [GetDBID] -----> [WaitDBID] -----> [Snp2Node] -----> ([Snp2NodeIng]) -----> [WaitSnpResp] -----> [Resp2Slice]
 * Snoop No Need Data:    [Free]                   ----->                   [Snp2Node] -----> ([Snp2NodeIng]) -----> [WaitSnpResp] -----> [Resp2Slice]
 * Snoop Nest Write 0:    [XXXX]                   ----->                   [Snp2Node] -----> ([Snp2NodeIng]) -----> [WaitSnpResp] -----> [Resp2Slice]
 * Snoop Nest Write 1:    [XXXX]                   ----->                                                                                 [Resp2Slice] * only need to snp one node
 *                                                   ^
 *                                                trans2Snp
 *
 *
 *
 * ************************************************************** Entry Transfer ********************************************************************************
 * Req:
 * [------] <- Req0x0 | [Req0x0] -> Req to slice | [------] <- Resp from slice | [Rsp0x0] -> resp to node
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 *
 *
 * Write:
 * [------] <- Wri0x0 | [Wri0x0] -> Wri to slice | [Wri0x0] <- Resp from slice | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 *
 *
 * Snoop:
 * [Snp0x0] <- Snp0x0 | [Snp0x0] -> Snp to node  | [Snp0x0] <- Resp from node  | [------] -> resp to slice
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 * [------]           | [------]                 | [------]                    | [------]
 *
 *
 * * Req Nest By Snp:
 * [------] <- Req0x0 | [Req0x0]                 | [Req0x0] need to wait snoop done
 * [------]           | [------] <-Snp0x0        | [Snp0x0]
 * [------]           | [------]                 | [------]
 * [------]           | [------]                 | [------]
 *
 *
 * * Write Nest By Snp:
 * [------] <- Wri0x0 | [Wri0x0] <-Snp0x0        | [Wri0x0] need to wait write done and transfer write to snoop
 * [------]           | [------]                 | [------]
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
 * { from(incoID) | to(incoID) | entryID | mshrIdx(mshrWay | mshrSet) | dbid }
 *
 *
 * Read / Dataless:
 * { Read / Dataless } TxReq  From CHI And Store In Intf          { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        |
 * { Req2Slice       } Req    From Intf And Send To Slice         { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALSLV } { pcuIdx.entryID = entryID }
 * { ReqAck          } ReqAck From Slice and Match With Entry ID                                                                                            | { pcuIdx.entryID == entryID }
 * { Resp2Node       } Resp   From Slice And Store In Intf        { chiIdx.nodeID = tgtID } { chiIdx.txnID = txnID }                                        | { pcuIdx.dbid = pcuIdx.dbid }
 * { Comp(Data)      } RxRsp  Send To CHI                         { TgtID = chiIdx.nodeID } { TxnID = chiIdx.txnID } { HomeNID = hnfID } { DBID = entryID } |
 * { CompAck         } TxRsp  From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 *
 *
 * Read with DMT: TODO
 * { Read            } TxReq  From CHI And Store In Intf          { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        |
 * { Req2Slice       } Req    From Intf And Send To Slice         { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALSLV } { pcuIdx.entryID = entryID }
 * { ReqAck          } ReqAck From Slice and Match With Entry ID                                                                                            | { pcuIdx.entryID == entryID }
 * { CompAck         } TxRsp  From CHI And Send Resp To Slice                                                                                               | { pcuIdx.to = TxnID.head } { pcuIdx.mshrIdx = TxnID.tail }
 *
 *
 * Write:
 * { Write           } TxReq  From CHI And Store In Intf          { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        |
 * { CompDBIDResp    } RxRsp  Send To CHI                         { TgtID = chiIdx.nodeID } { TxnID = chiIdx.txnID } { DBID = entryID }                     |
 * { WriteData       } TxRsp  From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Req2Slice       } Req    From Intf And Send To Slice                                                                                                   | { pcuIdx.to = chiMes.bankID } { pcuIdx.incfrom = LOCALSLV } { pcuIdx.entryID = entryID } { pcuIdx.dbid = pcuIdx.dbid }
 * { ReqAck          } ReqAck From Slice and Match With Entry ID                                                                                            | { pcuIdx.entryID == entryID }
 *
 *
 * Write with DWT: Not implemented in the system
 * { Write           } TxReq  From CHI And Store In Intf          { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        |
 * { Req2Slice       } Req    From Intf And Send To Slice         { chiIdx.nodeID = SrcID } { chiIdx.txnID = TxnID }                                        | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALSLV } { pcuIdx.entryID = entryID }
 * { ReqAck          } ReqAck From Slice and Match With Entry ID                                                                                            | { pcuIdx.entryID == entryID }
 *
 *
 * Snoop:
 * { Req2Node        } Req    From Slice And Store In Intf                                                                                                  | { pcuIdx.snpVec = idx.snpVec } { pcuIdx.mshrIdx = pcuIdx.mshrIdx }
 * { Snoop           } Req    Send To CHI                         { TgtID = snpID } { TxnID = entryID }                                                     |
 * { SnResp(Data)    } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { Resp2Slice      } Resp   Send To Slice                                                                                                                 | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALSLV } { pcuIdx.mshrIdx = mshrIdx }
 *
 *
 * Snoop with DCT: TODO
 * { Req2Node        } Req    From Slice And Store In Intf        { chiIdx.nodeID =  chiIdx.nodeID } { chiIdx.txnID =  chiIdx.txnID }                       | { pcuIdx.snpVec = idx.snpVec } { pcuIdx.mshrIdx = pcuIdx.mshrIdx }
 * { Snoop           } Req    Send To CHI                         { TgtID = snpID } { TxnID = entryID }                                                     |
 * { SnResp          } Resp   From CHI And Match With Entry ID    { TxnID == entryID }                                                                      |
 * { SnoopFwd        } Req    Send To CHI                         { TgtID = snpID } { TxnID = entryID } { FwdNID = chiIdx.nodeID } { FwdTxnID = chiIdx.txnID }
 * { SnpResp(Data)Fwded } Resp From CHI And Match With Entry ID   { TxnID == entryID }                                                                      |
 * { Resp2Slice      } Resp   Send To Slice                                                                                                                 | { pcuIdx.to = chiMes.bankID } { pcuIdx.from = LOCALSLV } { pcuIdx.mshrIdx = mshrIdx } { pcuIdx.dbid = pcuIdx.dbid }
 *
 *
 */

object RSState {
  val width = 4
  // commom
  val Free            = "b0000".U // 0x0
  val Req2Slice       = "b0001".U // 0x1
  val WaitSliceAck    = "b0010".U // 0x2
  val WaitSliceResp   = "b0011".U // 0x3
  val RCDB            = "b0100".U // 0x4 // Read & Clean DataBuffer
  val Resp2Node       = "b0101".U // 0x5
  val WaitCompAck     = "b0110".U // 0x6
  val Resp2Slice      = "b0111".U // 0x7
  val GetDBID         = "b1000".U // 0x8
  val WaitDBID        = "b1001".U // 0x9
  val DBIDResp2Node   = "b1010".U // 0xa
  val WaitData        = "b1011".U // 0xb
  val Snp2Node        = "b1100".U // 0xc
  val Snp2NodeIng     = "b1101".U // 0xd
  val WaitSnpResp     = "b1110".U // 0xe
}

class RSEntry(param: InterfaceParam)(implicit p: Parameters) extends DJBundle  {
  val state         = UInt(RSState.width.W)
  val nid           = UInt(param.entryIdBits.W)
  val indexMes      = new DJBundle with HasAddr with HasFromIncoID with HasMSHRWay with HasDBID
  val nestMes       = new Bundle {
    val waitWBDone  = Bool()
    val trans2Snp   = Bool()
  }
  val chiMes        = new RNSLVCHIMesBundle()
  val hasData       = Bool()
  val getDataNum    = UInt(beatNumBits.W)
  val getSnpRespOH  = UInt(nrRnfNode.W)

  def isFree        = state === RSState.Free
  def isReqBeSend   = state === RSState.Req2Slice & !nestMes.asUInt.orR & nid === 0.U
  def isRspBeSend   = (state === RSState.Resp2Node & chiMes.isRsp) | state === RSState.DBIDResp2Node
  def isDatBeSend   = state === RSState.Resp2Node & chiMes.isDat
  def isGetDBID     = state === RSState.GetDBID & nid === 0.U
  def isSendSnp     = state === RSState.Snp2Node
  def isSendSnpIng  = state === RSState.Snp2NodeIng
  def isLastBeat    = getDataNum === (nrBeat - 1).U
}




class RnSlaveIntf(rnSlvId: Int, param: InterfaceParam)(implicit p: Parameters) extends IntfBaseIO(isSlv = true, hasReq2Slice = true, hasDBRCReq = true) {
  // Del it
  io <> DontCare
  dontTouch(io)
// --------------------- Reg and Wire declaration ------------------------//
  val entrys            = RegInit(VecInit(Seq.fill(param.nrEntry) { 0.U.asTypeOf(new RSEntry(param)) }))
  // ENTRY Receive Task ID
  val entryFreeID       = Wire(UInt(param.entryIdBits.W))
  // ENTRY Send Req To Slice
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
  // ENTRY Send Resp To Slice ID
  val entryResp2SliceID = Wire(UInt(param.entryIdBits.W))
  // req from slice or txreq
  val taskNID           = Wire(UInt(param.entryIdBits.W))
  val taskSaveInIntf    = WireInit(0.U.asTypeOf(entrys(0).chiMes))
  val indexSaveInIntf   = WireInit(0.U.asTypeOf(entrys(0).indexMes))
  // snp to node
  val snpIsLast         = Wire(Bool())
  val snpAlreadySendVecReg = RegInit(0.U(nrRnfNode.W))



// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------------  Update Entry Value --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.zipWithIndex.foreach {
    case (entry, i) =>
      // ------------------------------------------- Update Base Values -------------------------------------------------- //
      /*
       * Receive New Req
       */
      when((io.chi.txreq.fire | io.req2Node.fire | io.resp2Node.fire) & entryFreeID === i.U) {
        entry               := 0.U.asTypeOf(entry)
        entry.indexMes      := indexSaveInIntf
        entry.chiMes        := taskSaveInIntf
        entry.nestMes       := 0.U.asTypeOf(entry.nestMes)
        assert(entry.state === RSState.Free, "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
      /*
       * Receive DBID From DataBuffer
       */
      }.elsewhen(io.dbSigs.wResp.fire & entryRecDBID === i.U) {
        entry.hasData       := true.B
        entry.indexMes.dbid := io.dbSigs.wResp.bits.dbid
        assert(entry.state === RSState.WaitDBID, "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
      /*
       * Receive CHI TX Dat or CHI TX Rsp
       */
      }.elsewhen((io.chi.txdat.fire & entryRecChiDatID === i.U) | (io.chi.txrsp.fire & entryRecChiRspID === i.U)) {
        val hitRespDat      = io.chi.txdat.fire & entryRecChiDatID === i.U
        val hitRespRsp      = io.chi.txrsp.fire & entryRecChiRspID === i.U
        entry.getDataNum    := entry.getDataNum + hitRespDat.asUInt
        entry.chiMes.resp   := Mux(hitRespDat, io.chi.txdat.bits.Resp, Mux(hitRespRsp & !entry.chiMes.retToSrc, io.chi.txrsp.bits.Resp, entry.chiMes.resp))
        when(hitRespDat) {
          assert(Mux(entry.chiMes.isSnp & entry.chiMes.retToSrc, entry.state === RSState.Snp2NodeIng | entry.state === RSState.WaitSnpResp, entry.state === RSState.WaitData), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
        }.elsewhen(hitRespRsp) {
          when(io.chi.txrsp.bits.Opcode === CHIOp.RSP.CompAck) { assert(entry.state === RSState.WaitCompAck, "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state) }
          .otherwise {                                           assert(entry.state === RSState.Snp2NodeIng | entry.state === RSState.WaitSnpResp, "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state) }
        }
      /*
       * Clean ENTRY Entry When Its Free
       */
      }.elsewhen(entry.isFree) {
        entry               := 0.U.asTypeOf(entry)
      }


      // ---------------------------------------------------- Set Task NID ------------------------------------------------- //
      /*
       * Set New NID
       * TODO: Advance judgment to avoid generating multiple repetitive logic
       */
      when((io.chi.txreq.fire | io.req2Node.fire | io.resp2Node.fire) & entryFreeID === i.U) {
        val snp2IntfHit    = io.req2Node.fire    & io.req2Node.bits.addrNoOff                         === indexSaveInIntf.addrNoOff & io.req2Node.bits.isSnp      // SNP add NID
        val resp2SliceHit = io.resp2Slice.fire  & entrys(entryResp2SliceID).indexMes.addrNoOff           === indexSaveInIntf.addrNoOff                               // SNP reduce NID
        val reqAckHit     = io.reqAck2Node.fire & entrys(io.reqAck2Node.bits.pcuId).indexMes.addrNoOff === indexSaveInIntf.addrNoOff & !io.reqAck2Node.bits.retry  // REQ reduce NID
        // req
        when(io.chi.txreq.fire) {
          entry.nid       := taskNID - resp2SliceHit.asUInt - reqAckHit.asUInt
          // assert
          assert(taskNID >= (resp2SliceHit.asTypeOf(taskNID) + reqAckHit.asTypeOf(taskNID)), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
          assert(!(snp2IntfHit & resp2SliceHit), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
          assert(Mux(taskSaveInIntf.isReq & CHIOp.REQ.isWriteX(taskSaveInIntf.opcode), !snp2IntfHit & !resp2SliceHit, true.B), "TODO")
        // snp or resp
        }.otherwise {
          entry.nid       := 0.U
          assert(taskNID === 0.U)
        }
      /*
       * Modify NID
       */
      }.elsewhen(!entry.isFree & entry.chiMes.isReq) {
        val snp2IntfHit   = io.req2Node.fire    & io.req2Node.bits.addrNoOff                         === entry.indexMes.addrNoOff & io.req2Node.bits.isSnp  // SNP add NID
        val resp2SliceHit = io.resp2Slice.fire  & entrys(entryResp2SliceID).indexMes.addrNoOff           === entry.indexMes.addrNoOff                           // SNP reduce NID
        val reqAckHit     = io.reqAck2Node.fire & entrys(io.reqAck2Node.bits.pcuId).indexMes.addrNoOff === entry.indexMes.addrNoOff & !io.reqAck2Node.bits.retry & io.reqAck2Node.bits.pcuId =/= i.U // REQ reduce NID
        entry.nid         := entry.nid + snp2IntfHit.asUInt - resp2SliceHit.asUInt - reqAckHit.asUInt
        // assert
        assert((entry.nid + snp2IntfHit.asUInt) >= (resp2SliceHit.asTypeOf(taskNID) + reqAckHit.asTypeOf(taskNID)), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
        assert(!(snp2IntfHit & resp2SliceHit), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
        assert(Mux(entry.chiMes.isReq & CHIOp.REQ.isWriteX(entry.chiMes.opcode), !snp2IntfHit & !resp2SliceHit, true.B), "TODO")
      /*
       * Reset NID
       */
      }.elsewhen(entry.isFree) {
        entry.nid           := 0.U
      }


      // ---------------------------------------------- Record Snp Resp --------------------------------------------------- //
      when(entry.chiMes.isSnp) {
        when(entry.state === RSState.Snp2NodeIng | entry.state === RSState.WaitSnpResp) {
          val rspHit        = io.chi.txrsp.fire & entryRecChiRspID === i.U
          val datHit        = io.chi.txdat.fire & entryRecChiDatID === i.U & entry.isLastBeat
          val rspId         = getMetaIdByNodeID(io.chi.txrsp.bits.SrcID)
          val datId         = getMetaIdByNodeID(io.chi.txdat.bits.SrcID)
          val rspIdOH       = Mux(rspHit, UIntToOH(rspId), 0.U)
          val datIdOH       = Mux(datHit, UIntToOH(datId), 0.U)
          entry.getSnpRespOH  := entry.getSnpRespOH | rspIdOH | datIdOH
          // assert
          val getSnpRespVec = Wire(Vec(nrRnfNode, Bool()))
          val tgtSnpVec     = Wire(Vec(nrRnfNode, Bool()))
          getSnpRespVec     := entry.getSnpRespOH.asBools
          tgtSnpVec         := entry.chiMes.tgtID(nrRnfNode - 1, 0).asBools
          assert(Mux(rspHit, !getSnpRespVec(rspId), true.B), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
          assert(Mux(datHit, !getSnpRespVec(datId), true.B), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
          assert(Mux(rspHit, tgtSnpVec(rspId),      true.B), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
          assert(Mux(datHit, tgtSnpVec(datId),      true.B), "RNSLV ENTRY[0x%x] STATE[0x%x]", i.U, entry.state)
        }.otherwise {
          entry.getSnpRespOH  := 0.U
        }
      }
  }



// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------  State Transfer -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  entrys.zipWithIndex.foreach {
    case(entry, i) =>
      switch(entry.state) {
        // State: Free
        is(RSState.Free) {
          val hit       = entryFreeID === i.U
          val reqHit    = io.chi.txreq.fire & !isWriteX(io.chi.txreq.bits.Opcode) & hit
          val writeHit  = io.chi.txreq.fire & isWriteX(io.chi.txreq.bits.Opcode) & hit
          val snpHit    = io.req2Node.fire & hit
          val respHit   = io.resp2Node.fire & hit
          val ret2Src   = io.req2Node.bits.retToSrc
          val rDB       = io.resp2Node.bits.needReadDB; assert(Mux(hit, !rDB, true.B), "TODO")
          entry.state   := Mux(reqHit, RSState.Req2Slice,
                            Mux(snpHit & ret2Src, RSState.GetDBID,
                              Mux(snpHit & !ret2Src, RSState.Snp2Node,
                                Mux(writeHit, RSState.GetDBID,
                                  Mux(respHit & rDB, RSState.RCDB,
                                    Mux(respHit & !rDB, RSState.Resp2Node, entry.state))))))
          assert(PopCount(Seq(reqHit, writeHit, snpHit, respHit)) <= 1.U)
        }
        // State: Req2Slice
        is(RSState.Req2Slice) {
          val hit       = io.req2Slice.fire & entrySendReqID === i.U
          entry.state   := Mux(hit, RSState.WaitSliceAck, entry.state)
        }
        // State: WaitSliceAck
        is(RSState.WaitSliceAck) {
          val hit       = io.reqAck2Node.fire & io.reqAck2Node.bits.pcuId === i.U
          entry.state   := Mux(hit, Mux(io.reqAck2Node.bits.retry, RSState.Req2Slice, RSState.Free), entry.state)
        }
        // State: Resp2Node
        is(RSState.Resp2Node) {
          val rxDatHit  = io.chi.rxdat.fire & io.chi.rxdat.bits.DBID === i.U & toBeatNum(io.chi.rxdat.bits.DataID) === (nrBeat - 1).U
          val rxRspHit  = io.chi.rxrsp.fire & io.chi.rxrsp.bits.DBID === i.U
          val expAck    = entry.chiMes.expCompAck
          entry.state   := Mux(rxDatHit | rxRspHit, Mux(expAck, RSState.WaitCompAck, RSState.Free), entry.state)
        }
        // State: WaitCompAck
        is(RSState.WaitCompAck) {
          val hit       = io.chi.txrsp.fire & io.chi.txrsp.bits.TxnID === i.U
          assert(Mux(hit, io.chi.txrsp.bits.Opcode === CompAck, true.B))
          entry.state   := Mux(hit, RSState.Free, entry.state)
        }
        // State: GetDBID
        is(RSState.GetDBID) {
          val hit       = io.dbSigs.wReq.fire & entryGetDBID === i.U; assert(Mux(hit, entry.nid === 0.U, true.B)) // TODO: Consider Write can go no sorting required
          entry.state   := Mux(hit, RSState.WaitDBID, entry.state)
        }
        // State: WaitDBID
        is(RSState.WaitDBID) {
          val hit       = io.dbSigs.wResp.fire & entryRecDBID === i.U
          entry.state   := Mux(hit, Mux(entry.chiMes.isSnp, RSState.Snp2Node, RSState.DBIDResp2Node), entry.state)
        }
        // State: DBIDResp2Node
        is(RSState.DBIDResp2Node) {
          val hit       = io.chi.rxrsp.fire & io.chi.rxrsp.bits.DBID === i.U
          entry.state   := Mux(hit, RSState.WaitData, entry.state)
        }
        // State: WaitData
        is(RSState.WaitData) {
          val hit       = io.chi.txdat.fire & io.chi.txdat.bits.TxnID === i.U
          entry.state   := Mux(hit & entry.isLastBeat, RSState.Req2Slice, entry.state)
        }
        // State: Snp2Node
        is(RSState.Snp2Node) {
          val hit       = io.chi.rxsnp.fire & entrySendSnpID === i.U
          entry.state   := Mux(hit, Mux(snpIsLast, RSState.WaitSnpResp, RSState.Snp2NodeIng), entry.state)
        }
        // State: Snp2NodeIng
        is(RSState.Snp2NodeIng) {
          val hit       = io.chi.rxsnp.fire & entrySendSnpID === i.U
          entry.state   := Mux(hit & snpIsLast, RSState.WaitSnpResp, entry.state)
        }
        // State: WaitSnpResp
        is(RSState.WaitSnpResp) {
          val rspHit    = io.chi.txrsp.fire & entryRecChiRspID === i.U
          val datHit    = io.chi.txdat.fire & entryRecChiDatID === i.U
          val shlGetNum = PopCount(entry.getSnpRespOH ^ entry.chiMes.tgtID)
          val nowGetNum = rspHit.asTypeOf(UInt(2.W)) + (datHit & entry.isLastBeat).asTypeOf(UInt(2.W))
          entry.state   := Mux(shlGetNum === nowGetNum, RSState.Resp2Slice, entry.state)
        }
        // State: Resp2Slice
        is(RSState.Resp2Slice) {
          val hit       = io.resp2Slice.fire & entryResp2SliceID === i.U
          entry.state   := Mux(hit, RSState.Free, entry.state)
        }
      }
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------- Receive Req From CHITXREQ, Req2Node From Slice or Resp From Slice------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get ENTRY Free ID
   */
  val entryFreeVec  = entrys.map(_.isFree)
  val entryFreeNum  = PopCount(entryFreeVec)
  entryFreeID       := PriorityEncoder(entryFreeVec)

  /*
   * Receive req2Node(Snoop)
   */
  val reqVal        = io.chi.txreq.valid
  val snpVal        = io.req2Node.valid; assert(Mux(snpVal, io.req2Node.bits.isSnp, true.B))
  val respVal       = io.resp2Node.valid
  // count NID
  val addrMatchVec  = entrys.map(_.indexMes.addrNoOff === indexSaveInIntf.addrNoOff)
  val reqOrSnpVec   = entrys.map { case p => p.chiMes.isReq | p.chiMes.isSnp }
  val taskMatchVec  = entryFreeVec.zip(addrMatchVec.zip(reqOrSnpVec)).map{ case(a, (b, c)) => !a & b & c }
  //                                  | RESP                                  | SNP                                     | REQ
  taskNID                        := Mux(respVal, 0.U,                          Mux(snpVal, 0.U,                          PopCount(taskMatchVec)))
  indexSaveInIntf.addr           := Mux(respVal, io.resp2Node.bits.addr,       Mux(snpVal, io.req2Node.bits.addr,        io.chi.txreq.bits.Addr))
  indexSaveInIntf.from           := Mux(respVal, io.resp2Node.bits.from,       Mux(snpVal, io.req2Node.bits.from,        DontCare))
  indexSaveInIntf.mshrWay        := Mux(respVal, DontCare,                     Mux(snpVal, io.req2Node.bits.mshrWay,     DontCare))
  indexSaveInIntf.dbid           := Mux(respVal, io.resp2Node.bits.dbid,       Mux(snpVal, 0.U,                          0.U))
  taskSaveInIntf.opcode          := Mux(respVal, io.resp2Node.bits.opcode,     Mux(snpVal, io.req2Node.bits.opcode,      io.chi.txreq.bits.Opcode))
  taskSaveInIntf.tgtID           := Mux(respVal, io.resp2Node.bits.tgtID,      Mux(snpVal, io.req2Node.bits.tgtID,       DontCare))
  taskSaveInIntf.txnID           := Mux(respVal, io.resp2Node.bits.txnID,      Mux(snpVal, io.req2Node.bits.txnID,       io.chi.txreq.bits.TxnID))
  taskSaveInIntf.srcID           := Mux(respVal, io.resp2Node.bits.srcID,      Mux(snpVal, io.req2Node.bits.srcID,       io.chi.txreq.bits.SrcID))
  taskSaveInIntf.retToSrc        := Mux(respVal, DontCare,                     Mux(snpVal, io.req2Node.bits.retToSrc,    DontCare))
  taskSaveInIntf.doNotGoToSD     := Mux(respVal, DontCare,                     Mux(snpVal, io.req2Node.bits.doNotGoToSD, DontCare))
  taskSaveInIntf.channel         := Mux(respVal, io.resp2Node.bits.channel,    Mux(snpVal, CHIChannel.SNP,               CHIChannel.REQ))
  taskSaveInIntf.resp            := Mux(respVal, io.resp2Node.bits.resp,       Mux(snpVal, 0.U,                          0.U))
  taskSaveInIntf.expCompAck      := Mux(respVal, io.resp2Node.bits.expCompAck, Mux(snpVal, false.B,                      io.chi.txreq.bits.ExpCompAck))

  /*
   * Set Ready Value
   */
  io.chi.txreq.ready    := entryFreeNum >= param.nrEvictEntry.U & !io.req2Node.valid & !io.resp2Node.valid
  io.req2Node.ready     := entryFreeNum > 0.U & !io.resp2Node.valid
  io.resp2Node.ready    := entryFreeNum > 0.U
  io.reqAck2Node.ready  := true.B


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- Select Entry send Req to Slice --------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Select one Entry
   */
  val reqBeSendVec  = entrys.map(_.isReqBeSend)
  entrySendReqID    := RREncoder(reqBeSendVec)

  /*
   * Send Req To Node
   */
  io.req2Slice.valid            := reqBeSendVec.reduce(_ | _)
  io.req2Slice.bits.channel     := entrys(entrySendReqID).chiMes.channel
  io.req2Slice.bits.opcode      := entrys(entrySendReqID).chiMes.opcode
  io.req2Slice.bits.addr        := entrys(entrySendReqID).indexMes.addr
  io.req2Slice.bits.srcID       := entrys(entrySendReqID).chiMes.srcID
  io.req2Slice.bits.txnID       := entrys(entrySendReqID).chiMes.txnID
  io.req2Slice.bits.expCompAck  := entrys(entrySendReqID).chiMes.expCompAck
  // IdMap
  io.req2Slice.bits.to.IncoId   := entrys(entrySendReqID).indexMes.mBank
  io.req2Slice.bits.from.IncoId := rnSlvId.U
  io.req2Slice.bits.pcuId       := entrySendReqID
  io.req2Slice.bits.dbid        := entrys(entrySendReqID).indexMes.dbid
  // Use in RnMaster
  io.req2Slice.bits.retToSrc    := false.B
  io.req2Slice.bits.doNotGoToSD := false.B



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- Select Entry send Resp to Node --------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Select one Entry to Send RxDat
   */
  val datBeSendVec  = entrys.map { case p => p.isDatBeSend & p.indexMes.dbid === io.dbSigs.dataFDB.bits.dbid }
  val datSelId      = PriorityEncoder(datBeSendVec)

  io.chi.rxdat.valid        := datBeSendVec.reduce(_ | _)
  io.chi.rxdat.bits         := DontCare
  io.chi.rxdat.bits.Opcode  := entrys(datSelId).chiMes.opcode
  io.chi.rxdat.bits.TgtID   := entrys(datSelId).chiMes.srcID
  io.chi.rxdat.bits.SrcID   := io.hnfID
  io.chi.rxdat.bits.TxnID   := entrys(datSelId).chiMes.txnID
  io.chi.rxdat.bits.HomeNID := io.hnfID
  io.chi.rxdat.bits.DBID    := datSelId
  io.chi.rxdat.bits.Resp    := entrys(datSelId).chiMes.resp
  io.chi.rxdat.bits.DataID  := io.dbSigs.dataFDB.bits.dataID
  io.chi.rxdat.bits.Data    := io.dbSigs.dataFDB.bits.data
  io.chi.rxdat.bits.BE      := Fill(io.chi.rxdat.bits.BE.getWidth, 1.U(1.W))

  io.dbSigs.dataFDB.ready   := io.chi.rxdat.ready


  /*
   * Select one Entry to Send RxRsp
   */
  val rspBeSendVec          = entrys.map { case p => p.isRspBeSend}
  val rspSelId              = PriorityEncoder(rspBeSendVec)

  io.chi.rxrsp.valid        := rspBeSendVec.reduce(_ | _)
  io.chi.rxrsp.bits         := DontCare
  io.chi.rxrsp.bits.Opcode  := Mux(entrys(rspSelId).chiMes.isRsp, entrys(rspSelId).chiMes.opcode, CompDBIDResp)
  io.chi.rxrsp.bits.TgtID   := Mux(entrys(rspSelId).chiMes.isRsp, entrys(rspSelId).chiMes.srcID,  entrys(rspSelId).chiMes.srcID)
  io.chi.rxrsp.bits.SrcID   := Mux(entrys(rspSelId).chiMes.isRsp, io.hnfID,                       io.hnfID)
  io.chi.rxrsp.bits.TxnID   := Mux(entrys(rspSelId).chiMes.isRsp, entrys(rspSelId).chiMes.txnID,  entrys(rspSelId).chiMes.txnID)
  io.chi.rxrsp.bits.DBID    := Mux(entrys(rspSelId).chiMes.isRsp, rspSelId,                       rspSelId)
  io.chi.rxrsp.bits.Resp    := Mux(entrys(rspSelId).chiMes.isRsp, entrys(rspSelId).chiMes.resp,   DontCare)



// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------------------- Receive Rsp Or Dat From Node ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Entry ID
   */
  entryRecChiRspID                := io.chi.txrsp.bits.TxnID(param.entryIdBits - 1, 0)
  entryRecChiDatID                := io.chi.txdat.bits.TxnID(param.entryIdBits - 1, 0)

  /*
   * Send Data To DataBuffer
   */
  io.dbSigs.dataTDB.valid         := io.chi.txdat.valid
  io.dbSigs.dataTDB.bits.dbid     := entrys(entryRecChiDatID).indexMes.dbid
  io.dbSigs.dataTDB.bits.data     := io.chi.txdat.bits.Data
  io.dbSigs.dataTDB.bits.dataID   := io.chi.txdat.bits.DataID


  /*
   * Set ready value
   */
  io.chi.txrsp.ready              := true.B
  io.chi.txdat.ready              := io.dbSigs.dataTDB.ready


// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- Get DBID From DataBuffer and Wait DataBuffer Resp ---------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Send Get DBID Req To From DataBuffer
   */
  val entryGetDBIDVec         = entrys.map(_.isGetDBID)
  entryGetDBID                := PriorityEncoder(entryGetDBIDVec)

  /*
   * Set DataBuffer Req Value
   */
  io.dbSigs.wReq.valid            := entryGetDBIDVec.reduce(_ | _)
  io.dbSigs.wReq.bits.from.IncoId := rnSlvId.U
  io.dbSigs.wReq.bits.pcuId     := entryGetDBID

  /*
   * Receive DBID From DataBuffer
   */
  entryRecDBID                    := io.dbSigs.wResp.bits.pcuId
  io.dbSigs.wResp.ready           := true.B



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- Select Entry send Snp to Node ---------------------------------------- //
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
  val snpShouldSendVec  = entrys(entrySendSnpID).chiMes.snpMetaVec
  val snpBeSendVec      = snpShouldSendVec ^ snpAlreadySendVecReg
  val snpTgtID          = getNodeIDByMetaId(PriorityEncoder(snpBeSendVec), 0)
  snpIsLast             := PopCount(snpBeSendVec.asBools) === 1.U; dontTouch(snpIsLast)
  snpAlreadySendVecReg  := Mux(io.chi.rxsnp.fire, Mux(snpIsLast, 0.U, snpAlreadySendVecReg | UIntToOH(getMetaIdByNodeID(snpTgtID))), snpAlreadySendVecReg)

  /*
   * Send Snp to Node
   */
  io.chi.rxsnp.valid          := entrySendSnpVec.reduce(_ | _) | entrySendSnpIngVec.reduce(_ | _)
  io.chi.rxsnp.bits           := DontCare
  io.chi.rxsnp.bits.Addr      := entrys(entrySendSnpID).indexMes.addr(addressBits - 1, 3)
  io.chi.rxsnp.bits.Opcode    := entrys(entrySendSnpID).chiMes.opcode
  io.chi.rxsnp.bits.TgtID     := snpTgtID
  io.chi.rxsnp.bits.SrcID     := io.hnfID
  io.chi.rxsnp.bits.TxnID     := entrySendSnpID
  io.chi.rxsnp.bits.FwdNID    := entrys(entrySendSnpID).chiMes.srcID
  io.chi.rxsnp.bits.FwdTxnID  := entrys(entrySendSnpID).chiMes.txnID
  io.chi.rxsnp.bits.RetToSrc  := entrys(entrySendSnpID).chiMes.retToSrc & snpAlreadySendVecReg === 0.U
  io.chi.rxsnp.bits.DoNotGoToSD := entrys(entrySendSnpID).chiMes.doNotGoToSD


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------ Select Entry send Resp to Slice --------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Select one Entry to Send RxDat
   */
  val respBeSendVec                 = entrys.map { case p => p.state === RSState.Resp2Slice }
  entryResp2SliceID                 := PriorityEncoder(respBeSendVec)

  io.resp2Slice.valid               := respBeSendVec.reduce(_ | _)
  io.resp2Slice.bits.from.IncoId    := rnSlvId.U
  io.resp2Slice.bits.to             := entrys(entryResp2SliceID).indexMes.from
  io.resp2Slice.bits.mshrSet        := entrys(entryResp2SliceID).indexMes.mSet
  io.resp2Slice.bits.mshrWay        := entrys(entryResp2SliceID).indexMes.mshrWay
  io.resp2Slice.bits.dbid           := entrys(entryResp2SliceID).indexMes.dbid
  io.resp2Slice.bits.isSnpResp      := true.B
  io.resp2Slice.bits.isReqResp      := false.B
  io.resp2Slice.bits.hasData        := entrys(entryResp2SliceID).hasData
  io.resp2Slice.bits.resp           := entrys(entryResp2SliceID).chiMes.resp
  io.resp2Slice.bits.fwdState.valid := CHIOp.SNP.isSnpXFwd(entrys(entryResp2SliceID).chiMes.opcode)
  io.resp2Slice.bits.fwdState.bits  := entrys(entryResp2SliceID).chiMes.fwdState






// ---------------------------  Assertion  -------------------------------- //
  val cntReg = RegInit(VecInit(Seq.fill(param.nrEntry) { 0.U(64.W) }))
  cntReg.zip(entrys).foreach { case(c, p) => c := Mux(p.isFree, 0.U, c + 1.U) }
  cntReg.zipWithIndex.foreach { case(c, i) => assert(c < TIMEOUT_RSINTF.U, "RNSLV ENTRY[0x%x] ADDR[0x%x] CHANNEL[%x] OP[0x%x] TIMEOUT", i.U, entrys(i).indexMes.addr, entrys(i).chiMes.channel, entrys(i).chiMes.opcode) }
}