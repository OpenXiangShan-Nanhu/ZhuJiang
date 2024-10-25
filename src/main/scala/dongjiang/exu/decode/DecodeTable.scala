package dongjiang.pcu.exu.decode

import dongjiang._
import dongjiang.pcu._
import dongjiang.pcu.exu.decode.InstBundle
import dongjiang.pcu.exu.decode.RespType._
import dongjiang.pcu.ChipType
import dongjiang.pcu.exu.decode.Inst._
import dongjiang.pcu.exu.decode.Code._
import dongjiang.chi._
import dongjiang.chi.CHIChannel._
import dongjiang.chi.CHIOp.REQ._
import dongjiang.chi.CHIOp.RSP._
import dongjiang.chi.CHIOp.DAT._
import dongjiang.chi.CHIOp.SNP._
import dongjiang.chi.ChiState._
import chisel3._
import chisel3.util._

/*
 * TODO: Update It
 * {Read / Dataless / Atomic / CMO} Request Processing Flow 0 (longest path):
 *                                                                  GetChiTxReq -> RecordInReqBuf -> IssueReq -> [Retry] -> [IssueReq] -> AllocMSHR -> ReadDir -> WaitDirResp -> Decode0 -> MpUpdateMSHR -> Process([Snoop] / [ReadDown] / [ReqRetry])
 *                                                                  |-------------------------- RnSlave -----------------------------|    |-- S0 --|  |-- S1 --|  |--- S2 ---|  |------------------------------- S3 ---------------------------------|
 *                                                                              -> [SnpCtlUpateMSHR] / [ReadCtlUpdateMSHR] -> [IssueResp] -> [Retry] -> [IssueResp] -> [ReadDir] -> [WaitDirResp] -> [Decode1] -> Process([SnpHlp] / [Replace] / [ReqRetry])
 *                                                                                 |----------------------------------- MSHRCtl ----------------------------------|    |-- S1 --|   |---------- S2 ----------|   |-------------------- S3 ----------------|
 *                                                                              -> Commit([ReadDB] / [WSDir] / [WCDir] / [ReadDS] / [WriteDS] / [Atomic]) -> MpUpdateMSHR
 *                                                                                 |----------------------------------------- S4 ---------------------------------------|
 *                                                                              -> UpdateReqBuf -> SendChiRxRsp/Dat -> [GetAck] -> RnUpdateMSHR -> MSHRRelease
 *                                                                                 |---------------------------- RnSlave ---------------------|   |- MSHRCtl -|
 *
 *
 * {Read / Dataless / Atomic / CMO} Request Processing Flow 1 (shortest path):
 *                                                                  GetChiTxReq -> RecordInReqBuf -> IssueReq -> AllocMSHR -> ReadDir -> WaitDirResp -> Decode0 -> MpUpdateMSHR -> Commit([WSDir] / [WCDir] / [ReadDS] / [Atomic]) -> UpdateReqBuf -> SendChiRxRsp/Dat -> RnUpdateMSHR -> MSHRRelease
 *                                                                  |-------------- RnSlave -----------------|   |-- S0 --|  |-- S1 --|  |--- S2 ---|   |-------- S3 ---------|    |-------------------- S4 ---------------------|    |------------------------- RnSlave -------------|   |- MSHRCtl -|
 *
 *
 * {Write} Request Processing Flow 0 (longest path):
 *                                                                  GetChiTxReq -> RecordInReqBuf -> GetDBID -> SendDBIDResp -> GetChiTxDat-> WBDataToDB-> IssueReq -> [Retry] -> [IssueReq] -> AllocMSHR -> ReadDir -> WaitDirResp -> Decode0 -> MpUpdateMSHR -> Process([Replace])
 *                                                                  |------------------------------------------------------------------- RnSlave ------------------------------------------|    |-- S0 --|  |-- S1 --|  |--- S2 ---|  |-------------------- S3 --------------------|
 *                                                                              -> Commit([WSDir] / [WCDir] / [ReadDS] / [WriteDS]) -> MpUpdateMSHR -> MSHRRelease
 *                                                                                 |------------------------------ S4 -----------------------------|  |- MSHRCtl -|
 *
 *
 * {SnpHlp} Request Processing Flow 0 (longest path):
 *                                                                  AllocMSHR -> [SnpCtlUpateMSHR] -> MpUpdateMSHR
 *                                                                  |-- S0 --|   |------------ MSHRCtl ----------|
 *
 * {Replace} Request Processing Flow 0 (longest path):
 *                                                                  AllocMSHR -> [ReadCtlUpdateMSHR] -> MpUpdateMSHR
 *                                                                  |-- S0 --|   |------------- MSHRCtl -----------|
 *
 *
 * decoder table: [opcode, srcState, srcHit, othState, othHit, hnState, hnHit] -> [Process Operations] + [srcNS, othNS, hnNS]
 *
 */



/*
 * When it need Snoop or ReadDown, it need to decode twice, and the result is based on the second decode
 * It cant be Commit and Snoop / ReadDown at the same time
 */



object LocalReadDecode {
  def readNotSharedDirty: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ --------------------------------------------------------------//
    LocalReqInst(ReadNotSharedDirty, I, I,   I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadNotSharedDirty, I, UC,  I) -> (Snoop    | SnpOp(SnpNotSharedDirty) | retToSrc),
    LocalReqInst(ReadNotSharedDirty, I, UD,  I) -> (Snoop    | SnpOp(SnpNotSharedDirty) | retToSrc),
    LocalReqInst(ReadNotSharedDirty, I, SC,  I) -> (Snoop    | SnpOp(SnpNotSharedDirty) | retToSrc),
    LocalReqInst(ReadNotSharedDirty, I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.UC)),
    LocalReqInst(ReadNotSharedDirty, I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadNotSharedDirty, I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, SC, SC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, SC, SD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),

    // ----------------------------------------------------------- LOCAL RESP ------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I,  I, RD,  HasData, rd = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp, HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp, HasData, snp = ChiResp.I_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp, HasData, snp = ChiResp.SC)     -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(SC)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp, HasData, snp = ChiResp.SC_PD)  -> (Commit | RDB2Src           | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I UD  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UD,  I, Snp, HasData, snp = ChiResp.I_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UD,  I, Snp, HasData, snp = ChiResp.SC_PD)  -> (Commit | RDB2Src           | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC,  I, Snp, HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC,  I, Snp, HasData, snp = ChiResp.SC)     -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(SC)),
    //  I  I UC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UC, RD,  HasData, rd = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UD, RD,  HasData, rd = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SC, RD,  HasData, rd = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SC, RD,  HasData, rd = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(SC)),
    //  I  I SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SD, RD,  HasData, rd = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SD, RD,  HasData, rd = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC)),
  )


  def readUnique: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(ReadUnique,  I, I,   I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadUnique,  I, UC,  I) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),
    LocalReqInst(ReadUnique,  I, UD,  I) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),
    LocalReqInst(ReadUnique,  I, SC,  I) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),
    LocalReqInst(ReadUnique,  I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique,  I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadUnique,  I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique,  I, SC, SC) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),
    LocalReqInst(ReadUnique,  I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadUnique,  I, SC, SD) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),

    LocalReqInst(ReadUnique, SC,  I,  I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadUnique, SC, SC,  I) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),
    LocalReqInst(ReadUnique, SC, SC, SC) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),
    LocalReqInst(ReadUnique, SC, SC, SD) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),
    LocalReqInst(ReadUnique, SC,  I, SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique, SC,  I, SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC_PD)),


    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadUnique,  I,  I,  I, RD,     HasData, rd = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, Snp,    HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, Snp,    HasData, snp = ChiResp.I_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I UD  I
    LocalRespInst(REQ, ReadUnique,  I, UD,  I, Snp,    HasData, snp = ChiResp.I_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC  I
    LocalRespInst(REQ, ReadUnique,  I, SC,  I, Snp,    HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UC
    LocalRespInst(REQ, ReadUnique,  I,  I, UC, RD,     HasData, rd = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadUnique,  I,  I, UD, RD,     HasData, rd = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadUnique,  I,  I, SC, RD,     HasData, rd = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadUnique,  I, SC, SC, Snp,    HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I SD
    LocalRespInst(REQ, ReadUnique,  I,  I, SD, RD,     HasData, rd = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadUnique,  I, SC, SD, Snp,    HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),

    // SC  I  I
    LocalRespInst(REQ, ReadUnique, SC,  I,  I, RD,     HasData, rd = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC  I
    LocalRespInst(REQ, ReadUnique, SC, SC,  I, Snp,    HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SC
    LocalRespInst(REQ, ReadUnique, SC, SC, SC, Snp,    HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SD
    LocalRespInst(REQ, ReadUnique, SC, SC, SD, Snp,    HasData, snp = ChiResp.I)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    // SC  I SC
    LocalRespInst(REQ, ReadUnique, SC,  I, SC, RD,     HasData, rd  = ChiResp.UC)     -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC  I SD
    LocalRespInst(REQ, ReadUnique, SC,  I, SD, RD,     HasData, rd  = ChiResp.UD_PD)  -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
  )


  def table: Seq[(UInt, UInt)] = readNotSharedDirty ++ readUnique
}


object LoaclDatalessDecode {
  def evict: Seq[(UInt, UInt)] = Seq(
    LocalReqInst(Evict, I,  I,  I) -> NothingTODO,
    LocalReqInst(Evict, I, UC,  I) -> NothingTODO,
    LocalReqInst(Evict, I, UD,  I) -> NothingTODO,
    LocalReqInst(Evict, I, SC,  I) -> NothingTODO,
    LocalReqInst(Evict, I, SC, SC) -> NothingTODO,
    LocalReqInst(Evict, I, SC, SD) -> NothingTODO,
    LocalReqInst(Evict, I,  I, SC) -> NothingTODO,
    LocalReqInst(Evict, I,  I, SD) -> NothingTODO,
    LocalReqInst(Evict, I,  I, UC) -> NothingTODO,
    LocalReqInst(Evict, I,  I, UD) -> NothingTODO,

    LocalReqInst(Evict, UC,  I,  I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | HnState(I)  | SrcState(I) | OthState(I)),
    LocalReqInst(Evict, SC,  I,  I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | HnState(I)  | SrcState(I) | OthState(I)),
    LocalReqInst(Evict, SC, SC,  I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | HnState(I)  | SrcState(I) | OthState(SC)),
    LocalReqInst(Evict, SC, SC, SC) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | HnState(SC) | SrcState(I) | OthState(SC)),
    LocalReqInst(Evict, SC, SC, SD) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | HnState(SD) | SrcState(I) | OthState(SC)),
    LocalReqInst(Evict, SC,  I, SC) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | HnState(UC) | SrcState(I) | OthState(I)),
    LocalReqInst(Evict, SC,  I, SD) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | HnState(UD) | SrcState(I) | OthState(I)),
  )

  def makeUnique: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(MakeUnique,  I,  I,  I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique,  I, SC,  I) -> (Snoop  | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I, SC, SC) -> (Snoop  | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I, SC, SD) -> (Snoop  | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I, UC,  I) -> (Snoop  | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I, UD,  I) -> (Snoop  | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I,  I, SC) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique,  I,  I, SD) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique,  I,  I, UC) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique,  I,  I, UD) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique, SC,  I,  I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique, SC, SC,  I) -> (Snoop  | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique, SC, SC, SC) -> (Snoop  | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique, SC, SC, SD) -> (Snoop  | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique, SC,  I, SC) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique, SC,  I, SD) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    //  I SC  I
    LocalRespInst(REQ, MakeUnique,  I, SC,  I, Snp, snp = ChiResp.I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    //  I SC SC
    LocalRespInst(REQ, MakeUnique,  I, SC, SC, Snp, snp = ChiResp.I) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    //  I SC SD
    LocalRespInst(REQ, MakeUnique,  I, SC, SD, Snp, snp = ChiResp.I) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    //  I UC  I
    LocalRespInst(REQ, MakeUnique,  I, UC,  I, Snp, snp = ChiResp.I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    //  I UD  I
    LocalRespInst(REQ, MakeUnique,  I, UD,  I, Snp, snp = ChiResp.I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    // SC SC  I
    LocalRespInst(REQ, MakeUnique, SC, SC,  I, Snp, snp = ChiResp.I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    // SC SC SC
    LocalRespInst(REQ, MakeUnique, SC, SC, SC, Snp, snp = ChiResp.I) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    // SC SC SD
    LocalRespInst(REQ, MakeUnique, SC, SC, SD, Snp, snp = ChiResp.I) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),



  )

  def table: Seq[(UInt, UInt)] = evict ++ makeUnique
}


object LoaclWriteDecode {
  def writeBackFull: Seq[(UInt, UInt)] = Seq(
    LocalReqInst(WriteBackFull,  I,  I,  I, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I, UC,  I, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I, UD,  I, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I, SC,  I, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I, SC, SC, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I, SC, SD, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I,  I, SC, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I,  I, SD, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I,  I, UC, HasData) -> CleanDB,
    LocalReqInst(WriteBackFull,  I,  I, UD, HasData) -> CleanDB,

    LocalReqInst(WriteBackFull, UD,  I,  I, HasData) -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //temporary fix
    LocalReqInst(WriteBackFull, UC,  I,  I, HasData) -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalReqInst(WriteBackFull, SC,  I,  I, HasData) -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalReqInst(WriteBackFull, SC, SC,  I, HasData) -> (WSFDir | WSDir | SrcState(I) | OthState(SC) | HnState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalReqInst(WriteBackFull, SC, SC, SC, HasData) -> (WSFDir |         SrcState(I) | OthState(SC) | HnState(SC) | CleanDB),
    LocalReqInst(WriteBackFull, SC, SC, SD, HasData) -> (WSFDir |         SrcState(I) | OthState(SC) | HnState(SD) | CleanDB),
    LocalReqInst(WriteBackFull, SC,  I, SC, HasData) -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UC) | CleanDB),
    LocalReqInst(WriteBackFull, SC,  I, SD, HasData) -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | CleanDB),
  )

  def table: Seq[(UInt, UInt)] = writeBackFull
}


object LoaclSnpUniqueEvictDecode {
  def snpUniqueEvict: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL RESP ------------------------------------------------------------//
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I,  I, Snp,  HasData, snp = ChiResp.I)    -> (WSDir | HnState(UC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I, SC, Snp,  HasData, snp = ChiResp.I)    -> (WSDir | HnState(UC) | CleanDB),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I, SD, Snp,  HasData, snp = ChiResp.I)    -> (WSDir | HnState(UD) | CleanDB),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I,  I, Snp,  HasData, snp = ChiResp.I_PD) -> (WSDir | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull))
  )

  def table: Seq[(UInt, UInt)] = snpUniqueEvict
}