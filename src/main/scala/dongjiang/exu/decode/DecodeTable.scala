package dongjiang.pcu.exu.decode

import dongjiang._
import dongjiang.pcu._
import dongjiang.pcu.exu.decode.InstBundle
import dongjiang.pcu.exu.decode.RespType._
import dongjiang.pcu.exu.decode.Inst._
import dongjiang.pcu.exu.decode.Code._
import dongjiang.chi._
import dongjiang.chi.CHIChannel._
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.SnpOpcode._
import zhujiang.chi._
import dongjiang.chi.ChiState._
import chisel3._
import chisel3.util._


/*
 * When it need Snoop or ReadDown, it need to decode twice, and the result is based on the second decode
 * It cant be (Commit / wSDir / wSFDir) and (Snoop / ReadDown / ReadDCU) at the same time
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
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I,  I, RD,  HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I,  I, RD,           sn = ChiResp.UC)      -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp, HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp, HasData, rn = ChiResp.I_PD)    -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp, HasData, rn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(SC)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp, HasData, rn = ChiResp.SC_PD)   -> (Commit | RDB2Src           | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I UD  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UD,  I, Snp, HasData, rn = ChiResp.I_PD)    -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UD,  I, Snp, HasData, rn = ChiResp.SC_PD)   -> (Commit | RDB2Src           | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC,  I, Snp, HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC,  I, Snp, HasData, rn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(SC)),
    //  I  I UC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UC, RD,  HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UC, RD,           sn = ChiResp.UC)      -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UD, RD,  HasData, sn = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UD, RD,           sn = ChiResp.UD_PD)   -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SC, RD,  HasData, sn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SC, RD,           sn = ChiResp.SC)      -> (                             WSFDir |                                                                  HnState(SC) | SrcState(SC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SC, RD,  HasData, sn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(SC)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SC, RD,           sn = ChiResp.SC)      -> (                             WSFDir |                                                                  HnState(SC) | SrcState(SC) | OthState(SC)),
    //  I  I SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SD, RD,  HasData, sn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SD, RD,           sn = ChiResp.SC)      -> (                             WSFDir |                                                                  HnState(SD) | SrcState(SC) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SD, RD,  HasData, sn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SD, RD,           sn = ChiResp.SC)      -> (                             WSFDir |                                                                  HnState(SD) | SrcState(SC) | OthState(SC)),
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
    LocalRespInst(REQ, ReadUnique,  I,  I,  I, RD,     HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I,  I, RD,              sn = ChiResp.UC)      -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, Snp,    HasData, rn = ChiResp.I_PD)    -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I UD  I
    LocalRespInst(REQ, ReadUnique,  I, UD,  I, Snp,    HasData, rn = ChiResp.I_PD)    -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC  I
    LocalRespInst(REQ, ReadUnique,  I, SC,  I, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UC
    LocalRespInst(REQ, ReadUnique,  I,  I, UC, RD,     HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I, UC, RD,              sn = ChiResp.UC)      -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadUnique,  I,  I, UD, RD,     HasData, sn = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I, UD, RD,              sn = ChiResp.UD_PD)   -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadUnique,  I,  I, SC, RD,     HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I, SC, RD,              sn = ChiResp.UC)      -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadUnique,  I, SC, SC, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I SD
    LocalRespInst(REQ, ReadUnique,  I,  I, SD, RD,     HasData, sn = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I, SD, RD,              sn = ChiResp.UD_PD)   -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadUnique,  I, SC, SD, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    // SC  I  I
    LocalRespInst(REQ, ReadUnique, SC,  I,  I, RD,     HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique, SC,  I,  I, RD,              sn = ChiResp.UC)      -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC  I
    LocalRespInst(REQ, ReadUnique, SC, SC,  I, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SC
    LocalRespInst(REQ, ReadUnique, SC, SC, SC, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SD
    LocalRespInst(REQ, ReadUnique, SC, SC, SD, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    // SC  I SC
    LocalRespInst(REQ, ReadUnique, SC,  I, SC, RD,     HasData, sn  = ChiResp.UC)     -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique, SC,  I, SC, RD,              sn  = ChiResp.UC)     -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    // SC  I SD
    LocalRespInst(REQ, ReadUnique, SC,  I, SD, RD,     HasData, sn  = ChiResp.UD_PD)  -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadUnique, SC,  I, SD, RD,              sn  = ChiResp.UD_PD)  -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UD) | OthState(I)),
  )


  def table: Seq[(UInt, UInt)] = readNotSharedDirty ++ readUnique
}



object LocalReadWithDCTDecode {
  def readNotSharedDirty: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ --------------------------------------------------------------//
    LocalReqInst(ReadNotSharedDirty, I, I,   I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadNotSharedDirty, I, UC,  I) -> (Snoop    | SnpOp(SnpNotSharedDirtyFwd) | retToSrc),
    LocalReqInst(ReadNotSharedDirty, I, UD,  I) -> (Snoop    | SnpOp(SnpNotSharedDirtyFwd) | retToSrc),
    LocalReqInst(ReadNotSharedDirty, I, SC,  I) -> (Snoop    | SnpOp(SnpNotSharedDirtyFwd)),
    LocalReqInst(ReadNotSharedDirty, I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.UC)),
    LocalReqInst(ReadNotSharedDirty, I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadNotSharedDirty, I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, SC, SC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, SC, SD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),

    // ----------------------------------------------------------- LOCAL RESP ------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I,  I, RD,      HasData, sn = ChiResp.UC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, SnpFwd,  HasData, rn = ChiResp.I,     fwd = ChiResp.SC)  -> (                   CleanDB | WSFDir |                                                                  HnState(I)  | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, SnpFwd,  HasData, rn = ChiResp.I_PD,  fwd = ChiResp.SC)  -> (                             WSFDir | WSDir |                                                          HnState(SD) | SrcState(SC) | OthState(I)  | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, SnpFwd,  HasData, rn = ChiResp.SC,    fwd = ChiResp.SC)  -> (                   CleanDB | WSFDir |                                                                  HnState(I)  | SrcState(SC) | OthState(SC)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, SnpFwd,  HasData, rn = ChiResp.SC_PD, fwd = ChiResp.SC)  -> (                             WSFDir | WSDir |                                                          HnState(SD) | SrcState(SC) | OthState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I UD  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UD,  I, SnpFwd,  HasData, rn = ChiResp.I_PD,  fwd = ChiResp.SC)  -> (                             WSFDir | WSDir |                                                          HnState(SD) | SrcState(SC) | OthState(I)  | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UD,  I, SnpFwd,  HasData, rn = ChiResp.SC_PD, fwd = ChiResp.SC)  -> (                             WSFDir | WSDir |                                                          HnState(SD) | SrcState(SC) | OthState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC,  I, SnpFwd,          rn = ChiResp.I,     fwd = ChiResp.SC)   -> (                             WSFDir |                                                                  HnState(I)  | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC,  I, SnpFwd,          rn = ChiResp.SC,    fwd = ChiResp.SC)   -> (                             WSFDir |                                                                  HnState(I)  | SrcState(SC) | OthState(SC)),
    //  I  I UC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UC, RD,      HasData, sn = ChiResp.UC)                       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UD, RD,      HasData, sn = ChiResp.UD_PD)                    -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SC, RD,      HasData, sn = ChiResp.SC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SC, RD,      HasData, sn = ChiResp.SC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(SC)),
    //  I  I SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SD, RD,      HasData, sn = ChiResp.SC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SD, RD,      HasData, sn = ChiResp.SC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC)),
  )


  def readUnique: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(ReadUnique,  I, I,   I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadUnique,  I, UC,  I) -> (Snoop    | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique,  I, UD,  I) -> (Snoop    | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique,  I, SC,  I) -> (Snoop    | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique,  I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique,  I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadUnique,  I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique,  I, SC, SC) -> (Snoop    | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique,  I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadUnique,  I, SC, SD) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),

    LocalReqInst(ReadUnique, SC,  I,  I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadUnique, SC, SC,  I) -> (Snoop    | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique, SC, SC, SC) -> (Snoop    | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique, SC, SC, SD) -> (Snoop    | SnpOp(SnpUnique)  | retToSrc),
    LocalReqInst(ReadUnique, SC,  I, SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique, SC,  I, SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC_PD)),


    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadUnique,  I,  I,  I, RD,     HasData, sn = ChiResp.UC)                      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UD_PD)  -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UD) | OthState(I)),
    //  I UD  I
    LocalRespInst(REQ, ReadUnique,  I, UD,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UD_PD)  -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC  I
    LocalRespInst(REQ, ReadUnique,  I, SC,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UC
    LocalRespInst(REQ, ReadUnique,  I,  I, UC, RD,     HasData, sn = ChiResp.UC)                      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadUnique,  I,  I, UD, RD,     HasData, sn = ChiResp.UD_PD)                   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadUnique,  I,  I, SC, RD,     HasData, sn = ChiResp.UC)                      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadUnique,  I, SC, SC, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I SD
    LocalRespInst(REQ, ReadUnique,  I,  I, SD, RD,     HasData, sn = ChiResp.UD_PD)                   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadUnique,  I, SC, SD, Snp,    HasData, rn = ChiResp.I)                       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),

    // SC  I  I
    LocalRespInst(REQ, ReadUnique, SC,  I,  I, RD,     HasData, sn = ChiResp.UC)                      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC  I
    LocalRespInst(REQ, ReadUnique, SC, SC,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SC
    LocalRespInst(REQ, ReadUnique, SC, SC, SC, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SD
    LocalRespInst(REQ, ReadUnique, SC, SC, SD, Snp,    HasData, rn = ChiResp.I)                       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    // SC  I SC
    LocalRespInst(REQ, ReadUnique, SC,  I, SC, RD,     HasData, sn  = ChiResp.UC)                     -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC  I SD
    LocalRespInst(REQ, ReadUnique, SC,  I, SD, RD,     HasData, sn  = ChiResp.UD_PD)                  -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
  )


  def table: Seq[(UInt, UInt)] = readNotSharedDirty ++ readUnique
}



object LoaclDatalessDecode {
  def evict: Seq[(UInt, UInt)] = Seq(
    LocalReqInst(Evict, I,  I,  I) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I, UC,  I) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I, UD,  I) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I, SC,  I) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I, SC, SC) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I, SC, SD) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I,  I, SC) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I,  I, SD) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I,  I, UC) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),
    LocalReqInst(Evict, I,  I, UD) ->  (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) ),

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
    LocalRespInst(REQ, MakeUnique,  I, SC,  I, Snp, rn = ChiResp.I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    //  I SC SC
    LocalRespInst(REQ, MakeUnique,  I, SC, SC, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    //  I SC SD
    LocalRespInst(REQ, MakeUnique,  I, SC, SD, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    //  I UC  I
    LocalRespInst(REQ, MakeUnique,  I, UC,  I, Snp, rn = ChiResp.I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    //  I UD  I
    LocalRespInst(REQ, MakeUnique,  I, UD,  I, Snp, rn = ChiResp.I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    // SC SC  I
    LocalRespInst(REQ, MakeUnique, SC, SC,  I, Snp, rn = ChiResp.I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    // SC SC SC
    LocalRespInst(REQ, MakeUnique, SC, SC, SC, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    // SC SC SD
    LocalRespInst(REQ, MakeUnique, SC, SC, SD, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),



  )

  def table: Seq[(UInt, UInt)] = evict ++ makeUnique
}


object LoaclWriteDecode {
  def writeBackFull: Seq[(UInt, UInt)] = Seq(
    LocalRespInst(REQ, WriteBackFull,  I,  I,  I, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, UC,  I, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, UD,  I, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, SC,  I, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, SC, SC, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, SC, SD, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I,  I, SC, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I,  I, SD, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I,  I, UC, WB, HasData, rn = ChiResp.I)     -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I,  I, UD, WB, HasData, rn = ChiResp.I)     -> CleanDB,

    LocalRespInst(REQ, WriteBackFull, UD,  I,  I, WB, HasData, rn = ChiResp.UD_PD) -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //temporary fix
    LocalRespInst(REQ, WriteBackFull, UC,  I,  I, WB, HasData, rn = ChiResp.UC)    -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, WriteBackFull, UC,  I,  I, WB, HasData, rn = ChiResp.UD_PD) -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, WriteBackFull, SC,  I,  I, WB, HasData, rn = ChiResp.SC)    -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, WriteBackFull, SC, SC,  I, WB, HasData, rn = ChiResp.SC)    -> (WSFDir | WSDir | SrcState(I) | OthState(SC) | HnState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, WriteBackFull, SC, SC, SC, WB, HasData, rn = ChiResp.SC)    -> (WSFDir |         SrcState(I) | OthState(SC) | HnState(SC) | CleanDB),
    LocalRespInst(REQ, WriteBackFull, SC, SC, SD, WB, HasData, rn = ChiResp.SC)    -> (WSFDir |         SrcState(I) | OthState(SC) | HnState(SD) | CleanDB),
    LocalRespInst(REQ, WriteBackFull, SC,  I, SC, WB, HasData, rn = ChiResp.SC)    -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UC) | CleanDB),
    LocalRespInst(REQ, WriteBackFull, SC,  I, SD, WB, HasData, rn = ChiResp.SC)    -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | CleanDB),
  )

  def table: Seq[(UInt, UInt)] = writeBackFull
}


object LoaclSnpUniqueEvictDecode {
  def snpUniqueEvict: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL RESP ------------------------------------------------------------//
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I,  I, Snp,  HasData, rn = ChiResp.I)    -> (WSDir | HnState(UC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I, SC, Snp,  HasData, rn = ChiResp.I)    -> (WSDir | HnState(UC) | CleanDB),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I, SD, Snp,  HasData, rn = ChiResp.I)    -> (WSDir | HnState(UD) | CleanDB),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I,  I, Snp,  HasData, rn = ChiResp.I_PD) -> (WSDir | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull))
  )

  def table: Seq[(UInt, UInt)] = snpUniqueEvict
}