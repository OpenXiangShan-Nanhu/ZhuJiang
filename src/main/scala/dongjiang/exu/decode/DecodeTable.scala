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
    LocalReqInst(ReadNotSharedDirty, I, UC,  I) -> (SnpOth   | SnpOp(SnpNotSharedDirty) | RetToSrc),
    LocalReqInst(ReadNotSharedDirty, I, UD,  I) -> (SnpOth   | SnpOp(SnpNotSharedDirty) | RetToSrc),
    LocalReqInst(ReadNotSharedDirty, I, SC,  I) -> (SnpOth   | SnpOp(SnpNotSharedDirty) | RetToSrc),
    LocalReqInst(ReadNotSharedDirty, I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.UC)),
    LocalReqInst(ReadNotSharedDirty, I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadNotSharedDirty, I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, SC, SC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, SC, SD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),

    // ----------------------------------------------------------- LOCAL RESP ------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I,  I, Read, HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I,  I, Read,          sn = ChiResp.UC)      -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp,  HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp,  HasData, rn = ChiResp.I_PD)    -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp,  HasData, rn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(SC)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UC,  I, Snp,  HasData, rn = ChiResp.SC_PD)   -> (Commit | RDB2Src           | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I UD  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UD,  I, Snp,  HasData, rn = ChiResp.I_PD)    -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, UD,  I, Snp,  HasData, rn = ChiResp.SC_PD)   -> (Commit | RDB2Src           | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC,  I, Snp,  HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC,  I, Snp,  HasData, rn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(I)  | SrcState(SC) | OthState(SC)),
    //  I  I UC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UC, Read, HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UC, Read,          sn = ChiResp.UC)      -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UD, Read, HasData, sn = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UD, Read,          sn = ChiResp.UD_PD)   -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SC, Read, HasData, sn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SC, Read,          sn = ChiResp.SC)      -> (                             WSFDir |                                                                  HnState(SC) | SrcState(SC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SC, Read, HasData, sn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(SC)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SC, Read,          sn = ChiResp.SC)      -> (                             WSFDir |                                                                  HnState(SC) | SrcState(SC) | OthState(SC)),
    //  I  I SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SD, Read, HasData, sn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(I)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SD, Read,          sn = ChiResp.SC)      -> (                             WSFDir |                                                                  HnState(SD) | SrcState(SC) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SD, Read, HasData, sn = ChiResp.SC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC)),
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SD, Read,          sn = ChiResp.SC)      -> (                             WSFDir |                                                                  HnState(SD) | SrcState(SC) | OthState(SC)),
  )


  def readUnique: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(ReadUnique,  I, I,   I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadUnique,  I, UC,  I) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),
    LocalReqInst(ReadUnique,  I, UD,  I) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),
    LocalReqInst(ReadUnique,  I, SC,  I) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),
    LocalReqInst(ReadUnique,  I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique,  I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadUnique,  I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique,  I, SC, SC) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),
    LocalReqInst(ReadUnique,  I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadUnique,  I, SC, SD) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),

    LocalReqInst(ReadUnique, SC,  I,  I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadUnique, SC, SC,  I) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),
    LocalReqInst(ReadUnique, SC, SC, SC) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),
    LocalReqInst(ReadUnique, SC, SC, SD) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),
    LocalReqInst(ReadUnique, SC,  I, SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique, SC,  I, SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC_PD)),


    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadUnique,  I,  I,  I, Read,   HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I,  I, Read,            sn = ChiResp.UC)      -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, Snp,    HasData, rn = ChiResp.I_PD)    -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I UD  I
    LocalRespInst(REQ, ReadUnique,  I, UD,  I, Snp,    HasData, rn = ChiResp.I_PD)    -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC  I
    LocalRespInst(REQ, ReadUnique,  I, SC,  I, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UC
    LocalRespInst(REQ, ReadUnique,  I,  I, UC, Read,   HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I, UC, Read,            sn = ChiResp.UC)      -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadUnique,  I,  I, UD, Read,   HasData, sn = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I, UD, Read,            sn = ChiResp.UD_PD)   -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadUnique,  I,  I, SC, Read,   HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I, SC, Read,            sn = ChiResp.UC)      -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadUnique,  I, SC, SC, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I SD
    LocalRespInst(REQ, ReadUnique,  I,  I, SD, Read,   HasData, sn = ChiResp.UD_PD)   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I,  I, SD, Read,            sn = ChiResp.UD_PD)   -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadUnique,  I, SC, SD, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    // SC  I  I
    LocalRespInst(REQ, ReadUnique, SC,  I,  I, Read,   HasData, sn = ChiResp.UC)      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique, SC,  I,  I, Read,            sn = ChiResp.UC)      -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC  I
    LocalRespInst(REQ, ReadUnique, SC, SC,  I, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SC
    LocalRespInst(REQ, ReadUnique, SC, SC, SC, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SD
    LocalRespInst(REQ, ReadUnique, SC, SC, SD, Snp,    HasData, rn = ChiResp.I)       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    // SC  I SC
    LocalRespInst(REQ, ReadUnique, SC,  I, SC, Read,   HasData, sn  = ChiResp.UC)     -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique, SC,  I, SC, Read,            sn  = ChiResp.UC)     -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    // SC  I SD
    LocalRespInst(REQ, ReadUnique, SC,  I, SD, Read,   HasData, sn  = ChiResp.UD_PD)  -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    LocalRespInst(REQ, ReadUnique, SC,  I, SD, Read,            sn  = ChiResp.UD_PD)  -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UD) | OthState(I)),
  )


  def readOnce: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(ReadOnce,  I, I,   I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadOnce,  I, UC,  I) -> (SnpOne   | SnpOp(SnpOnce)    | RetToSrc),
    LocalReqInst(ReadOnce,  I, UD,  I) -> (SnpOne   | SnpOp(SnpOnce)    | RetToSrc),
    LocalReqInst(ReadOnce,  I, SC,  I) -> (SnpOne   | SnpOp(SnpOnce)    | RetToSrc),
    LocalReqInst(ReadOnce,  I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.I)),
    LocalReqInst(ReadOnce,  I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.I)),
    LocalReqInst(ReadOnce,  I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.I)),
    LocalReqInst(ReadOnce,  I, SC, SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.I)),
    LocalReqInst(ReadOnce,  I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.I)),
    LocalReqInst(ReadOnce,  I, SC, SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.I)),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadOnce,  I,  I,  I, Read,   HasData, sn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    LocalRespInst(REQ, ReadOnce,  I,  I,  I, Read,            sn = ChiResp.I)     -> NothingTODO,
    //  I UC  I
    LocalRespInst(REQ, ReadOnce,  I, UC,  I, Snp,    HasData, rn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    LocalRespInst(REQ, ReadOnce,  I, UC,  I, Snp,    HasData, rn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB | WSFDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)  | HnState(I)  | SrcState(I) | OthState(UD)),
    //  I UD  I
    LocalRespInst(REQ, ReadOnce,  I, UD,  I, Snp,    HasData, rn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    //  I SC  I
    LocalRespInst(REQ, ReadOnce,  I, SC,  I, Snp,    HasData, rn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    //  I  I UC
    LocalRespInst(REQ, ReadOnce,  I,  I, UC, Read,   HasData, sn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    LocalRespInst(REQ, ReadOnce,  I,  I, UC, Read,            sn = ChiResp.I)     -> NothingTODO,
    //  I  I UD
    LocalRespInst(REQ, ReadOnce,  I,  I, UD, Read,   HasData, sn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    LocalRespInst(REQ, ReadOnce,  I,  I, UD, Read,            sn = ChiResp.I)     -> NothingTODO,
    //  I  I SC
    LocalRespInst(REQ, ReadOnce,  I,  I, SC, Read,   HasData, sn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    LocalRespInst(REQ, ReadOnce,  I,  I, SC, Read,            sn = ChiResp.I)     -> NothingTODO,
    //  I SC SC
    LocalRespInst(REQ, ReadOnce,  I, SC, SC, Read,   HasData, sn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    LocalRespInst(REQ, ReadOnce,  I, SC, SC, Read,            sn = ChiResp.I)     -> NothingTODO,
    //  I  I SD
    LocalRespInst(REQ, ReadOnce,  I,  I, SD, Read,   HasData, sn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    LocalRespInst(REQ, ReadOnce,  I,  I, SD, Read,            sn = ChiResp.I)     -> NothingTODO,
    //  I SC SD
    LocalRespInst(REQ, ReadOnce,  I, SC, SD, Read,   HasData, sn = ChiResp.I)     -> (Commit | RDB2Src | CleanDB |          RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I)),
    LocalRespInst(REQ, ReadOnce,  I, SC, SD, Read,            sn = ChiResp.I)     -> NothingTODO,
  )


  def table: Seq[(UInt, UInt)] = readNotSharedDirty ++ readUnique ++ readOnce
}



object LocalReadWithDCTDecode {
  def readNotSharedDirty: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ --------------------------------------------------------------//
    LocalReqInst(ReadNotSharedDirty, I, I,   I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadNotSharedDirty, I, UC,  I) -> (SnpOth   | SnpOp(SnpNotSharedDirtyFwd) | RetToSrc),
    LocalReqInst(ReadNotSharedDirty, I, UD,  I) -> (SnpOth   | SnpOp(SnpNotSharedDirtyFwd) | RetToSrc),
    LocalReqInst(ReadNotSharedDirty, I, SC,  I) -> (SnpOth   | SnpOp(SnpNotSharedDirtyFwd)),
    LocalReqInst(ReadNotSharedDirty, I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.UC)),
    LocalReqInst(ReadNotSharedDirty, I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadNotSharedDirty, I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, SC, SC) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),
    LocalReqInst(ReadNotSharedDirty, I, SC, SD) -> (ReadDCU  | ReadOp(ReadNoSnp)        | Resp(ChiResp.SC)),

    // ----------------------------------------------------------- LOCAL RESP ------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I,  I, Read,    HasData, sn = ChiResp.UC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
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
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UC, Read,    HasData, sn = ChiResp.UC)                       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, UD, Read,    HasData, sn = ChiResp.UD_PD)                    -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SC, Read,    HasData, sn = ChiResp.SC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SC, Read,    HasData, sn = ChiResp.SC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SC) | SrcState(SC) | OthState(SC)),
    //  I  I SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I,  I, SD, Read,    HasData, sn = ChiResp.SC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadNotSharedDirty,  I, SC, SD, Read,    HasData, sn = ChiResp.SC)                       -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.SC)    | HnState(SD) | SrcState(SC) | OthState(SC)),
  )


  def readUnique: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(ReadUnique,  I, I,   I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadUnique,  I, UC,  I) -> (SnpOth   | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique,  I, UD,  I) -> (SnpOth   | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique,  I, SC,  I) -> (SnpOth   | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique,  I, I,  UC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique,  I, I,  UD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadUnique,  I, I,  SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique,  I, SC, SC) -> (SnpOth   | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique,  I, I,  SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UD_PD)),
    LocalReqInst(ReadUnique,  I, SC, SD) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),

    LocalReqInst(ReadUnique, SC,  I,  I) -> (ReadDown | ReadOp(ReadNoSnp)),
    LocalReqInst(ReadUnique, SC, SC,  I) -> (SnpOth   | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique, SC, SC, SC) -> (SnpOth   | SnpOp(SnpUniqueFwd)),
    LocalReqInst(ReadUnique, SC, SC, SD) -> (SnpOth   | SnpOp(SnpUnique)  | RetToSrc),
    LocalReqInst(ReadUnique, SC,  I, SC) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC)),
    LocalReqInst(ReadUnique, SC,  I, SD) -> (ReadDCU  | ReadOp(ReadNoSnp) | Resp(ChiResp.UC_PD)),


    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // TODO: Consider a variation of the SC/SD mapping as UC/SD In Local
    //  I  I  I
    LocalRespInst(REQ, ReadUnique,  I,  I,  I, Read,   HasData, sn = ChiResp.UC)                      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I UC  I
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    LocalRespInst(REQ, ReadUnique,  I, UC,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UD_PD)  -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UD) | OthState(I)),
    //  I UD  I
    LocalRespInst(REQ, ReadUnique,  I, UD,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UD_PD)  -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC  I
    LocalRespInst(REQ, ReadUnique,  I, SC,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir |                                                                  HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UC
    LocalRespInst(REQ, ReadUnique,  I,  I, UC, Read,   HasData, sn = ChiResp.UC)                      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I UD
    LocalRespInst(REQ, ReadUnique,  I,  I, UD, Read,   HasData, sn = ChiResp.UD_PD)                   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I  I SC
    LocalRespInst(REQ, ReadUnique,  I,  I, SC, Read,   HasData, sn = ChiResp.UC)                      -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    //  I SC SC
    LocalRespInst(REQ, ReadUnique,  I, SC, SC, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    //  I  I SD
    LocalRespInst(REQ, ReadUnique,  I,  I, SD, Read,   HasData, sn = ChiResp.UD_PD)                   -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    //  I SC SD
    LocalRespInst(REQ, ReadUnique,  I, SC, SD, Snp,    HasData, rn = ChiResp.I)                       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),

    // SC  I  I
    LocalRespInst(REQ, ReadUnique, SC,  I,  I, Read,   HasData, sn = ChiResp.UC)                      -> (Commit | RDB2Src | CleanDB | WSFDir |         RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC  I
    LocalRespInst(REQ, ReadUnique, SC, SC,  I, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SC
    LocalRespInst(REQ, ReadUnique, SC, SC, SC, SnpFwd,          rn = ChiResp.I, fwd = ChiResp.UC)     -> (                             WSFDir | WSDir |                                                          HnState(I)  | SrcState(UC) | OthState(I)),
    // SC SC SD
    LocalRespInst(REQ, ReadUnique, SC, SC, SD, Snp,    HasData, rn = ChiResp.I)                       -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
    // SC  I SC
    LocalRespInst(REQ, ReadUnique, SC,  I, SC, Read,   HasData, sn  = ChiResp.UC)                     -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UC)    | HnState(I)  | SrcState(UC) | OthState(I)),
    // SC  I SD
    LocalRespInst(REQ, ReadUnique, SC,  I, SD, Read,   HasData, sn  = ChiResp.UD_PD)                  -> (Commit | RDB2Src | CleanDB | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.UD_PD) | HnState(I)  | SrcState(UD) | OthState(I)),
  )


  def table: Seq[(UInt, UInt)] = readNotSharedDirty ++ readUnique
}



object LoaclDatalessDecode {

  def cleanShared: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(CleanShared, UD, I,   I) -> (SnpAll | SnpOp(SnpCleanShared)        | RetToSrc),
    LocalReqInst(CleanShared, UC, I,   I) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared, SC, I,   I) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared, SC, I,  SC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared, SC, I,  SD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(SC)),
    LocalReqInst(CleanShared, SC, SC,  I) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared, SC, SC, SC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared, SC, SC, SD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(SC)),

    LocalReqInst(CleanShared,  I,  I,  I) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared,  I,  I, SC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared,  I,  I, SD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(SC)),
    LocalReqInst(CleanShared,  I,  I, UC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared,  I,  I, UD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(UC)),
    LocalReqInst(CleanShared,  I, UC,  I) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared,  I, UD,  I) -> (SnpAll | SnpOp(SnpCleanShared)        | RetToSrc),
    LocalReqInst(CleanShared,  I, SC,  I) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared,  I, SC, SC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanShared,  I, SC, SD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(SC)),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // UD  I  I
    LocalRespInst(REQ, CleanShared, UD,  I,  I, Snp, HasData, rn = ChiResp.UC_PD) -> (Commit | WSFDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(UC) | OthState(I)  | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    //  I UD  I
    LocalRespInst(REQ, CleanShared,  I, UD,  I, Snp, HasData, rn = ChiResp.UC_PD) -> (Commit | WSFDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I)  | OthState(UC) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
  )


  def cleanInvalid: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(CleanInvalid, UD, I,   I) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid, UC, I,   I) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid, SC, I,   I) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid, SC, I,  SC) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid, SC, I,  SD) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid, SC, SC,  I) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid, SC, SC, SC) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid, SC, SC, SD) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),

    LocalReqInst(CleanInvalid,  I,  I,  I) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(CleanInvalid,  I,  I, SC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(I)),
    LocalReqInst(CleanInvalid,  I,  I, SD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(I)),
    LocalReqInst(CleanInvalid,  I,  I, UC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(I)),
    LocalReqInst(CleanInvalid,  I,  I, UD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(I)),
    LocalReqInst(CleanInvalid,  I, UC,  I) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid,  I, UD,  I) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid,  I, SC,  I) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid,  I, SC, SC) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),
    LocalReqInst(CleanInvalid,  I, SC, SD) -> (SnpAll | SnpOp(SnpCleanInvalid)  | RetToSrc),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // UD  I  I
    LocalRespInst(REQ, CleanInvalid, UD,  I,  I, Snp, HasData, rn = ChiResp.I_PD)   -> (Commit | WSFDir|          RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    // UC  I  I
    LocalRespInst(REQ, CleanInvalid, UC,  I,  I, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    LocalRespInst(REQ, CleanInvalid, UC,  I,  I, Snp, HasData, rn = ChiResp.I_PD)   -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    // SC  I  I
    LocalRespInst(REQ, CleanInvalid, SC,  I,  I, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    // SC  I SC
    LocalRespInst(REQ, CleanInvalid, SC,  I, SC, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    // SC  I SD
    LocalRespInst(REQ, CleanInvalid, SC,  I, SD, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    // SC SC  I
    LocalRespInst(REQ, CleanInvalid, SC, SC,  I, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    // SC SC SC
    LocalRespInst(REQ, CleanInvalid, SC, SC, SC, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    // SC SC SD
    LocalRespInst(REQ, CleanInvalid, SC, SC, SD, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    //  I UC  I
    LocalRespInst(REQ, CleanInvalid,  I, UC,  I, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    LocalRespInst(REQ, CleanInvalid,  I, UC,  I, Snp, HasData, rn = ChiResp.I_PD)   -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    //  I UD  I
    LocalRespInst(REQ, CleanInvalid,  I, UD,  I, Snp, HasData, rn = ChiResp.I_PD)   -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    //  I SC  I
    LocalRespInst(REQ, CleanInvalid,  I, SC,  I, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    //  I SC SC
    LocalRespInst(REQ, CleanInvalid,  I, SC, SC, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
    //  I SC SD
    LocalRespInst(REQ, CleanInvalid,  I, SC, SD, Snp, HasData, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I) | WriteDown | WriOp(WriteNoSnpFullCleanInv)),
  )


  def makeInvalid: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(MakeInvalid, UD, I,   I) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid, UC, I,   I) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid, SC, I,   I) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid, SC, I,  SC) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid, SC, I,  SD) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid, SC, SC,  I) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid, SC, SC, SC) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid, SC, SC, SD) -> (SnpAll | SnpOp(SnpMakeInvalid)),

    LocalReqInst(MakeInvalid,  I,  I,  I) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(MakeInvalid,  I,  I, SC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(I)),
    LocalReqInst(MakeInvalid,  I,  I, SD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(I)),
    LocalReqInst(MakeInvalid,  I,  I, UC) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(I)),
    LocalReqInst(MakeInvalid,  I,  I, UD) -> (Commit | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | Flush | WSDir | HnState(I)),
    LocalReqInst(MakeInvalid,  I, UC,  I) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid,  I, UD,  I) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid,  I, SC,  I) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid,  I, SC, SC) -> (SnpAll | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeInvalid,  I, SC, SD) -> (SnpAll | SnpOp(SnpMakeInvalid)),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // UD  I  I
    LocalRespInst(REQ, MakeInvalid, UD,  I,  I, Snp, rn = ChiResp.I)      -> (Commit | WSFDir|          RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    // UC  I  I
    LocalRespInst(REQ, MakeInvalid, UC,  I,  I, Snp, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    // SC  I  I
    LocalRespInst(REQ, MakeInvalid, SC,  I,  I, Snp, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    // SC  I SC
    LocalRespInst(REQ, MakeInvalid, SC,  I, SC, Snp, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    // SC  I SD
    LocalRespInst(REQ, MakeInvalid, SC,  I, SD, Snp, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    // SC SC  I
    LocalRespInst(REQ, MakeInvalid, SC, SC,  I, Snp, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    // SC SC SC
    LocalRespInst(REQ, MakeInvalid, SC, SC, SC, Snp, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    // SC SC SD
    LocalRespInst(REQ, MakeInvalid, SC, SC, SD, Snp, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    //  I UC  I
    LocalRespInst(REQ, MakeInvalid,  I, UC,  I, Snp, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    //  I UD  I
    LocalRespInst(REQ, MakeInvalid,  I, UD,  I, Snp, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    //  I SC  I
    LocalRespInst(REQ, MakeInvalid,  I, SC,  I, Snp, rn = ChiResp.I)      -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    //  I SC SC
    LocalRespInst(REQ, MakeInvalid,  I, SC, SC, Snp, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
    //  I SC SD
    LocalRespInst(REQ, MakeInvalid,  I, SC, SD, Snp, rn = ChiResp.I)      -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(I)),
  )


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
    LocalReqInst(MakeUnique,  I, SC,  I) -> (SnpOth | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I, SC, SC) -> (SnpOth | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I, SC, SD) -> (SnpOth | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I, UC,  I) -> (SnpOth | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I, UD,  I) -> (SnpOth | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique,  I,  I, SC) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique,  I,  I, SD) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique,  I,  I, UC) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique,  I,  I, UD) -> (Commit | WSFDir | WSDir | RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique, SC,  I,  I) -> (Commit | WSFDir |         RespOp(Comp) | RespChnl(RSP) | Resp(ChiResp.UC) |  SrcState(UD) | OthState(I) | HnState(I)),
    LocalReqInst(MakeUnique, SC, SC,  I) -> (SnpOth | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique, SC, SC, SC) -> (SnpOth | SnpOp(SnpMakeInvalid)),
    LocalReqInst(MakeUnique, SC, SC, SD) -> (SnpOth | SnpOp(SnpMakeInvalid)),
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


  def table: Seq[(UInt, UInt)] = cleanShared ++ cleanInvalid ++ makeInvalid ++ evict ++ makeUnique
}


object LoaclWriteDecode {
  def writeBackFull: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(WriteBackFull, UD, I,   I) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull, UC, I,   I) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull, SC, I,   I) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull, SC, I,  SC) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull, SC, I,  SD) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull, SC, SC,  I) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull, SC, SC, SC) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull, SC, SC, SD) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),

    LocalReqInst(WriteBackFull,  I,  I,  I) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I,  I, SC) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I,  I, SD) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I,  I, UC) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I,  I, UD) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I, UC,  I) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I, UD,  I) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I, SC,  I) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I, SC, SC) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),
    LocalReqInst(WriteBackFull,  I, SC, SD) -> (Commit | RespOp(CompDBIDResp) |RespChnl(RSP) | Resp(ChiResp.I)),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // UD  I  I
    LocalRespInst(REQ, WriteBackFull, UD,  I,  I, CB, HasData, rn = ChiResp.UD_PD)  -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // UC  I  I
    LocalRespInst(REQ, WriteBackFull, UC,  I,  I, CB, HasData, rn = ChiResp.UC)     -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, WriteBackFull, UC,  I,  I, CB, HasData, rn = ChiResp.UD_PD)  -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC  I   I
    LocalRespInst(REQ, WriteBackFull, SC,  I,  I, CB, HasData, rn = ChiResp.SC)     -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UC) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC  I  SC
    LocalRespInst(REQ, WriteBackFull, SC,  I, SC, CB, HasData, rn = ChiResp.SC)     -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UC) | CleanDB),
    // SC  I SD
    LocalRespInst(REQ, WriteBackFull, SC,  I, SD, CB, HasData, rn = ChiResp.SC)     -> (WSFDir | WSDir | SrcState(I) | OthState(I)  | HnState(UD) | CleanDB),
    // SC SC  I
    LocalRespInst(REQ, WriteBackFull, SC, SC,  I, CB, HasData, rn = ChiResp.SC)     -> (WSFDir | WSDir | SrcState(I) | OthState(SC) | HnState(SC) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC SC SC
    LocalRespInst(REQ, WriteBackFull, SC, SC, SC, CB, HasData, rn = ChiResp.SC)     -> (WSFDir |         SrcState(I) | OthState(SC) | HnState(SC) | CleanDB),
    // SC SC SD
    LocalRespInst(REQ, WriteBackFull, SC, SC, SD, CB, HasData, rn = ChiResp.SC)     -> (WSFDir |         SrcState(I) | OthState(SC) | HnState(SD) | CleanDB),


    //  I XX XX
    LocalRespInst(REQ, WriteBackFull,  I,  I,  I, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I,  I, SC, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I,  I, SD, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I,  I, UC, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I,  I, UD, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, UC,  I, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, UD,  I, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, SC,  I, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, SC, SC, CB, HasData, rn = ChiResp.I)      -> CleanDB,
    LocalRespInst(REQ, WriteBackFull,  I, SC, SD, CB, HasData, rn = ChiResp.I)      -> CleanDB,
  )

  def writeUniqueFull: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(WriteUniqueFull,  I,  I,  I, HasData)      -> (WriteDown | WriOp(WriteNoSnpFull)),
    LocalReqInst(WriteUniqueFull,  I, UC,  I, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull) | SnpAll| SnpOp(SnpMakeInvalid)),
    LocalReqInst(WriteUniqueFull,  I, UD,  I, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull) | SnpAll| SnpOp(SnpMakeInvalid)),
    LocalReqInst(WriteUniqueFull,  I, SC,  I, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull) | SnpAll| SnpOp(SnpMakeInvalid)),
    LocalReqInst(WriteUniqueFull,  I, SC, SC, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull) | SnpAll| SnpOp(SnpMakeInvalid)),
    LocalReqInst(WriteUniqueFull,  I, SC, SD, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull) | SnpAll| SnpOp(SnpMakeInvalid)),
    LocalReqInst(WriteUniqueFull,  I,  I, SC, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull) | WSDir | HnState(UD)),
    LocalReqInst(WriteUniqueFull,  I,  I, SD, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull) | WSDir | HnState(UD)),
    LocalReqInst(WriteUniqueFull,  I,  I, UC, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull) | WSDir | HnState(UD)),
    LocalReqInst(WriteUniqueFull,  I,  I, UD, HasData)      -> (WriteDCU  | WriOp(WriteNoSnpFull)),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    //  I UC  I
    LocalRespInst(REQ, WriteUniqueFull, I, UC,  I, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD)),
    //  I UD  I
    LocalRespInst(REQ, WriteUniqueFull, I, UD,  I, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD)),
    //  I SC  I
    LocalRespInst(REQ, WriteUniqueFull, I, SC,  I, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD)),
    //  I SC SC
    LocalRespInst(REQ, WriteUniqueFull, I, SC, SC, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD)),
    //  I SC SD
    LocalRespInst(REQ, WriteUniqueFull, I, SC, SD, Snp, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD)),
  )


  def writeUniquePtl: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(WriteUniquePtl,  I,  I,  I, HasData)       -> (WriteDown | WriOp(WriteNoSnpPtl)),
    LocalReqInst(WriteUniquePtl,  I, UC,  I, HasData)       -> (SnpAll    | SnpOp(SnpUnique)     | RetToSrc),
    LocalReqInst(WriteUniquePtl,  I, UD,  I, HasData)       -> (SnpAll    | SnpOp(SnpUnique)     | RetToSrc),
    LocalReqInst(WriteUniquePtl,  I, SC,  I, HasData)       -> (SnpAll    | SnpOp(SnpUnique)     | RetToSrc),
    LocalReqInst(WriteUniquePtl,  I, SC, SC, HasData)       -> (WriteDCU  | WriOp(WriteNoSnpPtl) | SnpAll| SnpOp(SnpUnique)),
    LocalReqInst(WriteUniquePtl,  I, SC, SD, HasData)       -> (WriteDCU  | WriOp(WriteNoSnpPtl) | SnpAll| SnpOp(SnpUnique)),
    LocalReqInst(WriteUniquePtl,  I,  I, SC, HasData)       -> (WriteDCU  | WriOp(WriteNoSnpPtl) | WSDir | HnState(UD)),
    LocalReqInst(WriteUniquePtl,  I,  I, SD, HasData)       -> (WriteDCU  | WriOp(WriteNoSnpPtl) | WSDir | HnState(UD)),
    LocalReqInst(WriteUniquePtl,  I,  I, UC, HasData)       -> (WriteDCU  | WriOp(WriteNoSnpPtl) | WSDir | HnState(UD)),
    LocalReqInst(WriteUniquePtl,  I,  I, UD, HasData)       -> (WriteDCU  | WriOp(WriteNoSnpPtl)),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    //  I UC  I
    LocalRespInst(REQ, WriteUniquePtl, I, UC,  I, Snp, HasData, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I UD  I
    LocalRespInst(REQ, WriteUniquePtl, I, UD,  I, Snp, HasData, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC  I
    LocalRespInst(REQ, WriteUniquePtl, I, SC,  I, Snp, HasData, rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC SC
    LocalRespInst(REQ, WriteUniquePtl, I, SC, SC, Snp,          rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD)),
    //  I SC SD
    LocalRespInst(REQ, WriteUniquePtl, I, SC, SD, Snp,          rn = ChiResp.I) -> (Commit | WSFDir | WSDir | SrcState(I) | OthState(I) | HnState(UD)),
  )

  def table: Seq[(UInt, UInt)] = writeBackFull ++ writeUniqueFull ++ writeUniquePtl
}


object LoaclAtomicDecode {
  def atomicX: Seq[(UInt, UInt)] = Seq(
    // ----------------------------------------------------------- LOCAL REQ ----------------------------------------------------------------//
    LocalReqInst(AtomicLoadADD, UD, I,   I, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD, UC, I,   I, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD, SC, I,   I, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD, SC, I,  SC, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD, SC, I,  SD, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD, SC, SC,  I, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD, SC, SC, SC, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD, SC, SC, SD, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),

    LocalReqInst(AtomicLoadADD,  I,  I,  I, HasData) -> (ReadDown  | ReadOp(ReadNoSnp)),
    LocalReqInst(AtomicLoadADD,  I,  I, SC, HasData) -> (ReadDCU   | ReadOp(ReadNoSnp)),
    LocalReqInst(AtomicLoadADD,  I,  I, SD, HasData) -> (ReadDCU   | ReadOp(ReadNoSnp)),
    LocalReqInst(AtomicLoadADD,  I,  I, UC, HasData) -> (ReadDCU   | ReadOp(ReadNoSnp)),
    LocalReqInst(AtomicLoadADD,  I,  I, UD, HasData) -> (ReadDCU   | ReadOp(ReadNoSnp)),
    LocalReqInst(AtomicLoadADD,  I, UC,  I, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD,  I, UD,  I, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD,  I, SC,  I, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD,  I, SC, SC, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),
    LocalReqInst(AtomicLoadADD,  I, SC, SD, HasData) -> (SnpAll    | SnpOp(SnpUnique) | RetToSrc),

    // ----------------------------------------------------------- LOCAL RESP ---------------------------------------------------------------//
    // UD  I  I
    LocalRespInst(REQ, AtomicLoadADD, UD,  I,  I, Snp,  HasData, rn = ChiResp.I_PD)   -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // UC  I  I
    LocalRespInst(REQ, AtomicLoadADD, UC,  I,  I, Snp,  HasData, rn = ChiResp.I_PD)   -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, AtomicLoadADD, UC,  I,  I, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC  I  I
    LocalRespInst(REQ, AtomicLoadADD, SC,  I,  I, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC  I SC
    LocalRespInst(REQ, AtomicLoadADD, SC,  I, SC, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC  I SD
    LocalRespInst(REQ, AtomicLoadADD, SC,  I, SD, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC SC  I
    LocalRespInst(REQ, AtomicLoadADD, SC, SC,  I, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC SC SC
    LocalRespInst(REQ, AtomicLoadADD, SC, SC, SC, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    // SC SC SD
    LocalRespInst(REQ, AtomicLoadADD, SC, SC, SD, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I  I  I
    LocalRespInst(REQ, AtomicLoadADD,  I,  I,  I, Read, HasData, sn = ChiResp.UC)     -> (Commit | RDB2Src |          WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I  I SC
    LocalRespInst(REQ, AtomicLoadADD,  I,  I, SC, Read, HasData, sn = ChiResp.SC)     -> (Commit | RDB2Src |          WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I  I SD
    LocalRespInst(REQ, AtomicLoadADD,  I,  I, SD, Read, HasData, sn = ChiResp.SD)     -> (Commit | RDB2Src |          WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I  I UC
    LocalRespInst(REQ, AtomicLoadADD,  I,  I, UC, Read, HasData, sn = ChiResp.UC)     -> (Commit | RDB2Src |          WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I  I UD
    LocalRespInst(REQ, AtomicLoadADD,  I,  I, UD, Read, HasData, sn = ChiResp.UD_PD)  -> (Commit | RDB2Src |                  RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I UC  I
    LocalRespInst(REQ, AtomicLoadADD,  I, UC,  I, Snp,  HasData, rn = ChiResp.I_PD)   -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(REQ, AtomicLoadADD,  I, UC,  I, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I UD  I
    LocalRespInst(REQ, AtomicLoadADD,  I, UD,  I, Snp,  HasData, rn = ChiResp.I_PD)   -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC  I
    LocalRespInst(REQ, AtomicLoadADD,  I, SC,  I, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC SC
    LocalRespInst(REQ, AtomicLoadADD,  I, SC, SC, Snp,  HasData, rn = ChiResp.I)      -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
    //  I SC SD
    LocalRespInst(REQ, AtomicLoadADD,  I, SC, SD, Snp,  HasData, rn = ChiResp.I_PD)   -> (Commit | RDB2Src | WSFDir | WSDir | RespOp(CompData) | RespChnl(DAT) | Resp(ChiResp.I) | SrcState(I) | OthState(I) | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull)),
  )

  def table: Seq[(UInt, UInt)] = atomicX
}


object LoaclSnpUniqueEvictDecode {
  def snpUniqueEvict: Seq[(UInt, UInt)] = Seq(
    LocalSnpInst(SnpUniqueEvict,  I,  I,  I)    -> ERROE,
    LocalSnpInst(SnpUniqueEvict,  I,  I, SC)    -> ERROE,
    LocalSnpInst(SnpUniqueEvict,  I,  I, SD)    -> ERROE,
    LocalSnpInst(SnpUniqueEvict,  I,  I,  I)    -> ERROE,
    // ----------------------------------------------------------- LOCAL RESP ------------------------------------------------------------//
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I,  I, Snp,  HasData, rn = ChiResp.I)    -> (WSDir | HnState(UC) | WriteDCU | WriOp(WriteNoSnpFull)),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I, SC, Snp,  HasData, rn = ChiResp.I)    -> (WSDir | HnState(UC) | CleanDB),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I, SD, Snp,  HasData, rn = ChiResp.I)    -> (WSDir | HnState(UD) | CleanDB),
    LocalRespInst(SNP, SnpUniqueEvict,  I,  I,  I, Snp,  HasData, rn = ChiResp.I_PD) -> (WSDir | HnState(UD) | WriteDCU | WriOp(WriteNoSnpFull))
  )

  def table: Seq[(UInt, UInt)] = snpUniqueEvict
}