package dongjiang.frontend.decode

import dongjiang._
import dongjiang.frontend._
import dongjiang.frontend.decode.Decode.DecodeType
import dongjiang.frontend.decode.Inst._
import dongjiang.frontend.decode.Code.{wriSRC, _}
import dongjiang.frontend.decode.DecodeCHI._
import dongjiang.bundle._
import dongjiang.bundle.ChiChannel._
import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.SnpOpcode._
import zhujiang.chi._
import chisel3._
import chisel3.util._



object Read_LAN_DCT_DMT {
  // ReadNoSnp With ExpCompAck And EO
  def readNoSnp0: DecodeType = (fromLAN | toLAN | reqIs(ReadNoSnp) | expCompAck | isEO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I)) -> first(read(ReadNoSnp) | doDMT, emptyResp, noCmt)
  ))

  // ReadNoSnp Without ExpCompAck And EO
  def readNoSnp1: DecodeType = (fromLAN | toLAN | reqIs(ReadNoSnp) | isEO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I)) -> (tdop("reqs") | read(ReadNoSnp), Seq((datIs(CompData) | respIs(UC)) -> second(cdop("send", "clean") | cmtDat(CompData) | resp(UC))))
  ))

  // ReadNoSnp With ExpCompAck And NoOrder
  def readNoSnp2: DecodeType = (fromLAN | toLAN | reqIs(ReadNoSnp) | expCompAck | noOrder, readNoSnp0._2)

  // ReadOnce
  def readOnce: DecodeType = (fromLAN | toLAN | reqIs(ReadOnce) | expCompAck | isEO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> first(read(ReadNoSnp) | doDMT, emptyResp, noCmt),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC))  -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(I)),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC))  -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(I)),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(I)),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (tdop("reqs") | snpOne(SnpOnceFwd), Seq(
      (rspIs(SnpRespFwded)      | respIs(UC)    | fwdIs(I))   -> second(cdop("clean")),                                           // I UC  I
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(I))   -> second(cdop("clean")),                                           // I SC  I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(I))   -> second(cdop("clean") | wriSNP(false)),                           // I SC  I
      (rspIs(SnpRespFwded)      | respIs(UD)    | fwdIs(I))   -> second(cdop("clean")),                                           // I UD  I
      (datIs(SnpRespDataFwded)  | respIs(SC_PD) | fwdIs(I))   -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpFull), emptyResp, noCmt), // I SC  I
      (datIs(SnpRespDataFwded)  | respIs(I_PD)  | fwdIs(I))   -> second(cdop("save", "clean") | wriSNP(false) | wriLLC(UD)),      // I  I UD
      (rspIs(SnpResp)           | respIs(SC))                 -> second(read(ReadNoSnp) | doDMT, emptyResp, cdop("clean")),                  // I SC  I
      (rspIs(SnpResp)           | respIs(I))                  -> second(read(ReadNoSnp) | doDMT, emptyResp, cdop("clean") | wriSNP(false)),  // I  I  I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(I))   -> second(cdop("clean") | wriSNP(false)),                           // I  I  I
    )),
  ))

  // ReadNotSharedDirty
  def readNotSharedDirty: DecodeType = (fromLAN | toLAN | reqIs(ReadNotSharedDirty) | expCompAck | noOrder, Seq(
    // I I I  -> UC I I
    (sfMiss | llcIs(I))  -> first(read(ReadNoSnp) | doDMT, emptyResp, wriSRC(true)),
    // I I SC -> UC I I
    (sfMiss | llcIs(SC)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UC -> UC I I
    (sfMiss | llcIs(UC)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UD -> UD I I
    (sfMiss | llcIs(UD)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UD_PD) | wriSRC(true) | wriLLC(I)),
    // I V I
    (srcMiss | othHit | llcIs(I))  -> (tdop("reqs") | snpOne(SnpNotSharedDirtyFwd), Seq(
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(SC)) -> second(cdop("clean") | wriSRC(true)),  // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(SC)    | fwdIs(SC)) -> second(cdop("clean") | wriSRC(true)),  // SC SC I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(SC)) -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(I)     | fwdIs(SC)) -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(SC_PD) | fwdIs(SC)) -> second(tdop("send") | wriOrAtm(WriteNoSnpFull), emptyResp, waitSecDone | cdop("clean") | wriSRC(true)),  // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(I_PD)  | fwdIs(SC)) -> second(tdop("send") | wriOrAtm(WriteNoSnpFull), emptyResp, waitSecDone | cdop("clean") | wriSRC(true) | wriSNP(false)), // SC I  I
      (rspIs(SnpResp)           | respIs(I))                 -> second(read(ReadNoSnp) | doDMT, emptyResp, cdop("clean") | wriSRC(true) | wriSNP(false)) // UC I  I // No Fwd
    )),
    // V I I -> UC I I
    (srcHit | othMiss | llcIs(I))  -> first(read(ReadNoSnp) | doDMT, emptyResp, noCmt),
    // V V I
    (srcHit | othHit | llcIs(I))  -> (tdop("reqs") | snpOne(SnpNotSharedDirtyFwd), Seq(
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(SC)) -> second(cdop("clean") | noCmt), // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(SC)    | fwdIs(SC)) -> second(cdop("clean") | noCmt), // SC SC I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(SC)) -> second(cdop("clean") | wriSNP(false)), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(I)     | fwdIs(SC)) -> second(cdop("clean") | wriSNP(false)), // SC I  I
      (rspIs(SnpResp)           | respIs(I))                 -> second(read(ReadNoSnp) | doDMT, emptyResp, cdop("clean") | wriSNP(false)) // UC I  I // No Fwd
    )),
  ))

  // ReadUnique
  def readUnique: DecodeType = (fromLAN | toLAN | reqIs(ReadUnique) | expCompAck | noOrder, Seq(
    // I I I  -> UC I I
    (sfMiss | llcIs(I))  -> first(read(ReadNoSnp) | doDMT, emptyResp, wriSRC(true)),
    // I I SC -> UC I I
    (sfMiss | llcIs(SC)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UC -> UC I I
    (sfMiss | llcIs(UC)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UD -> UD I I
    (sfMiss | llcIs(UD)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UD_PD) | wriSRC(true) | wriLLC(I)),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (tdop("reqs") | snpOth(SnpUniqueFwd), Seq(
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(UC))    -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // UC I I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(UD_PD)) -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // UD I I
      (datIs(SnpRespData)       | respIs(I_PD))                 -> second(cdop("send", "clean") | cmtDat(CompData) | resp(UD_PD) | wriSRC(true) | wriSNP(false)), // UD I I // No Fwd
      (rspIs(SnpResp)           | respIs(I))                    -> second(read(ReadNoSnp) | doDMT, emptyResp, cdop("clean") | wriSRC(true) | wriSNP(false))       // UC I I // No Fwd
    )),
    // V I I -> UC I I
    (srcHit | othMiss | llcIs(I)) -> first(read(ReadNoSnp) | doDMT, emptyResp, noCmt),
    // V V I
    (srcHit | othHit | llcIs(I))  -> (tdop("reqs") | snpOth(SnpUniqueFwd), Seq(
      (rspIs(SnpRespFwded)  | respIs(I) | fwdIs(UC))  -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // UC I I
      (rspIs(SnpResp)       | respIs(I))              -> second(read(ReadNoSnp) | doDMT, emptyResp, cdop("clean") | wriSRC(true)  | wriSNP(false)) // UC I I // No Fwd
    )),
  ))

  // readNoSnp ++ readOnce ++ readNotSharedDirty ++ readUnique
  def table: Seq[DecodeType] = Seq(readNoSnp0, readNoSnp1, readNoSnp2, readOnce, readNotSharedDirty, readUnique)
}