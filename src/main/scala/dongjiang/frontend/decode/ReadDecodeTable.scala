package dongjiang.frontend.decode

import dongjiang._
import dongjiang.frontend._
import dongjiang.frontend.decode.Inst._
import dongjiang.frontend.decode.Code._
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
  def readNoSnp0: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadNoSnp) | expCompAck | isEO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I)) -> first(read(ReadNoSnp) | doDMT, noCmt)
  ))

  // ReadNoSnp Without ExpCompAck And EO
  def readNoSnp1: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadNoSnp) | isEO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I)) -> (tdop("reqs") | read(ReadNoSnp), Seq((datIs(CompData) | respIs(UC)) -> second(cdop("send", "clean") | cmtDat(CompData) | resp(UC))))
  ))

  // ReadNoSnp With ExpCompAck And NoOrder
  def readNoSnp2: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadNoSnp) | expCompAck | noOrder, readNoSnp0._2)

  // ReadOnce With Allocate
  def readOnce0: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadOnce) | expCompAck | allocate | ewa | noOrder, Seq(
    // I I I  -> UC I I
    (sfMiss | llcIs(I))   -> first(read(ReadNoSnp) | doDMT, wriSRC(true)),
    // I I SC -> UC I I
    (sfMiss | llcIs(SC))  -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(I)),
    // I I UC -> UC I I
    (sfMiss | llcIs(UC))  -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(I)),
    // I I UD -> UD I I
    (sfMiss | llcIs(UD))  -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(I)),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (tdop("reqs") | snpOne(SnpOnceFwd), Seq(
      (rspIs(SnpRespFwded)      | respIs(UC)    | fwdIs(I))   -> second(cdop("clean")), // I UC I
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(I))   -> second(cdop("clean")), // I SC I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(I))   -> second(cdop("clean") | wriSNP(false)), // I SC I
      (rspIs(SnpRespFwded)      | respIs(UD)    | fwdIs(I))   -> second(cdop("clean")), // I UD I
      (datIs(SnpRespDataFwded)  | respIs(SC_PD) | fwdIs(I))   -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpFull), noCmt), // I SC I
      (datIs(SnpRespDataFwded)  | respIs(I_PD)  | fwdIs(I))   -> second(cdop("save", "clean") | wriSNP(false) | wriLLC(UD)), // I I UD
      (rspIs(SnpResp)           | respIs(SC))                 -> second(read(ReadNoSnp) | doDMT, cdop("clean")), // I SC I
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(I))   -> second(cdop("clean")), // I SC I
      (rspIs(SnpResp)           | respIs(I))                  -> second(read(ReadNoSnp) | doDMT, cdop("clean") | wriSNP(false)), // I I I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(I))   -> second(cdop("clean") | wriSNP(false)), // I I I
    )),
  ))

  // ReadOnce Without Allocate
  def readOnce1: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadOnce) | expCompAck | noOrder, readOnce0._2)

  // ReadNotSharedDirty
  def readNotSharedDirty: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadNotSharedDirty) | expCompAck | noOrder, Seq(
    // I I I  -> UC I I
    (sfMiss | llcIs(I))  -> first(read(ReadNoSnp) | doDMT, wriSRC(true)),
    // I I SC -> UC I I
    (sfMiss | llcIs(SC)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UC -> UC I I
    (sfMiss | llcIs(UC)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UD -> UD I I
    (sfMiss | llcIs(UD)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UD_PD) | wriSRC(true) | wriLLC(I)),
    // I V I
    (srcMiss | othHit | llcIs(I))  -> (tdop("reqs") | snpOne(SnpNotSharedDirtyFwd), Seq(
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(SC)) -> second(cdop("clean") | wriSRC(true) | wriSNP(true)),  // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(SC)    | fwdIs(SC)) -> second(cdop("clean") | wriSRC(true) | wriSNP(true)),  // SC SC I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(SC)) -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(I)     | fwdIs(SC)) -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(SC_PD) | fwdIs(SC)) -> second(tdop("send") | wriOrAtm(WriteNoSnpFull), cdop("clean") | wriSRC(true) | wriSNP(true)),  // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(I_PD)  | fwdIs(SC)) -> second(tdop("send") | wriOrAtm(WriteNoSnpFull), cdop("clean") | wriSRC(true) | wriSNP(false)), // SC I  I
      (rspIs(SnpResp)           | respIs(I))                 -> second(read(ReadNoSnp) | doDMT, cdop("clean") | wriSRC(true) | wriSNP(false)) // UC I  I // No Fwd
    )),
    // V I I -> UC I I
    (srcHit | othMiss | llcIs(I))  -> first(read(ReadNoSnp) | doDMT, noCmt),
    // V V I
    (srcHit | othHit | llcIs(I))  -> (tdop("reqs") | snpOne(SnpNotSharedDirtyFwd), Seq(
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(SC)) -> second(cdop("clean") | noCmt), // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(SC)    | fwdIs(SC)) -> second(cdop("clean") | noCmt), // SC SC I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(SC)) -> second(cdop("clean") | wriSNP(false)), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(I)     | fwdIs(SC)) -> second(cdop("clean") | wriSNP(false)), // SC I  I
      (rspIs(SnpResp)           | respIs(I))                 -> second(read(ReadNoSnp) | doDMT, cdop("clean") | wriSNP(false)) // UC I  I // No Fwd
    )),
  ))

  // ReadUnique
  def readUnique: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadUnique) | expCompAck | noOrder, Seq(
    // I I I  -> UC I I
    (sfMiss | llcIs(I))  -> first(read(ReadNoSnp) | doDMT, wriSRC(true)),
    // I I SC -> UC I I
    (sfMiss | llcIs(SC)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UC -> UC I I
    (sfMiss | llcIs(UC)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UD -> UD I I
    (sfMiss | llcIs(UD)) -> first(cdop("reqs", "read", "send", "clean") | cmtDat(CompData) | resp(UD_PD) | wriSRC(true) | wriLLC(I)),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (tdop("reqs") | snpOth(SnpUniqueFwd), Seq(
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(UC))    -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // UC I I
      (datIs(SnpRespDataFwded)  | respIs(I)     | fwdIs(UD_PD)) -> second(cdop("clean") | wriSRC(true) | wriSNP(false)), // UD I I
      (datIs(SnpRespData)       | respIs(I_PD))                 -> second(cdop("send", "clean") | cmtDat(CompData) | resp(UD) | wriSRC(true) | wriSNP(false)), // UD I I // No Fwd
      (rspIs(SnpResp)           | respIs(I))                    -> second(read(ReadNoSnp) | doDMT, cdop("clean") | wriSRC(true) | wriSNP(false)) // UC I I // No Fwd
    )),
    // V I I -> UC I I
    (srcHit | othMiss | llcIs(I)) -> first(read(ReadNoSnp) | doDMT, noCmt),
    // V V I
    (srcHit | othHit | llcIs(I))  -> (tdop("reqs") | snpOth(SnpUniqueFwd), Seq(
      (rspIs(SnpRespFwded)  | respIs(I) | fwdIs(UC))  -> second(cdop("clean") | noCmt), // UC I I
      (rspIs(SnpResp)       | respIs(I))              -> second(read(ReadNoSnp) | doDMT, cdop("clean") | wriSNP(false)) // UC I I // No Fwd
    )),
  ))

  // readNoSnp ++ readOnce ++ readNotSharedDirty ++ readUnique
  def table: Seq[(UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))])] = Seq(readNoSnp0, readNoSnp1, readNoSnp2, readOnce0, readOnce1, readNotSharedDirty, readUnique)
}