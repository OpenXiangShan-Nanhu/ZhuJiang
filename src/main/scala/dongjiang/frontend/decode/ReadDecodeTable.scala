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
//  def readNoSnp: Seq[(UInt, UInt)] = Seq(
//    (isReq | toLAN | opIs(ReadNoSnp) | reqExpCompAck) -> (read | opcode(ReadNoSnp)),
//    (isReq | toLAN | opIs(ReadNoSnp))                 -> (read | opcode(ReadNoSnp) | needDB),
//  )

  // ReadNotSharedDirty To LAN
  def readNotSharedDirty: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadNotSharedDirty) | expCompAck, Seq(
    // I I I  -> UC I I
    (sfMiss | llcIs(I))  -> first(read(ReadNoSnp), wriSRC(true)),
    // I I SC -> UC I I
    (sfMiss | llcIs(SC)) -> first(cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UC -> UC I I
    (sfMiss | llcIs(UC)) -> first(cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UD -> UD I I
    (sfMiss | llcIs(UD)) -> first(cmtDat(CompData) | resp(UD_PD) | wriSRC(true) | wriLLC(I)),
    // I V I
    (srcMiss | othHit | llcIs(I))  -> (snpOth(SnpNotSharedDirtyFwd) | needDB, Seq(
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(SC)) -> second(wriSRC(true)), // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(SC)    | fwdIs(SC)) -> second(wriSRC(true)), // SC SC I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(SC)) -> second(wriSRC(true)), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(I)     | fwdIs(SC)) -> second(wriSRC(true)), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(SC_PD) | fwdIs(SC)) -> second(wriOrAtm(WriteNoSnpFull), wriSRC(true)), // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(I_PD)  | fwdIs(SC)) -> second(wriOrAtm(WriteNoSnpFull), wriSRC(true)), // SC I  I
      (rspIs(SnpResp)           | respIs(I))                 -> second(read(ReadNoSnp), wriSRC(true)) // UC I  I // No Fwd
    )),
    // V I I -> UC I I
    (srcHit | othMiss | llcIs(I))  -> first(read(ReadNoSnp), noCmt),
    // V V I
    (srcHit | othHit | llcIs(I))  -> (snpOth(SnpNotSharedDirtyFwd) | needDB, Seq(
      (rspIs(SnpRespFwded)      | respIs(SC)    | fwdIs(SC)) -> second(noCmt), // SC SC I
      (datIs(SnpRespDataFwded)  | respIs(SC)    | fwdIs(SC)) -> second(noCmt), // SC SC I
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(SC)) -> second(noCmt), // SC I  I
      (datIs(SnpRespDataFwded)  | respIs(I)     | fwdIs(SC)) -> second(noCmt), // SC I  I
      (rspIs(SnpResp)           | respIs(I))                 -> second(read(ReadNoSnp), noCmt) // UC I  I // No Fwd
    )),
  ))

  // ReadUnique To LAN
  def readUnique: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(ReadUnique) | expCompAck, Seq(
    // I I I  -> UC I I
    (sfMiss | llcIs(I))  -> first(read(ReadNoSnp), wriSRC(true)),
    // I I SC -> UC I I
    (sfMiss | llcIs(SC)) -> first(cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UC -> UC I I
    (sfMiss | llcIs(UC)) -> first(cmtDat(CompData) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UD -> UD I I
    (sfMiss | llcIs(UD)) -> first(cmtDat(CompData) | resp(UD_PD) | wriSRC(true) | wriLLC(I)),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (snpOth(SnpUniqueFwd) | needDB, Seq(
      (rspIs(SnpRespFwded)      | respIs(I)     | fwdIs(UC))    -> second(wriSRC(true)), // UC I I
      (datIs(SnpRespDataFwded)  | respIs(I)     | fwdIs(UD_PD)) -> second(wriSRC(true)), // UD I I
      (datIs(SnpRespData)       | respIs(I_PD))                 -> second(cmtDat(CompData) | resp(UD) | wriSRC(true)), // UD I I // No Fwd
      (rspIs(SnpResp)           | respIs(I))                    -> second(read(ReadNoSnp), wriSRC(true)) // UC I I // No Fwd
    )),
    // V I I -> UC I I
    (srcHit | othMiss | llcIs(I)) -> first(read(ReadNoSnp), noCmt),
    // V V I
    (srcHit | othHit | llcIs(I))  -> (snpOth(SnpUniqueFwd) | needDB, Seq(
      (rspIs(SnpRespFwded)  | respIs(I) | fwdIs(UC))  -> second(noCmt), // UC I I
      (rspIs(SnpResp)       | respIs(I))              -> second(read(ReadNoSnp), noCmt) // UC I I // No Fwd
    )),
  ))

  // readNoSnp ++ readOnce ++ readNotSharedDirty ++ readUnique ++ makeReadUnique
  def table: Seq[(UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))])] = Seq(readNotSharedDirty, readUnique)
}