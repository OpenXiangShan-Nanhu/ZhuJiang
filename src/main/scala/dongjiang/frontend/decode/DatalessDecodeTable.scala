package dongjiang.frontend.decode

import dongjiang._
import dongjiang.frontend._
import dongjiang.frontend.decode.Decode.DecodeType
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

object Dataless_LAN {
  // MakeUnique
  def makeUnique: DecodeType = (fromLAN | toLAN | reqIs(MakeUnique) | expCompAck, Seq(
    // I I I  -> UD I I
    (sfMiss | llcIs(I))  -> first(cmtRsp(Comp) | resp(UC) | wriSRC(true)),
    // I I SC -> UD I I
    (sfMiss | llcIs(SC)) -> first(cmtRsp(Comp) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UC -> UD I I
    (sfMiss | llcIs(UC)) -> first(cmtRsp(Comp) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I I UD -> UD I I
    (sfMiss | llcIs(UD)) -> first(cmtRsp(Comp) | resp(UC) | wriSRC(true) | wriLLC(I)),
    // I V I  -> UD I I
    (srcMiss | othHit  | llcIs(I)) -> (snpOth(SnpMakeInvalid), Seq((rspIs(SnpResp) | respIs(I)) -> second(cmtRsp(Comp) | resp(UC) | wriSRC(true) | wriSNP(false)))),
    // V I I  -> UD I I
    (srcHit  | othMiss | llcIs(I)) -> first(cmtRsp(Comp) | resp(UC)),
    // V V I  -> UD I I
    (srcHit  | othHit  | llcIs(I)) -> (snpOth(SnpMakeInvalid), Seq((rspIs(SnpResp) | respIs(I)) -> second(cmtRsp(Comp) | resp(UC) | wriSRC(true) | wriSNP(false)))),
  ))


  // Evict
  def evict: DecodeType = (fromLAN | toLAN | reqIs(Evict), Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))  -> first(cmtRsp(Comp) | resp(I)),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC)) -> first(cmtRsp(Comp) | resp(I)),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC)) -> first(cmtRsp(Comp) | resp(I)),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD)) -> first(cmtRsp(Comp) | resp(I)),
    // I V I  -> I V I
    (srcMiss | othHit | llcIs(I)) -> first(cmtRsp(Comp) | resp(I)),
    // V I I  -> I I I
    (srcHit | othMiss | llcIs(I)) -> first(cmtRsp(Comp) | resp(I) | wriSRC(false)),
    // V V I  -> I V I
    (srcHit | othHit | llcIs(I))  -> first(cmtRsp(Comp) | resp(I) | wriSRC(false)),
  ))

  // CleanShared
  def cleanShared: DecodeType = (fromLAN | toLAN | reqIs(CleanShared), Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))  -> first(cmtRsp(Comp) | resp(I)),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC)) -> first(cmtRsp(Comp) | resp(I)),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC)) -> first(cmtRsp(Comp) | resp(I)),
    // I I UD -> I I UC
    (sfMiss | llcIs(UD)) -> first(needDB | tdop("read", "send") | write(WriteNoSnpFull), cmtRsp(Comp) | resp(I) | wriLLC(UC)),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (snpAll(SnpCleanShared) | needDB, Seq(
      (rspIs(SnpResp)     | respIs(I))      -> second(cmtRsp(Comp) | resp(I) | wriSNP(false)), // I I  I
      (rspIs(SnpResp)     | respIs(UC))     -> second(cmtRsp(Comp) | resp(I)), // I UC I
      (rspIs(SnpResp)     | respIs(SC))     -> second(cmtRsp(Comp) | resp(I)), // I SC I
      (datIs(SnpRespData) | respIs(UC_PD))  -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp)), // I UC I
      (datIs(SnpRespData) | respIs(SC_PD))  -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp)), // I SC I
      (datIs(SnpRespData) | respIs(I_PD))   -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cdop("save") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UC)), // I I I
    )),
    // V I I
    (srcHit | othMiss | llcIs(I)) -> (snpAll(SnpCleanShared) | needDB, Seq(
      (rspIs(SnpResp)     | respIs(I))      -> second(cmtRsp(Comp) | resp(I) | wriSNP(false)), // I I  I
      (rspIs(SnpResp)     | respIs(UC))     -> second(cmtRsp(Comp) | resp(I)), // I UC I
      (rspIs(SnpResp)     | respIs(SC))     -> second(cmtRsp(Comp) | resp(I)), // I SC I
      (datIs(SnpRespData) | respIs(UC_PD))  -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp)), // I UC I
      (datIs(SnpRespData) | respIs(SC_PD))  -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp)), // I SC I
      (datIs(SnpRespData) | respIs(I_PD))   -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cdop("save") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UC)), // I I I
    )),
    // V V I  -> V V I
    (srcHit | othHit | llcIs(I)) -> first(cmtRsp(Comp) | resp(I)),
  ))

  // CleanInvalid
  def cleanInvalid: DecodeType = (fromLAN | toLAN | reqIs(CleanInvalid), Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))  -> first(cmtRsp(Comp) | resp(I)),
    // I I SC -> I I I
    (sfMiss | llcIs(SC)) -> first(cmtRsp(Comp) | resp(I) | wriLLC(I)),
    // I I UC -> I I I
    (sfMiss | llcIs(UC)) -> first(cmtRsp(Comp) | resp(I) | wriLLC(I)),
    // I I UD -> I I I
    (sfMiss | llcIs(UD)) -> first(needDB | tdop("read", "send") | write(WriteNoSnpFull), cmtRsp(Comp) | resp(I) | wriLLC(I)),
    // I V I  -> I I I
    (srcMiss | othHit | llcIs(I)) -> (snpAll(SnpCleanInvalid) | needDB, Seq(
      (rspIs(SnpResp)     | respIs(I))      -> second(cmtRsp(Comp) | resp(I) | wriSNP(false)),
      (datIs(SnpRespData) | respIs(I_PD))   -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp) | wriSNP(false)),
    )),
    // V I I  -> I I I
    (srcHit | othMiss | llcIs(I)) -> (snpAll(SnpCleanInvalid) | needDB, Seq(
      (rspIs(SnpResp)     | respIs(I))      -> second(cmtRsp(Comp) | resp(I) | wriSNP(false)),
      (datIs(SnpRespData) | respIs(I_PD))   -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp) | wriSNP(false)),
    )),
    // V V I  -> I V I
    (srcHit | othHit | llcIs(I))   -> (snpAll(SnpMakeInvalid), Seq(
      (rspIs(SnpResp) | respIs(I)) -> second(cmtRsp(Comp) | resp(I) | wriSNP(false)),
    )),
  ))

  // MakeInvalid
  def makeInvalid: DecodeType = (fromLAN | toLAN | reqIs(MakeInvalid), Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))  -> first(cmtRsp(Comp) | resp(I)),
    // I I SC -> I I I
    (sfMiss | llcIs(SC)) -> first(cmtRsp(Comp) | resp(I) | wriLLC(I)),
    // I I UC -> I I I
    (sfMiss | llcIs(UC)) -> first(cmtRsp(Comp) | resp(I) | wriLLC(I)),
    // I I UD -> I I I
    (sfMiss | llcIs(UD)) -> first(cmtRsp(Comp) | resp(I) | wriLLC(I)),
    // I V I  -> I I I
    (srcMiss | othHit | llcIs(I))  -> (snpAll(SnpMakeInvalid), Seq(
      (rspIs(SnpResp) | respIs(I)) -> second(cmtRsp(Comp) | resp(I) | wriSNP(false)),
    )),
    // V I I  -> I I I
    (srcHit | othMiss | llcIs(I))  -> (snpAll(SnpMakeInvalid), Seq(
      (rspIs(SnpResp) | respIs(I)) -> second(cmtRsp(Comp) | resp(I) | wriSNP(false)),
    )),
    // V V I  -> I V I
    (srcHit | othHit | llcIs(I))   -> (snpAll(SnpMakeInvalid), Seq(
      (rspIs(SnpResp) | respIs(I)) -> second(cmtRsp(Comp) | resp(I) | wriSNP(false)),
    )),
  ))
  
  // makeUnique ++ evict ++ cleanShared ++ cleanInvalid ++ makeInvalid
  def table: Seq[DecodeType] = Seq(makeUnique, evict, cleanShared, cleanInvalid, makeInvalid)
}