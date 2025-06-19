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

object Write_LAN {
  // WriteNoSnpPtl With EWA
  def writeNoSnpPtl_ewa_RO: DecodeType = (fromLAN | toLAN | reqIs(WriteNoSnpPtl) | ewa | isRO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I)) -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp))))
  ))

  // WriteNoSnpPtl Without EWA
  def writeNoSnpPtl_noEWA_OWO: DecodeType = (fromLAN | toLAN | reqIs(WriteNoSnpPtl) | isOWO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send") | write(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp))))
  ))

  // WriteNoSnpPtl With EWA
  def writeNoSnpPtl_ewa_OWO: DecodeType = (fromLAN | toLAN | reqIs(WriteNoSnpPtl) | ewa | isOWO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp))))
  ))

  // WriteUniquePtl Without Allocate
  def writeUniquePtl_noAlloc: DecodeType = (fromLAN | toLAN | reqIs(WriteUniquePtl) | ewa | isOWO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp)))),
    // I I SC -> I I I
    (sfMiss | llcIs(SC))  -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp) | wriLLC(I)))),
    // I I UC -> I I I
    (sfMiss | llcIs(UC))  -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp) | wriLLC(I)))),
    // I I UD -> I I I
    (sfMiss | llcIs(UD))  -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("read", "send") | write(WriteNoSnpPtl), cmtRsp(Comp) | wriLLC(I)))),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (waitRecDone | snpOth(SnpUnique) | retToSrc | needDB, Seq(
      (hasGotNCBWrData | datIs(SnpRespData) | respIs(I_PD)) -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp) | wriSNP(false)), // I I I
      (hasGotNCBWrData | datIs(SnpRespData) | respIs(I))    -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp) | wriSNP(false)), // I I I
      (hasGotNCBWrData | rspIs(SnpResp)     | respIs(I))    -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp) | wriSNP(false))  // I I I
    ))
  ))

  // WriteUniquePtl With Allocate
  def writeUniquePtl_alloc: DecodeType = (fromLAN | toLAN | reqIs(WriteUniquePtl) | allocate | ewa | isOWO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send") | write(WriteNoSnpPtl), cmtRsp(Comp)))),
    // I I SC -> I I UD
    (sfMiss | llcIs(SC))  -> first(waitRecDone, hasGotNCBWrData, cdop("read", "save") | cmtRsp(Comp) | wriLLC(UD)),
    // I I UC -> I I UD
    (sfMiss | llcIs(UC))  -> first(waitRecDone, hasGotNCBWrData, cdop("read", "save") | cmtRsp(Comp) | wriLLC(UD)),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> first(waitRecDone, hasGotNCBWrData, cdop("read", "save") | cmtRsp(Comp) | wriLLC(UD)),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (waitRecDone | snpOth(SnpUnique) | retToSrc | needDB, Seq(
      (hasGotNCBWrData | datIs(SnpRespData) | respIs(I_PD)) -> second(cdop("save") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UD)),          // I I UD
      (hasGotNCBWrData | datIs(SnpRespData) | respIs(I))    -> second(cdop("save") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UD)),          // I I UD
      (hasGotNCBWrData | rspIs(SnpResp)     | respIs(I))    -> second(tdop("send") | write(WriteNoSnpPtl),  cmtRsp(Comp) | wriSNP(false)) // I I I
    ))
  ))

  // WriteBackFull Without Allocate
  def writeBackFull_noAlloc: DecodeType = (fromLAN | toLAN | reqIs(WriteBackFull) | ewa | noOrder, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC))  -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC))  -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> first(waitRecDone, cbRespIs(I), noCmt),
    // I V I  -> I V I
    (srcMiss | othHit | llcIs(I)) -> first(waitRecDone, cbRespIs(I), noCmt),
    // V I I
    (srcHit | othMiss | llcIs(I)) -> (waitRecDone, Seq(
      cbRespIs(UD_PD) -> second(tdop("send") | write(WriteNoSnpFull), wriSRC(false)), // I I I
      cbRespIs(UC)    -> second(tdop("send") | write(WriteNoSnpFull), wriSRC(false)), // I I I
      cbRespIs(SC)    -> second(tdop("send") | write(WriteNoSnpFull), wriSRC(false)), // I I I
      cbRespIs(I)     -> second(wriSRC(false)), // I I I
    )),
    // V V I  -> I V I
    (srcHit | othHit  | llcIs(I)) -> (waitRecDone, Seq(
      cbRespIs(SC)    -> second(wriSRC(false)),
      cbRespIs(I)     -> second(wriSRC(false)),
    )),
  ))

  // WriteBackFull With Allocate
  def writeBackFull_alloc: DecodeType = (fromLAN | toLAN | reqIs(WriteBackFull) | allocate | ewa | noOrder, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC))  -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC))  -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> first(waitRecDone, cbRespIs(I), noCmt),
    // I V I  -> I V I
    (srcMiss | othHit | llcIs(I)) -> first(waitRecDone, cbRespIs(I), noCmt),
    // V I I
    (srcHit | othMiss | llcIs(I)) -> (waitRecDone, Seq(
      cbRespIs(UD_PD) -> second(cdop("save") | wriSRC(false) | wriLLC(UD)), // I I UD
      cbRespIs(UC)    -> second(cdop("save") | wriSRC(false) | wriLLC(UC)), // I I UC
      cbRespIs(SC)    -> second(cdop("save") | wriSRC(false) | wriLLC(UC)), // I I UC
      cbRespIs(I)     -> second(wriSRC(false)),                             // I I I
    )),
    // V V I  -> I V I
    (srcHit | othHit | llcIs(I))  -> (waitRecDone, Seq(
      cbRespIs(SC)    -> second(wriSRC(false)),
      cbRespIs(I)     -> second(wriSRC(false)),
    )),
  ))


  // WriteEvictOrEvict
  def writeEvictOrEvict: DecodeType = (fromLAN | toLAN | reqIs(WriteEvictOrEvict) | expCompAck | allocate | ewa | noOrder, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))  -> first(waitRecDone, cbRespIsCompAck, noCmt),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC)) -> first(waitRecDone, cbRespIsCompAck, noCmt),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC)) -> first(waitRecDone, cbRespIsCompAck, noCmt),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD)) -> first(waitRecDone, cbRespIsCompAck, noCmt),
    // I V I  -> I V I
    (srcMiss | othHit  | llcIs(I)) -> first(waitRecDone, cbRespIsCompAck, noCmt),
    // V I I
    (srcHit  | othMiss | llcIs(I)) -> (waitRecDone, Seq(
      cbRespIs(UD_PD) -> second(cdop("save") | wriSRC(false) | wriLLC(UD)), // I I UD
      cbRespIs(UC)    -> second(cdop("save") | wriSRC(false) | wriLLC(UC)), // I I UC
      cbRespIs(SC)    -> second(cdop("save") | wriSRC(false) | wriLLC(UC)), // I I UC
      cbRespIs(I)     -> second(wriSRC(false)),                             // I I I
    )),
    // V V I  -> I V I
    (srcHit  | othHit  | llcIs(I)) -> first(waitRecDone, cbRespIsCompAck, wriSRC(false)),
  ))

  // WriteCleanFull
  def writeCleanFull: DecodeType = (fromLAN | toLAN | reqIs(WriteCleanFull) | ewa | noOrder, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))  -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC)) -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC)) -> first(waitRecDone, cbRespIs(I), noCmt),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD)) -> first(waitRecDone, cbRespIs(I), noCmt),
    // I V I  -> I V I
    (srcMiss | othHit  | llcIs(I)) -> first(waitRecDone, cbRespIs(I), noCmt),
    // V I I -> V I I
    (srcHit  | othMiss | llcIs(I)) -> (waitRecDone, Seq(
      cbRespIs(UD_PD) -> second(tdop("send") | write(WriteNoSnpFull), noCmt),
      cbRespIs(UC)    -> second(noCmt),
      cbRespIs(SC)    -> second(noCmt),
      cbRespIs(I)     -> second(noCmt),
    )),
    // V V I  -> V V I
    (srcHit  | othHit  | llcIs(I)) -> (waitRecDone, Seq(
      cbRespIs(SC)    -> second(noCmt),
      cbRespIs(I)     -> second(noCmt),
    )),
  ))

  // writeNoSnpPtl ++ writeUniquePtl ++ writeBackFull ++ writeCleanFull ++ writeEvictOrEvict ++ writeCleanFull
  def table: Seq[DecodeType] = Seq(writeNoSnpPtl_ewa_RO, writeNoSnpPtl_noEWA_OWO, writeNoSnpPtl_ewa_OWO, writeUniquePtl_noAlloc, writeUniquePtl_alloc, writeEvictOrEvict, writeBackFull_noAlloc, writeBackFull_alloc, writeCleanFull)
}