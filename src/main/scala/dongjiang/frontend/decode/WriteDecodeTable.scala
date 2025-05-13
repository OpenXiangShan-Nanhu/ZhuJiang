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
  def writeNoSnpPtl_ewa: DecodeType = (fromLAN | toLAN | reqIs(WriteNoSnpPtl) | ewa | expCompAck | isOWO, Seq(
    (sfMiss | llcIs(I)) -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, noCmt)))
  ))

  // WriteNoSnpPtl Without EWA
  def writeNoSnpPtl_noEwa: DecodeType = (fromLAN | toLAN | reqIs(WriteNoSnpPtl) | expCompAck | isOWO, Seq(
    (sfMiss | llcIs(I)) -> (waitRecDone, Seq(hasGotNCBWrData -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, waitSecDone | cmtRsp(Comp))))
  ))

  // WriteUniquePtl With Allocate
  def writeUniquePtl_alloc: DecodeType = (fromLAN | toLAN | reqIs(WriteUniquePtl) | allocate | ewa | isOWO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> (waitRecDone, Seq(cbRespIs(I) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, cmtRsp(Comp)))),
    // I I SC -> I I UD
    (sfMiss | llcIs(SC))  -> first(waitRecDone, cbRespIs(I), cdop("read", "save", "clean") | cmtRsp(Comp) | wriLLC(UD)),
    // I I UC -> I I UD
    (sfMiss | llcIs(UC))  -> first(waitRecDone, cbRespIs(I), cdop("read", "save", "clean") | cmtRsp(Comp) | wriLLC(UD)),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> first(waitRecDone, cbRespIs(I), cdop("read", "save", "clean") | cmtRsp(Comp) | wriLLC(UD)),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (waitRecDone | tdop("reqs") | snpOth(SnpUnique) | retToSrc, Seq(
      (cbRespIs(I) | datIs(SnpRespData) | respIs(I_PD)) -> second(cdop("save", "clean") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UD)), // I I UD
      (cbRespIs(I) | datIs(SnpRespData) | respIs(I))    -> second(cdop("save", "clean") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UD)), // I I UD
      (cbRespIs(I) | rspIs(SnpResp)     | respIs(I))    -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp,  cmtRsp(Comp) | wriSNP(false)) // I I I
    ))
  ))

  // WriteUnique Without Allocate
  def writeUniquePtl_noAlloc: DecodeType = (fromLAN | toLAN | reqIs(WriteUniquePtl) | ewa | isOWO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> (waitRecDone, Seq(cbRespIs(I) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, waitSecDone | cmtRsp(Comp)))),
    // I I SC -> I I UD
    (sfMiss | llcIs(SC))  -> (waitRecDone, Seq(cbRespIs(I) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, waitSecDone | cmtRsp(Comp) | wriLLC(I)))),
    // I I UC -> I I UD
    (sfMiss | llcIs(UC))  -> (waitRecDone, Seq(cbRespIs(I) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, waitSecDone | cmtRsp(Comp) | wriLLC(I)))),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> (waitRecDone, Seq(cbRespIs(I) -> second(tdop("read", "send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, waitSecDone | cmtRsp(Comp) | wriLLC(I)))),
    // I V I
    (srcMiss | othHit | llcIs(I)) -> (waitRecDone | tdop("reqs") | snpOth(SnpUnique) | retToSrc, Seq(
      (cbRespIs(I) | datIs(SnpRespData) | respIs(I_PD)) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, cmtRsp(Comp) | wriSNP(false)), // I I I
      (cbRespIs(I) | datIs(SnpRespData) | respIs(I))    -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, cmtRsp(Comp) | wriSNP(false)), // I I I
      (cbRespIs(I) | rspIs(SnpResp)     | respIs(I))    -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), emptyResp, cmtRsp(Comp) | wriSNP(false))  // I I I
    ))
  ))


  // WriteBackFull
  def writeBackFull: DecodeType = (fromLAN | toLAN | reqIs(WriteBackFull) | allocate | ewa | noOrder, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> first(waitRecDone, cbRespIs(I), cdop("clean")),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC))  -> first(waitRecDone, cbRespIs(I), cdop("clean")),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC))  -> first(waitRecDone, cbRespIs(I), cdop("clean")),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> first(waitRecDone, cbRespIs(I), cdop("clean")),
    // I V I  -> I V I
    (srcMiss | othHit | llcIs(I)) -> first(waitRecDone, cbRespIs(I), cdop("clean")),
    // V I I
    (srcHit | othMiss | llcIs(I)) -> (waitRecDone, Seq(
      cbRespIs(UD_PD) -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UD)), // I I UD
      cbRespIs(UC)    -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UC)), // I I UC
      cbRespIs(SC)    -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UC)), // I I UC
      cbRespIs(I)     -> second(cdop("clean")         | wriSRC(false)), // I I I
    )),
    // V V I  -> I V I
    (srcHit | othHit | llcIs(I)) -> (waitRecDone, Seq(
      cbRespIs(SC)    -> second(cdop("clean") | wriSRC(false)),
      cbRespIs(I)     -> second(cdop("clean") | wriSRC(false)),
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
      cbRespIs(UD_PD) -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UD)), // I I UD
      cbRespIs(UC)    -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UC)), // I I UC
      cbRespIs(SC)    -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UC)), // I I UC
      cbRespIs(I)     -> second(cdop("clean")         | wriSRC(false)), // I I I
    )),
    // V V I  -> I V I
    (srcHit  | othHit  | llcIs(I)) -> first(waitRecDone, cbRespIsCompAck, wriSRC(false)),
  ))


  // writeNoSnpPtl ++ writeUniquePtl ++ writeBackFull ++ writeCleanFull ++ writeEvictOrEvict
  def table: Seq[DecodeType] = Seq(writeNoSnpPtl_ewa, writeNoSnpPtl_noEwa, writeUniquePtl_alloc, writeUniquePtl_noAlloc, writeEvictOrEvict, writeBackFull)
}