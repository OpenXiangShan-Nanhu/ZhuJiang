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

object Write_LAN {
  // WriteNoSnpPtl With EWA
  def writeNoSnpPtl_ewa: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(WriteNoSnpPtl) | ewa | expCompAck | isOWO, Seq(
    (srcMiss | othMiss | llcIs(I)) -> (tdop("reqs") | receive(CompDBIDResp), Seq(
      (datIs(NonCopyBackWriteData) | respIs(I)) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), noCmt)
    ))
  ))

  // WriteNoSnpPtl Without EWA
  def writeNoSnpPtl_noEwa: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(WriteNoSnpPtl) | expCompAck | isOWO, Seq(
    (srcMiss | othMiss | llcIs(I)) -> (tdop("reqs") | receive(DBIDResp), Seq(
      (datIs(NonCopyBackWriteData) | respIs(I)) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp))
    ))
  ))

  // WriteUniquePtl With Allocate
  def writeUniquePtl_alloc: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(WriteUniquePtl) | allocate | ewa | isOWO, Seq(
    // I I I  -> I I UD
    (sfMiss | llcIs(I))   -> (tdop("reqs") | receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> second(cdop("save", "clean") | cmtRsp(Comp) | wriLLC(UD)))),
    // I I SC -> I I UD
    (sfMiss | llcIs(SC))  -> (tdop("reqs") | receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> second(cdop("save", "clean") | cmtRsp(Comp) | wriLLC(UD)))),
    // I I UC -> I I UD
    (sfMiss | llcIs(UC))  -> (tdop("reqs") | receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> second(cdop("save", "clean") | cmtRsp(Comp)  | wriLLC(UD)))),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> (tdop("reqs") | receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> second(cdop("save", "clean") | cmtRsp(Comp)  | wriLLC(UD)))),
    // I V I
    // TODO: reqs twice because dataVec = 2
    // TODO: dataBuffer auto merge data
    (srcMiss | othHit | llcIs(I)) -> (tdop("reqs") | snpOth(SnpUnique) | retToSrc, Seq(
      (datIs(SnpRespData) | respIs(I_PD)) -> (receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> (cdop("save", "clean") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UD)))), // I I UD
      (datIs(SnpRespData) | respIs(I))    -> (receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> (cdop("save", "clean") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UD)))), // I I UD
      (rspIs(SnpResp)     | respIs(I))    -> (receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> (cdop("save", "clean") | cmtRsp(Comp) | wriSNP(false) | wriLLC(UD)))), // I I UD
    ))
  ))

  // WriteUnique Without Allocate
  // TODO: receive can send dataTask
  def writeUniquePtl_noAlloc: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(WriteUniquePtl) | ewa | isOWO, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> (tdop("reqs") | receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp)))),
    // I I SC -> I I UD
    (sfMiss | llcIs(SC))  -> (tdop("reqs") | receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp) | wriLLC(I)))),
    // I I UC -> I I UD
    (sfMiss | llcIs(UC))  -> (tdop("reqs") | receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp) | wriLLC(I)))),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> (tdop("reqs") | receive(DBIDResp), Seq((datIs(NonCopyBackWriteData) | respIs(I)) -> second(tdop("send", "clean") | wriOrAtm(WriteNoSnpPtl), waitSecDone | cmtRsp(Comp) | wriLLC(I)))),
    // I V I
    // TODO: decode 3 times
  ))


  // WriteBackFull
  def writeBackFull: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(WriteBackFull) | allocate | ewa | noOrder, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))   -> (receive(CompDBIDResp), Seq((datIs(CopyBackWriteData) | respIs(I)) -> second(noCmt))),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC))  -> (receive(CompDBIDResp), Seq((datIs(CopyBackWriteData) | respIs(I)) -> second(noCmt))),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC))  -> (receive(CompDBIDResp), Seq((datIs(CopyBackWriteData) | respIs(I)) -> second(noCmt))),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD))  -> (receive(CompDBIDResp), Seq((datIs(CopyBackWriteData) | respIs(I)) -> second(noCmt))),
    // I V I  -> I V I
    (srcMiss | othHit | llcIs(I)) -> (receive(CompDBIDResp), Seq((datIs(CopyBackWriteData) | respIs(I)) -> second(noCmt))),
    // V I I
    (srcHit | othMiss | llcIs(I)) -> (tdop("reqs") | receive(CompDBIDResp), Seq(
      (datIs(CopyBackWriteData) | respIs(UD_PD))  -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UD)), // I I UD
      (datIs(CopyBackWriteData) | respIs(UC))     -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UC)), // I I UC
      (datIs(CopyBackWriteData) | respIs(SC))     -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UC)), // I I UC
      (datIs(CopyBackWriteData) | respIs(I))      -> second(cdop("clean") | wriSRC(false)), // I I I
    )),
    // V V I  -> I V I
    (srcHit | othHit | llcIs(I)) -> (receive(CompDBIDResp), Seq(
      (datIs(CopyBackWriteData) | respIs(SC))     -> second(wriSRC(false)),
      (datIs(CopyBackWriteData) | respIs(I))      -> second(wriSRC(false)),
    )),
  ))


  // WriteEvictOrEvict
  def writeEvictOrEvict: (UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))]) = (fromLAN | toLAN | reqIs(WriteEvictOrEvict) | expCompAck | allocate | ewa | noOrder, Seq(
    // I I I  -> I I I
    (sfMiss | llcIs(I))  -> (receive(Comp), Seq((rspIs(CompAck) | respIs(I)) -> second(noCmt))),
    // I I SC -> I I SC
    (sfMiss | llcIs(SC)) -> (receive(Comp), Seq((rspIs(CompAck) | respIs(I)) -> second(noCmt))),
    // I I UC -> I I UC
    (sfMiss | llcIs(UC)) -> (receive(Comp), Seq((rspIs(CompAck) | respIs(I)) -> second(noCmt))),
    // I I UD -> I I UD
    (sfMiss | llcIs(UD)) -> (receive(Comp), Seq((rspIs(CompAck) | respIs(I)) -> second(noCmt))),
    // I V I  -> I V I
    (srcMiss | othHit  | llcIs(I)) -> (receive(Comp), Seq((rspIs(CompAck) | respIs(I)) -> second(noCmt))),
    // V I I
    (srcHit  | othMiss | llcIs(I)) -> (tdop("reqs") | receive(CompDBIDResp), Seq(
      (datIs(CopyBackWriteData) | respIs(UD_PD))  -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UD)), // I I UD
      (datIs(CopyBackWriteData) | respIs(UC))     -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UC)), // I I UC
      (datIs(CopyBackWriteData) | respIs(SC))     -> second(cdop("save", "clean") | wriSRC(false) | wriLLC(UC)), // I I UC
      (datIs(CopyBackWriteData) | respIs(I))      -> second(cdop("clean") | wriSRC(false)), // I I I
    )),
    // V V I  -> I V I
    (srcHit  | othHit  | llcIs(I)) -> (receive(Comp), Seq((rspIs(CompAck) | respIs(I)) -> second(wriSRC(false)))),
  ))


  // writeNoSnpPtl ++ writeUniquePtl ++ writeBackFull ++ writeCleanFull ++ writeEvictOrEvict
  def table: Seq[(UInt, Seq[(UInt, (UInt, Seq[(UInt, (UInt, Seq[(UInt, UInt)]))]))])] = Seq(writeNoSnpPtl_ewa, writeNoSnpPtl_noEwa, writeUniquePtl_alloc, writeUniquePtl_noAlloc, writeEvictOrEvict, writeBackFull)
}