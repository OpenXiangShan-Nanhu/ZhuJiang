package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle.{HasAddr, HasPackLLCTxnID, PackChi, _}
import dongjiang.data.HasAlready
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, HasPackDirMsg}
import dongjiang.data._
import zhujiang.chi.ReqOpcode._
import dongjiang.frontend._
import dongjiang.frontend.decode._

class CommitTask(implicit p: Parameters) extends DJBundle with HasPackChi with HasPackPosIndex
  with HasPackDirMsg with HasAlready with HasPackOperations with HasPackCmtCode with HasDsIdx with HasSnpTgt

class CMTask(implicit p: Parameters) extends DJBundle with HasPackChi with HasPackLLCTxnID
  with HasAlready with HasPackDataOp {
  val snpVec    = Vec(nrSfMetas, Bool())  // Only use in SnoopCM
  val wrillcWay = UInt(llcWayBits.W)      // Only use in WriOrAtmCm
  val fromRepl  = Bool()                  // from ReplaceCM
  val cbResp    = UInt(ChiResp.width.W)   // CopyBack Resp //  Only use in WriOrAtmCm
  val doDMT     = Bool()                  // Only use in ReadCM
  // def
  def needReqDB = dataOp.reqs & !alr.reqs
}

class PackCMTask(implicit p: Parameters) extends DJBundle { val task = new CMTask() }

class RespToCmt(implicit p: Parameters) extends DJBundle with HasPackLLCTxnID with HasPackTaskInst with HasAlready

class ReplTask(implicit p: Parameters) extends DJBundle with HasAlready with HasPackDirMsg with HasPackLLCTxnID {
  val wriSF   = Bool()
  val wriLLC  = Bool()
  def replSF  = wriSF  & !dir.sf.hit
  def replLLC = wriLLC & !dir.llc.hit
  def replDIR = replSF | replLLC
}

class GetAddr(frontend: Boolean = false)(implicit p: Parameters) extends DJBundle {
  val llcTxnIdOpt = if(!frontend) Some(Output(new LLCTxnID())) else None
  val posOpt      = if(frontend)  Some(Output(new PosIndex())) else None
  val result      = Input(new Addr())
  def llcTxnID    = llcTxnIdOpt.get
  def pos         = posOpt.get
}