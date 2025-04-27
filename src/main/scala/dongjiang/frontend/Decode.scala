package dongjiang.frontend

import chisel3._
import chisel3.util._
import chisel3.experimental.BundleLiterals._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import zhujiang.chi.DatOpcode._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle._
import dongjiang.directory._
import xs.utils.debug._
import dongjiang.frontend.decode._
import dongjiang.data._
import xs.utils.ParallelLookUp

class Decode(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // Configuration
    val config      = new DJConfigIO()
    val task_s2     = Flipped(Valid(new PackChi with HasAddr with HasPackPosIndex with HasAlready))
    val respDir_s3  = Flipped(Valid(new PackDirMsg))
    val task_s3     = Valid(new PackChi with HasAddr with HasPackPosIndex with HasPackDirMsg with HasAlready with HasPackTaskCode with HasPackCmtCode)
    val updNest_s3  = Decoupled(new PackPosIndex with HasNest)
    val fastData_s3 = Decoupled(new DataTask)
  })

  dontTouch(io)

  /*
   * Reg and Wire declaration
   */
  val chiInst_s2          = Wire(new ChiInst())
  val validReg_s3         = RegNext(io.task_s2.valid)
  val taskReg_s3          = RegEnable(io.task_s2.bits, 0.U.asTypeOf(io.task_s2.bits), io.task_s2.valid)
  val stateInst_s3        = Wire(new StateInst())

  /*
   * [S2]: Pre-Decode
   */
  chiInst_s2              := io.task_s2.bits.chi.getChiInst(io.task_s2.valid)
  val result_s2           = Decode.decode(chiInst_s2)
  val stateInstVecReg_s3  = RegEnable(VecInit(result_s2._1), io.task_s2.valid)
  val taskCodeVecReg_s3   = RegEnable(VecInit(result_s2._2), io.task_s2.valid)
  val commitVecReg_s3     = RegEnable(VecInit(result_s2._3), io.task_s2.valid)
  dontTouch(chiInst_s2)

  /*
   * [S3]: Decode
   */
  val dirValid_s3 = io.respDir_s3.valid
  stateInst_s3    := Mux(dirValid_s3, io.respDir_s3.bits.dir.getStateInst(taskReg_s3.chi.metaIdOH, validReg_s3), (new StateInst).Lit(_.valid -> true.B))
  dontTouch(stateInst_s3)
  HardwareAssertion.withEn(io.respDir_s3.valid, validReg_s3 & taskReg_s3.chi.memAttr.cacheable)

  /*
   * [S3]: Decode
   */
  val code_s3 = ParallelLookUp(stateInst_s3.asUInt, stateInstVecReg_s3.map(_.asUInt).zip(taskCodeVecReg_s3))
  val cmt_s3  = ParallelLookUp(stateInst_s3.asUInt, stateInstVecReg_s3.map(_.asUInt).zip(commitVecReg_s3))
  dontTouch(code_s3)
  dontTouch(cmt_s3)
  var stateInstVecCf = cf""
  stateInstVecReg_s3.zipWithIndex.foreach { case (inst, i) => stateInstVecCf = stateInstVecCf + cf"[$i] -> [${inst.asTypeOf(new StateInst)}]\n" }
  HardwareAssertion.withEn(PopCount(Cat(stateInstVecReg_s3.map(_.asUInt === stateInst_s3.asUInt))) === 1.U, validReg_s3, cf"StateInst Invalid: $stateInst_s3 \nLegal StateInst:\n$stateInstVecReg_s3")
  HardwareAssertion.withEn(code_s3.valid | cmt_s3.valid, validReg_s3)

  /*
   * [S3]: Output S3
   */
  // task_s3
  io.task_s3.valid          := validReg_s3
  io.task_s3.bits.chi       := taskReg_s3.chi
  io.task_s3.bits.addr      := taskReg_s3.addr
  io.task_s3.bits.pos       := taskReg_s3.pos
  io.task_s3.bits.dir       := Mux(dirValid_s3, io.respDir_s3.bits.dir, 0.U.asTypeOf(io.respDir_s3.bits.dir))
  io.task_s3.bits.alr       := taskReg_s3.alr
  io.task_s3.bits.alr.sData := io.fastData_s3.fire
  io.task_s3.bits.code      := code_s3
  io.task_s3.bits.commit    := Mux(code_s3.valid, 0.U.asTypeOf(new CommitCode), cmt_s3)
  HardwareAssertion.withEn(code_s3.valid, validReg_s3 & cmt_s3.invalid)

  // canNest_s3
  io.updNest_s3.valid       := validReg_s3 & code_s3.canNest
  io.updNest_s3.bits.pos    := taskReg_s3.pos
  io.updNest_s3.bits.canNest:= true.B
  assert(io.updNest_s3.ready)


  // TODO: fastResp
  // fastData
  val respCompData_s3 = cmt_s3.commit & cmt_s3.channel === ChiChannel.DAT & cmt_s3.commitOp === CompData // TODO: SnpRespData
  // valid
  io.fastData_s3.valid   := validReg_s3 & !code_s3.valid & respCompData_s3
  HardwareAssertion.withEn(io.respDir_s3.bits.dir.llc.hit, io.fastData_s3.valid)
  // txDat
  io.fastData_s3.bits               := DontCare
  io.fastData_s3.bits.txDat.DBID    := taskReg_s3.pos.getLLCTxnID(dirBank)
  io.fastData_s3.bits.txDat.Resp    := cmt_s3.resp
  io.fastData_s3.bits.txDat.Opcode  := CompData
  io.fastData_s3.bits.txDat.HomeNID := DontCare // remap in SAM
  io.fastData_s3.bits.txDat.TxnID   := taskReg_s3.chi.txnID
  io.fastData_s3.bits.txDat.SrcID   := taskReg_s3.chi.getNoC(io.config.ci)
  io.fastData_s3.bits.txDat.TgtID   := taskReg_s3.chi.nodeId
  // other bits
  io.fastData_s3.bits.dataOp.reqs   := true.B
  io.fastData_s3.bits.dataOp.read   := true.B
  io.fastData_s3.bits.dataOp.send   := true.B
  io.fastData_s3.bits.dataVec       := taskReg_s3.chi.dataVec
  io.fastData_s3.bits.ds.bank       := getDS(taskReg_s3.addr, io.respDir_s3.bits.dir.llc.way)._1
  io.fastData_s3.bits.ds.idx        := getDS(taskReg_s3.addr, io.respDir_s3.bits.dir.llc.way)._2
  io.fastData_s3.bits.llcTxnID.pos  := taskReg_s3.pos
  io.fastData_s3.bits.llcTxnID.dirBank := dirBank.U
  HardwareAssertion.withEn(cmt_s3.dataOp.reqs & cmt_s3.dataOp.read & cmt_s3.dataOp.send & cmt_s3.dataOp.clean, io.fastData_s3.valid)


  /*
   * HardwareAssertion placePipe
   */
   HardwareAssertion.placePipe(Int.MaxValue-2)
}