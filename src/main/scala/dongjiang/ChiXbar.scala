package dongjiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang.bundle._
import dongjiang.utils._

class ChiXbar(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    // rxReq
    val rxReq = new Bundle {
      val inVec   = Vec(nrIcn, Flipped(Decoupled(new ReqFlit(false))))
      val outVec  = Vec(djparam.nrDirBank, Decoupled(new ReqFlit(false)))
    }
    // rxSnp
    val rxSnp = new Bundle {
      val in      = Flipped(Decoupled(new SnoopFlit()))
      val outVec  = Vec(djparam.nrDirBank, Decoupled(new SnoopFlit()))
    }
    // txReq
    val txReq = new Bundle {
      val inVec   = Vec(nrIcn, Flipped(Decoupled(new ReqFlit(true))))
      val outVec  = Vec(nrIcn, Decoupled(new ReqFlit(true)))
    }
    // txSnp
    val txSnp = new Bundle {
      val inVec   = Vec(nrIcn, Flipped(Decoupled(new SnoopFlit())))
      val outVec  = Vec(nrIcn, Decoupled(new SnoopFlit()))
    }
    // txRsp
    val txRsp = new Bundle {
      val inVec   = Vec(nrIcn, Flipped(Decoupled(new RespFlit())))
      val outVec  = Vec(nrIcn, Decoupled(new RespFlit()))
    }
    // txDat
    val txDat = new Bundle {
      val inVec   = Vec(nrIcn, Flipped(Decoupled(new DataFlit())))
      val outVec  = Vec(nrIcn, Decoupled(new DataFlit()))
    }
    // cBusy
    val cBusy     = Input(UInt(3.W))
  })
  require(nrIcn <= 3)
  require(nrLanIcn <= 2)

  /*
   * Connect rxReq and rxSnp
   */
  /*
   * Redirect network:
   *
   * in_0 ------> redirect_0_0 ------> | \
   *      \                            | | ------> out_0
   *       \  --> redirect_1_0 ------> | /
   *        \/
   *        /\
   *       /  --> redirect_0_1 ------> | \
   *      /                            | | ------> out_1
   * in_1 ------> redirect_1_1 ------> | /
   */
  def rxRedir[T <: Bundle](inVec: Seq[DecoupledIO[T]], outVec: Seq[DecoupledIO[T]], inDirIdVec: Seq[UInt]): Unit = {
    val redirects = Seq.fill(inVec.size) { Seq.fill(outVec.size) { WireInit(0.U.asTypeOf(inVec(0))) } }
    inVec.foreach(_.ready := false.B)
    // redirect
    redirects.zipWithIndex.foreach {
      case(redirs, i) =>
        redirs.zip(inVec).zipWithIndex.foreach {
          case((redir, in), j) =>
            when(inDirIdVec(i) === j.U) {
              redir.valid := in.valid
              in.ready    := redir.ready
            }
            redir.bits  := in.bits
        }
    }
    // arbiter
    outVec.zip(redirects.transpose).foreach {
      case(out, redirs) =>
        out <> fastRRArb(redirs)
    }
  }

  // rxReq
  rxRedir(io.rxReq.inVec, io.rxReq.outVec, io.rxReq.inVec.map(in => getDirBank(in.bits.Addr)))

  // rxSnp
  rxRedir(Seq(io.rxSnp.in), io.rxSnp.outVec, Seq(getDirBank(Cat(io.rxSnp.in.bits.Addr, 0.U(3.W)))))

  /*
   * Connect txReq, txSnp, txRsp and txDat
   */
  // init
  io.txReq.inVec.zip(io.txReq.outVec).foreach { case(a, b) => a <> b }
  io.txSnp.inVec.zip(io.txSnp.outVec).foreach { case(a, b) => a <> b }
  io.txRsp.inVec.zip(io.txRsp.outVec).foreach { case(a, b) => a <> b }
  io.txDat.inVec.zip(io.txDat.outVec).foreach { case(a, b) => a <> b }
  /*
   * Select network: (*: Priority)
   * 1. in_0(!valid) & in_1(!valid):
   *    in_0 ------> out0
   *    in_1 ------> out1
   *
   * 2. in_0(valid) & in_1(!valid):
   *    in_0 -----> out_0(*)
   *         \
   *          ----> out_1
   *    in_1(DontCare)
   *
   * 3. in_0(valid) & in_1(!valid):
   *    in_0(DontCare)
   *          ----> out_0
   *         /
   *    in_1 -----> out_1(*)
   *
   * 4. in_0(valid) & in_1(valid):
   *    in_0 ------> out0
   *    in_1 ------> out1
   */
  def selectNetwork[T <: Bundle](inVec: Seq[DecoupledIO[T]], outVec: Seq[DecoupledIO[T]]): Unit = {
    require(inVec.size == 2 & outVec.size == 2, s"in.size = ${inVec.size} out.size = ${outVec.size}")
    val inOH    = Cat(inVec(1).valid, inVec(0).valid)
    val outOH0  = PriorityEncoderOH(Cat(outVec(1).ready, outVec(0).ready)).asBools
    val outOH1  = PriorityEncoderOH(Cat(outVec(0).ready, outVec(1).ready)).asBools
    switch(inOH) {
      is("b00".U) {
        inVec.zip(outVec).foreach { case(a, b) => a <> b }
      }
      is("b01".U) {
        inVec(0).ready  := outVec.map(_.ready).reduce(_ | _)
        outVec.zip(outOH0).foreach {
          case(out, valid) =>
            out.valid   := valid
            out.bits    := inVec(0).bits
        }
      }
      is("b10".U) {
        inVec(1).ready  := outVec.map(_.ready).reduce(_ | _)
        outVec.zip(outOH1).foreach {
          case (out, valid) =>
            out.valid   := valid
            out.bits    := inVec(1).bits
        }
      }
      is("b11".U) {
        inVec.zip(outVec).foreach { case(a, b) => a <> b }
      }
    }
  }

  // LAN 2to2 select network
  if(nrLanIcn == 2) {
    // tx req
    selectNetwork(io.txReq.inVec.take(2), io.txReq.outVec.take(2))
    // tx snp
    selectNetwork(io.txSnp.inVec.take(2), io.txSnp.outVec.take(2))
    // tx rsp
    selectNetwork(io.txRsp.inVec.take(2), io.txRsp.outVec.take(2))
    // tx dat
    selectNetwork(io.txDat.inVec.take(2), io.txDat.outVec.take(2))
  }

  // Set CBusy
  io.txRsp.outVec.foreach(_.bits.CBusy := io.cBusy)
  io.txDat.outVec.foreach(_.bits.CBusy := io.cBusy)
}
