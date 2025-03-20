package dongjiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang.bundle._
import dongjiang.utils._

// TODO: modify it
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
      val in      = Flipped(Decoupled(new ReqFlit(true)))
      val outVec  = Vec(nrIcn, Decoupled(new ReqFlit(true)))
    }
    // txSnp
    val txSnp = new Bundle {
      val in      = Flipped(Decoupled(new SnoopFlit()))
      val outVec  = Vec(nrIcn, Decoupled(new SnoopFlit()))
    }
    // txRsp
    val txRsp = new Bundle {
      val in      = Flipped(Decoupled(new RespFlit()))
      val outVec  = Vec(nrIcn, Decoupled(new RespFlit()))
    }
    // txDat
    val txDat = new Bundle {
      val in      = Flipped(Decoupled(new DataFlit()))
      val outVec  = Vec(nrIcn, Decoupled(new DataFlit()))
    }
    // cBusy
    val cBusy     = Input(UInt(3.W))
  })
  dontTouch(io)
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
  if(nrIcn == 1) {
    io.txReq.outVec.head <> io.txReq.in
    io.txSnp.outVec.head <> io.txSnp.in
    io.txRsp.outVec.head <> io.txRsp.in
    io.txDat.outVec.head <> io.txDat.in
  } else {
    // lan valid
    io.txReq.outVec.head.valid := io.txReq.in.valid & NocType.txIs(io.txReq.in.bits, LAN)
    io.txSnp.outVec.head.valid := io.txReq.in.valid & NocType.txIs(io.txSnp.in.bits, LAN)
    io.txRsp.outVec.head.valid := io.txReq.in.valid & NocType.txIs(io.txRsp.in.bits, LAN)
    io.txDat.outVec.head.valid := io.txReq.in.valid & NocType.txIs(io.txDat.in.bits, LAN)
    // bbn valid
    io.txReq.outVec.last.valid := io.txReq.in.valid & NocType.txIs(io.txReq.in.bits, BBN)
    io.txSnp.outVec.last.valid := io.txReq.in.valid & NocType.txIs(io.txSnp.in.bits, BBN)
    io.txRsp.outVec.last.valid := io.txReq.in.valid & NocType.txIs(io.txRsp.in.bits, BBN)
    io.txDat.outVec.last.valid := io.txReq.in.valid & NocType.txIs(io.txDat.in.bits, BBN)
    // ready
    io.txReq.in.ready := Mux(NocType.txIs(io.txReq.in.bits, LAN), io.txReq.outVec.head.ready, io.txReq.outVec.last.ready)
    io.txSnp.in.ready := Mux(NocType.txIs(io.txSnp.in.bits, LAN), io.txSnp.outVec.head.ready, io.txSnp.outVec.last.ready)
    io.txRsp.in.ready := Mux(NocType.txIs(io.txRsp.in.bits, LAN), io.txRsp.outVec.head.ready, io.txRsp.outVec.last.ready)
    io.txDat.in.ready := Mux(NocType.txIs(io.txDat.in.bits, LAN), io.txDat.outVec.head.ready, io.txDat.outVec.last.ready)
    // bits
    io.txReq.outVec.foreach(_.bits := io.txReq.in.bits)
    io.txSnp.outVec.foreach(_.bits := io.txSnp.in.bits)
    io.txRsp.outVec.foreach(_.bits := io.txRsp.in.bits)
    io.txDat.outVec.foreach(_.bits := io.txDat.in.bits)
  }

  // Set CBusy
  io.txRsp.outVec.foreach(_.bits.CBusy := io.cBusy)
  io.txDat.outVec.foreach(_.bits.CBusy := io.cBusy)
}
