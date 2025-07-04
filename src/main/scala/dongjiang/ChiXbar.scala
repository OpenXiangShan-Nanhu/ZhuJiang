package dongjiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import dongjiang.bundle._
import dongjiang.utils._
import xs.utils.debug.HAssert

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
    // rxReq
    val rxHpr = new Bundle {
      val inVec   = if(hasHPR) Some(Vec(nrIcn, Flipped(Decoupled(new ReqFlit(false))))) else None
      val outVec  = Vec(djparam.nrDirBank, Decoupled(new ReqFlit(false)))
    }
    // rxSnp
    val rxSnp = new Bundle {
      val in      = if(hasBBN) Some(Flipped(Decoupled(new SnoopFlit()))) else None
      val outVec  = if(hasBBN) Some(Vec(djparam.nrDirBank, Decoupled(new SnoopFlit()))) else None
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
  // Redirect network:
  //
  // in_0 ------> redirect_0_0 ------> | \
  //      \                            |r| ------> out_0
  //       \  --> redirect_1_0 ------> | /
  //        \/
  //        /\
  //       /  --> redirect_0_1 ------> | \
  //      /                            |r| ------> out_1
  // in_1 ------> redirect_1_1 ------> | /
  def rxRedir[T <: Bundle](inVec: Seq[DecoupledIO[T]], outVec: Seq[DecoupledIO[T]], inDirIdVec: Seq[UInt]): Unit = {
    val redirects = Seq.fill(inVec.size) { Seq.fill(outVec.size) { WireInit(0.U.asTypeOf(inVec(0))) } }
    // redirect
    redirects.zip(inVec).zipWithIndex.foreach {
      case((redirs, in), i) =>
        val readyVec = Wire(Vec(outVec.size, Bool()))
        redirs.zipWithIndex.foreach {
          case(redir, j) =>
            redir.valid := in.valid & inDirIdVec(i) === j.U
            redir.bits  := in.bits
            readyVec(j) := redir.ready
        }
        in.ready := readyVec(inDirIdVec(i))
    }
    // arbiter
    outVec.zip(redirects.transpose).foreach {
      case(out, redirs) =>
        out <> fastQosRRArb(redirs)
    }
  }

  // rxReq
  val reqRedirVec = Wire(chiselTypeOf(io.rxReq.outVec))
  rxRedir(io.rxReq.inVec, reqRedirVec, io.rxReq.inVec.map(in => getDirBank(in.bits.Addr)))

  // rxHpr
  val hprRedirVec = WireInit(0.U.asTypeOf(io.rxHpr.outVec))
  if(hasHPR) { rxRedir(io.rxHpr.inVec.get, hprRedirVec, io.rxHpr.inVec.get.map(in => getDirBank(in.bits.Addr))) }

  // rxSnp
  if(hasBBN) { rxRedir(Seq(io.rxSnp.in.get), io.rxSnp.outVec.get, Seq(getDirBank(Cat(io.rxSnp.in.map(_.bits.Addr).get, 0.U(3.W))))) }

  // Select high QoS network:
  //
  // reqReDir ---(QoS != 0xF)---> req.out
  //          \
  //     (QoS == 0xF)
  //            \
  //             ---> | \
  //                  | | ------> hpr.out
  // hprReDir ------> |*/
  io.rxReq.outVec.zip(io.rxHpr.outVec).zipWithIndex.foreach { case((req, hpr), i) =>
    // Req.QoS == 0xF
    when(reqRedirVec(i).bits.QoS === 0xF.U) {
      req.valid := false.B
      req.bits  := DontCare
      if(hasHPR) {
        hpr <> fastArb(Seq(hprRedirVec(i), reqRedirVec(i)))
      } else {
        hpr <> reqRedirVec(i)
      }
    // Req.QoS != 0xF
    }.otherwise {
      req <> reqRedirVec(i)
      if(hasHPR) {
        hpr <> hprRedirVec(i)
      } else {
        hpr.valid := false.B
        hpr.bits  := DontCare
      }
    }
    // HAssert
    HAssert.withEn(hpr.bits.QoS === 0xF.U, hpr.valid)
  }



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

  // clear NocType in SrcID
  io.txReq.outVec.foreach(_.bits.SrcID := 0.U)
  io.txSnp.outVec.foreach(_.bits.SrcID := 0.U)
  io.txRsp.outVec.foreach(_.bits.SrcID := 0.U)
  io.txDat.outVec.foreach(_.bits.SrcID := 0.U)

  // Set CBusy
  io.txRsp.outVec.foreach(_.bits.CBusy := io.cBusy)
  io.txDat.outVec.foreach(_.bits.CBusy := io.cBusy)
}
