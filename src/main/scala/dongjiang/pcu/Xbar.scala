package dongjiang.pcu

import dongjiang._
import dongjiang.utils.FastArb
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang.utils.FastArb._
import xs.utils.FastArbiter

// TODO: has some problem of XBar, req never enter slice_1

class IdMap(implicit p: Parameters) extends DJModule {
    val io = IO(new Bundle {
        val inBank      = Input(UInt(bankBits.W))
        val outBank     = Output(UInt(bankBits.W))
    })
    val outBank     = WireInit(0.U(bankBits.W))

    if (djparam.nrBank == 4) {
        // Use Bank [0]
        if(nrBankPerDJ == 1)      { outBank :=  0.U }
        // Use Bank [0 1]
        else if(nrBankPerDJ == 2) { outBank := io.inBank(0) | io.inBank(1) }
        // Use Bank [0 1 2 3]
        else if(nrBankPerDJ == 4) { outBank := io.inBank }
    } else if (djparam.nrBank == 2) {
        // Use Bank [0]
        if(nrBankPerDJ == 1)      { outBank :=  0.U }
        // Use Bank [0 1]
        else if(nrBankPerDJ == 2) { outBank := io.inBank(0) }
    } else {
        // Use Bank [0]
        outBank === 0.U
    }
    io.outBank := outBank
    require(nrBankPerDJ == 4 | nrBankPerDJ == 2 | nrBankPerDJ == 1)
    require(nrBankPerDJ <= djparam.nrBank)
}




class Xbar()(implicit p: Parameters) extends DJModule {
// ------------------------------------------ IO declaration ----------------------------------------------//
    val io = IO(new Bundle {
        // slice ctrl signals
        val req2Slice = new Bundle {
            val in = Vec(nrIntf, Flipped(Decoupled(new Req2SliceBundle()))) // expect SNMaster
            val out = Vec(nrBankPerDJ, Decoupled(new Req2SliceBundle()))
        }
        val reqAck2Node = new Bundle {
            val in = Vec(nrBankPerDJ, Flipped(Decoupled(new ReqAck2NodeBundle()))) // expect SNMaster
            val out = Vec(nrIntf, Decoupled(new ReqAck2NodeBundle()))
        }
        val resp2Node = new Bundle {
            val in = Vec(nrBankPerDJ, Flipped(Decoupled(new Resp2NodeBundle())))
            val out = Vec(nrIntf, Decoupled(new Resp2NodeBundle()))
        }
        val req2Node = new Bundle {
            val in = Vec(nrBankPerDJ, Flipped(Decoupled(new Req2NodeBundle())))
            val out = Vec(nrIntf, Decoupled(new Req2NodeBundle()))
        }
        val resp2Slice = new Bundle {
            val in = Vec(nrIntf, Flipped(Decoupled(new Resp2SliceBundle())))
            val out = Vec(nrBankPerDJ, Decoupled(new Resp2SliceBundle()))
        }
        // slice DataBuffer signals
        val dbSigs = new Bundle {
            val in0 = Vec(nrBankPerDJ + nrIntf, Flipped(Decoupled(new DBRCReq())))
            val in1 = Vec(nrIntf, Flipped(new DBBundle(hasDBRCReq = false)))
            val out = Vec(1, new DBBundle(hasDBRCReq = true))
        }
    })


// ------------------------------------------ Modules declaration And Connection ----------------------------------------------//
    val req2SliceIdMaps     = Seq.fill(nrIntf) { Module(new IdMap()) }

    // --------------------- Wire declaration ------------------------//
    val req2SliceReMap      = Wire(Vec(nrIntf, Decoupled(new Req2SliceBundle())))

    // --------------------- Connection ------------------------//
    // req2Slice bank ReMap
    req2SliceReMap.zip(io.req2Slice.in).foreach{ case(reMap, in) => reMap <> in }
    req2SliceIdMaps.zipWithIndex.foreach {
        case(m, i) =>
            m.io.inBank := io.req2Slice.in(i).bits.to.IncoId
            req2SliceReMap(i).bits.to.IncoId := m.io.outBank
    }

    def idSelDec2DecVec[T <: Bundle with HasToIncoID](in: DecoupledIO[T], out: Seq[DecoupledIO[T]]): Unit = {
        in.ready := false.B
        out.foreach(_.bits := in.bits)
        out.zipWithIndex.foreach {
            case (o, i) =>
                o.bits := in.bits
                val idMatch = WireInit(false.B)
                idMatch := in.bits.to.IncoId === i.U
                when(idMatch) {
                    o.valid := in.valid
                    in.ready := o.ready
                }.otherwise {
                    o.valid := false.B
                }
        }
    }

    // in ---> [queue0] ---> [idIndex] ---> [queue1] ---> [arbiter] ---> [queue2] ---> out
    def interConnect[T <: Bundle with HasToIncoID](in: Seq[DecoupledIO[T]], q0: Int, q1: Int, q2: Int, out: Seq[DecoupledIO[T]]): Unit = {
        val reDir = Seq.fill(in.size) { Seq.fill(out.size) { WireInit(0.U.asTypeOf(in(0))) }}
        in.zipWithIndex.foreach { case (m, i) => idSelDec2DecVec(Queue(m, q0), reDir(i)) }
        out.zipWithIndex.foreach { case (m, i) => m <> Queue(fastArbDec(reDir.map { case a => Queue(a(i), entries = q1) }), q2) }
    }

    // There is a lot of room for optimization of the connection
    interConnect(in = req2SliceReMap,                       q0 = 0, q1 = 0, q2 = 0, out = io.req2Slice.out)

    interConnect(in = io.reqAck2Node.in,                    q0 = 0, q1 = 0, q2 = 0, out = io.reqAck2Node.out)

    interConnect(in = io.resp2Node.in,                      q0 = 0, q1 = 0, q2 = 0, out = io.resp2Node.out)

    interConnect(in = io.req2Node.in,                       q0 = 0, q1 = 0, q2 = 0, out = io.req2Node.out)

    interConnect(in = io.resp2Slice.in,                     q0 = 0, q1 = 0, q2 = 0, out = io.resp2Slice.out)

    io.dbSigs.out(0).dbRCReq                                <> fastArbDec(io.dbSigs.in0)

    io.dbSigs.out(0).wReq                                   <> fastArbDec(io.dbSigs.in1.map(_.wReq))

    interConnect(in = io.dbSigs.out.map(_.wResp),           q0 = 0, q1 = 0, q2 = 0, out = io.dbSigs.in1.map(_.wResp))

    interConnect(in = io.dbSigs.out.map(_.dataFDB),         q0 = 0, q1 = 0, q2 = 0, out = io.dbSigs.in1.map(_.dataFDB))

    io.dbSigs.out(0).dataTDB                                <> fastArbDec(io.dbSigs.in1.map(_.dataTDB))

}