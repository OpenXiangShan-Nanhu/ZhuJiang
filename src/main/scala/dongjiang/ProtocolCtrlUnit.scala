package dongjiang.pcu

import dongjiang._
import zhujiang.chi._
import xijiang.Node
import dongjiang.chi._
import dongjiang.pcu._
import dongjiang.pcu.exu._
import dongjiang.pcu.intf._
import chisel3._
import chisel3.experimental.hierarchy.{instantiable, public}
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.perf.{DebugOptions, DebugOptionsKey}
import dongjiang.utils.FastArb._
import xijiang.router.base.IcnBundle
import zhujiang.HasZJParams


/*
 * System Architecture: (2 RNSLAVE, 1 RNMASTER, 1 SNMASTER, 1 DataBuffer and 2 EXU)
 *
 *                                          -----------------------------------------------------------------
 *                                          |                       |       Dir       |                     |
 *               ------------               |      -----------      -------------------      ---------      |                ------------
 *     CSN <---> | RNSLAVE  | <---> | <---> | ---> |  MSHR   | ---> | ProcessPipe * 2 | ---> | Queue | ---> |  <---> | <---> | RNMASTER | <---> CSN
 *               ------------       |       |      -----------      -------------------      ---------      |        |       ------------
 *                                  |       |                                |                              |        |
 *                                  |       -----------------------------------------------------------------        |
 *                                  |                                        |                                       |
 *                                  |                                 --------------                                 |
 *                                 XBar <-------------------------->  | DataBuffer | <----------------------------> XBar
 *                                  |                                 --------------                                 |
 *                                  |                                        |                                       |
 *                                  |       -----------------------------------------------------------------        |
 *                                  |       |                                |                              |        |
 *               ------------       |       |      -----------      -------------------      ---------      |        |       ------------
 *   Local <---> | RNSLAVE  | <---> | <---> | ---> |  MSHR   | ---> | ProcessPipe * 2 | ---> | Queue | ---> |  <---> | <---> | SNMASTER | <---> Local
 *               ------------               |      -----------      -------------------      ---------      |                ------------
 *                                          |                       |       Dir       |                     |
 *                                          -----------------------------------------------------------------
 */

@instantiable
class ProtocolCtrlUnit(localHf: Node, csnRf: Option[Node] = None, csnHf: Option[Node] = None, hasReset: Boolean = true)(implicit p: Parameters) extends DJRawModule
  with ImplicitClock with ImplicitReset {
  // ------------------------------------------ IO declaration ----------------------------------------------//
  @public val io  = IO(new Bundle {
    val hnfID     = Input(UInt(chiNodeIdBits.W))
    val toLocal   = Flipped(new IcnBundle(localHf, hasReset)) //TODO:Use DeviceIcnBundle
    val toCSNOpt  = if(hasCSNIntf) Some(new Bundle {
      val hn      = Flipped(new IcnBundle(csnHf.get))
      val rn      = Flipped(new IcnBundle(csnRf.get))
    }) else None
  })
  @public val reset = IO(Input(AsyncReset()))
  @public val clock = IO(Input(Clock()))
  val implicitClock = clock
  val implicitReset = reset

// ------------------------------------------ Modules declaration ----------------------------------------------//

    val localRnSlave    = Module(new RnSlaveIntf(IncoID.LOCALSLV, djparam.localRnSlaveIntf))
    val localSnMaster   = Module(new SnMasterIntf(IncoID.LOCALMAS, djparam.localSnMasterIntf))
    val csnRnSlaveOpt   = if (hasCSNIntf) Some(Module(new RnSlaveIntf(IncoID.CSNSLV, djparam.csnRnSlaveIntf.get))) else None
    val csnRnMasterOpt  = if (hasCSNIntf) Some(Module(new RnMasterIntf(IncoID.CSNMAS, djparam.csnRnMasterIntf.get))) else None
    val intfs           = if (hasCSNIntf) Seq(localRnSlave, localSnMaster, csnRnSlaveOpt.get, csnRnMasterOpt.get)
                          else            Seq(localRnSlave, localSnMaster)
    val databuffer      = Module(new DataBuffer())

    val xbar            = Module(new Xbar())
    val exus            = Seq.fill(nrBankPerDJ) { Module(new ExecuteUnit()) }
    exus.zipWithIndex.foreach { case(s, i) => s.io.sliceId := i.U }

    // TODO:
    exus.foreach(_.io.valid := true.B)


// ---------------------------------------------- Connection ---------------------------------------------------//
    /*
     * Connect LOCAL RING CHI IO
     * TODO: opposite direction
     */
    // tx req
    io.toLocal.tx.req.get   <> localRnSlave.io.chi.txreq
    // tx rsp & dat
    val rspQ = Module(new Queue(new RespFlit(), entries = 2))
    val datQ = Module(new Queue(new DataFlit(), entries = 2))
    io.toLocal.tx.resp.get  <> rspQ.io.enq
    io.toLocal.tx.data.get  <> datQ.io.enq
    // tx rsp
    localSnMaster.io.chi.rxrsp.valid    := rspQ.io.deq.valid & fromSnNode(rspQ.io.deq.bits.SrcID)
    localRnSlave.io.chi.txrsp.valid     := rspQ.io.deq.valid & !fromSnNode(rspQ.io.deq.bits.SrcID)
    localSnMaster.io.chi.rxrsp.bits     := rspQ.io.deq.bits
    localRnSlave.io.chi.txrsp.bits      := rspQ.io.deq.bits
    when(fromSnNode(rspQ.io.deq.bits.SrcID)) { rspQ.io.deq.ready := localSnMaster.io.chi.rxrsp.ready }.otherwise { rspQ.io.deq.ready := localRnSlave.io.chi.txrsp.ready }
    // tx dat
    localSnMaster.io.chi.rxdat.valid    := datQ.io.deq.valid & fromSnNode(datQ.io.deq.bits.SrcID)
    localRnSlave.io.chi.txdat.valid     := datQ.io.deq.valid & !fromSnNode(datQ.io.deq.bits.SrcID)
    localSnMaster.io.chi.rxdat.bits     := datQ.io.deq.bits
    localRnSlave.io.chi.txdat.bits      := datQ.io.deq.bits
    when(fromSnNode(datQ.io.deq.bits.SrcID)) { datQ.io.deq.ready := localSnMaster.io.chi.rxdat.ready }.otherwise { datQ.io.deq.ready := localRnSlave.io.chi.txdat.ready }
    // rx ereq & snp
    localSnMaster.io.chi.txreq          <> io.toLocal.rx.req.get
    localRnSlave.io.chi.rxsnp           <> io.toLocal.rx.snoop.get
    localSnMaster.io.chi.rxsnp          <> DontCare
    // rx rsp & dat
    fastArbDec(Seq(localSnMaster.io.chi.txrsp, localRnSlave.io.chi.rxrsp)) <> io.toLocal.rx.resp.get
    fastArbDec(Seq(localSnMaster.io.chi.txdat, localRnSlave.io.chi.rxdat)) <> io.toLocal.rx.data.get

    /*
     * Connect CSN CHI IO
     */
    if(hasCSNIntf) {
        // tx
        io.toCSNOpt.get.hn.tx.req.get   <> csnRnSlaveOpt.get.io.chi.txreq
        io.toCSNOpt.get.hn.tx.resp.get  <> csnRnSlaveOpt.get.io.chi.txrsp
        io.toCSNOpt.get.hn.tx.data.get  <> csnRnSlaveOpt.get.io.chi.txdat
        // rx
        csnRnSlaveOpt.get.io.chi.rxsnp  <> io.toCSNOpt.get.hn.rx.snoop.get
        csnRnSlaveOpt.get.io.chi.rxrsp  <> io.toCSNOpt.get.hn.rx.resp.get
        csnRnSlaveOpt.get.io.chi.rxdat  <> io.toCSNOpt.get.hn.rx.data.get
        // tx
        csnRnMasterOpt.get.io.chi.txreq <> io.toCSNOpt.get.rn.rx.req.get
        csnRnMasterOpt.get.io.chi.txrsp <> io.toCSNOpt.get.rn.rx.resp.get
        csnRnMasterOpt.get.io.chi.txdat <> io.toCSNOpt.get.rn.rx.data.get
        // rx
        io.toCSNOpt.get.rn.tx.snoop.get <> csnRnMasterOpt.get.io.chi.rxsnp
        io.toCSNOpt.get.rn.tx.resp.get  <> csnRnMasterOpt.get.io.chi.rxrsp
        io.toCSNOpt.get.rn.tx.data.get  <> csnRnMasterOpt.get.io.chi.rxdat
    }

    /*
     * Connect RNs <-> Xbar
     */

    intfs.zipWithIndex.foreach {
        case(intf, i) =>
            intf.io.hnfID                           := io.hnfID
            // slice ctrl signals
            if (intf.io.req2SliceOpt.nonEmpty) {
                xbar.io.req2Slice.in(i)             <> intf.io.req2SliceOpt.get
            } else {
                xbar.io.req2Slice.in(i)             <> DontCare
            }
            if (intf.io.reqAck2NodeOpt.nonEmpty) {
                xbar.io.reqAck2Node.out(i)          <> intf.io.reqAck2NodeOpt.get
            } else {
                xbar.io.reqAck2Node.out(i)          <> DontCare
            }
            if (intf.io.resp2NodeOpt.nonEmpty) {
                xbar.io.resp2Node.out(i)            <> intf.io.resp2NodeOpt.get
            } else {
                xbar.io.resp2Node.out(i)            <> DontCare
            }
            xbar.io.req2Node.out(i)                 <> intf.io.req2Node
            xbar.io.resp2Slice.in(i)                <> intf.io.resp2Slice
            // slice DataBuffer signals
            if (intf.io.dbSigs.dbRCReqOpt.nonEmpty) {
                xbar.io.dbSigs.in0(i)               <> intf.io.dbSigs.dbRCReqOpt.get
            } else {
                xbar.io.dbSigs.in0(i)               <> DontCare
            }
            xbar.io.dbSigs.in1(i).wReq              <> intf.io.dbSigs.wReq
            xbar.io.dbSigs.in1(i).wResp             <> intf.io.dbSigs.wResp
            xbar.io.dbSigs.in1(i).dataFDB           <> intf.io.dbSigs.dataFDB
            xbar.io.dbSigs.in1(i).dataTDB           <> intf.io.dbSigs.dataTDB
    }

    /*
     * Connect EXUs <-> Xbar
     */
    exus.zipWithIndex.foreach {
        case (exu, i) =>
            exu.io.hnfID                    := io.hnfID
            // slice ctrl signals
            xbar.io.req2Slice.out(i)        <> exu.io.req2Slice
            xbar.io.reqAck2Node.in(i)       <> exu.io.reqAck2Node
            xbar.io.resp2Node.in(i)         <> exu.io.resp2Node
            xbar.io.req2Node.in(i)          <> exu.io.req2Node
            xbar.io.resp2Slice.out(i)       <> exu.io.resp2Slice
            // slice DataBuffer signals
            xbar.io.dbSigs.in0(i+nrIntf)    <> exu.io.dbRCReq
    }

    /*
     * Connect DataBuffer
     */
    databuffer.io <> xbar.io.dbSigs.out(0)
}
