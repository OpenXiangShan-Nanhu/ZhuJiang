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
import xijiang.router.base.DeviceIcnBundle
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
 *
 *
 * * System Architecture: (1 RNSLAVE, 1 SNMASTER, 1 DataBuffer and 2 EXU)
 *
 *                                          -----------------------------------------------------------------
 *                                          |                       |       Dir       |                     |
 *               ------------               |      -----------      -------------------      ---------      |
 *   Local <---> | RNSLAVE  | <---> | <---> | ---> |  MSHR   | ---> | ProcessPipe * 2 | ---> | Queue | ---> |
 *               ------------       |       |      -----------      -------------------      ---------      |
 *                                  |       |                                |                              |
 *                                  |       -----------------------------------------------------------------
 *                                  |                                        |
 *                                  |                                 --------------
 *                                 XBar <-------------------------->  | DataBuffer |
 *                                  |                                 --------------
 *                                  |                                        |
 *                                  |       -----------------------------------------------------------------
 *                                  |       |                                |                              |
 *               ------------       |       |      -----------      -------------------      ---------      |
 *   Local <---> | SNMASTER | <---> | <---> | ---> |  MSHR   | ---> | ProcessPipe * 2 | ---> | Queue | ---> |
 *               ------------               |      -----------      -------------------      ---------      |
 *                                          |                       |       Dir       |                     |
 *                                          -----------------------------------------------------------------
 *
 *
 */

// TODO: update XSPerfAccumulate

@instantiable
class ProtocolCtrlUnit(localHf: Node, csnRf: Option[Node] = None, csnHf: Option[Node] = None)(implicit p: Parameters) extends DJRawModule
  with ImplicitClock with ImplicitReset {
  // ------------------------------------------ IO declaration ----------------------------------------------//
  @public val io  = IO(new Bundle {
    val hnfID         = Input(UInt(fullNodeIdBits.W))
    val pcuID         = Input(UInt(pcuBankBits.W)) // PCU Bank ID
    val dcuNodeIDVec  = Input(Vec(nrBankPerPCU, UInt(fullNodeIdBits.W))) // DCU Friend Node ID Vec
    val toLocal       = new DeviceIcnBundle(localHf)
    val toCSNOpt      = if(hasCSN) Some(new Bundle {
      val hn          = new DeviceIcnBundle(csnHf.get)
      val rn          = new DeviceIcnBundle(csnRf.get)
    }) else None
  })
  @public val reset   = IO(Input(AsyncReset()))
  @public val clock   = IO(Input(Clock()))
  val implicitClock   = clock
  val implicitReset   = reset

  /*
   * For Debug
   */
  if (p(DebugOptionsKey).EnableDebug) {
    dontTouch(io)
  }

  lazy val cacheable_hi = fullAddrBits - 1
  lazy val cacheable_lo = fullAddrBits - cacheableBits

  lazy val ccxChipID_hi = cacheable_lo - 1
  lazy val ccxChipID_lo = cacheable_lo - ccxChipBits

  lazy val useAddr_hi   = ccxChipID_lo - 1
  lazy val useAddr_lo   = offsetBits

  lazy val bankID_hi    = bankOff + fullBankBits - 1
  lazy val bankID_lo    = bankOff

  lazy val offset_hi    = offsetBits - 1
  lazy val offset_lo    = 0

  lazy val dirBank_hi   = useAddr_lo + dirBankBits - 1
  lazy val dirBank_lo   = useAddr_lo

  lazy val selfTag_hi   = useAddr_hi
  lazy val selfTag_lo   = useAddr_hi - sTagBits + 1

  lazy val selfSet_hi   = selfTag_lo - 1
  lazy val selfSet_lo   = dirBank_hi + 1

  lazy val sfTag_hi     = useAddr_hi
  lazy val sfTag_lo     = useAddr_hi - sfTagBits + 1

  lazy val sfSet_hi     = sfTag_lo - 1
  lazy val sfSet_lo     = dirBank_hi + 1


  print(
    s"""
       |DongJiang PCU Message: {
       |  Support Protocol: CHI-G
       |  fullAddr:   [${fullAddrBits-1}:0] = [cacheable] + [ccxChipID] + [useAddr1] + [bankID] + [useAddr0] + [offset] (Note: The useAddr = Cat(useAddr1, useAddr0))
       |  fullAddr:   [${fullAddrBits-1}:0] = [${cacheable_hi}:${cacheable_lo}] + [${ccxChipID_hi}:${ccxChipID_lo}] + [[${useAddr_hi}:${bankID_hi+1}] + [${bankID_hi}:${bankID_lo}] + [${bankID_lo-1}:${useAddr_lo}]] + [${offset_hi}:${offset_lo}]
       |  useAddr*:   [${useAddr_hi}:${useAddr_lo}] = [selfTag] + [selfSet] + [dirBank] / [sfTag] + [sfSet] + [dirBank] (Note: The bankID[${bankID_hi}:${bankID_lo}] need to be removed in useAddr*)
       |  useAddr*:   [${useAddr_hi}:${useAddr_lo}] = [${selfTag_hi}:${selfTag_lo}] + [${selfSet_hi}:${selfSet_lo}] + [${dirBank_hi}:${dirBank_lo}] / [${sfTag_hi}:${sfTag_lo}] + [${sfSet_hi}:${sfSet_lo}] + [${dirBank_hi}:${dirBank_lo}]
       |  selfWays:   ${djparam.selfWays}
       |  selfSets:   ${djparam.selfSets}
       |  sfWays:     ${djparam.sfDirWays}
       |  sfSets:     ${djparam.sfDirSets}
       |  nrDirBank:  ${djparam.nrDirBank}
       |  directory: setup = ${djparam.dirSetup} latency = ${djparam.dirLatency} extraHold = ${djparam.dirExtraHold}
       |  replacementPolicy: ${djparam.selfReplacementPolicy}
       |}
       |""".stripMargin)

// ------------------------------------------ Modules declaration ----------------------------------------------//
  // interfaces
  val localRnSlave    = Module(new RnSlaveIntf(djparam.localRnSlaveIntf, localHf))
  val localSnMaster   = Module(new SnMasterIntf(djparam.localSnMasterIntf, localHf))
  val csnRnSlaveOpt   = if (hasCSN) Some(Module(new RnSlaveIntf(djparam.csnRnSlaveIntf.get,csnHf.get))) else None
  val csnRnMasterOpt  = if (hasCSN) Some(Module(new RnMasterIntf(djparam.csnRnMasterIntf.get, csnRf.get))) else None
  val intfs           = if (hasCSN) Seq(localRnSlave, localSnMaster, csnRnSlaveOpt.get, csnRnMasterOpt.get)
                        else        Seq(localRnSlave, localSnMaster)
  // data buffer
  val databuffer      = Module(new DataBuffer())
  // xbar
  val xbar            = Module(new Xbar())
  // EXUs
  val exus            = Seq.fill(nrBankPerPCU) { Module(new ExecuteUnit()) }

  
// ---------------------------------------------- Connection ---------------------------------------------------//
  /*
   * Connect LOCAL RING CHI IO
   */
  localSnMaster.io.chi                    <> DontCare
  localRnSlave.io.chi                     <> DontCare

  // rx req
  localRnSlave.io.chi.rx.req.get          <> Queue(io.toLocal.rx.req.get, 2) // Adding queues for timing considerations
  
  // rx rsp
  val rxRespQ                             = Module(new Queue(new RespFlit(), 2)) // Adding queues for timing considerations
  rxRespQ.io.enq                          <> io.toLocal.rx.resp.get
  localSnMaster.io.chi.rx.resp.get.valid  := rxRespQ.io.deq.valid & fromSnNode(rxRespQ.io.deq.bits.asTypeOf(new RespFlit()).SrcID)
  localRnSlave.io.chi.rx.resp.get.valid   := rxRespQ.io.deq.valid & fromCcNode(rxRespQ.io.deq.bits.asTypeOf(new RespFlit()).SrcID)
  localSnMaster.io.chi.rx.resp.get.bits   := rxRespQ.io.deq.bits
  localRnSlave.io.chi.rx.resp.get.bits    := rxRespQ.io.deq.bits
  rxRespQ.io.deq.ready                    := fromSnNode(rxRespQ.io.deq.bits.asTypeOf(new RespFlit()).SrcID) | localRnSlave.io.chi.rx.resp.get.ready
  // assert
  assert(Mux(rxRespQ.io.deq.valid & fromSnNode(rxRespQ.io.deq.bits.asTypeOf(new RespFlit()).SrcID), localSnMaster.io.chi.rx.resp.get.ready, true.B))



  // rx data
  val rxDataQ                             = Module(new Queue(new DataFlit(), 2)) // Adding queues for timing considerations
  rxDataQ.io.enq                          <> io.toLocal.rx.data.get
  localSnMaster.io.chi.rx.data.get.valid  := rxDataQ.io.deq.valid & fromSnNode(rxDataQ.io.deq.bits.asTypeOf(new DataFlit()).SrcID)
  localRnSlave.io.chi.rx.data.get.valid   := rxDataQ.io.deq.valid & fromRnNode(rxDataQ.io.deq.bits.asTypeOf(new DataFlit()).SrcID)
  localSnMaster.io.chi.rx.data.get.bits   := rxDataQ.io.deq.bits
  localRnSlave.io.chi.rx.data.get.bits    := rxDataQ.io.deq.bits
  // assert
  assert(Mux(rxDataQ.io.deq.fire, (localSnMaster.io.chi.rx.data.get.ready & fromSnNode(rxDataQ.io.deq.bits.asTypeOf(new DataFlit()).SrcID)) |
                                  (localRnSlave.io.chi.rx.data.get.ready & fromRnNode(rxDataQ.io.deq.bits.asTypeOf(new DataFlit()).SrcID)), true.B))

  // tx req
  io.toLocal.tx.req.get                   <> Queue(localSnMaster.io.chi.tx.req.get, 2) // Adding queues for timing considerations
  // for debug
  if (p(DebugOptionsKey).EnableDebug) {
    val io_toLocal_tx_req_bits_DbgAddr    = Wire(UInt(fullAddrBits.W)); dontTouch(io_toLocal_tx_req_bits_DbgAddr)
    val tx_req_bits_DbgAddr_Q             = Module(new Queue(UInt(fullAddrBits.W), 2))
    // enq
    tx_req_bits_DbgAddr_Q.io.enq.valid    := localSnMaster.io.chi.tx.req.get.valid
    tx_req_bits_DbgAddr_Q.io.enq.bits     := localSnMaster.io.chi_tx_req_bits_DbgAddr.get
    // deq
    tx_req_bits_DbgAddr_Q.io.deq.ready    := io.toLocal.tx.req.get.ready
    io_toLocal_tx_req_bits_DbgAddr        := tx_req_bits_DbgAddr_Q.io.deq.bits
  }

  // tx snoop
  io.toLocal.tx.snoop.get                 <> Queue(localRnSlave.io.chi.tx.snoop.get, 2) // Adding queues for timing considerations

  // tx resp
  io.toLocal.tx.resp.get                  <> Queue(localRnSlave.io.chi.tx.resp.get, 2) // Adding queues for timing considerations

  // tx data
  val txDataQ                             = Module(new Queue(new DataFlit(), 2)) // Adding queues for timing considerations
  txDataQ.io.enq.valid                    := localSnMaster.io.chi.tx.data.get.valid | localRnSlave.io.chi.tx.data.get.valid
  txDataQ.io.enq.bits                     := Mux(localSnMaster.io.chi.tx.data.get.valid, localSnMaster.io.chi.tx.data.get.bits, localRnSlave.io.chi.tx.data.get.bits)
  localSnMaster.io.chi.tx.data.get.ready  := txDataQ.io.enq.ready
  localRnSlave.io.chi.tx.data.get.ready   := txDataQ.io.enq.ready & !localSnMaster.io.chi.tx.data.get.valid

  io.toLocal.tx.data.get                  <> txDataQ.io.deq


  /*
   * Connect CSN CHI IO
   */
  if(hasCSN) {
    io.toCSNOpt.get.hn <> csnRnSlaveOpt.get.io.chi
    io.toCSNOpt.get.rn <> csnRnMasterOpt.get.io.chi
  }

  // TODO: change Xbar -> Ring
  /*
   * Connect Intf <-> Xbar
   */
  intfs.zipWithIndex.foreach {
    case(intf, i) =>
      intf.io.hnfID                           := io.hnfID
      intf.io.pcuID                           := io.pcuID
      intf.io.fIDVec                          := io.dcuNodeIDVec
      // EXU ctrl signals
      if (intf.io.req2ExuOpt.nonEmpty) {
          xbar.io.req2Exu.in(i)               <> intf.io.req2ExuOpt.get
      } else {
          xbar.io.req2Exu.in(i)               <> DontCare
      }
      if (intf.io.reqAck2IntfOpt.nonEmpty) {
          xbar.io.reqAck2Intf.out(i)          <> intf.io.reqAck2IntfOpt.get
      } else {
          xbar.io.reqAck2Intf.out(i)          <> DontCare
      }
      if (intf.io.resp2IntfOpt.nonEmpty) {
          xbar.io.resp2Intf.out(i)            <> intf.io.resp2IntfOpt.get
      } else {
          xbar.io.resp2Intf.out(i)            <> DontCare
      }
      xbar.io.req2Intf.out(i)                 <> intf.io.req2Intf
      xbar.io.resp2Exu.in(i)                  <> intf.io.resp2Exu
      // Intf DataBuffer signals
      if (intf.io.dbSigs.dbRCReqOpt.nonEmpty) {
          xbar.io.dbSigs.in0(i)               <> intf.io.dbSigs.dbRCReqOpt.get
      } else {
          xbar.io.dbSigs.in0(i)               <> DontCare
      }
      xbar.io.dbSigs.in1(i).getDBID           <> intf.io.dbSigs.getDBID
      xbar.io.dbSigs.in1(i).dbidResp          <> intf.io.dbSigs.dbidResp
      xbar.io.dbSigs.in1(i).dataFDB           <> intf.io.dbSigs.dataFDB
      xbar.io.dbSigs.in1(i).dataTDB           <> intf.io.dbSigs.dataTDB
  }

  /*
   * Connect EXUs <-> Xbar
   */
  exus.zipWithIndex.foreach {
    case (exu, i) =>
      exu.io.dcuID                    := i.U
      exu.io.pcuID                    := io.pcuID
      // slice ctrl signals
      xbar.io.req2Exu.out(i)          <> exu.io.req2Exu
      xbar.io.reqAck2Intf.in(i)       <> exu.io.reqAck2Intf
      xbar.io.resp2Intf.in(i)         <> exu.io.resp2Intf
      xbar.io.req2Intf.in(i)          <> exu.io.req2Intf
      xbar.io.resp2Exu.out(i)         <> exu.io.resp2Exu
      // slice DataBuffer signals
      xbar.io.dbSigs.in0(i+nrIntf)    <> exu.io.dbRCReq
  }

  /*
   * Connect DataBuffer <-> Xbar
   */
  databuffer.io <> xbar.io.dbSigs.out(0)

  // Directly connect to DataBuffer, reduction of Mux using
  // rx
  rxDataQ.io.deq.ready              := databuffer.io.dataTDB.ready
  databuffer.io.dataTDB.bits.data   := rxDataQ.io.deq.bits.Data
  // tx
  txDataQ.io.enq.bits.Data          := databuffer.io.dataFDB.bits.data
  txDataQ.io.enq.bits.DataID        := databuffer.io.dataFDB.bits.dataID
  txDataQ.io.enq.bits.BE            := databuffer.io.dataFDB.bits.mask
}
