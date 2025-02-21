package dongjiang.pcu.exu

import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.SnpOpcode._
import zhujiang.chi._
import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import dongjiang.pcu.exu.decode._
import dongjiang.utils.StepRREncoder
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.perf.{DebugOptions, DebugOptionsKey, HasPerfLogging}
import MSHRStateOH._
import math._
import xs.utils.debug.{DomainInfo, HardwareAssertion}

object MSHRStateOH {
  // [free] ---> [beSend] ---> [alreadySend] ---> [waitResp] ---> [beSend] ---> [free]
  // [free] ---> [beSend] ---> [alreadySend] ---> [free]
  val width       = 4

  val FREE        = "b0001".U
  val BESEND      = "b0010".U
  val ALREADYSEND = "b0100".U
  val WAITRESP    = "b1000".U

  val Free        = 0
  val BeSend      = 1
  val AlreadySend = 2
  val WaitResp    = 3
}


class MSHREntry(implicit p: Parameters) extends DJBundle {
  // mshrMes
  val mshrMes         = new Bundle {
    val state         = UInt(MSHRStateOH.width.W)
    val mTag          = UInt(mshrTagBits.W)
    val waitIntfVec   = Vec(nrIntf, Bool()) // Wait Snoop Resp or Req Resp
    val selfLock      = Bool()
    val othLock       = Bool()
  }
  // req mes
  val chiMes          = new ExuChiMesBundle()
  val chiIndex        = new ChiIndexBundle()
  // resp mes
  val respMes         = new ExuRespMesBundle()

  def isValid         = !mshrMes.state(Free)
  def isFree          = mshrMes.state(Free)
  def isBeSend        = mshrMes.state(BeSend)
  def isAlreadySend   = mshrMes.state(AlreadySend)
  def isWaitResp      = mshrMes.state(WaitResp)
  def isResp          = respMes.slvResp.valid | respMes.mstResp.valid | respMes.fwdState.valid
  def isReq           = !isResp
  def respBeSend      = !mshrMes.othLock & isBeSend & isResp
  def reqBeSend       = !mshrMes.othLock & isBeSend & isReq
  def noWaitIntf      = !mshrMes.waitIntfVec.reduce(_ | _)

  def useAddr   (x: UInt): UInt = { Cat(mshrMes.mTag, x.asTypeOf(UInt(mshrSetBits.W))) }
  def dirBank   (x: UInt): UInt = { getDirBank(useAddr(x)) }
  def fullAddr  (x: UInt, d: UInt, p: UInt): UInt = getFullAddr(useAddr(x), d, p, chiIndex.offset)
  def minDirSet (x: UInt): UInt = useAddr(x)(minDirSetBits - 1, 0)
}


class MSHRCtl()(implicit p: Parameters) extends DJModule with HasPerfLogging {
  // --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val dcuID         = Input(UInt(dcuBankBits.W))
    val pcuID         = Input(UInt(pcuBankBits.W))
    // Req To EXU
    val req2Exu       = Flipped(Decoupled(new Req2ExuBundle()))
    // Ack To Node
    val reqAck2Intf   = Decoupled(new ReqAck2IntfBundle())
    // Resp To EXU
    val resp2Exu      = Flipped(Decoupled(new Resp2ExuBundle()))
    // Task To MainPipe
    val pipeTask      = Vec(2, Decoupled(new PipeTaskBundle()))
    // Update Task From MainPipe
    val updMSHRVec    = Vec(2, Flipped(Decoupled(new UpdateMSHRReqBundle())))
    // Directory Read Req
    val dirRReadyVec  = Input(Vec(djparam.nrDirBank, Bool()))
    val dirRead       = Vec(2, Valid(new DirReadBundle()))
    // Directory Read MSHR Set Mes
    val dirReadMshr   = Vec(2, Flipped(Valid(new DirReadMSHRBundle())))
    val mshrResp2Dir  = Vec(2, Valid(new MSHRRespDirBundle()))
  })

  // ------------------------ Module declaration ------------------------- //
  val reqAck_s0_q           = Module(new Queue(new ReqAck2IntfBundle(), entries = nrBankPerPCU, flow = false, pipe = true))

  // --------------------- Reg / Wire declaration ------------------------ //
  // MSHR Init
  val mshrInit              = WireInit(0.U.asTypeOf(new MSHREntry())); mshrInit.mshrMes.state := FREE
  // MSHR Table
  val mshrTableReg          = RegInit(VecInit(Seq.fill(djparam.nrMSHRSets) { VecInit(Seq.fill(djparam.nrMSHRWays) { mshrInit }) }))
  val mshrNSTable           = Wire(Vec(djparam.nrMSHRSets, Vec(djparam.nrMSHRWays, UInt(MSHRStateOH.width.W)))) // Next State Table
  dontTouch(mshrNSTable)
  // Transfer Req From Node To MSHREntry
  val mshrAlloc_s0          = WireInit(0.U.asTypeOf(new MSHREntry()))
  // task s0
  val reqWillSendVecVec     = Wire(Vec(djparam.nrMSHRSets, Vec(djparam.nrMSHRWays, Bool())))
  val respWillSendVecVec    = Wire(Vec(djparam.nrMSHRSets, Vec(djparam.nrMSHRWays, Bool())))
  val reqWillSendWayVec     = Wire(Vec(djparam.nrMSHRSets, UInt(mshrWayBits.W)))
  val respWillSendWayVec    = Wire(Vec(djparam.nrMSHRSets, UInt(mshrWayBits.W)))
  val taskReq_s0            = Wire(Valid(new PipeTaskBundle()))
  val canGoReq_s0           = Wire(Bool())
  val taskResp_s0           = Wire(Valid(new PipeTaskBundle()))
  val canGoResp_s0          = Wire(Bool())
  // task s1
  val taskReq_s1_g          = RegInit(0.U.asTypeOf(Valid(new PipeTaskBundle())))
  val canGoReq_s1           = Wire(Bool())
  val taskResp_s1_g         = RegInit(0.U.asTypeOf(Valid(new PipeTaskBundle())))
  val canGoResp_s1          = Wire(Bool())
  // dir read mshr
  val resp2dirRegVec        = RegInit(0.U.asTypeOf(io.mshrResp2Dir))

  /*
   * for Debug
   */
  val mshr_dbg_addr = Wire(Vec(djparam.nrMSHRSets, Vec(djparam.nrMSHRWays, UInt(fullAddrBits.W))))
  mshr_dbg_addr.zipWithIndex.foreach { case(set, i) => set.zipWithIndex.foreach { case(way, j) => way := mshrTableReg(i)(j).fullAddr(i.U, io.dcuID, io.pcuID) } }
  if (p(DebugOptionsKey).EnableDebug) {
    dontTouch(mshr_dbg_addr)
  }


  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------- S0: Receive Req From Node or Let It Retry ------------------------------------ //
  // ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get MSHR Mes
   */
  // mshrTableReg
  val req2ExuMSet     = io.req2Exu.bits.pcuMes.mSet; dontTouch(req2ExuMSet)
  val req2ExuDirSet   = io.req2Exu.bits.pcuMes.minDirSet; dontTouch(req2ExuDirSet)
  val nodeReqMatchVec = mshrTableReg(req2ExuMSet).map { case m => m.isValid & m.mshrMes.mTag === io.req2Exu.bits.pcuMes.mTag }
  val lockMatchVec    = mshrTableReg(req2ExuMSet).map { case m => m.isValid & m.minDirSet(req2ExuMSet) === io.req2Exu.bits.pcuMes.minDirSet & m.mshrMes.selfLock }
  val nodeReqInvVec   = mshrTableReg(req2ExuMSet).map(_.isFree)
  val nodeReqInvWay   = PriorityEncoder(nodeReqInvVec)

  /*
   * Get Block Message
   */
  val canReceiveReq   = !nodeReqMatchVec.reduce(_ | _) & !lockMatchVec.reduce(_ | _) & PopCount(nodeReqInvVec) > 0.U


  /*
   * Transfer Req From Node To MSHREntry
   */
  mshrAlloc_s0.mshrMes.mTag             := io.req2Exu.bits.pcuMes.mTag
  mshrAlloc_s0.chiMes.expCompAck        := io.req2Exu.bits.chiMes.expCompAck
  mshrAlloc_s0.chiMes.opcode            := io.req2Exu.bits.chiMes.opcode
  mshrAlloc_s0.chiIndex                 := io.req2Exu.bits.chiIndex
  when(io.req2Exu.bits.chiMes.isReq & ((isWriteX(io.req2Exu.bits.chiMes.opcode) & !isCBX(io.req2Exu.bits.chiMes.opcode)) | (isAtomicX(io.req2Exu.bits.chiMes.opcode)))) {
    mshrAlloc_s0.respMes.slvDBID.valid  := true.B
    mshrAlloc_s0.respMes.slvDBID.bits   := io.req2Exu.bits.pcuIndex.dbID
  }



  /*
   * Receive Req From Node and Determine if it needs retry
   */
  reqAck_s0_q.io.enq.valid        := io.req2Exu.valid
  reqAck_s0_q.io.enq.bits.retry   := !canReceiveReq
  reqAck_s0_q.io.enq.bits.to      := io.req2Exu.bits.from
  reqAck_s0_q.io.enq.bits.entryID := io.req2Exu.bits.pcuIndex.entryID
  io.reqAck2Intf                  <> reqAck_s0_q.io.deq
  io.req2Exu.ready                  := reqAck_s0_q.io.enq.ready


  HardwareAssertion(Mux(io.resp2Exu.valid, mshrTableReg(io.resp2Exu.bits.pcuIndex.mshrSet)(io.resp2Exu.bits.pcuIndex.mshrWay).isWaitResp, true.B))


val hwaFlags = Array.fill(16)(Wire(Bool()))
for (i <- 0 until 16) {
  hwaFlags(i) := true.B
}

  // ---------------------------------------------------------------------------------------------------------------------- //
  // ---------------------------------------------- S0: Update MSHR Table Value  ------------------------------------------ //
  // ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Update MSHRTable
   */
  mshrTableReg.zipWithIndex.foreach {
    case(m, i) =>
      m.zipWithIndex.foreach {
        case(m, j) =>
          val respPipeUpdMSHRHit  = io.updMSHRVec(PipeID.RESP).fire & !io.updMSHRVec(PipeID.RESP).bits.isRetry & io.updMSHRVec(PipeID.RESP).bits.mshrMatch(i, j)
          val reqPipeUpdMSHRHit   = io.updMSHRVec(PipeID.REQ).fire  & !io.updMSHRVec(PipeID.REQ).bits.isRetry  & io.updMSHRVec(PipeID.REQ).bits.mshrMatch(i, j)
          val updMSHRHit          = respPipeUpdMSHRHit | reqPipeUpdMSHRHit
          val updMSHRBits         = Mux(respPipeUpdMSHRHit, io.updMSHRVec(PipeID.RESP).bits, io.updMSHRVec(PipeID.REQ).bits)
          val resp2ExuHit         = io.resp2Exu.fire & io.resp2Exu.bits.pcuIndex.mshrMatch(i, j)
          val req2ExuHit          = io.req2Exu.fire  & canReceiveReq & i.U === req2ExuMSet & j.U === nodeReqInvWay
          HardwareAssertion(PopCount(Seq(respPipeUpdMSHRHit, reqPipeUpdMSHRHit, resp2ExuHit, req2ExuHit)) <= 1.U)

          /*
           * Pipe Update mshrTable value
           */
          when(updMSHRHit) {
            // req
            when(updMSHRBits.hasNewReq) {
              m                     := 0.U.asTypeOf(m)
              m.chiMes.opcode       := updMSHRBits.opcode
              m.chiMes.channel      := updMSHRBits.channel
              m.mshrMes.mTag        := updMSHRBits.mTag
            }
            // req or update
            m.respMes               := 0.U.asTypeOf(m.respMes)
            m.mshrMes.waitIntfVec   := updMSHRBits.waitIntfVec
            hwaFlags(0) := PopCount(m.mshrMes.waitIntfVec) === 0.U
            hwaFlags(1) := m.isAlreadySend

            /*
             * Resp Update mshrTable value
             */
          }.elsewhen(resp2ExuHit) {
            // Recovery of pending intf identifiers
            m.mshrMes.waitIntfVec(io.resp2Exu.bits.from) := false.B
            // Record Resp Mes
            when(io.resp2Exu.bits.pcuMes.isCompAck) {
              // Only use in DMT
              // Nothing to do and it has been receive master resp
              hwaFlags(2) := PopCount(m.mshrMes.waitIntfVec) === 1.U
              hwaFlags(3) := m.respMes.mstResp.valid
            }
            when(io.resp2Exu.bits.pcuMes.isSnpResp) {
              m.respMes.slvResp.valid   := true.B
              m.respMes.slvResp.bits    := io.resp2Exu.bits.chiMes.resp
              m.respMes.slvDBID.valid   := io.resp2Exu.bits.pcuMes.hasData
              m.respMes.slvDBID.bits    := io.resp2Exu.bits.pcuIndex.dbID
              m.respMes.fwdState.valid  := io.resp2Exu.bits.pcuMes.fwdSVald | m.respMes.fwdState.valid
              m.respMes.fwdState.bits   := Mux(io.resp2Exu.bits.pcuMes.fwdSVald, io.resp2Exu.bits.chiMes.fwdState, m.respMes.fwdState.bits)
            }.elsewhen(io.resp2Exu.bits.pcuMes.isReqResp) {
              m.respMes.mstResp.valid   := true.B
              m.respMes.mstResp.bits    := io.resp2Exu.bits.chiMes.resp
              m.respMes.mstDBID.valid   := io.resp2Exu.bits.pcuMes.hasData
              m.respMes.mstDBID.bits    := io.resp2Exu.bits.pcuIndex.dbID
            }.elsewhen(io.resp2Exu.bits.pcuMes.isWriResp) {
              when(io.resp2Exu.bits.from === IncoID.LOCALSLV.U) {
                m.respMes.slvResp.valid := true.B
                m.respMes.slvResp.bits  := io.resp2Exu.bits.chiMes.resp
                m.respMes.slvDBID.valid := true.B
                hwaFlags(4) := io.resp2Exu.bits.pcuMes.hasData
                m.respMes.slvDBID.bits  := io.resp2Exu.bits.pcuIndex.dbID
              }.elsewhen(io.resp2Exu.bits.from === IncoID.LOCALMST.U) {
                // Nothing to do and State Will be Free
              }.otherwise {
                hwaFlags(6) := false.B
              }
            }
            /*
             * Receive Node Req
             */
          }.elsewhen(req2ExuHit) {
            m := 0.U.asTypeOf(m)
            m := mshrAlloc_s0
            hwaFlags(9) := m.isFree
            /*
             * Clean MSHR Entry When Its Free
             */
          }.elsewhen(m.isFree) {
            m := 0.U.asTypeOf(m)
          }
          HardwareAssertion(hwaFlags(0), cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${m.fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${m.chiMes.channel}] OP[0x${m.chiMes.opcode}] STATE[0x${m.mshrMes.state}]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
          HardwareAssertion(hwaFlags(1), cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${m.fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${m.chiMes.channel}] OP[0x${m.chiMes.opcode}] STATE[0x${m.mshrMes.state}]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
          HardwareAssertion(hwaFlags(2), cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${m.fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${m.chiMes.channel}] OP[0x${m.chiMes.opcode}] STATE[0x${m.mshrMes.state}]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
          HardwareAssertion(hwaFlags(3), cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${m.fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${m.chiMes.channel}] OP[0x${m.chiMes.opcode}] STATE[0x${m.mshrMes.state}]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
          HardwareAssertion(hwaFlags(4))
          HardwareAssertion(hwaFlags(5), cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${m.fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${m.chiMes.channel}] OP[0x${m.chiMes.opcode}] STATE[0x${m.mshrMes.state}]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
          HardwareAssertion(hwaFlags(6))
          HardwareAssertion(hwaFlags(7), cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${m.fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${m.chiMes.channel}] OP[0x${m.chiMes.opcode}] STATE[0x${m.mshrMes.state}]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
          HardwareAssertion(hwaFlags(8), cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${m.fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${m.chiMes.channel}] OP[0x${m.chiMes.opcode}] STATE[0x${m.mshrMes.state}]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
          HardwareAssertion(hwaFlags(9), cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${m.fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${m.chiMes.channel}] OP[0x${m.chiMes.opcode}] STATE[0x${m.mshrMes.state}]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
        }
        HardwareAssertion.placePipe(1)
      
  }
  HardwareAssertion.placePipe(2)

  /*
   * Set ready value
   */
  io.updMSHRVec(PipeID.RESP).ready  := true.B
  io.updMSHRVec(PipeID.REQ).ready   := true.B
  io.resp2Exu.ready                 := true.B


  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------------- S0: Update MHSR Table Lock --------------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  mshrTableReg.zip(mshrNSTable).zipWithIndex.foreach {
    case ((mVec, ns), i) =>
      mVec.zip(ns).zipWithIndex.foreach {
        case ((m, ns), j) =>
          /*
           * Set othLock
           */
          // Free ---> BeSend
          when(m.isFree & ns(BeSend)) {
            val lockSrcVec      = mVec.map { case a => a.mshrMes.selfLock & a.minDirSet(i.U) === mshrAlloc_s0.minDirSet(i.U) }
            hwaFlags(10) := PopCount(lockSrcVec) <= 1.U
            m.mshrMes.othLock   := lockSrcVec.reduce(_ | _)
          // BeSend/WaitResp ---> BeSend
          }.elsewhen((m.isBeSend | m.isWaitResp) & ns(BeSend)) {
            val lockSrcVec      = mVec.map { case a => a.mshrMes.selfLock & a.minDirSet(i.U) === m.minDirSet(i.U) }
            hwaFlags(11) := PopCount(lockSrcVec) <= 1.U
            m.mshrMes.othLock   := lockSrcVec.reduce(_ | _)
          }.otherwise {
            m.mshrMes.othLock   := m.mshrMes.othLock
          }
          /*
           * Set selfLock
           */
          // BeSend ---> AlreadySend
          when(m.isBeSend & ns(AlreadySend)) {
            m.mshrMes.selfLock  := true.B
            hwaFlags(12) := !m.mshrMes.selfLock
          // WaitResp/AlreadySend ---> Free
          }.elsewhen((m.isAlreadySend | m.isWaitResp) & ns(Free)) {
            m.mshrMes.selfLock  := false.B
            hwaFlags(13) := Mux(m.isAlreadySend, m.mshrMes.selfLock, m.mshrMes.selfLock | m.chiMes.opcode =/= Replace)
          // Set by ProcessPipe
          }.elsewhen((io.updMSHRVec(PipeID.RESP).fire & io.updMSHRVec(PipeID.RESP).bits.unlock & io.updMSHRVec(PipeID.RESP).bits.mshrMatch(i, j)) |
                     (io.updMSHRVec(PipeID.REQ).fire  & io.updMSHRVec(PipeID.REQ).bits.unlock  & io.updMSHRVec(PipeID.REQ).bits.mshrMatch(i, j))) {
            m.mshrMes.selfLock  := false.B
            hwaFlags(14) := m.mshrMes.selfLock
          }.otherwise {
            m.mshrMes.selfLock  := m.mshrMes.selfLock
          }

          HardwareAssertion(hwaFlags(10), cf"MSHR[${i}][${j}]")
          HardwareAssertion(hwaFlags(11), cf"MSHR[${i}][${j}]")
          HardwareAssertion(hwaFlags(12), cf"MSHR[${i}][${j}]")
          HardwareAssertion(hwaFlags(13), cf"MSHR[${i}][${j}]")

          HardwareAssertion.placePipe(1)
      }
      HardwareAssertion.placePipe(2)
  }



  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------------- S0: Update MHSR Table State -------------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Update MSHR Table State
   */
  mshrTableReg.zipWithIndex.foreach {
    case (m, i) =>
      m.zipWithIndex.foreach {
        case (m, j) =>
          val nextState     = mshrNSTable(i)(j)
          // Free
          when(m.isFree) {
            val nodeHit     = io.req2Exu.valid & canReceiveReq & i.U === req2ExuMSet & j.U === nodeReqInvWay
            nextState       := Mux(nodeHit, BESEND, FREE)
            // BeSend
          }.elsewhen(m.isBeSend) {
            val reqhit      = taskReq_s0.valid  & canGoReq_s0  & taskReq_s0.bits.mshrMatch(i, j)
            val resphit     = taskResp_s0.valid & canGoResp_s0 & taskResp_s0.bits.mshrMatch(i, j)
            nextState       := Mux(reqhit | resphit, ALREADYSEND, BESEND)
          // AlreadySend
          }.elsewhen(m.isAlreadySend) {
            val respPipeHit = io.updMSHRVec(PipeID.RESP).fire & io.updMSHRVec(PipeID.RESP).bits.mshrMatch(i, j)
            val reqPipeHit  = io.updMSHRVec(PipeID.REQ).fire  & io.updMSHRVec(PipeID.REQ).bits.mshrMatch(i, j)
            val hit         = respPipeHit | reqPipeHit
            val retry       = hit & Mux(respPipeHit, io.updMSHRVec(PipeID.RESP).bits.isRetry,  io.updMSHRVec(PipeID.REQ).bits.isRetry)
            val update      = hit & Mux(respPipeHit, io.updMSHRVec(PipeID.RESP).bits.isUpdate, io.updMSHRVec(PipeID.REQ).bits.isUpdate)
            val clean       = hit & Mux(respPipeHit, io.updMSHRVec(PipeID.RESP).bits.isClean,  io.updMSHRVec(PipeID.REQ).bits.isClean)
            nextState       := Mux(retry, BESEND,
                                 Mux(update, WAITRESP,
                                   Mux(clean, FREE, ALREADYSEND)))
          // WaitResp
          }.elsewhen(m.isWaitResp) {
            val hit         = m.noWaitIntf
            val noResp      = m.respMes.noRespValid
            nextState       := Mux(hit, Mux(noResp, FREE, BESEND), WAITRESP)
            // assert(Mux(m.respMes.fwdState.valid, m.respMes.slvResp.valid, true.B), cf"MSHR[${i}][${j}]")
            hwaFlags(15) := Mux(m.respMes.fwdState.valid, m.respMes.slvResp.valid, true.B)
          }.otherwise {
            nextState       := m.mshrMes.state
          }
          m.mshrMes.state   := nextState
          HardwareAssertion(hwaFlags(15), cf"MSHR[${i}][${j}]")
          HardwareAssertion(PopCount(m.mshrMes.state) === 1.U, cf"MSHR[${i}][${j}]")
          HardwareAssertion(PopCount(nextState) === 1.U, cf"MSHR[${i}][${j}]")
          HardwareAssertion.placePipe(1)
      }
      HardwareAssertion.placePipe(2)
  }

  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------------- S0: Get task_s0 from MSHR ---------------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Can Send Set From Dir Read Ready
   */
  // TODO: if dont need to read Dir, it should not be ctrl by mshr selfLock and othLock
  mshrTableReg.zip(reqWillSendVecVec).zipWithIndex.foreach  { case((m, v), i) => m.zip(v).foreach { case(m, v) => v := m.reqBeSend  & io.dirRReadyVec(m.dirBank(i.U)) } }
  mshrTableReg.zip(respWillSendVecVec).zipWithIndex.foreach { case((m, v), i) => m.zip(v).foreach { case(m, v) => v := m.respBeSend & io.dirRReadyVec(m.dirBank(i.U)) } }

  respWillSendWayVec.zip(respWillSendVecVec).foreach { case(a, b) => a := StepRREncoder(b, canGoResp_s0) }
  reqWillSendWayVec.zip(reqWillSendVecVec).foreach   { case(a, b) => a := StepRREncoder(b, canGoReq_s0)  }

  /*
   * Get task_s0(resp) from MSHRTable
   */
  val respSendSet         = StepRREncoder(respWillSendVecVec.map(_.reduce(_ | _)), canGoResp_s0); dontTouch(respSendSet)
  val respSendWay         = WireInit(respWillSendWayVec(respSendSet)); dontTouch(respSendWay)
  val mshrResp            = mshrTableReg(respSendSet)(respSendWay)
  val taskRespValid       = respWillSendVecVec.map(_.reduce(_ | _)).reduce(_ | _)


  /*
   * Get task_s0(req) from MSHRTable
   */
  val reqSendSet          = StepRREncoder(reqWillSendVecVec.map(_.reduce(_ | _)), canGoReq_s0); dontTouch(reqSendSet)
  val reqSendWay          = WireInit(reqWillSendWayVec(reqSendSet)); dontTouch(reqSendWay)
  val mshrReq             = mshrTableReg(reqSendSet)(reqSendWay)
  val taskReqValid        = reqWillSendVecVec.map(_.reduce(_ | _)).reduce(_ | _)


  /*
   * Select resp task_s0
   */
  taskResp_s0.valid                 := taskRespValid
  taskResp_s0.bits.chiMes           := mshrResp.chiMes
  taskResp_s0.bits.chiIndex         := mshrResp.chiIndex
  taskResp_s0.bits.respMes          := mshrResp.respMes
  taskResp_s0.bits.taskMes.useAddr  := mshrResp.useAddr(respSendSet)
  taskResp_s0.bits.taskMes.pipeID   := PipeID.RESP
  taskResp_s0.bits.taskMes.mshrWay  := respSendWay
  taskResp_s0.bits.taskMes.readDir  := true.B // TODO
  canGoResp_s0                      := canGoResp_s1 | !taskResp_s1_g.valid


  /*
   * Select req task_s0
   */
  val blockTaskReq_s0               = taskResp_s0.valid & taskResp_s0.bits.taskMes.dirBank === taskReq_s0.bits.taskMes.dirBank
  taskReq_s0.valid                  := taskReqValid & !blockTaskReq_s0
  taskReq_s0.bits.chiMes            := mshrReq.chiMes
  taskReq_s0.bits.chiIndex          := mshrReq.chiIndex
  taskReq_s0.bits.respMes           := mshrReq.respMes
  taskReq_s0.bits.taskMes.useAddr   := mshrReq.useAddr(reqSendSet)
  taskReq_s0.bits.taskMes.pipeID    := PipeID.REQ
  taskReq_s0.bits.taskMes.mshrWay   := reqSendWay
  taskReq_s0.bits.taskMes.readDir   := true.B // TODO
  canGoReq_s0                       := canGoReq_s1 | !taskReq_s1_g.valid


  /*
   * Read Directory
   */
  // resp
  io.dirRead(PipeID.RESP).valid         := taskResp_s0.valid & canGoResp_s0
  io.dirRead(PipeID.RESP).bits.useAddr  := taskResp_s0.bits.taskMes.useAddr
  io.dirRead(PipeID.RESP).bits.mshrWay  := taskResp_s0.bits.taskMes.mshrWay
  io.dirRead(PipeID.RESP).bits.pipeID   := taskResp_s0.bits.taskMes.pipeID
  // req
  io.dirRead(PipeID.REQ).valid          := taskReq_s0.valid & canGoReq_s0
  io.dirRead(PipeID.REQ).bits.useAddr   := taskReq_s0.bits.taskMes.useAddr
  io.dirRead(PipeID.REQ).bits.mshrWay   := taskReq_s0.bits.taskMes.mshrWay
  io.dirRead(PipeID.REQ).bits.pipeID    := taskReq_s0.bits.taskMes.pipeID


  // ---------------------------------------------------------------------------------------------------------------------- //
  // ---------------------------------- S0: Read Dir and send task to MainPipe -------------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Set Task S1 Value
   */
  // resp
  taskResp_s1_g.valid       := Mux(taskResp_s0.valid, true.B, taskResp_s1_g.valid & !canGoResp_s1)
  taskResp_s1_g.bits        := Mux(taskResp_s0.valid & canGoResp_s0, taskResp_s0.bits, taskResp_s1_g.bits)
  canGoResp_s1              := io.pipeTask(PipeID.RESP).ready
  //req
  taskReq_s1_g.valid        := Mux(taskReq_s0.valid, true.B, taskReq_s1_g.valid & !canGoReq_s1)
  taskReq_s1_g.bits         := Mux(taskReq_s0.valid & canGoReq_s0, taskReq_s0.bits, taskReq_s1_g.bits)
  canGoReq_s1               := io.pipeTask(PipeID.REQ).ready

  /*
   * Send Task to Pipe
   */
  // resp
  io.pipeTask(PipeID.RESP).valid  := taskResp_s1_g.valid
  io.pipeTask(PipeID.RESP).bits   := taskResp_s1_g.bits
  // req
  io.pipeTask(PipeID.REQ).valid   := taskReq_s1_g.valid
  io.pipeTask(PipeID.REQ).bits    := taskReq_s1_g.bits



  // ------------------------ S2: Dir Read MSHR and MSHR Resp to Dir --------------------------//
  io.dirReadMshr.zip(resp2dirRegVec).foreach {
    case (read, resp) =>
      when(read.valid) {
        resp.valid          := true.B
        resp.bits.pipeID    := read.bits.pipeID
        resp.bits.dirBank   := read.bits.dirBank
        resp.bits.addrs.zipWithIndex.foreach {
          case (r, i) =>
            r.valid := mshrTableReg(read.bits.mshrSet)(i).isValid
            r.bits  := mshrTableReg(read.bits.mshrSet)(i).useAddr(read.bits.mshrSet)
        }
      }.otherwise {
        resp.valid    := false.B
        resp.bits     := 0.U.asTypeOf(resp.bits)
      }
  }
  io.mshrResp2Dir     := resp2dirRegVec


// -------------------------------------------------- Assertion ------------------------------------------------------ //
  // MSHR Timeout Check
  val cntMSHRReg = RegInit(VecInit(Seq.fill(djparam.nrMSHRSets) { VecInit(Seq.fill(djparam.nrMSHRWays) { 0.U(64.W) }) }))

  cntMSHRReg.zipWithIndex.foreach { case (c0, i) => c0.zipWithIndex.foreach { case(c1, j) => HardwareAssertion.checkTimeout(mshrTableReg(i)(j).isFree, TIMEOUT_MSHR,cf"MSHR[0x${i.U}][0x${j.U}] ADDR[0x${mshrTableReg(i)(j).fullAddr(i.U, io.dcuID, io.pcuID)}] CHANNEL[0x${mshrTableReg(i)(j).chiMes.channel}] OP[0x${mshrTableReg(i)(j).chiMes.opcode}] STATE[0x${mshrTableReg(i)(j).mshrMes.state}] TIMEOUT", i.U, j.U, mshrTableReg(i)(j).fullAddr(i.U, io.dcuID, io.pcuID), mshrTableReg(i)(j).chiMes.channel, mshrTableReg(i)(j).chiMes.opcode, mshrTableReg(i)(j).mshrMes.state)  } }
  HardwareAssertion.placePipe(2)
// -------------------------------------------------- Perf Counter ------------------------------------------------------ //
  require(djparam.nrMSHRWays >= 4 & djparam.nrMSHRWays % 4 == 0)
  for (i <- 0 until djparam.nrMSHRSets) {
    for (j <- 0 until (djparam.nrMSHRWays / 4)) {
      XSPerfAccumulate(s"pcu_MSHRCtl_entry_group[${i}][${j}]_deal_req_cnt", io.req2Exu.fire & canReceiveReq & req2ExuMSet === i.U & (j * 4).U <= nodeReqInvWay & nodeReqInvWay <= (j * 4 + 3).U)
      XSPerfAccumulate(s"pcu_MHSRCtl_group[${i}]_deal_req_cnt", io.req2Exu.fire & canReceiveReq & req2ExuMSet === i.U)
      XSPerfAccumulate(s"pcu_MSHRCtl_group[${i}]_req_block_cnt", io.req2Exu.fire & req2ExuMSet === i.U & !nodeReqMatchVec.reduce(_ | _) & PopCount(nodeReqInvVec) === 0.U)
    }
  }

  HardwareAssertion.placePipe(3)
}