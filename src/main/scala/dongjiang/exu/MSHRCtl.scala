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
import dongjiang.utils.Encoder._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.perf.{DebugOptions, DebugOptionsKey, HasPerfLogging}


object MSHRState {
  // [free] ---> [beSend] ---> [alreadySend] ---> [waitResp] ---> [beSend] ---> [free]
  // [free] ---> [beSend] ---> [alreadySend] ---> [free]
  val width       = 2
  val Free        = "b00".U
  val BeSend      = "b01".U
  val AlreadySend = "b10".U
  val WaitResp    = "b11".U
}


class MSHREntry(implicit p: Parameters) extends DJBundle {
  // mshrMes
  val mshrMes         = new Bundle {
    val state         = UInt(MSHRState.width.W)
    val mTag          = UInt(mshrTagBits.W)
    val lockDirSet    = Bool()
    val waitIntfVec   = Vec(nrIntf, Bool()) // Wait Snoop Resp or Req Resp
  }
  // req mes
  val chiMes          = new ExuChiMesBundle()
  val chiIndex        = new ChiIndexBundle()
  // resp mes
  val respMes         = new ExuRespMesBundle()

  def isValid         = mshrMes.state =/= MSHRState.Free
  def isFree          = mshrMes.state === MSHRState.Free
  def isBeSend        = mshrMes.state === MSHRState.BeSend
  def isAlreadySend   = mshrMes.state === MSHRState.AlreadySend
  def isWaitResp      = mshrMes.state === MSHRState.WaitResp
  def isResp          = respMes.slvResp.valid | respMes.mstResp.valid | respMes.fwdState.valid
  def isReq           = !isResp
  def respBeSend      = isBeSend & isResp
  def reqBeSend       = isBeSend & isReq
  def noWaitIntf      = !mshrMes.waitIntfVec.reduce(_ | _)

  def useAddr   (x: UInt): UInt = { Cat(mshrMes.mTag, x.asTypeOf(UInt(mshrSetBits.W))) }
  def dirBank   (x: UInt): UInt = { getDirBank(useAddr(x)) }
  def fullAddr  (x: UInt, d: UInt, p: UInt): UInt = getFullAddr(useAddr(x), d, p)
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
    val updMSHR       = Flipped(Decoupled(new UpdateMSHRReqBundle()))
    val updLockMSHR   = Vec(2, Flipped(Valid(UInt(minDirSetBits.W))))
    // Directory Read Req
    val dirRReadyVec  = Input(Vec(djparam.nrDirBank, Bool()))
    val dirRead       = Vec(2, Valid(new DirReadBundle()))
    // Directory Read MSHR Set Mes
    val dirReadMshr   = Vec(2, Flipped(Valid(new DirReadMSHRBundle())))
    val mshrResp2Dir  = Vec(2, Valid(new MSHRRespDirBundle()))
  })

  // TODO: Delete the following code when the coding is complete
  dontTouch(io)

  // ------------------------ Module declaration ------------------------- //
  val reqAck_s0_q           = Module(new Queue(new ReqAck2IntfBundle(), entries = nrBankPerPCU, flow = false, pipe = true))

  // --------------------- Reg / Wire declaration ------------------------ //
  // mshrTable
  val mshrTableReg          = RegInit(VecInit(Seq.fill(djparam.nrMSHRSets) { VecInit(Seq.fill(djparam.nrMSHRWays) { 0.U.asTypeOf(new MSHREntry()) }) }))
  val mshrLockVecReg        = RegInit(VecInit(Seq.fill(nrMinDirSet) { false.B })) // TODO: 2048 is so big
  // Transfer Req From Node To MSHREntry
  val mshrAlloc_s0          = WireInit(0.U.asTypeOf(new MSHREntry()))
  // task s0
  val reqWillSendVecVec     = Wire(Vec(djparam.nrMSHRSets, Vec(djparam.nrMSHRWays, Bool())))
  val respWillSendVecVec    = Wire(Vec(djparam.nrMSHRSets, Vec(djparam.nrMSHRWays, Bool())))
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
  val nodeReqMatchVec = mshrTableReg(req2ExuMSet).map { case m =>
    val hit = Wire(Bool())
    when(m.mshrMes.lockDirSet) {
      // [useAddr] = [mTag] + [mSet]
      // [useAddr] = [minDirTag] + [minDirSet] + [dirBank]
      hit := m.minDirSet(req2ExuMSet) === req2ExuDirSet
      require(minDirSetBits > mshrSetBits)
    }.otherwise {
      hit := m.mshrMes.mTag === io.req2Exu.bits.pcuMes.mTag
    }
    hit & m.isValid
  }
  val nodeReqInvVec   = mshrTableReg(req2ExuMSet).map(_.isFree)
  val nodeReqInvWay   = PriorityEncoder(nodeReqInvVec)

  /*
   * Get Block Message
   */
  val canReceiveReq   = !nodeReqMatchVec.reduce(_ | _) & PopCount(nodeReqInvVec) > 0.U


  /*
   * Transfer Req From Node To MSHREntry
   */
  mshrAlloc_s0.mshrMes.mTag             := io.req2Exu.bits.pcuMes.mTag
  mshrAlloc_s0.chiMes.expCompAck        := io.req2Exu.bits.chiMes.expCompAck
  mshrAlloc_s0.chiMes.opcode            := io.req2Exu.bits.chiMes.opcode
  mshrAlloc_s0.chiIndex                 := io.req2Exu.bits.chiIndex
  when(io.req2Exu.bits.chiMes.isReq & isWriteX(io.req2Exu.bits.chiMes.opcode) & !isCBX(io.req2Exu.bits.chiMes.opcode)) {
    mshrAlloc_s0.respMes.slvDBID.valid  := true.B
    mshrAlloc_s0.respMes.slvDBID.bits   := io.req2Exu.bits.pcuIndex.dbID
  }



  /*
   * Receive Req From Node and Determine if it needs retry
   * TODO: Setting blocking timer
   */
  io.req2Exu.ready                := reqAck_s0_q.io.enq.ready
  reqAck_s0_q.io.enq.valid        := io.req2Exu.valid
  reqAck_s0_q.io.enq.bits.retry   := !canReceiveReq
  reqAck_s0_q.io.enq.bits.to      := io.req2Exu.bits.from
  reqAck_s0_q.io.enq.bits.entryID := io.req2Exu.bits.pcuIndex.entryID
  io.reqAck2Intf                  <> reqAck_s0_q.io.deq


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
          /*
           * Pipe Update mshrTable value
           */
          when(io.updMSHR.valid & !io.updMSHR.bits.isRetry & io.updMSHR.bits.mshrMatch(i, j)) {
            // req
            when(io.updMSHR.bits.hasNewReq) {
              m                     := 0.U.asTypeOf(m)
              m.chiMes.opcode       := io.updMSHR.bits.opcode
              m.chiMes.channel      := io.updMSHR.bits.channel
              m.mshrMes.mTag        := io.updMSHR.bits.mTag
              m.mshrMes.lockDirSet  := io.updMSHR.bits.lockDirSet
            }
            // req or update
            m.respMes               := 0.U.asTypeOf(m.respMes)
            m.mshrMes.waitIntfVec   := io.updMSHR.bits.waitIntfVec
            assert(PopCount(m.mshrMes.waitIntfVec) === 0.U, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
            assert(m.isAlreadySend, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
            /*
             * Resp Update mshrTable value
             */
          }.elsewhen(io.resp2Exu.valid & io.resp2Exu.bits.pcuIndex.mshrMatch(i, j)) {
            assert(!io.resp2Exu.bits.pcuMes.isUpdate, "TODO")
            // Recovery of pending intf identifiers
            m.mshrMes.waitIntfVec(io.resp2Exu.bits.from) := false.B
            // Record Resp Mes
            when(io.resp2Exu.bits.pcuMes.isCompAck) {
              // Only use in DMT
              // Nothing to do and it has been receicve master resp
              assert(PopCount(m.mshrMes.waitIntfVec) === 1.U, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
              assert(m.respMes.mstResp.valid, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
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
                m.respMes.slvDBID.valid := true.B; assert(io.resp2Exu.bits.pcuMes.hasData)
                m.respMes.slvDBID.bits  := io.resp2Exu.bits.pcuIndex.dbID
              }.elsewhen(io.resp2Exu.bits.from === IncoID.LOCALMST.U) {
                // Nothing to do and State Will be Free
                assert(m.respMes.noRespValid, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
              }.otherwise {
                assert(false.B)
              }
            }
            assert(m.mshrMes.waitIntfVec(io.resp2Exu.bits.from), s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
            assert(m.isWaitResp, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
            /*
             * Receive Node Req
             */
          }.elsewhen(io.req2Exu.fire & canReceiveReq & i.U === req2ExuMSet & j.U === nodeReqInvWay) {
            m := 0.U.asTypeOf(m)
            m := mshrAlloc_s0
            assert(m.isFree, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.fullAddr(i.U, io.dcuID, io.pcuID), m.chiMes.channel, m.chiMes.opcode, m.mshrMes.state)
            /*
             * Clean MSHR Entry When Its Free
             */
          }.elsewhen(m.isFree) {
            m := 0.U.asTypeOf(m)
          }
      }
  }


  /*
   * Set ready value
   */
  io.updMSHR.ready := true.B
  io.resp2Exu.ready := true.B


  /*
   * Update Lock Vec
   */
  mshrLockVecReg.zipWithIndex.foreach {
    case(lock, i) =>
      when((io.updLockMSHR(0).valid & io.updLockMSHR(0).bits === i.U) | (io.updLockMSHR(1).valid & io.updLockMSHR(1).bits === i.U)) {
        lock := false.B
        assert(lock)
      }.elsewhen((taskReq_s0.valid  & canGoReq_s0  & taskReq_s0.bits.taskMes.readDir  & taskReq_s0.bits.taskMes.minDirSet === i.U) |   // Req Fire
                 (taskResp_s0.valid & canGoResp_s0 & taskResp_s0.bits.taskMes.readDir & taskResp_s0.bits.taskMes.minDirSet === i.U)) { // Resp Fire
        lock := true.B
      }
  }
  assert(Mux(io.updLockMSHR(0).valid & io.updLockMSHR(1).valid, !(io.updLockMSHR(0).bits === io.updLockMSHR(1).bits), true.B))



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
          switch(m.mshrMes.state) {
            // Free
            is(MSHRState.Free) {
              val nodeHit     = io.req2Exu.valid & canReceiveReq & i.U === req2ExuMSet & j.U === nodeReqInvWay
              m.mshrMes.state := Mux(nodeHit, MSHRState.BeSend, MSHRState.Free)
            }
            // BeSend
            is(MSHRState.BeSend) {
              val reqhit      = taskReq_s0.valid & canGoReq_s0 & taskReq_s0.bits.mshrMatch(i, j)
              val resphit     = taskResp_s0.valid & canGoResp_s0 & taskResp_s0.bits.mshrMatch(i, j)
              m.mshrMes.state := Mux(reqhit | resphit, MSHRState.AlreadySend, MSHRState.BeSend)
            }
            // AlreadySend
            is(MSHRState.AlreadySend) {
              val hit         = io.updMSHR.valid & io.updMSHR.bits.mshrMatch(i, j)
              val retry       = hit & io.updMSHR.bits.isRetry
              val update      = hit & io.updMSHR.bits.isUpdate
              val clean       = hit & io.updMSHR.bits.isClean
              m.mshrMes.state := Mux(retry, MSHRState.BeSend,
                Mux(update, MSHRState.WaitResp,
                  Mux(clean, MSHRState.Free, MSHRState.AlreadySend)))
            }
            // WaitResp
            is(MSHRState.WaitResp) {
              val hit         = m.noWaitIntf
              val noResp      = m.respMes.noRespValid
              m.mshrMes.state := Mux(hit, Mux(noResp, MSHRState.Free, MSHRState.BeSend), MSHRState.WaitResp)
              assert(Mux(m.respMes.fwdState.valid, m.respMes.slvResp.valid, true.B))
            }
          }
      }
  }

  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------------- S0: Get task_s0 from MSHR ---------------------------------------------- //
  // ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Can Send Set From Dir Read Ready and mshrLockVecReg
   */
  // TODO: if dont need to read Dir, it should not be ctrl by mshrLockVecReg
  mshrTableReg.zip(reqWillSendVecVec).zipWithIndex.foreach  { case((m, v), i) => m.zip(v).foreach { case(m, v) => v := m.reqBeSend  & io.dirRReadyVec(m.dirBank(i.U)) & !mshrLockVecReg(m.minDirSet(i.U)) } }
  mshrTableReg.zip(respWillSendVecVec).zipWithIndex.foreach { case((m, v), i) => m.zip(v).foreach { case(m, v) => v := m.respBeSend & io.dirRReadyVec(m.dirBank(i.U)) & !mshrLockVecReg(m.minDirSet(i.U)) } }


  /*
   * Get task_s0(resp) from MSHRTable
   */
  val respSendSet         = RREncoder(respWillSendVecVec.map(_.reduce(_ | _))); dontTouch(respSendSet)
  val respSendWay         = RREncoder(respWillSendVecVec(respSendSet)); dontTouch(respSendWay)
  val mshrResp            = mshrTableReg(respSendSet)(respSendWay)
  val taskRespValid       = respWillSendVecVec.map(_.reduce(_ | _)).reduce(_ | _)


  /*
   * Get task_s0(req) from MSHRTable
   */
  val reqSendSet          = RREncoder(reqWillSendVecVec.map(_.reduce(_ | _))); dontTouch(reqSendSet)
  val reqSendWay          = RREncoder(reqWillSendVecVec(reqSendSet)); dontTouch(reqSendWay)
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
  taskReq_s0.valid                  := taskReqValid & !(taskResp_s0.valid & taskResp_s0.bits.taskMes.dirBank === taskReq_s0.bits.taskMes.dirBank)
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
  cntMSHRReg.zipWithIndex.foreach { case (c0, i) => c0.zipWithIndex.foreach { case(c1, j) => c1 := Mux(mshrTableReg(i)(j).isFree, 0.U, c1 + 1.U)  } }
  cntMSHRReg.zipWithIndex.foreach { case (c0, i) => c0.zipWithIndex.foreach { case(c1, j) => assert(c1 < TIMEOUT_MSHR.U, "MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x] TIMEOUT", i.U, j.U, mshrTableReg(i)(j).fullAddr(i.U, io.dcuID, io.pcuID), mshrTableReg(i)(j).chiMes.channel, mshrTableReg(i)(j).chiMes.opcode, mshrTableReg(i)(j).mshrMes.state) } }

  // MSHRLock Timeout Check
  val cntLockReg = RegInit(VecInit(Seq.fill(djparam.nrMSHRSets) { 0.U(64.W) }))
  cntLockReg.zipWithIndex.foreach { case(c, i) => c := Mux(!mshrLockVecReg(i), 0.U , c + 1.U) }
  cntLockReg.zipWithIndex.foreach { case(c, i) => assert(c < TIMEOUT_MSLOCK.U, "MSHR LOCK [0x%x] TIMEOUT", i.U) }


// -------------------------------------------------- Perf Counter ------------------------------------------------------ //
  require(djparam.nrMSHRWays >= 4 & djparam.nrMSHRWays % 4 == 0)
  for (i <- 0 until djparam.nrMSHRSets) {
    for (j <- 0 until (djparam.nrMSHRWays / 4)) {
      XSPerfAccumulate(s"pcu_MSHRCtl_entry_group[${i}][${j}]_deal_req_cnt", io.req2Exu.fire & canReceiveReq & req2ExuMSet === i.U & (j * 4).U <= nodeReqInvWay & nodeReqInvWay <= (j * 4 + 3).U)
      XSPerfAccumulate(s"pcu_MHSRCtl_group[${i}]_deal_req_cnt", io.req2Exu.fire & canReceiveReq & req2ExuMSet === i.U)
      XSPerfAccumulate(s"pcu_MSHRCtl_group[${i}]_req_block_cnt", io.req2Exu.fire & req2ExuMSet === i.U & !nodeReqMatchVec.reduce(_ | _) & PopCount(nodeReqInvVec) === 0.U)
    }
  }
}