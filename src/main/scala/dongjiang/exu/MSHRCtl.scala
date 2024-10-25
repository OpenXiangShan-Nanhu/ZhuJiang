package dongjiang.pcu.exu

import dongjiang._
import dongjiang.pcu._
import dongjiang.chi._
import dongjiang.pcu.exu.decode._
import dongjiang.utils.Encoder._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._


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
  // state
  val state           = UInt(MSHRState.width.W)
  // addr
  val tag             = UInt(mshrTagBits.W)
  val bank            = UInt(bankBits.W)
  val lockDirSet      = Bool()
  // req mes
  val chiMes          = new MSHRCHIMesBundle()
  // resp mes
  val waitIntfVec     = Vec(nrIntf, Bool()) // Wait Snoop Resp or Req Resp
  val respMes         = new RespMesBundle()

  def isValid         = state =/= MSHRState.Free
  def isFree          = state === MSHRState.Free
  def isBeSend        = state === MSHRState.BeSend
  def isAlreadySend   = state === MSHRState.AlreadySend
  def isWaitResp      = state === MSHRState.WaitResp
  def isResp          = respMes.slvResp.valid | respMes.masResp.valid | respMes.fwdState.valid
  def isReq           = !isResp
  def respBeSend      = isBeSend & isResp
  def reqBeSend       = isBeSend & isReq

  def addr(x: UInt): UInt = { require(x.getWidth <= mshrSetBits); Cat(tag, x.asTypeOf(UInt(mshrSetBits.W)), bank, 0.U(offsetBits.W)) }
  def dirBank(x: UInt): UInt = { require(x.getWidth <= mshrSetBits); getDirBank(addr(x)) }
}


class MSHRCtl()(implicit p: Parameters) extends DJModule {
  // --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val sliceId       = Input(UInt(bankBits.W))
    // Req To Slice
    val req2Slice     = Flipped(Decoupled(new Req2SliceBundle()))
    // Ack To Node
    val reqAck2node   = Decoupled(new ReqAck2NodeBundle())
    // Resp To Slice
    val resp2Slice    = Flipped(Decoupled(new Resp2SliceBundle()))
    // Task To MainPipe
    val pipeTask      = Vec(2, Decoupled(new PipeTaskBundle()))
    // Update Task From MainPipe
    val updMSHR       = Flipped(Decoupled(new UpdateMSHRReqBundle()))
    val updLockMSHR   = Vec(2, Flipped(Valid(new MSHRSetBundle))) // TODO: It should be locked according to dir set but not mshr set
    // Directory Read Req
    val earlyRReqVec  = Vec(djparam.nrDirBank, Decoupled())
    val dirRead       = Vec(2, Valid(new DirReadBundle()))
    // Directory Read MSHR Set Mes
    val dirReadMshr   = Vec(2, Flipped(Valid(new DirReadMSHRBundle())))
    val mshrResp2Dir  = Vec(2, Valid(new MSHRRespDirBundle()))
  })

  // TODO: Delete the following code when the coding is complete
  dontTouch(io)

  // ------------------------ Module declaration ------------------------- //
  val reqAck_s0_q           = Module(new Queue(new ReqAck2NodeBundle(), entries = 4, flow = false, pipe = true))

  // --------------------- Reg / Wire declaration ------------------------ //
  // mshrTable
  val mshrTableReg          = RegInit(VecInit(Seq.fill(djparam.nrMSHRSets) { VecInit(Seq.fill(djparam.nrMSHRWays) { 0.U.asTypeOf(new MSHREntry()) }) }))
  val mshrLockVecReg        = RegInit(VecInit(Seq.fill(djparam.nrMSHRSets) { false.B })) // TODO: nrMSHRSets -> nrDirSet
  // Transfer Req From Node To MSHREntry
  val mshrAlloc_s0          = WireInit(0.U.asTypeOf(new MSHREntry()))
  // task s0
  val dirCanRecVec          = Wire(Vec(djparam.nrDirBank, Bool()))
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
  val reqAlreadyReadDirReg  = RegInit(false.B)
  val respAlreadyReadDirReg = RegInit(false.B)
  // dir read mshr
  val resp2dirRegVec        = RegInit(0.U.asTypeOf(io.mshrResp2Dir))


  // ---------------------------------------------------------------------------------------------------------------------- //
  // --------------------------------------- S0: Receive Req From Node or Let It Retry ------------------------------------ //
  // ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get MSHR Mes
   */
  // mshrTableReg
  val nodeReqMatchVec = mshrTableReg(io.req2Slice.bits.mSet).map { case m =>
    val hit = Wire(Bool())
    when(m.lockDirSet) {
      if(maxDirSetBits > mshrSetBits) {
        hit := m.tag(maxDirSetBits - mshrSetBits - 1, 0) === io.req2Slice.bits.mTag(maxDirSetBits - mshrSetBits - 1, 0)
      } else {
        hit := true.B
      }
    }.otherwise {
      hit := m.tag === io.req2Slice.bits.mTag & m.bank === io.req2Slice.bits.mBank
    }
    hit & m.isValid
  }
  val nodeReqInvVec   = mshrTableReg(io.req2Slice.bits.mSet).map(_.isFree)
  val nodeReqInvWay   = PriorityEncoder(nodeReqInvVec)
  // mshrLockVecReg
  val lockMatch       = mshrLockVecReg(io.req2Slice.bits.mSet)


  /*
   * Get Block Message
   */
  val blockByMHSR     = nodeReqMatchVec.reduce(_ | _) | lockMatch
  val canReceiveNode  = !blockByMHSR & PopCount(nodeReqInvVec) > 0.U


  /*
   * Transfer Req From Node To MSHREntry
   */
  mshrAlloc_s0.tag      := io.req2Slice.bits.mTag
  mshrAlloc_s0.bank     := io.req2Slice.bits.mBank
  mshrAlloc_s0.chiMes   := io.req2Slice.bits
  when(io.req2Slice.bits.isReq & CHIOp.REQ.isWriteX(io.req2Slice.bits.opcode)) {
    mshrAlloc_s0.respMes.slvDBID.valid := true.B
    mshrAlloc_s0.respMes.slvDBID.bits  := io.req2Slice.bits.dbid
  }



  /*
   * Receive Req From Node and Determine if it needs retry
   * TODO: Setting blocking timer
   */
  io.req2Slice.ready            := reqAck_s0_q.io.enq.ready
  reqAck_s0_q.io.enq.valid      := io.req2Slice.valid
  reqAck_s0_q.io.enq.bits.retry := !canReceiveNode
  reqAck_s0_q.io.enq.bits.to    := io.req2Slice.bits.from
  reqAck_s0_q.io.enq.bits.pcuId := io.req2Slice.bits.pcuId
  io.reqAck2node                <> reqAck_s0_q.io.deq


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
          when(io.updMSHR.valid & !io.updMSHR.bits.isRetry & io.updMSHR.bits.mSet === i.U & io.updMSHR.bits.mshrWay === j.U) {
            // req
            when(io.updMSHR.bits.hasNewReq) {
              m                 := 0.U.asTypeOf(m)
              m.chiMes.opcode   := io.updMSHR.bits.opcode
              m.chiMes.channel  := io.updMSHR.bits.channel
              m.tag             := io.updMSHR.bits.mTag
              m.bank            := io.updMSHR.bits.mBank
              m.lockDirSet      := io.updMSHR.bits.lockDirSet
            }
            // req or update
            m.respMes           := 0.U.asTypeOf(m.respMes)
            m.waitIntfVec       := io.updMSHR.bits.waitIntfVec
            assert(PopCount(m.waitIntfVec) === 0.U, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.addr(i.U), m.chiMes.channel, m.chiMes.opcode, m.state)
            assert(m.isAlreadySend, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.addr(i.U), m.chiMes.channel, m.chiMes.opcode, m.state)
            /*
             * Resp Update mshrTable value
             */
          }.elsewhen(io.resp2Slice.valid & io.resp2Slice.bits.mshrMatch(i.U, j.U)) {
            assert(!io.resp2Slice.bits.isUpdate, "TODO")
            // Recovery of pending intf identifiers
            m.waitIntfVec(io.resp2Slice.bits.from.intfId) := false.B
            // Record Resp Mes
            when(io.resp2Slice.bits.isSnpResp) {
              m.respMes.slvResp.valid   := true.B
              m.respMes.slvResp.bits    := io.resp2Slice.bits.resp
              m.respMes.slvDBID.valid   := io.resp2Slice.bits.hasData
              m.respMes.slvDBID.bits    := io.resp2Slice.bits.dbid
              m.respMes.fwdState.valid  := io.resp2Slice.bits.fwdState.valid | m.respMes.fwdState.valid
              m.respMes.fwdState.bits   := Mux(io.resp2Slice.bits.fwdState.valid, io.resp2Slice.bits.fwdState.bits, m.respMes.fwdState.bits)
            }.elsewhen(io.resp2Slice.bits.isReqResp) {
              m.respMes.masResp.valid   := true.B
              m.respMes.masResp.bits    := io.resp2Slice.bits.resp
              m.respMes.masDBID.valid   := io.resp2Slice.bits.hasData
              m.respMes.masDBID.bits    := io.resp2Slice.bits.dbid
            }.elsewhen(io.resp2Slice.bits.isWriResp) {
              assert(PopCount(m.waitIntfVec) === 1.U, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.addr(i.U), m.chiMes.channel, m.chiMes.opcode, m.state)
              assert(m.respMes.noRespValid, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.addr(i.U), m.chiMes.channel, m.chiMes.opcode, m.state)
              // Nothing to do and State Will be Free
            }
            assert(m.waitIntfVec(io.resp2Slice.bits.from.intfId), s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.addr(i.U), m.chiMes.channel, m.chiMes.opcode, m.state)
            assert(m.isWaitResp, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.addr(i.U), m.chiMes.channel, m.chiMes.opcode, m.state)
            /*
             * Receive Node Req
             */
          }.elsewhen(io.req2Slice.fire & canReceiveNode & i.U === io.req2Slice.bits.mSet & j.U === nodeReqInvWay) {
            m := 0.U.asTypeOf(m)
            m := mshrAlloc_s0
            assert(m.isFree, s"MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x]", i.U, j.U, m.addr(i.U), m.chiMes.channel, m.chiMes.opcode, m.state)
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
  io.resp2Slice.ready := true.B


  /*
   * Update Lock Vec
   */
  mshrLockVecReg.zipWithIndex.foreach {
    case(lock, i) =>
      when((io.updLockMSHR(0).valid & io.updLockMSHR(0).bits.mshrSet === i.U) | (io.updLockMSHR(1).valid & io.updLockMSHR(1).bits.mshrSet === i.U)) {
        lock := false.B
        assert(lock)
      }.elsewhen((taskReq_s0.valid & canGoReq_s0 & taskReq_s0.bits.readDir & taskReq_s0.bits.mSet === i.U) | // Req Fire
        (taskResp_s0.valid & canGoResp_s0 & taskResp_s0.bits.readDir & taskResp_s0.bits.mSet === i.U)) { // Resp Fire
        lock := true.B
      }
  }
  assert(Mux(io.updLockMSHR(0).valid & io.updLockMSHR(1).valid, !(io.updLockMSHR(0).bits.mshrSet === io.updLockMSHR(1).bits.mshrSet), true.B))



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
          switch(m.state) {
            // Free
            is(MSHRState.Free) {
              val nodeHit = io.req2Slice.valid & canReceiveNode & i.U === io.req2Slice.bits.mSet & j.U === nodeReqInvWay
              m.state     := Mux(nodeHit, MSHRState.BeSend, MSHRState.Free)
            }
            // BeSend
            is(MSHRState.BeSend) {
              val reqhit  = taskReq_s0.valid & canGoReq_s0 & taskReq_s0.bits.mSet === i.U & taskReq_s0.bits.mshrWay === j.U
              val resphit = taskResp_s0.valid & canGoResp_s0 & taskResp_s0.bits.mSet === i.U & taskResp_s0.bits.mshrWay === j.U
              m.state     := Mux(reqhit | resphit, MSHRState.AlreadySend, MSHRState.BeSend)
            }
            // AlreadySend
            is(MSHRState.AlreadySend) {
              val hit     = io.updMSHR.valid & io.updMSHR.bits.mSet === i.U & io.updMSHR.bits.mshrWay === j.U
              val retry   = hit & io.updMSHR.bits.isRetry
              val update  = hit & io.updMSHR.bits.isUpdate
              val clean   = hit & io.updMSHR.bits.isClean
              m.state     := Mux(retry, MSHRState.BeSend,
                Mux(update, MSHRState.WaitResp,
                  Mux(clean, MSHRState.Free, MSHRState.AlreadySend)))
            }
            // WaitResp
            is(MSHRState.WaitResp) {
              val hit     = !m.waitIntfVec.reduce(_ | _)
              val noResp  = m.respMes.noRespValid
              m.state     := Mux(hit, Mux(noResp, MSHRState.Free, MSHRState.BeSend), MSHRState.WaitResp)
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
  // TODO: Consider creating a copy of shiftReg in MSHR
  dirCanRecVec            := io.earlyRReqVec.map(_.ready)
  // TODO: if dont need to read Dir, it should not be ctrl by mshrLockVecReg
  mshrTableReg.zip(reqWillSendVecVec).zipWithIndex.foreach  { case((m, v), i) => m.zip(v).foreach { case(m, v) => v := m.reqBeSend & dirCanRecVec(m.dirBank(i.U)) & !mshrLockVecReg(i) } }
  mshrTableReg.zip(respWillSendVecVec).zipWithIndex.foreach { case((m, v), i) => m.zip(v).foreach { case(m, v) => v := m.respBeSend & dirCanRecVec(m.dirBank(i.U)) & !mshrLockVecReg(i) } }


  /*
   * Get task_s0(resp) from MSHRTable
   */
  val respSendSet         = RREncoder(respWillSendVecVec.map(_.reduce(_ | _))); dontTouch(respSendSet)
  val respSendWay         = RREncoder(respWillSendVecVec(respSendSet)); dontTouch(respSendWay)
  val taskResp            = mshrTableReg(respSendSet)(respSendWay)
  val taskRespValid       = respWillSendVecVec.map(_.reduce(_ | _)).reduce(_ | _)


  /*
   * Get task_s0(req) from MSHRTable
   */
  val reqSendSet          = RREncoder(reqWillSendVecVec.map(_.reduce(_ | _))); dontTouch(reqSendSet)
  val reqSendWay          = RREncoder(reqWillSendVecVec(reqSendSet)); dontTouch(reqSendWay)
  val taskReq             = mshrTableReg(reqSendSet)(reqSendWay)
  val taskReqValid        = reqWillSendVecVec.map(_.reduce(_ | _)).reduce(_ | _)


  /*
   * Select resp task_s0
   */
  taskResp_s0.valid         := taskRespValid
  taskResp_s0.bits.readDir  := true.B // TODO
  taskResp_s0.bits.addr     := taskResp.addr(respSendSet)
  taskResp_s0.bits.pipeId   := PipeID.RESP
  taskResp_s0.bits.mshrWay  := respSendWay
  taskResp_s0.bits.reqMes   := taskResp.chiMes
  taskResp_s0.bits.respMes  := taskResp.respMes
  canGoResp_s0              := canGoResp_s1 | !taskResp_s1_g.valid


  /*
   * Select req task_s0
   */
  taskReq_s0.valid          := taskReqValid & !(taskResp_s0.valid & taskResp_s0.bits.dirBank === taskReq_s0.bits.dirBank)
  taskReq_s0.bits.readDir   := true.B // TODO
  taskReq_s0.bits.addr      := taskReq.addr(reqSendSet)
  taskReq_s0.bits.pipeId    := PipeID.REQ
  taskReq_s0.bits.mshrWay   := reqSendWay
  taskReq_s0.bits.reqMes    := taskReq.chiMes
  taskReq_s0.bits.respMes   := taskReq.respMes
  canGoReq_s0               := canGoReq_s1 | !taskReq_s1_g.valid

  /*
   * Read Directory Early Req
   */
  io.earlyRReqVec.map(_.valid).zipWithIndex.foreach {
    case(v, i) =>
      v := (taskResp_s0.valid & canGoResp_s0 & taskResp_s0.bits.dirBank === i.U) | (taskReq_s0.valid & canGoReq_s0 & taskReq_s0.bits.dirBank === i.U)
  }
  assert(PopCount(io.earlyRReqVec.map(_.fire)) === PopCount(Seq(taskResp_s0.valid & canGoResp_s0, taskReq_s0.valid & canGoReq_s0)))


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
  respAlreadyReadDirReg           := Mux(respAlreadyReadDirReg, taskResp_s0.valid & canGoResp_s0, io.pipeTask(0).fire & !canGoResp_s1)
  io.pipeTask(PipeID.RESP).valid  := taskResp_s1_g.valid
  io.pipeTask(PipeID.RESP).bits   := taskResp_s1_g.bits
  // req
  reqAlreadyReadDirReg            := Mux(reqAlreadyReadDirReg, taskReq_s0.valid & canGoReq_s0, io.pipeTask(1).fire & !canGoReq_s1)
  io.pipeTask(PipeID.REQ).valid   := taskReq_s1_g.valid
  io.pipeTask(PipeID.REQ).bits    := taskReq_s1_g.bits

  /*
   * Read Directory
   */
  // resp
  io.dirRead(PipeID.RESP).valid         := taskResp_s1_g.valid & !respAlreadyReadDirReg
  io.dirRead(PipeID.RESP).bits.addr     := taskResp_s1_g.bits.addr
  io.dirRead(PipeID.RESP).bits.mshrWay  := taskResp_s1_g.bits.mshrWay
  io.dirRead(PipeID.RESP).bits.pipeId   := taskResp_s1_g.bits.pipeId
  // req
  io.dirRead(PipeID.REQ).valid          := taskReq_s1_g.valid & !reqAlreadyReadDirReg
  io.dirRead(PipeID.REQ).bits.addr      := taskReq_s1_g.bits.addr
  io.dirRead(PipeID.REQ).bits.mshrWay   := taskReq_s1_g.bits.mshrWay
  io.dirRead(PipeID.REQ).bits.pipeId    := taskReq_s1_g.bits.pipeId



  // ------------------------ S2: Dir Read MSHR and MSHR Resp to Dir --------------------------//
  io.dirReadMshr.zip(resp2dirRegVec).foreach {
    case (read, resp) =>
      when(read.valid) {
        resp.valid          := true.B
        resp.bits.pipeId    := read.bits.pipeId
        resp.bits.dirBankId := read.bits.dirBankId
        resp.bits.addrs.zipWithIndex.foreach {
          case (r, i) =>
            r.valid := mshrTableReg(read.bits.mshrSet)(i).isValid
            r.bits  := mshrTableReg(read.bits.mshrSet)(i).addr(read.bits.mshrSet)
        }
      }.otherwise {
        resp.valid    := false.B
        resp.bits     := 0.U.asTypeOf(resp.bits)
      }
  }
  io.mshrResp2Dir     := resp2dirRegVec


  // ---------------------------  Assertion  -------------------------------- //
  // MSHR Timeout Check
  val cntMSHRReg = RegInit(VecInit(Seq.fill(djparam.nrMSHRSets) { VecInit(Seq.fill(djparam.nrMSHRWays) { 0.U(64.W) }) }))
  cntMSHRReg.zipWithIndex.foreach { case (c0, i) => c0.zipWithIndex.foreach { case(c1, j) => c1 := Mux(mshrTableReg(i)(j).isFree, 0.U, c1 + 1.U)  } }
  cntMSHRReg.zipWithIndex.foreach { case (c0, i) => c0.zipWithIndex.foreach { case(c1, j) => assert(c1 < TIMEOUT_MSHR.U, "MSHR[0x%x][0x%x] ADDR[0x%x] CHANNEL[0x%x] OP[0x%x] STATE[0x%x] TIMEOUT", i.U, j.U, mshrTableReg(i)(j).addr(i.U), mshrTableReg(i)(j).chiMes.channel, mshrTableReg(i)(j).chiMes.opcode, mshrTableReg(i)(j).state) } }

  // MSHRLock Timeout Check
  val cntLockReg = RegInit(VecInit(Seq.fill(djparam.nrMSHRSets) { 0.U(64.W) }))
  cntLockReg.zipWithIndex.foreach { case(c, i) => c := Mux(!mshrLockVecReg(i), 0.U , c + 1.U) }
  cntLockReg.zipWithIndex.foreach { case(c, i) => assert(c < TIMEOUT_MSLOCK.U, "MSHR LOCK [0x%x] TIMEOUT", i.U) }
}