package dongjiang.pcu.exu

import zhujiang.chi.ReqOpcode._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import zhujiang.chi.SnpOpcode._
import zhujiang.chi._
import dongjiang._
import dongjiang.pcu._
import dongjiang.pcu.exu.decode._
import dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.ParallelLookUp
import xs.utils.perf.{DebugOptions, DebugOptionsKey}
import xs.utils.perf.HasPerfLogging


class ProcessPipe(implicit p: Parameters) extends DJModule with HasPerfLogging {
// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val dcuID       = Input(UInt(dcuBankBits.W))
    val pcuID       = Input(UInt(pcuBankBits.W))
    // Req To DataBuffer
    val dbRCReq     = Decoupled(new DBRCReq())
    // Resp From Directory
    val dirResp     = Flipped(Valid(new DirRespBundle()))
    // Write Req To Directory
    val dirWrite    = new DirWriteBundle()
    // Task From MSHR
    val task        = Flipped(Decoupled(new PipeTaskBundle()))
    // Update Task To MSHR
    val updMSHR     = Decoupled(new UpdateMSHRReqBundle())
    val updLockMSHR = Valid(UInt(minDirSetBits.W))
    // Req To Node
    val req2Intf    = Decoupled(new Req2IntfBundle())
    // Resp To Node
    val resp2Intf   = Decoupled(new Resp2IntfBundle())
  })

  // TODO: Delete the following code when the coding is complete
  dontTouch(io)

// --------------------- Modules declaration ------------------------//
  val taskQ   = Module(new Queue(new PipeTaskBundle(), entries = djparam.nrPipeTaskQueue, pipe = true, flow = true))
  val dirResQ = Module(new Queue(new DirRespBundle(), entries = djparam.nrPipeTaskQueue + 2, pipe = true, flow = true)) // one for mp_s1 read Dir before send task to mp_2, one for mp_s3

  dontTouch(taskQ.io.count)
  dontTouch(dirResQ.io.count)

// --------------------- Reg/Wire declaration ------------------------//
  // s2 signals
  val canGo_s2_req        = Wire(Bool())
  val canGo_s2_dir        = Wire(Bool())

  // s3 basic signals
  val canGo_s3            = Wire(Bool())
  val valid_s3            = Wire(Bool())
  val task_s3_needDir     = Wire(Bool())
  val task_s3             = WireInit(0.U.asTypeOf(Valid(new PipeTaskBundle()))); dontTouch(task_s3)
  val dirRes_s3           = WireInit(0.U.asTypeOf(Valid(new DirRespBundle()))); dontTouch(dirRes_s3)
  val srcMetaID_s3        = Wire(UInt((ccNodeIdBits+1).W))
  // s3 decode base signals
  val inst_s3             = Wire(new InstBundle()); dontTouch(inst_s3)
  val inst_req_s3         = WireInit(0.U.asTypeOf(new InstBundle()))
  val decode_s3           = Wire(new DecodeBundle()); dontTouch(decode_s3)
  val decode_req_s3       = Wire(new DecodeBundle())
  val snpNodeVec_s3       = WireInit(VecInit(Seq.fill(nrCcNode) { false.B }))
  // s3 execute(update MSHR) signals: task to do list
  val todo_s3             = WireInit(0.U.asTypeOf(new OperationsBundle())); dontTouch(todo_s3)
  val todo_s3_retry       = Wire(Bool()); dontTouch(todo_s3_retry)
  val todo_s3_replace     = Wire(Bool()); dontTouch(todo_s3_replace) // replace self Directory
  val todo_s3_sfEvict     = Wire(Bool()); dontTouch(todo_s3_sfEvict) // replace snoop filter
  val todo_s3_updateMSHR  = Wire(Bool()); dontTouch(todo_s3_updateMSHR)
  val todo_s3_cleanMSHR   = Wire(Bool()); dontTouch(todo_s3_cleanMSHR)
  val done_s3_g_updMSHR   = RegInit(false.B)


  // s4 basic signals
  val canGo_s4            = Wire(Bool())
  val valid_s4_g          = RegInit(false.B)
  // s4 signals get from s3
  val task_s4_g           = Reg(new PipeTaskBundle());    dontTouch(task_s4_g)
  val decode_s4_g         = Reg(new DecodeBundle());      dontTouch(decode_s4_g)
  val snpNodeVec_s4_g     = Reg(Vec(nrCcNode, Bool()));   dontTouch(snpNodeVec_s4_g)
  val dirRes_s4_g         = Reg(new DirRespBundle());     dontTouch(dirRes_s4_g)
  val needUnLockMSHR_s4_g = Reg(Bool());                  dontTouch(needUnLockMSHR_s4_g)
  val respType_s4_g       = Reg(UInt(RespType.width.W));  dontTouch(respType_s4_g)
  val srcMetaID_s4_g      = Reg(UInt(metaIdBits.W));      dontTouch(srcMetaID_s4_g)
  val todo_s4_g_replace   = Reg(Bool());                  dontTouch(todo_s4_g_replace) // replace self Directory
  val todo_s4_g_sfEvict   = Reg(Bool());                  dontTouch(todo_s4_g_sfEvict) // replace snoop filter
  // s4 execute signals: Set specific tasks value
  val dbid_s4             = Wire(Valid(UInt(dbIdBits.W)));  dontTouch(dbid_s4)
  val taskSnp_s4          = Wire(new Req2IntfBundle())
  val taskRD_s4           = WireInit(0.U.asTypeOf(new Req2IntfBundle()))
  val taskWD_s4           = WireInit(0.U.asTypeOf(new Req2IntfBundle()))
  val rcDBReq_s4          = WireInit(0.U.asTypeOf(new DBRCReq()))
  val readDCU_s4          = WireInit(0.U.asTypeOf(new Req2IntfBundle()))
  val writeDCU_s4         = WireInit(0.U.asTypeOf(new Req2IntfBundle()))
  val wSDir_s4            = WireInit(0.U.asTypeOf(io.dirWrite.s.bits))
  val wSFDir_s4           = WireInit(0.U.asTypeOf(io.dirWrite.sf.bits))
  val flush_s4            = WireInit(0.U.asTypeOf(new Req2IntfBundle()))
  val commit_s4           = WireInit(0.U.asTypeOf(new Resp2IntfBundle()))
  val taskRepl_s4         = WireInit(0.U.asTypeOf(new Req2IntfBundle()))
  val taskSnpEvict_s4     = WireInit(0.U.asTypeOf(new Req2IntfBundle()))
  // s4 execute signals: task to do list
  val todo_s4             = WireInit(0.U.asTypeOf(new OperationsBundle())); dontTouch(todo_s4)
  // s4 execute signals: Execute specific tasks
  val done_s4_g           = RegInit(0.U.asTypeOf(new OperationsBundle()))
  val done_s4_g_replace   = RegInit(false.B)
  val done_s4_g_sfEvict   = RegInit(false.B)
  val reqBeSend_s4        = Wire(Vec(7, new Req2IntfBundle()))


  /*
   * for Debug
   */
  // s3
  val task_s3_dbg_addr    = Wire(UInt(fullAddrBits.W))
  task_s3_dbg_addr        := task_s3.bits.taskMes.fullAddr(io.dcuID, io.pcuID)
  if (p(DebugOptionsKey).EnableDebug) dontTouch(task_s3_dbg_addr)
  // s4
  val task_s4_dbg_addr    = Wire(UInt(fullAddrBits.W))
  task_s4_dbg_addr        := task_s4_g.taskMes.fullAddr(io.dcuID, io.pcuID)
  if (p(DebugOptionsKey).EnableDebug) dontTouch(task_s4_dbg_addr)


// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- S2: Buffer input task/dirRes ----------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  // task queue
  taskQ.io.enq          <> io.task
  taskQ.io.deq.ready    := canGo_s2_req

  // dir result queue
  dirResQ.io.enq.valid  := io.dirResp.valid
  dirResQ.io.enq.bits   := io.dirResp.bits
  dirResQ.io.deq.ready  := canGo_s2_dir
  assert(Mux(io.dirResp.valid, dirResQ.io.enq.ready, true.B))

  // Can Go Signals
  canGo_s2_req          := canGo_s3 & (dirResQ.io.deq.valid | !task_s3_needDir)
  canGo_s2_dir          := canGo_s3 & taskQ.io.deq.valid & task_s3_needDir


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------- S3_Receive: Receive task and dirRes from s2 -------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Recieve task_s2
   */
  task_s3.valid         := taskQ.io.deq.valid
  task_s3.bits          := Mux(taskQ.io.deq.valid, taskQ.io.deq.bits, 0.U.asTypeOf(task_s3.bits))
  task_s3_needDir       := task_s3.bits.taskMes.readDir

  /*
   * Recieve dirRes
   */
  dirRes_s3.valid       := dirResQ.io.deq.valid
  dirRes_s3.bits        := Mux(dirRes_s3.valid, dirResQ.io.deq.bits, 0.U.asTypeOf(dirRes_s3.bits))

  /*
   * S3 base ctrl logic
   */
  val s2Fire            = taskQ.io.deq.fire
  valid_s3              := Mux(task_s3.bits.taskMes.readDir, task_s3.valid & dirRes_s3.valid, task_s3.valid)
  canGo_s3              := (canGo_s4 | !valid_s4_g) & (io.updMSHR.fire | done_s3_g_updMSHR)


// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- S3_Decode: Decode by task Message and Dir Result ------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Parse Dir Result
   */
  srcMetaID_s3    := Mux(fromRniNode(task_s3.bits.chiIndex.nodeID), Fill(srcMetaID_s3.getWidth, 1.U(1.W)), getMetaIDByNodeID(task_s3.bits.chiIndex.nodeID))
  assert(fromCcNode(task_s3.bits.chiIndex.nodeID) | fromRniNode(task_s3.bits.chiIndex.nodeID) | !task_s3.valid | (task_s3.bits.chiMes.opcode === SnpUniqueEvict & task_s3.bits.chiMes.isSnp))

  val srcHit_s3   = dirRes_s3.bits.sf.hit & !dirRes_s3.bits.sf.metaVec(srcMetaID_s3).isInvalid & !fromRniNode(task_s3.bits.chiIndex.nodeID)
  val srcState_s3 = Mux(srcHit_s3, dirRes_s3.bits.sf.metaVec(srcMetaID_s3).state, ChiState.I)

  val othHit_s3   = dirRes_s3.bits.sf.hit & (PopCount(dirRes_s3.bits.sf.metaVec.map(!_.isInvalid)) > srcHit_s3.asUInt)
  val sfHitID_s3  = PriorityEncoder(dirRes_s3.bits.sf.metaVec.map(!_.isInvalid))
  val othState_s3 = Mux(othHit_s3, dirRes_s3.bits.sf.metaVec(sfHitID_s3).state, ChiState.I)

  val hnHit_s3    = dirRes_s3.bits.s.hit
  val hnState_s3  = Mux(hnHit_s3, dirRes_s3.bits.s.metaVec(0).state, ChiState.I)

  /*
   * Set Inst value
   */
  val taskIsWriPtl_s3 = isWriXPtl(task_s3.bits.chiMes.opcode) & task_s3.bits.chiMes.isReq
  val taskIsCMO_s3    = isCMO(task_s3.bits.chiMes.opcode)     & task_s3.bits.chiMes.isReq
  val taskIsCB_s3     = isCBX(task_s3.bits.chiMes.opcode)     & task_s3.bits.chiMes.isReq
  val taskIsAtomic_s3 = isAtomicX(task_s3.bits.chiMes.opcode) & task_s3.bits.chiMes.isReq

  assert(Mux(taskIsWriPtl_s3 & task_s3.valid, !task_s3.bits.respMes.slvResp.valid, true.B))
  assert(Mux(taskIsWriPtl_s3 & task_s3.valid, task_s3.bits.respMes.slvDBID.valid, true.B))
  assert(Mux(taskIsAtomic_s3 & task_s3.valid, task_s3.bits.respMes.slvDBID.valid, true.B))
  assert(Mux(taskIsAtomic_s3 & task_s3.valid, !isAtomicStoreX(task_s3.bits.chiMes.opcode), true.B))


  inst_s3.channel     := task_s3.bits.chiMes.channel
  inst_s3.opcode      := Mux(taskIsAtomic_s3, AtomicLoadADD, task_s3.bits.chiMes.opcode) // When the task is an atomic operation, it is converted to AtomicLoadADD for decoding.
  inst_s3.srcState    := Mux(task_s3.bits.taskMes.readDir, srcState_s3, ChiState.I)
  inst_s3.othState    := Mux(task_s3.bits.taskMes.readDir, othState_s3, ChiState.I)
  inst_s3.hnState     := Mux(task_s3.bits.taskMes.readDir, hnState_s3, ChiState.I)
  inst_s3.respType    := Cat(taskIsCB_s3 & task_s3.bits.respMes.slvResp.valid,   // Copy Back Resp
                             task_s3.bits.respMes.mstResp.valid,              // Read Down Resp
                             task_s3.bits.respMes.fwdState.valid,             // Snoop Fwd Resp
                             task_s3.bits.respMes.slvResp.valid & !taskIsCB_s3)  // Snoop Resp
  inst_s3.slvResp     := task_s3.bits.respMes.slvResp.bits
  inst_s3.fwdState    := task_s3.bits.respMes.fwdState.bits
  inst_s3.mstResp     := task_s3.bits.respMes.mstResp.bits
  inst_s3.respHasData := task_s3.bits.respMes.slvDBID.valid | task_s3.bits.respMes.mstDBID.valid


  /*
   * Get Decode Result
   */
  // table
  var table = LoaclSnpUniqueEvictDecode.table ++ LoaclDatalessDecode.table ++ LoaclWriteDecode.table ++ LoaclAtomicDecode.table
  if(djparam.openDCT) table = table ++ LocalReadWithDCTDecode.table
  else table = table ++ LocalReadDecode.table
  // require
  table.zipWithIndex.foreach { case(t, i) =>
    val width0 = t._1.getWidth
    val width1 = inst_s3.asUInt.getWidth
    require(width0 == width1,  s"Index: $i: Inst Width $width0 =/= $width1")
  }
  table.zipWithIndex.foreach { case (t, i) =>
    val width0 = t._2.getWidth
    val width1 = decode_s3.asUInt.getWidth
    require(width0 == width1, s"Index: $i: Decode Width $width0 =/= $width1")
  }
  table.zipWithIndex.foreach { case (t, i) =>
    val inst   = t._1.asTypeOf(new InstBundle())
    val decode = t._1.asTypeOf(new DecodeBundle())
    assert(!(decode.cleanDB & decode.writeDCU), s"Index: $i")
  }
  // deocde
  decode_s3.decode(inst_s3, table)
  decode_req_s3.decode(inst_req_s3, table)
  // assert
  when(valid_s3) { assert(decode_s3.asUInt =/= Code.ERROE,
    "\n\nADDR[0x%x] DECODE ERROR: No inst match in decode table\n" +
      "INST: CHNL[0x%x] OP[0x%x] SRC[0x%x] OTH[0x%x] HN[0x%x] RESP[0x%x] DATA[0x%x] SLV[0x%x] FWD[0x%x] MST[0x%x]\n", task_s3.bits.taskMes.fullAddr(io.dcuID, io.pcuID),
    inst_s3.channel, inst_s3.opcode, inst_s3.srcState, inst_s3.othState, inst_s3.hnState, inst_s3.respType, inst_s3.respHasData, inst_s3.slvResp, inst_s3.fwdState, inst_s3.mstResp) }
  when(valid_s3) { when(decode_s3.wSDir)  { assert(decode_s3.commit | (inst_s3.opcode === SnpUniqueEvict & inst_s3.channel === CHIChannel.SNP) | (isWriteX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ) | (isReadX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ & djparam.openDMT.asBool)) } }
  when(valid_s3) { when(decode_s3.wSFDir) { assert(decode_s3.commit | (isWriteX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ) | (isReadX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ & djparam.openDMT.asBool)) } }



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------ S3_Req_Decode: Get Snp Target ----------------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Decode Req
   */
  inst_req_s3.channel   := inst_s3.channel
  inst_req_s3.opcode    := inst_s3.opcode
  inst_req_s3.srcState  := inst_s3.srcState
  inst_req_s3.othState  := inst_s3.othState
  inst_req_s3.hnState   := inst_s3.hnState
  inst_req_s3.respType  := RespType.NotResp

  /*
   * Set Snoop Target Value
   */
  val rnHitVec_s3     = dirRes_s3.bits.sf.metaVec.map(!_.isInvalid)
  val rnHitWithoutSrc = rnHitVec_s3.zipWithIndex.map { case(hit, i) => hit & i.U =/= srcMetaID_s3 }
  when(decode_req_s3.snpTgt === SnpTgt.ALL)       { snpNodeVec_s3 := rnHitVec_s3 }
  .elsewhen(decode_req_s3.snpTgt === SnpTgt.OTH)  { snpNodeVec_s3 := rnHitWithoutSrc }
  .elsewhen(decode_req_s3.snpTgt === SnpTgt.ONE)  { snpNodeVec_s3 := PriorityEncoderOH(rnHitWithoutSrc) } // TODO: Can be Optimized
  .otherwise                                      { snpNodeVec_s3 := 0.U.asTypeOf(snpNodeVec_s3) }
  // assert
  when(valid_s3 & decode_s3.snoop){
    assert(dirRes_s3.bits.sf.hit)
    assert(decode_s3.snpTgt =/= SnpTgt.NONE)
  }



// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- S3_Execute: Update MSHR ------------------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Set todo_s3 value
   */
  when(valid_s3) { todo_s3 := decode_s3 }

  /*
   * Send Retry to MSHR When need write Dir but cant do it
   *
   */
  todo_s3_retry       := todo_s3.wSDir & dirRes_s3.bits.s.replRetry | todo_s3.wSFDir & dirRes_s3.bits.sf.replRetry; assert(Mux(valid_s3, !todo_s3_retry, true.B), "TODO")
  todo_s3_replace     := todo_s3.wSDir & !hnHit_s3 & !dirRes_s3.bits.s.metaVec(0).isInvalid & !todo_s3_retry // TODO: Only need to replace when it is Dirty
  todo_s3_sfEvict     := todo_s3.wSFDir & !srcHit_s3 & !othHit_s3 & dirRes_s3.bits.sf.metaVec.map(!_.isInvalid).reduce(_ | _) & !todo_s3_retry
  todo_s3_updateMSHR  := decode_s3.needWaitSlv | decode_s3.needWaitMst | todo_s3_replace | todo_s3_sfEvict
  todo_s3_cleanMSHR   := !(todo_s3_retry | todo_s3_updateMSHR)
  assert(Mux(valid_s3, PopCount(Seq(todo_s3_retry, todo_s3_updateMSHR, todo_s3_cleanMSHR)) === 1.U, true.B))
  assert(Mux(valid_s3, PopCount(Seq(todo_s3_replace, todo_s3_sfEvict)) <= 1.U, true.B))
  assert(Mux(valid_s3 & todo_s3_replace, todo_s3.writeDCU, true.B))

  /*
   * Update MSHR Mes or let task retry
   */
  io.updMSHR.bits.mshrSet     := task_s3.bits.taskMes.mSet
  io.updMSHR.bits.mshrWay     := task_s3.bits.taskMes.mshrWay
  io.updMSHR.bits.updType     := Mux(todo_s3_retry, UpdMSHRType.RETRY,  UpdMSHRType.UPD)
  io.updMSHR.bits.waitIntfVec := (Mux(decode_s3.needWaitSlv | todo_s3_sfEvict, UIntToOH(IncoID.LOCALSLV.U), (todo_s3.readDown | todo_s3.readDCU) & task_s3.bits.chiMes.expCompAck & djparam.openDMT.asBool) |
                                  Mux(decode_s3.needWaitMst | todo_s3_replace, UIntToOH(IncoID.LOCALMST.U), 0.U)).asBools
  io.updMSHR.bits.mTag        := Mux(todo_s3_replace | todo_s3_sfEvict, Mux(todo_s3_replace, dirRes_s3.bits.s.mTag, dirRes_s3.bits.sf.mTag), task_s3.bits.taskMes.mTag)
  assert(!((decode_s3.needWaitSlv | todo_s3_sfEvict) & ((todo_s3.readDown | todo_s3.readDCU) & task_s3.bits.chiMes.expCompAck & djparam.openDMT.B)))
  // Only Use In New Req
  io.updMSHR.bits.hasNewReq   := todo_s3_replace | todo_s3_sfEvict
  io.updMSHR.bits.opcode      := Mux(todo_s3_replace, Replace,            SnpUniqueEvict)
  io.updMSHR.bits.channel     := Mux(todo_s3_replace, CHIChannel.REQ,     CHIChannel.SNP)
  io.updMSHR.bits.needUpdLock := todo_s3_replace
  // Common
  io.updMSHR.valid            := valid_s3 & (todo_s3_retry | todo_s3_updateMSHR | todo_s3_cleanMSHR) & !done_s3_g_updMSHR
  done_s3_g_updMSHR           := Mux(s2Fire, false.B, done_s3_g_updMSHR | io.updMSHR.fire)
  


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------- S4_Receive: Receive task and dirRes from s3 -------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
  * S4 base ctrl logic
  */
  val s3Valid         = task_s3.valid & !todo_s3_retry
  val s3Fire          = s3Valid & canGo_s3
  valid_s4_g          := Mux(s3Fire, true.B, valid_s4_g & !canGo_s4)

  /*
   * Recieve s3 signals
   */
  task_s4_g           := Mux(s3Fire,  task_s3.bits,     task_s4_g)
  decode_s4_g         := Mux(s3Fire,  decode_s3,        decode_s4_g)
  snpNodeVec_s4_g     := Mux(s3Fire,  snpNodeVec_s3,    snpNodeVec_s4_g)
  dirRes_s4_g         := Mux(s3Fire,  dirRes_s3.bits,   dirRes_s4_g)
  todo_s4_g_replace   := Mux(s3Fire,  todo_s3_replace,  todo_s4_g_replace)
  todo_s4_g_sfEvict   := Mux(s3Fire,  todo_s3_sfEvict,  todo_s4_g_sfEvict)
  respType_s4_g       := Mux(s3Fire,  inst_s3.respType, respType_s4_g)
  srcMetaID_s4_g      := Mux(s3Fire,  srcMetaID_s3,     srcMetaID_s4_g)
  needUnLockMSHR_s4_g := Mux(s3Fire,  task_s3.bits.taskMes.readDir & !io.updMSHR.bits.needUpdLock, needUnLockMSHR_s4_g)
  
// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------- S4_Execute: Set specific tasks value -----------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  val taskIsWriPtl_s4 = isWriXPtl(task_s4_g.chiMes.opcode) & task_s4_g.chiMes.isReq
  val taskIsCMO_s4    = isCMO(task_s4_g.chiMes.opcode) & task_s4_g.chiMes.isReq
  val taskIsCB_s4     = isCBX(task_s4_g.chiMes.opcode) & task_s4_g.chiMes.isReq
  val taskIsAtomic_s4 = isAtomicX(task_s4_g.chiMes.opcode) & task_s4_g.chiMes.isReq


  /*
   * Send Snoop to RN-F
   */
  // get dbid
  dbid_s4.valid       := valid_s4_g & (task_s4_g.respMes.slvDBID.valid | task_s4_g.respMes.mstDBID.valid)
  dbid_s4.bits        := Mux(task_s4_g.respMes.slvDBID.valid, task_s4_g.respMes.slvDBID.bits, task_s4_g.respMes.mstDBID.bits)
  assert(!(task_s4_g.respMes.slvDBID.valid & task_s4_g.respMes.mstDBID.valid))


  // taskSnp_s4
  taskSnp_s4.chiIndex.txnID       := task_s4_g.chiIndex.txnID
  taskSnp_s4.chiIndex.nodeID      := snpNodeVec_s4_g.asUInt
  taskSnp_s4.chiIndex.beatOH      := "b11".U
  taskSnp_s4.chiMes.channel       := CHIChannel.SNP
  taskSnp_s4.chiMes.doNotGoToSD   := true.B
  taskSnp_s4.chiMes.retToSrc      := decode_s4_g.retToSrc
  taskSnp_s4.chiMes.fwdState      := decode_s4_g.fwdState
  taskSnp_s4.chiMes.expCompAck    := false.B
  taskSnp_s4.chiMes.opcode        := decode_s4_g.snpOp
  taskSnp_s4.chiMes.resp          := DontCare
  taskSnp_s4.from                 := io.dcuID
  taskSnp_s4.to                   := IncoID.LOCALSLV.U
  taskSnp_s4.pcuIndex.mshrSet     := DontCare
  taskSnp_s4.pcuIndex.mshrWay     := task_s4_g.taskMes.mshrWay
  taskSnp_s4.pcuIndex.dbID        := dbid_s4.bits
  taskSnp_s4.pcuIndex.entryID     := DontCare
  taskSnp_s4.pcuMes.useAddr       := task_s4_g.taskMes.useAddr
  taskSnp_s4.pcuMes.doDMT         := DontCare
  taskSnp_s4.pcuMes.selfWay       := DontCare
  taskSnp_s4.pcuMes.toDCU         := DontCare
  taskSnp_s4.pcuMes.hasPcuDBID    := dbid_s4.valid; assert(Mux(decode_s4_g.snoop & dbid_s4.valid & valid_s4_g, taskIsWriPtl_s4 | taskIsAtomic_s4, true.B))


  /*
   * Send Read to SN(DDRC) / HN-F(CSN)
   */
  taskRD_s4                       := DontCare
  taskRD_s4.chiIndex              := task_s4_g.chiIndex
  taskRD_s4.chiMes.channel        := CHIChannel.REQ
  taskRD_s4.chiMes.expCompAck     := false.B
  taskRD_s4.chiMes.opcode         := decode_s4_g.rdOp
  taskRD_s4.chiMes.resp           := ChiResp.UC
  taskRD_s4.from                  := io.dcuID
  taskRD_s4.to                    := IncoID.LOCALMST.U
  taskRD_s4.pcuIndex.mshrWay      := task_s4_g.taskMes.mshrWay
  taskRD_s4.pcuMes.useAddr        := task_s4_g.taskMes.useAddr
  taskRD_s4.pcuMes.doDMT          := djparam.openDMT.asBool
  taskRD_s4.pcuMes.toDCU          := false.B


  /*
   * Send Write / Dataless to SN(DDRC) / HN-F(CSN)
   */
  taskWD_s4                       := DontCare
  taskWD_s4.chiIndex              := task_s4_g.chiIndex
  taskWD_s4.chiIndex.beatOH       := task_s4_g.chiIndex.beatOH
  taskWD_s4.chiMes.channel        := CHIChannel.REQ
  taskWD_s4.chiMes.expCompAck     := false.B
  taskWD_s4.chiMes.opcode         := decode_s4_g.wdOp
  taskWD_s4.from                  := io.dcuID
  taskWD_s4.to                    := IncoID.LOCALMST.U
  taskWD_s4.pcuIndex.mshrWay      := task_s4_g.taskMes.mshrWay
  taskWD_s4.pcuIndex.dbID         := dbid_s4.bits; assert(Mux(valid_s4_g & decode_s4_g.writeDown, dbid_s4.valid, true.B))
  taskWD_s4.pcuMes.useAddr        := task_s4_g.taskMes.useAddr
  taskWD_s4.pcuMes.selfWay        := DontCare
  taskWD_s4.pcuMes.toDCU          := false.B


  /*
   * Send Read / Clean to DataBuffer
   */
  rcDBReq_s4.to         := IncoID.LOCALSLV.U
  rcDBReq_s4.isRead     := decode_s4_g.rDB2Src
  rcDBReq_s4.isClean    := decode_s4_g.cleanDB
  rcDBReq_s4.dbID       := dbid_s4.bits; assert(Mux(valid_s4_g & decode_s4_g.rDB2Src, dbid_s4.valid, true.B))
  rcDBReq_s4.rBeatOH    := task_s4_g.chiIndex.beatOH
  rcDBReq_s4.exAtomic   := taskIsAtomic_s4


  /*
   * Send Read to SN(DCU)
   */
  readDCU_s4                        := DontCare
  readDCU_s4.chiIndex               := task_s4_g.chiIndex
  readDCU_s4.chiMes.channel         := CHIChannel.REQ
  readDCU_s4.chiMes.expCompAck      := false.B
  readDCU_s4.chiMes.opcode          := decode_s4_g.rdOp
  readDCU_s4.chiMes.resp            := decode_s4_g.resp
  readDCU_s4.from                   := io.dcuID
  readDCU_s4.to                     := IncoID.LOCALMST.U
  readDCU_s4.pcuIndex.mshrWay       := task_s4_g.taskMes.mshrWay
  readDCU_s4.pcuMes.useAddr         := task_s4_g.taskMes.useAddr
  readDCU_s4.pcuMes.doDMT           := djparam.openDMT.asBool
  readDCU_s4.pcuMes.selfWay         := OHToUInt(dirRes_s4_g.s.wayOH)
  readDCU_s4.pcuMes.toDCU           := true.B


  /*
   * Send Write to SN(DCU)
   */
  val snpRespHasData                = RespType.isSnpX(respType_s4_g) & task_s4_g.respMes.slvDBID.valid & !taskIsCB_s4
  writeDCU_s4                       := DontCare
  writeDCU_s4.chiIndex              := task_s4_g.chiIndex
  writeDCU_s4.chiIndex.beatOH       := task_s4_g.chiIndex.beatOH | Mux(snpRespHasData, "b11".U, "b00".U)
  writeDCU_s4.chiMes.channel        := CHIChannel.REQ
  writeDCU_s4.chiMes.expCompAck     := false.B
  writeDCU_s4.chiMes.opcode         := decode_s4_g.wdOp
  writeDCU_s4.from                  := io.dcuID
  writeDCU_s4.to                    := IncoID.LOCALMST.U
  writeDCU_s4.pcuIndex.mshrWay      := task_s4_g.taskMes.mshrWay
  writeDCU_s4.pcuIndex.dbID         := dbid_s4.bits; assert(Mux(valid_s4_g & decode_s4_g.writeDCU, dbid_s4.valid, true.B))
  writeDCU_s4.pcuMes.useAddr        := task_s4_g.taskMes.useAddr
  writeDCU_s4.pcuMes.selfWay        := OHToUInt(dirRes_s4_g.s.wayOH)
  writeDCU_s4.pcuMes.toDCU          := true.B


  /*
   * Send Repl to SN(DCU)
   */
  taskRepl_s4                       := DontCare
  taskRepl_s4.chiIndex              := task_s4_g.chiIndex
  taskRepl_s4.chiIndex.beatOH       := "b11".U; assert(Mux(valid_s4_g & todo_s4_g_replace, task_s4_g.chiIndex.fullSize | snpRespHasData, true.B))
  taskRepl_s4.chiMes.channel        := CHIChannel.REQ
  taskRepl_s4.chiMes.expCompAck     := false.B
  taskRepl_s4.chiMes.opcode         := Replace
  taskRepl_s4.from                  := io.dcuID
  taskRepl_s4.to                    := IncoID.LOCALMST.U
  taskRepl_s4.pcuIndex.mshrWay      := task_s4_g.taskMes.mshrWay
  taskRepl_s4.pcuIndex.dbID         := dbid_s4.bits; assert(Mux(valid_s4_g & todo_s4_g_replace, dbid_s4.valid, true.B))
  taskRepl_s4.pcuMes.useAddr        := dirRes_s4_g.s.useAddr
  taskRepl_s4.pcuMes.selfWay        := OHToUInt(dirRes_s4_g.s.wayOH)
  taskRepl_s4.pcuMes.toDCU          := false.B

 /*
  * Send Flush to DCU
  */
  flush_s4                       := DontCare
  flush_s4.chiIndex.beatOH       := "b11".U
  flush_s4.chiMes.channel        := CHIChannel.REQ
  flush_s4.chiMes.expCompAck     := false.B
  flush_s4.chiMes.opcode         := FlushDCU
  flush_s4.from                  := io.dcuID
  flush_s4.to                    := IncoID.LOCALMST.U
  flush_s4.pcuIndex.mshrWay      := task_s4_g.taskMes.mshrWay
  flush_s4.pcuMes.useAddr        := task_s4_g.taskMes.useAddr
  flush_s4.pcuMes.selfWay        := OHToUInt(dirRes_s4_g.s.wayOH)
  flush_s4.pcuMes.toDCU          := false.B


  /*
   * Send Commit to Intf
   */
  commit_s4                       := DontCare
  commit_s4.chiIndex              := task_s4_g.chiIndex
  commit_s4.chiMes.channel        := decode_s4_g.respChnl
  commit_s4.chiMes.expCompAck     := task_s4_g.chiMes.expCompAck
  commit_s4.chiMes.opcode         := decode_s4_g.respOp
  commit_s4.chiMes.resp           := decode_s4_g.resp
  commit_s4.from                  := io.dcuID
  commit_s4.to                    := IncoID.LOCALSLV.U
  commit_s4.pcuIndex.dbID         := dbid_s4.bits; assert(Mux(valid_s4_g & decode_s4_g.commit & decode_s4_g.respChnl === CHIChannel.DAT, dbid_s4.valid, true.B))
  commit_s4.pcuIndex.mshrSet      := task_s4_g.taskMes.mSet
  commit_s4.pcuIndex.mshrWay      := task_s4_g.taskMes.mshrWay
  commit_s4.pcuMes.useAddr        := task_s4_g.taskMes.useAddr


  /*
   * Send Snoop Evict to RN-F
   */
  val rnHitVec_s4                       = Wire(Vec(nrCcNode, Bool()))
  rnHitVec_s4                           := dirRes_s4_g.sf.metaVec.map(!_.isInvalid)

  taskSnpEvict_s4                       := DontCare
  taskSnpEvict_s4.chiIndex.txnID        := task_s4_g.chiIndex.txnID
  taskSnpEvict_s4.chiIndex.nodeID       := rnHitVec_s4.asUInt
  taskSnpEvict_s4.chiIndex.beatOH       := "b11".U
  taskSnpEvict_s4.chiMes.channel        := CHIChannel.SNP
  taskSnpEvict_s4.chiMes.doNotGoToSD    := true.B
  taskSnpEvict_s4.chiMes.retToSrc       := true.B
  taskSnpEvict_s4.chiMes.opcode         := SnpUnique
  taskSnpEvict_s4.from                  := io.dcuID
  taskSnpEvict_s4.to                    := IncoID.LOCALSLV.U
  taskSnpEvict_s4.pcuIndex.mshrWay      := task_s4_g.taskMes.mshrWay
  taskSnpEvict_s4.pcuMes.useAddr        := dirRes_s4_g.sf.useAddr


  /*
   * Set Write to Self Directory Task Value
   */
  wSDir_s4.useAddr          := task_s4_g.taskMes.useAddr
  wSDir_s4.wayOH            := dirRes_s4_g.s.wayOH
  wSDir_s4.metaVec(0).state := decode_s4_g.hnState
  wSDir_s4.replMes          := dirRes_s4_g.s.replMes


  /*
   * Set Write to Snoop Filter Directory Task Value
   */
  wSFDir_s4.useAddr         := task_s4_g.taskMes.useAddr
  wSFDir_s4.wayOH           := dirRes_s4_g.sf.wayOH
  wSFDir_s4.replMes         := dirRes_s4_g.sf.replMes
  wSFDir_s4.metaVec.zipWithIndex.foreach {
    case(a, i) =>
      when(RespType.isSnpX(respType_s4_g)) {
        when(snpNodeVec_s4_g(i))          { a.state := decode_s4_g.othState }
        .elsewhen(i.U === srcMetaID_s4_g) { a.state := decode_s4_g.srcState }
        .otherwise                        { a.state := dirRes_s4_g.sf.metaVec(i).state }
        assert(Mux(valid_s4_g & decode_s4_g.wSFDir, RespType.isSnpX(respType_s4_g) | RespType.isCB(respType_s4_g), true.B))
      }.otherwise {
        when(i.U === srcMetaID_s4_g)      { a.state := decode_s4_g.srcState }
        .otherwise                        { a.state := dirRes_s4_g.sf.metaVec(i).state; assert(Mux(valid_s4_g & decode_s4_g.wSFDir, a.state === decode_s4_g.othState | a.state === ChiState.I, true.B)) }
      }
  }



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------ S3_Execute: Execute specific tasks value based on decode results -----------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Send Req to Node
   */
  todo_s4           := decode_s4_g
  val reqDoneList   = Seq(done_s4_g.snoop, done_s4_g.readDown, done_s4_g.writeDown, done_s4_g.readDCU, done_s4_g.writeDCU, done_s4_g_sfEvict, done_s4_g.flush)
  val reqTodoList   = Seq(todo_s4.snoop     & !done_s4_g.snoop,
                          todo_s4.readDown  & !done_s4_g.readDown,
                          todo_s4.writeDown & !done_s4_g.writeDown,
                          todo_s4.readDCU   & !done_s4_g.readDCU,
                          todo_s4.writeDCU  & !done_s4_g.writeDCU,
                          todo_s4_g_sfEvict & !done_s4_g_sfEvict,
                          todo_s4.flush     & !done_s4_g.flush)
  val toBeSendId    = PriorityEncoder(reqTodoList)
  reqBeSend_s4(0)   := taskSnp_s4
  reqBeSend_s4(1)   := taskRD_s4
  reqBeSend_s4(2)   := taskWD_s4
  reqBeSend_s4(3)   := readDCU_s4
  reqBeSend_s4(4)   := Mux(todo_s4_g_replace, taskRepl_s4, writeDCU_s4) // writeDCU transfer to taskRepl_s4
  reqBeSend_s4(5)   := taskSnpEvict_s4
  reqBeSend_s4(6)   := flush_s4
  io.req2Intf.valid := valid_s4_g & reqTodoList.reduce(_ | _)
  io.req2Intf.bits  := reqBeSend_s4(toBeSendId)
  reqDoneList.zipWithIndex.foreach { case(d, i) => d := Mux(s3Fire, false.B, d | (io.req2Intf.fire & toBeSendId === i.U)) }
  // req
  val reqDone       = PopCount(reqTodoList) === 0.U | (PopCount(reqTodoList) === 1.U & io.req2Intf.fire)


  /*
   * Send Write Req to Directory
   */
  // self
  io.dirWrite.s.valid   := valid_s4_g & todo_s4.wSDir & !done_s4_g.wSDir
  io.dirWrite.s.bits    := wSDir_s4
  done_s4_g.wSDir       := Mux(s3Fire, false.B, done_s4_g.wSDir | io.dirWrite.s.fire)
  // sf
  io.dirWrite.sf.valid  := valid_s4_g & todo_s4.wSFDir & !done_s4_g.wSFDir
  io.dirWrite.sf.bits   := wSFDir_s4
  done_s4_g.wSFDir      := Mux(s3Fire, false.B, done_s4_g.wSFDir | io.dirWrite.sf.fire)
  // dir
  val dirTodoList       = Seq(todo_s4.wSDir  & !done_s4_g.wSDir  & !io.dirWrite.s.fire,
                              todo_s4.wSFDir & !done_s4_g.wSFDir & !io.dirWrite.sf.fire)
  val dirDone           = PopCount(dirTodoList) === 0.U


  /*
   * Send Read or Clean Req to DataBuffer
   */
  io.dbRCReq.valid      := valid_s4_g & ((todo_s4.rDB2Src & !done_s4_g.rDB2Src) | (todo_s4.cleanDB & !done_s4_g.cleanDB))
  io.dbRCReq.bits       := rcDBReq_s4
  done_s4_g.rDB2Src     := Mux(s3Fire, false.B, done_s4_g.rDB2Src | (io.dbRCReq.fire & io.dbRCReq.bits.isRead))
  done_s4_g.cleanDB     := Mux(s3Fire, false.B, done_s4_g.cleanDB | (io.dbRCReq.fire & io.dbRCReq.bits.isClean))
  val rcDBTodoList      = Seq(todo_s4.rDB2Src & !done_s4_g.rDB2Src & !io.dbRCReq.fire,
                              todo_s4.cleanDB & !done_s4_g.cleanDB & !io.dbRCReq.fire)
  val rcDBDone          = PopCount(rcDBTodoList) === 0.U

  /*
   * Send Commit to S4
   */
  io.resp2Intf.valid    := valid_s4_g & todo_s4.commit & !done_s4_g.commit
  io.resp2Intf.bits     := commit_s4
  done_s4_g.commit      := Mux(s3Fire, false.B, done_s4_g.commit | io.resp2Intf.fire)
  val comDone           = !(todo_s4.commit & !done_s4_g.commit & !io.resp2Intf.fire)

  /*
   * Set Can Go S3 Value
   */
  canGo_s4              := valid_s4_g & reqDone & dirDone & rcDBDone & comDone




// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------- S3_Execute: UnLock MshrLockVec ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Update MshrLockVec:
   * 1. S3 done but task dont need to commit
   */
  io.updLockMSHR.valid  := valid_s4_g & canGo_s4 & needUnLockMSHR_s4_g
  io.updLockMSHR.bits   := task_s4_g.taskMes.minDirSet


// ----------------------------------------------------- Assertion ------------------------------------------------------ //
  // S4
  val cnt_s4_g  = RegInit(0.U(64.W))
  cnt_s4_g      := Mux(!valid_s4_g | canGo_s4, 0.U, cnt_s4_g + 1.U)
  assert(cnt_s4_g < TIMEOUT_PIPEEXU.U, "ProcessPipe[0x%x] EXECUTE ADDR[0x%x] OP[0x%x] TIMEOUT", task_s4_g.taskMes.pipeID, task_s4_g.taskMes.fullAddr(io.dcuID, io.pcuID), task_s4_g.chiMes.opcode)

  // Other
  assert(!valid_s4_g | !todo_s4.asUInt.asBools.zip(done_s4_g.asUInt.asBools).map { case(todo, done) => !todo & done }.reduce(_ | _))



// -------------------------------------------------- Perf Counter ------------------------------------------------------ //
  // read
  XSPerfAccumulate("pcu_pipe_req_read_cnt",       valid_s3 & canGo_s3 & isReadX(task_s3.bits.chiMes.opcode) & task_s3.bits.chiMes.isReq & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_read_hit_cnt",   valid_s3 & canGo_s3 & isReadX(task_s3.bits.chiMes.opcode) & task_s3.bits.chiMes.isReq & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_read_miss_cnt",  valid_s3 & canGo_s3 & isReadX(task_s3.bits.chiMes.opcode) & task_s3.bits.chiMes.isReq & !todo_s3_retry & !hnHit_s3)
  // write
  XSPerfAccumulate("pcu_pipe_req_write_cnt",      valid_s3 & canGo_s3 & isWriteX(task_s3.bits.chiMes.opcode) & task_s3.bits.chiMes.isReq & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_write_hit_cnt",  valid_s3 & canGo_s3 & isWriteX(task_s3.bits.chiMes.opcode) & task_s3.bits.chiMes.isReq & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_write_miss_cnt", valid_s3 & canGo_s3 & isWriteX(task_s3.bits.chiMes.opcode) & task_s3.bits.chiMes.isReq & !todo_s3_retry & !hnHit_s3)
  // evict
  XSPerfAccumulate("pcu_pipe_req_evict_cnt",      valid_s3 & canGo_s3 & task_s3.bits.chiMes.opcode === Evict & task_s3.bits.chiMes.isReq & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_evict_hit_cnt",  valid_s3 & canGo_s3 & task_s3.bits.chiMes.opcode === Evict & task_s3.bits.chiMes.isReq & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_evict_miss_cnt", valid_s3 & canGo_s3 & task_s3.bits.chiMes.opcode === Evict & task_s3.bits.chiMes.isReq & !todo_s3_retry & !hnHit_s3)
  // makeUnique
  XSPerfAccumulate("pcu_pipe_req_makeUnique_cnt",       valid_s3 & canGo_s3 & task_s3.bits.chiMes.opcode === MakeUnique & task_s3.bits.chiMes.isReq & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_makeUnique_hit_cnt",   valid_s3 & canGo_s3 & task_s3.bits.chiMes.opcode === MakeUnique & task_s3.bits.chiMes.isReq & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_makeUnique_miss_cnt",  valid_s3 & canGo_s3 & task_s3.bits.chiMes.opcode === MakeUnique & task_s3.bits.chiMes.isReq & !todo_s3_retry & !hnHit_s3)
  // total
  XSPerfAccumulate("pcu_pipe_req_total_cnt",      valid_s3 & canGo_s3 & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_hit_total_cnt",  valid_s3 & canGo_s3 & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_miss_total_cnt", valid_s3 & canGo_s3 & !todo_s3_retry & !hnHit_s3)
  XSPerfAccumulate("pcu_pipe_retry_total_cnt",    valid_s3 & canGo_s3 & todo_s3_retry)


}