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
import dongjiang.utils.StepRREncoderOH

import xs.utils.debug.{DomainInfo, HardwareAssertion, awhen}


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
    // Req To Node
    val req2Intf    = Decoupled(new Req2IntfBundle())
    // Resp To Node
    val resp2Intf   = Decoupled(new Resp2IntfBundle())
  })

// --------------------- Modules declaration ------------------------//
  val taskQ   = Module(new Queue(new PipeTaskBundle(), entries = djparam.nrPipeTaskQueue, pipe = true, flow = true))
  val dirResQ = Module(new Queue(new DirRespBundle(), entries = djparam.nrPipeTaskQueue + 1, pipe = true, flow = true)) // Add one for mp_s1 read Dir before send task to mp_2

  dontTouch(taskQ.io.count)
  dontTouch(dirResQ.io.count)

// --------------------- Reg/Wire declaration ------------------------//
  // s2 signals
  val canGo_s2_req        = Wire(Bool())
  val canGo_s2_dir        = Wire(Bool())

  // s3 basic signals
  val canGo_s3            = Wire(Bool()); dontTouch(canGo_s3)
  val valid_s3            = Wire(Bool()); dontTouch(valid_s3)
  val task_s3_needDir     = Wire(Bool())
  val task_s3_g           = RegInit(0.U.asTypeOf(Valid(new PipeTaskBundle()))); dontTouch(task_s3_g)
  val dirRes_s3_g         = RegInit(0.U.asTypeOf(Valid(new DirRespBundle()))); dontTouch(dirRes_s3_g)
  val srcMetaID_s3        = Wire(UInt((metaIdBits+1).W)) // An extra bit is used to distinguish the RNI
  // s3 decode base signals
  val inst_s3             = Wire(new InstBundle()); dontTouch(inst_s3)
  val inst_req_s3         = WireInit(0.U.asTypeOf(new InstBundle())); dontTouch(inst_s3)
  val decode_s3           = Wire(new DecodeBundle()); dontTouch(decode_s3)
  val decode_req_s3       = Wire(new DecodeBundle()); dontTouch(decode_req_s3)
  val snpNodeVec_s3       = WireInit(VecInit(Seq.fill(nrCcNode) { false.B }))
  // s3 execute(update MSHR) signals: task to do list
  val todo_s3             = WireInit(0.U.asTypeOf(new OperationsBundle())); dontTouch(todo_s3)
  val todo_s3_retry       = Wire(Bool()); dontTouch(todo_s3_retry)
  val todo_s3_replace     = Wire(Bool()); dontTouch(todo_s3_replace) // replace self Directory
  val todo_s3_sfEvict     = Wire(Bool()); dontTouch(todo_s3_sfEvict) // replace snoop filter
  val todo_s3_updMSHR     = Wire(Bool()); dontTouch(todo_s3_updMSHR)
  val todo_s3_cleanMSHR   = Wire(Bool()); dontTouch(todo_s3_cleanMSHR)


  // s4 temp signals get from s3
  val valid_s4_temp_g     = RegInit(false.B)
  val s4_temp_g           = RegInit(0.U.asTypeOf(new Bundle {
    val task              = new PipeTaskBundle()
    val decode            = new DecodeBundle()
    val snpNodeVec        = Vec(nrCcNode, Bool())
    val dirRes            = new DirRespBundle()
    val respType          = UInt(RespType.width.W)
    val srcMetaID         = UInt((metaIdBits+1).W)
    val todo_replace      = Bool()
    val todo_sfEvict      = Bool()
    // mshr
    val todo_retry        = Bool()
    val todo_updMSHR      = Bool()
    val todo_cleanMSHR    = Bool()
  })); dontTouch(s4_temp_g)
  // s4 signals get from s3
  val s3                  = WireInit(0.U.asTypeOf(s4_temp_g)); dontTouch(s3)
  val rnHitVec_s3         = Wire(Vec(dirRes_s3_g.bits.sf.metaVec.size, Bool()))
  val valid_s4_g          = RegInit(false.B)
  val canGo_s4            = Wire(Bool())
  val s4_g                = RegInit(0.U.asTypeOf(s4_temp_g)); dontTouch(s4_g)
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
  val done_s4_g_updMSHR   = RegInit(false.B)
  val done_s4_g_sfEvict   = RegInit(false.B)
  val reqBeSend_s4        = Wire(Vec(7, new Req2IntfBundle()))
  // s4 execute signals: Execute Done
  val comUpdMSHR          = Wire(Bool()); dontTouch(comUpdMSHR)
  val reqDone             = Wire(Bool()); dontTouch(reqDone)
  val dirDone             = Wire(Bool()); dontTouch(dirDone)
  val rcDBDone            = Wire(Bool()); dontTouch(rcDBDone)
  val comDone             = Wire(Bool()); dontTouch(comDone)


  /*
   * for Debug
   */
  // s3
  val task_s3_dbg_addr      = Wire(UInt(fullAddrBits.W))
  task_s3_dbg_addr          := task_s3_g.bits.fullAddr(io.dcuID, io.pcuID)
  if (p(DebugOptionsKey).EnableDebug) dontTouch(task_s3_dbg_addr)
  // s4_temp
  val task_s4_temp_dbg_addr = Wire(UInt(fullAddrBits.W))
  task_s4_temp_dbg_addr     := s4_temp_g.task.fullAddr(io.dcuID, io.pcuID)
  if (p(DebugOptionsKey).EnableDebug) dontTouch(task_s4_temp_dbg_addr)
  // s4
  val task_s4_dbg_addr      = Wire(UInt(fullAddrBits.W))
  task_s4_dbg_addr          := s4_g.task.fullAddr(io.dcuID, io.pcuID)
  if (p(DebugOptionsKey).EnableDebug) dontTouch(task_s4_dbg_addr)


val hwaFlags = Array.fill(2)(Wire(Bool()))
for (i <- 0 until 2) {
  hwaFlags(i) := true.B
}

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
  HardwareAssertion(Mux(io.dirResp.valid, dirResQ.io.enq.ready, true.B))

  // Can Go Signals
  // TODO: consider readDir
  canGo_s2_req          := canGo_s3 | !task_s3_g.valid
  canGo_s2_dir          := canGo_s3 | !dirRes_s3_g.valid


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------- S3_Receive: Receive task and dirRes from s2 -------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  // TODO: consider readDir
  /*
   * Recieve task_s2
   */
  task_s3_g.valid       := Mux(taskQ.io.deq.valid, true.B, task_s3_g.valid & !canGo_s3)
  task_s3_g.bits        := Mux(taskQ.io.deq.fire, taskQ.io.deq.bits, task_s3_g.bits)
  task_s3_needDir       := task_s3_g.bits.taskMes.readDir

  /*
   * Recieve dirRes
   */
  dirRes_s3_g.valid     := Mux(dirResQ.io.deq.valid, true.B, dirRes_s3_g.valid & !(canGo_s3 & task_s3_needDir))
  dirRes_s3_g.bits      := Mux(dirResQ.io.deq.fire, dirResQ.io.deq.bits, dirRes_s3_g.bits)

  /*
   * S3 base ctrl logic
   */
  valid_s3              := task_s3_g.valid & (dirRes_s3_g.valid | !task_s3_needDir)
  canGo_s3              := valid_s3 & !valid_s4_temp_g


// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- S3_Decode: Decode by task Message and Dir Result ------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Parse Dir Result
   */
  val srcCCMetaID_s3  = getMetaIDByNodeID(task_s3_g.bits.chiIndex.nodeID)
  srcMetaID_s3        := Mux(fromRniNode(task_s3_g.bits.chiIndex.nodeID), Fill(srcMetaID_s3.getWidth, 1.U(1.W)), srcCCMetaID_s3)
  HardwareAssertion(fromCcNode(task_s3_g.bits.chiIndex.nodeID) | fromRniNode(task_s3_g.bits.chiIndex.nodeID) | !task_s3_g.valid | (task_s3_g.bits.chiMes.opcode === SnpUniqueEvict & task_s3_g.bits.chiMes.isSnp))

  val srcHit_s3       = dirRes_s3_g.bits.sf.hit & !dirRes_s3_g.bits.sf.metaVec(srcCCMetaID_s3).isInvalid & !fromRniNode(task_s3_g.bits.chiIndex.nodeID)
  val srcState_s3     = Mux(srcHit_s3, dirRes_s3_g.bits.sf.metaVec(srcCCMetaID_s3).state, ChiState.I)

  val othHit_s3       = dirRes_s3_g.bits.sf.hit & (PopCount(dirRes_s3_g.bits.sf.metaVec.map(!_.isInvalid)) > srcHit_s3.asUInt)
  val sfHitID_s3      = PriorityEncoder(dirRes_s3_g.bits.sf.metaVec.map(!_.isInvalid))
  val othState_s3     = Mux(othHit_s3, dirRes_s3_g.bits.sf.metaVec(sfHitID_s3).state, ChiState.I)

  val hnHit_s3        = dirRes_s3_g.bits.s.hit
  val hnState_s3      = Mux(hnHit_s3, dirRes_s3_g.bits.s.metaVec(0).state, ChiState.I)

  /*
   * Set Inst value
   */
  val hasData_s3      = task_s3_g.bits.respMes.slvDBID.valid | task_s3_g.bits.respMes.mstDBID.valid
  val taskIsWriPtl_s3 = isWriXPtl(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq
  val taskIsWriUni_s3 = isWriUniX(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq
  val taskIsCMO_s3    = isCMO(task_s3_g.bits.chiMes.opcode)     & task_s3_g.bits.chiMes.isReq
  val taskIsCB_s3     = isCBX(task_s3_g.bits.chiMes.opcode)     & task_s3_g.bits.chiMes.isReq
  val taskIsAtomic_s3 = isAtomicX(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq

  HardwareAssertion(Mux(taskIsWriPtl_s3 & task_s3_g.valid, !task_s3_g.bits.respMes.slvResp.valid, true.B))
  HardwareAssertion(Mux(taskIsWriPtl_s3 & task_s3_g.valid, task_s3_g.bits.respMes.slvDBID.valid, true.B))
  HardwareAssertion(Mux(taskIsAtomic_s3 & task_s3_g.valid, hasData_s3, true.B))
  HardwareAssertion(Mux(taskIsAtomic_s3 & task_s3_g.valid, !isAtomicStoreX(task_s3_g.bits.chiMes.opcode), true.B))


  inst_s3.channel       := task_s3_g.bits.chiMes.channel
  inst_s3.opcode        := Mux(taskIsAtomic_s3, AtomicLoadADD, task_s3_g.bits.chiMes.opcode) // When the task is an atomic operation, it is converted to AtomicLoadADD for decoding.
  inst_s3.srcState      := Mux(task_s3_needDir, srcState_s3, ChiState.I)
  inst_s3.othState      := Mux(task_s3_needDir, othState_s3, ChiState.I)
  inst_s3.hnState       := Mux(task_s3_needDir, hnState_s3,  ChiState.I)
  inst_s3.respType      := Cat(taskIsCB_s3 & task_s3_g.bits.respMes.slvResp.valid,  // Copy Back Resp
                               task_s3_g.bits.respMes.mstResp.valid,                // Read Down Resp
                               task_s3_g.bits.respMes.fwdState.valid,               // Snoop Fwd Resp
                               task_s3_g.bits.respMes.slvResp.valid & !taskIsCB_s3) // Snoop Resp
  inst_s3.slvResp       := task_s3_g.bits.respMes.slvResp.bits
  inst_s3.fwdState      := task_s3_g.bits.respMes.fwdState.bits
  inst_s3.mstResp       := task_s3_g.bits.respMes.mstResp.bits
  inst_s3.reqWithData   := hasData_s3 & task_s3_g.bits.taskMes.pipeID === PipeID.REQ
  inst_s3.respWithData  := hasData_s3 & task_s3_g.bits.taskMes.pipeID === PipeID.RESP


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
    HardwareAssertion(!(decode.cleanDB & decode.writeDCU), s"Index: $i")
  }
  HardwareAssertion.placePipe(1)
  // deocde
  decode_s3.decode(inst_s3, table)
  decode_req_s3.decode(inst_req_s3, table)
  // assert
  awhen(valid_s3) { 
   HardwareAssertion(
    decode_s3.asUInt =/= Code.ERROE,
    cf"""
    ADDR[0x${task_s3_g.bits.fullAddr(io.dcuID, io.pcuID)}] DECODE ERROR: No inst match in decode table
    INST: CHNL[0x${inst_s3.channel}] OP[0x${inst_s3.opcode}] SRC[0x${inst_s3.srcState}] OTH[0x${inst_s3.othState}] HN[0x${inst_s3.hnState}] REQDATA[0x${inst_s3.reqWithData}] RESP[0x${inst_s3.respType}] RESPDATA[0x${inst_s3.respWithData}] SLV[0x${inst_s3.slvResp}] FWD[0x${inst_req_s3.fwdState}] MST[0x${inst_req_s3.mstResp}]
    """) 
  }

  awhen(valid_s3&decode_s3.wSDir) { 
      HardwareAssertion(
        decode_s3.commit | 
        (inst_s3.opcode === SnpUniqueEvict & inst_s3.channel === CHIChannel.SNP) | 
        (isWriteX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ) | 
        (isReadX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ & djparam.openDMT.asBool))
  }

  awhen(valid_s3&decode_s3.wSFDir) { 
      HardwareAssertion(
        decode_s3.commit | 
        (isWriteX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ) | 
        (isReadX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ & djparam.openDMT.asBool)) 
  } 

  awhen(valid_s3) { 
    HardwareAssertion(
      decode_req_s3.asUInt =/= Code.ERROE,
      cf"""
      ADDR[0x${task_s3_g.bits.fullAddr(io.dcuID, io.pcuID)}] DECODE ERROR: No inst match in decode table
      INST: CHNL[0x${inst_req_s3.channel}] OP[0x${inst_req_s3.opcode}] SRC[0x${inst_req_s3.srcState}] OTH[0x${inst_req_s3.othState}] HN[0x${inst_req_s3.hnState}] REQDATA[0x${inst_s3.reqWithData}] RESP[0x${inst_req_s3.respType}] RESPDATA[0x${inst_req_s3.respWithData}] SLV[0x${inst_req_s3.slvResp}] FWD[0x${inst_req_s3.fwdState}] MST[0x${inst_req_s3.mstResp}]
      """
    ) 
  }

// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------ S3_Req_Decode: Get Snp Target ----------------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Decode Req
   */
  inst_req_s3.channel     := inst_s3.channel
  inst_req_s3.opcode      := inst_s3.opcode
  inst_req_s3.srcState    := inst_s3.srcState
  inst_req_s3.othState    := inst_s3.othState
  inst_req_s3.hnState     := inst_s3.hnState
  inst_req_s3.reqWithData := taskIsAtomic_s3 | taskIsWriUni_s3 // TODO: Dont use the req type directly for judgment
  inst_req_s3.respType    := RespType.NotResp

  /*
   * Set Snoop Target Value
   */
  rnHitVec_s3         := dirRes_s3_g.bits.sf.metaVec.map(!_.isInvalid); dontTouch(rnHitVec_s3)
  val rnHitWithoutSrc = rnHitVec_s3.zipWithIndex.map { case(hit, i) => hit & i.U =/= srcMetaID_s3 }
  when(decode_req_s3.snpTgt === SnpTgt.ALL)       { snpNodeVec_s3 := rnHitVec_s3 }
  .elsewhen(decode_req_s3.snpTgt === SnpTgt.OTH)  { snpNodeVec_s3 := rnHitWithoutSrc }
  .elsewhen(decode_req_s3.snpTgt === SnpTgt.ONE)  { snpNodeVec_s3 := StepRREncoderOH(rnHitWithoutSrc, canGo_s3 & decode_req_s3.snpTgt === SnpTgt.ONE).asBools }
  .otherwise                                      { snpNodeVec_s3 := 0.U.asTypeOf(snpNodeVec_s3) }
  // assert
  awhen(valid_s3 & decode_s3.snoop){
    HardwareAssertion(dirRes_s3_g.bits.sf.hit)
    HardwareAssertion(decode_s3.snpTgt =/= SnpTgt.NONE)
  }



// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------------------- S3_Execute: Get some todos ----------------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  // Send Retry to MSHR When need write Dir but cant do it
  todo_s3             := decode_s3
  todo_s3_retry       := todo_s3.wSDir & dirRes_s3_g.bits.s.replRetry | todo_s3.wSFDir & dirRes_s3_g.bits.sf.replRetry; HardwareAssertion(Mux(valid_s3, !todo_s3_retry, true.B), "TODO")
  todo_s3_replace     := todo_s3.wSDir & !hnHit_s3 & dirRes_s3_g.bits.s.metaVec(0).isDirty & !todo_s3_retry // Only need to replace when it is Dirty
  todo_s3_sfEvict     := todo_s3.wSFDir & !srcHit_s3 & !othHit_s3 & dirRes_s3_g.bits.sf.metaVec.map(!_.isInvalid).reduce(_ | _) & !todo_s3_retry
  todo_s3_updMSHR     := decode_s3.needWaitSlv | decode_s3.needWaitMst | todo_s3_replace | todo_s3_sfEvict
  todo_s3_cleanMSHR   := !(todo_s3_retry | todo_s3_updMSHR)
  HardwareAssertion(Mux(valid_s3, PopCount(Seq(todo_s3_retry, todo_s3_updMSHR, todo_s3_cleanMSHR)) === 1.U, true.B))
  HardwareAssertion(Mux(valid_s3, PopCount(Seq(todo_s3_replace, todo_s3_sfEvict)) <= 1.U, true.B))
  HardwareAssertion(Mux(valid_s3 & todo_s3_replace, todo_s3.writeDCU, true.B))


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------- S4_Receive: Receive task and dirRes from s3 -------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Recieve s3 signals
   */
  s3.task           := task_s3_g.bits
  s3.decode         := decode_s3
  s3.snpNodeVec     := snpNodeVec_s3
  s3.dirRes         := dirRes_s3_g.bits
  s3.todo_replace   := todo_s3_replace
  s3.todo_sfEvict   := todo_s3_sfEvict
  s3.respType       := inst_s3.respType
  s3.srcMetaID      := srcMetaID_s3
  s3.todo_retry     := todo_s3_retry
  s3.todo_updMSHR   := todo_s3_updMSHR
  s3.todo_cleanMSHR := todo_s3_cleanMSHR

  /*
  * S4 base ctrl logic
  */
  val s3Fire        = valid_s3 & canGo_s3
  val s4Fire        = valid_s4_g & canGo_s4

  switch(Cat(valid_s4_temp_g, valid_s4_g)) {
    is("b00".U) {
      // valid                                | bits
      valid_s4_temp_g := false.B;             s4_temp_g := DontCare
      valid_s4_g      := s3Fire;              s4_g      := s3
    }
    is("b01".U) {
      // valid                                | bits
      valid_s4_temp_g := s3Fire & !canGo_s4;  s4_temp_g := s3
      valid_s4_g      := s3Fire | !canGo_s4;  s4_g      := Mux(s4Fire, s3, s4_g) // bypass s4_temp
    }
    is("b11".U) {
      // valid                                | bits
      valid_s4_temp_g := s3Fire | !s4Fire;    s4_temp_g := Mux(s3Fire, s3, s4_temp_g)
      valid_s4_g      := true.B;              s4_g      := Mux(s4Fire, s4_temp_g, s4_g)
    }
  }
  HardwareAssertion(!(Cat(valid_s4_temp_g, valid_s4_g)==="b10".U),"Illegitimate")

  
// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------- S4_Execute: Set specific tasks value -----------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  val taskIsWriPtl_s4 = isWriXPtl(s4_g.task.chiMes.opcode)  & s4_g.task.chiMes.isReq
  val taskIsCMO_s4    = isCMO(s4_g.task.chiMes.opcode)      & s4_g.task.chiMes.isReq
  val taskIsCB_s4     = isCBX(s4_g.task.chiMes.opcode)      & s4_g.task.chiMes.isReq
  val taskIsAtomic_s4 = isAtomicX(s4_g.task.chiMes.opcode)  & s4_g.task.chiMes.isReq


  /*
   * Send Snoop to RN-F
   */
  // get dbid
  dbid_s4.valid       := valid_s4_g & (s4_g.task.respMes.slvDBID.valid | s4_g.task.respMes.mstDBID.valid)
  dbid_s4.bits        := Mux(s4_g.task.respMes.slvDBID.valid, s4_g.task.respMes.slvDBID.bits, s4_g.task.respMes.mstDBID.bits)
  HardwareAssertion(Mux(valid_s4_g, !(s4_g.task.respMes.slvDBID.valid & s4_g.task.respMes.mstDBID.valid), true.B))


  // taskSnp_s4
  taskSnp_s4.chiIndex.txnID       := s4_g.task.chiIndex.txnID
  taskSnp_s4.chiIndex.nodeID      := s4_g.task.chiIndex.nodeID
  taskSnp_s4.chiIndex.size        := chiFullSize.U
  taskSnp_s4.chiIndex.offset      := 0.U
  taskSnp_s4.chiMes.channel       := CHIChannel.SNP
  taskSnp_s4.chiMes.doNotGoToSD   := true.B
  taskSnp_s4.chiMes.retToSrc      := s4_g.decode.retToSrc
  taskSnp_s4.chiMes.fwdState      := s4_g.decode.fwdState
  taskSnp_s4.chiMes.expCompAck    := false.B
  taskSnp_s4.chiMes.opcode        := s4_g.decode.snpOp
  taskSnp_s4.chiMes.resp          := DontCare
  taskSnp_s4.from                 := io.dcuID
  taskSnp_s4.to                   := IncoID.LOCALSLV.U
  taskSnp_s4.pcuIndex.mshrSet     := DontCare
  taskSnp_s4.pcuIndex.mshrWay     := s4_g.task.taskMes.mshrWay
  taskSnp_s4.pcuIndex.dbID        := dbid_s4.bits
  taskSnp_s4.pcuIndex.entryID     := DontCare
  taskSnp_s4.pcuMes.useAddr       := s4_g.task.taskMes.useAddr
  taskSnp_s4.pcuMes.doDMT         := DontCare
  taskSnp_s4.pcuMes.selfWay       := DontCare
  taskSnp_s4.pcuMes.toDCU         := DontCare
  taskSnp_s4.pcuMes.snpTgtVec     := s4_g.snpNodeVec
  taskSnp_s4.pcuMes.hasPcuDBID    := dbid_s4.valid
  HardwareAssertion(Mux(s4_g.decode.snoop & dbid_s4.valid & valid_s4_g, taskIsWriPtl_s4 | taskIsAtomic_s4, true.B))
  taskSnp_s4.pcuMes.snpNeedDB     := s4_g.decode.snpNeedDB


  /*
   * Send Read to SN(DDRC) / HN-F(CSN)
   */
  taskRD_s4                       := DontCare
  taskRD_s4.chiIndex.txnID        := s4_g.task.chiIndex.txnID
  taskRD_s4.chiIndex.nodeID       := s4_g.task.chiIndex.nodeID
  taskRD_s4.chiIndex.size         := Mux(taskIsAtomic_s4, chiFullSize.U,     s4_g.task.chiIndex.size)
  taskRD_s4.chiIndex.offset       := Mux(taskIsAtomic_s4, 0.U(offsetBits.W), s4_g.task.chiIndex.offset)
  taskRD_s4.chiMes.channel        := CHIChannel.REQ
  taskRD_s4.chiMes.expCompAck     := false.B
  taskRD_s4.chiMes.opcode         := s4_g.decode.rdOp
  taskRD_s4.chiMes.resp           := ChiResp.UC
  taskRD_s4.from                  := io.dcuID
  taskRD_s4.to                    := IncoID.LOCALMST.U
  taskRD_s4.pcuIndex.mshrWay      := s4_g.task.taskMes.mshrWay
  taskRD_s4.pcuIndex.dbID         := dbid_s4.bits
  taskRD_s4.pcuMes.useAddr        := s4_g.task.taskMes.useAddr
  taskRD_s4.pcuMes.doDMT          := djparam.openDMT.asBool & !(taskIsAtomic_s4)
  taskRD_s4.pcuMes.toDCU          := false.B
  taskRD_s4.pcuMes.hasPcuDBID     := dbid_s4.valid
  HardwareAssertion(Mux((s4_g.decode.readDCU | s4_g.decode.readDown) & dbid_s4.valid & valid_s4_g, taskIsAtomic_s4, true.B))


  /*
   * Send Write / Dataless to SN(DDRC) / HN-F(CSN)
   */
  taskWD_s4                       := DontCare
  taskWD_s4.chiIndex              := s4_g.task.chiIndex
  taskWD_s4.chiIndex.size         := s4_g.task.chiIndex.size
  taskWD_s4.chiIndex.offset       := s4_g.task.chiIndex.offset
  taskWD_s4.chiMes.channel        := CHIChannel.REQ
  taskWD_s4.chiMes.expCompAck     := false.B
  taskWD_s4.chiMes.opcode         := s4_g.decode.wdOp
  taskWD_s4.from                  := io.dcuID
  taskWD_s4.to                    := IncoID.LOCALMST.U
  taskWD_s4.pcuIndex.mshrWay      := s4_g.task.taskMes.mshrWay
  taskWD_s4.pcuIndex.dbID         := dbid_s4.bits
  HardwareAssertion(Mux(valid_s4_g & s4_g.decode.writeDown, dbid_s4.valid, true.B))
  taskWD_s4.pcuMes.useAddr        := s4_g.task.taskMes.useAddr
  taskWD_s4.pcuMes.selfWay        := DontCare
  taskWD_s4.pcuMes.toDCU          := false.B


  /*
   * Send Read / Clean to DataBuffer
   */
  rcDBReq_s4.to         := IncoID.LOCALSLV.U
  rcDBReq_s4.isRead     := s4_g.decode.rDB2Src
  rcDBReq_s4.isClean    := s4_g.decode.cleanDB
  rcDBReq_s4.dbID       := dbid_s4.bits
  HardwareAssertion(Mux(valid_s4_g & s4_g.decode.rDB2Src, dbid_s4.valid, true.B))
  rcDBReq_s4.rBeatOH    := s4_g.task.chiIndex.beatOH
  rcDBReq_s4.exAtomic   := taskIsAtomic_s4


  /*
   * Send Read to SN(DCU)
   */
  readDCU_s4                        := DontCare
  readDCU_s4.chiIndex.txnID         := s4_g.task.chiIndex.txnID
  readDCU_s4.chiIndex.nodeID        := s4_g.task.chiIndex.nodeID
  readDCU_s4.chiIndex.size          := Mux(taskIsAtomic_s4, chiFullSize.U,     s4_g.task.chiIndex.size)
  readDCU_s4.chiIndex.offset        := Mux(taskIsAtomic_s4, 0.U(offsetBits.W), s4_g.task.chiIndex.offset)
  readDCU_s4.chiMes.channel         := CHIChannel.REQ
  readDCU_s4.chiMes.expCompAck      := false.B
  readDCU_s4.chiMes.opcode          := s4_g.decode.rdOp
  readDCU_s4.chiMes.resp            := s4_g.decode.resp
  readDCU_s4.from                   := io.dcuID
  readDCU_s4.to                     := IncoID.LOCALMST.U
  readDCU_s4.pcuIndex.mshrWay       := s4_g.task.taskMes.mshrWay
  readDCU_s4.pcuIndex.dbID          := dbid_s4.bits
  readDCU_s4.pcuMes.useAddr         := s4_g.task.taskMes.useAddr
  readDCU_s4.pcuMes.doDMT           := djparam.openDMT.asBool & !(taskIsAtomic_s4)
  readDCU_s4.pcuMes.selfWay         := OHToUInt(s4_g.dirRes.s.wayOH)
  readDCU_s4.pcuMes.toDCU           := true.B
  readDCU_s4.pcuMes.hasPcuDBID      := dbid_s4.valid
  HardwareAssertion(Mux((s4_g.decode.readDCU | s4_g.decode.readDown) & dbid_s4.valid & valid_s4_g, taskIsAtomic_s4, true.B))


  /*
   * Send Write to SN(DCU)
   */
  val snpRespHasData                = RespType.isSnpX(s4_g.respType) & s4_g.task.respMes.slvDBID.valid & !taskIsCB_s4
  writeDCU_s4                       := DontCare
  writeDCU_s4.chiIndex              := s4_g.task.chiIndex
  writeDCU_s4.chiIndex.size         := Mux(taskIsAtomic_s4 | snpRespHasData, chiFullSize.U,     s4_g.task.chiIndex.size)
  writeDCU_s4.chiIndex.offset       := Mux(taskIsAtomic_s4 | snpRespHasData, 0.U(offsetBits.W), s4_g.task.chiIndex.offset)
  writeDCU_s4.chiMes.channel        := CHIChannel.REQ
  writeDCU_s4.chiMes.expCompAck     := false.B
  writeDCU_s4.chiMes.opcode         := s4_g.decode.wdOp
  writeDCU_s4.from                  := io.dcuID
  writeDCU_s4.to                    := IncoID.LOCALMST.U
  writeDCU_s4.pcuIndex.mshrWay      := s4_g.task.taskMes.mshrWay
  writeDCU_s4.pcuIndex.dbID         := dbid_s4.bits
  HardwareAssertion(Mux(valid_s4_g & s4_g.decode.writeDCU, dbid_s4.valid, true.B))
  writeDCU_s4.pcuMes.useAddr        := s4_g.task.taskMes.useAddr
  writeDCU_s4.pcuMes.selfWay        := OHToUInt(s4_g.dirRes.s.wayOH)
  writeDCU_s4.pcuMes.toDCU          := true.B


  /*
   * Send Repl to SN(DCU)
   */
  taskRepl_s4                       := DontCare
  taskRepl_s4.chiIndex              := s4_g.task.chiIndex
  taskRepl_s4.chiIndex.size         := chiFullSize.U
  HardwareAssertion(Mux(valid_s4_g & s4_g.todo_replace, s4_g.task.chiIndex.fullSize | snpRespHasData, true.B))
  taskRepl_s4.chiIndex.offset       := 0.U
  taskRepl_s4.chiMes.channel        := CHIChannel.REQ
  taskRepl_s4.chiMes.expCompAck     := false.B
  taskRepl_s4.chiMes.opcode         := Replace
  taskRepl_s4.from                  := io.dcuID
  taskRepl_s4.to                    := IncoID.LOCALMST.U
  taskRepl_s4.pcuIndex.mshrWay      := s4_g.task.taskMes.mshrWay
  taskRepl_s4.pcuIndex.dbID         := dbid_s4.bits
  HardwareAssertion(Mux(valid_s4_g & s4_g.todo_replace, dbid_s4.valid, true.B))
  taskRepl_s4.pcuMes.useAddr        := s4_g.dirRes.s.useAddr
  taskRepl_s4.pcuMes.selfWay        := OHToUInt(s4_g.dirRes.s.wayOH)
  taskRepl_s4.pcuMes.toDCU          := false.B

 /*
  * Send Flush to DCU
  */
  flush_s4                       := DontCare
  flush_s4.chiIndex.size         := chiFullSize.U
  flush_s4.chiIndex.offset       := 0.U
  flush_s4.chiMes.channel        := CHIChannel.REQ
  flush_s4.chiMes.expCompAck     := false.B
  flush_s4.chiMes.opcode         := FlushDCU
  flush_s4.from                  := io.dcuID
  flush_s4.to                    := IncoID.LOCALMST.U
  flush_s4.pcuIndex.mshrWay      := s4_g.task.taskMes.mshrWay
  flush_s4.pcuMes.useAddr        := s4_g.task.taskMes.useAddr
  flush_s4.pcuMes.selfWay        := OHToUInt(s4_g.dirRes.s.wayOH)
  flush_s4.pcuMes.toDCU          := false.B


  /*
   * Send Commit to Intf
   */
  commit_s4                       := DontCare
  commit_s4.chiIndex              := s4_g.task.chiIndex
  commit_s4.chiMes.channel        := s4_g.decode.respChnl
  commit_s4.chiMes.expCompAck     := s4_g.task.chiMes.expCompAck
  commit_s4.chiMes.opcode         := s4_g.decode.respOp
  commit_s4.chiMes.resp           := s4_g.decode.resp
  commit_s4.from                  := io.dcuID
  commit_s4.to                    := IncoID.LOCALSLV.U
  commit_s4.pcuIndex.dbID         := dbid_s4.bits
  HardwareAssertion(Mux(valid_s4_g & s4_g.decode.commit & s4_g.decode.respChnl === CHIChannel.DAT, dbid_s4.valid, true.B))
  commit_s4.pcuIndex.mshrSet      := s4_g.task.taskMes.mSet
  commit_s4.pcuIndex.mshrWay      := s4_g.task.taskMes.mshrWay
  commit_s4.pcuMes.useAddr        := s4_g.task.taskMes.useAddr


  /*
   * Send Snoop Evict to RN-F
   */
  taskSnpEvict_s4                       := DontCare
  taskSnpEvict_s4.chiIndex.size         := chiFullSize.U
  taskSnpEvict_s4.chiIndex.offset       := 0.U
  taskSnpEvict_s4.chiMes.channel        := CHIChannel.SNP
  taskSnpEvict_s4.chiMes.doNotGoToSD    := true.B
  taskSnpEvict_s4.chiMes.retToSrc       := true.B
  taskSnpEvict_s4.chiMes.opcode         := SnpUnique
  taskSnpEvict_s4.from                  := io.dcuID
  taskSnpEvict_s4.to                    := IncoID.LOCALSLV.U
  taskSnpEvict_s4.pcuIndex.mshrWay      := s4_g.task.taskMes.mshrWay
  taskSnpEvict_s4.pcuMes.useAddr        := s4_g.dirRes.sf.useAddr
  taskSnpEvict_s4.pcuMes.snpTgtVec      := s4_g.dirRes.sf.metaVec.map(!_.isInvalid)
  taskSnpEvict_s4.pcuMes.snpNeedDB      := true.B


  /*
   * Set Write to Self Directory Task Value
   */
  wSDir_s4.useAddr          := s4_g.task.taskMes.useAddr
  wSDir_s4.wayOH            := s4_g.dirRes.s.wayOH
  wSDir_s4.metaVec(0).state := s4_g.decode.hnState
  wSDir_s4.replMes          := s4_g.dirRes.s.replMes


  /*
   * Set Write to Snoop Filter Directory Task Value
   */
  wSFDir_s4.useAddr         := s4_g.task.taskMes.useAddr
  wSFDir_s4.wayOH           := s4_g.dirRes.sf.wayOH
  wSFDir_s4.replMes         := s4_g.dirRes.sf.replMes
  wSFDir_s4.metaVec.zipWithIndex.foreach {
    case(a, i) =>
      when(RespType.isSnpX(s4_g.respType)) {
        when(s4_g.snpNodeVec(i))          { a.state := s4_g.decode.othState }
        .elsewhen(i.U === s4_g.srcMetaID) { a.state := s4_g.decode.srcState }
        .otherwise                        { a.state := s4_g.dirRes.sf.metaVec(i).state }
        hwaFlags(0) := Mux(valid_s4_g & s4_g.decode.wSFDir, RespType.isSnpX(s4_g.respType) | RespType.isCB(s4_g.respType), true.B)
      }.otherwise {
        when(i.U === s4_g.srcMetaID)      { a.state := s4_g.decode.srcState }
        .otherwise                        { a.state := s4_g.dirRes.sf.metaVec(i).state; }
      }
      when((!RespType.isSnpX(s4_g.respType))&(!(i.U === s4_g.srcMetaID))){
        hwaFlags(1) := Mux(valid_s4_g & s4_g.decode.wSFDir, a.state === s4_g.decode.othState | a.state === ChiState.I, true.B)
      }
      HardwareAssertion(hwaFlags(0))
      HardwareAssertion(hwaFlags(1))
  }
  HardwareAssertion.placePipe(1)


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- S4_Execute: Update MSHR ------------------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Update MSHR Mes or let task retry
   */
  io.updMSHR.bits.mshrSet     := s4_g.task.taskMes.mSet
  io.updMSHR.bits.mshrWay     := s4_g.task.taskMes.mshrWay
  io.updMSHR.bits.updType     := Mux(s4_g.todo_retry, UpdMSHRType.RETRY, UpdMSHRType.UPD)
  io.updMSHR.bits.waitIntfVec := (Mux(s4_g.decode.needWaitSlv | s4_g.todo_sfEvict, UIntToOH(IncoID.LOCALSLV.U), (s4_g.decode.readDown | s4_g.decode.readDCU) & s4_g.task.chiMes.expCompAck & djparam.openDMT.asBool) |
                                  Mux(s4_g.decode.needWaitMst | s4_g.todo_replace, UIntToOH(IncoID.LOCALMST.U), 0.U)).asBools
  io.updMSHR.bits.mTag        := Mux(s4_g.todo_replace, s4_g.dirRes.s.mTag, Mux(s4_g.todo_sfEvict, s4_g.dirRes.sf.mTag, s4_g.task.taskMes.mTag))
  HardwareAssertion(!((s4_g.decode.needWaitSlv | s4_g.todo_sfEvict) & ((s4_g.decode.readDown | s4_g.decode.readDCU) & s4_g.task.chiMes.expCompAck & djparam.openDMT.B)))
  // Only Use In New Req
  io.updMSHR.bits.hasNewReq   := s4_g.todo_replace | s4_g.todo_sfEvict
  io.updMSHR.bits.opcode      := Mux(s4_g.todo_replace, Replace, SnpUniqueEvict)
  io.updMSHR.bits.channel     := Mux(s4_g.todo_replace, CHIChannel.REQ, CHIChannel.SNP)
  // Unlock MSHR
  io.updMSHR.bits.unlock      := s4_g.task.taskMes.readDir & !s4_g.todo_replace
  // Common
  io.updMSHR.valid            := valid_s4_g & !done_s4_g_updMSHR & dirDone
  HardwareAssertion(Mux(valid_s4_g, s4_g.todo_retry | s4_g.todo_updMSHR | s4_g.todo_cleanMSHR, true.B))
  done_s4_g_updMSHR           := Mux(s4Fire, false.B, done_s4_g_updMSHR | io.updMSHR.fire)
  // updata mshr
  comUpdMSHR                  := done_s4_g_updMSHR | io.updMSHR.fire



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------ S4_Execute: Execute specific tasks value based on decode results -----------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Send Req to Node
   */
  todo_s4           := s4_g.decode
  val reqDoneList   = Seq(done_s4_g.snoop, done_s4_g.readDown, done_s4_g.writeDown, done_s4_g.readDCU, done_s4_g.writeDCU, done_s4_g_sfEvict, done_s4_g.flush)
  val reqTodoList   = Seq(todo_s4.snoop     & !done_s4_g.snoop,
                          todo_s4.readDown  & !done_s4_g.readDown,
                          todo_s4.writeDown & !done_s4_g.writeDown,
                          todo_s4.readDCU   & !done_s4_g.readDCU,
                          todo_s4.writeDCU  & !done_s4_g.writeDCU,
                          s4_g.todo_sfEvict & !done_s4_g_sfEvict,
                          todo_s4.flush     & !done_s4_g.flush)
  val toBeSendId    = PriorityEncoder(reqTodoList)
  reqBeSend_s4(0)   := taskSnp_s4
  reqBeSend_s4(1)   := taskRD_s4
  reqBeSend_s4(2)   := taskWD_s4
  reqBeSend_s4(3)   := readDCU_s4
  reqBeSend_s4(4)   := Mux(s4_g.todo_replace, taskRepl_s4, writeDCU_s4) // writeDCU transfer to taskRepl_s4
  reqBeSend_s4(5)   := taskSnpEvict_s4
  reqBeSend_s4(6)   := flush_s4
  io.req2Intf.valid := valid_s4_g & !s4_g.todo_retry & reqTodoList.reduce(_ | _)
  io.req2Intf.bits  := reqBeSend_s4(toBeSendId)
  reqDoneList.zipWithIndex.foreach { case(d, i) => d := Mux(s4Fire, false.B, d | (io.req2Intf.fire & toBeSendId === i.U)) }
  // req
  reqDone           := s4_g.todo_retry | PopCount(reqTodoList) === 0.U | (PopCount(reqTodoList) === 1.U & io.req2Intf.fire)


  /*
   * Send Write Req to Directory
   */
  // self
  io.dirWrite.s.valid   := valid_s4_g & !s4_g.todo_retry & todo_s4.wSDir & !done_s4_g.wSDir
  io.dirWrite.s.bits    := wSDir_s4
  done_s4_g.wSDir       := Mux(s4Fire, false.B, done_s4_g.wSDir | io.dirWrite.s.fire)
  // sf
  io.dirWrite.sf.valid  := valid_s4_g & !s4_g.todo_retry & todo_s4.wSFDir & !done_s4_g.wSFDir
  io.dirWrite.sf.bits   := wSFDir_s4
  done_s4_g.wSFDir      := Mux(s4Fire, false.B, done_s4_g.wSFDir | io.dirWrite.sf.fire)
  // dir
  val dirTodoList       = Seq(todo_s4.wSDir  & !done_s4_g.wSDir  & !io.dirWrite.s.fire,
                              todo_s4.wSFDir & !done_s4_g.wSFDir & !io.dirWrite.sf.fire)
  dirDone               := s4_g.todo_retry | PopCount(dirTodoList) === 0.U


  /*
   * Send Read or Clean Req to DataBuffer
   */
  io.dbRCReq.valid      := valid_s4_g & !s4_g.todo_retry & ((todo_s4.rDB2Src & !done_s4_g.rDB2Src) | (todo_s4.cleanDB & !done_s4_g.cleanDB))
  io.dbRCReq.bits       := rcDBReq_s4
  done_s4_g.rDB2Src     := Mux(s4Fire, false.B, done_s4_g.rDB2Src | (io.dbRCReq.fire & io.dbRCReq.bits.isRead))
  done_s4_g.cleanDB     := Mux(s4Fire, false.B, done_s4_g.cleanDB | (io.dbRCReq.fire & io.dbRCReq.bits.isClean))
  val rcDBTodoList      = Seq(todo_s4.rDB2Src & !done_s4_g.rDB2Src & !io.dbRCReq.fire,
                              todo_s4.cleanDB & !done_s4_g.cleanDB & !io.dbRCReq.fire)
  rcDBDone              := s4_g.todo_retry | PopCount(rcDBTodoList) === 0.U

  /*
   * Send Commit to S4
   */
  io.resp2Intf.valid    := valid_s4_g & !s4_g.todo_retry & todo_s4.commit & !done_s4_g.commit
  io.resp2Intf.bits     := commit_s4
  done_s4_g.commit      := Mux(s4Fire, false.B, done_s4_g.commit | io.resp2Intf.fire)
  comDone               := s4_g.todo_retry | !(todo_s4.commit & !done_s4_g.commit & !io.resp2Intf.fire)

  /*
   * Set Can Go S3 Value
   */
  canGo_s4              := valid_s4_g & comUpdMSHR & reqDone & dirDone & rcDBDone & comDone


// ----------------------------------------------------- Assertion ------------------------------------------------------ //
  // S4
  val cnt_s4_g  = RegInit(0.U(64.W))
  cnt_s4_g      := Mux(!valid_s4_g | canGo_s4, 0.U, cnt_s4_g + 1.U)
  HardwareAssertion.checkTimeout(!valid_s4_g | canGo_s4, TIMEOUT_PIPEEXU, cf"ProcessPipe[0x${s4_g.task.taskMes.pipeID}] EXECUTE ADDR[0x${s4_g.task.fullAddr(io.dcuID, io.pcuID)}] OP[0x${s4_g.task.chiMes.opcode}] TIMEOUT", s4_g.task.taskMes.pipeID, s4_g.task.chiMes.opcode)


  // Other
  HardwareAssertion(!valid_s4_g | !todo_s4.asUInt.asBools.zip(done_s4_g.asUInt.asBools).map { case(todo, done) => !todo & done }.reduce(_ | _))



// -------------------------------------------------- Perf Counter ------------------------------------------------------ //
  // read
  XSPerfAccumulate("pcu_pipe_req_read_cnt",       valid_s3 & canGo_s3 & isReadX(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_read_hit_cnt",   valid_s3 & canGo_s3 & isReadX(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_read_miss_cnt",  valid_s3 & canGo_s3 & isReadX(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq & !todo_s3_retry & !hnHit_s3)
  // write
  XSPerfAccumulate("pcu_pipe_req_write_cnt",      valid_s3 & canGo_s3 & isWriteX(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_write_hit_cnt",  valid_s3 & canGo_s3 & isWriteX(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_write_miss_cnt", valid_s3 & canGo_s3 & isWriteX(task_s3_g.bits.chiMes.opcode) & task_s3_g.bits.chiMes.isReq & !todo_s3_retry & !hnHit_s3)
  // evict
  XSPerfAccumulate("pcu_pipe_req_evict_cnt",      valid_s3 & canGo_s3 & task_s3_g.bits.chiMes.opcode === Evict & task_s3_g.bits.chiMes.isReq & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_evict_hit_cnt",  valid_s3 & canGo_s3 & task_s3_g.bits.chiMes.opcode === Evict & task_s3_g.bits.chiMes.isReq & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_evict_miss_cnt", valid_s3 & canGo_s3 & task_s3_g.bits.chiMes.opcode === Evict & task_s3_g.bits.chiMes.isReq & !todo_s3_retry & !hnHit_s3)
  // makeUnique
  XSPerfAccumulate("pcu_pipe_req_makeUnique_cnt",       valid_s3 & canGo_s3 & task_s3_g.bits.chiMes.opcode === MakeUnique & task_s3_g.bits.chiMes.isReq & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_makeUnique_hit_cnt",   valid_s3 & canGo_s3 & task_s3_g.bits.chiMes.opcode === MakeUnique & task_s3_g.bits.chiMes.isReq & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_makeUnique_miss_cnt",  valid_s3 & canGo_s3 & task_s3_g.bits.chiMes.opcode === MakeUnique & task_s3_g.bits.chiMes.isReq & !todo_s3_retry & !hnHit_s3)
  // total
  XSPerfAccumulate("pcu_pipe_req_total_cnt",      valid_s3 & canGo_s3 & !todo_s3_retry)
  XSPerfAccumulate("pcu_pipe_req_hit_total_cnt",  valid_s3 & canGo_s3 & !todo_s3_retry & hnHit_s3)
  XSPerfAccumulate("pcu_pipe_req_miss_total_cnt", valid_s3 & canGo_s3 & !todo_s3_retry & !hnHit_s3)
  XSPerfAccumulate("pcu_pipe_retry_total_cnt",    valid_s3 & canGo_s3 & todo_s3_retry)


  HardwareAssertion.placePipe(2)
}