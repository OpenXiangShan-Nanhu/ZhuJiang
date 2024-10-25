package dongjiang.pcu.exu

import dongjiang._
import dongjiang.pcu._
import dongjiang.pcu.exu.decode._
import dongjiang.chi._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.ParallelLookUp

class ProcessPipe(implicit p: Parameters) extends DJModule {
// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val hnfID       = Input(UInt(chiNodeIdBits.W))
    val sliceId     = Input(UInt(bankBits.W))
    // Req To DataBuffer
    val dbRCReq   = Decoupled(new DBRCReq())
    // Resp From Directory
    val dirResp     = Flipped(Valid(new DirRespBundle()))
    // Write Req To Directory
    val dirWrite    = new DirWriteBundle()
    // Task From MSHR
    val task        = Flipped(Decoupled(new PipeTaskBundle()))
    // Update Task To MSHR
    val updMSHR     = Decoupled(new UpdateMSHRReqBundle())
    val updLockMSHR = Valid(new MSHRSetBundle)
    // Req To Node
    val req2Node    = Decoupled(new Req2NodeBundle())
    // Resp To Node
    val resp2Node   = Decoupled(new Resp2NodeBundle())
  })

  // TODO: Delete the following code when the coding is complete
  dontTouch(io)

// --------------------- Modules declaration ------------------------//
  val taskQ   = Module(new Queue(new PipeTaskBundle(), entries = djparam.nrMpTaskQueue, pipe = false, flow = false))
  val dirResQ = Module(new Queue(new DirRespBundle(), entries = djparam.nrMpTaskQueue + 2, pipe = true, flow = true)) // one for mp_s1 read Dir before send task to mp_2, one for mp_s3

// --------------------- Reg/Wire declaration ------------------------//
  // s2 signals
  val canGo_s2            = Wire(Bool())
  val task_s2             = WireInit(0.U.asTypeOf(Valid(new PipeTaskBundle())))
  // s3 basic signals
  val valid_s3            = Wire(Bool())
  val canGo_s3            = Wire(Bool());
  val dirCanGo_s3         = Wire(Bool())
  val taskNext_s3         = WireInit(0.U.asTypeOf(new PipeTaskBundle()))
  val task_s3_g           = RegInit(0.U.asTypeOf(Valid(new PipeTaskBundle())))
  val dirRes_s3           = WireInit(0.U.asTypeOf(Valid(new DirRespBundle())))
  val taskChipType        = Wire(UInt(ChipType.width.W))
  val srcMetaId           = Wire(UInt(rnfNodeIdBits.W))
  val sfHitVec            = Wire(Vec(nrRnfNode, Bool()))
  val othHitVec           = Wire(Vec(nrRnfNode, Bool()))
  // s3 decode base signals
  val inst_s3             = Wire(new InstBundle());  dontTouch(inst_s3)
  val decode_s3           = Wire(new DecodeBundle());  dontTouch(decode_s3)
  // s3 execute signals: Set specific tasks value
  val taskSnp_s3          = WireInit(0.U.asTypeOf(new Req2NodeBundle()))
  val taskRD_s3           = WireInit(0.U.asTypeOf(new Req2NodeBundle()))
  val taskWD_s3           = WireInit(0.U.asTypeOf(new Req2NodeBundle()))
  val rcDBReq_s3          = WireInit(0.U.asTypeOf(new DBRCReq()))
  val readDCU_s3          = WireInit(0.U.asTypeOf(new Req2NodeBundle()))
  val writeDCU_s3         = WireInit(0.U.asTypeOf(new Req2NodeBundle()))
  val wSDir_s3            = WireInit(0.U.asTypeOf(io.dirWrite.s.bits))
  val wSFDir_s3           = WireInit(0.U.asTypeOf(io.dirWrite.sf.bits))
  val commit_s3           = WireInit(0.U.asTypeOf(new Resp2NodeBundle()))
  val taskRepl_s3         = WireInit(0.U.asTypeOf(new Req2NodeBundle()))
  val taskSnpEvict_s3     = WireInit(0.U.asTypeOf(new Req2NodeBundle()))
  // s3 execute signals: task to do list
  val todo_s3             = WireInit(0.U.asTypeOf(new OperationsBundle()))
  val todo_s3_retry       = Wire(Bool())
  val todo_s3_updateMSHR  = Wire(Bool())
  val todo_s3_replace     = Wire(Bool()) // replace self Directory
  val todo_s3_sfEvict     = Wire(Bool()) // replace snoop filter
  val todo_s3_cleanMSHR   = Wire(Bool())
  // s3 execute signals: Execute specific tasks
  val done_s3_g           = RegInit(0.U.asTypeOf(new OperationsBundle()))
  val done_s3_g_updMSHR   = RegInit(false.B)
  val done_s3_g_sfEvict   = RegInit(false.B)
  val reqBeSend_s3        = Wire(Vec(6, new Req2NodeBundle()))






// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- S2: Buffer input task/dirRes ----------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  // task queue
  taskQ.io.enq          <> io.task
  task_s2.valid         := taskQ.io.deq.valid
  task_s2.bits          := taskQ.io.deq.bits
  taskQ.io.deq.ready    := canGo_s2

  canGo_s2              := canGo_s3 | !task_s3_g.valid

  // dir result queue
  dirResQ.io.enq.valid  := io.dirResp.valid
  dirResQ.io.enq.bits   := io.dirResp.bits
  assert(Mux(io.dirResp.valid, dirResQ.io.enq.ready, true.B))

  // Check Get Dir Result
  // TODO: parameterization
  val enqShiftReg       = RegInit(0.U(3.W))
  val alreadyGetDir     = RegInit(false.B)
  enqShiftReg           := Cat(taskQ.io.enq.fire & taskQ.io.enq.bits.readDir, enqShiftReg(2, 1))
  alreadyGetDir         := Mux(alreadyGetDir, !taskQ.io.enq.fire, dirResQ.io.enq.fire & !enqShiftReg(0))
  assert(Mux(enqShiftReg(0), dirResQ.io.enq.fire | alreadyGetDir, true.B), "ProcessPipe need get dir result")


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------- S3_Receive: Receive task and dirRes from s2 -------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Reset Done Reg When Get New Task Form S2
   */
  val s2Fire = task_s2.valid & canGo_s2
  val rstDone = s2Fire


  /*
   * Recieve task_s2
   */
  task_s3_g.valid       := Mux(task_s2.valid, true.B, task_s3_g.valid & !canGo_s3)
  taskNext_s3           := Mux(s2Fire, task_s2.bits, task_s3_g.bits)
  task_s3_g.bits        := taskNext_s3

  /*
   * Recieve dirRes
   */
  dirRes_s3.valid       := dirResQ.io.deq.valid
  dirRes_s3.bits        := dirResQ.io.deq.bits
  dirResQ.io.deq.ready  := dirCanGo_s3

  /*
   * S3 base ctrl logic
   */
  dirCanGo_s3           := canGo_s3 & task_s3_g.valid & taskNext_s3.readDir
  valid_s3              := Mux(task_s3_g.bits.readDir, task_s3_g.valid & dirRes_s3.valid, task_s3_g.valid)



// ---------------------------------------------------------------------------------------------------------------------- //
// --------------------------------- S3_Decode: Decode by task Message and Dir Result ------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Parse Dir Result
   */
  srcMetaId     := getMetaIdByNodeID(task_s3_g.bits.reqMes.srcID)
  val srcHit    = dirRes_s3.bits.sf.hit & !dirRes_s3.bits.sf.metaVec(srcMetaId).isInvalid
  val srcState  = Mux(srcHit, dirRes_s3.bits.sf.metaVec(srcMetaId).state, ChiState.I)

  val othHit    = dirRes_s3.bits.sf.hit & (PopCount(dirRes_s3.bits.sf.metaVec.map(!_.isInvalid)) > srcHit.asUInt)
  val sfHitId   = PriorityEncoder(dirRes_s3.bits.sf.metaVec.map(!_.isInvalid))
  val othState  = Mux(othHit, dirRes_s3.bits.sf.metaVec(sfHitId).state, ChiState.I)

  val hnHit     = dirRes_s3.bits.s.hit
  val hnState   = Mux(hnHit, dirRes_s3.bits.s.metaVec(0).state, ChiState.I)

  /*
   * Set Inst value
   */
  taskChipType      := getChipTypeByAddr(task_s3_g.bits.addr)
  inst_s3.chipType  := taskChipType
  inst_s3.channel   := task_s3_g.bits.reqMes.channel
  inst_s3.opcode    := task_s3_g.bits.reqMes.opcode
  inst_s3.srcState  := Mux(task_s3_g.bits.readDir, srcState, ChiState.I)
  inst_s3.othState  := Mux(task_s3_g.bits.readDir, othState, ChiState.I)
  inst_s3.hnState   := Mux(task_s3_g.bits.readDir, hnState, ChiState.I)
  inst_s3.respType  := Cat(task_s3_g.bits.respMes.masResp.valid,  // Read Down Resp
                           task_s3_g.bits.respMes.fwdState.valid, // Snoop Fwd Resp
                           task_s3_g.bits.respMes.slvResp.valid)  // Snoop Resp
  inst_s3.snpResp   := task_s3_g.bits.respMes.slvResp.bits
  inst_s3.fwdState  := task_s3_g.bits.respMes.fwdState.bits
  inst_s3.rdResp    := task_s3_g.bits.respMes.masResp.bits
  inst_s3.respHasData := (task_s3_g.bits.respMes.masDBID.valid | task_s3_g.bits.respMes.slvDBID.valid).asUInt
  assert(Mux(valid_s3, !inst_s3.chipType === ChipType.CSN, true.B), "TODO")

  /*
   * Get Decode Result
   */
  val table = LocalReadDecode.table ++ LoaclSnpUniqueEvictDecode.table ++ LoaclDatalessDecode.table ++ LoaclWriteDecode.table
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
  decode_s3.decode(inst_s3, table)
  when(valid_s3) { assert(decode_s3.asUInt =/= 0.U,
    "\n\nADDR[0x%x] DECODE ERROR: No inst match in decode table\n" +
      "INST: CHIP[0x%x] CHNL[0x%x] OP[0x%x] SRC[0x%x] OTH[0x%x] HN[0x%x] RESP[0x%x] DATA[0x%x] SNP[0x%x] FWD[0x%x] RD[0x%x]\n", task_s3_g.bits.addr,
    inst_s3.chipType, inst_s3.channel, inst_s3.opcode, inst_s3.srcState, inst_s3.othState, inst_s3.hnState, inst_s3.respType, inst_s3.respHasData, inst_s3.snpResp, inst_s3.fwdState, inst_s3.rdResp) }
  when(valid_s3) { when(decode_s3.wSDir)  { assert(decode_s3.commit | (inst_s3.opcode === CHIOp.SNP.SnpUniqueEvict & inst_s3.channel === CHIChannel.SNP) | (CHIOp.REQ.isWriteX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ)) } }
  when(valid_s3) { when(decode_s3.wSFDir) { assert(decode_s3.commit | (CHIOp.REQ.isWriteX(inst_s3.opcode) & inst_s3.channel === CHIChannel.REQ)) } }


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------- S3_Execute: Set specific tasks value -----------------------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Send Snoop to RN-F
   */
  // taskSnp_s3
  // TODO: Combine the following judgment logic into the S3_Decode
  sfHitVec                := dirRes_s3.bits.sf.metaVec.map(!_.isInvalid)
  othHitVec.zipWithIndex.foreach { case(hit, i) => hit := dirRes_s3.bits.sf.hit & sfHitVec(i) & srcMetaId =/= i.U }

  taskSnp_s3              := DontCare
  taskSnp_s3.channel      := CHIChannel.SNP
  taskSnp_s3.addr         := task_s3_g.bits.addr
  taskSnp_s3.mshrWay      := task_s3_g.bits.mshrWay
  taskSnp_s3.from.IncoId  := io.sliceId
  taskSnp_s3.to.IncoId    := IncoID.LOCALSLV.U; assert(Mux(valid_s3, !inst_s3.chipType === ChipType.CSN, true.B), "TODO")
  taskSnp_s3.tgtID        := othHitVec.asUInt; require(taskSnp_s3.tgtID.getWidth >= othHitVec.getWidth)
  taskSnp_s3.srcID        := DontCare
  taskSnp_s3.txnID        := DontCare
  taskSnp_s3.opcode       := decode_s3.snpOp
  taskSnp_s3.retToSrc     := decode_s3.retToSrc
  taskSnp_s3.doNotGoToSD  := true.B


  /*
   * Send Read to SN(DDRC) / HN-F(CSN)
   */
  taskRD_s3             := DontCare
  taskRD_s3.addr        := task_s3_g.bits.addr
  taskRD_s3.mshrWay     := task_s3_g.bits.mshrWay
  taskRD_s3.tgtID       := Mux(taskChipType === ChipType.Local, ddrcNodeId.U, DontCare); assert(Mux(valid_s3, !inst_s3.chipType === ChipType.CSN, true.B), "TODO")
  taskRD_s3.srcID       := Mux(taskChipType === ChipType.Local, io.hnfID, csnHnfNodeId.U)
  taskRD_s3.opcode      := decode_s3.rdOp
  taskRD_s3.expCompAck  := Mux(taskChipType === ChipType.Local, false.B, true.B)
  taskRD_s3.from.IncoId := io.sliceId
  taskRD_s3.to.IncoId   := Mux(taskChipType === ChipType.Local, IncoID.LOCALMAS.U, IncoID.CSNMAS.U)

  /*
   * Send Write / Dataless to SN(DDRC) / HN-F(CSN)
   */
  // taskWD_s3
  assert(Mux(valid_s3, !decode_s3.writeDown, true.B), "TODO")

  /*
   * Send Read / Clean to DataBuffer
   */
  val rcDBID            = Mux(task_s3_g.bits.respMes.slvDBID.valid, task_s3_g.bits.respMes.slvDBID.bits, task_s3_g.bits.respMes.masDBID.bits)
  assert(!(task_s3_g.bits.respMes.slvDBID.valid & task_s3_g.bits.respMes.masDBID.valid))
  rcDBReq_s3.to         := task_s3_g.bits.reqMes.from
  rcDBReq_s3.isRead     := decode_s3.rDB2Src
  rcDBReq_s3.isClean    := decode_s3.cleanDB
  rcDBReq_s3.dbid       := rcDBID


  /*
   * Send Read to SN(DCU)
   */
  // readDCU_s3
  readDCU_s3.addr           := task_s3_g.bits.addr
  readDCU_s3.selfWay        := OHToUInt(dirRes_s3.bits.s.wayOH)
  readDCU_s3.mshrWay        := task_s3_g.bits.mshrWay
  readDCU_s3.channel        := CHIChannel.REQ
  readDCU_s3.opcode         := decode_s3.rdOp
  readDCU_s3.from.IncoId    := io.sliceId
  readDCU_s3.to.IncoId      := IncoID.LOCALMAS.U
  readDCU_s3.tgtID          := getSnNodeIDByBankId(task_s3_g.bits.bank, 0)
  readDCU_s3.srcID          := task_s3_g.bits.reqMes.srcID
  readDCU_s3.resp           := decode_s3.resp


  /*
   * Send Write to SN(DCU)
   */
  // writeDCU_s3
  writeDCU_s3.addr          := task_s3_g.bits.addr
  writeDCU_s3.selfWay       := OHToUInt(dirRes_s3.bits.s.wayOH)
  writeDCU_s3.mshrWay       := task_s3_g.bits.mshrWay
  writeDCU_s3.channel       := CHIChannel.REQ
  writeDCU_s3.opcode        := decode_s3.wdOp
  writeDCU_s3.from.IncoId   := io.sliceId
  writeDCU_s3.to.IncoId     := IncoID.LOCALMAS.U
  writeDCU_s3.tgtID         := getSnNodeIDByBankId(task_s3_g.bits.bank, 1)
  writeDCU_s3.srcID         := task_s3_g.bits.reqMes.srcID
  writeDCU_s3.dbid          := rcDBID


  /*
   * Send Write to SN(DCU)
   */
  // writeDCU_s3
  taskRepl_s3.addr          := dirRes_s3.bits.s.addr
  taskRepl_s3.selfWay       := OHToUInt(dirRes_s3.bits.s.wayOH)
  taskRepl_s3.mshrWay       := task_s3_g.bits.mshrWay
  taskRepl_s3.channel       := CHIChannel.REQ
  taskRepl_s3.opcode        := CHIOp.REQ.Replace
  taskRepl_s3.from.IncoId   := io.sliceId
  taskRepl_s3.to.IncoId     := IncoID.LOCALMAS.U
  taskRepl_s3.tgtID         := getSnNodeIDByBankId(task_s3_g.bits.bank, 2)
  taskRepl_s3.srcID         := io.hnfID
  taskRepl_s3.dbid          := rcDBID


  /*
   * Send Write to Self Directory
   */
  wSDir_s3.addr             := task_s3_g.bits.addr
  wSDir_s3.wayOH            := dirRes_s3.bits.s.wayOH
  wSDir_s3.metaVec(0).state := decode_s3.hnState
  wSDir_s3.replMes          := dirRes_s3.bits.s.replMes

  /*
   * Send Write to Snoop Filter Directory
   */
  wSFDir_s3.addr            := task_s3_g.bits.addr
  wSFDir_s3.wayOH           := dirRes_s3.bits.sf.wayOH
  wSFDir_s3.replMes         := dirRes_s3.bits.sf.replMes
  wSFDir_s3.metaVec.zip(dirRes_s3.bits.sf.metaVec).zipWithIndex.foreach {
    case((a, b), i) =>
      when(!b.isInvalid & i.U =/= srcMetaId) { a.state := decode_s3.othState }
      .elsewhen(i.U === srcMetaId)           { a.state := decode_s3.srcState }
      .otherwise                             { a.state := ChiState.I }
  }

  /*
   * Send Commit to S4
   */
  commit_s3.addr            := task_s3_g.bits.addr
  commit_s3.channel         := decode_s3.respChnl
  commit_s3.opcode          := decode_s3.respOp
  commit_s3.srcID           := task_s3_g.bits.reqMes.srcID
  commit_s3.txnID           := task_s3_g.bits.reqMes.txnID
  commit_s3.resp            := decode_s3.resp
  commit_s3.dbid            := rcDBID
  commit_s3.needReadDB      := !decode_s3.rDB2Src & decode_s3.respChnl === CHIChannel.DAT
  commit_s3.fwdState        := DontCare
  commit_s3.expCompAck      := task_s3_g.bits.reqMes.expCompAck
  commit_s3.from.IncoId     := io.sliceId
  commit_s3.to              := task_s3_g.bits.reqMes.from


  /*
   * Send Snoop Evict to RN-F
   */
  taskSnpEvict_s3             := DontCare
  taskSnpEvict_s3.channel     := CHIChannel.SNP
  taskSnpEvict_s3.addr        := dirRes_s3.bits.sf.addr
  taskSnpEvict_s3.from.IncoId := io.sliceId
  taskSnpEvict_s3.to.IncoId   := IncoID.LOCALSLV.U; assert(Mux(valid_s3, !inst_s3.chipType === ChipType.CSN, true.B), "TODO")
  taskSnpEvict_s3.mshrWay     := task_s3_g.bits.mshrWay
  taskSnpEvict_s3.tgtID       := sfHitVec.asUInt
  taskSnpEvict_s3.srcID       := DontCare
  taskSnpEvict_s3.txnID       := DontCare
  taskSnpEvict_s3.opcode      := CHIOp.SNP.SnpUnique
  taskSnpEvict_s3.retToSrc    := true.B
  taskSnpEvict_s3.doNotGoToSD := true.B



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
  todo_s3_replace     := todo_s3.wSDir & !hnHit & !dirRes_s3.bits.s.metaVec(0).isInvalid & !todo_s3_retry
  todo_s3_sfEvict     := todo_s3.wSFDir & !srcHit & !othHit & dirRes_s3.bits.sf.metaVec.map(!_.isInvalid).reduce(_ | _) & !todo_s3_retry
  todo_s3_updateMSHR  := todo_s3.reqToMas | todo_s3.reqToSlv | todo_s3_replace | todo_s3_sfEvict
  todo_s3_cleanMSHR   := !(todo_s3_retry | todo_s3_updateMSHR)
  assert(Mux(valid_s3, PopCount(Seq(todo_s3_retry, todo_s3_updateMSHR, todo_s3_cleanMSHR)) === 1.U, true.B))
  assert(Mux(valid_s3, PopCount(Seq(todo_s3_replace, todo_s3_sfEvict)) <= 1.U, true.B))
  assert(Mux(valid_s3 & todo_s3_replace, todo_s3.writeDCU, true.B))

  /*
   * Update MSHR Mes or let task retry
   */
  io.updMSHR.bits.addr        := Mux(todo_s3_replace | todo_s3_sfEvict, Mux(todo_s3_replace, dirRes_s3.bits.s.addr, dirRes_s3.bits.sf.addr), task_s3_g.bits.addr)
  io.updMSHR.bits.updType     := Mux(todo_s3_retry, UpdMSHRType.RETRY, UpdMSHRType.UPD)
  io.updMSHR.bits.mshrWay     := task_s3_g.bits.mshrWay
  // Use In Update // TODO: Complete hasCSNIntf
  io.updMSHR.bits.waitIntfVec := (Mux(todo_s3.reqToSlv | todo_s3_sfEvict, UIntToOH(IncoID.LOCALSLV.U), 0.U) |
                                  Mux(todo_s3.reqToMas | todo_s3_replace, UIntToOH(IncoID.LOCALMAS.U), 0.U)).asBools
  require(!hasCSNIntf)
  // Use In New Req
  io.updMSHR.bits.hasNewReq   := todo_s3_replace | todo_s3_sfEvict
  io.updMSHR.bits.opcode      := Mux(todo_s3_replace, CHIOp.REQ.Replace, CHIOp.SNP.SnpUniqueEvict)
  io.updMSHR.bits.channel     := Mux(todo_s3_replace, CHIChannel.REQ,    CHIChannel.SNP)
  io.updMSHR.bits.lockDirSet  := todo_s3_replace
  // Common
  io.updMSHR.valid            := valid_s3 & (todo_s3_retry | todo_s3_updateMSHR | todo_s3_cleanMSHR) & !done_s3_g_updMSHR
  done_s3_g_updMSHR           := Mux(rstDone, false.B, done_s3_g_updMSHR | io.updMSHR.fire)
  val updDone                 = io.updMSHR.fire | done_s3_g_updMSHR


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------ S3_Execute: Execute specific tasks value based on decode results -----------------------------//
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Send Req to Node
   */
  val canSendReq    = valid_s3 & !todo_s3_retry
  val reqDoneList   = Seq(done_s3_g.snoop, done_s3_g.readDown, done_s3_g.writeDown, done_s3_g.readDCU, done_s3_g.writeDCU, done_s3_g_sfEvict)
  val reqTodoList   = Seq(todo_s3.snoop     & !done_s3_g.snoop,
                          todo_s3.readDown  & !done_s3_g.readDown,
                          todo_s3.writeDown & !done_s3_g.writeDown,
                          todo_s3.readDCU   & !done_s3_g.readDCU,
                          todo_s3.writeDCU  & !done_s3_g.writeDCU,
                          todo_s3_sfEvict   & !done_s3_g_sfEvict)
  val toBeSendId    = PriorityEncoder(reqTodoList)
  reqBeSend_s3(0)   := taskSnp_s3
  reqBeSend_s3(1)   := taskRD_s3
  reqBeSend_s3(2)   := taskWD_s3
  reqBeSend_s3(3)   := readDCU_s3
  reqBeSend_s3(4)   := Mux(todo_s3_replace, taskRepl_s3, writeDCU_s3) // writeDCU transfer to taskRepl_s3
  reqBeSend_s3(5)   := taskSnpEvict_s3
  io.req2Node.valid := canSendReq & reqTodoList.reduce(_ | _)
  io.req2Node.bits  := reqBeSend_s3(toBeSendId)
  reqDoneList.zipWithIndex.foreach { case(d, i) => d := Mux(rstDone, false.B, d | (io.req2Node.fire & toBeSendId === i.U)) }
  // req
  val reqDone       = PopCount(reqTodoList) === 0.U | (PopCount(reqTodoList) === 1.U & io.req2Node.fire)


  /*
   * Send Write Req to Directory
   */
  // self
  io.dirWrite.s.valid   := valid_s3 & !todo_s3_retry & todo_s3.wSDir & !done_s3_g.wSDir
  io.dirWrite.s.bits    := wSDir_s3
  done_s3_g.wSDir       := Mux(rstDone, false.B, done_s3_g.wSDir | io.dirWrite.s.fire)
  // sf
  io.dirWrite.sf.valid  := valid_s3 & !todo_s3_retry & todo_s3.wSFDir & !done_s3_g.wSFDir
  io.dirWrite.sf.bits   := wSFDir_s3
  done_s3_g.wSFDir      := Mux(rstDone, false.B, done_s3_g.wSFDir | io.dirWrite.sf.fire)
  // dir
  val dirTodoList       = Seq(todo_s3.wSDir  & !done_s3_g.wSDir  & !io.dirWrite.s.fire,
                              todo_s3.wSFDir & !done_s3_g.wSFDir & !io.dirWrite.sf.fire)
  val dirDone           = PopCount(dirTodoList) === 0.U


  /*
   * Send Read or Clean Req to DataBuffer
   */
  io.dbRCReq.valid      := valid_s3 & !todo_s3_retry & ((todo_s3.rDB2Src & !done_s3_g.rDB2Src) | (todo_s3.cleanDB & !done_s3_g.cleanDB))
  io.dbRCReq.bits       := rcDBReq_s3
  done_s3_g.rDB2Src     := Mux(rstDone, false.B, done_s3_g.rDB2Src | (io.dbRCReq.fire & io.dbRCReq.bits.isRead))
  done_s3_g.cleanDB     := Mux(rstDone, false.B, done_s3_g.cleanDB | (io.dbRCReq.fire & io.dbRCReq.bits.isClean))
  val rcDBTodoList      = Seq(todo_s3.rDB2Src & !done_s3_g.rDB2Src & !io.dbRCReq.fire,
                              todo_s3.cleanDB & !done_s3_g.cleanDB & !io.dbRCReq.fire)
  val rcDBDone          = PopCount(rcDBTodoList) === 0.U

  /*
   * Send Commit to S4
   */
  io.resp2Node.valid    := valid_s3 & !todo_s3_retry & todo_s3.commit & !done_s3_g.commit
  io.resp2Node.bits     := commit_s3
  done_s3_g.commit      := Mux(rstDone, false.B, done_s3_g.commit | io.resp2Node.fire)
  val comDone           = !(todo_s3.commit & !done_s3_g.commit & !io.resp2Node.fire)

  /*
   * Set Can Go S3 Value
   */
  canGo_s3              := valid_s3 & reqDone & dirDone & rcDBDone & comDone & updDone




// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------- S3_Execute: UnLock MshrLockVec ------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Update MshrLockVec:
   * 1. S3 done but task dont need to commit
   */
  io.updLockMSHR.valid        := valid_s3 & canGo_s3 & task_s3_g.bits.readDir
  io.updLockMSHR.bits.mshrSet := task_s3_g.bits.mSet


// ----------------------------------------------------- Assertion ------------------------------------------------------ //
  // S3
  val cnt_s3_g  = RegInit(0.U(64.W))
  cnt_s3_g      := Mux(!valid_s3 | canGo_s3, 0.U, cnt_s3_g + 1.U)
  assert(cnt_s3_g < TIMEOUT_EXU.U, "ProcessPipe[0x%x] EXECUTE ADDR[0x%x] OP[0x%x] TIMEOUT", task_s3_g.bits.pipeId, task_s3_g.bits.addr, task_s3_g.bits.reqMes.opcode)

  // Other
  assert(!valid_s3 | !todo_s3.asUInt.asBools.zip(done_s3_g.asUInt.asBools).map { case(todo, done) => !todo & done }.reduce(_ | _))
}