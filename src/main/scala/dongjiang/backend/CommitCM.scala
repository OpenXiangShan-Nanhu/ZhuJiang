package dongjiang.backend

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import zhujiang.chi._
import zhujiang.chi.RspOpcode._
import zhujiang.chi.DatOpcode._
import dongjiang._
import dongjiang.utils._
import dongjiang.bundle.{HasAddr, HasPackLLCTxnID, PackChi, _}
import dongjiang.data.HasAlready
import xs.utils.debug._
import dongjiang.directory.{DirEntry, DirMsg, HasPackDirMsg}
import dongjiang.data._
import zhujiang.chi.ReqOpcode._
import dongjiang.frontend._
import dongjiang.frontend.decode._

class CMState(implicit p: Parameters) extends DJBundle {
  val intl = new DJBundle {
    // internal send
    val s = new DJBundle {
      val data0     = Bool() // Send Data Task(reqs, read, send)
      val data1     = Bool() // Send Data Task(save, clean)  // Note: Need wait repl done
      val wriDir    = Bool() // Send Write Task To Replace   // Note: Need wait data0 done
      val canNest   = Bool() // Indicate PoS This Entry Can Be Nested
      val secTask   = Bool() // Send Task To TaskCM
      // TODO: val dataFwd = Bool() // Send CompData in SnpFwd
    }
    // internal wait
    val w = new DJBundle {
      val cmResp    = Bool() // Wait TaskCM Resp
      val secResp   = Bool() // Wait Second TaskCM Resp
      val repl      = Bool() // Wait Replace Done
      val data0     = Bool() // Wait Data Task(read, send, send) Done
      val data1     = Bool() // Wait Data Task(save, clean) Done
    }
  }
  // chi send/wait
  val chi = new DJBundle {
    val s_resp      = Bool() // Send Resp(Comp, SnpResp) To RN/HN
    val w_ack       = Bool() // Wait CompAck From RN
  }
  // other
  val clean         = Bool() // Clean PoS Tag and Dir Lock
  val valid         = Bool() // Valid
}

// TODO: optimize architecture
class CommitCM(dirBank: Int)(implicit p: Parameters) extends DJModule {
  /*
   * IO declaration
   */
  val io = IO(new Bundle {
    val config      = new DJConfigIO()
    // Commit Task In
    val alloc       = Flipped(Valid(new CommitTask))
    // CHI
    val txRsp       = Decoupled(new RespFlit())
    val rxRsp       = Flipped(Valid(new RespFlit()))
    // Send Task To CM
    val cmAllocVec  = Vec(nrTaskCM, Decoupled(new CMTask))
    // Resp From TaskCM
    val respCmt     = Flipped(Valid(new RespToCmt))
    // Send Task To Replace
    val replAlloc   = Decoupled(new ReplTask)
    val replResp    = Flipped(Valid(new PackPosIndex() with HasAlready))
    // Send Task To Data
    val dataTask    = Decoupled(new DataTask)
    val dataResp    = Flipped(Valid(new PackPosIndex()))
    // Update PoS Message
    val updPosNest  = Decoupled(new PackPosIndex with HasNest)
    val cleanPos    = Valid(new PackPosIndex with HasChiChannel)
    val unlock      = Valid(new PosIndex())
  })

  /*
   * Reg and Wire declaration
   */
  val cmTable       = RegInit(VecInit(Seq.fill(posSets) { VecInit(Seq.fill(posWays) { 0.U.asTypeOf(new CMState()) }) }))
  val msgTable      = Reg(Vec(posSets, Vec(posWays, new PackChi with HasPackDirMsg with HasAlready with HasPackCmtCode
                                                      with HasDsIdx with HasPackTaskInst with HasPackTaskCode with HasSnpTgt)))
  // receive
  val cm_rec        = Wire(new CMState())
  val msg_rec       = Wire(chiselTypeOf(msgTable.head.head))
  // respCmt
  val code_rCmt     = Wire(new TaskCode())
  val cmt_rCmt      = Wire(new CommitCode())
  val cm_rCmt       = Wire(new CMState())
  // data(read, send, save, clean)
  val cmVec2_d      = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_d       = Wire(new PosIndex())
  // data0(reqs, read, send)
  val cmVec2_d0     = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_d0      = Wire(new PosIndex())
  // data1(save, clean)
  val cmVec2_d1     = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_d1      = Wire(new PosIndex())
  // write directory
  val cmVec2_wDir   = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_wDir    = Wire(new PosIndex())
  // can nest
  val cmVec2_nest   = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_nest    = Wire(new PosIndex())
  // send task
  val cmVec2_task   = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_task    = Wire(new PosIndex())
  val alloc_task    = Wire(new CMTask())
  // can nest
  val cmVec2_clean  = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_clean   = Wire(new PosIndex()); dontTouch(cmPoS_clean)
  // send resp to chi
  val cmVec2_sRsp   = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_sRsp    = Wire(new PosIndex())

  dontTouch(code_rCmt)
  dontTouch(cmt_rCmt)

  /*
   * [rec] Receive Task IO
   */
  val alloc_rec         = io.alloc.bits
  val taskVal_rec       = alloc_rec.ops.valid
  val cmtVal_rec        = alloc_rec.commit.valid
  // check decode result
  when(io.alloc.valid) {
    HardwareAssertion(taskVal_rec ^ cmtVal_rec)
    HardwareAssertion.withEn(alloc_rec.commit.asUInt === 0.U, !cmtVal_rec)
    HardwareAssertion.withEn(!taskVal_rec, io.alloc.bits.alr.sData)
    HardwareAssertion.withEn(alloc_rec.commit.channel === ChiChannel.DAT, alloc_rec.commit.dataOp.send)
  }
  // msg
  msg_rec.chi           := alloc_rec.chi
  msg_rec.dir           := alloc_rec.dir
  msg_rec.alr           := alloc_rec.alr
  msg_rec.commit        := alloc_rec.commit
  msg_rec.ds            := alloc_rec.ds
  msg_rec.inst          := DontCare
  msg_rec.code          := DontCare
  msg_rec.code.snoop    := alloc_rec.ops.snoop
  msg_rec.code.read     := alloc_rec.ops.read
  msg_rec.code.dataless := alloc_rec.ops.dataless
  msg_rec.code.wriOrAtm := alloc_rec.ops.wriOrAtm
  msg_rec.code.receive  := alloc_rec.ops.receive
  msg_rec.snpTgt        := alloc_rec.snpTgt
  // cm internal send
  cm_rec.intl.s.data0   := cm_rec.intl.w.data0 & !alloc_rec.alr.sData
  cm_rec.intl.s.data1   := cm_rec.intl.w.data1
  cm_rec.intl.s.wriDir  := alloc_rec.commit.wriDir
  cm_rec.intl.s.canNest := false.B
  cm_rec.intl.s.secTask := false.B
  // cm internal wait
  cm_rec.intl.w.cmResp  := alloc_rec.ops.valid
  cm_rec.intl.w.secResp := false.B
  cm_rec.intl.w.repl    := cm_rec.intl.s.wriDir
  cm_rec.intl.w.data0   := alloc_rec.commit.dataOp.reqs | alloc_rec.commit.dataOp.read | alloc_rec.commit.dataOp.send
  cm_rec.intl.w.data1   := alloc_rec.commit.dataOp.save | alloc_rec.commit.dataOp.clean
  // cm chi send
  cm_rec.chi.s_resp     := alloc_rec.commit.commit & alloc_rec.commit.channel === ChiChannel.RSP
  // cm chi wait
  cm_rec.chi.w_ack      := alloc_rec.chi.expCompAck
  // valid
  cm_rec.clean          := false.B
  cm_rec.valid          := true.B

  /*
   * [RESP] Receive Resp From TaskCM
   */
  val respVal       = io.respCmt.valid
  val respCmt       = io.respCmt.bits
  val pos_resp      = respCmt.llcTxnID.pos
  val cm_resp       = cmTable(pos_resp.set)(pos_resp.way)
  val msg_resp      = msgTable(pos_resp.set)(pos_resp.way)
  val taskInst_resp = Mux(respVal, Mux(cm_resp.intl.w.cmResp,  io.respCmt.bits.inst, msg_resp.inst), 0.U.asTypeOf(new TaskInst))
  val secInst_resp  = Mux(respVal & cm_resp.intl.w.secResp, io.respCmt.bits.inst, 0.U.asTypeOf(new TaskInst))
  val deoccde_resp  = Decode.decode(msg_resp.chi.getChiInst(respVal), msg_resp.dir.getStateInst(msg_resp.chi.metaIdOH, respVal), taskInst_resp, secInst_resp)._2

  // get decode result
  val code_temp = deoccde_resp._2
  val cmt_temp  = deoccde_resp._3
  when(cm_resp.intl.w.cmResp) {
    code_rCmt   := code_temp
    cmt_rCmt    := Mux(cmt_temp.waitSecDone, 0.U.asTypeOf(new CommitCode), cmt_temp)
  }.elsewhen(cm_resp.intl.w.secResp) {
    code_rCmt   := 0.U.asTypeOf(new TaskCode)
    cmt_rCmt    := cmt_temp
  }.otherwise {
    code_rCmt   := 0.U.asTypeOf(new TaskCode)
    cmt_rCmt    := 0.U.asTypeOf(new CommitCode)
  }
  // HardwareAssertion
  when(respVal) {
    HardwareAssertion.withEn(code_rCmt.valid, cmt_temp.waitSecDone)
    HardwareAssertion.withEn(code_rCmt.asUInt === 0.U, !code_rCmt.valid)
    HardwareAssertion.withEn(cmt_rCmt.asUInt === 0.U, !cmt_rCmt.valid)
  }
  // cm internal send
  cm_rCmt.intl.s.data0    := cmt_rCmt.dataOp.reqs | cmt_rCmt.dataOp.read | cmt_rCmt.dataOp.send
  cm_rCmt.intl.s.data1    := cmt_rCmt.dataOp.save | cmt_rCmt.dataOp.clean
  cm_rCmt.intl.s.wriDir   := cmt_rCmt.wriDir
  cm_rCmt.intl.s.canNest  := code_rCmt.canNest
  cm_rCmt.intl.s.secTask  := code_rCmt.valid
  // cm internal wait
  cm_rCmt.intl.w.cmResp   := false.B
  cm_rCmt.intl.w.secResp  := code_rCmt.valid
  cm_rCmt.intl.w.repl     := cm_rCmt.intl.s.wriDir
  cm_rCmt.intl.w.data0    := cm_rCmt.intl.s.data0
  cm_rCmt.intl.w.data1    := cm_rCmt.intl.s.data1
  // cm chi send
  cm_rCmt.chi.s_resp      := cmt_rCmt.commit & cmt_rCmt.channel === ChiChannel.RSP
  // cm chi wait
  cm_rCmt.chi.w_ack       := cm_resp.chi.w_ack
  // valid
  cm_rCmt.clean           := false.B
  cm_rCmt.valid           := true.B

  /*
   * [DataTask] Priority: data > data0 > data1
   */
  // data (reqs, read, send, save, clean)
  cmVec2_d.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.data0 & (b.intl.s.data1 & !b.intl.w.repl) } }
  cmPoS_d.set   := RREncoder(cmVec2_d.map(_.reduce(_ | _)))
  cmPoS_d.way   := RREncoder(cmVec2_d(cmPoS_d.set))
  val valid_d   = cmVec2_d.asUInt.orR

  // data0 (reqs, read, send)
  cmVec2_d0.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.data0 } }
  cmPoS_d0.set  := RREncoder(cmVec2_d0.map(_.reduce(_ | _)))
  cmPoS_d0.way  := RREncoder(cmVec2_d0(cmPoS_d0.set))
  val valid_d0  = cmVec2_d0.asUInt.orR

  // data1 (save, clean)
  cmVec2_d1.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.data1 & !b.intl.w.repl } }
  cmPoS_d1.set  := RREncoder(cmVec2_d1.map(_.reduce(_ | _)))
  cmPoS_d1.way  := RREncoder(cmVec2_d1(cmPoS_d1.set))
  val valid_d1  = cmVec2_d1.asUInt.orR

  // dontTouch
  dontTouch(valid_d)
  dontTouch(valid_d0)
  dontTouch(valid_d1)

  // data task
  val cmPos_dt  = PriorityMux(Seq(
    valid_d     -> cmPoS_d,
    valid_d0    -> cmPoS_d0,
    valid_d1    -> cmPoS_d1,
  ))
  val msg_dt    = msgTable(cmPos_dt.set)(cmPos_dt.way)

  // io.dataTask
  io.dataTask.valid := valid_d | valid_d0 | valid_d1
  io.dataTask.bits  := DontCare
  // dataOp
  when(valid_d) {
    io.dataTask.bits.dataOp       := msg_dt.commit.dataOp
  }.elsewhen(valid_d0) {
    io.dataTask.bits.dataOp       := 0.U.asTypeOf(new DataOp)
    io.dataTask.bits.dataOp.reqs  := msg_dt.commit.dataOp.reqs
    io.dataTask.bits.dataOp.read  := msg_dt.commit.dataOp.read
    io.dataTask.bits.dataOp.send  := msg_dt.commit.dataOp.send
  }.otherwise {
    io.dataTask.bits.dataOp       := 0.U.asTypeOf(new DataOp)
    io.dataTask.bits.dataOp.save  := msg_dt.commit.dataOp.save
    io.dataTask.bits.dataOp.clean := msg_dt.commit.dataOp.clean
  }
  when(io.dataTask.valid) {
    HardwareAssertion(PopCount(io.dataTask.bits.dataOp.asUInt) =/= 0.U)
    HardwareAssertion.withEn(!io.dataTask.bits.dataOp.reqs, msg_dt.alr.reqs)
  }
  // txDat
  // TODO: SnpRespData
  io.dataTask.bits.txDat.DBID       := cmPos_dt.getLLCTxnID(dirBank)
  io.dataTask.bits.txDat.Resp       := msg_dt.commit.resp
  io.dataTask.bits.txDat.Opcode     := msg_dt.commit.commitOp
  io.dataTask.bits.txDat.HomeNID    := DontCare // remap in SAM
  io.dataTask.bits.txDat.TxnID      := msg_dt.chi.txnID
  io.dataTask.bits.txDat.SrcID      := msg_dt.chi.getNoC(io.config.ci)
  io.dataTask.bits.txDat.TgtID      := msg_dt.chi.nodeId
  // other bits
  io.dataTask.bits.dataVec          := msg_dt.chi.dataVec
  io.dataTask.bits.ds               := msg_dt.ds
  io.dataTask.bits.llcTxnID.dirBank := dirBank.U
  io.dataTask.bits.llcTxnID.pos     := cmPos_dt



  /*
   * [wriDir]
   */
  cmVec2_wDir.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.wriDir & !b.intl.w.data0 } }
  cmPoS_wDir.set      := RREncoder(cmVec2_wDir.map(_.reduce(_ | _)))
  cmPoS_wDir.way      := RREncoder(cmVec2_wDir(cmPoS_wDir.set))
  val msg_wDir        = msgTable(cmPoS_wDir.set)(cmPoS_wDir.way)
  // valid
  io.replAlloc.valid  := cmVec2_wDir.asUInt.orR
  // bits
  io.replAlloc.bits.alr               := msg_wDir.alr
  io.replAlloc.bits.llcTxnID.dirBank  := dirBank.U
  io.replAlloc.bits.llcTxnID.pos      := cmPoS_wDir
  // sf
  io.replAlloc.bits.wriSF             := msg_wDir.commit.wriSF
  io.replAlloc.bits.dir.sf.hit        := msg_wDir.dir.sf.hit
  io.replAlloc.bits.dir.sf.wayOH      := msg_wDir.dir.sf.wayOH
  io.replAlloc.bits.dir.sf.metaVec.map(_.state).zipWithIndex.foreach {
    case(s, i) =>
      val metaIdOH = msg_wDir.chi.metaIdOH
      val snpVec   = msg_wDir.dir.getSnpVec(msg_wDir.snpTgt, metaIdOH)
      s := PriorityMux(Seq(
        metaIdOH(i.U) -> msg_wDir.commit.srcValid,
        snpVec(i)     -> msg_wDir.commit.snpValid,
        true.B        -> msg_wDir.dir.sf.metaVec(i).state
      ))
  }
  // llc
  io.replAlloc.bits.wriLLC            := msg_wDir.commit.wriLLC
  io.replAlloc.bits.dir.llc.hit       := msg_wDir.dir.llc.hit
  io.replAlloc.bits.dir.llc.wayOH     := msg_wDir.dir.llc.wayOH
  io.replAlloc.bits.dir.llc.metaVec.head.state := msg_wDir.commit.llcState


  /*
   * [canNest]
   * TODO: cant nest when get cmResp
   */
  cmVec2_nest.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.canNest } }
  cmPoS_nest.set  := RREncoder(cmVec2_nest.map(_.reduce(_ | _)))
  cmPoS_nest.way  := RREncoder(cmVec2_nest(cmPoS_nest.set))
  io.updPosNest.valid         := cmVec2_nest.asUInt.orR
  io.updPosNest.bits.pos      := cmPoS_nest
  io.updPosNest.bits.canNest  := true.B

  /*
   * [secTask]
   */
  cmVec2_task.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.secTask } }
  cmPoS_task.set  := RREncoder(cmVec2_task.map(_.reduce(_ | _)))
  cmPoS_task.way  := RREncoder(cmVec2_task(cmPoS_task.set))
  val msg_task    = msgTable(cmPoS_task.set)(cmPoS_task.way)
  val valid_task  = cmVec2_task.asUInt.orR & !io.replAlloc.valid
  val fire_task   = io.cmAllocVec.map(_.fire).reduce(_ | _)
  // chi
  alloc_task.chi                  := msg_task.chi
  alloc_task.chi.channel          := Mux(msg_task.code.snoop, ChiChannel.SNP, ChiChannel.REQ)
  alloc_task.chi.opcode           := msg_task.code.opcode
  alloc_task.chi.expCompAck       := msg_task.code.expCompAck
  alloc_task.chi.retToSrc         := msg_task.code.retToSrc
  // other
  alloc_task.dataOp               := msg_task.code.dataOp
  alloc_task.llcTxnID.pos         := cmPoS_task
  alloc_task.llcTxnID.dirBank     := dirBank.U
  alloc_task.alr                  := msg_task.alr
  alloc_task.snpVec               := msg_task.dir.getSnpVec(msg_task.code.snpTgt, msg_task.chi.metaIdOH)
  alloc_task.ds                   := msg_task.ds
  alloc_task.fromRepl             := false.B
  alloc_task.cbResp               := msg_task.dir.llc.metaVec.head.cbResp
  alloc_task.doDMT                := msg_task.code.doDMT
  // alloc
  io.cmAllocVec(CMID.SNP).valid   := valid_task & msg_task.code.snoop
  io.cmAllocVec(CMID.READ).valid  := valid_task & msg_task.code.read
  io.cmAllocVec(CMID.DL).valid    := valid_task & msg_task.code.dataless
  io.cmAllocVec(CMID.WOA).valid   := valid_task & msg_task.code.wriOrAtm
  io.cmAllocVec(CMID.REC).valid   := valid_task & msg_task.code.receive
  io.cmAllocVec.foreach(_.bits    := alloc_task)
  HardwareAssertion(PopCount(io.cmAllocVec.map(_.fire)) <= 1.U)

  /*
   * [cleanPoS]/[unLock]
   */
  cmVec2_clean.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.valid & b.clean } }
  cmPoS_clean.set   := RREncoder(cmVec2_clean.map(_.reduce(_ | _)))
  cmPoS_clean.way   := RREncoder(cmVec2_clean(cmPoS_clean.set))
  // valid
  io.cleanPos.valid         := cmVec2_clean.asUInt.orR
  io.unlock.valid           := cmVec2_clean.asUInt.orR
  // bits
  io.cleanPos.bits.pos      := cmPoS_clean
  io.cleanPos.bits.channel  := msgTable(cmPoS_clean.set)(cmPoS_clean.way).chi.channel
  io.unlock.bits            := cmPoS_clean

  /*
   * [s_resp]
   */
  cmVec2_sRsp.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.chi.s_resp } }
  cmPoS_sRsp.set  := RREncoder(cmVec2_sRsp.map(_.reduce(_ | _)))
  cmPoS_sRsp.way  := RREncoder(cmVec2_sRsp(cmPoS_sRsp.set))
  val msg_sRsp    = msgTable(cmPoS_sRsp.set)(cmPoS_sRsp.way)
  // valid
  io.txRsp.valid  := cmVec2_sRsp.asUInt.orR
  // bits
  io.txRsp.bits           := DontCare
  io.txRsp.bits.DBID      := cmPoS_sRsp.getLLCTxnID(dirBank)
  io.txRsp.bits.FwdState  := msg_sRsp.commit.fwdResp
  io.txRsp.bits.Resp      := msg_sRsp.commit.resp
  io.txRsp.bits.Opcode    := msg_sRsp.commit.commitOp
  io.txRsp.bits.TxnID     := msg_sRsp.chi.txnID
  io.txRsp.bits.SrcID     := msg_sRsp.chi.getNoC(io.config.ci)
  io.txRsp.bits.TgtID     := msg_sRsp.chi.nodeId


  /*
   * Modify Ctrl Machine Table
   */
  cmTable.zip(msgTable).zipWithIndex.foreach {
    case((cmVec, msgVec), i) =>
      cmVec.zip(msgVec).zipWithIndex.foreach {
        case((cm, msg), j) =>
          val allocHit    = io.alloc.fire     & io.alloc.bits.pos.idxMatch(i, j)
          val taskRespHit = io.respCmt.fire   & io.respCmt.bits.llcTxnID.pos.idxMatch(i, j) & !msg.commit.valid
          val waitReplHit = io.replResp.valid & io.replResp.bits.pos.idxMatch(i, j)

          // Message
          // Alloc
          when(allocHit) {
            msg := msg_rec
          // Get Resp From TaskCM
          }.elsewhen(taskRespHit) {
            msg.alr.reqs    := msg.alr.reqs | respCmt.alr.reqs
            msg.inst        := respCmt.inst
            msg.code        := code_rCmt
            msg.commit      := cmt_rCmt
          }.elsewhen(waitReplHit){
            msg.commit.dataOp.save  := msg.commit.dataOp.save  | io.replResp.bits.alr.sRepl
            msg.commit.dataOp.clean := msg.commit.dataOp.clean | io.replResp.bits.alr.reqs
            HAssert.withEn(io.replResp.bits.alr.reqs, io.replResp.bits.alr.sRepl)
          }.elsewhen(cm.clean) {
            msg := 0.U.asTypeOf(msg)
          }

          // llcTxnID
          val llcTxnID      = Wire(new LLCTxnID())
          llcTxnID.dirBank  := dirBank.U
          llcTxnID.pos.set  := i.U
          llcTxnID.pos.way  := j.U

          // hit
          val dataTaskHit     = io.dataTask.fire   & cmPos_dt.idxMatch(i, j)
          val wDirHit         = io.replAlloc.fire  & cmPoS_wDir.idxMatch(i, j)
          val canNestHit      = io.updPosNest.fire & cmPoS_nest.idxMatch(i, j)
          val secTaskHit      = fire_task          & cmPoS_task.idxMatch(i, j)
          val waitDataAckHit  = io.dataResp.valid  & io.dataResp.bits.pos.idxMatch(i, j)
          val respChiHit      = io.txRsp.fire      & cmPoS_sRsp.idxMatch(i, j)
          val waitCompAckHit  = io.rxRsp.fire      & io.rxRsp.bits.Opcode === CompAck & io.rxRsp.bits.TxnID === llcTxnID.get

          // need save or clean
          val replNeedSData1  = waitReplHit        & (io.replResp.bits.alr.reqs | io.replResp.bits.alr.sRepl)

          // transfer state
          when(allocHit) {
            cm  := cm_rec
          }.elsewhen(taskRespHit) {
            cm  := cm_rCmt
            // It is also possible to receive an CompAck when receiving a task resp.
            cm.chi.w_ack        :=  cm.chi.w_ack       & !(waitCompAckHit | msg.chi.reqIs(WriteEvictOrEvict)) // WriteEvictOrEvict Get CompAck in ReceiveCM
          }.otherwise {
            // internal send
            cm.intl.s.data0     :=  cm.intl.s.data0    & !(dataTaskHit & (io.dataTask.bits.dataOp.reqs | io.dataTask.bits.dataOp.read | io.dataTask.bits.dataOp.send))
            cm.intl.s.data1     := (cm.intl.s.data1    & !(dataTaskHit & (io.dataTask.bits.dataOp.save | io.dataTask.bits.dataOp.clean))) | replNeedSData1
            cm.intl.s.wriDir    :=  cm.intl.s.wriDir   & !wDirHit
            cm.intl.s.canNest   :=  cm.intl.s.canNest  & !canNestHit
            cm.intl.s.secTask   :=  cm.intl.s.secTask  & !secTaskHit
            // internal wait
            cm.intl.w.cmResp    :=  cm.intl.w.cmResp // will be change in taskRespHit
            cm.intl.w.secResp   :=  cm.intl.w.secResp  & !taskRespHit
            cm.intl.w.repl      :=  cm.intl.w.repl     & !waitReplHit
            cm.intl.w.data0     :=  cm.intl.w.data0    & !(waitDataAckHit & !cm.intl.s.data0)
            cm.intl.w.data1     := (cm.intl.w.data1    & !(waitDataAckHit & !cm.intl.s.data1 & !cm.intl.w.repl)) | replNeedSData1
            // chi send
            cm.chi.s_resp       :=  cm.chi.s_resp      & !respChiHit
            // chi wait
            cm.chi.w_ack        :=  cm.chi.w_ack       & !(waitCompAckHit | msg.chi.reqIs(WriteEvictOrEvict)) // WriteEvictOrEvict Get CompAck in ReceiveCM
            // clean and valid
            cm.clean  := !(cm.intl.s.asUInt | cm.intl.w.asUInt | cm.chi.s_resp | cm.chi.w_ack).orR
            cm.valid  := cm.valid & !(cm.clean & cmPoS_clean.idxMatch(i, j))

            // HWA
            HardwareAssertion.withEn(!cm.valid, allocHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.w.cmResp | cm.intl.w.secResp, taskRespHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.s.data0  | cm.intl.s.data1,   dataTaskHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.w.data0  | cm.intl.w.data1 | cm.intl.w.repl, waitDataAckHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.s.wriDir,  wDirHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.s.canNest, canNestHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.s.secTask, secTaskHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.w.repl,    waitReplHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.chi.s_resp,     respChiHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.chi.w_ack | msg.chi.reqIs(WriteEvictOrEvict),  waitCompAckHit, cf"Commit Index[${i}][${j}]")
            // TIMEOUT
            val timeout = HardwareAssertion.checkTimeout(!cm.valid, TIMEOUT_COMMIT, cf"TIMEOUT: Commit Index[${i}][${j}]" +
              cf"\nInternal Send: ${cm.intl.s}\nInternal Wait: ${cm.intl.w}\nCHI: ${cm.chi}\n${msg.chi.getChiInst()}\n${msg.dir.getStateInst(msg.chi.metaIdOH)}")
          }
      }
  }


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}