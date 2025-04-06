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
import dongjiang.data.HasAlrDB
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
      val reqDB     = Bool() // Req DB
      val cleanDB   = Bool() // Clean Data Buffer (will be ctrl by waitChiDat)
      val dataTask  = Bool() // Send Data Task (will be ctrl by waitChiDat)
      // val fwdTask   = Bool() // Send CompData in SnpFwd
      val wriDir    = Bool() // Send Write Task To Replace
      val canNest   = Bool() // Indicate PoS This Entry Can Be Nested
      val secTask   = Bool() // Send Task To TaskCM
    }
    // internal wait
    val w = new DJBundle {
      val cmResp    = Bool() // Wait TaskCM Resp
      val secResp   = Bool() // Wait Second TaskCM Resp
      val waitRepl  = Bool() // Wait Replace Done
      val waitData  = Bool() // Wait Data Done
    }
  }
  val chi = new DJBundle {
    // chi send
    val s = new DJBundle {
      val respChi2Rn  = Bool() // Send Resp To RN Or Data
      val respChi2Hn  = Bool() // Send Resp To HN Or Data
      val dbidChi2Rn  = Bool() // Send XDBIDResp To RN (will be ctrl by reqDB)
      val compChi2Rn  = Bool() // Send Comp To RN (will be ctrl by waitChiComp)
    }
    // chi wait
    val w = new DJBundle {
      val waitChiAck  = Bool() // Wait CompAck From RN
      val waitChiDat0 = Bool() // Wait Data Beat 0
      val waitChiDat1 = Bool() // Wait Data Beat 1
    }
  }
  // other
  val clean = Bool() // Clean PoS Tag and Dir Lock
  val valid = Bool() // Valid
}

class CommitTask(implicit p: Parameters) extends DJBundle with HasPackChi with HasPackPosIndex
  with HasPackDirMsg with HasAlrDB with HasPackOperations with HasPackCmtCode with HasDsIdx with HasSnpTgt

// TODO: no need Addr
class CMTask(implicit p: Parameters) extends DJBundle with HasPackChi with HasAddr with HasPackLLCTxnID
  with HasAlrDB with HasDsIdx {
  val needDB    = Bool()
  val snpVec    = Vec(nrSfMetas, Bool())
  val fromRepl  = Bool() // from ReplaceCM
  val cbResp    = UInt(ChiResp.width.W) // CopyBack Resp(only use in WriOrAtmCM)
  def doDMT     = !needDB
}

class PackCMTask(implicit p: Parameters) extends DJBundle {
  val task = new CMTask()
}

class RespToCmt(implicit p: Parameters) extends DJBundle with HasPackLLCTxnID with HasPackTaskInst with HasAlrDB

class ReplTask(implicit p: Parameters) extends DJBundle with HasAlrDB with HasAddr with HasPackDirMsg with HasPackLLCTxnID {
  val wriSF         = Bool()
  val wriLLC        = Bool()
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
    val rxDat       = Flipped(Valid(new DataFlit())) // Dont use rxDat.Data/BE in Backend
    // Get Full Addr In PoS
    val getPosAddr  = Output(new PosIndex())
    val posRespAddr = Input(new Addr())
    // Send Task To CM
    val cmAllocVec  = Vec(nrTaskCM, Decoupled(new CMTask))
    // Resp From TaskCM
    val respCmt     = Flipped(Valid(new RespToCmt))
    // Send Task To Replace
    val replAlloc   = Decoupled(new ReplTask)
    val replResp    = Flipped(Valid(new PackPosIndex()))
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
  val msgTable      = Reg(Vec(posSets, Vec(posWays, new PackChi with HasPackDirMsg with HasAlrDB with HasPackCmtCode
                                                      with HasDsIdx with HasPackTaskInst with HasPackTaskCode with HasSnpTgt)))
  // receive
  val cm_rec        = Wire(new CMState())
  val msg_rec       = WireInit(0.U.asTypeOf(msgTable.head.head))
  // respCmt
  val code_rCmt     = Wire(new TaskCode())
  val cmt_rCmt      = Wire(new CommitCode())
  val cm_rCmt       = Wire(new CMState())
  // reqDB
  val cmVec2_rDB    = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_rDB     = Wire(new PosIndex())
  // cleanDB
  val cmVec2_cDB    = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_cDB     = Wire(new PosIndex())
  // send data task
  val cmVec2_sdt    = Wire(Vec(posSets, Vec(posWays, Bool())))
  val cmPoS_sdt     = Wire(new PosIndex())
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

  /*
   * [rec] Receive Task IO
   */
  val alloc_rec       = io.alloc.bits
  val wriOrEvtHit_rec = alloc_rec.chi.isReq(WriteEvictOrEvict) & (alloc_rec.dir.sf.hit | alloc_rec.dir.llc.hit)
  val owoCanComp_rec  = alloc_rec.chi.isOWO & alloc_rec.chi.memAttr.ewa & !alloc_rec.ops.snoop
  val needSdbid_rec   = alloc_rec.chi.needSendDBID(alloc_rec.dir.sf.hit) & !alloc_rec.alrDB.reqs
  // msg
  msg_rec.chi           := alloc_rec.chi
  msg_rec.dir           := alloc_rec.dir
  msg_rec.alrDB         := alloc_rec.alrDB
  msg_rec.commit        := alloc_rec.commit
  msg_rec.ds            := alloc_rec.ds
  msg_rec.snpTgt        := alloc_rec.snpTgt
  msg_rec.code.snoop    := alloc_rec.ops.snoop
  msg_rec.code.read     := alloc_rec.ops.read
  msg_rec.code.dataless := alloc_rec.ops.dataless
  msg_rec.code.wriOrAtm := alloc_rec.ops.wriOrAtm
  // cm internal send
  cm_rec.intl.s.reqDB       := needSdbid_rec
  cm_rec.intl.s.cleanDB     := cm_rec.intl.s.dataTask
  cm_rec.intl.s.dataTask    := alloc_rec.commit.commit & alloc_rec.commit.channel === ChiChannel.DAT & !alloc_rec.alrDB.fast
  cm_rec.intl.s.wriDir      := alloc_rec.commit.wriSF | alloc_rec.commit.wriLLC
  cm_rec.intl.s.canNest     := false.B
  cm_rec.intl.s.secTask     := false.B
  // cm internal wait
  cm_rec.intl.w.cmResp      := alloc_rec.ops.valid
  cm_rec.intl.w.secResp     := false.B
  cm_rec.intl.w.waitRepl    := cm_rec.intl.s.wriDir
  cm_rec.intl.w.waitData    := alloc_rec.alrDB.fast | cm_rec.intl.s.dataTask
  // cm chi send
  cm_rec.chi.s.respChi2Rn   := alloc_rec.chi.isReq & alloc_rec.commit.commit & alloc_rec.commit.channel === ChiChannel.RSP
  cm_rec.chi.s.respChi2Hn   := alloc_rec.chi.isSnp & alloc_rec.commit.commit & alloc_rec.commit.channel === ChiChannel.RSP
  cm_rec.chi.s.dbidChi2Rn   := needSdbid_rec
  cm_rec.chi.s.compChi2Rn   := wriOrEvtHit_rec | owoCanComp_rec | (cm_rec.chi.s.dbidChi2Rn & (alloc_rec.chi.isCopyBackWrite | alloc_rec.chi.isAtomic))
  // cm chi wait
  cm_rec.chi.w.waitChiAck   := alloc_rec.chi.expCompAck
  cm_rec.chi.w.waitChiDat0  := alloc_rec.chi.dataVec(0) & alloc_rec.chi.isWrite
  cm_rec.chi.w.waitChiDat1  := alloc_rec.chi.dataVec(1) & alloc_rec.chi.isWrite
  // valid
  cm_rec.clean := false.B
  cm_rec.valid := true.B

  /*
   * [RESP] Receive Resp From TaskCM
   */
  val respCmt       = io.respCmt.bits
  val pos_resp      = respCmt.llcTxnID.pos
  val cm_resp       = cmTable(pos_resp.set)(pos_resp.way)
  val msg_resp      = msgTable(pos_resp.set)(pos_resp.way)
  val taskInst_resp = Mux(cm_resp.intl.w.cmResp,  io.respCmt.bits.inst, msg_resp.inst)
  val secInst_resp  = Mux(cm_resp.intl.w.secResp, io.respCmt.bits.inst, 0.U.asTypeOf(new TaskInst))
  val deoccde_resp  = Decode.decode(msg_resp.chi.getChiInst, msg_resp.dir.getStateInst(msg_resp.chi.metaId), taskInst_resp, secInst_resp)._4
  // get decode result
  code_rCmt         := deoccde_resp._2
  cmt_rCmt          := Mux(deoccde_resp._3.waitSecDone & cm_resp.intl.w.cmResp, 0.U.asTypeOf(new CommitCode), deoccde_resp._3)
  dontTouch(code_rCmt)
  dontTouch(cmt_rCmt)
  // HardwareAssertion
  HardwareAssertion.withEn(deoccde_resp._2.flag, io.respCmt.valid, cf"Task inst invalid in Commit[${pos_resp.set}][${pos_resp.way}]" +
    cf"\n${msg_resp.chi.getChiInst}\n${msg_resp.dir.getStateInst(msg_resp.chi.metaId)}\n${taskInst_resp}\nSec${secInst_resp}")
  HardwareAssertion.withEn(deoccde_resp._3.flag,  io.respCmt.valid, cf"Task inst invalid in Commit[${pos_resp.set}][${pos_resp.way}]" +
    cf"\n${msg_resp.chi.getChiInst}\n${msg_resp.dir.getStateInst(msg_resp.chi.metaId)}\n${taskInst_resp}\nSec${secInst_resp}")
  // cm internal send
  cm_rCmt.intl.s.reqDB      := code_rCmt.needDB & !msg_resp.alrDB.reqs
  cm_rCmt.intl.s.cleanDB    := cm_rCmt.intl.s.dataTask
  cm_rCmt.intl.s.dataTask   := cmt_rCmt.commit & cmt_rCmt.channel === ChiChannel.DAT
  cm_rCmt.intl.s.wriDir     := cmt_rCmt.wriSF | cmt_rCmt.wriLLC
  cm_rCmt.intl.s.canNest    := code_rCmt.canNest
  cm_rCmt.intl.s.secTask    := code_rCmt.valid
  // cm internal wait
  cm_rCmt.intl.w.cmResp     := false.B
  cm_rCmt.intl.w.secResp    := !code_rCmt.invalid
  cm_rCmt.intl.w.waitRepl   := cm_rCmt.intl.s.wriDir
  cm_rCmt.intl.w.waitData   := cm_rCmt.intl.s.dataTask
  // cm chi send
  cm_rCmt.chi.s.respChi2Rn  := msg_resp.chi.isReq & cmt_rCmt.commit & cmt_rCmt.channel === ChiChannel.RSP
  cm_rCmt.chi.s.respChi2Hn  := msg_resp.chi.isSnp & cmt_rCmt.commit & cmt_rCmt.channel === ChiChannel.RSP
  cm_rCmt.chi.s.dbidChi2Rn  := false.B
  cm_rCmt.chi.s.compChi2Rn  := false.B // TODO
  // cm chi wait
  cm_rCmt.chi.w.waitChiAck  := msg_resp.chi.expCompAck
  cm_rCmt.chi.w.waitChiDat0 := false.B
  cm_rCmt.chi.w.waitChiDat1 := false.B
  // valid
  cm_rCmt.clean := false.B
  cm_rCmt.valid := true.B
  HardwareAssertion.withEn(!msg_resp.alrDB.reqs, io.respCmt.valid & respCmt.alrDB.reqs, cf"Commit Index[${pos_resp.set}][${pos_resp.way}]")



  /*
   * [reqDB] / [cleanDB] / [DataTask]
   */
  // [reqDB]
  cmVec2_rDB.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.reqDB } }
  cmPoS_rDB.set     := RREncoder(cmVec2_rDB.map(_.reduce(_ | _)))
  cmPoS_rDB.way     := RREncoder(cmVec2_rDB(cmPoS_rDB.set))
  val valid_rDB     = cmVec2_rDB.asUInt.orR
  // [cleanDB]
  cmVec2_cDB.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.cleanDB } }
  cmPoS_cDB.set     := RREncoder(cmVec2_cDB.map(_.reduce(_ | _)))
  cmPoS_cDB.way     := RREncoder(cmVec2_cDB(cmPoS_cDB.set))
  val valid_cDB     = cmVec2_cDB.asUInt.orR
  // [DataTask]
  cmVec2_sdt.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.dataTask } }
  cmPoS_sdt.set     := RREncoder(cmVec2_sdt.map(_.reduce(_ | _)))
  cmPoS_sdt.way     := RREncoder(cmVec2_sdt(cmPoS_sdt.set))
  val valid_sdt     = cmVec2_sdt.asUInt.orR

  // io.dataTask
  // clean > dataTask > req
  // TODO: fwdTask
  val cmPos_dt      = Mux(valid_cDB, cmPoS_cDB, Mux(valid_sdt, cmPoS_sdt, cmPoS_rDB))
  val cm_dt         = cmTable(cmPos_dt.set)(cmPos_dt.way)
  val msg_dt        = msgTable(cmPos_dt.set)(cmPos_dt.way)
  io.dataTask.valid := valid_rDB | valid_cDB | valid_sdt
  io.dataTask.bits  := DontCare
  when(valid_cDB) {
    io.dataTask.bits.dataOp.clean := true.B
  }.elsewhen(valid_sdt) {
    io.dataTask.bits.dataOp.reqs  := cm_dt.intl.s.reqDB
    io.dataTask.bits.dataOp.clean := true.B
    io.dataTask.bits.dataOp.read  := cm_dt.intl.s.reqDB
    io.dataTask.bits.dataOp.send  := true.B
  }.otherwise {
    io.dataTask.bits.dataOp.reqs  := true.B
  }
  // TODO: SnpRespData
  io.dataTask.bits.llcTxnID.dirBank := dirBank.U
  io.dataTask.bits.llcTxnID.pos     := cmPos_dt
  io.dataTask.bits.ds               := msg_dt.ds
  io.dataTask.bits.dataVec          := msg_dt.chi.dataVec
  io.dataTask.bits.txDat.DBID       := cmPos_dt.getLLCTxnID(dirBank)
  io.dataTask.bits.txDat.Resp       := msg_dt.commit.resp
  io.dataTask.bits.txDat.Opcode     := msg_dt.commit.commitOp
  io.dataTask.bits.txDat.HomeNID    := DontCare // remap in SAM
  io.dataTask.bits.txDat.TxnID      := msg_dt.chi.txnID
  io.dataTask.bits.txDat.SrcID      := msg_dt.chi.getNoC(io.config.ci)
  io.dataTask.bits.txDat.TgtID      := msg_dt.chi.nodeId

  /*
   * [wriDir]
   */
  cmVec2_wDir.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.intl.s.wriDir } }
  cmPoS_wDir.set  := RREncoder(cmVec2_wDir.map(_.reduce(_ | _)))
  cmPoS_wDir.way  := RREncoder(cmVec2_wDir(cmPoS_wDir.set))
  val msg_wDir    = msgTable(cmPoS_wDir.set)(cmPoS_wDir.way)
  // valid
  io.replAlloc.valid  := cmVec2_wDir.asUInt.orR
  // bits
  io.replAlloc.bits.alrDB     := msg_wDir.alrDB
  io.replAlloc.bits.addr      := io.posRespAddr.addr
  io.replAlloc.bits.llcTxnID.dirBank  := dirBank.U
  io.replAlloc.bits.llcTxnID.pos      := cmPoS_wDir
  // sf
  io.replAlloc.bits.wriSF         := msg_wDir.commit.wriSF
  io.replAlloc.bits.dir.sf.hit    := msg_wDir.dir.sf.hit
  io.replAlloc.bits.dir.sf.wayOH  := msg_wDir.dir.sf.wayOH
  io.replAlloc.bits.dir.sf.metaVec.map(_.state).zipWithIndex.foreach {
    case(s, i) =>
      val metaId = msg_wDir.chi.metaId
      val snpVec = msg_wDir.dir.getSnpVec(msg_wDir.snpTgt, metaId)
      s := PriorityMux(Seq(
        (metaId === i.U) -> msg_wDir.commit.srcValid,
        snpVec(i)        -> msg_wDir.commit.snpValid,
        true.B           -> msg_wDir.dir.sf.metaVec(i).state
      ))
  }
  // llc
  io.replAlloc.bits.wriLLC        := msg_wDir.commit.wriLLC
  io.replAlloc.bits.dir.llc.hit   := msg_wDir.dir.llc.hit
  io.replAlloc.bits.dir.llc.wayOH := msg_wDir.dir.llc.wayOH
  io.replAlloc.bits.dir.llc.metaVec.head.state := msg_wDir.commit.llcState


  /*
   * [canNest]
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
  alloc_task.chi            := msg_task.chi
  alloc_task.chi.channel    := Mux(msg_task.code.snoop, ChiChannel.SNP, ChiChannel.REQ)
  alloc_task.chi.opcode     := msg_task.code.opcode
  alloc_task.chi.expCompAck := msg_task.code.expCompAck
  alloc_task.chi.retToSrc   := msg_task.code.retToSrc
  alloc_task.needDB         := msg_task.code.needDB
  // other
  alloc_task.addr             := io.posRespAddr.addr
  alloc_task.llcTxnID.pos     := cmPoS_task
  alloc_task.llcTxnID.dirBank := dirBank.U
  alloc_task.alrDB            := msg_task.alrDB
  alloc_task.ds.bank          := DontCare
  alloc_task.ds.idx           := DontCare
  alloc_task.fromRepl         := false.B
  alloc_task.cbResp           := DontCare
  alloc_task.snpVec           := msg_task.dir.getSnpVec(msg_task.code.snpTgt, msg_task.chi.metaId)
  // alloc
  io.cmAllocVec(CMID.SNP).valid   := valid_task & msg_task.code.snoop
  io.cmAllocVec(CMID.READ).valid  := valid_task & msg_task.code.read
  io.cmAllocVec(CMID.DL).valid    := valid_task & msg_task.code.dataless
  io.cmAllocVec(CMID.WOA).valid   := valid_task & msg_task.code.wriOrAtm
  io.cmAllocVec.foreach(_.bits    := alloc_task)

  /*
   * [cleanPoS]
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
   * [respChi2Rn] / [respChi2Hn] / [dbidChi2Rn] / [compChi2Rn]
   */
  cmVec2_sRsp.zip(cmTable).foreach { case(a, b) => a.zip(b).foreach { case(a, b) => a := b.chi.s.asUInt.orR } }
  cmPoS_sRsp.set  := RREncoder(cmVec2_sRsp.map(_.reduce(_ | _)))
  cmPoS_sRsp.way  := RREncoder(cmVec2_sRsp(cmPoS_sRsp.set))
  val msg_sRsp    = msgTable(cmPoS_sRsp.set)(cmPoS_sRsp.way)
  val sType       = cmTable(cmPoS_sRsp.set)(cmPoS_sRsp.way).chi.s
  // valid
  io.txRsp.valid  := cmVec2_sRsp.asUInt.orR
  // bits
  io.txRsp.bits           := DontCare
  io.txRsp.bits.DBID      := cmPoS_sRsp.getLLCTxnID(dirBank)
  io.txRsp.bits.FwdState  := msg_sRsp.commit.fwdResp
  io.txRsp.bits.Resp      := msg_sRsp.commit.resp
  io.txRsp.bits.Opcode    := Mux(sType.dbidChi2Rn, Mux(sType.compChi2Rn, CompDBIDResp, DBIDResp), Mux(sType.compChi2Rn, Comp, msg_sRsp.commit.commitOp))
  io.txRsp.bits.TxnID     := msg_sRsp.chi.txnID
  io.txRsp.bits.SrcID     := msg_sRsp.chi.getNoC(io.config.ci)
  io.txRsp.bits.TgtID     := msg_sRsp.chi.nodeId


  /*
   * Get Addr
   */
  io.getPosAddr := Mux(io.replAlloc.valid, cmPoS_wDir, cmPoS_task)


  /*
   * Modify Ctrl Machine Table
   */
  cmTable.zip(msgTable).zipWithIndex.foreach {
    case((cmVec, msgVec), i) =>
      cmVec.zip(msgVec).zipWithIndex.foreach {
        case((cm, msg), j) =>
          val allocHit    = io.alloc.fire   & io.alloc.bits.pos.idxMatch(i, j)
          val taskRespHit = io.respCmt.fire & io.respCmt.bits.llcTxnID.pos.idxMatch(i, j)

          // Message
          // Alloc
          when(allocHit) {
            msg := msg_rec
          // Get Resp From TaskCM
          }.elsewhen(taskRespHit & !msg.commit.valid) {
            msg.alrDB.reqs  := msg.alrDB.reqs | respCmt.alrDB.reqs
            msg.inst        := respCmt.inst
            msg.code        := code_rCmt
            msg.commit      := cmt_rCmt
          }.elsewhen(cm.clean) {
            msg := 0.U.asTypeOf(msg)
          }

          // llcTxnID
          val llcTxnID = Wire(new LLCTxnID())
          llcTxnID.dirBank := dirBank.U
          llcTxnID.pos.set := i.U
          llcTxnID.pos.way := j.U

          // hit
          val dataTaskHit     = io.dataTask.fire & cmPos_dt.idxMatch(i, j)
          val wDirHit         = io.replAlloc.fire & cmPoS_wDir.idxMatch(i, j)
          val canNestHit      = io.updPosNest.fire & cmPoS_nest.idxMatch(i, j)
          val secTaskHit      = fire_task & cmPoS_task.idxMatch(i, j)
          val waitReplHit     = io.replResp.valid & io.replResp.bits.pos.idxMatch(i, j)
          val waitDataAckHit  = io.dataResp.valid & io.dataResp.bits.pos.idxMatch(i, j)
          val respChiHit      = io.txRsp.fire & cmPoS_sRsp.idxMatch(i, j)
          val waitCompAckHit  = io.rxRsp.fire & io.rxRsp.bits.Opcode === CompAck  & io.rxRsp.bits.TxnID === llcTxnID.get
          // TODO
          val waitDat0Hit     = io.rxDat.fire & (io.rxDat.bits.Opcode === CopyBackWriteData | io.rxDat.bits.Opcode === NonCopyBackWriteData) &
                                io.rxDat.bits.TxnID === llcTxnID.get & io.rxDat.bits.DataID === "b00".U
          val waitDat1Hit     = io.rxDat.fire & (io.rxDat.bits.Opcode === CopyBackWriteData | io.rxDat.bits.Opcode === NonCopyBackWriteData) &
                                io.rxDat.bits.TxnID === llcTxnID.get & io.rxDat.bits.DataID === "b10".U

          // transfer state
          when(allocHit) {
            cm  := cm_rec
          }.elsewhen(taskRespHit & !msg.commit.valid) {
            cm  := cm_rCmt
          }.otherwise {
            // internal send
            when(dataTaskHit) {
              cm.intl.s.reqDB     := false.B
              cm.intl.s.cleanDB   := false.B
              cm.intl.s.dataTask  := false.B
            }
            cm.intl.s.wriDir    := cm.intl.s.wriDir   & !wDirHit
            cm.intl.s.canNest   := cm.intl.s.canNest  & !canNestHit
            cm.intl.s.secTask   := cm.intl.s.secTask  & !secTaskHit
            // internal wait
            cm.intl.w.cmResp    := cm.intl.w.cmResp // will be change in taskRespHit
            cm.intl.w.secResp   := cm.intl.w.secResp  & !taskRespHit
            cm.intl.w.waitRepl  := cm.intl.w.waitRepl & !waitReplHit
            cm.intl.w.waitData  := cm.intl.w.waitData & !waitDataAckHit
            // chi send
            when(respChiHit) {
              cm.chi.s.respChi2Rn := false.B
              cm.chi.s.respChi2Hn := false.B
              cm.chi.s.dbidChi2Rn := false.B
              cm.chi.s.compChi2Rn := false.B
            }
            // chi wait
            cm.chi.w.waitChiAck   := cm.chi.w.waitChiAck  & !waitCompAckHit
            cm.chi.w.waitChiDat0  := cm.chi.w.waitChiDat0 & !waitDat0Hit
            cm.chi.w.waitChiDat1  := cm.chi.w.waitChiDat1 & !waitDat1Hit
            // clean and valid
            cm.clean  := !(cm.intl.s.asUInt | cm.intl.w.asUInt | cm.chi.s.asUInt | cm.chi.w.asUInt).orR
            cm.valid  := cm.valid & !(cm.clean & cmPoS_clean.idxMatch(i, j))

            // HWA
            HardwareAssertion.withEn(!cm.valid, allocHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.w.cmResp | cm.intl.w.secResp, taskRespHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.s.reqDB | cm.intl.s.cleanDB | cm.intl.s.dataTask, dataTaskHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.s.wriDir,      wDirHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.s.canNest,     canNestHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.s.secTask,     secTaskHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.intl.w.waitRepl,    waitReplHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.valid,              waitDataAckHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.chi.w.waitChiAck,   waitCompAckHit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.chi.w.waitChiDat0,  waitDat0Hit, cf"Commit Index[${i}][${j}]")
            HardwareAssertion.withEn(cm.chi.w.waitChiDat1,  waitDat1Hit, cf"Commit Index[${i}][${j}]")

            HardwareAssertion.checkTimeout(!cm.valid, TIMEOUT_COMMIT, cf"TIMEOUT: Commit Index[${i}][${j}]")
          }
      }
  }


  /*
   * HardwareAssertion placePipe
   */
  HardwareAssertion.placePipe(Int.MaxValue-2)
}