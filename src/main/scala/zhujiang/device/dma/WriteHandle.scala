package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.amba.axi4._
import org.chipsalliance.cde.config.Parameters
import zhujiang._
import zhujiang.chi._
import zhujiang.axi._
import xs.utils.sram._

object WRState {
  val width        = 3
  val Free         = "b000".U
  val SendWrReq    = "b001".U
  val WaitDBID     = "b010".U
  val SendWrData   = "b011".U
  val WaitComp     = "b100".U
  val SendCompAck  = "b101".U
  val Complete     = "b110".U
}

object AWState {
  val width       = 2
  val Free        = "b00".U
  val ReceiveData = "b01".U
  val Comp        = "b10".U
}

class AWEntry(implicit p: Parameters) extends ZJBundle {
    val addr            = UInt(raw.W)
    val unalign         = Bool()
    val burst           = UInt(BurstMode.width.W)
    val size            = UInt(3.W)
    val awid            = UInt(8.W)
    val state           = UInt(AWState.width.W)
    val nid             = UInt(zjParams.dmaParams.nidBits.W)
}

class WrStateEntry(implicit p: Parameters) extends ZJBundle {
  val areid        = UInt(log2Ceil(zjParams.dmaParams.entrySize).W)
  val dbid         = UInt(12.W)
  val last         = Bool()
  val full         = Bool()
  val separate     = Bool()
  val sendReqOrder = UInt(6.W)
  val state        = UInt(WRState.width.W)
  val tgtID        = UInt(niw.W)
}

class WriteHandle(implicit p: Parameters) extends ZJModule{
  
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits)

  val io = IO(new Bundle {
    // AXI4 Interface
    val axi_aw = Flipped(Decoupled(new AWFlit(axiParams)))
    val axi_w  = Flipped(Decoupled(new WFlit(axiParams)))
    val axi_b  = Decoupled(new BFlit(axiParams))

    // CHI Interface
    val chi_txreq = Decoupled(new ReqFlit)
    val chi_rxrsp = Flipped(Decoupled(new RespFlit))
    val chi_txrsp = Decoupled(new RespFlit)
    val chi_txdat = Decoupled(new DataFlit)
  })

  

  //---------------------------------------------------------------------------------------------------------------------------------//
  //-------------------------------------------------- Reg and Wire Define ----------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  val awEntrys           = RegInit(VecInit.fill(dmaParams.entrySize)(0.U.asTypeOf(new AWEntry)))
  val dataSram           = Module(new SRAMTemplate(gen = UInt(dw.W), set = dmaParams.bufferSize, singlePort = true))
  val wrStateEntrys      = RegInit(VecInit.fill(dmaParams.bufferSize)(0.U.asTypeOf(new WrStateEntry)))
  val maskSram           = Module(new SRAMTemplate(gen = UInt(bew.W), set = dmaParams.bufferSize, singlePort = true))

  val mergeDataReg       = RegInit(0.U(dw.W))
  val mergeMaskReg       = RegInit(0.U(bew.W))
  
  //---------------------------------------------------------------------------------------------------------------------------------//
  //---------------------------------------------------------- Logic ----------------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  val awFreeVec         = awEntrys.map(_.state === AWState.Free)
  val selFreeAWEntry    = PriorityEncoder(awFreeVec)
  val awBusyVec         = awEntrys.map(_.state =/= AWState.Free)
  val awWrDataVec       = awEntrys.map(a => a.state === AWState.ReceiveData && a.nid === 0.U)
  val selWrDataEntry    = PriorityEncoder(awWrDataVec)
  val wrBe              = io.axi_w.bits.strb.asTypeOf(Vec(dw/8, UInt(1.W)))
  val mask              = RegInit(VecInit(Seq.fill(dw/8){0.U(8.W)}))
  val data              = RegInit(0.U(dw.W))
  val strbReg           = RegInit(0.U(bew.W))
  

  /* 
  Sram start Index pointer and End Index pointer
   */

  val startIndex        = RegInit(0.U(log2Ceil(dmaParams.bufferSize).W))
  val endIndex          = RegInit(0.U(log2Ceil(dmaParams.bufferSize).W))
  

  /* 
  judge is ready to send AXI B Response
   */

  val awCompVec         = awEntrys.map(_.state === AWState.Comp)
  val selCompEntry      = PriorityEncoder(awCompVec)
  val wrStateAreidVec   = wrStateEntrys.map(w => w.state =/= WRState.Free && w.areid === selCompEntry)

  val sendAxiBValid     = awCompVec.reduce(_|_) && !wrStateAreidVec.reduce(_|_)

  /* 
  merge Data and Mask logic
   */

  val mergeValid_s1     = RegInit(false.B)
  val judgeSendReq      = RegInit(true.B)
  mergeValid_s1        := io.axi_w.fire

  when(io.axi_w.fire){
    data    := io.axi_w.bits.data
    strbReg := io.axi_w.bits.strb
  }
  

  when(io.axi_w.fire){
    mask.zip(wrBe).foreach{
    case(m, b) =>
      when(b === 1.U){
        m := 255.U
      }.otherwise{
        m := 0.U   }
    }
  }
  val lastWrData        = RegNext(RegNext(io.axi_w.bits.last && io.axi_w.fire))
  val maskValid         = mergeMaskReg(bew - 1) || lastWrData
  
  when(mergeValid_s1 && !mergeMaskReg(bew - 1)){
  mergeDataReg         := (mask.asTypeOf(UInt(dw.W)) & data) | mergeDataReg
  mergeMaskReg         := strbReg | mergeMaskReg
  }.elsewhen(mergeMaskReg(bew - 1) && mergeValid_s1){
  mergeDataReg         := mask.asTypeOf(UInt(dw.W)) & data
  mergeMaskReg         := strbReg
  }.elsewhen(lastWrData && mergeValid_s1){
  mergeDataReg         := mask.asTypeOf(UInt(dw.W)) & data
  mergeMaskReg         := strbReg
  }.elsewhen(!mergeValid_s1 && lastWrData){
    mergeDataReg       := 0.U
    mergeMaskReg       := 0.U
  }
  val mergeValid_s2     = RegNext(mergeValid_s1)

 
  dataSram.io.w.req.valid        := Mux(maskValid & mergeValid_s2, true.B, false.B)
  dataSram.io.w.req.bits.setIdx  := Mux(maskValid & mergeValid_s2, endIndex, 0.U)
  dataSram.io.w.req.bits.data(0) := Mux(maskValid & mergeValid_s2, mergeDataReg, 0.U)

  maskSram.io.w.req.valid        := Mux(maskValid & mergeValid_s2, true.B, false.B)
  maskSram.io.w.req.bits.setIdx  := Mux(maskValid & mergeValid_s2, endIndex, 0.U)
  maskSram.io.w.req.bits.data(0) := Mux(maskValid & mergeValid_s2, mergeMaskReg, 0.U)


  val sendReqNum                  = wrStateEntrys.map(w => w.areid === selWrDataEntry && (w.state === WRState.SendWrReq || w.state === WRState.WaitDBID && (w.full && !w.last || !w.full)))

  when(mergeMaskReg(bew - 1) && !lastWrData && judgeSendReq && mergeValid_s2){ 
    wrStateEntrys(endIndex).areid := selWrDataEntry
    wrStateEntrys(endIndex).state := WRState.SendWrReq
    wrStateEntrys(endIndex).full  := Mux(awEntrys(selWrDataEntry).unalign, false.B, true.B)
    wrStateEntrys(endIndex).last  := Mux(awEntrys(selWrDataEntry).unalign, true.B, false.B)
    wrStateEntrys(endIndex).sendReqOrder := PopCount(sendReqNum) - (io.chi_rxrsp.fire && 
    (io.chi_rxrsp.bits.Opcode === RspOpcode.CompDBIDResp || io.chi_rxrsp.bits.Opcode === RspOpcode.DBIDResp)).asUInt

    endIndex := endIndex + 1.U
    judgeSendReq := Mux(awEntrys(selWrDataEntry).unalign, true.B, false.B)

  }.elsewhen(mergeMaskReg(bew - 1) && !lastWrData && !judgeSendReq && mergeValid_s2){
    wrStateEntrys(endIndex).areid := selWrDataEntry
    wrStateEntrys(endIndex).state := Mux(awEntrys(selWrDataEntry).unalign, WRState.SendWrReq, WRState.WaitDBID)
    wrStateEntrys(endIndex).full  := Mux(awEntrys(selWrDataEntry).unalign, false.B, true.B)
    wrStateEntrys(endIndex).last  := true.B
    wrStateEntrys(endIndex).sendReqOrder := PopCount(sendReqNum) - (io.chi_rxrsp.fire && 
    (io.chi_rxrsp.bits.Opcode === RspOpcode.CompDBIDResp || io.chi_rxrsp.bits.Opcode === RspOpcode.DBIDResp)).asUInt

    endIndex := endIndex + 1.U
    judgeSendReq := true.B
  }.elsewhen(lastWrData && judgeSendReq  && mergeValid_s2){
    wrStateEntrys(endIndex).areid := selWrDataEntry
    wrStateEntrys(endIndex).state := WRState.SendWrReq
    wrStateEntrys(endIndex).full  := false.B
    wrStateEntrys(endIndex).last  := true.B
    wrStateEntrys(endIndex).sendReqOrder := PopCount(sendReqNum) - (io.chi_rxrsp.fire && 
    (io.chi_rxrsp.bits.Opcode === RspOpcode.CompDBIDResp || io.chi_rxrsp.bits.Opcode === RspOpcode.DBIDResp)).asUInt

    endIndex := endIndex + 1.U
    judgeSendReq := true.B
  }.elsewhen(lastWrData && !judgeSendReq  && mergeValid_s2){
    wrStateEntrys(endIndex).areid := selWrDataEntry
    wrStateEntrys(endIndex).state := WRState.WaitDBID
    wrStateEntrys(endIndex).full  := true.B
    wrStateEntrys(endIndex).last  := true.B
    wrStateEntrys(endIndex).sendReqOrder := PopCount(sendReqNum) - (io.chi_rxrsp.fire && 
    (io.chi_rxrsp.bits.Opcode === RspOpcode.CompDBIDResp || io.chi_rxrsp.bits.Opcode === RspOpcode.DBIDResp)).asUInt

    endIndex := endIndex + 1.U
    judgeSendReq := true.B
  }


  /* 
  select Entry to send WriteUnique and Send Request Flit to HN
   */

  val sramSendReqVec    = wrStateEntrys.map(w => w.sendReqOrder === 0.U && w.state === WRState.SendWrReq)
  val txWrReqValid      = sramSendReqVec.reduce(_|_)
  val selSendReqEntry   = PriorityEncoder(sramSendReqVec)
  val txReqFlit         = Wire(new ReqFlit)
  txReqFlit            := 0.U.asTypeOf(txReqFlit)

  txReqFlit.Addr       := Mux(txWrReqValid, awEntrys(wrStateEntrys(selSendReqEntry).areid).addr, 0.U)
  txReqFlit.ExpCompAck := Mux(txWrReqValid, true.B, false.B)
  txReqFlit.Opcode     := Mux(txWrReqValid & awEntrys(wrStateEntrys(selSendReqEntry).areid).addr(raw - 1), ReqOpcode.WriteNoSnpPtl, Mux(txWrReqValid, ReqOpcode.WriteUniquePtl, 0.U))
  txReqFlit.Order      := Mux(txWrReqValid, "b10".U, 0.U)
  txReqFlit.SrcID      := 1.U
  txReqFlit.TxnID      := selSendReqEntry
  txReqFlit.Size       := Mux(wrStateEntrys(selSendReqEntry).full, 6.U, 5.U)

  // val txnid            = Wire(UInt(log2Ceil(dmaParams.bufferSize).W))
  val txnid               = io.chi_rxrsp.bits.TxnID(log2Ceil(dmaParams.bufferSize) - 1, 0) + 1.U


  when(io.chi_rxrsp.fire && wrStateEntrys(io.chi_rxrsp.bits.TxnID).full && (io.chi_rxrsp.bits.Opcode === RspOpcode.DBIDResp || io.chi_rxrsp.bits.Opcode === RspOpcode.CompDBIDResp)){
    wrStateEntrys(txnid).state := WRState.SendWrData
    wrStateEntrys(txnid).dbid  := io.chi_rxrsp.bits.DBID
    wrStateEntrys(txnid).tgtID := io.chi_rxrsp.bits.SrcID
  }

  when(wrStateEntrys(startIndex + 1.U).state === WRState.Free && startIndex =/= endIndex && wrStateEntrys(startIndex).state === WRState.Free){
    startIndex := startIndex + 1.U
  }

  /* 
  select Entry which should send data to HN and send WriteData to HN
   */

  val sendDataVec      = wrStateEntrys.map(_.state === WRState.SendWrData)
  val sendDataValid    = sendDataVec.reduce(_|_)
  val selSendDataEntry = PriorityEncoder(sendDataVec)

  val txDatFlit        = RegInit(0.U.asTypeOf(new DataFlit))

  dataSram.io.r.req.valid       := sendDataValid & !dataSram.io.w.req.valid
  dataSram.io.r.req.bits.setIdx := selSendDataEntry
  maskSram.io.r.req.valid       := sendDataValid & !maskSram.io.w.req.valid
  maskSram.io.r.req.bits.setIdx := selSendDataEntry

  

  val selSendReg   = RegInit(0.U(log2Ceil(dmaParams.bufferSize).W))
  selSendReg       := selSendDataEntry

  when(RegNext(dataSram.io.r.req.fire)){
    txDatFlit.Opcode := Mux(wrStateEntrys(selSendReg).separate, DatOpcode.NonCopyBackWriteData, DatOpcode.NCBWrDataCompAck)
    txDatFlit.SrcID  := 1.U
    txDatFlit.DataID := Mux(wrStateEntrys(selSendReg).full &&  wrStateEntrys(selSendReg).last, 2.U, 0.U)
    txDatFlit.TxnID  := wrStateEntrys(selSendReg).dbid
    txDatFlit.TgtID  := wrStateEntrys(selSendReg).tgtID
    txDatFlit.Data   := dataSram.io.r.resp.data(0)
    txDatFlit.BE     := maskSram.io.r.resp.data(0)
  }

  /* 
  select Entry which is ready to send compack to HN and send CHI_RSP Flit to HN
   */

   val sendCompAckVec      = wrStateEntrys.map(_.state === WRState.SendCompAck)
   val sendCompAckValid    = sendCompAckVec.reduce(_|_)
   val selSendCompAckEntry = PriorityEncoder(sendCompAckVec)

   val txRspFlit           = WireInit(0.U.asTypeOf(new RespFlit))
   when(sendCompAckValid){
    txRspFlit.Opcode   := RspOpcode.CompAck
    txRspFlit.SrcID    := 1.U
    txRspFlit.TgtID    := wrStateEntrys(selSendCompAckEntry).tgtID
    txRspFlit.TxnID    := wrStateEntrys(selSendCompAckEntry).dbid
   }

  //---------------------------------------------------------------------------------------------------------------------------------//
  //---------------------------------------------------------- FSM ------------------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  awEntrys.zipWithIndex.foreach{
    case(a, i) =>
      switch(a.state){
        is(AWState.Free){
          val hit = io.axi_aw.fire && selFreeAWEntry === i.U
          when(hit){
            a.state   := AWState.ReceiveData
            a.addr    := io.axi_aw.bits.addr & (~31.U(48.W))
            a.unalign := io.axi_aw.bits.addr(5)
            a.burst   := io.axi_aw.bits.burst
            a.size    := io.axi_aw.bits.size
            a.nid     := PopCount(awBusyVec) - (io.axi_b.fire).asUInt
            a.awid    := io.axi_aw.bits.id
          }
        }
        is(AWState.ReceiveData){
          val hit       = a.nid === 0.U && lastWrData  
          val reduceNid = a.nid =/= 0.U && lastWrData
          val addrIncr  = wrStateEntrys(io.chi_txreq.bits.TxnID).areid  === i.U && io.chi_txreq.fire
          val alignHit  = a.nid === 0.U && mergeValid_s2
          a.unalign := Mux(alignHit, false.B, a.unalign)
          a.state   := Mux(hit, AWState.Comp, a.state)
          a.nid     := Mux(reduceNid, a.nid - 1.U, a.nid)
          a.addr    := Mux(addrIncr && io.chi_txreq.bits.Size === 5.U, a.addr + 32.U, Mux(addrIncr && io.chi_txreq.bits.Size === 6.U, a.addr + 64.U, a.addr))
        }
        is(AWState.Comp){
          val hit       = a.nid === 0.U && io.axi_b.bits.id === a.awid && io.axi_b.fire
          val addrIncr  = wrStateEntrys(io.chi_txreq.bits.TxnID).areid  === i.U && io.chi_txreq.fire
          a.addr       := Mux(addrIncr && io.chi_txreq.bits.Size === 5.U, a.addr + 32.U, Mux(addrIncr && io.chi_txreq.bits.Size === 6.U, a.addr + 64.U, a.addr))
          when(hit){
            a := 0.U.asTypeOf(a)
          }
        }
      }
  }

  wrStateEntrys.zipWithIndex.foreach{
    case(w, i) =>
      switch(w.state){
        is(WRState.Free){
          
        }
        is(WRState.SendWrReq){
          val hit = io.chi_txreq.fire && io.chi_txreq.bits.TxnID === i.U
          val reduceHit = io.chi_rxrsp.fire && (io.chi_rxrsp.bits.Opcode === RspOpcode.CompDBIDResp || io.chi_rxrsp.bits.Opcode === RspOpcode.DBIDResp) && (awEntrys(w.areid).nid === 0.U)  && (w.sendReqOrder =/= 0.U)
          when(hit){
            w.state := WRState.WaitDBID
          }
          when(reduceHit){
            w.sendReqOrder  := w.sendReqOrder - 1.U
          }

        }
        is(WRState.WaitDBID){
          val aggHit    = io.chi_rxrsp.fire && io.chi_rxrsp.bits.TxnID === i.U && io.chi_rxrsp.bits.Opcode === RspOpcode.CompDBIDResp
          val sepHit    = io.chi_rxrsp.fire && io.chi_rxrsp.bits.TxnID === i.U && io.chi_rxrsp.bits.Opcode === RspOpcode.DBIDResp
          

          when(aggHit || sepHit){
            w.state    := WRState.SendWrData
            w.dbid     := io.chi_rxrsp.bits.DBID
            w.tgtID    := io.chi_rxrsp.bits.SrcID
            w.separate := Mux(sepHit, true.B, false.B)
          }
        }
        is(WRState.SendWrData){
          val hit    = dataSram.io.r.req.fire && dataSram.io.r.req.bits.setIdx === i.U
          when(hit){
            w.state   := Mux(!w.separate, WRState.Complete, WRState.WaitComp)
          }
        }
        is(WRState.WaitComp){
          val hit = io.chi_rxrsp.fire && io.chi_rxrsp.bits.Opcode === RspOpcode.Comp && io.chi_rxrsp.bits.TxnID === i.U
          when(hit){
            w.state := WRState.SendCompAck
          }
        }
        is(WRState.SendCompAck){
          val hit = io.chi_txrsp.fire && io.chi_txrsp.bits.TxnID === w.dbid //TODO
          when(hit){
            w := 0.U.asTypeOf(w)
          }
        }
        is(WRState.Complete){
          val hit = !w.separate &&  io.chi_txdat.fire && io.chi_txdat.bits.TxnID === w.dbid &&
           (io.chi_txdat.bits.DataID === 0.U && !w.full || w.full && io.chi_txdat.bits.DataID === 0.U && !w.last ||
          io.chi_txdat.bits.DataID === 2.U && w.last && w.full)
          when(hit){
            w := 0.U.asTypeOf(w)
          }
        }
      }
  }

  //---------------------------------------------------------------------------------------------------------------------------------//
  //--------------------------------------------------------- IO Logic --------------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  //CHI interface

  io.chi_txrsp.valid := sendCompAckValid
  io.chi_txrsp.bits  := txRspFlit
  io.chi_txreq.valid := txWrReqValid
  io.chi_txreq.bits  := txReqFlit
  io.chi_txdat.valid := RegNext(RegNext(dataSram.io.r.req.fire))
  io.chi_txdat.bits  := Mux(RegNext(RegNext(dataSram.io.r.req.fire)), txDatFlit, 0.U.asTypeOf(txDatFlit))
  io.chi_rxrsp.ready := true.B

  //AXI interface
  io.axi_aw.ready    := awFreeVec.reduce(_|_)
  io.axi_w.ready     := endIndex + 1.U =/= startIndex
  io.axi_b.valid     := sendAxiBValid
  io.axi_b.bits      := 0.U.asTypeOf(io.axi_b.bits)
  io.axi_b.bits.id   := awEntrys(selCompEntry).awid
  
}