  package zhujiang.device.dma

  import chisel3._
  import chisel3.util._
  import org.chipsalliance.cde.config._
  import org.chipsalliance.cde.config.Parameters
  import zhujiang._
  import zhujiang.chi._
  import zhujiang.axi._
  import xs.utils.sram._

  //---------------------------------------------------------------------------------------------------------------------------------//
  //---------------------------------------------------- Module Define --------------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
class ReadHandle(implicit p: Parameters) extends ZJModule{
  private val dmaParams = zjParams.dmaParams
  private val axiParams = AxiParams(dataBits = dw, addrBits = raw, idBits = dmaParams.idBits)
  val io = IO(new Bundle {
    // AXI4 Interface
    val axi_ar = Flipped(Decoupled(new ARFlit(axiParams)))
    val axi_r  = Decoupled(new RFlit(axiParams))

    // CHI Interface
    val chi_txreq = Decoupled(new ReqFlit)
    val chi_rxrsp = Flipped(Decoupled(new RespFlit))
    val chi_rxdat = Flipped(Decoupled(new DataFlit))
    val chi_txrsp = Decoupled(new RespFlit)
  })
  //---------------------------------------------------------------------------------------------------------------------------------//
  //---------------------------------------------------- Reg and Wire Define --------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  val axiEntrys      = RegInit(VecInit.fill(dmaParams.axiEntrySize)(0.U.asTypeOf(new AXIREntry)))
  val sramSelector   = Module(new SRAMSelector)
  val readSram       = Module(new SRAMTemplate(gen = UInt(dw.W), set = dmaParams.chiEntrySize, singlePort = true))

  val chiEntrys      = RegInit(VecInit.fill(dmaParams.chiEntrySize)(0.U.asTypeOf(new CHIREntry)))
  val sendDataReg    = RegInit(0.U(dw.W))
  val sendDataValid  = WireInit(false.B)
  val dataTxnid      = WireInit(io.chi_rxdat.bits.TxnID(log2Ceil(dmaParams.chiEntrySize) - 1, 0))
  val rspTxnid       = WireInit(io.chi_rxrsp.bits.TxnID(log2Ceil(dmaParams.chiEntrySize) - 1, 0))  

  //---------------------------------------------------------------------------------------------------------------------------------//
  //----------------------------------------------------------- Logic ---------------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  val axiFreeVec      = axiEntrys.map(_.state === AXIRState.Free)
  val selFreeAxiEntry = PriorityEncoder(axiFreeVec)
  val axiSendVec      = axiEntrys.map(_.state === AXIRState.Send)
  val selSendAxiEntry = PriorityEncoder(axiSendVec)

  val nidVec          = axiEntrys.map(a => a.arid === io.axi_ar.bits.id && a.state =/= AXIRState.Free)


  val chiFreeVec     = chiEntrys.map(_.state === CHIRState.Free)
  val chiSendAckVec  = chiEntrys.map(_.state === CHIRState.sendCompAck)
  val chiSendDatVec  = chiEntrys.map(c => c.state === CHIRState.SendData && c.num === 0.U)

  val selSendAckChiEntry = PriorityEncoder(chiSendAckVec)
  val selSendDatChiEntry = PriorityEncoder(chiSendDatVec)


  sramSelector.io.idle := chiFreeVec

  val sendHalfReq    = io.chi_txreq.ready && axiSendVec.reduce(_|_) && sramSelector.io.idleNum >= 1.U && (axiEntrys(selSendAxiEntry).addr(5) &&
  axiEntrys(selSendAxiEntry).sendReqNum === 0.U || axiEntrys(selSendAxiEntry).len === axiEntrys(selSendAxiEntry).sendReqNum)
  val sendFullReq    = io.chi_txreq.ready && axiSendVec.reduce(_|_) && sramSelector.io.idleNum >= 2.U && 
  axiEntrys(selSendAxiEntry).sendReqNum + 1.U <= axiEntrys(selSendAxiEntry).len && !axiEntrys(selSendAxiEntry).addr(5)

  axiEntrys(selSendAxiEntry).sendReqNum := Mux(sendHalfReq, axiEntrys(selSendAxiEntry).sendReqNum + 1.U, 
                                            Mux(sendFullReq, axiEntrys(selSendAxiEntry).sendReqNum + 2.U, axiEntrys(selSendAxiEntry).sendReqNum))
  when(sendFullReq){
    chiEntrys(sramSelector.io.out0).areid := selSendAxiEntry
    chiEntrys(sramSelector.io.out0).full  := true.B
    chiEntrys(sramSelector.io.out0).last  := false.B
    chiEntrys(sramSelector.io.out0).num   := axiEntrys(selSendAxiEntry).sendReqNum - axiEntrys(selSendAxiEntry).sendDatNum - (io.axi_r.fire && 
    io.axi_r.bits.id === axiEntrys(selSendAxiEntry).arid && axiEntrys(selSendAxiEntry).nid === 0.U).asUInt
    chiEntrys(sramSelector.io.out0).next  := sramSelector.io.out1
    chiEntrys(sramSelector.io.out0).state := CHIRState.Wait

    chiEntrys(sramSelector.io.out1).areid := selSendAxiEntry
    chiEntrys(sramSelector.io.out1).full  := true.B
    chiEntrys(sramSelector.io.out1).last  := true.B
    chiEntrys(sramSelector.io.out1).num   := axiEntrys(selSendAxiEntry).sendReqNum + 1.U - axiEntrys(selSendAxiEntry).sendDatNum - (io.axi_r.fire &&
    io.axi_r.bits.id === axiEntrys(selSendAxiEntry).arid && axiEntrys(selSendAxiEntry).nid === 0.U).asUInt
    chiEntrys(sramSelector.io.out1).state := CHIRState.Wait

    axiEntrys(selSendAxiEntry).addr       := axiEntrys(selSendAxiEntry).addr + 64.U 
  }.elsewhen(sendHalfReq){
    chiEntrys(sramSelector.io.out0).areid := selSendAxiEntry
    chiEntrys(sramSelector.io.out0).full  := false.B
    chiEntrys(sramSelector.io.out0).last  := true.B
    chiEntrys(sramSelector.io.out0).num   := axiEntrys(selSendAxiEntry).sendReqNum - axiEntrys(selSendAxiEntry).sendDatNum - (io.axi_r.fire &&
    io.axi_r.bits.id === axiEntrys(selSendAxiEntry).arid && axiEntrys(selSendAxiEntry).nid === 0.U).asUInt
    chiEntrys(sramSelector.io.out0).state := CHIRState.Wait

    axiEntrys(selSendAxiEntry).addr       := axiEntrys(selSendAxiEntry).addr + 32.U 
  }
  val txReqFlit     = Wire(new ReqFlit)
  txReqFlit        := 0.U.asTypeOf(txReqFlit)
  txReqFlit.Addr   := axiEntrys(selSendAxiEntry).addr
  txReqFlit.Opcode := Mux(axiEntrys(selSendAxiEntry).addr(raw - 1), ReqOpcode.ReadNoSnp, ReqOpcode.ReadOnce)
  txReqFlit.ExpCompAck := true.B
  txReqFlit.Order  := "b11".U
  txReqFlit.TxnID  := sramSelector.io.out0
  txReqFlit.Size   := Mux(sendHalfReq, "b101".U, "b110".U)
  txReqFlit.SrcID  := 1.U

  val sramWrDatValidReg = RegInit(false.B)
  val sramWrDatReg      = RegEnable(io.chi_rxdat.bits.Data, io.chi_rxdat.fire)
  val sramWrSetReg      = RegInit(0.U(log2Ceil(dmaParams.chiEntrySize).W))
  val sramWrSet         = Mux(io.chi_rxdat.bits.DataID === 2.U, chiEntrys(dataTxnid).next, dataTxnid)

  sramWrDatValidReg := io.chi_rxdat.fire
  sramWrSetReg      := sramWrSet

  readSram.io.w.req.valid := sramWrDatValidReg
  readSram.io.w.req.bits.setIdx := sramWrSetReg
  readSram.io.w.req.bits.data(0) := sramWrDatReg

  val txRspFlit = Wire(new RespFlit)
  txRspFlit       := 0.U.asTypeOf(txRspFlit)
  txRspFlit.TxnID := chiEntrys(selSendAckChiEntry).dbid
  txRspFlit.SrcID := 1.U
  txRspFlit.Opcode := RspOpcode.CompAck
  txRspFlit.TgtID  := chiEntrys(selSendAckChiEntry).homeNid
  
  readSram.io.r.req.valid := chiSendDatVec.reduce(_|_) && !readSram.io.w.req.valid
  readSram.io.r.req.bits.setIdx := selSendDatChiEntry
  
  val sramRdDatReg     = RegInit(0.U(dw.W))
  sramRdDatReg        := readSram.io.r.resp.data(0)
  val sramRdDatValid   = RegNext(RegNext(readSram.io.r.req.fire))
  val sramRdSet        = RegNext(RegNext(selSendDatChiEntry))

  val axiRFlit     = Wire(new RFlit(axiParams))
  axiRFlit        := 0.U.asTypeOf(axiRFlit)
  axiRFlit.data   := sramRdDatReg
  axiRFlit.id     := axiEntrys(chiEntrys(sramRdSet).areid).arid 
  axiRFlit.last   := axiEntrys(chiEntrys(sramRdSet).areid).sendDatNum === axiEntrys(chiEntrys(sramRdSet).areid).len && axiEntrys(chiEntrys(sramRdSet).areid).state =/= AXIRState.Free

  when(io.chi_rxdat.fire && io.chi_rxdat.bits.DataID === 2.U){
    chiEntrys(chiEntrys(dataTxnid).next).state := CHIRState.SendData
  }
  




  //---------------------------------------------------------------------------------------------------------------------------------//
  //-------------------------------------------------------- State Transfer ---------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  axiEntrys.zipWithIndex.foreach{
    case(a, i) =>
      switch(a.state){
        is(CHIRState.Free){
          val hit = io.axi_ar.fire && selFreeAxiEntry === i.U
          val nid = PopCount(nidVec) - (io.axi_r.fire && io.axi_r.bits.last && io.axi_r.bits.id === io.axi_ar.bits.id).asUInt
          when(hit){
            a.state  := AXIRState.Send
            a.addr   := io.axi_ar.bits.addr & (~31.U(raw.W))
            a.arid   := io.axi_ar.bits.id
            a.len    := io.axi_ar.bits.len
            a.burst  := io.axi_ar.bits.burst
            a.nid    := nid
            a.sendReqNum := 0.U
            a.sendDatNum := 0.U
          }
        }
        is(AXIRState.Send){
          val compHit = (a.sendReqNum === (a.len + 1.U)) && selSendAxiEntry === i.U
          val datNumAdd = io.axi_r.fire && io.axi_r.bits.id === a.arid && a.nid === 0.U
          when(compHit){
            a.state := AXIRState.Comp
          }
          a.sendDatNum := Mux(datNumAdd, a.sendDatNum + 1.U, a.sendDatNum)
        }
        is(AXIRState.Comp){
          val hit = io.axi_r.fire && io.axi_r.bits.id === a.arid && a.nid === 0.U && io.axi_r.bits.last
          val datNumAdd = io.axi_r.fire && io.axi_r.bits.id === a.arid && a.nid === 0.U
          a.sendDatNum := Mux(datNumAdd, a.sendDatNum + 1.U, a.sendDatNum)
          when(hit){
            a := 0.U.asTypeOf(a)
          }
        }
      }
  }
  axiEntrys.foreach{
    case(a) =>
      when(a.state =/= AXIRState.Free && a.nid =/= 0.U && io.axi_r.fire && io.axi_r.bits.last && io.axi_r.bits.id === a.arid){
        a.nid := a.nid - 1.U
      }
  }

  chiEntrys.foreach{
    case(c) =>
      when(c.num =/= 0.U && io.axi_r.fire && io.axi_r.bits.id === axiEntrys(c.areid).arid && axiEntrys(c.areid).nid === 0.U){
        c.num := c.num - 1.U
      }
  }
  
  chiEntrys.zipWithIndex.foreach{
    case(c, i) =>
      switch(c.state){
        is(CHIRState.Wait){
          val dataFirHit = io.chi_rxdat.fire && io.chi_rxdat.bits.DataID === 0.U && dataTxnid === i.U
          val dataSecHit = io.chi_rxdat.fire && io.chi_rxdat.bits.DataID === 2.U && dataTxnid === i.U
          val receHit    = io.chi_rxrsp.fire && rspTxnid  === i.U && io.chi_rxrsp.bits.Opcode === RspOpcode.ReadReceipt
          when(dataFirHit){
            c.haveRecDataFir := true.B
            c.homeNid        := io.chi_rxdat.bits.HomeNID
            c.dbid           := io.chi_rxdat.bits.DBID
          }.elsewhen(dataSecHit){
            c.haveRecDataSec := true.B 
          }
          when(receHit){
            c.haveRecReceipt := true.B
          }
          when(c.full && c.haveRecDataFir && c.haveRecDataSec && c.haveRecReceipt || !c.full && c.haveRecDataFir && c.haveRecReceipt){
            c.state := CHIRState.sendCompAck
          }
        }
        is(CHIRState.sendCompAck){
          val hit = io.chi_txrsp.fire && io.chi_txrsp.bits.TxnID === c.dbid && io.chi_txrsp.bits.TgtID === c.homeNid
          when(hit){
            c.state := CHIRState.SendData
          }
        }
        is(CHIRState.SendData){
          val hit = readSram.io.r.req.fire && readSram.io.r.req.bits.setIdx === i.U
          when(hit){
            c.state := CHIRState.Comp
          }
        }
        is(CHIRState.Comp){
          val hit = io.axi_r.fire && io.axi_r.bits.id === axiEntrys(c.areid).arid && axiEntrys(c.areid).nid === 0.U && c.num === 0.U
          when(hit){
            c := 0.U.asTypeOf(c)
          }
        }
      }
  }
 
  //---------------------------------------------------------------------------------------------------------------------------------//
  //----------------------------------------------------------- IO Interface --------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  
  io.chi_txreq.valid   := sendHalfReq || sendFullReq
  io.chi_txreq.bits    := txReqFlit
  io.chi_txrsp.valid   := chiSendAckVec.reduce(_|_)
  io.chi_txrsp.bits    := txRspFlit
  io.axi_r.valid       := sramRdDatValid
  io.axi_r.bits        := axiRFlit
  io.chi_rxdat.ready   := true.B
  io.chi_rxrsp.ready   := true.B
  io.axi_ar.ready      := axiFreeVec.reduce(_|_)



/* 
 * assert logic
 */
  when(io.chi_rxdat.fire && io.chi_rxdat.bits.DataID === 0.U){
    assert(!chiEntrys(dataTxnid).haveRecDataFir, "CHIEntrys haveRecDataFir logic is error")
  }
  when(io.chi_rxdat.fire && io.chi_rxdat.bits.DataID === 2.U){
    assert(!chiEntrys(dataTxnid).haveRecDataSec, "CHIEntrys haveRecDataSec logic is error")
    assert(chiEntrys(dataTxnid).full && !chiEntrys(dataTxnid).last, "the full&last logic of CHIEntrys is error")
  }
  when(io.chi_rxrsp.fire && io.chi_rxrsp.bits.Opcode === RspOpcode.ReadReceipt){
    assert(!chiEntrys(rspTxnid).haveRecReceipt, "CHIEntrys haveRecReceipt logic is error")
  }
}