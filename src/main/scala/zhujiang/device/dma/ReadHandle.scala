package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.cde.config.Parameters
import zhujiang._
import zhujiang.chi._
import zhujiang.axi._
import xs.utils.sram._


object ARState {
  val width = 2
  val Free  = "b00".U
  val Send  = "b01".U
  val Comp  = "b10".U
}
class AREntry(implicit p: Parameters) extends ZJBundle {
    val state           = UInt(ARState.width.W)
    val addr            = UInt(raw.W)
    val burst           = UInt(BurstMode.width.W)
    val len             = UInt(8.W)
    val arid            = UInt(8.W)
    val nid             = UInt(log2Ceil(zjParams.dmaParams.axiEntrySize).W)
    val sendReqNum      = UInt(6.W)
    val sendDatNum      = UInt(6.W)  
}

class sramStateEntry(implicit p : Parameters) extends ZJBundle {
  val areid         = UInt(log2Ceil(zjParams.dmaParams.axiEntrySize).W)
  val state         = UInt(SRAMState.width.W)
  val num           = UInt(6.W)
  val last          = Bool()
  val next          = UInt((log2Ceil(zjParams.dmaParams.chiEntrySize)).W)
  val full          = Bool()
  val homeNid       = UInt(niw.W)
  val dbid          = UInt(12.W)
}

object SRAMState {
  val width        = 3
  val Free         = "b000".U
  val sendReq      = "b001".U
  val Wait         = "b010".U
  val WaitData     = "b011".U
  val WaitReceipt  = "b100".U
  val SendCompAck  = "b101".U
  val SendData     = "b110".U
  val Comp         = "b111".U
}
class SRAMSelector(implicit p: Parameters) extends ZJModule {
  private val dmaParams = zjParams.dmaParams
  val io = IO(new Bundle() {
    val idle = Input(Vec(dmaParams.chiEntrySize, Bool()))
    val idleNum = Output(UInt((log2Ceil(dmaParams.chiEntrySize) + 1).W))
    val out0 = UInt(log2Ceil(dmaParams.chiEntrySize).W)
    val out1 = UInt(log2Ceil(dmaParams.chiEntrySize).W)
  })
  io.idleNum := PopCount(io.idle)
  io.out0    := PriorityEncoder(io.idle)
  val idle1   = WireInit(io.idle)
  idle1(io.out0) := false.B
  io.out1    := PriorityEncoder(idle1)
}

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
  //-------------------------------------------------- Reg and Wire Define ----------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//

  val arEntrys          = RegInit(VecInit.fill(dmaParams.axiEntrySize)(0.U.asTypeOf(new AREntry)))
  val sramSelector      = Module(new SRAMSelector)
  val readSram          = Module(new SRAMTemplate(gen = UInt(dw.W), set = dmaParams.chiEntrySize, singlePort = true))


  val sramStateEntrys   = RegInit(VecInit.fill(dmaParams.chiEntrySize)(0.U.asTypeOf(new sramStateEntry)))
  val sendDataReg       = RegInit(0.U(dw.W))
  val sendDataRegValid  = WireInit(false.B)
  val dataTxnid         = WireInit(io.chi_rxdat.bits.TxnID(log2Ceil(dmaParams.chiEntrySize) - 1, 0))



  
  

  //---------------------------------------------------------------------------------------------------------------------------------//
  //---------------------------------------------------------- Logic ----------------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  val sramFreeVec       = sramStateEntrys.map(_.state === SRAMState.Free)
  val sramSendReqVec    = sramStateEntrys.map( s => s.state === SRAMState.sendReq)
  val sramSendAckVec    = sramStateEntrys.map(_.state === SRAMState.SendCompAck)
  val selSramSendReq    = PriorityEncoder(sramSendReqVec)
  val selSendAck        = PriorityEncoder(sramSendAckVec)

  val sramReceiveFirstVec    = sramStateEntrys.map(s => s.state === SRAMState.SendData && s.num === 0.U && !s.last)
  val sramReceiveSecondVec   = sramStateEntrys.map(s => s.state === SRAMState.SendData && s.num === 0.U && s.last)
  val sramReceiveVec         = sramStateEntrys.map(s => s.state === SRAMState.SendData && s.num === 0.U)
  val selSramReceive         = PriorityEncoder(sramReceiveVec)

  sramSelector.io.idle := sramFreeVec

  val arFreeVec         = arEntrys.map(_.state === ARState.Free)
  val selFreeAREntry    = PriorityEncoder(arFreeVec)
  val arSendVec         = arEntrys.map(_.state === ARState.Send)
  val selSendAREntry    = PriorityEncoder(arSendVec)

  val nidVec            = arEntrys.map(a => a.arid === io.axi_ar.bits.id && a.state =/= ARState.Free)

  val sendHalfReq       = arSendVec.reduce(_|_) && (sramSelector.io.idleNum >= 1.U) && (
  arEntrys(selSendAREntry).len === arEntrys(selSendAREntry).sendReqNum || 
  arEntrys(selSendAREntry).addr(5) && arEntrys(selSendAREntry).sendReqNum === 0.U)

  val sendFullReq       = arSendVec.reduce(_|_) && (sramSelector.io.idleNum >= 2.U) && arEntrys(selSendAREntry).sendReqNum + 1.U <= arEntrys(selSendAREntry).len  && !arEntrys(selSendAREntry).addr(5)

  arEntrys(selSendAREntry).sendReqNum := Mux(sendHalfReq && PopCount(sramFreeVec) >= 1.U, arEntrys(selSendAREntry).sendReqNum + 1.U, 
                                            Mux(sendFullReq && PopCount(sramFreeVec) >= 2.U, arEntrys(selSendAREntry).sendReqNum + 2.U, arEntrys(selSendAREntry).sendReqNum))
  when(sendFullReq){
    sramStateEntrys(sramSelector.io.out1).areid := selSendAREntry
    sramStateEntrys(sramSelector.io.out1).full  := true.B
    sramStateEntrys(sramSelector.io.out1).last  := true.B
    sramStateEntrys(sramSelector.io.out1).next  := 0.U
    sramStateEntrys(sramSelector.io.out1).num   := arEntrys(selSendAREntry).sendReqNum - arEntrys(selSendAREntry).sendDatNum - (io.axi_r.fire && io.axi_r.bits.id === arEntrys(sramStateEntrys(sramSelector.io.out1).areid).arid && arEntrys(sramStateEntrys(sramSelector.io.out1).areid).nid === 0.U).asUInt
    sramStateEntrys(sramSelector.io.out1).state := SRAMState.WaitData

    sramStateEntrys(sramSelector.io.out0).areid := selSendAREntry
    sramStateEntrys(sramSelector.io.out0).last  := false.B
    sramStateEntrys(sramSelector.io.out0).full  := true.B
    sramStateEntrys(sramSelector.io.out0).next  := sramSelector.io.out1
    sramStateEntrys(sramSelector.io.out0).num   := arEntrys(selSendAREntry).sendReqNum - arEntrys(selSendAREntry).sendDatNum - (io.axi_r.fire && io.axi_r.bits.id === arEntrys(sramStateEntrys(sramSelector.io.out0).areid).arid && arEntrys(sramStateEntrys(sramSelector.io.out0).areid).nid === 0.U).asUInt
    sramStateEntrys(sramSelector.io.out0).state := SRAMState.sendReq

  }
  when(sendHalfReq){
    sramStateEntrys(sramSelector.io.out0).areid := selSendAREntry
    sramStateEntrys(sramSelector.io.out0).last  := true.B
    sramStateEntrys(sramSelector.io.out0).full  := false.B
    sramStateEntrys(sramSelector.io.out0).next  := 0.U
    sramStateEntrys(sramSelector.io.out0).num   := arEntrys(selSendAREntry).sendReqNum - arEntrys(selSendAREntry).sendDatNum - (io.axi_r.fire && io.axi_r.bits.id === arEntrys(sramStateEntrys(sramSelector.io.out0).areid).arid && arEntrys(sramStateEntrys(sramSelector.io.out0).areid).nid === 0.U).asUInt
    sramStateEntrys(sramSelector.io.out0).state := SRAMState.sendReq

  }
  
  val txReqValid         = sramSendReqVec.reduce(_|_)
  val txReqFlit          = Wire(new ReqFlit)
  txReqFlit             := 0.U.asTypeOf(txReqFlit)
  txReqFlit.Addr        := arEntrys(sramStateEntrys(selSramSendReq).areid).addr
  txReqFlit.Opcode      := Mux(arEntrys(sramStateEntrys(selSramSendReq).areid).addr(raw - 1), ReqOpcode.ReadNoSnp, ReqOpcode.ReadOnce)
  txReqFlit.Order       := "b11".U
  txReqFlit.TxnID       := selSramSendReq
  txReqFlit.ExpCompAck  := true.B
  txReqFlit.Size        := Mux(!sramStateEntrys(selSramSendReq).full, "b101".U, "b110".U)
  txReqFlit.SrcID       := 1.U

  val sramWrDatReg       = RegInit(0.U(dw.W))
  val sramWrDatRegValid  = RegInit(false.B)
  val writeSram          = sramStateEntrys(dataTxnid).state =/= SRAMState.Wait && sramStateEntrys(dataTxnid).state =/= SRAMState.WaitData && io.chi_rxdat.fire
  val selWrEntry         = Mux(writeSram, sramStateEntrys(dataTxnid).next, dataTxnid)
  val sramWrSet            = RegNext(selWrEntry)
  sramWrDatRegValid     := io.chi_rxdat.fire
  when(io.chi_rxdat.fire){
    sramWrDatReg      := io.chi_rxdat.bits.Data
  }

  readSram.io.w.req.valid := sramWrDatRegValid
  readSram.io.w.req.bits.setIdx := sramWrSet
  readSram.io.w.req.bits.data(0) := sramWrDatReg

  val sramRdDatReg      = RegInit(0.U(dw.W))
  val sramRdDatRegValid = RegInit(false.B)
  val sramRdValidWire   = WireInit(sramReceiveVec.reduce(_|_))
  val sramRdSet         = Mux(sramReceiveFirstVec.reduce(_|_) && PopCount(sramReceiveFirstVec) >= 1.U, PriorityEncoder(sramReceiveFirstVec), PriorityEncoder(sramReceiveSecondVec))
  val sramRdSetReg      = RegNext(sramRdSet)
  val axiRSet           = RegNext(sramRdSetReg)


  readSram.io.r.req.valid       := sramRdValidWire & !readSram.io.w.req.valid
  readSram.io.r.req.bits.setIdx := sramRdSet
  sramRdDatReg           := readSram.io.r.resp.data(0)
  sramRdDatRegValid      := RegNext(readSram.io.r.req.fire)

  val axiRFlit      = Wire(new RFlit(axiParams))
  axiRFlit         := 0.U.asTypeOf(axiRFlit)
  axiRFlit.data    := sramRdDatReg
  axiRFlit.id      := arEntrys(sramStateEntrys(axiRSet).areid).arid
  axiRFlit.last    := arEntrys(sramStateEntrys(axiRSet).areid).sendDatNum === arEntrys(sramStateEntrys(axiRSet).areid).len & arEntrys(sramStateEntrys(axiRSet).areid).state =/= ARState.Free

  when(writeSram){
    sramStateEntrys(sramStateEntrys(dataTxnid).next).state := SRAMState.SendCompAck
    sramStateEntrys(sramStateEntrys(dataTxnid).next).dbid  := io.chi_rxdat.bits.DBID
    sramStateEntrys(sramStateEntrys(dataTxnid).next).homeNid := io.chi_rxdat.bits.HomeNID
  }

  val txRspFlit     = Wire(new RespFlit)
  txRspFlit        := 0.U.asTypeOf(txRspFlit)
  txRspFlit.TxnID  := sramStateEntrys(selSendAck).dbid
  txRspFlit.SrcID  := 1.U
  txRspFlit.Opcode := RspOpcode.CompAck
  txRspFlit.TgtID  := sramStateEntrys(selSendAck).homeNid
  

  sramStateEntrys.foreach{
    case(sram) =>
      when(sram.num =/= 0.U && io.axi_r.fire && io.axi_r.bits.id === arEntrys(sram.areid).arid && arEntrys(sram.areid).nid === 0.U){
        sram.num := sram.num - 1.U
      }
  }


  //---------------------------------------------------------------------------------------------------------------------------------//
  //------------------------------------------------------- State Transfer ----------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//
  arEntrys.zipWithIndex.foreach{
    case(a, i) =>
      switch(a.state){
        is(ARState.Free){
          val hit = io.axi_ar.fire && selFreeAREntry === i.U
          val nid = PopCount(nidVec) - (io.axi_r.fire && io.axi_r.bits.last && io.axi_r.bits.id === io.axi_ar.bits.id).asUInt
          when(hit){
            a.state := ARState.Send
            a.addr  := io.axi_ar.bits.addr & (~31.U(48.W))
            a.arid  := io.axi_ar.bits.id
            a.len   := io.axi_ar.bits.len
            a.burst := io.axi_ar.bits.burst
            a.nid   := nid
            a.sendReqNum := 0.U
            a.sendDatNum := 0.U
          }
        }
        is(ARState.Send){
          val compHit = (a.sendReqNum === (a.len + 1.U)) && selSendAREntry === i.U
          a.state := Mux(compHit, ARState.Comp, a.state)
          val numAdd = io.axi_r.fire && io.axi_r.bits.id === a.arid && a.nid === 0.U
          a.sendDatNum := Mux(numAdd, a.sendDatNum + 1.U , a.sendDatNum)
          val addrIncrFullHit = io.chi_txreq.fire && selSendAREntry === i.U && io.chi_txreq.bits.Size === 6.U
          val addrIncrHalfHit = io.chi_txreq.fire && selSendAREntry === i.U && io.chi_txreq.bits.Size === 5.U
          a.addr := Mux(addrIncrHalfHit, a.addr + 32.U, Mux(addrIncrFullHit, a.addr + 64.U, a.addr))
          
        }
        is(ARState.Comp){
          val hit = a.arid === io.axi_r.bits.id && io.axi_r.fire && a.nid === 0.U && io.axi_r.bits.last
          val numAdd = io.axi_r.fire && io.axi_r.bits.id === a.arid && a.nid === 0.U
          a.sendDatNum := Mux(numAdd, a.sendDatNum + 1.U , a.sendDatNum)
          when(hit){
            a := 0.U.asTypeOf(a)
          }
          
          
        }
      }
    }

    sramStateEntrys.zipWithIndex.foreach{
      case(sram, i) =>
        switch(sram.state){
          is(SRAMState.Free){
            
          }
          is(SRAMState.sendReq){
            val hit = io.chi_txreq.fire && (io.chi_txreq.bits.TxnID === i.U).asBool
            sram.state := Mux(hit, SRAMState.Wait, sram.state)
          }
          is(SRAMState.Wait){
            val receTxnid  = io.chi_rxrsp.bits.TxnID
            val dataTxnid  = io.chi_rxdat.bits.TxnID
            val dataHit    = io.chi_rxdat.fire && dataTxnid === i.U 
            val receiptHit = io.chi_rxrsp.fire & (io.chi_rxrsp.bits.Opcode === RspOpcode.ReadReceipt).asBool && receTxnid === i.U 
            when(dataHit && receiptHit){
              sram.state := Mux(sram.last && !sram.full, SRAMState.SendCompAck, SRAMState.SendData)
              sram.dbid  := io.chi_rxdat.bits.DBID(11,0)
              sram.homeNid := io.chi_rxdat.bits.HomeNID
            }.elsewhen(dataHit){
              sram.state  := SRAMState.WaitReceipt
              sram.dbid   := io.chi_rxdat.bits.DBID(11,0)
              sram.homeNid := io.chi_rxdat.bits.HomeNID
            }.elsewhen(receiptHit){
              sram.state  := SRAMState.WaitData
            }
          }
          is(SRAMState.WaitData){
            val dataTxnid  = io.chi_rxdat.bits.TxnID
            val dataHit    = io.chi_rxdat.fire && dataTxnid === i.U 
            when(dataHit){
              sram.state := Mux(sram.last && !sram.full, SRAMState.SendCompAck, SRAMState.SendData)
              sram.dbid  := io.chi_rxdat.bits.DBID(11, 0)
              sram.homeNid := io.chi_rxdat.bits.HomeNID
            }
          }
          is(SRAMState.WaitReceipt){
            val receTxnid  = io.chi_rxrsp.bits.TxnID
            val receiptHit        = io.chi_rxrsp.fire && (io.chi_rxrsp.bits.Opcode === RspOpcode.ReadReceipt).asBool && receTxnid === i.U
            when(receiptHit){
              sram.state := Mux(sram.last && !sram.full, SRAMState.SendData, SRAMState.SendCompAck)
            }
          }
          is(SRAMState.SendCompAck){
            val sendHit    =  sramSendAckVec.reduce(_|_) && selSendAck === i.U
            sram.state    := Mux(sendHit, SRAMState.SendData, sram.state)
          }
          is(SRAMState.SendData){
            val hit = readSram.io.r.req.fire && readSram.io.r.req.bits.setIdx === i.U
            when(hit){
              sram.state := SRAMState.Comp
            }
          }
          is(SRAMState.Comp){
            val hit = io.axi_r.fire && io.axi_r.bits.id === arEntrys(sram.areid).arid && 
            arEntrys(sram.areid).nid === 0.U && sram.num === 0.U
            when(hit){
              sram := 0.U.asTypeOf(sram)
            }
          }
        }
    }
  

  //---------------------------------------------------------------------------------------------------------------------------------//
  //---------------------------------------------------- IO Interface ---------------------------------------------------------------//
  //---------------------------------------------------------------------------------------------------------------------------------//

  io.axi_ar.ready       := arFreeVec.reduce(_|_)
  io.chi_rxrsp.ready    := true.B
  io.chi_txreq.bits     := txReqFlit
  io.chi_txreq.valid    := txReqValid
  io.chi_rxdat.ready    := true.B 
  io.chi_txrsp.valid    := sramSendAckVec.reduce(_|_)
  io.chi_txrsp.bits     := txRspFlit
  io.axi_r.valid        := sramRdDatRegValid
  io.axi_r.bits         := axiRFlit

}