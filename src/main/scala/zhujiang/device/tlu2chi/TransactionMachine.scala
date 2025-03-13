package zhujiang.device.tlu2chi

import chisel3._
import chisel3.util._
import xs.utils.debug.LeakChecker
import org.chipsalliance.cde.config._
import xijiang.Node
import xijiang.router.base.DeviceIcnBundle
import zhujiang.ZJModule
import zhujiang.chi._
import zhujiang.tilelink.{AOpcode, DFlit, DOpcode, TilelinkParams}
import zhujiang.ZJParametersKey
import coursier.core.shaded.sourcecode.Enclosing.Machine

object MachineState {
  val width = 4

  val IDLE         = 0.U(width.W)
  val SEND_REQ     = 1.U(width.W) // ReadNoSnp, WriteNoSnpPtl, WriteSnpFull
  val RECV_RSP     = 2.U(width.W) // DBIDResp
  val RECV_CMP     = 3.U(width.W) // Comp
  val SEND_DAT     = 4.U(width.W) // NonCopyBackWrData
  val SEND_ACK     = 5.U(width.W) // CompAck
  val RECV_RECEIPT = 6.U(width.W) // ReadReceipt
  val RECV_DAT     = 7.U(width.W) // CompData
  val RETURN_DAT   = 8.U(width.W) // Return data to TL(AccessAckData)
  val RETURN_ACK   = 9.U(width.W) // Return ack to TL(AccessAck)
}

class TaskBundle(tlParams: TilelinkParams)(implicit p: Parameters) extends Bundle {
  val address = UInt(tlParams.addrBits.W)
  val opcode = UInt(3.W)
  val param = UInt(3.W)
  val source = UInt(tlParams.sourceBits.W)
  val data = UInt(tlParams.dataBits.W)
  val mask = UInt((tlParams.dataBits / 8).W)
  val size = UInt(3.W)
}

class MachineStatus(tlParams: TilelinkParams)(implicit p: Parameters) extends Bundle {
  val state = Output(UInt(MachineState.width.W))
  val nextState = Output(UInt(MachineState.width.W))
  val address = UInt(tlParams.addrBits.W)
}

class TransactionMachine(node: Node, tlParams: TilelinkParams, outstanding: Int)(implicit p: Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val id = Input(UInt(log2Ceil(outstanding).W))
    val alloc = Flipped(ValidIO(new TaskBundle(tlParams)))
    val status = Output(new MachineStatus(tlParams))
    val issueReqEn = Input(Bool())
    val issueAckEn = Input(Bool())

    val icn = new DeviceIcnBundle(node)
    val tld = Decoupled(new DFlit(tlParams))
  })

  io <> DontCare

  private val txreq = Wire(Decoupled(new ReqFlit))
  io.icn.tx.req.get.valid := txreq.valid
  io.icn.tx.req.get.bits := txreq.bits.asTypeOf(io.icn.tx.req.get.bits)
  txreq.ready := io.icn.tx.req.get.ready
  private val txdat = Wire(Decoupled(new DataFlit))
  io.icn.tx.data.get.valid := txdat.valid
  io.icn.tx.data.get.bits := txdat.bits.asTypeOf(io.icn.tx.data.get.bits)
  txdat.ready := io.icn.tx.data.get.ready
  private val txrsp = Wire(Decoupled(new RespFlit))
  io.icn.tx.resp.get.valid := txrsp.valid
  io.icn.tx.resp.get.bits := txrsp.bits.asTypeOf(io.icn.tx.resp.get.bits)
  txrsp.ready := io.icn.tx.resp.get.ready
  private val rxrsp = Wire(Decoupled(new RespFlit))
  rxrsp.valid := io.icn.rx.resp.get.valid
  rxrsp.bits := io.icn.rx.resp.get.bits.asTypeOf(rxrsp.bits)
  io.icn.rx.resp.get.ready := rxrsp.ready
  private val rxdat = Wire(Decoupled(new DataFlit))
  rxdat.valid := io.icn.rx.data.get.valid
  rxdat.bits := io.icn.rx.data.get.bits.asTypeOf(rxdat.bits)
  io.icn.rx.data.get.ready := rxdat.ready

  private val state = RegInit(MachineState.IDLE)
  private val nextState = WireInit(MachineState.IDLE)
  private val task = RegInit(0.U.asTypeOf(new TaskBundle(tlParams)))
  private val rspDBID = RegInit(0.U(rxrsp.bits.DBID.getWidth.W))
  private val rspSrcID = RegInit(0.U(niw.W))

  when(io.alloc.fire) {
    task := io.alloc.bits
  }

  nextState := state
  switch(state) {
    is(MachineState.IDLE) {
      when(io.alloc.fire) {
        nextState := MachineState.SEND_REQ
      }

      assert(!txreq.fire)
      assert(!rxdat.fire)
      assert(!rxrsp.fire)
    }

    is(MachineState.SEND_REQ) {
      when(txreq.fire) {
        when(task.opcode === AOpcode.Get) {
          nextState := MachineState.RECV_RECEIPT
        }.otherwise {
          nextState := MachineState.RECV_RSP
          assert(task.opcode === AOpcode.PutFullData || task.opcode === AOpcode.PutPartialData)
        }
      }
    }

    is(MachineState.RECV_RECEIPT) {
      when(rxrsp.fire) {
        assert(rxrsp.bits.Opcode === RspOpcode.ReadReceipt)

        nextState := MachineState.RECV_DAT
      }
    }

    is(MachineState.RECV_DAT) {
      when(rxdat.fire) {
        assert(rxdat.bits.Opcode === DatOpcode.CompData)

        nextState := MachineState.RETURN_DAT
        task.data := rxdat.bits.Data
      }
    }

    is(MachineState.RECV_RSP) {
      val rspIsDBID = rxrsp.bits.Opcode === RspOpcode.DBIDResp
      val rspIsDBIDOrd  = rxrsp.bits.Opcode === RspOpcode.DBIDRespOrd

      when(rxrsp.fire) {
        assert(rspIsDBID || rspIsDBIDOrd)
        assert(rxrsp.bits.RespErr === RespErr.NormalOkay, "TODO: handle error")

        rspDBID := rxrsp.bits.DBID
        rspSrcID := rxrsp.bits.SrcID

        nextState := MachineState.SEND_DAT
      }
    }

    is(MachineState.SEND_DAT) {
      when(txdat.fire) {
        nextState := MachineState.RECV_CMP
      }
    }

    is(MachineState.RECV_CMP) {
      when(rxrsp.fire) {
        assert(rxrsp.bits.Opcode === RspOpcode.Comp)

        nextState := MachineState.SEND_ACK
      }
    }

    is(MachineState.SEND_ACK) {
      when(txrsp.fire) {
        nextState := MachineState.RETURN_ACK
      }
    }

    is(MachineState.RETURN_DAT) {
      when(io.tld.fire) {
        nextState := MachineState.IDLE
      }
    }

    is(MachineState.RETURN_ACK) {
      when(io.tld.fire) {
        nextState := MachineState.IDLE
      }
    }
  }
  state := nextState

  txreq.valid := state === MachineState.SEND_REQ && io.issueReqEn
  txreq.bits := DontCare
  txreq.bits.Addr := task.address
  txreq.bits.Opcode := Mux(task.opcode === AOpcode.Get, ReqOpcode.ReadNoSnp, ReqOpcode.WriteNoSnpPtl)
  txreq.bits.TxnID := io.id
//  txreq.bits.AllowRetry := false.B
  txreq.bits.ExpCompAck := task.opcode =/= AOpcode.Get
  txreq.bits.MemAttr := MemAttr(allocate = false.B, cacheable = false.B, device = true.B, ewa = false.B /* EAW can take any value for ReadNoSnp/WriteNoSnp* */).asUInt
  txreq.bits.Size := task.size
  txreq.bits.Order := Mux(task.opcode === AOpcode.Get, Order.EndpointOrder, Order.OWO)

  private val chiDataBytes = p(ZJParametersKey).dataBits / 8
  private val tlDataBytes = tlParams.dataBits / 8
  private val segNum = chiDataBytes / tlDataBytes
  private val segIdx = if(segNum > 1) task.address(log2Ceil(chiDataBytes) - 1, log2Ceil(tlDataBytes)) else 0.U
  private val maskVec = Wire(Vec(segNum, UInt(tlDataBytes.W)))
  maskVec.zipWithIndex.foreach({ case (a, b) => a := Mux(b.U === segIdx, task.mask, 0.U) })
  txdat.valid := state === MachineState.SEND_DAT
  txdat.bits := DontCare
  txdat.bits.DBID := DontCare
  txdat.bits.TgtID := rspSrcID
  txdat.bits.BE := maskVec.asUInt
  txdat.bits.Data := Fill(segNum, task.data)
  txdat.bits.Opcode := DatOpcode.NonCopyBackWriteData
//  txdat.bits.CCID := task.address(log2Ceil(chiDataBytes), log2Ceil(chiDataBytes) - 2 + 1)
  txdat.bits.DataID := task.address(5, 4)
  txdat.bits.Resp := 0.U
  txdat.bits.TxnID := rspDBID

  txrsp.valid := state === MachineState.SEND_ACK && io.issueAckEn
  txrsp.bits := DontCare
  txrsp.bits.TgtID := rspSrcID
  txrsp.bits.TxnID := rspDBID
  txrsp.bits.Opcode := RspOpcode.CompAck

  io.tld.valid := state === MachineState.RETURN_DAT || state === MachineState.RETURN_ACK
  io.tld.bits := DontCare
  io.tld.bits.data := Mux(state === MachineState.RETURN_DAT, task.data, 0.U)
  io.tld.bits.corrupt := false.B
  io.tld.bits.opcode := Mux(state === MachineState.RETURN_DAT, DOpcode.AccessAckData, DOpcode.AccessAck)
  io.tld.bits.param := DontCare
  io.tld.bits.sink := io.id
  io.tld.bits.source := task.source
  io.tld.bits.size := 3.U // 2^3 = 8 bytes

  io.status.state := state
  io.status.nextState := nextState
  io.status.address := task.address

  rxrsp.ready := true.B
  rxdat.ready := true.B

  LeakChecker(io.status.state =/= MachineState.IDLE, io.status.state === MachineState.IDLE, Some("machine_valid"), 10000)
}
