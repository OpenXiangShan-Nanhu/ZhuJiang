package zhujiang.device.dma

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper}
import zhujiang.axi.{AxiParams, WFlit, RFlit}
import zhujiang.{ZJBundle, ZJModule}
import zhujiang.chi.DataFlit
import xs.utils.sram.DualPortSramTemplate
import zhujiang.chi.ReqOpcode
import zhujiang.chi.DatOpcode
import xijiang._

class ChiDataBufferFreelist(ctrlSize: Int, bufferSize: Int)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private class ChiDataBufferFreelistPtr extends CircularQueuePtr[ChiDataBufferFreelistPtr](bufferSize)

  private object ChiDataBufferFreelistPtr {
    def apply(f: Bool, v: UInt): ChiDataBufferFreelistPtr = {
      val ptr = Wire(new ChiDataBufferFreelistPtr)
      ptr.flag := f
      ptr.value := v
      ptr
    }
  }

  val io = IO(new Bundle {
    val req = Flipped(Decoupled(Bool()))
    val resp = Valid(new DataBufferAlloc(bufferSize))
    val release = Input(Valid(UInt(log2Ceil(bufferSize).W)))
    val idle = Output(Bool())
  })
  private val freelist = RegInit(VecInit(Seq.tabulate(bufferSize)(_.U(log2Ceil(bufferSize).W))))
  private val headPtr = RegInit(ChiDataBufferFreelistPtr(f = false.B, v = 0.U))
  private val tailPtr = RegInit(ChiDataBufferFreelistPtr(f = true.B, v = 0.U))
  private val availableSlots = RegInit(bufferSize.U(log2Ceil(bufferSize + 1).W))
  assert(availableSlots === distanceBetween(tailPtr, headPtr))

  private val dataWidthInBytesShift = log2Ceil(dw / 8)
  private val allocMoreThanOne = io.req.bits

  private val reqNum = Mux(allocMoreThanOne, 2.U, 1.U)
  io.req.ready := availableSlots >= reqNum

  for(i <- io.resp.bits.buf.indices) {
    io.resp.bits.buf(i) := freelist((headPtr + i.U).value)
  }
  io.resp.valid := io.req.fire
  io.resp.bits.num := reqNum - 1.U
  io.idle := headPtr.value === tailPtr.value && headPtr.flag =/= tailPtr.flag

  private val allocNum = Mux(io.req.fire, reqNum, 0.U)
  private val relNum = Mux(io.release.valid, 1.U, 0.U)

  when(io.req.fire || io.release.valid) {
    headPtr := headPtr + allocNum
    tailPtr := tailPtr + relNum
    availableSlots := (availableSlots +& relNum) - allocNum
  }
  freelist(tailPtr.value) := Mux(io.release.valid, io.release.bits, freelist(tailPtr.value))
}

class ChiDataBufferRdRam(axiParams: AxiParams, bufferSize: Int)(implicit p: Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val writeDataReq = Flipped(Decoupled(new writeRdDataBuffer(bufferSize)))
    val readDataReq = Flipped(Decoupled(new readRdDataBuffer(bufferSize, axiParams)))
    val readDataResp = Decoupled(new RFlit(axiParams))
    val relSet = Valid(UInt(log2Ceil(bufferSize).W))
  })

  private val dataRam = Module(new DualPortSramTemplate(
    gen = UInt(dw.W),
    set = bufferSize,
    bypassWrite = false,
    suffix = "_rni_rdb",
    hasMbist = hasMbist
  ))

  private val wrRamPipe = Pipe(io.writeDataReq.fire, io.writeDataReq.bits)
  private val readRamStage1Pipe = Module(new Queue(new readRdDataBuffer(bufferSize, axiParams), entries = 1, pipe = false))
  private val readRamStage2Pipe = Module(new Queue(new respDataBuffer(bufferSize), entries = 2, pipe = false))
  private val rFlitBdl = WireInit(0.U.asTypeOf(new RFlit(axiParams)))


  dataRam.io.wreq.valid := wrRamPipe.valid
  dataRam.io.wreq.bits.data(0) := wrRamPipe.bits.data
  dataRam.io.wreq.bits.addr := wrRamPipe.bits.set

  readRamStage1Pipe.io.enq.valid := io.readDataReq.valid
  readRamStage1Pipe.io.enq.bits.id := io.readDataReq.bits.id
  readRamStage1Pipe.io.enq.bits.last := io.readDataReq.bits.last
  readRamStage1Pipe.io.enq.bits.resp := io.readDataReq.bits.resp
  readRamStage1Pipe.io.enq.bits.set := io.readDataReq.bits.set
  readRamStage1Pipe.io.enq.bits.originId := io.readDataReq.bits.originId
  readRamStage1Pipe.io.enq.bits.streamId := io.readDataReq.bits.streamId
  readRamStage1Pipe.io.deq.ready := readRamStage2Pipe.io.enq.ready

  readRamStage2Pipe.io.enq.valid := readRamStage1Pipe.io.deq.valid
  readRamStage2Pipe.io.enq.bits.id := readRamStage1Pipe.io.deq.bits.id
  readRamStage2Pipe.io.enq.bits.last := readRamStage1Pipe.io.deq.bits.last
  readRamStage2Pipe.io.enq.bits.resp := readRamStage1Pipe.io.deq.bits.resp
  readRamStage2Pipe.io.enq.bits.data := dataRam.io.rresp.bits.asUInt
  readRamStage2Pipe.io.enq.bits.streamId := readRamStage1Pipe.io.deq.bits.streamId
  readRamStage2Pipe.io.deq.ready := io.readDataResp.ready

  dataRam.io.rreq.valid := io.readDataReq.valid & readRamStage1Pipe.io.enq.ready
  dataRam.io.rreq.bits := io.readDataReq.bits.set

  rFlitBdl := 0.U.asTypeOf(rFlitBdl)
  rFlitBdl.id := readRamStage2Pipe.io.deq.bits.id
  rFlitBdl.data := readRamStage2Pipe.io.deq.bits.data
  rFlitBdl.resp := readRamStage2Pipe.io.deq.bits.resp
  rFlitBdl.last := readRamStage2Pipe.io.deq.bits.last
  rFlitBdl.user := readRamStage2Pipe.io.deq.bits.streamId

  io.readDataReq.ready := readRamStage1Pipe.io.enq.ready
  io.readDataResp.valid := readRamStage2Pipe.io.deq.valid
  io.readDataResp.bits := rFlitBdl
  io.writeDataReq.ready := true.B
  io.relSet.valid := readRamStage1Pipe.io.deq.fire
  io.relSet.bits := readRamStage1Pipe.io.deq.bits.set

  assert(dataRam.io.wreq.ready, "if ready is false, please use Queue, Do not use Pipe")
}

class DataBufferForRead(node: Node)(implicit p: Parameters) extends ZJModule {
  private val axiParams = node.axiDevParams.get.extPortParams.getOrElse(AxiParams())
  private val axiParamRUser = axiParams.copy(userBits = log2Ceil(node.outstanding), idBits = log2Ceil(node.outstanding))
  val io = IO(new Bundle {
    val alloc = Flipped(Decoupled(Bool()))
    val axiR = Decoupled(new RFlit(axiParamRUser))
    val wrDB = Flipped(Decoupled(new writeRdDataBuffer(node.outstanding)))
    val rdDB = Flipped(Decoupled(new readRdDataBuffer(node.outstanding, axiParams)))
    val allocRsp = Valid(new DataBufferAlloc(node.outstanding))
  })

  private val dataBuffer = Module(new ChiDataBufferRdRam(axiParamRUser, node.outstanding))
  private val freelist = Module(new ChiDataBufferFreelist(node.outstanding, node.outstanding))

  freelist.io.req.valid := io.alloc.valid
  freelist.io.req.bits := io.alloc.bits
  freelist.io.release.valid := dataBuffer.io.relSet.valid
  freelist.io.release.bits := dataBuffer.io.relSet.bits

  dataBuffer.io.readDataReq <> io.rdDB
  dataBuffer.io.writeDataReq <> io.wrDB
  io.axiR <> dataBuffer.io.readDataResp

  io.alloc.ready := freelist.io.req.ready
  io.allocRsp.valid := freelist.io.resp.valid
  io.allocRsp.bits := freelist.io.resp.bits
}

class ChiDataBufferWrRam(bufferSize: Int)(implicit p: Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val writeDataReq = Flipped(Decoupled(new writeWrDataBuffer(bufferSize)))
    val readDataReq = Flipped(Decoupled(new readWrDataBuffer(bufferSize)))
    val readDataResp = Decoupled(new DataFlit)
    val relSet = Valid(UInt(log2Ceil(bufferSize).W))
  })

  private val dataRam = Module(new DualPortSramTemplate(
    gen = UInt(8.W),
    set = bufferSize,
    way = bew,
    hasMbist = hasMbist,
    suffix = "_rni_wdb"
  ))

  private val maskRam = RegInit(0.U.asTypeOf(Vec(bufferSize, UInt(bew.W))))

  private val wrRamPipe = Pipe(io.writeDataReq.fire, io.writeDataReq.bits)
  private val rdRamQ = Module(new Queue(new readWrDataBuffer(bufferSize), entries = 2, flow = false, pipe = false))
  private val readRamState1Pipe = Module(new Queue(new readWrDataBuffer(bufferSize), entries = 1, pipe = true))
  private val readRamStage2Pipe = Module(new Queue(new DataFlit, entries = 1, pipe = true))
  private val wDataVec = Wire(Vec(bew, UInt(8.W)))

  maskRam.zipWithIndex.foreach {
    case (m, i) =>
      when(wrRamPipe.valid & wrRamPipe.bits.set === i.U) {
        m := wrRamPipe.bits.mask | m
      }.elsewhen(readRamState1Pipe.io.deq.fire & readRamState1Pipe.io.deq.bits.set === i.U) {
        m := 0.U
      }
  }

  wDataVec := wrRamPipe.bits.data.asTypeOf(wDataVec)

  dataRam.io.wreq.valid := wrRamPipe.valid
  dataRam.io.wreq.bits.data := wDataVec
  dataRam.io.wreq.bits.mask.get := wrRamPipe.bits.mask
  dataRam.io.wreq.bits.addr := wrRamPipe.bits.set

  dataRam.io.rreq.valid := rdRamQ.io.deq.valid & readRamState1Pipe.io.enq.ready
  dataRam.io.rreq.bits := rdRamQ.io.deq.bits.set

  rdRamQ.io.enq <> io.readDataReq

  readRamState1Pipe.io.enq <> rdRamQ.io.deq
  readRamState1Pipe.io.deq.ready := readRamStage2Pipe.io.enq.ready

  readRamStage2Pipe.io.enq.valid := readRamState1Pipe.io.deq.valid
  readRamStage2Pipe.io.enq.bits := 0.U.asTypeOf(readRamStage2Pipe.io.enq.bits)
  readRamStage2Pipe.io.enq.bits.DataID := readRamState1Pipe.io.deq.bits.dataID
  readRamStage2Pipe.io.enq.bits.TxnID := readRamState1Pipe.io.deq.bits.txnID
  readRamStage2Pipe.io.enq.bits.BE := maskRam(readRamState1Pipe.io.deq.bits.set)
  readRamStage2Pipe.io.enq.bits.Opcode := DatOpcode.NonCopyBackWriteData
  readRamStage2Pipe.io.enq.bits.Data := dataRam.io.rresp.bits.asTypeOf(UInt(dw.W))
  readRamStage2Pipe.io.enq.bits.SrcID := 1.U
  readRamStage2Pipe.io.enq.bits.TgtID := readRamState1Pipe.io.deq.bits.tgtId

  io.readDataResp <> readRamStage2Pipe.io.deq
  io.writeDataReq.ready := true.B
  io.relSet.valid := readRamState1Pipe.io.deq.fire
  io.relSet.bits := readRamState1Pipe.io.deq.bits.set

  assert(dataRam.io.wreq.ready, "if ready is false, please use Queue, Do not use Pipe")
}

class DataBufferForWrite(bufferSize: Int, ctrlSize: Int)(implicit p: Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val alloc = Flipped(Decoupled(Bool()))
    val dData = Decoupled(new DataFlit)
    val wrDB = Flipped(Decoupled(new writeWrDataBuffer(bufferSize)))
    val rdDB = Flipped(Decoupled(new readWrDataBuffer(bufferSize)))
    val allocRsp = Valid(new DataBufferAlloc(bufferSize))
  })

  private val freelist = Module(new ChiDataBufferFreelist(ctrlSize, bufferSize))
  private val dataBuffer = Module(new ChiDataBufferWrRam(bufferSize))

  freelist.io.req.valid := io.alloc.valid
  freelist.io.req.bits := io.alloc.bits
  freelist.io.release.valid := dataBuffer.io.relSet.valid
  freelist.io.release.bits := dataBuffer.io.relSet.bits

  dataBuffer.io.readDataReq <> io.rdDB
  dataBuffer.io.writeDataReq <> io.wrDB
  io.dData <> dataBuffer.io.readDataResp

  io.alloc.ready := freelist.io.req.ready
  io.allocRsp.valid := freelist.io.resp.valid
  io.allocRsp.bits := freelist.io.resp.bits

}
