package zhujiang.device.bridge.axi

import chisel3._
import chisel3.util._
import xs.utils.sram.SRAMTemplate
import org.chipsalliance.cde.config.Parameters
import xs.utils.queue.MimoQueue
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper}
import zhujiang.axi.{AxiParams, WFlit}
import zhujiang.{ZJBundle, ZJModule}
import zhujiang.chi.{DatOpcode, DataFlit}

class AxiDataBufferCtrlEntry(bufferSize: Int)(implicit p: Parameters) extends ZJBundle {
  val buf = Vec(512 / dw, UInt(log2Ceil(bufferSize).W))
  val dataIdOffset = UInt(2.W)
  val recvMax = UInt(3.W)
  val recvCnt = UInt(3.W)
}

class AxiDataBufferAllocReq(ctrlSize: Int) extends Bundle {
  val idxOH = UInt(ctrlSize.W)
  val size = UInt(3.W)
  val dataIdOffset = UInt(2.W)
}

class AxiDataBufferReadReq(axiParams: AxiParams, bufferSize: Int) extends Bundle {
  val set = UInt(log2Ceil(bufferSize).W)
  val flit = new WFlit(axiParams)
}

class AxiDataBufferTxReq(axiParams: AxiParams, ctrlSize: Int) extends Bundle {
  val idxOH = UInt(ctrlSize.W)
  val flit = new WFlit(axiParams)
}

class AxiDataBufferFreelist(ctrlSize: Int, bufferSize: Int)(implicit p: Parameters) extends ZJModule with HasCircularQueuePtrHelper {
  private class AxiDataBufferFreelistPtr extends CircularQueuePtr[AxiDataBufferFreelistPtr](bufferSize)

  private object AxiDataBufferFreelistPtr {
    def apply(f: Bool, v: UInt): AxiDataBufferFreelistPtr = {
      val ptr = Wire(new AxiDataBufferFreelistPtr)
      ptr.flag := f
      ptr.value := v
      ptr
    }
  }

  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new AxiDataBufferAllocReq(ctrlSize)))
    val resp = Valid(new AxiDataBufferCtrlEntry(bufferSize))
    val release = Input(Valid(new AxiDataBufferCtrlEntry(bufferSize)))
    val cancel = Input(Valid(new AxiDataBufferCtrlEntry(bufferSize)))
    val idle = Output(Bool())
  })
  private val maxAllocOnce = 512 / dw
  private val freelist = RegInit(VecInit(Seq.tabulate(bufferSize)(_.U(log2Ceil(bufferSize).W))))
  private val headPtr = RegInit(AxiDataBufferFreelistPtr(f = false.B, v = 0.U))
  private val tailPtr = RegInit(AxiDataBufferFreelistPtr(f = true.B, v = 0.U))
  private val availableSlots = RegInit(bufferSize.U(log2Ceil(bufferSize + 1).W))
  private val releaseMergeQueue = Module(new MimoQueue(UInt(log2Ceil(bufferSize).W), maxAllocOnce * 2, maxAllocOnce * 2, maxAllocOnce * 4))
  assert(availableSlots === distanceBetween(tailPtr, headPtr))
  assert(headPtr <= tailPtr)

  when(io.req.valid) {
    assert(io.req.bits.size <= 6.U)
  }
  private val dataWidthInBytesShift = log2Ceil(dw / 8)
  private val allocMoreThanOne = io.req.bits.size > dataWidthInBytesShift.U
  private val reqNum = Mux(allocMoreThanOne, 1.U(3.W) << (io.req.bits.size - dataWidthInBytesShift.U), 1.U).asUInt
  io.req.ready := availableSlots >= reqNum
  for(i <- io.resp.bits.buf.indices) {
    io.resp.bits.buf(i) := freelist((headPtr + i.U).value)
  }
  io.resp.valid := io.req.fire
  io.resp.bits.recvMax := reqNum - 1.U
  io.resp.bits.recvCnt := 0.U
  io.resp.bits.dataIdOffset := io.req.bits.dataIdOffset
  io.idle := headPtr.value === tailPtr.value && headPtr.flag =/= tailPtr.flag

  releaseMergeQueue.io.enq.take(maxAllocOnce).zipWithIndex.foreach({ case(e, i) =>
    e.valid := io.release.valid && i.U <= io.release.bits.recvMax
    e.bits := io.release.bits.buf(i)
    when(e.valid) {
      assert(e.ready)
    }
  })
  releaseMergeQueue.io.enq.drop(maxAllocOnce).zipWithIndex.foreach({ case(e, i) =>
    e.valid := io.cancel.valid && i.U <= io.cancel.bits.recvMax
    e.bits := io.cancel.bits.buf(i)
    when(e.valid) {
      assert(e.ready)
    }
  })
  releaseMergeQueue.io.deq.foreach(_.ready := true.B)

  private val allocNum = Mux(io.req.fire, reqNum, 0.U)
  private val relNum = releaseMergeQueue.io.count
  private val relValid = releaseMergeQueue.io.count.orR
  when(io.req.fire || relValid) {
    headPtr := headPtr + allocNum
    tailPtr := tailPtr + relNum
    availableSlots := (availableSlots +& relNum) - allocNum
  }
  private val releaseMatchSeq = for(i <- releaseMergeQueue.io.deq.indices) yield (releaseMergeQueue.io.deq(i).valid, (tailPtr + i.U).value)
  private val releaseEntrySeq = releaseMergeQueue.io.deq.map(_.bits)
  for(idx <- freelist.indices) {
    val releaseMatch = releaseMatchSeq.map(elm => elm._1 && elm._2 === idx.U)
    when(Cat(releaseMatch).orR) {
      freelist(idx) := Mux1H(releaseMatch, releaseEntrySeq)
    }
  }
  assert(releaseMergeQueue.io.count <= (maxAllocOnce * 2).U)
}

class AxiDataBufferRam(axiParams: AxiParams, bufferSize: Int)(implicit p: Parameters) extends ZJModule {
  require(axiParams.dataBits == dw)
  val io = IO(new Bundle {
    val writeData = Flipped(Decoupled(new DataFlit))
    val readDataReq = Flipped(Decoupled(new AxiDataBufferReadReq(axiParams, bufferSize)))
    val readDataResp = Decoupled(new WFlit(axiParams))
  })
  private val maskRam = Mem(bufferSize, UInt(bew.W))
  private val dataRam = Module(new SRAMTemplate(
    gen = UInt(dw.W),
    set = bufferSize,
    singlePort = false,
    suffix = "_sn_db",
    hasMbist = hasMbist
  ))
  when(io.writeData.valid) {
    maskRam.write(io.writeData.bits.TxnID(log2Ceil(bufferSize) - 1, 0), io.writeData.bits.BE)
  }

  dataRam.io.w.req.valid := io.writeData.valid
  dataRam.io.w.req.bits.setIdx := io.writeData.bits.TxnID(log2Ceil(bufferSize) - 1, 0)
  dataRam.io.w.req.bits.data := io.writeData.bits.Data.asTypeOf(dataRam.io.w.req.bits.data)
  io.writeData.ready := dataRam.io.w.req.ready

  private val readStage1Pipe = Module(new Queue(new WFlit(axiParams), entries = 1, pipe = true))
  private val readStage2Pipe = Module(new Queue(new WFlit(axiParams), entries = 1, pipe = true))

  io.readDataReq.ready := readStage1Pipe.io.enq.ready && dataRam.io.r.req.ready

  readStage1Pipe.io.enq.valid := io.readDataReq.valid && dataRam.io.r.req.ready
  readStage1Pipe.io.enq.bits := io.readDataReq.bits.flit
  dataRam.io.r.req.valid := io.readDataReq.valid && readStage1Pipe.io.enq.ready
  dataRam.io.r.req.bits.setIdx := io.readDataReq.bits.set

  readStage1Pipe.io.deq.ready := readStage2Pipe.io.enq.ready
  readStage2Pipe.io.enq.valid := readStage1Pipe.io.deq.valid
  readStage2Pipe.io.enq.bits := readStage1Pipe.io.deq.bits
  readStage2Pipe.io.enq.bits.strb := maskRam.read(RegEnable(io.readDataReq.bits.set, io.readDataReq.fire))
  readStage2Pipe.io.enq.bits.data := dataRam.io.r.resp.data.asUInt

  io.readDataResp <> readStage2Pipe.io.deq
}

class AxiDataBuffer(axiParams: AxiParams, ctrlSize: Int, bufferSize: Int)(implicit p: Parameters) extends ZJModule {
  val io = IO(new Bundle {
    val alloc = Flipped(Decoupled(new AxiDataBufferAllocReq(ctrlSize)))
    val icn = Flipped(Decoupled(new DataFlit))
    val toCmDat = Output(Valid(new DataFlit))
    val fromCmDat = Flipped(Decoupled(new AxiDataBufferTxReq(axiParams, ctrlSize)))
    val axi = Decoupled(new WFlit(axiParams))
  })
  private val dataBuffer = Module(new AxiDataBufferRam(axiParams, bufferSize))
  private val freelist = Module(new AxiDataBufferFreelist(ctrlSize, bufferSize))
  private val ctrlValidVec = RegInit(VecInit(Seq.fill(ctrlSize)(false.B)))
  private val ctrlInfoVec = Reg(Vec(ctrlSize, new AxiDataBufferCtrlEntry(bufferSize)))
  private val txReqValid = RegInit(false.B)
  private val txReqBitsReg = RegEnable(io.fromCmDat.bits, io.fromCmDat.fire)
  private val ctrlSelReg = RegEnable(Mux1H(io.fromCmDat.bits.idxOH, ctrlInfoVec), io.fromCmDat.fire)
  private val txReqCntReg = Reg(UInt(8.W))
  private val rxDatValidReg = RegNext(io.icn.valid, false.B)
  private val rxDatBitsReg = RegEnable(io.icn.bits, io.icn.valid)
  private val icnSelCtrlDelay = ctrlInfoVec(rxDatBitsReg.TxnID(log2Ceil(ctrlSize) - 1, 0))

  io.alloc.ready := freelist.io.req.ready
  freelist.io.req.valid := io.alloc.valid
  freelist.io.req.bits := io.alloc.bits
  freelist.io.release.valid := dataBuffer.io.readDataReq.fire && ctrlSelReg.recvMax === txReqCntReg
  freelist.io.release.bits := ctrlSelReg
  freelist.io.cancel.valid := rxDatValidReg && rxDatBitsReg.Opcode === DatOpcode.WriteDataCancel
  freelist.io.cancel.bits := icnSelCtrlDelay

  for(idx <- ctrlValidVec.indices) {
    when(freelist.io.resp.valid && io.alloc.bits.idxOH(idx)) {
      ctrlValidVec(idx) := true.B
    }.elsewhen(freelist.io.release.valid && txReqBitsReg.idxOH(idx)) {
      ctrlValidVec(idx) := false.B
    }.elsewhen(freelist.io.cancel.valid && rxDatBitsReg.TxnID(log2Ceil(ctrlSize) - 1, 0) === idx.U) {
      ctrlValidVec(idx) := false.B
    }

    when(freelist.io.resp.valid && io.alloc.bits.idxOH(idx)) {
      ctrlInfoVec(idx).buf := freelist.io.resp.bits.buf
      ctrlInfoVec(idx).recvMax := freelist.io.resp.bits.recvMax
      ctrlInfoVec(idx).dataIdOffset := freelist.io.resp.bits.dataIdOffset
    }

    when(freelist.io.resp.valid && io.alloc.bits.idxOH(idx)) {
      ctrlInfoVec(idx).recvCnt := freelist.io.resp.bits.recvCnt
    }.elsewhen(io.icn.fire && io.icn.bits.TxnID === idx.U) {
      assert(ctrlInfoVec(idx).recvCnt <= ctrlInfoVec(idx).recvMax)
      ctrlInfoVec(idx).recvCnt := ctrlInfoVec(idx).recvCnt + 1.U
    }
  }

  private val allowNewTx = freelist.io.release.valid || !txReqValid
  io.fromCmDat.ready := allowNewTx
  when(allowNewTx) {
    txReqValid := io.fromCmDat.valid
  }
  when(io.fromCmDat.fire) {
    txReqCntReg := 0.U
  }.elsewhen(dataBuffer.io.readDataReq.fire) {
    txReqCntReg := txReqCntReg + 1.U
  }

  private val icnSelCtrl = ctrlInfoVec(io.icn.bits.TxnID(log2Ceil(ctrlSize) - 1, 0))
  dataBuffer.io.writeData.valid := io.icn.valid && io.icn.bits.Opcode =/= DatOpcode.WriteDataCancel
  io.icn.ready := dataBuffer.io.writeData.ready
  dataBuffer.io.writeData.bits := io.icn.bits
  private val _dataID = io.icn.bits.DataID - icnSelCtrl.dataIdOffset
  private val bufIdx = if(dw == 128) {
    _dataID(1, 0)
  } else if(dw == 256) {
    _dataID(1, 1)
  } else {
    0.U
  }
  when(io.icn.fire && (io.icn.bits.Opcode === DatOpcode.NonCopyBackWriteData || io.icn.bits.Opcode === DatOpcode.NCBWrDataCompAck)) {
    assert(io.icn.bits.DataID >= icnSelCtrl.dataIdOffset)
  }
  dataBuffer.io.writeData.bits.TxnID := icnSelCtrl.buf(bufIdx)

  dataBuffer.io.readDataReq.valid := txReqValid
  dataBuffer.io.readDataReq.bits.set := ctrlSelReg.buf(txReqCntReg(log2Ceil(ctrlSelReg.buf.length) - 1, 0))
  dataBuffer.io.readDataReq.bits.flit := txReqBitsReg.flit
  dataBuffer.io.readDataReq.bits.flit.last := txReqCntReg === ctrlSelReg.recvMax

  io.axi <> dataBuffer.io.readDataResp

  io.toCmDat.valid := rxDatValidReg && (icnSelCtrlDelay.recvCnt === (icnSelCtrlDelay.recvMax + 1.U) || rxDatBitsReg.Opcode === DatOpcode.WriteDataCancel)
  io.toCmDat.bits := rxDatBitsReg
}
