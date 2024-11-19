package zhujiang.tilelink

import chisel3._
import chisel3.util._

abstract class BaseTLULPeripheral(tlParams: TilelinkParams) extends Module {
  def regSeq: Seq[(String, Data, Data, Int, Option[UInt], Option[UInt])] //ReadSrc, WriteDst, Address, Write Mask, Read Mask
  val tls = IO(Flipped(new TLULBundle(tlParams)))
  val myclock = WireInit(clock)
  val myreset = WireInit(reset)

  private val wen = tls.a.fire && (tls.a.bits.opcode === AOpcode.PutFullData || tls.a.bits.opcode === AOpcode.PutPartialData)
  private val wdata = tls.a.bits.data
  private val wmask = Mux(tls.a.bits.opcode === AOpcode.PutFullData, Fill(tls.a.bits.mask.getWidth, true.B), tls.a.bits.mask)
  private val ren = tls.a.fire && tls.a.bits.opcode === AOpcode.Get
  private val addr = tls.a.bits.address
  private val respValid = wen || ren
  when(tls.a.valid) {
    assert(respValid)
  }

  private val accessPipe = Module(new Queue(new DFlit(tlParams), entries = 1, pipe = true))
  tls.a.ready := accessPipe.io.enq.ready
  accessPipe.io.enq.valid := respValid
  accessPipe.io.enq.bits := DontCare
  accessPipe.io.enq.bits.opcode := Mux(ren, DOpcode.AccessAckData, DOpcode.AccessAck)
  accessPipe.io.enq.bits.source := tls.a.bits.source
  tls.d <> accessPipe.io.deq

  def genWriteMap(): Map[String, Bool] = {
    require(tlParams.dataBits == 64)
    withClockAndReset(myclock, myreset) {
      val pregSeq = for((name, read, write, addr, writeMask, readMask) <- regSeq) yield {
        val width = read.getWidth
        require(width <= tlParams.dataBits)
        val regSize = if(width == 8) 0
        else if(width == 16) 1
        else if(width == 32) 2
        else if(width == 64) 3
        else 4
        require(regSize < 4)
        val checkAlignMask = (0x1L << regSize) - 1
        require((addr & checkAlignMask) == 0)
        val maxByteIdx = addr + (width / 8) - 1
        (name, read, write, addr.U(tlParams.addrBits.W), writeMask.getOrElse(Fill(width, true.B)), readMask.getOrElse(Fill(width, true.B)), regSize, maxByteIdx)
      }
      val maxByte = pregSeq.map(r => r._8).max + 1
      val busBytes = tlParams.dataBits / 8
      val readMatrixRow = maxByte / busBytes
      val readMatrix = Wire(Vec(readMatrixRow, Vec(busBytes, UInt(8.W))))
      readMatrix := 0.U.asTypeOf(readMatrix)
      dontTouch(readMatrix)

      val addrMatchVec = pregSeq.map(elm => elm._4(tlParams.addrBits - 1, log2Ceil(busBytes)) === addr(tlParams.addrBits - 1, log2Ceil(busBytes)))
      val addrMatch = WireInit(Cat(addrMatchVec).orR)
      val readLegalData = WireInit(readMatrix(addr(log2Ceil(readMatrixRow * busBytes) - 1, log2Ceil(busBytes))).asUInt)
      dontTouch(addrMatch)
      dontTouch(readLegalData)
      accessPipe.io.enq.bits.data := Mux(addr < readMatrixRow.U, readLegalData, 0.U)
      accessPipe.io.enq.bits.corrupt := !addrMatch

      (for(idx <- regSeq.indices) yield {
        val target = pregSeq(idx)
        val name = target._1
        val src = target._2
        val dst = target._3
        val regAddr = target._4
        val regWmask = target._5
        val regRmask = target._6
        val size = target._7
        val lanes = 1 << size
        val width = 8 << size
        val wdataVec = wdata.asTypeOf(Vec(wdata.getWidth / width, UInt(width.W)))
        val wBitsMask = Cat(Seq.tabulate(wdata.getWidth / 8)(i => Fill(8, wmask(i))).reverse)
        val wmaskVec = wBitsMask.asTypeOf(Vec(wdata.getWidth / width, UInt(width.W)))
        val segIdx = if(size == 0) regAddr(2, 0)
        else if(size == 1) regAddr(2, 1)
        else if(size == 2) regAddr(2)
        else 0.U

        val rmaskByLanes = regRmask.asTypeOf(Vec(lanes, UInt(8.W)))
        val srcDataLanes = src.asTypeOf(Vec(lanes, UInt(8.W)))
        for(l <- srcDataLanes.indices) {
          val laneAddr = regAddr + l.U
          val row = laneAddr(log2Ceil(readMatrixRow * busBytes) - 1, log2Ceil(busBytes))
          val off = laneAddr(log2Ceil(busBytes) - 1, 0)
          readMatrix(row)(off) := rmaskByLanes(l) & srcDataLanes(l)
        }

        val finalWdata = WireInit(wdataVec(segIdx))
        val finalWmask = WireInit(regWmask & wmaskVec(segIdx))
        val keepMask = (~finalWmask).asUInt
        val regUpdate = Wire(Bool())
        dontTouch(wdataVec)
        dontTouch(wmaskVec)
        dontTouch(finalWdata)
        dontTouch(finalWmask)
        dontTouch(regUpdate)
        wdataVec.suggestName(s"${name}_write_data_vec")
        wmaskVec.suggestName(s"${name}_write_mask_vec")
        finalWdata.suggestName(s"${name}_write_data")
        finalWmask.suggestName(s"${name}_write_mask")
        regUpdate.suggestName(s"${name}_write_en")
        regUpdate := wen && addr(tlParams.addrBits - 1, 3) === regAddr(tlParams.addrBits - 1, 3)
        when(regUpdate) {
          dst := ((finalWmask & finalWdata) | (keepMask & src.asUInt)).asTypeOf(dst)
        }
        name -> regUpdate
      }).toMap
    }
  }
}
