package dongjiang.pcu

import dongjiang._
import AtomicOp._
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.perf.HasPerfLogging
import xs.utils.debug.{DomainInfo, HardwareAssertion}


object AtomicOp {
  val width = 4
  val LDADD   = 0x0
  val LDCLR   = 0x1
  val LDEOR   = 0x2
  val LDSET   = 0x3
  val LDSMAX  = 0x4
  val LDSMIN  = 0x5
  val LDUMAX  = 0x6
  val LDUMIN  = 0x7
  val SWAP    = 0x8
  val COMPARE = 0x9
  val NONE    = 0xF
}


class AtomicDataBundle(implicit p: Parameters) extends DJBundle {
  val data    = UInt(256.W)
  val mask    = UInt(32.W)
  val swapFst = Bool() // Only use in compare
}


class APUEntry(implicit p: Parameters) extends DJBundle {
  val op          = UInt(AtomicOp.width.W)
  val atomic      = new AtomicDataBundle()
  val initOff     = Bool()
}

/*
 * !!!!!!!!! This is a highly customizable module for CHI only !!!!!!!!!
 */
class AtomicProcessUnit()(implicit p: Parameters) extends DJModule with HasPerfLogging {
// ------------------------------------------ IO declaration --------------------------------------------- //
  val io = IO(new DJBundle {
    val in            = Flipped(Valid(new DJBundle with HasDBID {
      val op          = UInt(AtomicOp.width.W)
      val data        = UInt(256.W)
      val atomic      = new AtomicDataBundle()
    }))
    val out           = Valid(new DJBundle with HasDBID {
      val data        = UInt(256.W)
    })
  })

  // assert
  HardwareAssertion(io.in.bits.op <= COMPARE.U | !io.in.valid)
  HardwareAssertion(Mux(io.in.bits.op === COMPARE.U, PopCount(io.in.bits.atomic.mask) <= 32.U, PopCount(io.in.bits.atomic.mask) <= 8.U) | !io.in.valid)


// ----------------------------------------- Reg and Wire declaration ------------------------------------ //
  // S1: Parse Input
  val valid_s1_g        = RegNext(io.in.valid)
  val in_s1_g           = RegEnable(io.in.bits, io.in.valid)
  val amoDataVec_s1     = Wire(Vec(32, UInt(8.W)))
  val inDataVec_s1      = Wire(Vec(32, UInt(8.W)))
  val loadDataVec_s1    = Wire(Vec(8,  UInt(8.W)))
  val swapDataVec_s1    = Wire(Vec(16, UInt(8.W)))
  val compDataVec_s1    = Wire(Vec(16, UInt(8.W)))
  val initDataVec_s1    = Wire(Vec(16, UInt(8.W)))
  // S2: Execute
  val valid_s2_g        = RegNext(valid_s1_g)
  // base
  val opcodeOH_s2_g     = RegEnable(UIntToOH(in_s1_g.op), valid_s1_g)
  val inDataVec_s2_g    = RegEnable(inDataVec_s1,         valid_s1_g)
  val dbID_s2_g         = RegEnable(in_s1_g.dbID,         valid_s1_g)
  val firstIdx_s2_g     = Reg(UInt(5.W))
  val lastIdx_s2_g      = Reg(UInt(5.W))
  // execute

  val amoDataVec_s2_g   = RegEnable(amoDataVec_s1,  valid_s1_g)
  val loadDataVec_s2_g  = RegEnable(loadDataVec_s1, valid_s1_g)
  val swapDataVec_s2_g  = RegEnable(swapDataVec_s1, valid_s1_g)
  val compDataVec_s2_g  = RegEnable(compDataVec_s1, valid_s1_g)
  val initDataVec_s2_g  = RegEnable(initDataVec_s1, valid_s1_g)
  val dealData_s2       = WireInit(0.U(128.W))
  // S3: Output
  val valid_s3_g        = RegNext(valid_s2_g)
  val inDataVec_s3_g    = RegEnable(inDataVec_s2_g, valid_s2_g)
  val dbID_s3_g         = RegEnable(dbID_s2_g,      valid_s2_g)
  val firstIdx_s3_g     = RegEnable(firstIdx_s2_g,  valid_s2_g)
  val lastIdx_s3_g      = RegEnable(lastIdx_s2_g,   valid_s2_g)
  val dealData_s3_g     = RegEnable(dealData_s2,    valid_s2_g)
  val outDataVec_s3     = Wire(Vec(32, UInt(8.W)))




// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------ S1: Parse Data Input ------------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Parse Data Input
   */
  val firstByte_s1    = PriorityEncoder(in_s1_g.atomic.mask)
  val bytesNum_s1     = PopCount(in_s1_g.atomic.mask).asUInt
  val halfBytesNum_s1 = (bytesNum_s1 >> 1).asUInt
  amoDataVec_s1       := in_s1_g.atomic.data.asTypeOf(amoDataVec_s1)
  inDataVec_s1        := in_s1_g.data.asTypeOf(inDataVec_s1)
  HardwareAssertion(bytesNum_s1 > 0.U | !valid_s1_g)

  /*
   * Parse loadData
   */
  loadDataVec_s1.zipWithIndex.foreach {
    case(load, i) =>
      load := Mux(i.U < bytesNum_s1, amoDataVec_s1(i.U + firstByte_s1), 0.U)
  }

  /*
   * Parse swapData
   */
  swapDataVec_s1.zipWithIndex.foreach {
    case(swap, i) =>
      when(in_s1_g.op === SWAP.U) {
        swap := Mux(i.U < bytesNum_s1, amoDataVec_s1(i.U + firstByte_s1), 0.U)
      }.elsewhen(in_s1_g.atomic.swapFst) {
        swap := Mux(i.U < halfBytesNum_s1, amoDataVec_s1(i.U + firstByte_s1), 0.U)
      }.otherwise {
        swap := Mux(i.U < halfBytesNum_s1, amoDataVec_s1(i.U + firstByte_s1 + halfBytesNum_s1), 0.U)
      }
  }

  /*
   * Parse compData
   */
  compDataVec_s1.zipWithIndex.foreach {
    case (comp, i) =>
     when(in_s1_g.atomic.swapFst) {
        comp := Mux(i.U < halfBytesNum_s1, amoDataVec_s1(i.U + firstByte_s1 + halfBytesNum_s1), 0.U)
      }.otherwise {
        comp := Mux(i.U < halfBytesNum_s1, amoDataVec_s1(i.U + firstByte_s1), 0.U)
      }
  }

  /*
   * Parse initData
   */
  initDataVec_s1.zipWithIndex.foreach {
    case (init, i) =>
      when(in_s1_g.op === COMPARE.U & in_s1_g.atomic.swapFst) {
        init := Mux(i.U < halfBytesNum_s1, inDataVec_s1(i.U + firstByte_s1 + halfBytesNum_s1), 0.U)
      }.elsewhen(in_s1_g.op === COMPARE.U) {
        init := Mux(i.U < halfBytesNum_s1, inDataVec_s1(i.U + firstByte_s1), 0.U)
      }.otherwise {
        init := Mux(i.U < bytesNum_s1,     inDataVec_s1(i.U + firstByte_s1), 0.U)
      }
  }


// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- S2: Excute Atomic -------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Get Index
   */
  when(valid_s1_g) {
    when(in_s1_g.op === COMPARE.U & in_s1_g.atomic.swapFst) {
      firstIdx_s2_g := firstByte_s1 + halfBytesNum_s1
      lastIdx_s2_g  := firstByte_s1 + bytesNum_s1
    }.elsewhen(in_s1_g.op === COMPARE.U) {
      firstIdx_s2_g := firstByte_s1
      lastIdx_s2_g  := firstByte_s1 + halfBytesNum_s1
    }.otherwise {
      firstIdx_s2_g := firstByte_s1
      lastIdx_s2_g  := firstByte_s1 + bytesNum_s1
    }
  }

  /*
   * Get the absolute value
   */
  val sIntBit_s2_g  = RegEnable(((bytesNum_s1 << 3.U).asUInt - 1.U).asTypeOf(UInt(6.W)), valid_s1_g)
  def getAbs_s2(in: UInt): UInt = {
    val outVec      = Wire(Vec(64, Bool()))
    when(in(sIntBit_s2_g)){
      val temp      = ~(in - 1.U)
      temp.asBools.zipWithIndex.foreach { case(t, i) => outVec(i) := Mux(i.U < sIntBit_s2_g, t, false.B) }
    }.otherwise {
      outVec        := in.asBools
    }
    outVec.asUInt
  }

  /*
   * Execute
   */
  // Get txnData and initData
  val txnData_s2  = loadDataVec_s2_g.asTypeOf(UInt(64.W))
  val initData_s2 = initDataVec_s2_g.asTypeOf(txnData_s2)
  // LDADD
  when(opcodeOH_s2_g(LDADD)) {
    dealData_s2   := txnData_s2 + initData_s2
  // LDCLR
  }.elsewhen(opcodeOH_s2_g(LDCLR)) {
    dealData_s2   := initData_s2 & (~txnData_s2).asUInt
  // LDEOR
  }.elsewhen(opcodeOH_s2_g(LDEOR)) {
    dealData_s2   := initData_s2 ^ txnData_s2
  // LDSET
  }.elsewhen(opcodeOH_s2_g(LDSET)) {
    dealData_s2   := initData_s2 | txnData_s2
  // LDSMAX
  }.elsewhen(opcodeOH_s2_g(LDSMAX)) {
    val txnBigger = Wire(Bool())
    when(txnData_s2(sIntBit_s2_g) === initData_s2(sIntBit_s2_g)) {
      val txnAbs  = Mux(txnData_s2(sIntBit_s2_g), getAbs_s2(txnData_s2), txnData_s2)
      val initAbs = Mux(txnData_s2(sIntBit_s2_g), getAbs_s2(initData_s2), initData_s2)
      txnBigger   := txnAbs > initAbs
    }.otherwise {
      txnBigger   := !txnData_s2(sIntBit_s2_g)
    }
    dealData_s2   := Mux(txnBigger, txnData_s2, initData_s2)
  // LDSMIN
  }.elsewhen(opcodeOH_s2_g(LDSMIN)) {
    val iniBigger = Wire(Bool())
    when(txnData_s2(sIntBit_s2_g) === initData_s2(sIntBit_s2_g)) {
      val txnAbs  = Mux(txnData_s2(sIntBit_s2_g), getAbs_s2(txnData_s2), txnData_s2)
      val initAbs = Mux(txnData_s2(sIntBit_s2_g), getAbs_s2(initData_s2), initData_s2)
      iniBigger   := txnAbs < initAbs
    }.otherwise {
      iniBigger   := txnData_s2(sIntBit_s2_g)
    }
    dealData_s2   := Mux(iniBigger, txnData_s2, initData_s2)
  // LDUMAX
  }.elsewhen(opcodeOH_s2_g(LDUMAX)) {
    dealData_s2   := Mux(txnData_s2 > initData_s2, txnData_s2, initData_s2)
  // LDUMIN
  }.elsewhen(opcodeOH_s2_g(LDUMIN)) {
    dealData_s2   := Mux(txnData_s2 < initData_s2, txnData_s2, initData_s2)
    // AtomicSwap
  }.elsewhen(opcodeOH_s2_g(SWAP)) {
    val swapData  = swapDataVec_s2_g.asTypeOf(UInt(64.W))
    dealData_s2   := swapData
    // AtomicCompare
  }.elsewhen(opcodeOH_s2_g(COMPARE)) {
    val swapData  = swapDataVec_s2_g.asTypeOf(UInt(128.W))
    val compData  = compDataVec_s2_g.asTypeOf(UInt(128.W))
    val initData  = initDataVec_s2_g.asTypeOf(compData)
    dealData_s2   := Mux(compData === initData, swapData, compData)
  }




// ---------------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------------- S3: Output Result ---------------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  val dealDataVec_s3  = dealData_s3_g.asTypeOf(Vec(16, UInt(8.W)))
  outDataVec_s3.zipWithIndex.foreach {
    case(out, i) =>
      val dealIdx = Wire(UInt(4.W))
      dealIdx := i.U - firstIdx_s3_g
      out := Mux(firstIdx_s3_g <= i.U & i.U < lastIdx_s3_g, dealDataVec_s3(dealIdx), inDataVec_s3_g(i))
  }

  io.out.valid      := valid_s3_g
  io.out.bits.dbID  := dbID_s3_g
  io.out.bits.data  := outDataVec_s3.asUInt


  HardwareAssertion.placePipe(3)
}
















