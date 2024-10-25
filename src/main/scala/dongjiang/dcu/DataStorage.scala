package dongjiang.dcu

import dongjiang._
import dongjiang.chi._
import chisel3.{util, _}
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.sram.SRAMTemplate
import chisel3.util.random.LFSR
import freechips.rocketchip.util.ReplacementPolicy


object SramCtrlState {
  // [Free] ---> [ReqFire] ---> [WaitMcp]  ---> [GetResp]
  //   ReplSram: [writeRepl]    [readRepl]      [getRepl]
  //                      MSHR: [readMSHR]      [mshrResp]
  //
  //
  // [Free] ---> [ReqFire] ---> [WaitMcp]  ---> [GetResp]
  //                            [EarlyReq] ---> [ReqFire] --->  [WaitMcp] ---> [GetResp] ---> [Free]
  //
  // [Free] ---> [ReqFire] ---> [WaitMcp]  ---> [GetResp]
  //                                            [EarlyReq] ---> [ReqFire] ---> [WaitMcp]  ---> [GetResp] ---> [Free]
  val width               = 3
  val Free                = "b000".U
  val ReqFire             = "b100".U
  val WaitMcp             = "b010".U
  val GetResp             = "b001".U
  val GetResp_ReqFire     = "b101".U
}



class SramCtrlBundle extends Bundle {
  val shiftState  = UInt(SramCtrlState.width.W)
  val shiftRen    = UInt(SramCtrlState.width.W)

  def canRecReq   = shiftState === SramCtrlState.Free     | shiftState === SramCtrlState.WaitMcp          | shiftState === SramCtrlState.GetResp
  def isReqFire   = shiftState === SramCtrlState.ReqFire  | shiftState === SramCtrlState.GetResp_ReqFire
  def isWaitMcp   = shiftState === SramCtrlState.WaitMcp
  def isGetResp   = (shiftState === SramCtrlState.GetResp | shiftState === SramCtrlState.GetResp_ReqFire) & shiftRen(0)

  def isRReqFire  = isReqFire & shiftRen(2)
  def isWReqFire  = isReqFire & !shiftRen(2)
}

class DsWriteBundle(indexBits: Int)(implicit p: Parameters) extends DJBundle {
  val index = UInt(indexBits.W)
  val data  = UInt(dataBits.W)
}


class DataStorage(sets: Int)(implicit p: Parameters) extends DJModule {

  val indexBits     = log2Ceil(sets)

// --------------------- IO declaration ------------------------//
  val io = IO(new Bundle {
    val id        = Input(UInt(dirBankBits.W))
    val earlyRReq = Flipped(Decoupled())
    val earlyWReq = Flipped(Decoupled())
    val read      = Input(UInt(indexBits.W))
    val write     = Input(new DsWriteBundle(indexBits))
    val resp      = Valid(UInt(dataBits.W))
  })

// --------------------- Modules declaration ------------------------//
  val arrays      = Seq.fill(nrBeat) { Module(new SRAMTemplate(UInt(beatBits.W), sets, way = 1, singlePort = true, multicycle = 2, holdMcp = true)) }

//// ----------------------- Reg/Wire declaration --------------------------//
  // Base
  val sramCtrlReg = RegInit(0.U.asTypeOf(new SramCtrlBundle))
  // s2
  val valid_s2    = WireInit(false.B)
  val resp_s2     = Wire(UInt(dataBits.W))
  // s3
  val valid_s3_g  = RegInit(false.B)
  val resp_s3_g   = Reg(UInt(dataBits.W))


// ---------------------------------------------------------------------------------------------------------------------- //
// -------------------------------------------------- S1: Read / Write SRAM --------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Set SramCtrl Value
   */
  sramCtrlReg.shiftState  := Cat(io.earlyRReq.fire | io.earlyWReq.fire, sramCtrlReg.shiftState(SramCtrlState.width - 1, 1))
  sramCtrlReg.shiftRen    := Cat(io.earlyRReq.fire,                     sramCtrlReg.shiftRen(SramCtrlState.width - 1, 1))
  assert(!(io.earlyRReq.fire & io.earlyWReq.fire))


  /*
   * Set EarlyReq Ready
   */
  io.earlyRReq.ready        := sramCtrlReg.canRecReq & !io.earlyWReq.valid
  io.earlyWReq.ready        := sramCtrlReg.canRecReq


  /*
   * Read / Write Req SRAM
   */
  // early
  arrays.zipWithIndex.foreach {
    case(a, i) =>
      a.io.earlyRen.get       := io.earlyRReq.fire
      a.io.earlyWen.get       := io.earlyWReq.fire
      // ren
      a.io.r.req.valid        := sramCtrlReg.isRReqFire
      a.io.r.req.bits.setIdx  := io.read
      // wen
      a.io.w.req.valid        := sramCtrlReg.isWReqFire
      a.io.w.req.bits.setIdx  := io.write.index
      a.io.w.req.bits.data.foreach(_ := io.write.data(beatBits * (i + 1) - 1, beatBits * i))
  }



// ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------- S2: Receive SRAM Resp ---------------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------------- //

  /*
   * Receive Meta SRAM resp
   */
  valid_s2      := sramCtrlReg.isGetResp
  resp_s2       := Cat(arrays.map(_.io.r.resp.data(0)).reverse)


// ---------------------------------------------------------------------------------------------------------------------- //
// ---------------------------------------------------- S3: Output Resp  ------------------------------------------------ //
// ---------------------------------------------------------------------------------------------------------------------- //
  /*
   * Receive S2
   */
  valid_s3_g    := valid_s2
  resp_s3_g     := resp_s2

  /*
   * Output Resp
   */
  io.resp.valid := valid_s3_g
  io.resp.bits  := resp_s3_g


}