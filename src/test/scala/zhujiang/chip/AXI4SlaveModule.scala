/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package zhujiang.chip

import chisel3._
import chisel3.util._
import freechips.rocketchip.amba.axi4.{AXI4Parameters, AXI4SlaveNode, AXI4SlaveParameters, AXI4SlavePortParameters}
import freechips.rocketchip.diplomacy.{AddressSet, RegionType, TransferSizes}
import org.chipsalliance.cde.config.Parameters
import org.chipsalliance.diplomacy.lazymodule._
import xs.utils._

abstract class AXI4SlaveModule(
  address: Seq[AddressSet],
  executable: Boolean = true,
  beatBytes: Int = 8,
  burstLen: Int = 1
)(implicit p: Parameters) extends LazyModule {
  val node = AXI4SlaveNode(Seq(AXI4SlavePortParameters(
    Seq(AXI4SlaveParameters(
      address,
      regionType = RegionType.UNCACHED,
      executable = executable,
      supportsWrite = TransferSizes(1, beatBytes * burstLen),
      supportsRead = TransferSizes(1, beatBytes * burstLen),
      interleavedId = Some(0)
    )),
    beatBytes = beatBytes
  )))
  def module: AXI4SlaveModuleImp
}

class AXI4SlaveModuleImp(outer: AXI4SlaveModule) extends LazyModuleImp(outer) {
  val (in, edge) = outer.node.in.head
  // do not let MMIO AXI signals optimized out
  chisel3.dontTouch(in)

  when(in.aw.fire){
    assert(in.aw.bits.burst === AXI4Parameters.BURST_INCR, "only support busrt incr!")
  }
  when(in.ar.fire){
    assert(in.ar.bits.burst === AXI4Parameters.BURST_INCR, "only support busrt incr!")
  }

  val s_idle :: s_rdata :: s_wdata :: s_wresp :: Nil = Enum(4)

  val state = RegInit(s_idle)

  switch(state){
    is(s_idle){
      when(in.ar.fire){
        state := s_rdata
      }
      when(in.aw.fire){
        state := s_wdata
      }
    }
    is(s_rdata){
      when(in.r.fire && in.r.bits.last){
        state := s_idle
      }
    }
    is(s_wdata){
      when(in.w.fire && in.w.bits.last){
        state := s_wresp
      }
    }
    is(s_wresp){
      when(in.b.fire){
        state := s_idle
      }
    }
  }


  val fullMask = MaskExpand(in.w.bits.strb)

  def genWdata(originData: UInt) = (originData & (~fullMask).asUInt) | (in.w.bits.data & fullMask)

  val raddr = Wire(UInt())
  val (readBeatCnt, rLast) = {
    val c = Counter(256)
    val len = HoldUnless(in.ar.bits.len, in.ar.fire)
    raddr := HoldUnless(in.ar.bits.addr, in.ar.fire)
    in.r.bits.last := (c.value === len)

    when(in.r.fire) {
      c.inc()
      when(in.r.bits.last) {
        c.value := 0.U
      }
    }
    when(in.ar.fire) {
      assert(
        in.ar.bits.len === 0.U ||
          in.ar.bits.len === 1.U ||
          in.ar.bits.len === 3.U ||
          in.ar.bits.len === 7.U ||
          in.ar.bits.len === 15.U
      )
    }
    (c.value, in.r.bits.last)
  }

  in.ar.ready := state === s_idle
  in.r.bits.resp := AXI4Parameters.RESP_OKAY
  in.r.valid := state === s_rdata


  val waddr = Wire(UInt())
  val (writeBeatCnt, wLast) = {
    val c = Counter(256)
    waddr := HoldUnless(in.aw.bits.addr, in.aw.fire)
    when(in.w.fire) {
      c.inc()
      when(in.w.bits.last) {
        c.value := 0.U
      }
    }
    (c.value, in.w.bits.last)
  }

  in.aw.ready := state === s_idle && !in.ar.valid
  in.w.ready := state === s_wdata

  in.b.bits.resp := AXI4Parameters.RESP_OKAY
  in.b.valid := state===s_wresp

  in.b.bits.id := RegEnable(in.aw.bits.id, in.aw.fire)
  in.b.bits.user := RegEnable(in.aw.bits.user, in.aw.fire)
  in.r.bits.id := RegEnable(in.ar.bits.id, in.ar.fire)
  in.r.bits.user := RegEnable(in.ar.bits.user, in.ar.fire)
}
