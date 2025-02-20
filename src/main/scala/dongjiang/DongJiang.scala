package dongjiang

import xijiang.Node
import chisel3._
import chisel3.experimental.hierarchy.{instantiable, public}
import chisel3.util._
import org.chipsalliance.cde.config._
import xs.utils.perf.{DebugOptions, DebugOptionsKey}
import xijiang.router.base.DeviceIcnBundle
import xs.utils.debug.{DomainInfo, HardwareAssertion}



@instantiable
class DongJiang(localHnf: Seq[Node], csnHnx: Option[Node] = None)(implicit p: Parameters) extends DJRawModule
  with ImplicitClock with ImplicitReset {
  // ------------------------------------------ IO declaration ----------------------------------------------//
  val nrLocalIcn  = localHnf.length
  @public val io  = IO(new Bundle {
    // Configuration Signals
    val flushCacheReq = Input(Valid(UInt(nrCcNode.W)))
    val flushCacheAck = Output(UInt(nrBank.W))
    val closeLLC      = Input(Bool())
    // Local ICN
    val hnfIdVec      = Input(Vec(nrLocalIcn, UInt(nodeIdBits.W)))
    val frinedsVec    = Input(Vec(nrLocalIcn, Vec(nrFriendsNodeMax, UInt(nodeIdBits.W))))
    val localIcnVec   = MixedVec(localHnf.map(n => new DeviceIcnBundle(n)))
    // CSN ICN
    val hnsId         = if(csnHnx.nonEmpty) Some(Input(UInt(nodeIdBits.W))) else None
    val c2cChipVec    = if(csnHnx.nonEmpty) Some(Input(Vec(nrCcNode, Vec(nrChip, Bool())))) else None
    val csnIcn        = if(csnHnx.nonEmpty) Some(new DeviceIcnBundle(csnHnx.get)) else None
  })
  @public val reset   = IO(Input(AsyncReset()))
  @public val clock   = IO(Input(Clock()))
  val implicitClock   = clock
  val implicitReset   = reset

  if(hasCSN) {
    require(csnHnx.nonEmpty)
  }

  io <> DontCare

  /*
   * Hardware Assertion Node And IO
   */
  HardwareAssertion(true.B)
  private val assertionNode = HardwareAssertion.placePipe(Int.MaxValue - 1, true)
  @public
  val assertionOut = IO(assertionNode.assertion.cloneType)
  @public
  val assertionInfo = DomainInfo(assertionNode.desc)
  assertionOut <> assertionNode.assertion

}
