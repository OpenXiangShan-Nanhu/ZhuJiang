package xijiang.tfs

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import zhujiang.chi.Flit
import zhujiang.{HasZJParams, ZJParametersKey}

object TrafficSimTx {
  def connTfsTx[T <: Data](txGen: TrafficSimTx, tx: DecoupledIO[T], nodeId: UInt, chn: UInt, clock: Clock, reset: Reset): Unit = {
    tx.valid := txGen.io.tx.valid
    tx.bits := txGen.io.tx.bits.asTypeOf(tx.bits)
    txGen.io.tx.ready := tx.ready
    txGen.io.nodeId := nodeId
    txGen.io.chn := chn
    txGen.io.clock := clock
    txGen.io.reset := reset
  }
}

class TrafficSimTx(implicit val p: Parameters) extends BlackBox with HasBlackBoxInline with HasZJParams {
  val io = IO(new Bundle {
    val tx = Decoupled(UInt(maxFlitBits.W))
    val nodeId = Input(UInt(niw.W))
    val chn = Input(UInt(8.W))
    val clock = Input(Clock())
    val reset = Input(Reset())
  })
  private val modName = s"${p(ZJParametersKey).modulePrefix}TrafficSimTx"
  override val desiredName = modName

  setInline(s"$modName.sv",
    s"""
       |module $modName (
       |  input \t\t\t\t\tclock,
       |  input \t\t\t\t\treset,
       |  input  [7:0] \t\tchn,
       |  input  [${niw - 1}:0] \t\tnodeId,
       |  output \t\t\t\t\ttx_valid,
       |  input \t\t\t\t\ttx_ready,
       |  output [${maxFlitBits - 1}:0] \ttx_bits
       |);
       |  import "DPI-C" function void tfs_get_tx_flit(
       |    input shortint \t\tnode_id,
       |    input \t\t\t\t\tbyte chn,
       |    output [${maxFlitBits - 1}:0] \tflit,
       |    output \t\t\t\t\tvalid,
       |    input \t\t\t\t\tready,
       |    input \t\t\t\t\treset
       |  );
       |  wire [15:0] nid;
       |  assign nid = {${16 - niw}'h0, nodeId};
       |
       |  always @(posedge clock) begin
       |    tfs_get_tx_flit(nid, chn, tx_bits, tx_valid, tx_ready, reset);
       |  end
       |endmodule""".stripMargin)
}
