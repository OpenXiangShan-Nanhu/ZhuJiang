package xijiang.tfb

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.NodeType
import zhujiang.{HasZJParams, ZJParametersKey}

class NodeRegister(implicit val p: Parameters) extends BlackBox with HasBlackBoxInline with HasZJParams {
  val io = IO(new Bundle {
    val nodeId = Input(UInt(niw.W))
    val nodeType = Input(UInt((NodeType.width + 1).W))
  })
  private val modName = s"${p(ZJParametersKey).modulePrefix}TrafficBoardNodeRegister"
  override val desiredName = modName
  setInline(s"$modName.sv",
    s"""
       |module $modName (
       |  input  [${niw - 1}:0] \t\tnodeId,
       |  input  [${NodeType.width}:0] \t\tnodeType
       |);
       |`ifndef NO_TRAFFIC_BOARD
       |  import "DPI-C" function void tfb_register_node (
       |    input  shortint \t\tnode_id,
       |    input  shortint \t\tnode_type
       |  );
       |  wire [15:0] nid;
       |  wire [15:0] nt;
       |  assign nid = {${16 - niw}'h0, nodeId};
       |  assign nt = {${16 - NodeType.width - 1}'h0, nodeType};
       |`ifdef VERILATOR
       |  initial tfb_register_node(nid, nt);
       |`else
       |  initial #1 tfb_register_node(nid, nt);
       |`endif // VERILATOR
       |`endif // NO_TRAFFIC_BOARD
       |endmodule""".stripMargin)
}
