package dongjiang.ddrc

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import chisel3.experimental.ExtModule

class MemHelper extends ExtModule with HasExtModuleInline {
  val dataBits = 64
    
  val clk      = IO(Input(Clock()))
  val ren      = IO(Input(Bool()))
  val rIdx     = IO(Input(UInt(dataBits.W)))
  val rdata    = IO(Output(UInt(dataBits.W)))

  val wen      = IO(Input(Bool()))
  val wIdx     = IO(Input(UInt(dataBits.W)))
  val wdata    = IO(Input(UInt(dataBits.W)))

  val verilogLines  = Seq(
    """ module MemHelper(""",
    """  input           clk,""",
    """  input           ren,""",
    """  input  [63:0]   rIdx,""",
    """  output [63:0]   rdata,""",
    """  input  [63:0]   wIdx,""",
    """  input  [63:0]   wdata,""",
    """  input           wen""",

    """  );""",
    """ import "DPI-C" function void mem_write_helper (""",
    """    input bit      wen,""",
    """    input longint  wIdx,""",
    """    input longint  wdata""",
    """  );""",
    "",
    """ import "DPI-C" function longint mem_read_helper (""",
    """    input bit     ren,""",
    """    input longint rIdx""",
    """ );""",
    "",
    "",
    """  assign rdata = mem_read_helper(ren, rIdx);""",
    "",
    """ always @(posedge clk) begin""",
    """   mem_write_helper(wen, wIdx, wdata);""",
    """ end""",
    "",
    """ endmodule"""
  )

  setInline(s"$desiredName.v", verilogLines.mkString("\n"))
}




