package zhujiang.system

import chisel3._
import org.chipsalliance.cde.config.Parameters
import zhujiang.axi.AxiBundle
import zhujiang._
import zhujiang.chip.AttrHelper
import xijiang.router.base.IcnBundle

abstract class BaseSystem(implicit p:Parameters) extends ZJModule with AttrHelper {
  override val desiredName = "ZSystemTestTop"
  val io = IO(new Bundle {
    val onReset = Output(Bool())
  })

  val timer = RegInit(0.U(64.W))
  timer := timer + 1.U

  def ccnDrv: Seq[(IcnBundle, String)]
  def saxiDrv: Seq[(AxiBundle, String)]
  def hwaDrv: Seq[(AxiBundle, String)]

  lazy val ccnPorts = ccnDrv.map(drv => IO(new IcnBundle(drv._1.node)(drv._1.p)))
  lazy val saxiPorts = saxiDrv.map(drv => IO(Flipped(new AxiBundle(drv._1.params))))
  lazy val hwaPorts = hwaDrv.map(drv => IO(Flipped(new AxiBundle(drv._1.params))))

  def runIOAutomation():Unit = {
    ccnPorts.zip(ccnDrv).foreach({ case(a, b) =>
      a <> b._1
      a.suggestName(s"ccn${b._2}")
    })
    saxiPorts.zip(saxiDrv).foreach({ case(a, b) =>
      a <> b._1
      a.suggestName(s"s_axi${b._2}")
    })
    hwaPorts.zip(hwaDrv).foreach({ case(a, b) =>
      a <> b._1
      a.suggestName(s"s_axi_hwa${b._2}")
    })
  }
}
