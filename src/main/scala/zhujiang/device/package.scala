package zhujiang

import zhujiang.axi.{AxiParams, AxiSlvParamsCalc}

package object device {
  case class AxiDeviceParams(
    buffers: Int = 0,
    outstanding: Int = 8,
    wrapper: String = "default",
    attr:String = "",
    extPortParams: Option[AxiParams] = None
  )
}
