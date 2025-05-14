package zhujiang.utils

import chisel3._
import chisel3.util.experimental.BoringUtils
import xs.utils.sram.SramPowerCtl

import scala.collection.mutable

object SramPwrCtlBoring {
  private val pwrCtlQueue = mutable.Queue[SramPowerCtl]()

  def addSink(sink:Option[SramPowerCtl]):Unit = sink.foreach(pwrCtlQueue.addOne)

  def addSink(sink:SramPowerCtl):Unit = pwrCtlQueue.addOne(sink)

  def getSrc():SramPowerCtl = {
    val res = Wire(new SramPowerCtl)
    pwrCtlQueue.foreach(bd => BoringUtils.bore(bd) := res)
    pwrCtlQueue.clear()
    res
  }
}
