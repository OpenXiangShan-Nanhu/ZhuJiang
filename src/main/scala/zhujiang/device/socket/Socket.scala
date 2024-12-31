package zhujiang.device.socket

import chisel3._
import chisel3.util._
import xijiang.Node

object SocketType {
  val decoupled:Int = 0
  val async:Int = 1
  val aurora:Int = 2

  def getType(t:String):Int = {
    t.toLowerCase match {
      case "decoupled" => decoupled
      case "async" => async
      case "aurora6466" => 2
    }
  }
}

class SocketIcnSide(socketType:Int, node:Node) extends Module {
  val io = IO(new Bundle {

  })
}

class SocketDevSide(socketType:Int, node:Node) extends Module {

}
