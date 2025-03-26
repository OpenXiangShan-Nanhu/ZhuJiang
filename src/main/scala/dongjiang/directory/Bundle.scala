package dongjiang.directory

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import dongjiang.bundle._
import dongjiang.frontend.decode._

trait HasDirParam extends DJBundle { this: DJBundle =>
  def paramType: String

  def sets     : Int = if(paramType == "llc") llcSets           else sfSets
  def ways     : Int = if(paramType == "llc") djparam.llcWays   else djparam.sfWays
  def tagBits  : Int = if(paramType == "llc") llcTagBits        else sfTagBits
  def setBits  : Int = if(paramType == "llc") llcSetBits        else sfSetBits
  def nrMetas  : Int = if(paramType == "llc") 1                 else nrSfMetas
  def wayBits  : Int = log2Ceil(ways)
}

class DirParam(dirType: String)(implicit p: Parameters) extends DJBundle with HasDirParam {
  override def paramType: String = dirType
}

/*
 * DirBaseMsg:
 * HasDirBaseMsg -> DirEntry/HasDirMsg -> DirMsg -> PackDirMsg -> HasPackDirMsg
 */
trait HasDirBaseMsg extends DJBundle { this: DJBundle with HasDirParam =>
  val wayOH   = UInt(ways.W)
  val hit     = Bool()
  val metaVec = Vec(nrMetas, new ChiState(paramType))

  def allVec: Seq[Bool] = metaVec.map(_.state.asBool)
  def othVec(metaId: UInt): Seq[Bool] = metaVec.map(_.state.asBool).zipWithIndex.map { case (m, i) => m & metaId =/= i.U }

  def srcHit(metaId: UInt): Bool = hit & metaVec(metaId).state.asBool
  def othHit(metaId: UInt): Bool = hit & othVec(metaId).reduce(_ | _)
}

class DirEntry(dirType: String)(implicit p: Parameters) extends DJBundle with HasDirParam with HasAddr with HasDirBaseMsg {
  override def paramType: String = dirType
  override def addrType : String = dirType
}

trait HasDirMsg extends DJBundle { this: DJBundle =>
  val llc = new DJBundle with HasDirParam with HasDirBaseMsg {
    override def paramType: String = "llc"
  }
  val sf  = new DJBundle with HasDirParam with HasDirBaseMsg {
    override def paramType: String = "sf"
  }
  def getStateInst(metaId: UInt): StateInst = {
    val inst = Wire(new StateInst)
    inst.valid    := true.B
    inst.srcHit   := sf.srcHit(metaId)
    inst.othHit   := sf.othHit(metaId)
    inst.llcState := Mux(llc.hit, llc.metaVec.head.state, ChiState.I)
    inst
  }
}

class DirMsg(implicit p: Parameters) extends DJBundle with HasDirMsg

trait HasPackDirMsg extends DJBundle { this: DJBundle => val dir = new DirMsg() }

class PackDirMsg(implicit p: Parameters) extends DJBundle with HasPackDirMsg

