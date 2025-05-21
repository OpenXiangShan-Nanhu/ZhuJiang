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
  val wayOH     = UInt(ways.W) // TODO: dont use OneHot
  val hit       = Bool()
  val metaVec   = Vec(nrMetas, new ChiState(paramType))

  def way       = OHToUInt(wayOH)
  def meta      = metaVec.head // Only use in LLC

  def metaIsVal = Cat(metaVec.map(_.isValid)).orR
  def metaIsInv = !metaIsVal

  def allVec: Seq[Bool] = metaVec.map(_.state.asBool)
  def othVec(metaIdOH: UInt): Seq[Bool] = allVec.zipWithIndex.map { case (v, i) => v & !VecInit(metaIdOH.asBools)(i) }

  def srcHit(metaIdOH: UInt): Bool = hit & metaVec(OHToUInt(metaIdOH)).state.asBool & metaIdOH.orR
  def othHit(metaIdOH: UInt): Bool = hit & othVec(metaIdOH).reduce(_ | _)
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
  def getStateInst(metaIdOH: UInt): StateInst = {
    require(metaIdOH.getWidth == nrSfMetas)
    val inst = Wire(new StateInst)
    inst.valid    := true.B
    inst.srcHit   := sf.srcHit(metaIdOH)
    inst.othHit   := sf.othHit(metaIdOH)
    inst.llcState := Mux(llc.hit, llc.metaVec.head.state, ChiState.I)
    inst
  }

  def getSnpVec(snpTgt: UInt, metaIdOH: UInt): Vec[Bool] = {
    require(metaIdOH.getWidth == nrSfMetas)
    val allVec = Wire(Vec(nrSfMetas, Bool()))
    val othVec = Wire(Vec(nrSfMetas, Bool()))
    val oneVec = Wire(Vec(nrSfMetas, Bool()))
    allVec     := sf.allVec
    othVec     := sf.othVec(metaIdOH)
    oneVec     := PriorityEncoderOH(othVec)
    val snpVec = PriorityMux(Seq(
      (snpTgt === SnpTgt.ALL) -> allVec,
      (snpTgt === SnpTgt.ONE) -> oneVec,
      (snpTgt === SnpTgt.OTH) -> othVec,
      true.B                  -> 0.U.asTypeOf(allVec)
    ))
    snpVec
  }
}

class DirMsg(implicit p: Parameters) extends DJBundle with HasDirMsg

trait HasPackDirMsg extends DJBundle { this: DJBundle => val dir = new DirMsg() }

class PackDirMsg(implicit p: Parameters) extends DJBundle with HasPackDirMsg

