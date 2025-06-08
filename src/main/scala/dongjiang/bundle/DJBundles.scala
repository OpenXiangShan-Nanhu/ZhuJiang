package dongjiang.bundle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import dongjiang._
import zhujiang.chi._
import dongjiang.frontend.decode._
import dongjiang.frontend.decode.Decode._
import zhujiang.chi.ReqOpcode.WriteEvictOrEvict

/*
 * NoC Type
 */
object NocType {
  def rxIs (flit: Flit, t: Int) : Bool = flit.tgt === t.U
  def txIs (flit: Flit, t: Int) : Bool = flit.src === t.U
}


/*
 * Addr:
 * HasAddr -> Addr
 */
trait HasAddr extends DJBundle { this: DJBundle =>
  val addr      = UInt(addrBits.W)

  def addrType: String = "llc"
  object Addr {
    def ci        = getCI(getUseAddr(addr))
    def useAddr   = getUseAddr(addr)
    def bankId    = if(bankBits != 0) getBankId(addr) else 0.U
    def offset    = getOffset(addr)
    def dirBank   = getDirBank(addr)
    def llcTag    = getLlcTag(addr)
    def llcSet    = getLlcSet(addr)
    def sfTag     = getSfTag(addr)
    def sfSet     = getSfSet(addr)
    def posTag    = getPosTag(addr)
    def posSet    = if(posSetBits != 0) getPosSet(addr) else 0.U
    def dirSet    = if(sfSets > llcSets) sfSet else llcSet

    def getDataId = Mux(offset(offsetBits-1), "b10".U, "b00".U)

    def isToLAN(selfCI: UInt)  = ci === selfCI
    def isToBBN(selfCI: UInt)  = ci =/= selfCI

    def catByX(bank: UInt, tag: UInt, tagBits: Int, set: UInt, setBits: Int, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = {
      require(bank.getWidth    == bankBits,    s"bankBits:    ${bank.getWidth} =/= ${bankBits}")
      require(tag.getWidth     == tagBits,     s"tagBits:     ${tag.getWidth} =/= ${tagBits}")
      require(set.getWidth     == setBits,     s"setBits:     ${set.getWidth} =/= ${setBits}")
      require(offset.getWidth  == offsetBits,  s"offsetBits:  ${offset.getWidth} =/= ${offsetBits}")
      val useAddr_ = if(djparam.nrDirBank > 1) {
        require(dirBank.getWidth == dirBankBits, s"dirBankBits: ${dirBank.getWidth} =/= ${dirBankBits}")
        Cat(tag, set, dirBank)
      } else {
        Cat(tag, set)
      }
      val addr_ = Cat(useAddr_(useAddrBits-1, bankId_lo-offsetBits), bank, useAddr_(bankId_lo-offsetBits-1, 0), offset)
      require(addr_.getWidth   == addrBits)
      addr := addr_
    }
    def catPoS(bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, posTagBits, set, posSetBits, dirBank, offset)
    def catLLC(bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, llcTagBits, set, llcSetBits, dirBank, offset)
    def catSF (bank: UInt, tag: UInt, set: UInt, dirBank: UInt, offset: UInt = 0.U(offsetBits.W)) = catByX(bank, tag, sfTagBits,  set, sfSetBits,  dirBank, offset)

    def cat(bank: UInt, tag: UInt, set: UInt, dirBank: UInt): Unit = if(addrType == "llc") catLLC(bank, tag, set, dirBank) else catSF(bank, tag, set, dirBank)
    def tag: UInt = if(addrType == "llc") llcTag else sfTag
    def set: UInt = if(addrType == "llc") llcSet else sfSet
  }
}

class Addr(dirType: String = "llc")(implicit p: Parameters) extends DJBundle with HasAddr {
  override def addrType: String = dirType
}

trait HasAddrValid { this: DJBundle => val addrVal = Bool() }

/*
 * HnIdx:
 * HasHnIdx -> HnIndex
 */
trait HasHnIdx extends DJBundle { this: DJBundle =>
  val dirBank   = UInt(dirBankBits.W)
  val pos       = new DJBundle {
    val set     = UInt(posSetBits.W)
    val way     = UInt(posWayBits.W)
  }
  def getTxnID  = Cat(dirBank, pos.set, pos.way)
}

class HnIndex(implicit p: Parameters) extends DJBundle with HasHnIdx

trait HasPackHnIdx extends DJBundle { this: DJBundle => val hnIdx = new HnIndex() }

class PackHnIdx(implicit p: Parameters) extends DJBundle with HasPackHnIdx


/*
 * HnTxnID:
 * HasHnTxnID -> HnTxnID
 */
trait HasHnTxnID extends DJBundle { this: DJBundle =>
  val hnTxnID = UInt(hnTxnIDBits.W)

  def dirBank = if(dirBankBits == 0) 0.U else hnTxnID(dirBank_hn_hi, dirBank_hn_lo)
  def posSet  = if(posSetBits == 0) 0.U else hnTxnID(posSet_hn_hi, posSet_hn_lo)
  def posWay  = hnTxnID(posWay_hn_hi, posWay_hn_lo)

  def getHnIdx  = {
    val idx     = Wire(new HnIndex)
    idx.dirBank := dirBank
    idx.pos.set := posSet
    idx.pos.way := posWay
    idx
  }

  // Note: only set pos.set and dirBank
  def getAddr = {
    val a = Wire(new Addr())
    a.Addr.catPoS(0.U(bankBits.W), 0.U(posTagBits.W), posSet, dirBank)
    a.addr
  }
}

class HnTxnID(implicit p: Parameters) extends DJBundle with HasHnTxnID

/*
 * HasDataVec
 */
object DataVec {
  def Full = VecInit(Seq(true.B, true.B))
  def Zero = VecInit(Seq(false.B, false.B))
}

trait HasDataVec extends DJBundle { this: DJBundle =>
  val dataVec     = Vec(djparam.nrBeat, Bool())
  def isZero      = PopCount(dataVec) === 0.U
  def isFullSize  = dataVec.asUInt.andR
  def isHalfSize  = !isZero & !isFullSize
  def getSize     = Mux(isFullSize, 6.U, Mux(isHalfSize, 5.U, 0.U))
  def getOffset   = {
    val offset = Wire(UInt(offsetBits.W))
    when(isFullSize | isZero) {
      offset := 0.U
    }.otherwise {
      val beatIdx = PriorityEncoder(dataVec)
      offset := beatIdx << beatSize
    }
    offset
  }
}


/*
 * Chi:
 * HasChi -> Chi -> HasPackChi -> PackChi
 */
trait HasChi { this: DJBundle with HasNodeId with HasChiChannel with HasChiOp
  with HasChiOrderAndExpCompAck with HasChiSnpField with HasQoS with HasDataVec =>
  // REQ
  val txnID         = UInt(ChiTxnIdBits.W)
  val memAttr       = new MemAttr()
  // SNP
  val fwdNID        = UInt(nodeIdBits.W)
  val fwdTxnID      = UInt(ChiFwdTxnIdBits.W)
  val retToSrc      = Bool()
  // Flag
  val toLAN         = Bool() // TODO: It should be in CommitTask?
  def toBBN         = !toLAN
  def getNoC        = Mux(toLAN, LAN.U, BBN.U)

  def getChiInst: ChiInst = {
    val inst = Wire(new ChiInst)
    inst.valid      := true.B
    inst.channel    := channel
    inst.fromLAN    := fromLAN
    inst.toLAN      := toLAN
    inst.opcode     := opcode
    inst.expCompAck := expCompAck
    inst.allocate   := Mux(isWrite, memAttr.allocate, false.B)
    inst.ewa        := Mux(isWrite, memAttr.ewa,      false.B)
    inst.order      := order
    inst
  }
}

class Chi(implicit p: Parameters) extends DJBundle with HasNodeId with HasChiChannel
  with HasChiOp with HasChiOrderAndExpCompAck with HasChiSnpField with HasQoS with HasDataVec with HasChi

trait HasPackChi { this: DJBundle => val chi = new Chi() }

class PackChi(implicit p: Parameters) extends DJBundle with HasPackChi

/*
 * GetAddr
 */
class GetAddr(frontend: Boolean = false)(implicit p: Parameters) extends DJBundle {
  val hnIdx   = new HnIndex
  val result  = Input(new Addr)
}

/*
 * HasAlready
 */
trait HasAlready { this: DJBundle =>
  val alr           = new DJBundle {
    val reqDB       = Bool() // Already request DataCM and DataBuf. If set it, commit will clean when done all
    val sData       = Bool() // Already send DataTask to read DS to CHI in S3(Decode) and Commit need wait DataResp
    val getSnpData  = Bool() // Already get data from SnpRespDataX
  }
}

/*
 * HasDecList
 */
trait HasDecList { this: DJBundle => val decList = MixedVec(UInt(w_ci.W), UInt(w_si.W), UInt(w_ti.W), UInt(w_sti.W)) }

class PackDecList(implicit p: Parameters) extends DJBundle with HasDecList
