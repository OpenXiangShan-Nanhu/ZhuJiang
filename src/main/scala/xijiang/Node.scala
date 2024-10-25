package xijiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.router._
import xijiang.router.base.BaseRouter
import zhujiang.ZJParametersKey
import zhujiang.chi.{NodeIdBundle, ReqAddrBundle, SnpAddrBundle}

object NodeType {
  val CC: Int = 0
  val RF: Int = 1
  val RI: Int = 2
  val HF: Int = 3
  val HI: Int = 4
  val C: Int = 5
  val S: Int = 6
  val P: Int = 7
  def width: Int = log2Ceil(P)
  def min: Int = 0
  def max: Int = P
}

case class NodeParam(
  attr: String = "",
  nodeType: Int = NodeType.P,
  splitFlit: Boolean = false,
  bankId: Int = 0, // Only applied in SN and HNF
  mainMemory: Boolean = false, // Only applied in SN
  cpuNum: Int = 1, // Only applied in CC
  addressRange: (Long, Long) = (0L, 0L), // Only applied in HNI
  defaultHni: Boolean = false, // Only applied in HNI
  outstanding: Int = 16 // Only applied in HNI
)

case class Node(
  attr: String = "",
  nodeType: Int = NodeType.P,
  csnNode: Boolean = false,
  nidBits: Int = 7,
  aidBits: Int = 3,
  ringSize: Int = 3,
  globalId: Int = 0,
  splitFlit: Boolean = false,
  domainId:Int = 0,
  bankId: Int = 0, // Only applied in SN, HNF and CRF
  bankBits: Int = 1, // Only applied in SN, HNF and CRF
  mainMemory: Boolean = false, // Only applied in SN
  cpuNum: Int = 1, // Only applied in CCN
  clusterId: Int = 0, //Only applied in CCN
  addressRange: (Long, Long) = (0L, 0L), // Only applied in HNI
  defaultHni: Boolean = false, // Only applied in HNI
  outstanding: Int = 16 // Only applied in HNI, CCN and SN
) {
  require(NodeType.min <= nodeType && nodeType <= NodeType.max)

  val net = if(csnNode) 1 else 0
  private val netOff = nidBits + aidBits
  val nodeId = (net << netOff) | (globalId << aidBits)

  var leftNodes: Seq[Node] = Seq()
  var rightNodes: Seq[Node] = Seq()

  def genRouter(p: Parameters): BaseRouter = {
    val res = nodeType match {
      case NodeType.CC => Module(new RxReqRouter(this)(p))
      case NodeType.RF => Module(new RxReqRouter(this)(p))
      case NodeType.RI => Module(new RxReqRouter(this)(p))
      case NodeType.HF => Module(new BaseRouter(this)(p))
      case NodeType.HI => Module(new BaseRouter(this)(p))
      case NodeType.C => Module(new ChipToChipRouter(this)(p))
      case NodeType.S => Module(new BaseRouter(this)(p))
      case _ => Module(new BaseRouter(this)(p))
    }
    res.suggestName(routerName)
    res
  }

  private lazy val routerName:String = {
    val csnStr = if(csnNode && nodeType != NodeType.C) "c" else ""
    val nstr = nodeType match {
      case NodeType.CC => s"ccn_$domainId"
      case NodeType.RF => s"rnf_pcu_$bankId"
      case NodeType.RI => s"rni_$attr"
      case NodeType.HF => s"hnf_pcu_$bankId"
      case NodeType.HI => s"hni_$attr"
      case NodeType.C => s"c2c_$domainId"
      case NodeType.S => if(attr == "") s"sn_dcu_$bankId" else s"sn_$attr"
      case _ => "pip"
    }
    s"$csnStr${nstr}_id_0x${nodeId.toHexString}"
  }

  private lazy val routerPrefixStr: String = if(csnNode) "Csn" else ""
  private lazy val icnPrefixStr: String = if(csnNode) "csn_" else ""
  private lazy val snAttrStr: String = if(mainMemory) "mem_" else "dcu_"
  private def icnStrGen(pfx: String, body: String) = s"$icnPrefixStr$pfx${body}_id_${nodeId.toHexString}"
  private def routerStrGen(body: String) = s"Router$routerPrefixStr${body}_0x${nodeId.toHexString}"
  lazy val (routerStr, icnStr, nodeStr): (String, String, String) = nodeType match {
    case NodeType.CC => (routerStrGen("CpuCluster"), icnStrGen("", "ccn"), "CCN")
    case NodeType.RF => (routerStrGen("RequestFull"), icnStrGen("", if(csnNode) s"rnf_bank_$bankId" else "rnf"), "RNF")
    case NodeType.RI => (routerStrGen("RequestIo"), icnStrGen("", "rni"), "RNI")
    case NodeType.HF => (routerStrGen("HomeFull"), icnStrGen("", s"hnf_bank_$bankId"), "HNF")
    case NodeType.HI => (routerStrGen("HomeIo"), icnStrGen("", "hni"), "HNI")
    case NodeType.C => (routerStrGen("ChipToChip"), icnStrGen("", "c2c"), "C2C")
    case NodeType.S => (routerStrGen("Subordinate"), icnStrGen(snAttrStr, s"sn_bank_$bankId"), "SN")
    case _ => (routerStrGen("Pipeline"), icnStrGen("", "pip"), "PIP")
  }

  lazy val (ejects, injects): (Seq[String], Seq[String]) = {
    val res = nodeType match {
      case NodeType.CC => (Seq("REQ", "RSP", "DAT", "SNP"), Seq("REQ", "RSP", "DAT"))
      case NodeType.RF => (Seq("RSP", "DAT", "SNP"), Seq("REQ", "RSP", "DAT"))
      case NodeType.RI => (Seq("RSP", "DAT"), Seq("REQ", "RSP", "DAT"))
      case NodeType.HF => (Seq("REQ", "RSP", "DAT"), Seq("RSP", "DAT", "SNP", "ERQ"))
      case NodeType.HI => (Seq("REQ", "RSP", "DAT"), Seq("RSP", "DAT", "ERQ"))
      case NodeType.C => (Seq("RSP", "DAT", "SNP", "REQ"), Seq("RSP", "DAT", "SNP", "REQ"))
      case NodeType.S => (Seq("ERQ", "DAT"), Seq("RSP", "DAT"))
      case _ => (Seq(), Seq())
    }
    val illegal1 = res._1.contains("REQ") && res._1.contains("ERQ")
    val illegal2 = res._2.contains("REQ") && res._2.contains("ERQ")
    require(!illegal1, "Cannot eject from both REQ and ERQ")
    require(!illegal2, "Cannot inject to both REQ and ERQ")
    res
  }

  private def getLegalTgtSeq(ring: Seq[Node], chn: String): Seq[Int] = {
    import NodeType._
    val legalTgtTypeSeq = nodeType match {
      case CC => chn match {
        case "REQ" => Seq(CC, HF, HI)
        case "RSP" => Seq(CC, HF, HI)
        case "DAT" => Seq(CC, RF, RI, HF, HI, S)
        case _ => Seq[Int]()
      }
      case RF => chn match {
        case "REQ" => Seq(CC, HF, HI)
        case "RSP" => Seq(CC, HF, HI)
        case "DAT" => Seq(CC, RF, RI, HF, HI, S)
        case _ => Seq[Int]()
      }
      case RI => chn match {
        case "REQ" => Seq(CC, HF, HI)
        case "RSP" => Seq(CC, HF, HI)
        case "DAT" => Seq(CC, HF, HI, S)
        case _ => Seq[Int]()
      }
      case HF => chn match {
        case "RSP" => Seq(CC, RF, RI)
        case "DAT" => Seq(CC, RF, RI, S)
        case "SNP" => Seq(CC, RF)
        case "ERQ" => Seq(S)
        case _ => Seq[Int]()
      }
      case HI => chn match {
        case "RSP" => Seq(CC, RF, RI)
        case "DAT" => Seq(CC, RF, RI)
        case "ERQ" => Seq(S)
        case _ => Seq[Int]()
      }
      case S => chn match {
        case "RSP" => Seq(CC, RF, RI, HF, HI)
        case "DAT" => Seq(CC, RF, RI, HF, HI)
        case _ => Seq[Int]()
      }
      case C => chn match {
        case "REQ" => Seq(HF)
        case "RSP" => Seq(RF, HF)
        case "DAT" => Seq(RF, HF)
        case "SNP" => Seq(RF)
        case _ => Seq[Int]()
      }
      case _ => Seq[Int]()
    }
    require(legalTgtTypeSeq.nonEmpty, s"node 0x${nodeId.toHexString} has no inject channel $chn")
    val res = ring.filter(n => legalTgtTypeSeq.contains(n.nodeType)).map(_.nodeId).filterNot(_ == nodeId)
    if(nodeType == NodeType.S && !mainMemory) {
      res ++ ring.filter(n => n.nodeType == NodeType.S && n.mainMemory).map(_.nodeId)
    } else {
      res
    }
  }

  def checkLegalInjectTarget(ring: Seq[Node], chn: String, tgt: NodeIdBundle, valid: Bool, nid: UInt): Unit = {
    val legalTgtSeq = getLegalTgtSeq(ring, chn)
    require(legalTgtSeq.nonEmpty, s"targets are empty when making node 0x${nodeId.toHexString} of $chn")
    val tgtHitSeq = legalTgtSeq.map(n => n.U(tgt.getWidth.W) === tgt.router)
    val legalStr = legalTgtSeq.map(i => s"0x${i.toHexString} ").reduce(_ + _)
    val legal = Cat(tgtHitSeq).orR
    when(valid) {
      assert(legal, cf"Illegal target id 0x${tgt.asUInt}%x of $chn flit @ node 0x${nid}%x legal target_id: $legalStr")
    }
  }

  private def hnfAddrCheck(addr: ReqAddrBundle): Bool = {
    !addr.mmio && addr.checkBank(bankBits, bankId.U)
  }

  private def hniAddrCheck(addr: ReqAddrBundle, chip: UInt): Bool = {
    val addrMin = addressRange._1.U(addr.getWidth.W)(addr.devAddr.getWidth - 1, 0)
    val addrMax = addressRange._2.U(addr.getWidth.W)(addr.devAddr.getWidth - 1, 0)
    if(defaultHni) {
      addr.mmio
    } else {
      addr.mmio && addr.chip === chip && addrMin <= addr.devAddr && addr.devAddr < addrMax
    }
  }

  def isReqCompleter(addr: ReqAddrBundle, chip: UInt): Bool = {
    import NodeType._
    nodeType match {
      case CC => hniAddrCheck(addr, chip)
      case HF => hnfAddrCheck(addr)
      case HI => hniAddrCheck(addr, chip)
      case _ => false.B
    }
  }

  var friends = Seq[Node]()

  def findSn(bank: UInt, check: Bool): UInt = {
    require(nodeType == NodeType.HF)
    val selOH = friends.map(n => n.bankId.U === bank)
    val selBits = friends.map(n => n.nodeId.U((1 + nidBits + aidBits).W))
    when(check) {
      val legal = PopCount(selOH) === 1.U
      assert(legal)
    }
    Mux1H(selOH, selBits)
  }

  def checkSnTgt(tgt: UInt): Bool = {
    val tgtRouter = tgt & Cat(Fill(1 + nidBits, true.B), Fill(aidBits, false.B))
    friends.map(_.nodeId.U((1 + nidBits + aidBits).W) === tgtRouter).reduce(_ || _)
  }

  lazy val leftsStr = if(leftNodes.nonEmpty) leftNodes.map(e => "0x" + e.nodeId.toHexString).reduce((a: String, b: String) => s"$a, $b") else ""
  lazy val rightsStr = if(rightNodes.nonEmpty) rightNodes.map(e => "0x" + e.nodeId.toHexString).reduce((a: String, b: String) => s"$a, $b") else ""
  lazy val friendsStr = if(friends.nonEmpty) friends.map(e => "0x" + e.nodeId.toHexString).reduce((a: String, b: String) => s"$a, $b") else ""
  override def toString = {
    val head =
      s"""
         |  $routerStr {
         |    node_id: 0x${nodeId.toHexString}
         |    lefts: $leftsStr
         |    rights: $rightsStr,
         |    domainId: $domainId,
         |    routerName: $routerName
         |""".stripMargin

    val frdsStr = if((nodeType == NodeType.HF || nodeType == NodeType.S && !mainMemory) && !csnNode) {
      s"""    friends: $friendsStr
         |    bank: $bankId
         |""".stripMargin
    } else {
      ""
    }

    val snDevStr = if(nodeType == NodeType.S) {
      s"""    device: ${if(mainMemory) "mem" else "dcu"}
         |""".stripMargin
    } else {
      ""
    }

    val ccAttrStr = if(nodeType == NodeType.CC) {
      s"""    mhartid: ${Seq.tabulate(cpuNum)(i => i + clusterId).map(_.toString).reduce((a:String, b:String) => s"$a, $b")}
         |""".stripMargin
    } else {
      ""
    }

    val hdAttrStr = if(nodeType == NodeType.HI) {
      s"""    default_iocu: $defaultHni
         |""".stripMargin
    } else {
      ""
    }

    val addrStr = if(nodeType == NodeType.HI && !defaultHni || nodeType == NodeType.CC) {
      s"""    address: (0x${addressRange._1.toHexString}, 0x${(addressRange._2 - 1).toHexString})
         |""".stripMargin
    } else {
      ""
    }

    head + frdsStr + snDevStr + ccAttrStr + hdAttrStr + addrStr + "  }\n"
  }
}