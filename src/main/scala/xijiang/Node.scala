package xijiang

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xijiang.router._
import xijiang.router.base.BaseRouter
import xs.utils.debug.HardwareAssertionKey
import zhujiang.chi.{MemAttr, NodeIdBundle, ReqAddrBundle}
import zhujiang.device.AxiDeviceParams

object NodeType {
  val encodingsMap = Seq(
    "CC" -> 0,
    "RF" -> 1,
    "RI" -> 2,
    "HF" -> 3,
    "HI" -> 4,
    "S" -> 5,
    "M" -> 6,
    "RH" -> 7,
    "P" -> 8
  ).toMap

  val strMap = encodingsMap.map(elm => elm._2 -> elm._1)
  val CC: Int = encodingsMap("CC")
  val RF: Int = encodingsMap("RF")
  val RI: Int = encodingsMap("RI")
  val HF: Int = encodingsMap("HF")
  val HI: Int = encodingsMap("HI")
  val S: Int = encodingsMap("S")
  val M: Int = encodingsMap("M")
  val RH: Int = encodingsMap("RH")
  val P: Int = encodingsMap("P")
  val min: Int = 0
  val max: Int = encodingsMap.values.max
  def HX: Int = HF
  def width: Int = log2Ceil(max)

  val cppDefines:String = encodingsMap.filterNot(_._1 == "P").map(elm => s"#define ${elm._1}_TYPE ${elm._2}\n").reduce(_ + _)
  def testAxiDev(nt: Int): Boolean = nt == HI || nt == RI || nt == RH || nt == S || nt == M
}

case class NodeParam(
  nodeType: Int = NodeType.P,
  bankId: Int = 0, // Only applied in HNF
  hfpId: Int = 0, // HNF port id // Only applied in HNF)
  cpuNum: Int = 1, // Only applied in CC
  addrSets: Seq[(Long, Long)] = Seq((0L, 0L)), // Only applied in HNI
  defaultHni: Boolean = false, // Only applied in HNI
  socket: String = "sync",
  axiDevParams: Option[AxiDeviceParams] = None
)

case class Node(
  nodeType: Int = NodeType.P,
  nidBits: Int = 5,
  aidBits: Int = 3,
  ringSize: Int = 3,
  globalId: Int = 0,
  domainId:Int = 0,
  bankId: Int = 0, // Only applied in HNF
  hfpId: Int = 0, // Only applied in HNF
  bankBits: Int = 1, // Only applied in HNF
  cpuNum: Int = 1, // Only applied in CCN
  clusterId: Int = 0, //Only applied in CCN
  addrSets: Seq[(Long, Long)] = Seq((0L, 0L)), // Only applied in HNI
  defaultHni: Boolean = false, // Only applied in HNI
  socket: String = "sync",
  axiDevParams: Option[AxiDeviceParams] = None
) {
  require(NodeType.min <= nodeType && nodeType <= NodeType.max)

  val nodeId = globalId << aidBits

  var leftNodes: Seq[Node] = Seq()
  var rightNodes: Seq[Node] = Seq()
  lazy val outstanding = axiDevParams.map(_.outstanding).getOrElse(8)

  def genRouter(p: Parameters): BaseRouter = {
    val res = nodeType match {
      case NodeType.CC => Module(new RnRouter(this)(p))
      case NodeType.RF => Module(new RnRouter(this)(p))
      case NodeType.RI => Module(new RnRouter(this)(p))
      case NodeType.RH => Module(new RnRouter(this)(p))
      case _ => Module(new BaseRouter(this)(p))
    }
    res.suggestName(routerName)
    res
  }

  private lazy val routerName:String = {
    val nstr = nodeType match {
      case NodeType.CC => s"ccn_$domainId"
      case NodeType.RF => "rnf"
      case NodeType.RI => "rni"
      case NodeType.RH => "rnh"
      case NodeType.HF => s"hnf_$bankId"
      case NodeType.HI => "hni"
      case NodeType.M => "mn"
      case NodeType.S => "sn"
      case _ => "pip"
    }
    s"ring_stop_${nstr}_id_0x${nodeId.toHexString}"
  }

  lazy val deviceName:String = {
    val nidStr = s"0x${nodeId.toHexString}"
    val nstr = nodeType match {
      case NodeType.CC => s"ccn_${domainId}_$nidStr"
      case NodeType.RF => s"core_${domainId}_$nidStr"
      case NodeType.RI => s"axi_to_chi_$nidStr"
      case NodeType.RH => s"axi_to_chi_$nidStr"
      case NodeType.HF => s"hnf_$bankId"
      case NodeType.HI => s"chi_to_axi_lite_$nidStr"
      case NodeType.M =>  s"mnDev"
      case NodeType.S =>  s"chi_to_axi_$nidStr"
      case _ =>           s"pip_${nodeId.toHexString}"
    }
    nstr
  }

  private def icnStrGen(pfx: String, body: String) = s"$pfx${body}_id_${nodeId.toHexString}"
  private def routerStrGen(body: String) = s"Router${body}_0x${nodeId.toHexString}"
  lazy val (routerStr, icnStr, nodeStr): (String, String, String) = nodeType match {
    case NodeType.CC => (routerStrGen("CpuCluster"), icnStrGen("", "ccn"), "CCN")
    case NodeType.RF => (routerStrGen("RequestFull"), icnStrGen("", "rnf"), "RNF")
    case NodeType.RI => (routerStrGen("RequestIo"), icnStrGen("", "rni"), "RNI")
    case NodeType.RH => (routerStrGen("RequestHigh"), icnStrGen("", "rnh"), "RNH")
    case NodeType.HF => (routerStrGen("HomeFull"), icnStrGen("", s"hnf_bank_$bankId"), "HNF")
    case NodeType.HI => (routerStrGen("HomeIo"), icnStrGen("", "hni"), "HNI")
    case NodeType.M => (routerStrGen("Misc"), icnStrGen("", "mn"), "MN")
    case NodeType.S => (routerStrGen("Subordinate"), icnStrGen("", s"sn"), "SN")
    case _ => (routerStrGen("Pipeline"), icnStrGen("", "pip"), "PIP")
  }

  private lazy val (_ejects, _injects): (Seq[String], Seq[String]) = {
    val res = nodeType match {
      case NodeType.CC => (Seq("REQ", "RSP", "DAT", "SNP"), Seq("REQ", "RSP", "DAT"))
      case NodeType.RF => (Seq("RSP", "DAT", "SNP"), Seq("REQ", "RSP", "DAT"))
      case NodeType.RI => (Seq("RSP", "DAT"), Seq("REQ", "RSP", "DAT"))
      case NodeType.RH => (Seq("RSP", "DAT"), Seq("HPR", "RSP", "DAT"))
      case NodeType.HF => (Seq("REQ", "RSP", "DAT", "HPR"), Seq("RSP", "DAT", "SNP", "ERQ"))
      case NodeType.HI => (Seq("REQ", "RSP", "DAT"), Seq("RSP", "DAT", "ERQ"))
      case NodeType.S => (Seq("ERQ", "DAT"), Seq("RSP", "DAT"))
      case _ => (Seq(), Seq())
    }
    val illegal1 = res._1.contains("REQ") && res._1.contains("ERQ") || res._1.contains("HPR") && res._1.contains("ERQ")
    val illegal2 = res._2.contains("REQ") && res._2.contains("ERQ") || res._2.contains("ERQ") && res._2.contains("HPR")
    require(!illegal1)
    require(!illegal2)
    res
  }

  def ejects(implicit p:Parameters):Seq[String] = {
    nodeType match {
      case NodeType.M => if(p(HardwareAssertionKey).enable) _ejects :+ "DBG" else _ejects
      case _ => _ejects
    }
  }

  def injects(implicit p:Parameters):Seq[String] = {
    nodeType match {
      case NodeType.CC => if(p(HardwareAssertionKey).enable) _injects :+ "DBG" else _injects
      case NodeType.RF => if(p(HardwareAssertionKey).enable) _injects :+ "DBG" else _injects
      case NodeType.HF => if(p(HardwareAssertionKey).enable && hfpId == 0) _injects :+ "DBG" else _injects
      case _ => _injects
    }
  }

  private def getLegalTgtSeq(ring: Seq[Node], chn: String): Seq[Int] = {
    import NodeType._
    val legalTgtTypeSeq = nodeType match {
      case CC => chn match {
        case "REQ" => Seq(CC, HF, HI, RH)
        case "RSP" => Seq(CC, HF, RI, HI, RH)
        case "DAT" => Seq(CC, RF, RI, HF, HI, RH)
        case "DBG" => Seq(M)
        case _ => Seq[Int]()
      }
      case RF => chn match {
        case "REQ" => Seq(CC, HF, HI)
        case "RSP" => Seq(CC, HF, HI)
        case "DAT" => Seq(CC, RF, RI, HF, HI, RH)
        case "DBG" => Seq(M)
        case _ => Seq[Int]()
      }
      case RI => chn match {
        case "REQ" => Seq(CC, HF, HI)
        case "RSP" => Seq(CC, HF, HI)
        case "DAT" => Seq(CC, HF, HI)
        case "DBG" => Seq(M)
        case _ => Seq[Int]()
      }
      case RH => chn match {
        case "HPR" => Seq(CC, HF, HI)
        case "RSP" => Seq(CC, HF, HI)
        case "DAT" => Seq(CC, HF, HI)
        case "DBG" => Seq(M)
        case _ => Seq[Int]()
      }
      case HF => chn match {
        case "RSP" => Seq(CC, RF, RI, RH)
        case "DAT" => Seq(CC, RF, RI, S, RH)
        case "SNP" => Seq(CC, RF)
        case "ERQ" => Seq(S)
        case "DBG" => Seq(M)
        case _ => Seq[Int]()
      }
      case HI => chn match {
        case "RSP" => Seq(CC, RF, RI, RH)
        case "DAT" => Seq(CC, RF, RI, RH)
        case "ERQ" => Seq(S)
        case "DBG" => Seq(M)
        case _ => Seq[Int]()
      }
      case S => chn match {
        case "RSP" => Seq(CC, RF, RI, HF, HI, RH)
        case "DAT" => Seq(CC, RF, RI, HF, HI, RH)
        case "DBG" => Seq(M)
        case _ => Seq[Int]()
      }
      case _ => Seq[Int]()
    }
    require(legalTgtTypeSeq.nonEmpty, s"node 0x${nodeId.toHexString} has no inject channel $chn")
    val res = ring.filter(n => legalTgtTypeSeq.contains(n.nodeType)).map(_.nodeId).filterNot(_ == nodeId)
    res
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

  private def hnfAddrCheck(addr: ReqAddrBundle, memAttr:MemAttr, bankOff:Int): Bool = {
    !memAttr.device && addr.checkBank(bankBits, bankId.U, bankOff)
  }

  def addrCheck(addr: ReqAddrBundle, ci: UInt):Bool = {
    val devAddrBits = addr.devAddr.getWidth
    val addrMatch = addrSets.map(elm => (addr.devAddr & elm._2.U(devAddrBits.W)) === elm._1.U(devAddrBits.W))
    addr.ci === ci && Cat(addrMatch).orR
  }

  private def hniAddrCheck(addr: ReqAddrBundle, ci: UInt, memAttr:MemAttr): Bool = {
    if(defaultHni) {
      memAttr.device
    } else {
      memAttr.device && addrCheck(addr, ci)
    }
  }

  def isReqCompleter(addr: ReqAddrBundle, ci: UInt, memAttr:MemAttr, bankOff:Int): Bool = {
    import NodeType._
    nodeType match {
      case CC => hniAddrCheck(addr, ci, memAttr)
      case HF => hnfAddrCheck(addr, memAttr, bankOff)
      case HI => hniAddrCheck(addr, ci, memAttr)
      case _ => false.B
    }
  }

  var friends = Seq[Node]()

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

    val frdsStr = if(friends.nonEmpty) {
      s"""    friends: $friendsStr
         |""".stripMargin
    } else {
      ""
    }

    val bankStr = if (nodeType == NodeType.HF) {
      s"""    bank: $bankId
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
      s"""    default_hni: $defaultHni
         |""".stripMargin
    } else {
      ""
    }

    val addrStr = if(nodeType == NodeType.HI && !defaultHni || nodeType == NodeType.CC || nodeType == NodeType.S) {
      s"""    addrSets: {
         |${addrSets.map(elm => String.format(s"      0x%0${11}X, 0x%0${11}X", elm._1, elm._2)).reduce((a:String, b:String) => s"$a\n$b")}
         |    }
         |""".stripMargin
    } else {
      ""
    }

    head + frdsStr + bankStr + ccAttrStr + hdAttrStr + addrStr + "  }\n"
  }
}