package zhujiang

import chisel3._
import chisel3.util.log2Ceil
import org.chipsalliance.cde.config.{Field, Parameters}
import xijiang.c2c.C2cParams
import xijiang.tfb.TrafficBoardParams
import xijiang.tfs.TrafficSimParams
import xijiang.{Node, NodeParam, NodeType}

import scala.math.abs

case object ZJParametersKey extends Field[ZJParameters]

object ZhujiangGlobal {
  var nodeNidBits: Int = 0
  var nodeAidBits: Int = 0
  var localNodeParams: Seq[NodeParam] = Seq()
  var csnNodeParams: Seq[NodeParam] = Seq()
  var raw: Long = 0
  private var initialized = false

  lazy val localRing: Seq[Node] = if(localNodeParams.nonEmpty) getRing(localNodeParams, false) else Seq()
  lazy val csnRing: Seq[Node] = if(csnNodeParams.nonEmpty) getRing(csnNodeParams, true) else Seq()

  def initialize(nidBits: Int, aidBits: Int, lns: Seq[NodeParam], cns: Seq[NodeParam], paddrBits: Long): Unit = {
    if(!initialized) {
      nodeNidBits = nidBits
      nodeAidBits = aidBits
      localNodeParams = lns
      csnNodeParams = cns
      initialized = true
      raw = paddrBits
    }
  }

  private def getBankBits(nodeParams: Seq[NodeParam]): Int = {
    val maxBank = nodeParams.map(_.bankId).max
    log2Ceil(1 + maxBank)
  }

  private def getRing(nodeParams: Seq[NodeParam], csn: Boolean): Seq[Node] = {
    require(nodeParams.size >= 3)
    var ccId: Long = 0
    val rnfs = nodeParams.filter(_.nodeType == NodeType.RF)
    val sns = nodeParams.filter(_.nodeType == NodeType.S)
    val hnfs = nodeParams.filter(_.nodeType == NodeType.HF)

    val rnfBankBits = if(csn && rnfs.length > 1) getBankBits(rnfs) else 0
    val snBankBits = if(!csn && sns.length > 1) getBankBits(sns) else 0
    val hnfBankBits = if(hnfs.length > 1) getBankBits(hnfs) else 0

    val mmioBase = 1L << (raw - 1)

    if(!csn) require(nodeParams.count(n => n.nodeType == NodeType.HI && n.defaultHni) == 1)
    val nodes = for((np, idx) <- nodeParams.zipWithIndex) yield {
      val ccAddr = ((ccId << 16) + mmioBase, ((ccId + np.cpuNum) << 16) + mmioBase)
      val hiAddr = (np.addressRange._1 + mmioBase, np.addressRange._2 + mmioBase)
      val n = Node(
        attr = np.attr,
        nodeType = np.nodeType,
        csnNode = csn,
        nidBits = nodeNidBits,
        aidBits = nodeAidBits,
        ringSize = nodeParams.length,
        globalId = idx,
        splitFlit = np.splitFlit,
        bankId = if(np.nodeType == NodeType.S || np.nodeType == NodeType.HF || np.nodeType == NodeType.RF && csn) np.bankId else 0,
        bankBits = if(np.nodeType == NodeType.RF) {
          rnfBankBits
        } else if(np.nodeType == NodeType.HF) {
          hnfBankBits
        } else if(np.nodeType == NodeType.S) {
          snBankBits
        } else {
          0
        },
        mainMemory = if(np.nodeType == NodeType.S) np.mainMemory else false,
        cpuNum = if(np.nodeType == NodeType.CC) np.cpuNum else 0,
        clusterId = if(np.nodeType == NodeType.CC) ccId.toInt else 0,
        addressRange = if(np.nodeType == NodeType.CC) {
          ccAddr
        } else if(np.nodeType == NodeType.CC) {
          hiAddr
        } else {
          (0L, 0L)
        },
        defaultHni = if(np.nodeType == NodeType.HI) np.defaultHni else false,
        outstanding = if(np.nodeType == NodeType.HI || np.nodeType == NodeType.CC || np.nodeType == NodeType.S) np.outstanding else 0,
      )
      if(np.nodeType == NodeType.CC) ccId = ccId + np.cpuNum
      n
    }

    for((n, i) <- nodes.zipWithIndex) {
      val ns = nodes.slice(i + 1, nodes.length) ++ nodes.slice(0, i)
      val odd = i % 2 == 1
      val half = if(odd) (ns.length + 1) / 2 else ns.length / 2
      n.rightNodes = ns.slice(0, half)
      n.leftNodes = ns.slice(half, ns.length).reverse
    }

    if(!csn) {
      val dcuNodes = nodes.filter(n => n.nodeType == NodeType.S && !n.mainMemory)
      val dcuGroupsMaps = dcuNodes.groupBy(_.bankId)
      for((_, dcus) <- dcuGroupsMaps) {
        require(dcus.length <= 2)
        if(dcus.length == 1) {
          dcus.head.friends = nodes
        } else {
          val dcusPos = dcus.map(d => nodes.indexOf(d))
          val idxMin = dcusPos.min
          val idxMax = dcusPos.max
          val segment0 = nodes.slice(idxMin + 1, idxMax)
          val segment1 = nodes.slice(idxMax + 1, nodes.length) ++ nodes.slice(0, idxMin)
          val half0 = segment0.length / 2
          val half1 = segment1.length / 2
          val friendsOfIdxMin = segment0.slice(0, half0) ++ segment1.slice(half1, segment1.length)
          val friendsOfIdxMax = segment1.slice(0, half1) ++ segment0.slice(half0, segment0.length)
          if(idxMin == dcusPos.head) {
            dcus.head.friends = friendsOfIdxMin
            dcus.last.friends = friendsOfIdxMax
          } else {
            dcus.head.friends = friendsOfIdxMax
            dcus.last.friends = friendsOfIdxMin
          }
        }
      }

      val pcuNodes = nodes.filter(n => n.nodeType == NodeType.HF)
      val memNode = nodes.filter(n => n.nodeType == NodeType.S && n.mainMemory)
      require(pcuNodes.nonEmpty)
      val nto1 = pcuNodes.length > dcuNodes.length
      val dcuOfPcu = pcuNodes.map({ p =>
        if(pcuNodes.length == 1) dcuNodes
        else if(nto1) dcuNodes.filter(d => (p.bankId & ((1 << d.bankBits) - 1)) == d.bankId)
        else dcuNodes.filter(d => (d.bankId & ((1 << p.bankBits) - 1)) == p.bankId)
      })
      for((p, ds) <- pcuNodes.zip(dcuOfPcu)) {
        val bankDcuMap = ds.groupBy(_.bankId)
        val pcuPos = nodes.indexOf(p)
        p.friends = bankDcuMap.map({ case (_, n) =>
          require(n.nonEmpty)
          val disSeq = n.map(d => abs(nodes.indexOf(d) - pcuPos))
          val pos = disSeq.indexOf(disSeq.min)
          n(pos)
        }).toSeq ++ memNode
      }
    }
    nodes
  }

}

case class ZJParameters(
  ringId: Int = 0,
  modulePrefix: String = "",
  nodeNidBits: Int = 7,
  nodeAidBits: Int = 3,
  dataBits: Int = 256,
  M: Int = 0,
  PB: Int = 0,
  E: Int = 0,
  R: Int = 0,
  S: Int = 0,
  Y: Int = 0,
  DC: Boolean = false,
  P: Boolean = false,
  clusterIdBits: Int = 8,
  bankOff: Int = 12,
  ccnSpaceBits: Int = 16,
  snoopEjectBufDepth: Int = 8,
  reqEjectBufDepth: Int = 8,
  localNodeParams: Seq[NodeParam] = Seq(),
  csnNodeParams: Seq[NodeParam] = Seq(),
  c2cParams: C2cParams = C2cParams(),
  tfbParams: Option[TrafficBoardParams] = Some(TrafficBoardParams()),
  tfsParams: Option[TrafficSimParams] = None,
  injectRsvdTimerShift: Int = 8
) {
  lazy val requestAddrBits = 48
  lazy val snoopAddrBits = requestAddrBits - 3
  lazy val nodeNetBits = 1
  lazy val nodeIdBits: Int = nodeNetBits + nodeNidBits + nodeAidBits
  lazy val beBits: Int = dataBits / 8
  lazy val dataCheckBits: Int = if(DC) dataBits / 8 else 0
  lazy val poisonBits: Int = if(P) dataBits / 64 else 0
  private lazy val dwb = if(dataBits == 128) {
    234
  } else if(dataBits == 256) {
    383
  } else {
    681
  }
  lazy val reqFlitBits = 3 * (nodeIdBits - 7) + 88 + requestAddrBits + Y + M + PB + S + E.max(R)
  lazy val respFlitBits = 2 * (nodeIdBits - 7) + 65
  lazy val snoopFlitBits = 3 * (nodeIdBits - 7) + 59 + snoopAddrBits + M + E
  lazy val dataFlitBits = 3 * (nodeIdBits - 7) + dwb + Y + dataCheckBits + poisonBits
  lazy val maxFlitBits = Seq(reqFlitBits, respFlitBits, snoopFlitBits, dataFlitBits).max
  require(nodeIdBits >= 7 && nodeIdBits <= 11)

  ZhujiangGlobal.initialize(nodeNidBits, nodeAidBits, localNodeParams, csnNodeParams, requestAddrBits)
  val localRing = ZhujiangGlobal.localRing
  val csnRing = ZhujiangGlobal.csnRing
}

trait HasZJParams {
  implicit val p: Parameters
  val zjParams = p(ZJParametersKey)
  lazy val M = zjParams.M
  lazy val PB = zjParams.PB
  lazy val E = zjParams.E
  lazy val R = zjParams.R
  lazy val S = zjParams.S
  lazy val Y = zjParams.Y
  lazy val raw = zjParams.requestAddrBits
  lazy val saw = zjParams.snoopAddrBits
  lazy val niw = zjParams.nodeIdBits
  lazy val dcw = zjParams.dataCheckBits
  lazy val pw = zjParams.poisonBits
  lazy val dw = zjParams.dataBits
  lazy val bew = zjParams.beBits
  lazy val nodeAidBits = zjParams.nodeAidBits
  lazy val reqFlitBits = zjParams.reqFlitBits
  lazy val respFlitBits = zjParams.respFlitBits
  lazy val snoopFlitBits = zjParams.snoopFlitBits
  lazy val dataFlitBits = zjParams.dataFlitBits
  lazy val maxFlitBits = zjParams.maxFlitBits
  lazy val nodeNidBits = zjParams.nodeNidBits
  lazy val nodeNetBits = zjParams.nodeNetBits
  lazy val hasTfb = zjParams.tfbParams.isDefined
  lazy val bankOff = zjParams.bankOff
  lazy val clusterIdBits = zjParams.clusterIdBits
}

class ZJBundle(implicit val p: Parameters) extends Bundle with HasZJParams

class ZJModule(implicit val p: Parameters) extends Module with HasZJParams

class ZJRawModule(implicit val p: Parameters) extends RawModule with HasZJParams