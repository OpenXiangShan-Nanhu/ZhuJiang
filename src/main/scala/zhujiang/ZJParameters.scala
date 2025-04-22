package zhujiang

import chisel3._
import chisel3.util.log2Ceil
import dongjiang.DJParam
import org.chipsalliance.cde.config.{Field, Parameters}
import xijiang.c2c.C2cParams
import xijiang.tfb.TrafficBoardParams
import xijiang.tfs.TrafficSimParams
import xijiang.{Node, NodeParam, NodeType}
import xs.utils.debug.HardwareAssertionKey
import zhujiang.chi._
import zhujiang.device.dma.DmaParams

import scala.collection.mutable

case object ZJParametersKey extends Field[ZJParameters]

object ZhujiangGlobal {
  private val islandPool = mutable.Map[String, Seq[Node]]()
  private val islandPtrMap = mutable.Map[String, RawModule]()

  def addRing(tag:String, island:RawModule):Unit = {
    if(!islandPtrMap.contains(tag)) {
      islandPtrMap.addOne(tag, island)
    }
  }

  private def decorate(key:String, body:Seq[String]): Seq[String] = {
    s"$key = {\n" +: body.map(b => s"  $b") :+ "},\n"
  }

  def getRingDesc(name:String):Seq[String] = {
    val path = islandPtrMap(name).pathName
    val snf = islandPool(name).filter(_.nodeType == NodeType.S)
    val rni = islandPool(name).filter(_.nodeType == NodeType.RI)
    val hnf = islandPool(name).filter(_.nodeType == NodeType.HF).groupBy(_.bankId).toSeq
    val snfStr = snf.map(n => s"{\"$path.${n.deviceName}\", {0x${n.nodeId.toHexString}}},\n")
    val rniStr = rni.map(n => s"{\"$path.${n.deviceName}\", {0x${n.nodeId.toHexString}}},\n")
    val hnfStrSeq = hnf.map({ case(_, ns) =>
      val idStr = ns.map(n => s"0x${n.nodeId.toHexString}, ").reduce(_ + _)
      s"{\"$path.${ns.head.deviceName}\", {$idStr}},\n"
    })
    decorate("snf", snfStr) ++ decorate("rni", rniStr) ++ decorate("hnf", hnfStrSeq)
  }

  def getIsland(nidBits: Int, aidBits: Int, lns: Seq[NodeParam], csb: Int, lanAddrBits:Int, tag: String): Seq[Node] = {
    if(!islandPool.contains(tag)) islandPool.addOne(tag -> getRing(lns, csb, nidBits, aidBits, lanAddrBits))
    islandPool(tag)
  }

  private def getBankBits(nodeParams: Seq[NodeParam]): Int = {
    val maxBank = nodeParams.map(_.bankId).max
    log2Ceil(1 + maxBank)
  }

  private def getRing(nodeParams: Seq[NodeParam], cpuSpaceBits: Int, nidBits: Int, aidBits: Int, lanAddrBits:Int): Seq[Node] = {
    require(nodeParams.size >= 3)
    var ccId: Long = 0

    val hnfs = nodeParams.filter(_.nodeType == NodeType.HF)

    val hnfBankBits = if(hnfs.length > 1) getBankBits(hnfs) else 0

    require(nodeParams.count(n => n.nodeType == NodeType.HI && n.defaultHni) == 1)

    var ccid = 0
    var rfid = 0
    var riid = 0
    var hfid = 0
    var hiid = 0
    var sid = 0
    var pid = 0

    def getDomainId(nt: Int): Int = {
      nt match {
        case NodeType.CC => ccid = ccid + 1; ccid - 1
        case NodeType.RF => rfid = rfid + 1; rfid - 1
        case NodeType.RI => riid = riid + 1; riid - 1
        case NodeType.HF => hfid = hfid + 1; hfid - 1
        case NodeType.HI => hiid = hiid + 1; hiid - 1
        case NodeType.S => sid = sid + 1; sid - 1
        case _ => pid = pid + 1; pid - 1
      }
    }

    val lanAddrMask = (0x1L << lanAddrBits) - 1
    val cpuAddrMask = (0x1L << cpuSpaceBits) - 1

    val nodes = for((np, idx) <- nodeParams.zipWithIndex) yield {
      val addrSets = np.nodeType match {
        case NodeType.HI => np.addrSets
        case NodeType.S  => np.addrSets
        case NodeType.CC => Seq((ccId << cpuSpaceBits, lanAddrMask ^ cpuAddrMask))
        case _           => Seq((0x0L, 0x0L))
      }
      val n = Node(
        attr = np.attr,
        nodeType = np.nodeType,
        nidBits = nidBits,
        aidBits = aidBits,
        ringSize = nodeParams.length,
        globalId = idx,
        domainId = getDomainId(np.nodeType),
        bankId = if(np.nodeType == NodeType.HF) np.bankId else 0,
        hfpId = if(np.nodeType == NodeType.HF) np.hfpId else 0,
        bankBits = if(np.nodeType == NodeType.HF) hnfBankBits else 0,
        cpuNum = if(np.nodeType == NodeType.CC) np.cpuNum else 0,
        clusterId = if(np.nodeType == NodeType.CC) ccId.toInt else 0,
        addrSets = addrSets,
        defaultHni = if(np.nodeType == NodeType.HI) np.defaultHni else false,
        outstanding = if(np.nodeType == NodeType.HI || np.nodeType == NodeType.CC || np.nodeType == NodeType.S) np.outstanding else 0,
        socket = np.socket
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

    // Select hnf to cc friends
    val hfNodes = nodes.filter(n => n.nodeType == NodeType.HF)
    val hfGroupsMaps = hfNodes.groupBy(_.bankId)
    for((_, hfs) <- hfGroupsMaps) {
      require(hfs.map(_.hfpId).distinct.length == hfs.length)
      if(hfs.length == 1) {
        hfs.head.friends = nodes.filterNot(n => n.nodeType == NodeType.HF || n.nodeType == NodeType.HI || n.nodeType == NodeType.P)
      } else {
        val hfsPos = hfs.map(d => nodes.indexOf(d))
        val idxMin = hfsPos.min
        val idxMax = hfsPos.max
        val segment0 = nodes.slice(idxMin + 1, idxMax)
        val segment1 = nodes.slice(idxMax + 1, nodes.length) ++ nodes.slice(0, idxMin)
        val half0 = segment0.length / 2
        val half1 = segment1.length / 2
        val friendsOfIdxMin = segment0.slice(0, half0) ++ segment1.slice(half1, segment1.length)
        val friendsOfIdxMax = segment1.slice(0, half1) ++ segment0.slice(half0, segment0.length)
        if(idxMin == hfsPos.head) {
          hfs.head.friends = friendsOfIdxMin.filterNot(n => n.nodeType == NodeType.HF || n.nodeType == NodeType.HI || n.nodeType == NodeType.P)
          hfs.last.friends = friendsOfIdxMax.filterNot(n => n.nodeType == NodeType.HF || n.nodeType == NodeType.HI || n.nodeType == NodeType.P)
        } else {
          hfs.head.friends = friendsOfIdxMax.filterNot(n => n.nodeType == NodeType.HF || n.nodeType == NodeType.HI || n.nodeType == NodeType.P)
          hfs.last.friends = friendsOfIdxMin.filterNot(n => n.nodeType == NodeType.HF || n.nodeType == NodeType.HI || n.nodeType == NodeType.P)
        }
      }
    }

    // Select cc/rni to hnf friends
    val rnNodes = nodes.filter(n => n.nodeType == NodeType.CC || n.nodeType == NodeType.RI)
    require(hnfs.nonEmpty)
    val hfOnePort = hnfs.map(_.hfpId).distinct.length == 1
    for(rn <- rnNodes){
      if(hfOnePort) {
        rn.friends = nodes.filter(n => n.nodeType == NodeType.HF)
      } else {
        val rnPos = nodes.indexOf(rn)
        hfGroupsMaps.values.foreach { hfGroup =>
          val hfGroupPos = hfGroup.map(d => nodes.indexOf(d))
          val hfDistance = hfGroupPos.map(d => math.abs(d - rnPos))
          val hfMinIndex = hfDistance.indexOf(hfDistance.min)
          rn.friends = rn.friends ++ Seq(hfGroup(hfMinIndex))
        }
      }
      require(rn.friends.length == hfGroupsMaps.toSeq.length)
    }
    nodes
  }
}

case class ZJParameters(
  ciSuffix: String = "",
  modulePrefix: String = "",
  nodeNidBits: Int = 5,
  nodeAidBits: Int = 3,
  ciIdBits: Int = 4,
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
  hnxBankOff: Int = 12,
  hnxCgThreshold: Int = 32,
  cpuSpaceBits: Int = 20,
  reqEjectBufDepth: Int = 5,
  externalInterruptNum: Int = 32,
  clusterCacheSizeInB: Int = 512 * 1024,
  cacheSizeInB: Int = 16 * 1024 * 1024,
  cacheWays: Int = 16,
  snoopFilterWays: Int = 16,
  hnxOutstanding: Int = 64 * 4,
  hnxDirSRAMBank: Int = 2,
  hnxPipelineDepth: Int = 0,
  splitFlit:Boolean = true,
  r2rPos: Seq[Int] = Seq(),
  nodeParams: Seq[NodeParam] = Seq(),
  dmaParams: DmaParams = DmaParams(),
  c2cParams: C2cParams = C2cParams(),
  tfbParams: Option[TrafficBoardParams] = Some(TrafficBoardParams()),
  tfsParams: Option[TrafficSimParams] = None,
  djParamsOpt: Option[DJParam] = None,
  hasMbist: Boolean = false,
) {
  lazy val cachelineBytes = 64
  lazy val requestAddrBits = 48
  lazy val snoopAddrBits = requestAddrBits - 3
  lazy val nodeIdBits: Int = nodeNidBits + nodeAidBits
  lazy val beBits: Int = dataBits / 8
  lazy val dataCheckBits: Int = if(DC) dataBits / 8 else 0
  lazy val poisonBits: Int = if(P) dataBits / 64 else 0
  require(nodeIdBits >= 7 && nodeIdBits <= 11)
  private lazy val nrX = nodeParams.filter(_.nodeType == NodeType.HF).groupBy(_.bankId).size
  private lazy val nrC = nodeParams.count(_.nodeType == NodeType.CC)
  private lazy val nrP = nodeParams.filter(_.nodeType == NodeType.CC).map(_.cpuNum).sum
  private lazy val nrD = nodeParams.count(_.nodeType == NodeType.RI)
  private lazy val nrM = nodeParams.count(_.nodeType == NodeType.S)
  private lazy val nrG = nodeParams.count(_.nodeType == NodeType.HI)
  private lazy val cacheSizeInMiB = cacheSizeInB / 1024 / 1024
  lazy val ciName: String = s"ZCI${nrX}X${nrC}C${nrP}P${nrD}D${nrM}M${nrG}G$cacheSizeInMiB$ciSuffix"
  lazy val ringName: String = s"ZRING${nrX}X${nrC}C${nrP}P${nrD}D${nrM}M${nrG}G$cacheSizeInMiB$ciSuffix"

  lazy val island: Seq[Node] = ZhujiangGlobal.getIsland(nodeNidBits, nodeAidBits, nodeParams, cpuSpaceBits, requestAddrBits - ciIdBits, ciName)

  private lazy val bank = nodeParams.filter(_.hfpId == 0).count(_.nodeType == NodeType.HF)
  private lazy val cores = nodeParams.count(_.nodeType == NodeType.CC)
  private lazy val originSfSizeInB = clusterCacheSizeInB * 2 * cores / bank
  private lazy val sfSetSize = cachelineBytes * snoopFilterWays
  private lazy val fixedSfSets = 1 << log2Ceil(originSfSizeInB / sfSetSize)
  private lazy val fixedSfSizeInB =  fixedSfSets * sfSetSize
  lazy val djParams = djParamsOpt.getOrElse(DJParam(
    addressBits = requestAddrBits,
    llcSizeInB = cacheSizeInB / bank,
    sfSizeInB = fixedSfSizeInB,
    llcWays = cacheWays,
    sfWays = snoopFilterWays,
    nrDSBank = hnxDirSRAMBank * 2,
    nrDirBank = hnxDirSRAMBank,
    nrPoS = hnxOutstanding / bank,
    dataBufSizeInByte = 64 * hnxOutstanding / bank,
  ))
}

trait HasZJParams {
  implicit val p: Parameters
  lazy val zjParams = p(ZJParametersKey)
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
  lazy val nodeNidBits = zjParams.nodeNidBits
  lazy val hasTfb = zjParams.tfbParams.isDefined
  lazy val hnxBankOff = zjParams.hnxBankOff
  lazy val ciIdBits = zjParams.ciIdBits
  lazy val clusterIdBits = zjParams.clusterIdBits
  lazy val cpuIdBits = clusterIdBits - ciIdBits
  lazy val hasMbist = zjParams.hasMbist

  lazy val rreqFlitBits = new RReqFlit()(p).getWidth
  lazy val hreqFlitBits = new HReqFlit()(p).getWidth
  lazy val respFlitBits = new RespFlit()(p).getWidth
  lazy val snoopFlitBits = new SnoopFlit()(p).getWidth
  lazy val dataFlitBits = new DataFlit()(p).getWidth
  lazy val ringHrqFlitBits = hreqFlitBits.max(snoopFlitBits)
  lazy val maxFlitBits = Seq(rreqFlitBits, respFlitBits, snoopFlitBits, dataFlitBits, hreqFlitBits).max

  lazy val debugFlitBits = niw + niw + p(HardwareAssertionKey).maxInfoBits
}

class ZJBundle(implicit val p: Parameters) extends Bundle with HasZJParams

class ZJModule(implicit val p: Parameters) extends Module with HasZJParams {
  override def resetType = Module.ResetType.Asynchronous
}

class ZJRawModule(implicit val p: Parameters) extends RawModule with HasZJParams