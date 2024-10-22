package xijiang.tfb

import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xs.utils.FileRegisters
import zhujiang.ZJParametersKey
import zhujiang.chi.ChannelEncodings

case class TrafficBoardParams(
  timeOut: Int = 500
)

object TrafficBoardFileManager {
  def release(p: Parameters): Unit = {
    FileRegisters.add("env/tfb/include", "traffic_board.h", header, true)
    FileRegisters.add("env/tfb/src", "traffic_board.cpp", source(p), true)
  }

  def release(header_dir: String, src_dir: String, p: Parameters): Unit = {
    FileRegisters.add(header_dir, "traffic_board.h", header, dontCarePrefix = true)
    FileRegisters.add(src_dir, "traffic_board.cpp", source(p), dontCarePrefix = true)
  }

  private def getEject(nt: Int, csn: Boolean) = {
    if(csn) {
      Node(nodeType = nt).ejects.filterNot(_ == "ERQ")
    } else {
      Node(nodeType = nt).ejects
    }
  }

  val allNodeTypeMap: Seq[(String, Int, String, Seq[String])] = Seq(
    ("lcc", NodeType.CC | (0 << NodeType.width), "LOCAL_TGT_POOL", getEject(NodeType.CC, false)),
    ("lrf", NodeType.RF | (0 << NodeType.width), "LOCAL_TGT_POOL", getEject(NodeType.RF, false)),
    ("lri", NodeType.RI | (0 << NodeType.width), "LOCAL_TGT_POOL", getEject(NodeType.RI, false)),
    ("lhf", NodeType.HF | (0 << NodeType.width), "LOCAL_TGT_POOL", getEject(NodeType.HF, false)),
    ("lhi", NodeType.HI | (0 << NodeType.width), "LOCAL_TGT_POOL", getEject(NodeType.HI, false)),
    ("lsn", NodeType.S | (0 << NodeType.width), "LOCAL_TGT_POOL", getEject(NodeType.S, false)),
    ("crf", NodeType.RF | (1 << NodeType.width), "C2C_TGT_POOL", getEject(NodeType.RF, true)),
    ("chf", NodeType.HF | (1 << NodeType.width), "C2C_TGT_POOL", getEject(NodeType.HF, true)),
    ("c2c", NodeType.C | (1 << NodeType.width), "CSN_TGT_POOL", getEject(NodeType.C, true)),
  )

  val allNodeTypeDefs: String = allNodeTypeMap.map({ case (n, v, _, _) => s"#define ${n.toUpperCase}_TYPE $v\n" }).reduce(_ ++ _)

  private val nodePoolInitFn: String = allNodeTypeMap.map({ case (n, _, _, _) => s"  nodes_pool[${n.toUpperCase}_TYPE] = vector<uint16_t>();\n" }).reduce(_ ++ _)

  def header: String =
    """
      |#ifndef __TRAFFIC_BOARD_H__
      |#define __TRAFFIC_BOARD_H__
      |#include "svdpi.h"
      |
      |#ifdef __cplusplus
      |extern "C" {
      |#endif
      |
      |void tfb_register_node(short int node_id, short int node_type);
      |
      |void tfb_flit_monitor(short int node_id, short int node_type, svBit inject, char flit_type, const svBitVecVal *flit, svBit *fault);
      |
      |uint8_t tfb_step();
      |
      |uint16_t tfb_get_nodes_size(uint16_t type);
      |
      |uint16_t tfb_get_nodes(uint16_t type, uint16_t *nodes_array_ptr);
      |
      |void tfb_verbose();
      |
      |#ifdef __cplusplus
      |}
      |#endif
      |
      |#endif //__TRAFFIC_BOARD_H__
      |""".stripMargin

  def source(p: Parameters) = {
    val params = p(ZJParametersKey)
    s"""
       |#include "svdpi.h"
       |#include <cstdint>
       |#include <cstdio>
       |#include <cstring>
       |#include <list>
       |#include <unordered_map>
       |#include <vector>
       |#include <mutex>
       |#include <memory>
       |#include <sstream>
       |#include <iomanip>
       |
       |#define FLIT_SIZE ${params.maxFlitBits}
       |#define TIME_OUT ${params.tfbParams.get.timeOut}
       |#define NODE_NID_BITS ${params.nodeNidBits}
       |#define NODE_AID_BITS ${params.nodeAidBits}
       |#define NODE_NET_BITS ${params.nodeNetBits}
       |#define FLIT_BUF_SIZE ${(params.maxFlitBits + 7) / 8}
       |
       |$allNodeTypeDefs
       |
       |#define REQ ${ChannelEncodings.REQ}
       |#define RSP ${ChannelEncodings.RSP}
       |#define DAT ${ChannelEncodings.DAT}
       |#define SNP ${ChannelEncodings.SNP}
       |#define ERQ ${ChannelEncodings.ERQ}
       |
       |#define TYPE_NET_OFF ${NodeType.width}
       |#define TYPE_NET_BITS 1
       |
       |#define NODE_AID_BITS ${params.nodeAidBits}
       |#define NODE_ID_BITS ${params.nodeIdBits}
       |#define TGT_ID_OFF 4
       |#define SRC_ID_OFF (TGT_ID_OFF + NODE_ID_BITS)
       |
       |#define TFB_ERR(...)                  \\
       |  {                                   \\
       |    TrafficBoard::get_instance().info_lock.lock();                                  \\
       |    fprintf(stderr, "\\n[TFB ERROR] @ %lu: ", TrafficBoard::get_instance().global_timer); \\
       |    fprintf(stderr, __VA_ARGS__);     \\
       |    fflush(stderr);                   \\
       |    TrafficBoard::get_instance().info_lock.unlock();                                 \\
       |  }
       |#define TFB_INFO(...)                 \\
       |  {                                   \\
       |    TrafficBoard::get_instance().info_lock.lock();                                  \\
       |    fprintf(stdout, "\\n[TFB INFO] @ %lu: ", TrafficBoard::get_instance().global_timer); \\
       |    fprintf(stdout, __VA_ARGS__);     \\
       |    fflush(stdout);                   \\
       |    TrafficBoard::get_instance().info_lock.unlock();                                 \\
       |  }
       |
       |
       |using namespace std;
       |
       |inline uint64_t get_field(uint64_t vec, uint8_t offset, uint8_t width) {
       |  return (vec >> offset) & ((1 << width) - 1);
       |}
       |
       |string get_flit_str(const uint8_t *flit) {
       |  ostringstream ss;
       |  ss << hex << "0x";
       |  int i = FLIT_BUF_SIZE;
       |  while(i-- > 0) ss << setw(2) << setfill('0') << (uint16_t)flit[i];
       |  return ss.str();
       |}
       |
       |namespace traffic_board {
       |struct TrafficBoardEntry {
       |  uint64_t inject_time;
       |  uint64_t timer;
       |  uint8_t flit[FLIT_BUF_SIZE];
       |};
       |
       |typedef unordered_map<uint16_t, unordered_map<uint8_t, list<unique_ptr<TrafficBoardEntry>>>> scb_t;
       |typedef unordered_map<uint16_t, unordered_map<uint8_t, unique_ptr<mutex>>> lock_map_t;
       |
       |class TrafficBoard {
       |  private:
       |  TrafficBoard();
       |  // tgt_id -> flit_type -> list -> entry
       |  scb_t scoreboard = scb_t();
       |  // tgt_id -> flit_type -> list_lock
       |  lock_map_t locks = lock_map_t();
       |  mutex scb_lock;
       |
       |  public:
       |  mutex info_lock;
       |  unordered_map<uint16_t, vector<uint16_t>> nodes_pool;
       |  uint64_t global_timer = 0;
       |  bool verbose = false;
       |  TrafficBoard(const TrafficBoard &) = delete;
       |  TrafficBoard &operator=(const TrafficBoard &) = delete;
       |  bool step();
       |  static TrafficBoard &get_instance() {
       |    static TrafficBoard instance;
       |    return instance;
       |  }
       |  void register_node(uint16_t node_id, uint16_t node_type);
       |  svBit add_record(uint16_t node_id, uint16_t node_type, uint8_t chn, const svBitVecVal *flit);
       |  bool match_record(uint16_t node_id, uint16_t node_type, uint8_t chn, const svBitVecVal *flit);
       |  uint16_t get_tgt_c2c(uint16_t chip);
       |};
       |
       |TrafficBoard::TrafficBoard(){
       |$nodePoolInitFn
       |}
       |
       |void TrafficBoard::register_node(uint16_t node_id, uint16_t node_type) {
       |  lock_guard lg(scb_lock);
       |  if(scoreboard.count(node_id) != 0) {
       |    TFB_ERR("cannot register node 0x%x more than once!\\n", node_id);
       |    return;
       |  }
       |  if(nodes_pool.count(node_type) == 0) {
       |    TFB_ERR("unknown node type %d!\\n", node_type);
       |    return;
       |  }
       |  scoreboard[node_id] = unordered_map<uint8_t, list<unique_ptr<TrafficBoardEntry>>>();
       |  scoreboard[node_id][REQ] = list<unique_ptr<TrafficBoardEntry>>();
       |  scoreboard[node_id][RSP] = list<unique_ptr<TrafficBoardEntry>>();
       |  scoreboard[node_id][DAT] = list<unique_ptr<TrafficBoardEntry>>();
       |  scoreboard[node_id][SNP] = list<unique_ptr<TrafficBoardEntry>>();
       |  scoreboard[node_id][ERQ] = list<unique_ptr<TrafficBoardEntry>>();
       |  locks[node_id] = unordered_map<uint8_t, unique_ptr<mutex>>();
       |  locks[node_id][REQ] = make_unique<mutex>();
       |  locks[node_id][RSP] = make_unique<mutex>();
       |  locks[node_id][DAT] = make_unique<mutex>();
       |  locks[node_id][SNP] = make_unique<mutex>();
       |  locks[node_id][ERQ] = make_unique<mutex>();
       |  nodes_pool[node_type].push_back(node_id);
       |}
       |
       |#define MONITOR_ERR(cond, ...) \\
       |  if(cond) {                   \\
       |    TFB_ERR(__VA_ARGS__);      \\
       |    return 1;                  \\
       |  }
       |
       |uint16_t TrafficBoard::get_tgt_c2c(uint16_t chip) {
       |  for(const auto &c2c: nodes_pool[C2C_TYPE]) if(get_field(c2c, 0, NODE_AID_BITS) == chip) return c2c;
       |  return 0;
       |}
       |
       |svBit TrafficBoard::add_record(uint16_t node_id, uint16_t node_type, uint8_t chn, const svBitVecVal *flit) {
       |  MONITOR_ERR(scoreboard.count(node_id) == 0, "node 0x%x is sampled before registered!\\n", node_id);
       |  uint16_t tgt_id = get_field(*((const uint64_t *)flit), TGT_ID_OFF, NODE_ID_BITS);
       |  uint16_t src_id = get_field(*((const uint64_t *)flit), SRC_ID_OFF, NODE_ID_BITS);
       |  bool c2c = node_type == C2C_TYPE;
       |  bool csn = get_field(node_type, TYPE_NET_OFF, TYPE_NET_BITS) == 0x1;
       |  uint16_t final_tgt_id = tgt_id;
       |  if(verbose) {
       |    string &&flit_str = get_flit_str((const uint8_t *)flit);
       |    TFB_INFO("node_id 0x%x inject flit with src_id: 0x%x tgt_id: 0x%x on chn %d flit:\\n%s\\n", node_id, src_id, tgt_id, chn, flit_str.c_str());
       |  }
       |  if(csn && !c2c) {
       |    uint16_t src_chip = get_field(node_id, 0, NODE_AID_BITS);
       |    uint16_t tgt_chip = get_field(tgt_id, 0, NODE_AID_BITS);
       |    final_tgt_id = get_tgt_c2c(tgt_chip);
       |    MONITOR_ERR(src_chip == tgt_chip, "csn node 0x%x injects illegal flit with tgt_id: 0x%x, target chip_id cannot be chip_id of itself!\\n", node_id, tgt_id);
       |    MONITOR_ERR(final_tgt_id == 0, "csn node 0x%x injects illegal flit with tgt_id: 0x%x tgt_chip: 0x%x, target c2c cannot be found!\\n", node_id, tgt_id, tgt_chip);
       |  }
       |  MONITOR_ERR(scoreboard.count(final_tgt_id) == 0, "node 0x%x injected flit target node 0x%x is not registered on chn %d!\\n", node_id, final_tgt_id, chn);
       |  auto entry = make_unique<TrafficBoardEntry>();
       |  memcpy(entry->flit, flit, FLIT_BUF_SIZE);
       |  entry->inject_time = global_timer;
       |  entry->timer = 0;
       |  mutex *lock = locks[final_tgt_id][chn].get();
       |  lock_guard lg(*lock);
       |  auto &record_list = scoreboard[final_tgt_id][chn];
       |  record_list.push_back(std::move(entry));
       |  return 0;
       |}
       |
       |bool TrafficBoard::match_record(uint16_t node_id, uint16_t node_type, uint8_t chn, const svBitVecVal *flit) {
       |  bool found = false;
       |  mutex *lock = locks[node_id][chn].get();
       |  lock_guard lg(*lock);
       |  auto &record_list = scoreboard[node_id][chn];
       |  auto pos = record_list.begin();
       |  while(pos != record_list.end()) {
       |    found = (0 == memcmp((*pos)->flit, flit, FLIT_BUF_SIZE));
       |    if(found) break;
       |    pos++;
       |  }
       |  if(found) {
       |    record_list.erase(pos);
       |    if(verbose) {
       |      uint16_t src_id = get_field(*((const uint64_t *)flit), SRC_ID_OFF, NODE_ID_BITS);
       |      uint16_t tgt_id = get_field(*((const uint64_t *)flit), TGT_ID_OFF, NODE_ID_BITS);
       |      string &&flit_str = get_flit_str((const uint8_t *)flit);
       |      TFB_INFO("node_id: 0x%x eject flit with src_id: 0x%x tgt_id: 0x%x on chn %d flit:\\n%s\\n", node_id, src_id, tgt_id, chn, flit_str.c_str());
       |    }
       |  }
       |  return found;
       |}
       |
       |bool TrafficBoard::step() {
       |  bool time_out = false;
       |  global_timer++;
       |  for(auto &[k0, v0]: scoreboard) {
       |    for(auto &[k1, v1]: v0) {
       |      for(auto &d: v1) {
       |        d->timer++;
       |        if(d->timer > TIME_OUT) {
       |          TFB_ERR("node 0x%x chn %d inject time %lu time out!\\n", k0, k1, d->inject_time);
       |          time_out = true;
       |        }
       |      }
       |    }
       |  }
       |  return time_out;
       |}
       |}// namespace traffic_board
       |
       |using namespace traffic_board;
       |extern "C" {
       |void tfb_register_node(short int node_id, short int node_type) {
       |  TrafficBoard::get_instance().register_node(node_id, node_type);
       |}
       |
       |void tfb_flit_monitor(short int node_id, short int node_type, svBit inject, char flit_type, const svBitVecVal *flit, svBit *fault) {
       |  auto &tfb = TrafficBoard::get_instance();
       |  if(inject) {
       |    *fault = tfb.add_record(node_id, node_type, flit_type, flit);
       |  } else {
       |    bool found = tfb.match_record(node_id, node_type, flit_type, flit);
       |    uint16_t src_id = get_field(*((const uint64_t *)flit), SRC_ID_OFF, NODE_ID_BITS);
       |    uint16_t tgt_id = get_field(*((const uint64_t *)flit), TGT_ID_OFF, NODE_ID_BITS);
       |    if(!found) {
       |      string &&flit_str = get_flit_str((const uint8_t *)flit);
       |      TFB_ERR("node_id: 0x%x eject flit with src_id: 0x%x tgt_id: 0x%x not found on chn %d flit:\\n%s\\n", node_id, src_id, tgt_id, flit_type, flit_str.c_str());
       |    }
       |    *fault = found ? 0 : 1;
       |  }
       |}
       |
       |uint8_t tfb_step() {
       |  return TrafficBoard::get_instance().step();
       |}
       |
       |uint16_t tfb_get_nodes_size(uint16_t type) {
       |  const auto &tfb = TrafficBoard::get_instance();
       |  if(tfb.nodes_pool.count(type) == 0) {
       |    TFB_ERR("wrong node type %d\\n", type);
       |    return 0;
       |  }
       |  return tfb.nodes_pool.at(type).size();
       |}
       |
       |uint16_t tfb_get_nodes(uint16_t type, uint16_t *nodes_array_ptr) {
       |  const auto &tfb = TrafficBoard::get_instance();
       |  if(tfb.nodes_pool.count(type) == 0) {
       |    TFB_ERR("wrong node type %d\\n", type);
       |    return 1;
       |  }
       |  const auto &vec = tfb.nodes_pool.at(type);
       |  for(int i = 0; i < vec.size(); i++) nodes_array_ptr[i] = vec.at(i);
       |  return 0;
       |}
       |
       |void tfb_verbose() {
       |  TrafficBoard::get_instance().verbose = true;
       |}
       |}""".stripMargin
  }
}
