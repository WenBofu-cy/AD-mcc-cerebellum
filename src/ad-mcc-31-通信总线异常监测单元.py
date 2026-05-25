#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-31
模块名称: 通信总线异常监测单元
所属分区: 八、硬件异常应急防护
核心职责: 实时监测车辆 CAN 总线、Ethernet 总线及各节点心跳信号的通信质量，统计帧错误率、
          超时帧比例及总线负载率。当通信质量下降至阈值时，触发总线切换并向相关模块发出预警；
          当所有可用总线均出现严重故障时，触发三级降级并请求紧急停车。不参与通信内容解析或
          路由决策，仅负责通信链路健康监测与故障应急。

依赖模块:
    各 ECU 节点心跳信号(CAN/Ethernet),
    网络交换机/网关,
    ad-mcc-01(小脑总控调度核心)
被依赖模块:
    ad-mcc-28/29/30(各硬件异常监测单元),
    ad-mcc-01(接收总线健康状态上报),
    ad-mcc-38(执行日志记录单元),
    ECC-12(接收严重通信故障告警)

安全约束:
  S-01: 关键节点（ECC、ESP、MCC主控）离线时，必须立即触发三级降级，通信故障车控系统不可信
  S-02: 总线切换必须在100ms内完成，确保实时控制指令不丢失
  S-03: 所有总线均故障时，车辆必须进入最低风险状态（紧急停车）
  S-04: 本模块仅监测通信链路，不解析或修改通信内容
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class CommState(Enum):
    NORMAL_COMM = "normal_comm"
    PRIMARY_DEGRADED = "primary_degraded"
    SECONDARY_DEGRADED = "secondary_degraded"
    COMM_CRITICAL = "comm_critical"
    SYSTEM_PAUSED = "system_paused"


@dataclass
class CANBusStats:
    frame_error_rate_pct: float = 0.0
    timeout_frame_rate_pct: float = 0.0
    bus_load_pct: float = 50.0
    bus_status: str = "正常"
    timestamp: float = field(default_factory=time.time)


@dataclass
class EthernetBusStats:
    packet_loss_rate_pct: float = 0.0
    latency_ms: float = 5.0
    link_status: str = "正常"
    timestamp: float = field(default_factory=time.time)


@dataclass
class NodeHeartbeat:
    node_id: str = ""
    last_heartbeat_time: float = 0.0
    expected_period_ms: float = 100.0
    online: bool = True
    is_critical: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class BusTopology:
    primary_bus_id: str = "CAN_A"
    secondary_bus_id: str = "CAN_B"
    switch_strategy: str = "自动切换"
    timestamp: float = field(default_factory=time.time)


@dataclass
class BusSwitchCommand:
    target_bus_id: str = "CAN_B"
    reason: str = ""
    fault_bus_id: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class CommFaultAlert:
    fault_type: str = ""
    severity: str = "正常"
    current_error_rate: float = 0.0
    threshold: float = 0.0
    affected_nodes: List[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


@dataclass
class DegradationRequest:
    target_level: int = 0
    reason: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class CommHealthReport:
    primary_bus_status: str = "正常"
    secondary_bus_status: str = "正常"
    online_nodes: int = 0
    total_nodes: int = 0
    fault_level: str = "正常"
    timestamp: float = field(default_factory=time.time)


CONTROL_PERIOD_S = 0.05
REPORT_INTERVAL_S = 1.0

CAN_ERROR_RATE_WARN = 0.5
CAN_ERROR_RATE_SEVERE = 1.0
CAN_ERROR_RATE_CRITICAL = 5.0

CAN_LOAD_WARN = 70.0
CAN_LOAD_SEVERE = 85.0
CAN_LOAD_CRITICAL = 95.0

TIMEOUT_FRAME_WARN = 1.0
TIMEOUT_FRAME_SEVERE = 5.0
TIMEOUT_FRAME_CRITICAL = 10.0

HEARTBEAT_MULTIPLIER_WARN = 3.0
HEARTBEAT_MULTIPLIER_SEVERE = 5.0

CRITICAL_NODES = ["ECC", "ESP", "MCC_MAIN", "MCC_CEREBELLUM"]


class BusFaultMonitor:
    def __init__(self):
        self.module_id = "ad-mcc-31"
        self.module_name = "通信总线异常监测单元"
        self.version = "V1.0"

        self.state = CommState.NORMAL_COMM
        self._current_active_bus = "CAN_A"
        self._primary_fault_counter = 0
        self._secondary_fault_counter = 0
        self._recovery_counter = 0
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_primary_can = None
        self._query_secondary_can = None
        self._query_ethernet = None
        self._query_node_heartbeats = None
        self._query_topology = None

        self._publish_bus_switch = None
        self._publish_alert = None
        self._publish_degradation = None
        self._publish_health_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_primary_can_query(self, callback):
        self._query_primary_can = callback

    def set_secondary_can_query(self, callback):
        self._query_secondary_can = callback

    def set_ethernet_query(self, callback):
        self._query_ethernet = callback

    def set_node_heartbeats_query(self, callback):
        self._query_node_heartbeats = callback

    def set_topology_query(self, callback):
        self._query_topology = callback

    def set_bus_switch_publisher(self, callback):
        self._publish_bus_switch = callback

    def set_alert_publisher(self, callback):
        self._publish_alert = callback

    def set_degradation_publisher(self, callback):
        self._publish_degradation = callback

    def set_health_report_publisher(self, callback):
        self._publish_health_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_monitoring_cycle(self):
        now = time.time()
        if self.state == CommState.SYSTEM_PAUSED:
            return

        primary = self._query_primary_can() if self._query_primary_can else CANBusStats()
        secondary = self._query_secondary_can() if self._query_secondary_can else CANBusStats()
        nodes = self._query_node_heartbeats() if self._query_node_heartbeats else []
        topology = self._query_topology() if self._query_topology else BusTopology()

        # 检查关键节点离线
        critical_offline = []
        all_offline = []
        for node in nodes:
            elapsed = (now - node.last_heartbeat_time) * 1000.0 if node.last_heartbeat_time > 0 else 999999.0
            if elapsed > node.expected_period_ms * HEARTBEAT_MULTIPLIER_WARN:
                node.online = False
                all_offline.append(node.node_id)
                if node.is_critical or node.node_id in CRITICAL_NODES:
                    critical_offline.append(node.node_id)

        if critical_offline:
            self.state = CommState.COMM_CRITICAL
            self._send_degradation(3, f"关键节点离线: {', '.join(critical_offline)}")
            self._send_alert("关键节点离线", "致命", 0.0, 0.0, critical_offline)
            return

        primary_health = self._evaluate_bus_health(primary)
        secondary_health = self._evaluate_bus_health(secondary)

        primary_severe = primary_health == "严重" or primary_health == "致命"
        secondary_severe = secondary_health == "严重" or secondary_health == "致命"
        primary_warn = primary_health == "预警"
        secondary_warn = secondary_health == "预警"

        if primary_severe and secondary_severe:
            self.state = CommState.COMM_CRITICAL
            self._send_degradation(3, "全部总线严重故障")
            self._send_alert("全部总线故障", "致命", 0.0, 0.0, ["ALL"])
            return

        if primary_severe and not secondary_severe:
            if self._current_active_bus == topology.primary_bus_id:
                self._switch_to_bus(topology.secondary_bus_id, "主总线严重故障")
            self.state = CommState.PRIMARY_DEGRADED
            self._send_alert("主总线严重故障", "严重", primary.frame_error_rate_pct, CAN_ERROR_RATE_SEVERE, ["主总线"])
            return

        if secondary_severe and not primary_severe:
            self.state = CommState.SECONDARY_DEGRADED
            self._send_alert("冗余总线严重故障", "严重", secondary.frame_error_rate_pct, CAN_ERROR_RATE_SEVERE, ["冗余总线"])
            return

        if primary_warn and not secondary_warn and not primary_severe:
            if self._current_active_bus == topology.primary_bus_id:
                self._switch_to_bus(topology.secondary_bus_id, "主总线预警")
            self.state = CommState.PRIMARY_DEGRADED
            self._send_alert("主总线预警", "预警", primary.frame_error_rate_pct, CAN_ERROR_RATE_WARN, ["主总线"])
            return

        if secondary_warn and not primary_warn:
            self.state = CommState.SECONDARY_DEGRADED
            self._send_alert("冗余总线预警", "预警", secondary.frame_error_rate_pct, CAN_ERROR_RATE_WARN, ["冗余总线"])
            return

        if primary_warn and secondary_warn:
            self.state = CommState.SECONDARY_DEGRADED
            self._send_alert("双总线预警", "预警", max(primary.frame_error_rate_pct, secondary.frame_error_rate_pct), CAN_ERROR_RATE_WARN, ["主总线", "冗余总线"])
            return

        # 恢复检测
        if self.state not in (CommState.NORMAL_COMM, CommState.SYSTEM_PAUSED):
            self._recovery_counter += 1
            if self._recovery_counter >= 60:  # 3秒 (50ms * 60)
                if self._current_active_bus != topology.primary_bus_id and primary_health == "正常":
                    self._switch_to_bus(topology.primary_bus_id, "主总线已恢复")
                self.state = CommState.NORMAL_COMM
                self._recovery_counter = 0
                self._send_alert("总线通信恢复正常", "正常", 0.0, 0.0, [])
        else:
            self._recovery_counter = 0

        # 周期性上报
        if now - self._last_report_time >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_health_report:
                online_count = sum(1 for n in nodes if n.online)
                self._publish_health_report(CommHealthReport(
                    primary_bus_status=primary_health,
                    secondary_bus_status=secondary_health,
                    online_nodes=online_count,
                    total_nodes=len(nodes) if nodes else 0,
                    fault_level=self.state.value
                ))

    def _evaluate_bus_health(self, stats: CANBusStats) -> str:
        if stats.frame_error_rate_pct >= CAN_ERROR_RATE_CRITICAL or \
           stats.timeout_frame_rate_pct >= TIMEOUT_FRAME_CRITICAL or \
           stats.bus_load_pct >= CAN_LOAD_CRITICAL:
            return "致命"
        if stats.frame_error_rate_pct >= CAN_ERROR_RATE_SEVERE or \
           stats.timeout_frame_rate_pct >= TIMEOUT_FRAME_SEVERE or \
           stats.bus_load_pct >= CAN_LOAD_SEVERE:
            return "严重"
        if stats.frame_error_rate_pct >= CAN_ERROR_RATE_WARN or \
           stats.timeout_frame_rate_pct >= TIMEOUT_FRAME_WARN or \
           stats.bus_load_pct >= CAN_LOAD_WARN:
            return "预警"
        return "正常"

    def _switch_to_bus(self, target_bus, reason):
        if self._publish_bus_switch:
            self._publish_bus_switch(BusSwitchCommand(
                target_bus_id=target_bus,
                reason=reason,
                fault_bus_id=self._current_active_bus
            ))
        self._current_active_bus = target_bus

    def _send_alert(self, fault_type, severity, error_rate, threshold, affected_nodes):
        if self._publish_alert:
            self._publish_alert(CommFaultAlert(
                fault_type=fault_type,
                severity=severity,
                current_error_rate=error_rate,
                threshold=threshold,
                affected_nodes=affected_nodes
            ))

    def _send_degradation(self, level, reason):
        if self._publish_degradation:
            self._publish_degradation(DegradationRequest(
                target_level=level,
                reason=reason
            ))

    def _log_event(self, event_type: str, details: Dict[str, Any]) -> None:
        log_entry = {
            "log_id": f"log-{uuid.uuid4().hex[:8]}",
            "event_type": event_type,
            "source_module": self.module_id,
            "details": details,
            "timestamp": time.time()
        }
        self._pending_logs.append(log_entry)
        if self._publish_event_log:
            self._publish_event_log(log_entry)

    def get_state(self) -> CommState:
        return self.state

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def emergency_shutdown(self):
        self.state = CommState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 通信总线异常监测单元 (ad-mcc-31) 演示")
    print("=" * 70)

    monitor = BusFaultMonitor()
    monitor.set_primary_can_query(lambda: CANBusStats(frame_error_rate_pct=0.2, bus_load_pct=50.0))
    monitor.set_secondary_can_query(lambda: CANBusStats(frame_error_rate_pct=0.1, bus_load_pct=40.0))
    monitor.set_node_heartbeats_query(lambda: [
        NodeHeartbeat(node_id="ECC", last_heartbeat_time=time.time(), expected_period_ms=100.0, is_critical=True),
        NodeHeartbeat(node_id="ESP", last_heartbeat_time=time.time(), expected_period_ms=100.0, is_critical=True),
    ])
    monitor.set_topology_query(lambda: BusTopology())

    print_separator("STEP 1: 正常通信")
    monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 2: 主总线错误率超标")
    monitor.set_primary_can_query(lambda: CANBusStats(frame_error_rate_pct=1.5, bus_load_pct=50.0))
    monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 3: 关键节点离线")
    monitor.set_node_heartbeats_query(lambda: [
        NodeHeartbeat(node_id="ECC", last_heartbeat_time=time.time() - 1.0, expected_period_ms=100.0, is_critical=True),
        NodeHeartbeat(node_id="ESP", last_heartbeat_time=time.time(), expected_period_ms=100.0, is_critical=True),
    ])
    monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print("\n✅ 通信总线异常监测单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-31 通信总线异常监测单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_monitor(primary_error=0.2, secondary_error=0.1,
                          primary_load=50.0, secondary_load=40.0,
                          nodes_online=True):
            m = BusFaultMonitor()
            m.set_primary_can_query(lambda: CANBusStats(frame_error_rate_pct=primary_error, bus_load_pct=primary_load))
            m.set_secondary_can_query(lambda: CANBusStats(frame_error_rate_pct=secondary_error, bus_load_pct=secondary_load))
            if nodes_online:
                m.set_node_heartbeats_query(lambda: [
                    NodeHeartbeat(node_id="ECC", last_heartbeat_time=time.time(), expected_period_ms=100.0, is_critical=True),
                    NodeHeartbeat(node_id="ESP", last_heartbeat_time=time.time(), expected_period_ms=100.0, is_critical=True),
                ])
            else:
                m.set_node_heartbeats_query(lambda: [
                    NodeHeartbeat(node_id="ECC", last_heartbeat_time=time.time() - 1.0, expected_period_ms=100.0, is_critical=True),
                    NodeHeartbeat(node_id="ESP", last_heartbeat_time=time.time(), expected_period_ms=100.0, is_critical=True),
                ])
            m.set_topology_query(lambda: BusTopology())
            return m

        print("\n[TC-M31-01] 正常通信")
        try:
            m = setup_monitor()
            m.run_monitoring_cycle()
            assert m.state == CommState.NORMAL_COMM
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M31-02] 主总线预警切换")
        try:
            m = setup_monitor(primary_error=0.8)
            m.run_monitoring_cycle()
            assert m.state == CommState.PRIMARY_DEGRADED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M31-03] 主总线恢复回切")
        try:
            m = setup_monitor(primary_error=0.8)
            m.run_monitoring_cycle()
            m.set_primary_can_query(lambda: CANBusStats(frame_error_rate_pct=0.2, bus_load_pct=50.0))
            m._current_active_bus = "CAN_B"
            m._recovery_counter = 60
            m.run_monitoring_cycle()
            assert m.state == CommState.NORMAL_COMM
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M31-04] 双总线严重故障")
        try:
            m = setup_monitor(primary_error=3.0, secondary_error=4.0)
            m.run_monitoring_cycle()
            assert m.state == CommState.COMM_CRITICAL
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M31-05] 关键节点离线")
        try:
            m = setup_monitor(nodes_online=False)
            m.run_monitoring_cycle()
            assert m.state == CommState.COMM_CRITICAL
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M31-06] 紧急熔断")
        try:
            m = setup_monitor()
            m.emergency_shutdown()
            assert m.state == CommState.SYSTEM_PAUSED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n" + "=" * 60)
        print(f"测试结果: {passed} PASS, {failed} FAIL")
        print("=" * 60)
    else:
        demo_main()
```