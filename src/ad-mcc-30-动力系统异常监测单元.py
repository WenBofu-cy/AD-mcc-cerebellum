#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-30
模块名称: 动力系统异常监测单元
所属分区: 八、硬件异常应急防护
核心职责: 实时监测驱动电机/发动机的输出扭矩响应延迟、温度、逆变器状态及动力系统通信状态，
          判断动力系统是否存在电气故障、过热、响应滞后或通信失效。当检测到异常时，根据故障
          严重等级触发分级降级策略：轻微异常时发出预警并限制最大输出功率；严重异常时触发降级，
          限制功率至50%并限制车速；致命异常时触发三级降级并禁止自动驾驶。不参与动力控制决策，
          仅做故障诊断与降级触发。

依赖模块:
    驱动电机控制器/发动机 ECU(CAN总线),
    ad-mcc-01(小脑总控调度核心),
    ad-mcc-31(通信总线异常监测单元)
被依赖模块:
    ad-mcc-01(接收异常告警与降级触发),
    ad-mcc-09(油门开度解算单元),
    ad-mcc-38(执行日志记录单元),
    ECC-12(接收严重故障告警)

安全约束:
  S-01: 动力控制器离线超过200ms时，必须立即禁止自动驾驶，不得依赖最后有效值
  S-02: 动力系统过热（>180°C）必须立即降低功率至0，防止永久损坏
  S-03: 故障恢复后需持续监测 3 秒确认正常，方可解除降级，防止间歇性故障反复触发
  S-04: 本模块仅做故障诊断与降级触发，不直接干预动力执行器控制
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class FaultLevel(Enum):
    NORMAL_MONITOR = "normal_monitor"
    MINOR_FAULT = "minor_fault"
    SEVERE_FAULT = "severe_fault"
    CRITICAL_FAULT = "critical_fault"
    SYSTEM_PAUSED = "system_paused"


@dataclass
class TorqueResponse:
    target_torque_nm: float = 0.0
    actual_torque_nm: float = 0.0
    response_latency_ms: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class PowertrainTemperature:
    temperature_c: float = 25.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class PowertrainFaultCode:
    fault_code: int = 0
    level: str = ""
    description: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class ControllerHeartbeat:
    online: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class BusStatus:
    frame_error_rate_pct: float = 0.0
    node_heartbeat_ok: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class PowertrainFaultAlert:
    fault_type: str = ""
    severity: FaultLevel = FaultLevel.NORMAL_MONITOR
    current_value: float = 0.0
    threshold: float = 0.0
    suggested_action: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class DegradationRequest:
    target_level: int = 0
    reason: str = ""
    speed_limit_kmh: float = 120.0
    power_limit_ratio: float = 1.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class PowerLimitCommand:
    max_power_ratio: float = 1.0
    reason: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class PowertrainHealthReport:
    response_latency: float = 0.0
    temperature: float = 25.0
    controller_online: bool = True
    bus_status: str = "正常"
    fault_level: FaultLevel = FaultLevel.NORMAL_MONITOR
    timestamp: float = field(default_factory=time.time)


CONTROL_PERIOD_S = 0.01
REPORT_INTERVAL_S = 1.0

LATENCY_WARN_MS = 100.0
LATENCY_SEVERE_MS = 200.0
LATENCY_CRITICAL_MS = 500.0

TEMP_WARN_C = 140.0
TEMP_SEVERE_C = 160.0
TEMP_CRITICAL_C = 180.0

BUS_ERROR_RATE_WARN = 0.5
BUS_ERROR_RATE_SEVERE = 1.0

CONTROLLER_OFFLINE_CRITICAL_MS = 200.0

DURATION_CYCLES = 50  # 500ms at 10ms
RECOVERY_CYCLES = 300  # 3 seconds


class PowertrainFaultMonitor:
    def __init__(self):
        self.module_id = "ad-mcc-30"
        self.module_name = "动力系统异常监测单元"
        self.version = "V1.0"

        self.state = FaultLevel.NORMAL_MONITOR
        self._latency_fault_counter = 0
        self._temp_fault_counter = 0
        self._bus_fault_counter = 0
        self._offline_timer = 0.0
        self._recovery_counter = 0
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_torque_response = None
        self._query_temperature = None
        self._query_fault_code = None
        self._query_controller_heartbeat = None
        self._query_bus_status = None

        self._publish_alert = None
        self._publish_degradation = None
        self._publish_power_limit = None
        self._publish_health_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_torque_response_query(self, callback):
        self._query_torque_response = callback

    def set_temperature_query(self, callback):
        self._query_temperature = callback

    def set_fault_code_query(self, callback):
        self._query_fault_code = callback

    def set_controller_heartbeat_query(self, callback):
        self._query_controller_heartbeat = callback

    def set_bus_status_query(self, callback):
        self._query_bus_status = callback

    def set_alert_publisher(self, callback):
        self._publish_alert = callback

    def set_degradation_publisher(self, callback):
        self._publish_degradation = callback

    def set_power_limit_publisher(self, callback):
        self._publish_power_limit = callback

    def set_health_report_publisher(self, callback):
        self._publish_health_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_monitoring_cycle(self):
        now = time.time()
        if self.state == FaultLevel.SYSTEM_PAUSED:
            return

        torque = self._query_torque_response() if self._query_torque_response else TorqueResponse()
        temp = self._query_temperature() if self._query_temperature else PowertrainTemperature()
        fault = self._query_fault_code() if self._query_fault_code else PowertrainFaultCode()
        heartbeat = self._query_controller_heartbeat() if self._query_controller_heartbeat else ControllerHeartbeat()
        bus = self._query_bus_status() if self._query_bus_status else BusStatus()

        # 控制器离线计时
        if not heartbeat.online:
            self._offline_timer += CONTROL_PERIOD_S
        else:
            self._offline_timer = 0.0

        if self._offline_timer * 1000.0 >= CONTROLLER_OFFLINE_CRITICAL_MS:
            self.state = FaultLevel.CRITICAL_FAULT
            self._send_degradation(3, "动力控制器离线", 0.0, 0.0)
            self._send_alert("控制器离线", FaultLevel.CRITICAL_FAULT, 0.0, 0.0, "禁止自动驾驶")
            return

        latency = torque.response_latency_ms
        temperature = temp.temperature_c
        bus_error = bus.frame_error_rate_pct

        latency_severe = latency > LATENCY_SEVERE_MS
        latency_warn = latency > LATENCY_WARN_MS and not latency_severe

        temp_critical = temperature > TEMP_CRITICAL_C
        temp_severe = temperature > TEMP_SEVERE_C and not temp_critical
        temp_warn = temperature > TEMP_WARN_C and not temp_severe and not temp_critical

        bus_severe = bus_error > BUS_ERROR_RATE_SEVERE
        bus_warn = bus_error > BUS_ERROR_RATE_WARN and not bus_severe

        if latency_severe or latency_warn:
            self._latency_fault_counter += 1
        else:
            self._latency_fault_counter = 0

        if temp_severe or temp_warn:
            self._temp_fault_counter += 1
        else:
            self._temp_fault_counter = 0

        if bus_severe or bus_warn:
            self._bus_fault_counter += 1
        else:
            self._bus_fault_counter = 0

        critical_trigger = False
        severe_trigger = False
        minor_trigger = False
        reason_parts = []

        if temp_critical:
            critical_trigger = True
            reason_parts.append("动力系统过热(>180°C)")
        if fault.fault_code != 0 and fault.level == "致命":
            critical_trigger = True
            reason_parts.append(f"致命故障: {fault.description}")

        if critical_trigger:
            self.state = FaultLevel.CRITICAL_FAULT
            self._send_degradation(3, " + ".join(reason_parts), 0.0, 0.0)
            self._send_alert(" + ".join(reason_parts), FaultLevel.CRITICAL_FAULT, 0.0, 0.0, "立即停车")
            self._send_power_limit(0.0, "动力系统致命故障")
            return

        if latency_severe and self._latency_fault_counter >= DURATION_CYCLES:
            severe_trigger = True
            reason_parts.append("响应延迟严重")
        if temp_severe and self._temp_fault_counter >= DURATION_CYCLES:
            severe_trigger = True
            reason_parts.append("温度过高")
        if bus_severe and self._bus_fault_counter >= DURATION_CYCLES:
            severe_trigger = True
            reason_parts.append("通信总线异常")
        if fault.fault_code != 0 and fault.level == "严重":
            severe_trigger = True
            reason_parts.append(f"严重故障: {fault.description}")

        if severe_trigger:
            self.state = FaultLevel.SEVERE_FAULT
            reason = " + ".join(reason_parts)
            self._send_degradation(2, reason, 40.0, 0.5)
            self._send_power_limit(0.5, reason)
            self._send_alert(reason, FaultLevel.SEVERE_FAULT, 0.0, 0.0, "功率限制50%")
        else:
            if latency_warn and self._latency_fault_counter >= DURATION_CYCLES:
                minor_trigger = True
                reason_parts.append("响应延迟偏高")
            if temp_warn and self._temp_fault_counter >= DURATION_CYCLES:
                minor_trigger = True
                reason_parts.append("温度偏高")
            if bus_warn and self._bus_fault_counter >= DURATION_CYCLES:
                minor_trigger = True
                reason_parts.append("通信总线预警")

            if minor_trigger:
                self.state = FaultLevel.MINOR_FAULT
                reason = " + ".join(reason_parts)
                self._send_power_limit(0.8, reason)
                self._send_alert(reason, FaultLevel.MINOR_FAULT, 0.0, 0.0, "限制功率80%")
            else:
                if self.state not in (FaultLevel.NORMAL_MONITOR, FaultLevel.SYSTEM_PAUSED):
                    self._recovery_counter += 1
                    if self._recovery_counter >= RECOVERY_CYCLES:
                        self.state = FaultLevel.NORMAL_MONITOR
                        self._recovery_counter = 0
                        self._send_power_limit(1.0, "动力系统故障恢复")
                        self._send_alert("动力系统恢复正常", FaultLevel.NORMAL_MONITOR, 0.0, 0.0, "")
                else:
                    self._recovery_counter = 0

        if now - self._last_report_time >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_health_report:
                self._publish_health_report(PowertrainHealthReport(
                    response_latency=latency,
                    temperature=temperature,
                    controller_online=heartbeat.online,
                    bus_status="异常" if bus_severe or bus_warn else "正常",
                    fault_level=self.state
                ))

    def _send_alert(self, fault_type, severity, value, threshold, action):
        if self._publish_alert:
            self._publish_alert(PowertrainFaultAlert(
                fault_type=fault_type,
                severity=severity,
                current_value=value,
                threshold=threshold,
                suggested_action=action
            ))

    def _send_degradation(self, level, reason, speed_limit, power_limit):
        if self._publish_degradation:
            self._publish_degradation(DegradationRequest(
                target_level=level,
                reason=reason,
                speed_limit_kmh=speed_limit,
                power_limit_ratio=power_limit
            ))

    def _send_power_limit(self, ratio, reason):
        if self._publish_power_limit:
            self._publish_power_limit(PowerLimitCommand(
                max_power_ratio=ratio,
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

    def get_state(self) -> FaultLevel:
        return self.state

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def emergency_shutdown(self):
        self.state = FaultLevel.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 动力系统异常监测单元 (ad-mcc-30) 演示")
    print("=" * 70)

    monitor = PowertrainFaultMonitor()
    monitor.set_torque_response_query(lambda: TorqueResponse(response_latency_ms=30.0))
    monitor.set_temperature_query(lambda: PowertrainTemperature(temperature_c=100.0))
    monitor.set_fault_code_query(lambda: PowertrainFaultCode())
    monitor.set_controller_heartbeat_query(lambda: ControllerHeartbeat(online=True))
    monitor.set_bus_status_query(lambda: BusStatus())

    print_separator("STEP 1: 正常监测")
    monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 2: 响应延迟偏高预警")
    monitor.set_torque_response_query(lambda: TorqueResponse(response_latency_ms=120.0))
    for _ in range(60):
        monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 3: 温度严重过高")
    monitor.set_temperature_query(lambda: PowertrainTemperature(temperature_c=165.0))
    for _ in range(60):
        monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print("\n✅ 动力系统异常监测单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-30 动力系统异常监测单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_monitor(latency=30.0, temp=100.0, online=True, fault_code=0, error_rate=0.0):
            m = PowertrainFaultMonitor()
            m.set_torque_response_query(lambda: TorqueResponse(response_latency_ms=latency))
            m.set_temperature_query(lambda: PowertrainTemperature(temperature_c=temp))
            m.set_controller_heartbeat_query(lambda: ControllerHeartbeat(online=online))
            m.set_fault_code_query(lambda: PowertrainFaultCode(fault_code=fault_code))
            m.set_bus_status_query(lambda: BusStatus(frame_error_rate_pct=error_rate))
            return m

        print("\n[TC-M30-01] 正常状态")
        try:
            m = setup_monitor()
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M30-02] 响应延迟预警")
        try:
            m = setup_monitor(latency=120.0)
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m.state == FaultLevel.MINOR_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M30-03] 温度过高严重")
        try:
            m = setup_monitor(temp=165.0)
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m.state == FaultLevel.SEVERE_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M30-04] 控制器离线致命")
        try:
            m = setup_monitor(online=False)
            m._offline_timer = 0.25  # 250ms
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.CRITICAL_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M30-05] 故障恢复")
        try:
            m = setup_monitor()
            m.state = FaultLevel.SEVERE_FAULT
            m._recovery_counter = 300
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M30-06] 紧急熔断")
        try:
            m = setup_monitor()
            m.emergency_shutdown()
            assert m.state == FaultLevel.SYSTEM_PAUSED
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