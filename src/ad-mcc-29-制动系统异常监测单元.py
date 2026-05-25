#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-29
模块名称: 制动系统异常监测单元
所属分区: 八、硬件异常应急防护
核心职责: 实时监测制动主缸压力传感器双路信号一致性、制动液位、制动踏板开关状态及制动系统
          通信状态，判断制动系统是否存在传感器故障、液位过低、管路泄漏或通信失效。当检测到
          异常时，根据故障严重等级触发分级降级策略：轻微异常时发出预警并限制制动压力上限；
          严重异常时触发降级，限制车速至30km/h并优先使用再生制动；致命异常时触发三级降级并
          禁止自动驾驶。不参与制动控制决策，仅做故障诊断与降级触发。

依赖模块:
    制动主缸压力传感器(CAN总线),
    制动液位传感器(CAN总线),
    制动踏板开关(CAN总线),
    ad-mcc-01(小脑总控调度核心),
    ad-mcc-31(通信总线异常监测单元)
被依赖模块:
    ad-mcc-01(接收异常告警与降级触发),
    ad-mcc-13(制动压力解算单元),
    ad-mcc-17(再生制动优先协调单元),
    ad-mcc-38(执行日志记录单元),
    ECC-12(接收严重故障告警)

安全约束:
  S-01: 制动压力传感器双路失效时，必须立即禁止自动驾驶，不得依赖单路或历史数据
  S-02: 制动液位过低（<20%）时，视为制动系统严重故障，必须立即停车
  S-03: 制动踏板开关失效时，无法确认驾驶员制动意图，禁止自动驾驶
  S-04: 故障恢复后需持续监测 3 秒确认正常，方可解除降级，防止间歇性故障反复触发
  S-05: 本模块仅做故障诊断与降级触发，不直接干预制动执行器控制
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
class DualPressureSensor:
    primary_mpa: float = 0.0
    primary_timeout: bool = False
    secondary_mpa: float = 0.0
    secondary_timeout: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class BrakeFluidLevel:
    level_pct: float = 100.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class BrakePedalSwitch:
    pressed: bool = False
    malfunction: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class BrakeFaultCode:
    fault_code: int = 0
    level: str = ""
    description: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class BusStatus:
    frame_error_rate_pct: float = 0.0
    node_heartbeat_ok: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class BrakeFaultAlert:
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
    brake_pressure_limit: float = 10.0
    regen_priority: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class PressureLimitCommand:
    max_pressure_mpa: float = 10.0
    reason: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class BrakeHealthReport:
    primary_pressure: float = 0.0
    secondary_pressure: float = 0.0
    pressure_deviation: float = 0.0
    fluid_level: float = 100.0
    bus_status: str = "正常"
    fault_level: FaultLevel = FaultLevel.NORMAL_MONITOR
    timestamp: float = field(default_factory=time.time)


CONTROL_PERIOD_S = 0.005
REPORT_INTERVAL_S = 1.0

DEVIATION_WARN_MPA = 0.3
DEVIATION_SEVERE_MPA = 0.5

FLUID_WARN_PCT = 80.0
FLUID_SEVERE_PCT = 50.0
FLUID_CRITICAL_PCT = 20.0

BUS_ERROR_RATE_WARN = 0.5
BUS_ERROR_RATE_SEVERE = 1.0

DURATION_CYCLES = 100  # 500ms at 5ms
RECOVERY_CYCLES = 600  # 3 seconds


class BrakeFaultMonitor:
    def __init__(self):
        self.module_id = "ad-mcc-29"
        self.module_name = "制动系统异常监测单元"
        self.version = "V1.0"

        self.state = FaultLevel.NORMAL_MONITOR
        self._pressure_fault_counter = 0
        self._fluid_fault_counter = 0
        self._bus_fault_counter = 0
        self._recovery_counter = 0
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_pressure_sensor = None
        self._query_fluid_level = None
        self._query_pedal_switch = None
        self._query_brake_fault = None
        self._query_bus_status = None

        self._publish_alert = None
        self._publish_degradation = None
        self._publish_pressure_limit = None
        self._publish_health_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_pressure_sensor_query(self, callback):
        self._query_pressure_sensor = callback

    def set_fluid_level_query(self, callback):
        self._query_fluid_level = callback

    def set_pedal_switch_query(self, callback):
        self._query_pedal_switch = callback

    def set_brake_fault_query(self, callback):
        self._query_brake_fault = callback

    def set_bus_status_query(self, callback):
        self._query_bus_status = callback

    def set_alert_publisher(self, callback):
        self._publish_alert = callback

    def set_degradation_publisher(self, callback):
        self._publish_degradation = callback

    def set_pressure_limit_publisher(self, callback):
        self._publish_pressure_limit = callback

    def set_health_report_publisher(self, callback):
        self._publish_health_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_monitoring_cycle(self):
        now = time.time()
        if self.state == FaultLevel.SYSTEM_PAUSED:
            return

        pressure = self._query_pressure_sensor() if self._query_pressure_sensor else DualPressureSensor()
        fluid = self._query_fluid_level() if self._query_fluid_level else BrakeFluidLevel()
        pedal = self._query_pedal_switch() if self._query_pedal_switch else BrakePedalSwitch()
        fault = self._query_brake_fault() if self._query_brake_fault else BrakeFaultCode()
        bus = self._query_bus_status() if self._query_bus_status else BusStatus()

        primary_lost = pressure.primary_timeout
        secondary_lost = pressure.secondary_timeout
        dual_lost = primary_lost and secondary_lost

        if dual_lost:
            self.state = FaultLevel.CRITICAL_FAULT
            self._send_degradation(3, "制动压力传感器双路失效", 0.0, 0.0)
            self._send_alert("传感器双路失效", FaultLevel.CRITICAL_FAULT, 0.0, 0.0, "禁止自动驾驶")
            return

        if primary_lost or secondary_lost:
            deviation = 0.0
        else:
            deviation = abs(pressure.primary_mpa - pressure.secondary_mpa)

        fluid_level = fluid.level_pct
        bus_error = bus.frame_error_rate_pct
        pedal_failure = pedal.malfunction

        deviation_severe = deviation > DEVIATION_SEVERE_MPA
        deviation_warn = deviation > DEVIATION_WARN_MPA
        fluid_critical = fluid_level < FLUID_CRITICAL_PCT
        fluid_severe = fluid_level < FLUID_SEVERE_PCT
        fluid_warn = fluid_level < FLUID_WARN_PCT
        bus_severe = bus_error > BUS_ERROR_RATE_SEVERE
        bus_warn = bus_error > BUS_ERROR_RATE_WARN

        if deviation_severe or deviation_warn:
            self._pressure_fault_counter += 1
        else:
            self._pressure_fault_counter = 0

        if fluid_severe or fluid_warn:
            self._fluid_fault_counter += 1
        else:
            self._fluid_fault_counter = 0

        if bus_severe or bus_warn:
            self._bus_fault_counter += 1
        else:
            self._bus_fault_counter = 0

        severe_trigger = False
        minor_trigger = False
        critical_trigger = False
        reason_parts = []

        if fluid_critical:
            critical_trigger = True
            reason_parts.append("制动液位致命(<20%)")
        if pedal_failure:
            critical_trigger = True
            reason_parts.append("制动踏板开关失效")
        if deviation_severe and self._pressure_fault_counter >= DURATION_CYCLES:
            severe_trigger = True
            reason_parts.append("压力偏差严重")
        if fluid_severe and self._fluid_fault_counter >= DURATION_CYCLES:
            severe_trigger = True
            reason_parts.append("制动液位过低")
        if bus_severe and self._bus_fault_counter >= DURATION_CYCLES:
            severe_trigger = True
            reason_parts.append("通信总线异常")
        if fault.fault_code != 0 and fault.level in ("严重", "致命"):
            severe_trigger = True
            reason_parts.append(f"ESP故障: {fault.description}")

        if deviation_warn and self._pressure_fault_counter >= DURATION_CYCLES:
            minor_trigger = True
            reason_parts.append("压力偏差预警")
        if fluid_warn and self._fluid_fault_counter >= DURATION_CYCLES:
            minor_trigger = True
            reason_parts.append("制动液位偏低")
        if bus_warn and self._bus_fault_counter >= DURATION_CYCLES:
            minor_trigger = True
            reason_parts.append("通信总线预警")

        reason = " + ".join(reason_parts) if reason_parts else ""

        if critical_trigger:
            self.state = FaultLevel.CRITICAL_FAULT
            self._send_degradation(3, reason, 0.0, 0.0)
            self._send_alert(reason, FaultLevel.CRITICAL_FAULT, 0.0, 0.0, "禁止自动驾驶")
        elif severe_trigger:
            self.state = FaultLevel.SEVERE_FAULT
            self._send_degradation(2, reason, 30.0, 5.0, regen_priority=True)
            self._send_pressure_limit(5.0, reason)
            self._send_alert(reason, FaultLevel.SEVERE_FAULT, 0.0, 0.0, "二级降级，再生优先")
        elif minor_trigger:
            self.state = FaultLevel.MINOR_FAULT
            self._send_pressure_limit(8.0, reason)
            self._send_alert(reason, FaultLevel.MINOR_FAULT, 0.0, 0.0, "限制制动压力")
        else:
            if self.state not in (FaultLevel.NORMAL_MONITOR, FaultLevel.SYSTEM_PAUSED):
                self._recovery_counter += 1
                if self._recovery_counter >= RECOVERY_CYCLES:
                    self.state = FaultLevel.NORMAL_MONITOR
                    self._recovery_counter = 0
                    self._send_pressure_limit(10.0, "故障恢复")
                    self._send_alert("制动系统恢复正常", FaultLevel.NORMAL_MONITOR, 0.0, 0.0, "")
            else:
                self._recovery_counter = 0

        if now - self._last_report_time >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_health_report:
                self._publish_health_report(BrakeHealthReport(
                    primary_pressure=pressure.primary_mpa,
                    secondary_pressure=pressure.secondary_mpa,
                    pressure_deviation=deviation,
                    fluid_level=fluid_level,
                    bus_status="异常" if bus_severe or bus_warn else "正常",
                    fault_level=self.state
                ))

    def _send_alert(self, fault_type, severity, value, threshold, action):
        if self._publish_alert:
            self._publish_alert(BrakeFaultAlert(
                fault_type=fault_type,
                severity=severity,
                current_value=value,
                threshold=threshold,
                suggested_action=action
            ))

    def _send_degradation(self, level, reason, speed_limit, pressure_limit, regen_priority=False):
        if self._publish_degradation:
            self._publish_degradation(DegradationRequest(
                target_level=level,
                reason=reason,
                speed_limit_kmh=speed_limit,
                brake_pressure_limit=pressure_limit,
                regen_priority=regen_priority
            ))

    def _send_pressure_limit(self, max_pressure, reason):
        if self._publish_pressure_limit:
            self._publish_pressure_limit(PressureLimitCommand(
                max_pressure_mpa=max_pressure,
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
    print("  AD-mcc-cerebellum 制动系统异常监测单元 (ad-mcc-29) 演示")
    print("=" * 70)

    monitor = BrakeFaultMonitor()
    monitor.set_pressure_sensor_query(lambda: DualPressureSensor(primary_mpa=2.0, secondary_mpa=1.9))
    monitor.set_fluid_level_query(lambda: BrakeFluidLevel(level_pct=95.0))
    monitor.set_pedal_switch_query(lambda: BrakePedalSwitch())
    monitor.set_brake_fault_query(lambda: BrakeFaultCode())
    monitor.set_bus_status_query(lambda: BusStatus())

    print_separator("STEP 1: 正常监测")
    monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 2: 液位偏低预警")
    monitor.set_fluid_level_query(lambda: BrakeFluidLevel(level_pct=70.0))
    for _ in range(120):
        monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 3: 压力偏差严重")
    monitor.set_pressure_sensor_query(lambda: DualPressureSensor(primary_mpa=3.0, secondary_mpa=2.3))
    for _ in range(120):
        monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print("\n✅ 制动系统异常监测单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-29 制动系统异常监测单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_monitor(pri_mpa=2.0, sec_mpa=1.9, pri_timeout=False, sec_timeout=False,
                          fluid=95.0, pedal_ok=True, fault_code=0, error_rate=0.0):
            m = BrakeFaultMonitor()
            m.set_pressure_sensor_query(lambda: DualPressureSensor(
                primary_mpa=pri_mpa, secondary_mpa=sec_mpa,
                primary_timeout=pri_timeout, secondary_timeout=sec_timeout
            ))
            m.set_fluid_level_query(lambda: BrakeFluidLevel(level_pct=fluid))
            m.set_pedal_switch_query(lambda: BrakePedalSwitch(malfunction=not pedal_ok))
            m.set_brake_fault_query(lambda: BrakeFaultCode(fault_code=fault_code))
            m.set_bus_status_query(lambda: BusStatus(frame_error_rate_pct=error_rate))
            return m

        print("\n[TC-M29-01] 正常状态")
        try:
            m = setup_monitor()
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M29-02] 液位偏低预警")
        try:
            m = setup_monitor(fluid=70.0)
            for _ in range(120):
                m.run_monitoring_cycle()
            assert m.state == FaultLevel.MINOR_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M29-03] 压力偏差严重")
        try:
            m = setup_monitor(pri_mpa=3.0, sec_mpa=2.3)
            for _ in range(120):
                m.run_monitoring_cycle()
            assert m.state == FaultLevel.SEVERE_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M29-04] 液位致命")
        try:
            m = setup_monitor(fluid=15.0)
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.CRITICAL_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M29-05] 双路传感器失效")
        try:
            m = setup_monitor(pri_timeout=True, sec_timeout=True)
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.CRITICAL_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M29-06] 故障恢复")
        try:
            m = setup_monitor()
            m.state = FaultLevel.SEVERE_FAULT
            m._recovery_counter = 600
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.NORMAL_MONITOR
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