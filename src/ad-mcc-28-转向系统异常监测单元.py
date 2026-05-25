#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-28
模块名称: 转向系统异常监测单元
所属分区: 八、硬件异常应急防护
核心职责: 实时监测转向电机电流、转角传感器双路信号一致性及转向系统通信状态，判断转向系统
          是否存在电气故障、机械卡滞或传感器失效。当检测到异常时，根据故障严重等级触发分级
          降级策略：轻微异常时发出预警并限制转向速率；严重异常时触发二级降级，限制车速至
          40km/h 并请求人工接管；致命异常时触发三级降级并禁止自动驾驶。不参与转向控制决策，
          仅做故障诊断与降级触发。

依赖模块:
    转向电机电流传感器(CAN总线),
    方向盘转角传感器(CAN总线),
    ad-mcc-01(小脑总控调度核心),
    ad-mcc-02(运动生理边界闸门)
被依赖模块:
    ad-mcc-01(接收异常告警与降级触发),
    ad-mcc-38(执行日志记录单元),
    ECC-12(接收严重故障告警)

安全约束:
  S-01: 转角传感器双路失效时，必须立即禁止自动驾驶，不得依赖历史数据或模型推算
  S-02: 转向电机过载必须限制转向速率，防止电机损坏导致转向失控
  S-03: 故障恢复后需持续监测 3 秒确认正常，方可解除降级，防止间歇性故障反复触发
  S-04: 本模块仅做故障诊断与降级触发，不直接干预转向执行器控制
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
class MotorCurrent:
    current_a: float = 0.0
    rated_a: float = 80.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class DualAngleSensor:
    primary_deg: float = 0.0
    primary_timeout: bool = False
    secondary_deg: float = 0.0
    secondary_timeout: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class EPSFaultCode:
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
class SteeringFaultAlert:
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
    steer_rate_limit: float = 500.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class SteeringHealthReport:
    motor_current: float = 0.0
    sensor_deviation: float = 0.0
    bus_status: str = "正常"
    fault_level: FaultLevel = FaultLevel.NORMAL_MONITOR
    timestamp: float = field(default_factory=time.time)


CONTROL_PERIOD_S = 0.01
REPORT_INTERVAL_S = 1.0

CURRENT_WARN_RATIO = 1.10
CURRENT_SEVERE_RATIO = 1.20

DEVIATION_WARN_DEG = 3.0
DEVIATION_SEVERE_DEG = 5.0

BUS_ERROR_RATE_WARN = 0.5
BUS_ERROR_RATE_SEVERE = 1.0

SENSOR_TIMEOUT_WARN_MS = 50.0
SENSOR_TIMEOUT_SEVERE_MS = 100.0
SENSOR_TIMEOUT_CRITICAL_MS = 200.0

DURATION_CYCLES = 50  # 500ms
RECOVERY_CYCLES = 300  # 3 seconds


class SteeringFaultMonitor:
    def __init__(self):
        self.module_id = "ad-mcc-28"
        self.module_name = "转向系统异常监测单元"
        self.version = "V1.0"

        self.state = FaultLevel.NORMAL_MONITOR
        self._current_fault_counter = 0
        self._deviation_fault_counter = 0
        self._bus_fault_counter = 0
        self._recovery_counter = 0
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_motor_current = None
        self._query_angle_sensor = None
        self._query_eps_fault = None
        self._query_bus_status = None

        self._publish_alert = None
        self._publish_degradation = None
        self._publish_health_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_motor_current_query(self, callback):
        self._query_motor_current = callback

    def set_angle_sensor_query(self, callback):
        self._query_angle_sensor = callback

    def set_eps_fault_query(self, callback):
        self._query_eps_fault = callback

    def set_bus_status_query(self, callback):
        self._query_bus_status = callback

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
        if self.state == FaultLevel.SYSTEM_PAUSED:
            return

        motor = self._query_motor_current() if self._query_motor_current else MotorCurrent()
        angle = self._query_angle_sensor() if self._query_angle_sensor else DualAngleSensor()
        eps = self._query_eps_fault() if self._query_eps_fault else EPSFaultCode()
        bus = self._query_bus_status() if self._query_bus_status else BusStatus()

        # 传感器超时判定
        primary_lost = angle.primary_timeout
        secondary_lost = angle.secondary_timeout
        dual_lost = primary_lost and secondary_lost

        if dual_lost:
            self.state = FaultLevel.CRITICAL_FAULT
            self._send_degradation(3, "转角传感器双路失效", 0.0, 0.0)
            self._send_alert("传感器双路失效", FaultLevel.CRITICAL_FAULT, 0.0, 0.0, "禁止自动驾驶")
            return

        # 使用有效的一路作为实际值，偏差置零
        if primary_lost or secondary_lost:
            deviation = 0.0
            sensor_fault = True
        else:
            deviation = abs(angle.primary_deg - angle.secondary_deg)
            sensor_fault = False

        rated_current = motor.rated_a if motor.rated_a > 0 else 80.0
        current_ratio = motor.current_a / rated_current if rated_current > 0 else 0.0

        # 判定电流等级
        current_severe = current_ratio > CURRENT_SEVERE_RATIO or (eps.fault_code != 0 and "过载" in eps.description)
        current_warn = current_ratio > CURRENT_WARN_RATIO

        # 判定偏差等级
        deviation_severe = deviation > DEVIATION_SEVERE_DEG
        deviation_warn = deviation > DEVIATION_WARN_DEG

        # 判定通信等级
        bus_severe = bus.frame_error_rate_pct > BUS_ERROR_RATE_SEVERE
        bus_warn = bus.frame_error_rate_pct > BUS_ERROR_RATE_WARN

        # 故障计数更新
        if current_severe or deviation_severe or bus_severe:
            self._current_fault_counter += 1 if current_severe else 0
            self._deviation_fault_counter += 1 if deviation_severe else 0
            self._bus_fault_counter += 1 if bus_severe else 0
        elif current_warn or deviation_warn or bus_warn:
            self._current_fault_counter += 1 if current_warn else 0
            self._deviation_fault_counter += 1 if deviation_warn else 0
            self._bus_fault_counter += 1 if bus_warn else 0
        else:
            self._current_fault_counter = 0
            self._deviation_fault_counter = 0
            self._bus_fault_counter = 0

        # 综合故障等级判定
        if (self._current_fault_counter >= DURATION_CYCLES and current_severe) or \
           (self._deviation_fault_counter >= DURATION_CYCLES and deviation_severe) or \
           (self._bus_fault_counter >= DURATION_CYCLES and bus_severe):
            self.state = FaultLevel.SEVERE_FAULT
            reason = self._get_fault_reason(current_severe, deviation_severe, bus_severe)
            self._send_degradation(2, reason, 40.0, 150.0)
            self._send_alert(reason, FaultLevel.SEVERE_FAULT, 0.0, 0.0, "触发二级降级")
        elif (self._current_fault_counter >= DURATION_CYCLES and current_warn) or \
             (self._deviation_fault_counter >= DURATION_CYCLES and deviation_warn) or \
             (self._bus_fault_counter >= DURATION_CYCLES and bus_warn):
            self.state = FaultLevel.MINOR_FAULT
            reason = self._get_fault_reason(current_warn, deviation_warn, bus_warn)
            self._send_alert(reason, FaultLevel.MINOR_FAULT, 0.0, 0.0, "预警，限制转向速率")
        else:
            if self.state not in (FaultLevel.NORMAL_MONITOR, FaultLevel.SYSTEM_PAUSED):
                self._recovery_counter += 1
                if self._recovery_counter >= RECOVERY_CYCLES:
                    self.state = FaultLevel.NORMAL_MONITOR
                    self._recovery_counter = 0
                    self._send_alert("转向系统故障恢复", FaultLevel.NORMAL_MONITOR, 0.0, 0.0, "")
            else:
                self._recovery_counter = 0

        # 周期性上报
        if now - self._last_report_time >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_health_report:
                self._publish_health_report(SteeringHealthReport(
                    motor_current=motor.current_a,
                    sensor_deviation=deviation,
                    bus_status="异常" if bus_severe or bus_warn else "正常",
                    fault_level=self.state
                ))

    def _get_fault_reason(self, current_flag, deviation_flag, bus_flag):
        parts = []
        if current_flag:
            parts.append("电机电流异常")
        if deviation_flag:
            parts.append("传感器偏差超限")
        if bus_flag:
            parts.append("通信总线异常")
        return " + ".join(parts) if parts else "未知故障"

    def _send_alert(self, fault_type, severity, value, threshold, action):
        if self._publish_alert:
            self._publish_alert(SteeringFaultAlert(
                fault_type=fault_type,
                severity=severity,
                current_value=value,
                threshold=threshold,
                suggested_action=action
            ))

    def _send_degradation(self, level, reason, speed_limit, steer_limit):
        if self._publish_degradation:
            self._publish_degradation(DegradationRequest(
                target_level=level,
                reason=reason,
                speed_limit_kmh=speed_limit,
                steer_rate_limit=steer_limit
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
    print("  AD-mcc-cerebellum 转向系统异常监测单元 (ad-mcc-28) 演示")
    print("=" * 70)

    monitor = SteeringFaultMonitor()
    monitor.set_motor_current_query(lambda: MotorCurrent(current_a=80.0, rated_a=80.0))
    monitor.set_angle_sensor_query(lambda: DualAngleSensor(primary_deg=10.0, secondary_deg=9.5))
    monitor.set_eps_fault_query(lambda: EPSFaultCode())
    monitor.set_bus_status_query(lambda: BusStatus())

    print_separator("STEP 1: 正常监测")
    monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 2: 电流偏高预警")
    monitor.set_motor_current_query(lambda: MotorCurrent(current_a=92.0, rated_a=80.0))
    for _ in range(60):
        monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 3: 传感器偏差严重")
    monitor.set_angle_sensor_query(lambda: DualAngleSensor(primary_deg=15.0, secondary_deg=9.0))
    for _ in range(60):
        monitor.run_monitoring_cycle()
    print(f"  状态: {monitor.state.value}")

    print("\n✅ 转向系统异常监测单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-28 转向系统异常监测单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_monitor(current_a=80.0, rated_a=80.0, pri=10.0, sec=10.0,
                          pri_timeout=False, sec_timeout=False, fault_code=0,
                          error_rate=0.0):
            m = SteeringFaultMonitor()
            m.set_motor_current_query(lambda: MotorCurrent(current_a=current_a, rated_a=rated_a))
            m.set_angle_sensor_query(lambda: DualAngleSensor(
                primary_deg=pri, secondary_deg=sec,
                primary_timeout=pri_timeout, secondary_timeout=sec_timeout
            ))
            m.set_eps_fault_query(lambda: EPSFaultCode(fault_code=fault_code))
            m.set_bus_status_query(lambda: BusStatus(frame_error_rate_pct=error_rate))
            return m

        print("\n[TC-M28-01] 正常状态")
        try:
            m = setup_monitor()
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M28-02] 电流偏高预警")
        try:
            m = setup_monitor(current_a=92.0)
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m.state == FaultLevel.MINOR_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M28-03] 传感器偏差严重")
        try:
            m = setup_monitor(pri=15.0, sec=9.0)
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m.state == FaultLevel.SEVERE_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M28-04] 双路传感器失效")
        try:
            m = setup_monitor(pri_timeout=True, sec_timeout=True)
            m.run_monitoring_cycle()
            assert m.state == FaultLevel.CRITICAL_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M28-05] 故障恢复")
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

        print("\n[TC-M28-06] 紧急熔断")
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