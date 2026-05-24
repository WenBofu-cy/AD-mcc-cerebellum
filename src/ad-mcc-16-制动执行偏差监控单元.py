#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-16
模块名称: 制动执行偏差监控单元
所属分区: 四、制动控制集群
核心职责: 实时对比 ad-mcc-15 输出的最终制动压力目标指令与车辆实际制动主缸压力
          （来自 CAN 总线压力传感器），计算压力偏差与制动响应延迟。当偏差超过预设阈值时，
          触发制动异常告警并上报 ECC，同时根据偏差量级建议降级或紧急检查。同时将偏差数据
          周期性推送至 ad-mcc-36 执行闭环反馈单元，供运动闭环回执与运动质量评估使用。
          不参与任何操控指令的修改，仅做监控与告警。

依赖模块:
    ad-mcc-15(制动平顺防点头单元，提供最终平顺化制动压力指令),
    制动压力传感器(CAN总线),
    ad-mcc-36(执行闭环反馈单元，消费偏差数据),
    ad-mcc-38(执行日志记录单元，记录偏差事件)
被依赖模块:
    ad-mcc-01(小脑总控调度核心，接收制动偏差告警),
    ad-mcc-03(全身运动状态归集中心，接收制动执行状态),
    ECC-12(通过 CerebellumBus 接收严重偏差告警)

安全约束:
  S-01: 本模块仅做监控与告警，不参与任何操控指令的修改与生成
  S-02: 制动压力传感器离线时必须明确标记并使用最后有效值填充，不可伪造在线状态
  S-03: 严重制动压力偏差告警（≥1.0 MPa）必须立即上报 ECC-12，触发安全降级评估，不得等待判定延迟
  S-04: 偏差数据须与原始目标指令和传感器数据保持一致，不得篡改
  S-05: 告警抑制机制不得遗漏新的告警类型。同一类型告警可抑制，不同类型告警须分别上报
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


class MonitorState(Enum):
    NORMAL_MONITOR = "normal_monitor"
    DEVIATION_WARNING = "deviation_warning"
    DEVIATION_CRITICAL = "deviation_critical"
    SENSOR_OFFLINE = "sensor_offline"
    SYSTEM_PAUSED = "system_paused"


class AlertSeverity(Enum):
    NORMAL = "正常"
    WARNING = "预警"
    CRITICAL = "严重"


@dataclass
class SmoothBrakeCommand:
    timestamp: float = field(default_factory=time.time)
    corrected_pressure_mpa: float = 0.0
    original_pressure_mpa: float = 0.0
    anti_dive_state: str = ""
    correction_amount: float = 0.0


@dataclass
class SensorHealth:
    sensor_id: str = "brake_pressure_sensor"
    online: bool = True
    data_valid: bool = True
    signal_quality: str = "良好"


@dataclass
class BrakeStatus:
    target_pressure_mpa: float = 0.0
    actual_pressure_mpa: float = 0.0
    pressure_deviation_mpa: float = 0.0
    response_latency_ms: float = 0.0
    online_status: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class DeviationAlert:
    alert_type: str = ""
    target_value: float = 0.0
    actual_value: float = 0.0
    deviation_amount: float = 0.0
    duration_ms: float = 0.0
    suggested_action: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class DeviationData:
    command_id: str = ""
    pressure_deviation_mpa: float = 0.0
    rate_deviation_pct: float = 0.0
    response_latency_ms: float = 0.0
    online_status: str = "正常"
    timestamp: float = field(default_factory=time.time)


@dataclass
class SensorFaultAlert:
    sensor_id: str = ""
    fault_type: str = ""
    last_valid_value: float = 0.0
    impact_assessment: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class MonitorStatusReport:
    current_state: MonitorState = MonitorState.NORMAL_MONITOR
    recent_deviation_mpa: float = 0.0
    alert_count: int = 0
    sensor_health: str = "良好"
    timestamp: float = field(default_factory=time.time)


# 偏差阈值
PRESSURE_DEV_NORMAL_MAX_MPA = 0.3
PRESSURE_DEV_WARN_THRESHOLD_MPA = 0.5
PRESSURE_DEV_CRITICAL_THRESHOLD_MPA = 1.0

LATENCY_NORMAL_MAX_MS = 100.0
LATENCY_WARN_THRESHOLD_MS = 150.0
LATENCY_CRITICAL_THRESHOLD_MS = 200.0

WARN_DELAY_CYCLES = 100  # 500ms / 5ms
CONTROL_PERIOD_S = 0.005
REPORT_INTERVAL_S = 1.0
SENSOR_TIMEOUT_MS = 100.0
SPIKE_DEVIATION_MPA = 2.0
SPIKE_CONFIRM_FRAMES = 3


class BrakeDeviationMonitor:
    def __init__(self):
        self.module_id = "ad-mcc-16"
        self.module_name = "制动执行偏差监控单元"
        self.version = "V1.0"

        self.state = MonitorState.NORMAL_MONITOR

        self._warn_counter = 0
        self._spike_counter = 0
        self._last_pressure_deviation = 0.0
        self._sensor_timeout_counter = 0
        self._last_valid_pressure = 0.0
        self._alert_suppressed = {"pressure": False, "latency": False}
        self._total_alerts = 0
        self._total_fault_events = 0
        self._last_report_time = 0.0
        self._recent_deviation = 0.0
        self._pending_logs = []

        self._query_brake_command = None
        self._query_actual_pressure = None
        self._query_sensor_health = None

        self._publish_brake_status = None
        self._publish_deviation_alert = None
        self._publish_deviation_data = None
        self._publish_sensor_fault = None
        self._publish_status_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_brake_command_query(self, callback):
        self._query_brake_command = callback

    def set_actual_pressure_query(self, callback):
        self._query_actual_pressure = callback

    def set_sensor_health_query(self, callback):
        self._query_sensor_health = callback

    def set_brake_status_publisher(self, callback):
        self._publish_brake_status = callback

    def set_deviation_alert_publisher(self, callback):
        self._publish_deviation_alert = callback

    def set_deviation_data_publisher(self, callback):
        self._publish_deviation_data = callback

    def set_sensor_fault_publisher(self, callback):
        self._publish_sensor_fault = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_monitoring_cycle(self):
        if self.state == MonitorState.SYSTEM_PAUSED:
            return None

        now = time.time()

        sensor_health = self._query_sensor_health() if self._query_sensor_health else SensorHealth()
        if not sensor_health.online or not sensor_health.data_valid:
            if self.state != MonitorState.SENSOR_OFFLINE:
                self.state = MonitorState.SENSOR_OFFLINE
                self._total_fault_events += 1
                self._publish_sensor_fault_alert(sensor_health)
            actual_pressure = self._last_valid_pressure
        else:
            if self.state == MonitorState.SENSOR_OFFLINE:
                self.state = MonitorState.NORMAL_MONITOR
            actual_pressure = self._query_actual_pressure() if self._query_actual_pressure else 0.0
            self._last_valid_pressure = actual_pressure

        brake_cmd = self._query_brake_command() if self._query_brake_command else None
        if brake_cmd is None:
            return None

        target_pressure = brake_cmd.corrected_pressure_mpa
        response_latency = (now - brake_cmd.timestamp) * 1000.0
        pressure_dev = actual_pressure - target_pressure

        # 噪声过滤
        if abs(pressure_dev - self._last_pressure_deviation) > SPIKE_DEVIATION_MPA:
            self._spike_counter += 1
            if self._spike_counter < SPIKE_CONFIRM_FRAMES:
                pressure_dev = self._last_pressure_deviation
            else:
                self._spike_counter = 0
        else:
            self._spike_counter = 0
        self._last_pressure_deviation = pressure_dev

        alert_triggered = False
        alert_type_parts = []
        alert_severity = AlertSeverity.NORMAL
        alert_action = ""

        dev_abs = abs(pressure_dev)

        if dev_abs >= PRESSURE_DEV_CRITICAL_THRESHOLD_MPA or response_latency > LATENCY_CRITICAL_THRESHOLD_MS:
            self.state = MonitorState.DEVIATION_CRITICAL
            alert_triggered = True
            alert_severity = AlertSeverity.CRITICAL
            if dev_abs >= PRESSURE_DEV_CRITICAL_THRESHOLD_MPA:
                alert_type_parts.append("严重压力偏差")
            if response_latency > LATENCY_CRITICAL_THRESHOLD_MS:
                alert_type_parts.append("严重响应延迟")
            alert_action = "触发降级，检查制动系统"
            self._warn_counter = 0
        elif dev_abs >= PRESSURE_DEV_WARN_THRESHOLD_MPA or response_latency > LATENCY_WARN_THRESHOLD_MS:
            self._warn_counter += 1
            if self._warn_counter >= WARN_DELAY_CYCLES:
                self.state = MonitorState.DEVIATION_WARNING
                alert_triggered = True
                alert_severity = AlertSeverity.WARNING
                if dev_abs >= PRESSURE_DEV_WARN_THRESHOLD_MPA:
                    alert_type_parts.append("压力偏差超限")
                if response_latency > LATENCY_WARN_THRESHOLD_MS:
                    alert_type_parts.append("响应延迟超限")
                alert_action = "降低制动需求，检查制动管路"
        else:
            self._warn_counter = 0
            if self.state not in (MonitorState.SENSOR_OFFLINE, MonitorState.SYSTEM_PAUSED):
                if self.state != MonitorState.NORMAL_MONITOR:
                    self._log_event("DEVIATION_RECOVERED", {"deviation": pressure_dev})
                self.state = MonitorState.NORMAL_MONITOR
                self._reset_alert_suppression()

        # 状态输出
        status = BrakeStatus(
            target_pressure_mpa=round(target_pressure, 4),
            actual_pressure_mpa=round(actual_pressure, 4),
            pressure_deviation_mpa=round(pressure_dev, 4),
            response_latency_ms=round(response_latency, 2),
            online_status=sensor_health.online
        )
        if self._publish_brake_status:
            self._publish_brake_status(status)

        # 告警输出
        if alert_triggered:
            alert_type_str = " + ".join(alert_type_parts)
            if not self._is_alert_suppressed(alert_type_parts):
                self._total_alerts += 1
                if self._publish_deviation_alert:
                    self._publish_deviation_alert(DeviationAlert(
                        alert_type=alert_type_str,
                        target_value=target_pressure,
                        actual_value=actual_pressure,
                        deviation_amount=round(dev_abs, 4),
                        duration_ms=self._warn_counter * CONTROL_PERIOD_S * 1000.0,
                        suggested_action=alert_action
                    ))
                self._suppress_alert(alert_type_parts)

        # 偏差数据推送
        deviation_data = DeviationData(
            command_id=f"brake-{brake_cmd.timestamp}",
            pressure_deviation_mpa=round(pressure_dev, 4),
            response_latency_ms=round(response_latency, 2),
            online_status="降级" if self.state == MonitorState.SENSOR_OFFLINE else "正常"
        )
        if self._publish_deviation_data:
            self._publish_deviation_data(deviation_data)

        if dev_abs > PRESSURE_DEV_NORMAL_MAX_MPA or response_latency > LATENCY_NORMAL_MAX_MS:
            if self._publish_event_log:
                self._publish_event_log({
                    "event_type": "brake_deviation",
                    "target": target_pressure,
                    "actual": actual_pressure,
                    "deviation": pressure_dev,
                    "latency_ms": response_latency,
                    "timestamp": now
                })

        self._recent_deviation = dev_abs
        if now - self._last_report_time >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_status_report:
                self._publish_status_report(MonitorStatusReport(
                    current_state=self.state,
                    recent_deviation_mpa=round(self._recent_deviation, 4),
                    alert_count=self._total_alerts,
                    sensor_health="良好" if sensor_health.online else "异常"
                ))

        return status

    def _is_alert_suppressed(self, alert_types: List[str]) -> bool:
        for alert_type in alert_types:
            if ("压力" in alert_type or "pressure" in alert_type) and not self._alert_suppressed.get("pressure", False):
                return False
            if ("延迟" in alert_type or "latency" in alert_type) and not self._alert_suppressed.get("latency", False):
                return False
        return True

    def _suppress_alert(self, alert_types: List[str]):
        for alert_type in alert_types:
            if "压力" in alert_type:
                self._alert_suppressed["pressure"] = True
            if "延迟" in alert_type:
                self._alert_suppressed["latency"] = True

    def _reset_alert_suppression(self):
        self._alert_suppressed = {"pressure": False, "latency": False}

    def _publish_sensor_fault_alert(self, health: SensorHealth):
        if self._publish_sensor_fault:
            self._publish_sensor_fault(SensorFaultAlert(
                sensor_id=health.sensor_id,
                fault_type="离线" if not health.online else "数据校验失败",
                last_valid_value=self._last_valid_pressure,
                impact_assessment="制动偏差监控降级，使用最后有效值"
            ))

    def _log_event(self, event_type: str, details: Dict[str, Any]) -> None:
        self._pending_logs.append({
            "log_id": f"log-{uuid.uuid4().hex[:8]}",
            "event_type": event_type,
            "source_module": self.module_id,
            "details": details,
            "timestamp": time.time()
        })

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def get_state(self) -> MonitorState:
        return self.state

    def get_statistics(self) -> Dict[str, Any]:
        return {
            "state": self.state.value,
            "total_alerts": self._total_alerts,
            "total_fault_events": self._total_fault_events,
            "recent_deviation_mpa": self._recent_deviation,
        }

    def emergency_shutdown(self):
        self.state = MonitorState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，保留最后有效状态")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 制动执行偏差监控单元 (ad-mcc-16) 演示")
    print("=" * 70)

    monitor = BrakeDeviationMonitor()
    monitor.set_actual_pressure_query(lambda: 3.2)
    monitor.set_sensor_health_query(lambda: SensorHealth(online=True, data_valid=True))

    print_separator("STEP 1: 正常偏差 (0.2 MPa)")
    monitor.set_brake_command_query(lambda: SmoothBrakeCommand(corrected_pressure_mpa=3.0))
    status = monitor.run_monitoring_cycle()
    if status:
        print(f"  目标压力: {status.target_pressure_mpa} MPa")
        print(f"  实际压力: {status.actual_pressure_mpa} MPa")
        print(f"  偏差: {status.pressure_deviation_mpa} MPa")
        print(f"  状态: {monitor.state.value}")

    print_separator("STEP 2: 预警偏差 (0.7 MPa) 持续")
    monitor.set_actual_pressure_query(lambda: 3.7)
    monitor.set_brake_command_query(lambda: SmoothBrakeCommand(corrected_pressure_mpa=3.0))
    for _ in range(120):
        status = monitor.run_monitoring_cycle()
    print(f"  偏差: {status.pressure_deviation_mpa} MPa")
    print(f"  状态: {monitor.state.value}")

    print_separator("STEP 3: 严重偏差 (1.2 MPa)")
    monitor.set_actual_pressure_query(lambda: 4.2)
    status = monitor.run_monitoring_cycle()
    print(f"  偏差: {status.pressure_deviation_mpa} MPa")
    print(f"  状态: {monitor.state.value}")

    print("\n✅ 制动执行偏差监控单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-16 制动执行偏差监控单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_monitor(actual_pressure=3.2, sensor_online=True):
            m = BrakeDeviationMonitor()
            m.set_actual_pressure_query(lambda: actual_pressure)
            m.set_sensor_health_query(lambda: SensorHealth(online=sensor_online, data_valid=sensor_online))
            return m

        # TC-M16-01: 正常偏差
        print("\n[TC-M16-01] 正常偏差 (0.2 MPa)")
        try:
            m = setup_monitor()
            m.set_brake_command_query(lambda: SmoothBrakeCommand(corrected_pressure_mpa=3.0))
            status = m.run_monitoring_cycle()
            assert status is not None
            assert abs(status.pressure_deviation_mpa) <= 0.3
            assert m.state == MonitorState.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M16-02: 预警偏差持续
        print("\n[TC-M16-02] 预警偏差 (0.7 MPa) 持续")
        try:
            m = setup_monitor(actual_pressure=3.7)
            m.set_brake_command_query(lambda: SmoothBrakeCommand(corrected_pressure_mpa=3.0))
            for _ in range(120):
                m.run_monitoring_cycle()
            assert m.state == MonitorState.DEVIATION_WARNING
            assert m.get_statistics()['total_alerts'] >= 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M16-03: 严重偏差
        print("\n[TC-M16-03] 严重偏差 (1.2 MPa)")
        try:
            m = setup_monitor(actual_pressure=4.2)
            m.set_brake_command_query(lambda: SmoothBrakeCommand(corrected_pressure_mpa=3.0))
            m.run_monitoring_cycle()
            assert m.state == MonitorState.DEVIATION_CRITICAL
            assert m.get_statistics()['total_alerts'] >= 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M16-04: 传感器离线
        print("\n[TC-M16-04] 传感器离线")
        try:
            m = setup_monitor(sensor_online=False)
            m.set_brake_command_query(lambda: SmoothBrakeCommand(corrected_pressure_mpa=3.0))
            m._sensor_timeout_counter = 1000
            m.run_monitoring_cycle()
            assert m.state == MonitorState.SENSOR_OFFLINE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M16-05: 偏差恢复
        print("\n[TC-M16-05] 偏差恢复")
        try:
            m = setup_monitor(actual_pressure=3.7)
            m.set_brake_command_query(lambda: SmoothBrakeCommand(corrected_pressure_mpa=3.0))
            for _ in range(120):
                m.run_monitoring_cycle()
            assert m.state == MonitorState.DEVIATION_WARNING
            m.set_actual_pressure_query(lambda: 3.1)
            for _ in range(20):
                m.run_monitoring_cycle()
            assert m.state == MonitorState.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M16-06: 紧急熔断
        print("\n[TC-M16-06] 紧急熔断")
        try:
            m = setup_monitor()
            m.emergency_shutdown()
            assert m.state == MonitorState.SYSTEM_PAUSED
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