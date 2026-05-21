#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-07
模块名称: 转向执行偏差监控单元
所属分区: 二、转向控制集群
核心职责: 实时对比 ad-mcc-06 输出的约束后目标方向盘转角与实际方向盘转角（来自 CAN 总线
          转角传感器），计算偏差量（角度偏差、速率偏差、响应延迟）。当偏差超出预设阈值时，
          即时向 ad-mcc-01 上报告警，并标记该转向指令的执行质量。同时将偏差数据周期性推送
          至 ad-mcc-36 执行闭环反馈单元。不参与任何场景判断与驾驶决策。

依赖模块:
    ad-mcc-06(横向冲击度约束单元，提供约束后目标转角指令),
    ad-mcc-36(执行闭环反馈单元，消费偏差数据),
    ad-mcc-38(执行日志记录单元，记录偏差事件)
被依赖模块:
    ad-mcc-01(小脑总控调度核心，接收偏差超限告警)

安全约束:
  S-01: 本模块仅做偏差监控与告警上报，不直接干预转向执行
  S-02: 转角传感器离线时必须使用最后有效值继续监控并明确标记数据质量降级
  S-03: 严重偏差（角度>10°或速率>40%）应立即上报，不得等待判定延迟
  S-04: 告警抑制机制不得遗漏新的告警类型
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


# ==================== 枚举定义 ====================

class MonitorState(Enum):
    """转向偏差监控单元内部状态"""
    NORMAL_MONITOR = "normal_monitor"
    DEVIATION_ALERT = "deviation_alert"
    SENSOR_FAULT = "sensor_fault"
    SYSTEM_PAUSED = "system_paused"


class DeviationType(Enum):
    """偏差类型"""
    ANGLE = "角度偏差"
    RATE = "速率偏差"
    LATENCY = "响应延迟"
    SENSOR_TIMEOUT = "传感器超时"


class AlertSeverity(Enum):
    """告警严重等级"""
    NORMAL = "正常"
    WARNING = "警告"
    CRITICAL = "严重"


# ==================== 数据结构 ====================

@dataclass
class ConstrainedSteeringCommand:
    """约束后目标方向盘转角指令（来自 ad-mcc-06）"""
    msg_id: str = ""
    timestamp: float = field(default_factory=time.time)
    constrained_angle_deg: float = 0.0
    constrained_angle_rate_deg_per_s: float = 0.0
    original_rate_deg_per_s: float = 0.0
    constraint_triggered: bool = False
    constraint_reason: str = ""


@dataclass
class ActualSteeringData:
    """实际方向盘转角数据（来自 CAN 总线）"""
    angle_deg: float = 0.0
    angle_rate_deg_per_s: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class SensorHealth:
    """转角传感器健康状态"""
    sensor_id: str = "steering_angle_sensor"
    online: bool = True
    data_valid: bool = True
    signal_quality: str = "良好"


@dataclass
class DeviationAlert:
    """转向偏差超限告警"""
    alert_type: str = ""
    target_value: float = 0.0
    actual_value: float = 0.0
    deviation_amount: float = 0.0
    duration_ms: float = 0.0
    suggested_action: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class DeviationData:
    """转向执行偏差数据（发送至 ad-mcc-36）"""
    command_id: str = ""
    angle_deviation_deg: float = 0.0
    rate_deviation_pct: float = 0.0
    response_latency_ms: float = 0.0
    online_status: str = "正常"
    timestamp: float = field(default_factory=time.time)


@dataclass
class SensorFaultAlert:
    """传感器异常告警"""
    sensor_id: str = ""
    fault_type: str = ""
    last_valid_value: float = 0.0
    impact_assessment: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class MonitorStatusReport:
    """偏差监控状态上报"""
    current_state: MonitorState = MonitorState.NORMAL_MONITOR
    recent_deviation_deg: float = 0.0
    alert_count: int = 0
    sensor_health: str = "良好"
    timestamp: float = field(default_factory=time.time)


# ==================== 主类定义 ====================

class SteeringDeviationMonitor:
    """
    转向执行偏差监控单元
    
    职责:
    1. 实时对比目标转角与实际转角，计算角度偏差、速率偏差、响应延迟
    2. 偏差超阈值时即时上报告警至 ad-mcc-01
    3. 周期性推送偏差数据至 ad-mcc-36 供闭环反馈
    4. 监控转角传感器健康状态，异常时降级处理
    """

    # 偏差判定阈值
    ANGLE_NORMAL_MAX_DEG = 3.0
    ANGLE_WARN_THRESHOLD_DEG = 5.0
    ANGLE_CRITICAL_THRESHOLD_DEG = 10.0

    RATE_NORMAL_MAX_PCT = 15.0
    RATE_WARN_THRESHOLD_PCT = 20.0
    RATE_CRITICAL_THRESHOLD_PCT = 40.0

    LATENCY_NORMAL_MAX_MS = 30.0
    LATENCY_WARN_THRESHOLD_MS = 50.0
    LATENCY_CRITICAL_THRESHOLD_MS = 100.0

    SENSOR_TIMEOUT_NORMAL_MAX_MS = 10.0
    SENSOR_TIMEOUT_WARN_MS = 50.0
    SENSOR_TIMEOUT_CRITICAL_MS = 100.0

    # 判定延迟（控制周期数）
    ANGLE_WARN_DELAY_CYCLES = 50    # 500ms / 10ms
    RATE_WARN_DELAY_CYCLES = 50
    LATENCY_WARN_DELAY_CYCLES = 100  # 1000ms / 10ms

    # 控制周期（秒）
    CONTROL_PERIOD_S = 0.01  # 100Hz

    # 状态上报间隔（秒）
    REPORT_INTERVAL_S = 1.0

    def __init__(self):
        self.module_id = "ad-mcc-07"
        self.module_name = "转向执行偏差监控单元"
        self.version = "V1.0"

        self.state = MonitorState.NORMAL_MONITOR

        # 偏差持续计数器
        self._angle_exceed_counter: int = 0
        self._rate_exceed_counter: int = 0
        self._latency_exceed_counter: int = 0

        # 告警抑制标记
        self._alert_suppressed: Dict[str, bool] = {
            "angle": False,
            "rate": False,
            "latency": False,
        }

        # 最后有效值（传感器离线时使用）
        self._last_valid_angle: float = 0.0
        self._last_valid_rate: float = 0.0

        # 统计
        self._total_alerts: int = 0
        self._total_fault_events: int = 0

        # 状态上报
        self._last_report_time: float = 0.0
        self._recent_deviation: float = 0.0

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调
        self._query_constrained_command = None       # Callable[[], Optional[ConstrainedSteeringCommand]]
        self._query_actual_angle = None              # Callable[[], float]
        self._query_actual_rate = None               # Callable[[], float]
        self._query_sensor_health = None             # Callable[[], SensorHealth]

        # 输出回调
        self._publish_alert = None                   # Callable[[DeviationAlert], None]
        self._publish_deviation_data = None          # Callable[[DeviationData], None]
        self._publish_sensor_fault = None            # Callable[[SensorFaultAlert], None]
        self._publish_status_report = None           # Callable[[MonitorStatusReport], None]
        self._publish_event_log = None               # Callable[[Dict[str, Any]], None]

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_constrained_command_query(self, callback):
        self._query_constrained_command = callback

    def set_actual_angle_query(self, callback):
        self._query_actual_angle = callback

    def set_actual_rate_query(self, callback):
        self._query_actual_rate = callback

    def set_sensor_health_query(self, callback):
        self._query_sensor_health = callback

    def set_alert_publisher(self, callback):
        self._publish_alert = callback

    def set_deviation_data_publisher(self, callback):
        self._publish_deviation_data = callback

    def set_sensor_fault_publisher(self, callback):
        self._publish_sensor_fault = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    # ========== 主循环 ==========
    def run_monitoring_cycle(self) -> Optional[DeviationData]:
        """
        执行一次偏差监控周期（100Hz）
        
        Returns:
            偏差数据，若无新目标指令则返回 None
        """
        if self.state == MonitorState.SYSTEM_PAUSED:
            return None

        now = time.time()

        # 传感器健康检查
        sensor_health = self._query_sensor_health() if self._query_sensor_health else SensorHealth()
        if not sensor_health.online or not sensor_health.data_valid:
            if self.state != MonitorState.SENSOR_FAULT:
                self.state = MonitorState.SENSOR_FAULT
                self._total_fault_events += 1
                self._publish_sensor_fault_alert(sensor_health)
            # 使用最后有效值继续监控
        else:
            if self.state == MonitorState.SENSOR_FAULT:
                self.state = MonitorState.NORMAL_MONITOR

        # 接收目标转角指令
        command = self._query_constrained_command() if self._query_constrained_command else None
        if command is None:
            return None

        # 获取实际转角
        actual_angle = self._query_actual_angle() if self._query_actual_angle else 0.0
        actual_rate = self._query_actual_rate() if self._query_actual_rate else 0.0

        # 更新最后有效值
        if sensor_health.online and sensor_health.data_valid:
            self._last_valid_angle = actual_angle
            self._last_valid_rate = actual_rate
        else:
            actual_angle = self._last_valid_angle
            actual_rate = self._last_valid_rate

        target_angle = command.constrained_angle_deg
        target_rate = command.constrained_angle_rate_deg_per_s

        # 计算偏差
        angle_deviation = abs(target_angle - actual_angle)
        rate_deviation_pct = 0.0
        if abs(target_rate) > 0.1:
            rate_deviation_pct = abs(target_rate - actual_rate) / abs(target_rate) * 100.0
        elif abs(actual_rate) > 0.1:
            rate_deviation_pct = 100.0  # 目标为0但实际有速率，视为完全偏差

        response_latency_ms = (now - command.timestamp) * 1000.0

        # 偏差判定与告警
        self._evaluate_and_alert(
            angle_deviation, rate_deviation_pct, response_latency_ms,
            target_angle, actual_angle, target_rate, actual_rate
        )

        # 构建偏差数据
        deviation_data = DeviationData(
            command_id=command.msg_id,
            angle_deviation_deg=round(angle_deviation, 3),
            rate_deviation_pct=round(rate_deviation_pct, 2),
            response_latency_ms=round(response_latency_ms, 2),
            online_status="降级" if self.state == MonitorState.SENSOR_FAULT else "正常"
        )

        # 推送至 ad-mcc-36
        if self._publish_deviation_data:
            self._publish_deviation_data(deviation_data)

        # 偏差事件记录（非零偏差）
        if angle_deviation > self.ANGLE_NORMAL_MAX_DEG or rate_deviation_pct > self.RATE_NORMAL_MAX_PCT:
            if self._publish_event_log:
                self._publish_event_log({
                    "event_type": "steering_deviation",
                    "target_angle": target_angle,
                    "actual_angle": actual_angle,
                    "angle_deviation": angle_deviation,
                    "rate_deviation_pct": rate_deviation_pct,
                    "latency_ms": response_latency_ms,
                    "timestamp": now
                })

        self._recent_deviation = angle_deviation

        # 周期性状态上报
        if now - self._last_report_time >= self.REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_status_report:
                self._publish_status_report(MonitorStatusReport(
                    current_state=self.state,
                    recent_deviation_deg=round(self._recent_deviation, 3),
                    alert_count=self._total_alerts,
                    sensor_health="良好" if sensor_health.online else "异常"
                ))

        return deviation_data

    # ========== 偏差评估与告警 ==========
    def _evaluate_and_alert(self, angle_dev: float, rate_pct: float, latency_ms: float,
                            target_angle: float, actual_angle: float,
                            target_rate: float, actual_rate: float):
        """评估偏差并触发告警"""
        alert_triggered = False
        alert_type_parts = []

        # 严重角度偏差：立即上报
        if angle_dev > self.ANGLE_CRITICAL_THRESHOLD_DEG:
            alert_triggered = True
            alert_type_parts.append("严重角度偏差")
            self._angle_exceed_counter = 0
        elif angle_dev > self.ANGLE_WARN_THRESHOLD_DEG:
            self._angle_exceed_counter += 1
            if self._angle_exceed_counter >= self.ANGLE_WARN_DELAY_CYCLES:
                alert_triggered = True
                alert_type_parts.append("角度偏差超限")
                self._angle_exceed_counter = 0
        else:
            self._angle_exceed_counter = max(0, self._angle_exceed_counter - 1)

        # 严重速率偏差：立即上报
        if rate_pct > self.RATE_CRITICAL_THRESHOLD_PCT:
            alert_triggered = True
            alert_type_parts.append("严重速率偏差")
            self._rate_exceed_counter = 0
        elif rate_pct > self.RATE_WARN_THRESHOLD_PCT:
            self._rate_exceed_counter += 1
            if self._rate_exceed_counter >= self.RATE_WARN_DELAY_CYCLES:
                alert_triggered = True
                alert_type_parts.append("速率偏差超限")
                self._rate_exceed_counter = 0
        else:
            self._rate_exceed_counter = max(0, self._rate_exceed_counter - 1)

        # 严重响应延迟：立即上报
        if latency_ms > self.LATENCY_CRITICAL_THRESHOLD_MS:
            alert_triggered = True
            alert_type_parts.append("响应延迟严重")
            self._latency_exceed_counter = 0
        elif latency_ms > self.LATENCY_WARN_THRESHOLD_MS:
            self._latency_exceed_counter += 1
            if self._latency_exceed_counter >= self.LATENCY_WARN_DELAY_CYCLES:
                alert_triggered = True
                alert_type_parts.append("响应延迟超限")
                self._latency_exceed_counter = 0
        else:
            self._latency_exceed_counter = max(0, self._latency_exceed_counter - 1)

        # 发送告警
        if alert_triggered:
            alert_type = " + ".join(alert_type_parts)
            if not self._is_alert_suppressed(alert_type_parts):
                self.state = MonitorState.DEVIATION_ALERT
                self._total_alerts += 1
                deviation_amount = max(angle_dev, rate_pct if rate_pct > 100 else 0)
                if self._publish_alert:
                    self._publish_alert(DeviationAlert(
                        alert_type=alert_type,
                        target_value=target_angle,
                        actual_value=actual_angle,
                        deviation_amount=round(deviation_amount, 3),
                        suggested_action="检查转向执行器或触发降级"
                    ))
                self._suppress_alert(alert_type_parts)
        else:
            # 偏差恢复，重置告警抑制
            if self.state == MonitorState.DEVIATION_ALERT:
                self.state = MonitorState.NORMAL_MONITOR
            self._reset_alert_suppression()

    # ========== 告警抑制机制 ==========
    def _is_alert_suppressed(self, alert_types: List[str]) -> bool:
        """检查当前告警类型是否已被抑制"""
        for alert_type in alert_types:
            if "角度" in alert_type and not self._alert_suppressed.get("angle", False):
                return False
            if "速率" in alert_type and not self._alert_suppressed.get("rate", False):
                return False
            if "延迟" in alert_type and not self._alert_suppressed.get("latency", False):
                return False
        return True

    def _suppress_alert(self, alert_types: List[str]):
        """抑制已上报的告警类型"""
        for alert_type in alert_types:
            if "角度" in alert_type:
                self._alert_suppressed["angle"] = True
            if "速率" in alert_type:
                self._alert_suppressed["rate"] = True
            if "延迟" in alert_type:
                self._alert_suppressed["latency"] = True

    def _reset_alert_suppression(self):
        """重置所有告警抑制标记"""
        self._alert_suppressed = {"angle": False, "rate": False, "latency": False}

    # ========== 传感器故障告警 ==========
    def _publish_sensor_fault_alert(self, health: SensorHealth):
        """发布传感器异常告警"""
        if self._publish_sensor_fault:
            self._publish_sensor_fault(SensorFaultAlert(
                sensor_id=health.sensor_id,
                fault_type="离线" if not health.online else "数据校验失败",
                last_valid_value=self._last_valid_angle,
                impact_assessment="转向偏差监控降级，使用最后有效值"
            ))

    # ========== 查询接口 ==========
    def get_state(self) -> MonitorState:
        return self.state

    def get_recent_deviation(self) -> float:
        return self._recent_deviation

    # ========== 日志与统计 ==========
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

    def get_statistics(self) -> Dict[str, Any]:
        return {
            "state": self.state.value,
            "total_alerts": self._total_alerts,
            "total_fault_events": self._total_fault_events,
            "recent_deviation_deg": self._recent_deviation,
        }

    def emergency_shutdown(self):
        self.state = MonitorState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，保持最后偏差数据")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 转向执行偏差监控单元 (ad-mcc-07) 演示")
    print("=" * 70)

    monitor = SteeringDeviationMonitor()
    monitor.set_actual_angle_query(lambda: 14.5)
    monitor.set_actual_rate_query(lambda: 95.0)
    monitor.set_sensor_health_query(lambda: SensorHealth(online=True, data_valid=True))

    print_separator("STEP 1: 正常偏差监控")
    monitor.set_constrained_command_query(lambda: ConstrainedSteeringCommand(
        msg_id="CMD-001",
        constrained_angle_deg=15.0,
        constrained_angle_rate_deg_per_s=100.0,
    ))
    data = monitor.run_monitoring_cycle()
    if data:
        print(f"  角度偏差: {data.angle_deviation_deg}°")
        print(f"  速率偏差: {data.rate_deviation_pct}%")
        print(f"  响应延迟: {data.response_latency_ms}ms")
        print(f"  在线状态: {data.online_status}")

    print_separator("STEP 2: 偏差超限触发告警")
    monitor.set_actual_angle_query(lambda: 8.0)  # 目标15°，实际8°，偏差7°
    for _ in range(60):  # 持续600ms触发告警
        data = monitor.run_monitoring_cycle()
    print(f"  当前状态: {monitor.state.value}")
    print(f"  累计告警数: {monitor.get_statistics()['total_alerts']}")

    print_separator("STEP 3: 传感器离线降级")
    monitor.set_sensor_health_query(lambda: SensorHealth(online=False, data_valid=False))
    data = monitor.run_monitoring_cycle()
    print(f"  当前状态: {monitor.state.value}")
    print(f"  在线状态: {data.online_status if data else 'N/A'}")

    print("\n✅ 转向执行偏差监控单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-07 转向执行偏差监控单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_monitor(actual_angle=14.8, actual_rate=95.0, sensor_online=True, sensor_valid=True):
            m = SteeringDeviationMonitor()
            m.set_actual_angle_query(lambda: actual_angle)
            m.set_actual_rate_query(lambda: actual_rate)
            m.set_sensor_health_query(lambda: SensorHealth(online=sensor_online, data_valid=sensor_valid))
            return m

        # TC-M07-01: 正常偏差不触发告警
        print("\n[TC-M07-01] 正常偏差不触发告警")
        try:
            m = setup_monitor(actual_angle=14.8)
            m.set_constrained_command_query(lambda: ConstrainedSteeringCommand(
                msg_id="T01", constrained_angle_deg=15.0, constrained_angle_rate_deg_per_s=100.0
            ))
            data = m.run_monitoring_cycle()
            assert data is not None
            assert data.angle_deviation_deg <= 3.0
            assert m.state == MonitorState.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M07-02: 角度偏差超限触发告警
        print("\n[TC-M07-02] 角度偏差超限触发告警")
        try:
            m = setup_monitor(actual_angle=8.0)
            m.set_constrained_command_query(lambda: ConstrainedSteeringCommand(
                msg_id="T02", constrained_angle_deg=15.0, constrained_angle_rate_deg_per_s=100.0
            ))
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m.state == MonitorState.DEVIATION_ALERT
            assert m._total_alerts >= 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M07-03: 严重角度偏差立即告警
        print("\n[TC-M07-03] 严重角度偏差立即告警")
        try:
            m = setup_monitor(actual_angle=3.0)
            m.set_constrained_command_query(lambda: ConstrainedSteeringCommand(
                msg_id="T03", constrained_angle_deg=15.0, constrained_angle_rate_deg_per_s=100.0
            ))
            m.run_monitoring_cycle()  # 12°偏差，应立即触发
            assert m._total_alerts >= 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M07-04: 传感器离线触发 SENSOR_FAULT
        print("\n[TC-M07-04] 传感器离线触发 SENSOR_FAULT")
        try:
            m = setup_monitor(sensor_online=False, sensor_valid=False)
            m.set_constrained_command_query(lambda: ConstrainedSteeringCommand(
                msg_id="T04", constrained_angle_deg=15.0
            ))
            m.run_monitoring_cycle()
            assert m.state == MonitorState.SENSOR_FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M07-05: 紧急熔断
        print("\n[TC-M07-05] 紧急熔断")
        try:
            m = setup_monitor()
            m.emergency_shutdown()
            assert m.state == MonitorState.SYSTEM_PAUSED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M07-06: 告警抑制与恢复
        print("\n[TC-M07-06] 告警抑制与恢复")
        try:
            m = setup_monitor(actual_angle=8.0)
            m.set_constrained_command_query(lambda: ConstrainedSteeringCommand(
                msg_id="T06", constrained_angle_deg=15.0
            ))
            # 触发告警
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m._total_alerts == 1
            # 继续同一告警不应增加计数（抑制）
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m._total_alerts == 1
            # 恢复后重置
            m.set_actual_angle_query(lambda: 14.8)
            for _ in range(10):
                m.run_monitoring_cycle()
            assert m.state == MonitorState.NORMAL_MONITOR
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