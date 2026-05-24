#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-12
模块名称: 动力执行偏差监控单元
所属分区: 三、动力控制集群
核心职责: 实时对比 ad-mcc-11 输出的最终目标车速/油门指令所对应的期望车速与车辆实际车速
          （来自 CAN 总线车速传感器），计算速度偏差与响应延迟。当速度偏差超过预设阈值时，
          触发动力异常告警并上报 ECC，同时根据偏差量级建议降级或检查动力系统。
          不参与任何操控指令的修改，仅做监控与告警。

依赖模块:
    ad-mcc-11(加速平顺滤波单元，提供最终油门目标指令及期望加速度),
    车速传感器(CAN总线)
被依赖模块:
    ad-mcc-01(小脑总控调度核心，接收速度偏差告警),
    ad-mcc-03(全身运动状态归集中心，接收动力执行状态),
    ECC-12(通过 CerebellumBus 接收严重偏差告警)

安全约束:
  S-01: 本模块仅做监控与告警，不参与任何操控指令的修改与生成
  S-02: 车速传感器离线时必须明确标记并使用最后有效值填充，不可伪造在线状态
  S-03: 严重速度偏差告警（≥5km/h）必须立即上报 ECC-12，触发安全降级评估
  S-04: 偏差数据须与原始目标指令和传感器数据保持一致，不得篡改
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


# ==================== 枚举定义 ====================

class MonitorState(Enum):
    """动力执行偏差监控单元内部状态"""
    NORMAL_MONITOR = "normal_monitor"
    DEVIATION_WARNING = "deviation_warning"
    DEVIATION_CRITICAL = "deviation_critical"
    SENSOR_OFFLINE = "sensor_offline"
    SYSTEM_PAUSED = "system_paused"


class AlertSeverity(Enum):
    """告警严重等级"""
    NORMAL = "正常"
    WARNING = "预警"
    CRITICAL = "严重"


# ==================== 数据结构 ====================

@dataclass
class FinalThrottleCommand:
    """最终油门目标指令（来自 ad-mcc-11）"""
    timestamp: float = field(default_factory=time.time)
    filtered_throttle_pct: float = 0.0
    expected_acceleration_ms2: float = 0.0
    filter_method: str = ""
    filter_alpha: float = 0.0


@dataclass
class PowertrainStatus:
    """动力执行状态（发送至 ad-mcc-03）"""
    target_speed_kmh: float = 0.0
    actual_speed_kmh: float = 0.0
    speed_deviation_kmh: float = 0.0
    throttle_pct: float = 0.0
    response_latency_ms: float = 0.0
    online_status: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class DeviationAlert:
    """动力偏差告警"""
    deviation_amount: float = 0.0
    alert_level: AlertSeverity = AlertSeverity.NORMAL
    suggested_action: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class SensorFaultAlert:
    """传感器离线告警"""
    sensor_id: str = "vehicle_speed_sensor"
    fault_type: str = ""
    last_valid_time: float = 0.0
    impact: str = "动力偏差监控降级，使用最后有效值"


# ==================== 偏差判定阈值 ====================

SPEED_DEV_NORMAL_MAX_KMH = 2.0
SPEED_DEV_WARN_THRESHOLD_KMH = 3.0
SPEED_DEV_CRITICAL_THRESHOLD_KMH = 5.0

LATENCY_NORMAL_MAX_MS = 100.0
LATENCY_WARN_THRESHOLD_MS = 150.0
LATENCY_CRITICAL_THRESHOLD_MS = 200.0

# 预警需持续 500ms (50个周期 @ 10ms)
WARN_DELAY_CYCLES = 50
# 控制周期
CONTROL_PERIOD_S = 0.01
# 传感器超时阈值
SENSOR_TIMEOUT_MS = 100.0

# 噪声跳变阈值
SPIKE_DEVIATION_KMH = 10.0


# ==================== 主类定义 ====================

class PowertrainDeviationMonitor:
    """
    动力执行偏差监控单元
    
    职责:
    1. 对比目标车速与实际车速，计算速度偏差与响应延迟
    2. 偏差超限时上报告警 (预警持续确认，严重立即上报)
    3. 监测车速传感器健康，异常时使用最后有效值降级监控
    4. 周期性推送动力执行状态至 ad-mcc-03
    """

    def __init__(self):
        self.module_id = "ad-mcc-12"
        self.module_name = "动力执行偏差监控单元"
        self.version = "V1.0"

        self.state = MonitorState.NORMAL_MONITOR

        # 偏差持续计数器
        self._warn_counter = 0
        self._spike_counter = 0
        self._last_deviation = 0.0

        # 传感器相关
        self._sensor_timeout_counter = 0
        self._last_valid_speed = 0.0

        # 历史值
        self._prev_target_speed = 0.0
        self._prev_actual_speed = 0.0
        self._prev_throttle = 0.0

        # 统计
        self._total_alerts = 0
        self._total_faults = 0

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部回调
        self._query_throttle_command = None           # Callable[[], Optional[FinalThrottleCommand]]
        self._query_actual_speed = None               # Callable[[], float]
        self._query_target_speed = None               # Callable[[], Optional[float]]  来自 ad-mcc-01
        self._query_sensor_online = None              # Callable[[], bool]

        # 输出回调
        self._publish_powertrain_status = None        # Callable[[PowertrainStatus], None]
        self._publish_deviation_alert = None          # Callable[[DeviationAlert], None]
        self._publish_sensor_fault_alert = None       # Callable[[SensorFaultAlert], None]

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_throttle_command_query(self, callback):
        self._query_throttle_command = callback

    def set_actual_speed_query(self, callback):
        self._query_actual_speed = callback

    def set_target_speed_query(self, callback):
        self._query_target_speed = callback

    def set_sensor_online_query(self, callback):
        self._query_sensor_online = callback

    def set_powertrain_status_publisher(self, callback):
        self._publish_powertrain_status = callback

    def set_deviation_alert_publisher(self, callback):
        self._publish_deviation_alert = callback

    def set_sensor_fault_publisher(self, callback):
        self._publish_sensor_fault_alert = callback

    # ========== 主循环 ==========
    def run_monitoring_cycle(self) -> Optional[PowertrainStatus]:
        """
        执行一次动力偏差监控周期 (100Hz)
        """
        if self.state == MonitorState.SYSTEM_PAUSED:
            return None

        now = time.time()

        # 传感器在线检测
        sensor_online = self._query_sensor_online() if self._query_sensor_online else True
        if not sensor_online:
            self._sensor_timeout_counter += 1
            if self._sensor_timeout_counter * CONTROL_PERIOD_S * 1000 > SENSOR_TIMEOUT_MS:
                if self.state != MonitorState.SENSOR_OFFLINE:
                    self.state = MonitorState.SENSOR_OFFLINE
                    self._total_faults += 1
                    if self._publish_sensor_fault_alert:
                        self._publish_sensor_fault_alert(SensorFaultAlert(
                            fault_type="离线",
                            last_valid_time=now
                        ))
            # 使用最后有效值
            actual_speed = self._last_valid_speed
        else:
            self._sensor_timeout_counter = 0
            if self.state == MonitorState.SENSOR_OFFLINE:
                self.state = MonitorState.NORMAL_MONITOR
            actual_speed = self._query_actual_speed() if self._query_actual_speed else 0.0
            self._last_valid_speed = actual_speed

        # 接收最终油门指令
        throttle_cmd = self._query_throttle_command() if self._query_throttle_command else None
        if throttle_cmd is None:
            return None

        # 获取目标车速 (优先从 ad-mcc-01 获取，否则用期望加速度估算短期目标)
        target_speed = None
        if self._query_target_speed:
            target_speed = self._query_target_speed()
        if target_speed is None:
            # 简易估算：基于上一帧实际车速和期望加速度推算本帧目标车速
            target_speed = self._prev_actual_speed + throttle_cmd.expected_acceleration_ms2 * CONTROL_PERIOD_S * 3.6  # 转为 km/h
            # 标记为估算值（不影响监控逻辑，但可记录）

        throttle_pct = throttle_cmd.filtered_throttle_pct
        response_latency = (now - throttle_cmd.timestamp) * 1000.0

        # 计算偏差
        speed_dev = target_speed - actual_speed

        # 噪声处理：瞬间跳变滤波
        if abs(speed_dev - self._last_deviation) > SPIKE_DEVIATION_KMH:
            self._spike_counter += 1
            if self._spike_counter < 3:  # 连续3帧确认为真实跳变
                speed_dev = self._last_deviation  # 暂时沿用上帧偏差
            else:
                self._spike_counter = 0
        else:
            self._spike_counter = 0

        self._last_deviation = speed_dev

        # 偏差判定
        alert_level = AlertSeverity.NORMAL
        alert_action = ""
        dev_abs = abs(speed_dev)

        if dev_abs >= SPEED_DEV_CRITICAL_THRESHOLD_KMH or response_latency > LATENCY_CRITICAL_THRESHOLD_MS:
            self.state = MonitorState.DEVIATION_CRITICAL
            alert_level = AlertSeverity.CRITICAL
            alert_action = "触发降级，检查动力系统"
            self._warn_counter = 0
        elif dev_abs >= SPEED_DEV_WARN_THRESHOLD_KMH or response_latency > LATENCY_WARN_THRESHOLD_MS:
            self._warn_counter += 1
            if self._warn_counter >= WARN_DELAY_CYCLES:
                self.state = MonitorState.DEVIATION_WARNING
                alert_level = AlertSeverity.WARNING
                alert_action = "降低动力需求，检查动力系统"
                self._warn_counter = 0  # 防止持续重复上报，后续由状态管理
        else:
            self._warn_counter = 0
            if self.state not in (MonitorState.SENSOR_OFFLINE, MonitorState.SYSTEM_PAUSED):
                self.state = MonitorState.NORMAL_MONITOR

        # 构建动力执行状态
        status = PowertrainStatus(
            target_speed_kmh=round(target_speed, 2),
            actual_speed_kmh=round(actual_speed, 2),
            speed_deviation_kmh=round(speed_dev, 2),
            throttle_pct=round(throttle_pct, 2),
            response_latency_ms=round(response_latency, 2),
            online_status=sensor_online
        )
        if self._publish_powertrain_status:
            self._publish_powertrain_status(status)

        # 上报告警 (仅状态切换时或严重时？简化：预警/严重状态持续期间每次周期都上报？规格未明确，为避免风暴，只在状态切换时上报，或者如 ad-mcc-07 一样使用抑制。这里采用类似 ad-mcc-07 的告警抑制，但简单实现：预警状态首次触发时发送，后续不重复；严重每次发送)
        if alert_level in (AlertSeverity.WARNING, AlertSeverity.CRITICAL):
            # 简化处理：发送告警，由上层处理重复。或者我们记录上次告警等级，只有变化时才发送。
            # 这里采用简单逻辑：如果当前告警等级不同于上一周期，或者严重等级每次都发
            if not hasattr(self, '_last_alert_level') or self._last_alert_level != alert_level or alert_level == AlertSeverity.CRITICAL:
                self._last_alert_level = alert_level
                alert = DeviationAlert(
                    deviation_amount=round(speed_dev, 2),
                    alert_level=alert_level,
                    suggested_action=alert_action
                )
                if self._publish_deviation_alert:
                    self._publish_deviation_alert(alert)
                self._total_alerts += 1

        # 更新历史
        self._prev_target_speed = target_speed
        self._prev_actual_speed = actual_speed
        self._prev_throttle = throttle_pct

        return status

    # ========== 查询接口 ==========
    def get_state(self) -> MonitorState:
        return self.state

    def get_statistics(self) -> Dict[str, Any]:
        return {
            "state": self.state.value,
            "total_alerts": self._total_alerts,
            "total_faults": self._total_faults,
            "last_deviation": self._last_deviation
        }

    def emergency_shutdown(self):
        self.state = MonitorState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，保留最后有效状态")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 动力执行偏差监控单元 (ad-mcc-12) 演示")
    print("=" * 70)

    monitor = PowertrainDeviationMonitor()
    monitor.set_actual_speed_query(lambda: 59.0)
    monitor.set_target_speed_query(lambda: 60.0)
    monitor.set_sensor_online_query(lambda: True)

    print_separator("STEP 1: 正常偏差 (1km/h)")
    monitor.set_throttle_command_query(lambda: FinalThrottleCommand(
        filtered_throttle_pct=30.0,
        expected_acceleration_ms2=0.2
    ))
    status = monitor.run_monitoring_cycle()
    if status:
        print(f"  目标车速: {status.target_speed_kmh} km/h")
        print(f"  实际车速: {status.actual_speed_kmh} km/h")
        print(f"  速度偏差: {status.speed_deviation_kmh} km/h")
        print(f"  状态: {monitor.state.value}")

    print_separator("STEP 2: 预警偏差 (4km/h) 持续")
    monitor.set_actual_speed_query(lambda: 56.0)
    for i in range(60):  # 600ms 足够触发预警
        status = monitor.run_monitoring_cycle()
    print(f"  速度偏差: {status.speed_deviation_kmh} km/h")
    print(f"  状态: {monitor.state.value}")
    print(f"  累计告警: {monitor.get_statistics()['total_alerts']}")

    print_separator("STEP 3: 严重偏差 (6km/h)")
    monitor.set_actual_speed_query(lambda: 54.0)
    status = monitor.run_monitoring_cycle()
    print(f"  速度偏差: {status.speed_deviation_kmh} km/h")
    print(f"  状态: {monitor.state.value}")

    print("\n✅ 动力执行偏差监控单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-12 动力执行偏差监控单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_monitor(actual_speed=59.0, target_speed=60.0, sensor_online=True):
            m = PowertrainDeviationMonitor()
            m.set_actual_speed_query(lambda: actual_speed)
            m.set_target_speed_query(lambda: target_speed)
            m.set_sensor_online_query(lambda: sensor_online)
            return m

        # TC-M12-01: 正常偏差
        print("\n[TC-M12-01] 正常偏差 (1km/h)")
        try:
            m = setup_monitor(actual_speed=59.0)
            m.set_throttle_command_query(lambda: FinalThrottleCommand(filtered_throttle_pct=30.0, expected_acceleration_ms2=0.2))
            status = m.run_monitoring_cycle()
            assert status is not None
            assert abs(status.speed_deviation_kmh) <= 2.0
            assert m.state == MonitorState.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M12-02: 预警偏差持续
        print("\n[TC-M12-02] 预警偏差 (4km/h) 持续 500ms")
        try:
            m = setup_monitor(actual_speed=56.0)
            m.set_throttle_command_query(lambda: FinalThrottleCommand(filtered_throttle_pct=35.0))
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m.state == MonitorState.DEVIATION_WARNING
            assert m.get_statistics()['total_alerts'] >= 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M12-03: 严重偏差立即告警
        print("\n[TC-M12-03] 严重偏差 (6km/h)")
        try:
            m = setup_monitor(actual_speed=54.0)
            m.set_throttle_command_query(lambda: FinalThrottleCommand(filtered_throttle_pct=40.0))
            m.run_monitoring_cycle()
            assert m.state == MonitorState.DEVIATION_CRITICAL
            assert m.get_statistics()['total_alerts'] >= 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M12-04: 传感器离线
        print("\n[TC-M12-04] 传感器离线")
        try:
            m = setup_monitor(sensor_online=False)
            m.set_throttle_command_query(lambda: FinalThrottleCommand(filtered_throttle_pct=30.0))
            # 需要超过超时时间，手动设置计数器
            m._sensor_timeout_counter = 1000  # 模拟已超时
            m.run_monitoring_cycle()
            assert m.state == MonitorState.SENSOR_OFFLINE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M12-05: 紧急熔断
        print("\n[TC-M12-05] 紧急熔断")
        try:
            m = setup_monitor()
            m.emergency_shutdown()
            assert m.state == MonitorState.SYSTEM_PAUSED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M12-06: 偏差恢复
        print("\n[TC-M12-06] 偏差恢复")
        try:
            m = setup_monitor(actual_speed=56.0)
            m.set_throttle_command_query(lambda: FinalThrottleCommand(filtered_throttle_pct=35.0))
            for _ in range(60):
                m.run_monitoring_cycle()
            assert m.state == MonitorState.DEVIATION_WARNING
            # 恢复
            m.set_actual_speed_query(lambda: 59.8)
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