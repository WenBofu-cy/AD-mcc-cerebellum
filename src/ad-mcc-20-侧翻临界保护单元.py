#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-20
模块名称: 侧翻临界保护单元
所属分区: 五、车身姿态稳定
核心职责: 基于 ad-mcc-18 提供的侧向加速度、侧倾角、侧翻风险等级及当前车速，实时判断车辆
          是否面临侧翻危险。当侧向加速度逼近侧翻临界阈值或侧倾角过大时，主动执行分级保护
          策略：一级预警限制车速与转向速率；二级临界主动制动、降扭并禁止转向加剧；三级紧急
          全力制动与最大程度限速。所有保护动作须经 ad-mcc-02 边界校验，且当与 ad-mcc-19 横摆
          控制同时激活时，本模块拥有更高优先级。不参与常规工况下的行驶决策。

依赖模块:
    ad-mcc-18(车身姿态实时监测单元),
    车速传感器(CAN总线),
    ad-mcc-32(车辆尺寸参数管理单元),
    ad-mcc-34(动力与制动参数管理单元)
被依赖模块:
    ad-mcc-01(小脑总控调度核心),
    ad-mcc-02(运动生理边界闸门),
    ad-mcc-19(横摆稳定控制单元),
    ESP 执行器,
    动力控制器

安全约束:
  S-01: 三级紧急保护时，制动指令可达到制动系统最大允许压力，安全优先于舒适性
  S-02: 侧翻保护优先级高于横摆稳定控制（ad-mcc-19），两者同时激活时本模块主导制动分配
  S-03: 保护动作不得突破 ad-mcc-02 设定的车辆动力学绝对极限
  S-04: 本模块仅负责侧翻风险应对，不参与常规制动、转向或驱动控制
  S-05: 三级紧急保护时，禁止转向加剧指令必须无条件执行，防止驾驶员误操作加剧侧翻
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class ProtectionLevel(Enum):
    NORMAL_MONITOR = "normal_monitor"
    LEVEL1_WARNING = "level1_warning"
    LEVEL2_CRITICAL = "level2_critical"
    LEVEL3_EMERGENCY = "level3_emergency"
    SYSTEM_PAUSED = "system_paused"


class RollRiskLevel(Enum):
    LOW = "低风险"
    MEDIUM = "中风险"
    HIGH = "高风险"
    CRITICAL = "临界"


@dataclass
class AttitudeVector:
    lateral_accel_ms2: float = 0.0
    roll_deg: float = 0.0
    yaw_rate_deg_per_s: float = 0.0
    roll_risk: RollRiskLevel = RollRiskLevel.LOW
    confidence: float = 1.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class RollRiskReport:
    risk_level: RollRiskLevel = RollRiskLevel.LOW
    lateral_accel_ms2: float = 0.0
    roll_angle_deg: float = 0.0
    safety_margin_pct: float = 100.0
    trigger_condition: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class VehicleDimensionParams:
    cg_height_m: float = 0.55
    track_width_m: float = 1.6


@dataclass
class ActuatorLimits:
    max_brake_pressure_mpa: float = 10.0
    max_torque_reduction_ratio: float = 0.7


@dataclass
class SpeedLimitCommand:
    target_speed_limit_kmh: float = 120.0
    reason: str = ""
    level: int = 0


@dataclass
class BrakeIntervention:
    pressure_mpa: float = 0.0
    reason: str = ""
    priority: int = 0


@dataclass
class SteerLockCommand:
    lock_steering: bool = False
    max_angle_rate_deg_per_s: float = 500.0


@dataclass
class TorqueReduction:
    reduction_ratio: float = 0.0
    reason: str = ""


@dataclass
class YawControlStatus:
    state: str = "normal_monitor"
    brake_request_active: bool = False
    torque_request_active: bool = False


@dataclass
class ProtectionStatus:
    state: ProtectionLevel = ProtectionLevel.NORMAL_MONITOR
    lateral_accel: float = 0.0
    roll_angle: float = 0.0
    safety_margin_pct: float = 100.0
    current_speed_limit: float = 120.0
    brake_pressure: float = 0.0
    torque_reduction: float = 0.0


CONTROL_PERIOD_S = 0.005
REPORT_INTERVAL_S = 0.5


class RolloverProtectionController:
    def __init__(self):
        self.module_id = "ad-mcc-20"
        self.module_name = "侧翻临界保护单元"
        self.version = "V1.0"

        self.state = ProtectionLevel.NORMAL_MONITOR
        self._prev_state = ProtectionLevel.NORMAL_MONITOR
        self._vehicle = VehicleDimensionParams()
        self._actuator_limits = ActuatorLimits()
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []
        self._protection_active = False

        self._query_attitude = None
        self._query_roll_risk = None
        self._query_speed = None
        self._query_vehicle_params = None
        self._query_actuator_limits = None
        self._query_yaw_status = None

        self._publish_speed_limit = None
        self._publish_brake = None
        self._publish_steer_lock = None
        self._publish_torque_reduction = None
        self._publish_status = None
        self._publish_event_log = None
        self._publish_yaw_coordination = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_attitude_query(self, callback):
        self._query_attitude = callback

    def set_roll_risk_query(self, callback):
        self._query_roll_risk = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_vehicle_params_query(self, callback):
        self._query_vehicle_params = callback

    def set_actuator_limits_query(self, callback):
        self._query_actuator_limits = callback

    def set_yaw_status_query(self, callback):
        self._query_yaw_status = callback

    def set_speed_limit_publisher(self, callback):
        self._publish_speed_limit = callback

    def set_brake_publisher(self, callback):
        self._publish_brake = callback

    def set_steer_lock_publisher(self, callback):
        self._publish_steer_lock = callback

    def set_torque_reduction_publisher(self, callback):
        self._publish_torque_reduction = callback

    def set_status_publisher(self, callback):
        self._publish_status = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def set_yaw_coordination_publisher(self, callback):
        self._publish_yaw_coordination = callback

    def run_protection_cycle(self):
        now = time.time()
        if self.state == ProtectionLevel.SYSTEM_PAUSED:
            return

        attitude = self._query_attitude() if self._query_attitude else None
        risk_report = self._query_roll_risk() if self._query_roll_risk else None
        if attitude is None or attitude.confidence < 0.5:
            return

        speed = self._query_speed() if self._query_speed else 0.0
        if self._query_vehicle_params:
            params = self._query_vehicle_params()
            if params:
                self._vehicle = params
        if self._query_actuator_limits:
            limits = self._query_actuator_limits()
            if limits:
                self._actuator_limits = limits

        risk = risk_report.risk_level if risk_report else attitude.roll_risk
        self._prev_state = self.state

        if risk == RollRiskLevel.CRITICAL:
            self.state = ProtectionLevel.LEVEL3_EMERGENCY
        elif risk == RollRiskLevel.HIGH:
            self.state = ProtectionLevel.LEVEL2_CRITICAL
        elif risk == RollRiskLevel.MEDIUM:
            self.state = ProtectionLevel.LEVEL1_WARNING
        else:
            self.state = ProtectionLevel.NORMAL_MONITOR

        brake_pressure = 0.0
        torque_reduction = 0.0

        if self.state == ProtectionLevel.LEVEL3_EMERGENCY:
            brake_pressure = self._actuator_limits.max_brake_pressure_mpa
            torque_reduction = 0.7
            self._apply_protection(
                speed_limit=0.0,
                brake_pressure=brake_pressure,
                lock_steer=True,
                steer_rate=0.0,
                torque_reduction=torque_reduction,
                reason="侧翻紧急保护"
            )
        elif self.state == ProtectionLevel.LEVEL2_CRITICAL:
            brake_pressure = 1.5
            torque_reduction = 0.3
            self._apply_protection(
                speed_limit=40.0,
                brake_pressure=brake_pressure,
                lock_steer=False,
                steer_rate=150.0,
                torque_reduction=torque_reduction,
                reason="侧翻临界保护"
            )
        elif self.state == ProtectionLevel.LEVEL1_WARNING:
            self._apply_protection(
                speed_limit=speed,
                brake_pressure=0.0,
                lock_steer=False,
                steer_rate=300.0,
                torque_reduction=0.0,
                reason="侧翻预警"
            )
        else:
            self._release_all()

        # 横摆协调
        yaw_status = self._query_yaw_status() if self._query_yaw_status else None
        if yaw_status and yaw_status.state != "normal_monitor":
            if self.state in (ProtectionLevel.LEVEL2_CRITICAL, ProtectionLevel.LEVEL3_EMERGENCY):
                if self._publish_yaw_coordination:
                    self._publish_yaw_coordination({
                        "event": "rollover_priority",
                        "rollover_state": self.state.value,
                        "action": "degrade_yaw_control"
                    })

        # 状态上报
        if self.state != self._prev_state or (now - self._last_report_time) >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_status:
                self._publish_status(ProtectionStatus(
                    state=self.state,
                    lateral_accel=attitude.lateral_accel_ms2,
                    roll_angle=attitude.roll_deg,
                    safety_margin_pct=risk_report.safety_margin_pct if risk_report else 100.0,
                    current_speed_limit=speed if self.state == ProtectionLevel.LEVEL1_WARNING else (40.0 if self.state == ProtectionLevel.LEVEL2_CRITICAL else 0.0),
                    brake_pressure=brake_pressure,
                    torque_reduction=torque_reduction
                ))
            if self.state != self._prev_state and self._publish_event_log:
                self._publish_event_log({
                    "event": "rollover_protection_state_change",
                    "from": self._prev_state.value,
                    "to": self.state.value,
                    "risk": risk.value,
                    "timestamp": now,
                })

    def _apply_protection(self, speed_limit, brake_pressure, lock_steer, steer_rate, torque_reduction, reason):
        self._protection_active = True
        if self._publish_speed_limit:
            self._publish_speed_limit(SpeedLimitCommand(
                target_speed_limit_kmh=speed_limit,
                reason=reason,
                level=1 if speed_limit > 0 else 3,
            ))
        if brake_pressure > 0 and self._publish_brake:
            self._publish_brake(BrakeIntervention(
                pressure_mpa=brake_pressure,
                reason=reason,
                priority=3 if lock_steer else 2,
            ))
        if self._publish_steer_lock:
            self._publish_steer_lock(SteerLockCommand(
                lock_steering=lock_steer,
                max_angle_rate_deg_per_s=steer_rate,
            ))
        if self._publish_torque_reduction:
            self._publish_torque_reduction(TorqueReduction(
                reduction_ratio=torque_reduction,
                reason=reason,
            ))

    def _release_all(self):
        self._protection_active = False
        if self._publish_brake:
            self._publish_brake(BrakeIntervention(pressure_mpa=0.0, reason="侧翻风险解除"))
        if self._publish_steer_lock:
            self._publish_steer_lock(SteerLockCommand(lock_steering=False, max_angle_rate_deg_per_s=500.0))
        if self._publish_torque_reduction:
            self._publish_torque_reduction(TorqueReduction(reduction_ratio=0.0, reason="侧翻风险解除"))
        if self._publish_speed_limit:
            self._publish_speed_limit(SpeedLimitCommand(target_speed_limit_kmh=250.0, reason="侧翻风险解除", level=0))
        if self._publish_yaw_coordination:
            self._publish_yaw_coordination({
                "event": "rollover_priority_release",
                "action": "restore_yaw_control"
            })

    def get_state(self) -> ProtectionLevel:
        return self.state

    def emergency_shutdown(self):
        self.state = ProtectionLevel.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 侧翻临界保护单元 (ad-mcc-20) 演示")
    print("=" * 70)

    ctrl = RolloverProtectionController()
    ctrl.set_speed_query(lambda: 80.0)
    ctrl.set_attitude_query(lambda: AttitudeVector(lateral_accel_ms2=2.0, roll_deg=2.0, roll_risk=RollRiskLevel.LOW))
    ctrl.set_roll_risk_query(lambda: RollRiskReport(risk_level=RollRiskLevel.LOW))
    ctrl.set_vehicle_params_query(lambda: VehicleDimensionParams())
    ctrl.set_actuator_limits_query(lambda: ActuatorLimits())

    print_separator("STEP 1: 正常监控")
    ctrl.run_protection_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 2: 中风险预警")
    ctrl.set_roll_risk_query(lambda: RollRiskReport(risk_level=RollRiskLevel.MEDIUM, lateral_accel_ms2=4.0, roll_angle_deg=4.0))
    ctrl.set_attitude_query(lambda: AttitudeVector(lateral_accel_ms2=4.0, roll_deg=4.0, roll_risk=RollRiskLevel.MEDIUM))
    ctrl.run_protection_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 3: 临界紧急")
    ctrl.set_roll_risk_query(lambda: RollRiskReport(risk_level=RollRiskLevel.CRITICAL, lateral_accel_ms2=9.0, roll_angle_deg=11.0))
    ctrl.set_attitude_query(lambda: AttitudeVector(lateral_accel_ms2=9.0, roll_deg=11.0, roll_risk=RollRiskLevel.CRITICAL))
    ctrl.run_protection_cycle()
    print(f"  状态: {ctrl.state.value}")

    print("\n✅ 侧翻临界保护单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-20 侧翻临界保护单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(lat=2.0, roll=2.0, risk=RollRiskLevel.LOW, speed=80.0):
            c = RolloverProtectionController()
            c.set_speed_query(lambda: speed)
            c.set_attitude_query(lambda: AttitudeVector(lateral_accel_ms2=lat, roll_deg=roll, roll_risk=risk))
            c.set_roll_risk_query(lambda: RollRiskReport(risk_level=risk, lateral_accel_ms2=lat, roll_angle_deg=roll))
            c.set_vehicle_params_query(lambda: VehicleDimensionParams())
            c.set_actuator_limits_query(lambda: ActuatorLimits())
            return c

        print("\n[TC-M20-01] 低风险不触发")
        try:
            c = setup_ctrl()
            c.run_protection_cycle()
            assert c.state == ProtectionLevel.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M20-02] 中风险一级预警")
        try:
            c = setup_ctrl(risk=RollRiskLevel.MEDIUM, lat=4.0, roll=4.0)
            c.run_protection_cycle()
            assert c.state == ProtectionLevel.LEVEL1_WARNING
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M20-03] 高风险二级临界")
        try:
            c = setup_ctrl(risk=RollRiskLevel.HIGH, lat=7.0, roll=7.0)
            c.run_protection_cycle()
            assert c.state == ProtectionLevel.LEVEL2_CRITICAL
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M20-04] 临界三级紧急")
        try:
            c = setup_ctrl(risk=RollRiskLevel.CRITICAL, lat=10.0, roll=12.0)
            c.run_protection_cycle()
            assert c.state == ProtectionLevel.LEVEL3_EMERGENCY
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M20-05] 风险解除恢复")
        try:
            c = setup_ctrl(risk=RollRiskLevel.HIGH)
            c.run_protection_cycle()
            assert c.state == ProtectionLevel.LEVEL2_CRITICAL
            c.set_roll_risk_query(lambda: RollRiskReport(risk_level=RollRiskLevel.LOW))
            c.set_attitude_query(lambda: AttitudeVector(lateral_accel_ms2=1.0, roll_deg=1.0, roll_risk=RollRiskLevel.LOW))
            c.run_protection_cycle()
            assert c.state == ProtectionLevel.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M20-06] 紧急熔断")
        try:
            c = setup_ctrl()
            c.emergency_shutdown()
            assert c.state == ProtectionLevel.SYSTEM_PAUSED
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