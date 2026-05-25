#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-21
模块名称: 颠簸路面姿态补偿单元
所属分区: 五、车身姿态稳定
核心职责: 在非铺装颠簸路面行驶时，基于 ad-mcc-18 提供的俯仰角、垂向加速度及世界模型
          提供的路面高度异常标注，主动调整悬架阻尼特性（若配备可调悬架），柔化车身晃动
          幅度与冲击感。同时向 ad-mcc-01 建议降低车速上限，并协调 ad-mcc-09 与 ad-mcc-13
          采用柔和操控参数，减少因路面激励引起的车身俯仰与垂向振动。不参与常规铺装路面的
          行驶决策，仅在检测到持续颠簸时激活。

依赖模块:
    ad-mcc-18(车身姿态实时监测单元),
    ad-44(独立世界模型库),
    ad-mcc-32(车辆尺寸参数管理单元),
    车速传感器(CAN总线)
被依赖模块:
    ad-mcc-01(小脑总控调度核心),
    ad-mcc-09(油门开度解算单元),
    ad-mcc-13(制动压力解算单元),
    悬架控制器

安全约束:
  S-01: 本模块仅建议限速与柔和参数，不直接控制油门、制动或转向执行器
  S-02: 紧急制动时，颠簸补偿不得干预制动力分配或制动压力建立
  S-03: 若车辆未配备可调悬架，不得输出悬架阻尼调整指令
  S-04: 颠簸补偿的车速上限建议不得低于 20 km/h，确保车辆仍具备基本通行能力
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math
from collections import deque


class BumpState(Enum):
    NORMAL_MONITOR = "normal_monitor"
    MILD_BUMP = "mild_bump"
    SEVERE_BUMP = "severe_bump"
    SYSTEM_PAUSED = "system_paused"


@dataclass
class AttitudeVector:
    pitch_deg: float = 0.0
    vertical_accel_ms2: float = 0.0
    longitudinal_accel_ms2: float = 0.0
    confidence: float = 1.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class RoadSurfaceAnnotation:
    road_type: str = "铺装"
    bump_index: float = 0.0
    pothole_depths: List[float] = field(default_factory=list)
    rut_depth: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class SuspensionParams:
    suspension_type: str = "被动悬架"
    damping_range: str = "无"
    max_travel_mm: float = 100.0
    response_time_ms: float = 50.0


@dataclass
class SuspensionDampingCommand:
    target_damping_mode: str = "标准模式"
    front_damping_ratio: float = 1.0
    rear_damping_ratio: float = 1.0
    reason: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class SpeedLimitSuggestion:
    suggested_speed_limit_kmh: float = 120.0
    reason: str = ""
    bump_level: int = 0
    timestamp: float = field(default_factory=time.time)


@dataclass
class SoftControlParams:
    suggested_accel_limit: float = 3.0
    suggested_decel_limit: float = 5.0
    suggested_jerk_limit: float = 5.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class BumpStatusReport:
    state: BumpState = BumpState.NORMAL_MONITOR
    pitch_rms: float = 0.0
    vertical_rms: float = 0.0
    damping_mode: str = "标准模式"
    suggested_speed: float = 120.0
    timestamp: float = field(default_factory=time.time)


CONTROL_PERIOD_S = 0.005
WINDOW_DURATION_S = 1.0
WINDOW_SIZE = int(WINDOW_DURATION_S / CONTROL_PERIOD_S)

MILD_BUMP_INDEX = 1.0
SEVERE_BUMP_INDEX = 2.0

PITCH_NORM_DEG = 2.0
VERTICAL_NORM_G = 0.3 * 9.81

MIN_SPEED_LIMIT_KMH = 20.0
SEVERE_SPEED_LIMIT_KMH = 25.0

NORMAL_ACCEL_LIMIT = 3.0
NORMAL_DECEL_LIMIT = 5.0
NORMAL_JERK_LIMIT = 5.0


class BumpCompensationController:
    def __init__(self):
        self.module_id = "ad-mcc-21"
        self.module_name = "颠簸路面姿态补偿单元"
        self.version = "V1.0"

        self.state = BumpState.NORMAL_MONITOR
        self._prev_state = BumpState.NORMAL_MONITOR
        self._pitch_window = deque(maxlen=WINDOW_SIZE)
        self._vertical_window = deque(maxlen=WINDOW_SIZE)
        self._suspension_params = SuspensionParams()
        self._has_adjustable_suspension = False
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_attitude = None
        self._query_road_surface = None
        self._query_speed = None
        self._query_suspension_params = None

        self._publish_suspension_command = None
        self._publish_speed_limit_suggestion = None
        self._publish_soft_control_params = None
        self._publish_status_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_attitude_query(self, callback):
        self._query_attitude = callback

    def set_road_surface_query(self, callback):
        self._query_road_surface = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_suspension_params_query(self, callback):
        self._query_suspension_params = callback

    def set_suspension_command_publisher(self, callback):
        self._publish_suspension_command = callback

    def set_speed_limit_suggestion_publisher(self, callback):
        self._publish_speed_limit_suggestion = callback

    def set_soft_control_params_publisher(self, callback):
        self._publish_soft_control_params = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_compensation_cycle(self):
        now = time.time()
        if self.state == BumpState.SYSTEM_PAUSED:
            return

        attitude = self._query_attitude() if self._query_attitude else None
        if attitude is None or attitude.confidence < 0.5:
            return

        speed = self._query_speed() if self._query_speed else 0.0
        road = self._query_road_surface() if self._query_road_surface else None

        if self._query_suspension_params:
            params = self._query_suspension_params()
            if params:
                self._suspension_params = params
                self._has_adjustable_suspension = "可调" in params.suspension_type or "CDC" in params.suspension_type.upper() or "空气" in params.suspension_type

        self._pitch_window.append(attitude.pitch_deg)
        self._vertical_window.append(attitude.vertical_accel_ms2)

        if len(self._pitch_window) < WINDOW_SIZE:
            return

        pitch_fluctuation = max(self._pitch_window) - min(self._pitch_window)
        vertical_values = list(self._vertical_window)
        vertical_rms = math.sqrt(sum(v * v for v in vertical_values) / len(vertical_values))

        bump_index = 0.5 * (pitch_fluctuation / PITCH_NORM_DEG) + 0.5 * (vertical_rms / VERTICAL_NORM_G)

        self._prev_state = self.state

        if bump_index >= SEVERE_BUMP_INDEX:
            self.state = BumpState.SEVERE_BUMP
        elif bump_index >= MILD_BUMP_INDEX:
            self.state = BumpState.MILD_BUMP
        else:
            self.state = BumpState.NORMAL_MONITOR

        if self.state == BumpState.SEVERE_BUMP:
            self._apply_severe_compensation(speed)
        elif self.state == BumpState.MILD_BUMP:
            self._apply_mild_compensation(speed)
        else:
            self._restore_normal()

        if self.state != self._prev_state:
            if self._publish_event_log:
                self._publish_event_log({
                    "event": "bump_compensation_state_change",
                    "from": self._prev_state.value,
                    "to": self.state.value,
                    "bump_index": round(bump_index, 2),
                    "timestamp": now,
                })

        if now - self._last_report_time >= 1.0:
            self._last_report_time = now
            if self._publish_status_report:
                damping_mode = "标准模式"
                suggested_speed = speed
                if self.state == BumpState.SEVERE_BUMP:
                    damping_mode = "越野模式"
                    suggested_speed = SEVERE_SPEED_LIMIT_KMH
                elif self.state == BumpState.MILD_BUMP:
                    damping_mode = "舒适模式"
                    suggested_speed = speed
                self._publish_status_report(BumpStatusReport(
                    state=self.state,
                    pitch_rms=round(pitch_fluctuation, 2),
                    vertical_rms=round(vertical_rms, 2),
                    damping_mode=damping_mode,
                    suggested_speed=suggested_speed,
                ))

    def _apply_severe_compensation(self, speed: float):
        if self._has_adjustable_suspension and self._publish_suspension_command:
            self._publish_suspension_command(SuspensionDampingCommand(
                target_damping_mode="越野模式",
                front_damping_ratio=0.5,
                rear_damping_ratio=0.7,
                reason="重度颠簸"
            ))
        if self._publish_speed_limit_suggestion:
            self._publish_speed_limit_suggestion(SpeedLimitSuggestion(
                suggested_speed_limit_kmh=SEVERE_SPEED_LIMIT_KMH,
                reason="重度颠簸路面",
                bump_level=2
            ))
        if self._publish_soft_control_params:
            self._publish_soft_control_params(SoftControlParams(
                suggested_accel_limit=NORMAL_ACCEL_LIMIT * 0.5,
                suggested_decel_limit=NORMAL_DECEL_LIMIT * 0.4,
                suggested_jerk_limit=NORMAL_JERK_LIMIT * 0.5
            ))

    def _apply_mild_compensation(self, speed: float):
        if self._has_adjustable_suspension and self._publish_suspension_command:
            self._publish_suspension_command(SuspensionDampingCommand(
                target_damping_mode="舒适模式",
                front_damping_ratio=0.7,
                rear_damping_ratio=0.7,
                reason="轻度颠簸"
            ))
        if self._publish_speed_limit_suggestion:
            self._publish_speed_limit_suggestion(SpeedLimitSuggestion(
                suggested_speed_limit_kmh=max(speed, MIN_SPEED_LIMIT_KMH),
                reason="轻度颠簸路面",
                bump_level=1
            ))
        if self._publish_soft_control_params:
            self._publish_soft_control_params(SoftControlParams(
                suggested_accel_limit=NORMAL_ACCEL_LIMIT * 0.8,
                suggested_decel_limit=NORMAL_DECEL_LIMIT * 0.7,
                suggested_jerk_limit=NORMAL_JERK_LIMIT * 0.8
            ))

    def _restore_normal(self):
        if self._prev_state != BumpState.NORMAL_MONITOR:
            if self._has_adjustable_suspension and self._publish_suspension_command:
                self._publish_suspension_command(SuspensionDampingCommand(
                    target_damping_mode="标准模式",
                    front_damping_ratio=1.0,
                    rear_damping_ratio=1.0,
                    reason="颠簸解除"
                ))
            if self._publish_speed_limit_suggestion:
                self._publish_speed_limit_suggestion(SpeedLimitSuggestion(
                    suggested_speed_limit_kmh=250.0,
                    reason="颠簸解除",
                    bump_level=0
                ))
            if self._publish_soft_control_params:
                self._publish_soft_control_params(SoftControlParams(
                    suggested_accel_limit=NORMAL_ACCEL_LIMIT,
                    suggested_decel_limit=NORMAL_DECEL_LIMIT,
                    suggested_jerk_limit=NORMAL_JERK_LIMIT
                ))

    def get_state(self) -> BumpState:
        return self.state

    def emergency_shutdown(self):
        self.state = BumpState.SYSTEM_PAUSED
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
    print("  AD-mcc-cerebellum 颠簸路面姿态补偿单元 (ad-mcc-21) 演示")
    print("=" * 70)

    ctrl = BumpCompensationController()
    ctrl.set_speed_query(lambda: 30.0)
    ctrl.set_attitude_query(lambda: AttitudeVector(pitch_deg=1.0, vertical_accel_ms2=1.0))
    ctrl.set_suspension_params_query(lambda: SuspensionParams(suspension_type="CDC可调悬架"))

    print_separator("STEP 1: 正常路面")
    for _ in range(200):
        ctrl.run_compensation_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 2: 轻度颠簸 (俯仰波动 3°, 垂向RMS 3.0)")
    ctrl.set_attitude_query(lambda: AttitudeVector(pitch_deg=3.0, vertical_accel_ms2=3.0))
    for _ in range(200):
        ctrl.run_compensation_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 3: 重度颠簸 (俯仰波动 6°, 垂向RMS 7.0)")
    ctrl.set_attitude_query(lambda: AttitudeVector(pitch_deg=6.0, vertical_accel_ms2=7.0))
    for _ in range(200):
        ctrl.run_compensation_cycle()
    print(f"  状态: {ctrl.state.value}")

    print("\n✅ 颠簸路面姿态补偿单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-21 颠簸路面姿态补偿单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(pitch=1.0, vert=1.0, speed=40.0, suspension_type="被动悬架"):
            c = BumpCompensationController()
            c.set_speed_query(lambda: speed)
            c.set_attitude_query(lambda: AttitudeVector(pitch_deg=pitch, vertical_accel_ms2=vert))
            c.set_suspension_params_query(lambda: SuspensionParams(suspension_type=suspension_type))
            return c

        print("\n[TC-M21-01] 正常路面不触发")
        try:
            c = setup_ctrl()
            for _ in range(200):
                c.run_compensation_cycle()
            assert c.state == BumpState.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M21-02] 轻度颠簸触发")
        try:
            c = setup_ctrl(pitch=3.5, vert=3.5)
            for _ in range(200):
                c.run_compensation_cycle()
            assert c.state == BumpState.MILD_BUMP
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M21-03] 重度颠簸触发")
        try:
            c = setup_ctrl(pitch=6.5, vert=7.0)
            for _ in range(200):
                c.run_compensation_cycle()
            assert c.state == BumpState.SEVERE_BUMP
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M21-04] 颠簸减弱恢复")
        try:
            c = setup_ctrl(pitch=6.5, vert=7.0)
            for _ in range(200):
                c.run_compensation_cycle()
            assert c.state == BumpState.SEVERE_BUMP
            c.set_attitude_query(lambda: AttitudeVector(pitch_deg=0.5, vertical_accel_ms2=0.5))
            for _ in range(200):
                c.run_compensation_cycle()
            assert c.state == BumpState.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M21-05] 无可调悬架不输出悬架指令")
        try:
            c = setup_ctrl(pitch=3.5, vert=3.5, suspension_type="被动悬架")
            suspension_cmd = None
            def trap_cmd(cmd):
                nonlocal suspension_cmd
                suspension_cmd = cmd
            c.set_suspension_command_publisher(trap_cmd)
            for _ in range(200):
                c.run_compensation_cycle()
            assert c.state == BumpState.MILD_BUMP
            assert suspension_cmd is None
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M21-06] 紧急熔断")
        try:
            c = setup_ctrl()
            c.emergency_shutdown()
            assert c.state == BumpState.SYSTEM_PAUSED
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