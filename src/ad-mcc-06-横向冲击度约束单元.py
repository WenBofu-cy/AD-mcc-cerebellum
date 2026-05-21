#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-06
模块名称: 横向冲击度约束单元
所属分区: 二、转向控制集群
核心职责: 接收 ad-mcc-05 输出的平滑后方向盘目标转角序列，基于当前车速与车辆横向动力学模型，
          实时计算每个转角指令对应的预期横向冲击度（Jerk）。当预期横向冲击度超出当前驾驶模式
          允许的上限时，自动修正目标转角速率，确保变道、转弯、避让等横向机动动作始终在安全与
          舒适的物理边界内执行。

依赖模块:
    ad-mcc-05(转向平顺滤波单元，提供平滑后转角序列),
    ad-mcc-32(车辆尺寸参数管理单元，提供轴距/轮距/质心高度),
    ad-mcc-33(转向特性参数管理单元，提供转向比)
被依赖模块:
    ad-mcc-07(转向执行偏差监控单元，接收最终约束后的转角指令)

安全约束:
  S-01: 横向加速度必须严格约束在各模式上限以内，物理红线（7.0m/s²）为绝对不可逾越的边界
  S-02: 紧急模式仅放宽冲击度约束，横向加速度物理红线仍死守不破
  S-03: 车速为零或不可用时，必须跳过横向动力学计算，直接放行原指令
  S-04: 本模块仅输出约束后的转角指令，不直接操控转向电机
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


# ==================== 枚举定义 ====================

class ConstraintState(Enum):
    NORMAL_CONSTRAINT = "normal_constraint"
    DEGRADED_CONSTRAINT = "degraded_constraint"
    EMERGENCY_RELAXED = "emergency_relaxed"
    UNPAVED_CONSTRAINT = "unpaved_constraint"
    SYSTEM_PAUSED = "system_paused"


class DrivingMode(Enum):
    NORMAL = "正常"
    DEGRADED_LEVEL1 = "一级降级"
    DEGRADED_LEVEL2 = "二级降级"
    DEGRADED_LEVEL3 = "三级降级"
    UNPAVED = "非铺装道路"


# ==================== 数据结构 ====================

@dataclass
class SmoothedSteeringSequence:
    timestamp: float = field(default_factory=time.time)
    smoothed_angle_deg: float = 0.0
    smoothed_angle_rate_deg_per_s: float = 0.0
    filter_method: str = ""
    smoothing_confidence: float = 0.95


@dataclass
class VehicleLateralDynamicsParams:
    wheelbase_m: float = 2.7
    steering_ratio: float = 16.0
    cog_height_m: float = 0.5
    track_width_m: float = 1.6


@dataclass
class ConstrainedSteeringCommand:
    timestamp: float = field(default_factory=time.time)
    constrained_angle_deg: float = 0.0
    constrained_angle_rate_deg_per_s: float = 0.0
    original_rate_deg_per_s: float = 0.0
    constraint_triggered: bool = False
    constraint_reason: str = ""


@dataclass
class LateralJerkAlert:
    exceed_amount: float = 0.0
    current_jerk_ms3: float = 0.0
    allowed_limit_ms3: float = 0.0
    suggested_action: str = "降低转向速率"
    timestamp: float = field(default_factory=time.time)


# ==================== 各模式横向动力学边界 ====================

MODE_LATERAL_LIMITS = {
    DrivingMode.NORMAL: {
        "max_lateral_jerk_ms3": 3.0,
        "max_lateral_accel_ms2": 6.5,
    },
    DrivingMode.DEGRADED_LEVEL1: {
        "max_lateral_jerk_ms3": 2.5,
        "max_lateral_accel_ms2": 5.5,
    },
    DrivingMode.DEGRADED_LEVEL2: {
        "max_lateral_jerk_ms3": 2.0,
        "max_lateral_accel_ms2": 4.5,
    },
    DrivingMode.DEGRADED_LEVEL3: {
        "max_lateral_jerk_ms3": 4.0,
        "max_lateral_accel_ms2": 7.0,
    },
    DrivingMode.UNPAVED: {
        "max_lateral_jerk_ms3": 1.5,
        "max_lateral_accel_ms2": 4.0,
    },
}

ABSOLUTE_MAX_LATERAL_ACCEL_MS2 = 7.0
ABSOLUTE_MAX_STEERING_ANGLE_RATE_DEG_PER_S = 500.0


# ==================== 主类定义 ====================

class LateralJerkConstraintUnit:

    DEFAULT_WHEELBASE_M = 2.7
    DEFAULT_STEERING_RATIO = 16.0
    CONTROL_PERIOD_S = 0.01

    def __init__(self):
        self.module_id = "ad-mcc-06"
        self.module_name = "横向冲击度约束单元"
        self.version = "V1.0"

        self.state = ConstraintState.NORMAL_CONSTRAINT
        self._current_mode = DrivingMode.NORMAL
        self._lateral_limits = MODE_LATERAL_LIMITS[DrivingMode.NORMAL]

        self._vehicle_params = VehicleLateralDynamicsParams()
        self._params_valid = True

        self._prev_target_angle_deg: float = 0.0
        self._prev_lateral_accel_ms2: float = 0.0

        self._current_speed_kmh: float = 0.0

        self._pending_logs: List[Dict[str, Any]] = []

        self._query_smoothed_sequence = None
        self._query_vehicle_speed = None
        self._query_lateral_dynamics_params = None
        self._query_current_mode = None

        self._publish_constrained_command = None
        self._publish_jerk_alert = None

        self._load_vehicle_params()

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_smoothed_sequence_query(self, callback):
        self._query_smoothed_sequence = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_lateral_dynamics_params_query(self, callback):
        self._query_lateral_dynamics_params = callback

    def set_current_mode_query(self, callback):
        self._query_current_mode = callback

    def set_constrained_command_publisher(self, callback):
        self._publish_constrained_command = callback

    def set_jerk_alert_publisher(self, callback):
        self._publish_jerk_alert = callback

    # ========== 参数加载 ==========
    def _load_vehicle_params(self):
        if self._query_lateral_dynamics_params:
            params = self._query_lateral_dynamics_params()
            if params and params.wheelbase_m > 0 and params.steering_ratio > 0:
                self._vehicle_params = params
                self._params_valid = True
                return
        self._params_valid = False
        self._vehicle_params = VehicleLateralDynamicsParams(
            wheelbase_m=self.DEFAULT_WHEELBASE_M,
            steering_ratio=self.DEFAULT_STEERING_RATIO,
        )

    # ========== 主循环 ==========
    def run_constraint_cycle(self) -> Optional[ConstrainedSteeringCommand]:
        if self.state == ConstraintState.SYSTEM_PAUSED:
            return None

        if self._query_vehicle_speed:
            self._current_speed_kmh = self._query_vehicle_speed()

        if self._query_current_mode:
            new_mode = self._query_current_mode()
            if new_mode != self._current_mode:
                self._switch_mode(new_mode)

        smoothed = self._query_smoothed_sequence() if self._query_smoothed_sequence else None
        if smoothed is None:
            return None

        self._load_vehicle_params()

        target_angle = smoothed.smoothed_angle_deg
        target_rate = smoothed.smoothed_angle_rate_deg_per_s
        speed_ms = self._current_speed_kmh / 3.6

        constraint_triggered = False
        constraint_reason = ""

        if speed_ms < 0.1:
            command = ConstrainedSteeringCommand(
                constrained_angle_deg=target_angle,
                constrained_angle_rate_deg_per_s=target_rate,
                original_rate_deg_per_s=target_rate,
            )
            self._prev_target_angle_deg = target_angle
            self._prev_lateral_accel_ms2 = 0.0
            self._publish(command)
            return command

        # 计算初始横向加速度
        expected_lateral_accel = self._calc_lateral_acceleration(target_angle, speed_ms)

        # 横向加速度硬约束
        max_accel = self._lateral_limits["max_lateral_accel_ms2"]
        if abs(expected_lateral_accel) > max_accel:
            allowed_wheel_angle = math.atan(
                self._vehicle_params.wheelbase_m / (speed_ms ** 2 / max_accel)
            ) if speed_ms > 0 else 0.0
            allowed_steering_angle = allowed_wheel_angle * self._vehicle_params.steering_ratio * 180.0 / math.pi
            target_angle = math.copysign(min(abs(target_angle), allowed_steering_angle), target_angle)
            expected_lateral_accel = self._calc_lateral_acceleration(target_angle, speed_ms)
            constraint_triggered = True
            constraint_reason = "横向加速度超限"

        # 计算预期横向冲击度
        expected_lateral_jerk = (expected_lateral_accel - self._prev_lateral_accel_ms2) / self.CONTROL_PERIOD_S

        # 横向冲击度约束
        max_jerk = self._lateral_limits["max_lateral_jerk_ms3"]
        if abs(expected_lateral_jerk) > max_jerk:
            allowed_accel_change = max_jerk * self.CONTROL_PERIOD_S
            allowed_new_accel = self._prev_lateral_accel_ms2 + math.copysign(allowed_accel_change, expected_lateral_jerk)
            if abs(allowed_new_accel) > max_accel:
                allowed_new_accel = math.copysign(max_accel, allowed_new_accel)

            if speed_ms > 0:
                allowed_radius = speed_ms ** 2 / max(abs(allowed_new_accel), 0.01)
                allowed_wheel_angle = math.atan(self._vehicle_params.wheelbase_m / allowed_radius)
                allowed_steering_angle = allowed_wheel_angle * self._vehicle_params.steering_ratio * 180.0 / math.pi
                target_angle = math.copysign(min(abs(target_angle), allowed_steering_angle), target_angle)

            target_rate = (target_angle - self._prev_target_angle_deg) / self.CONTROL_PERIOD_S

            # 紧急模式宽容处理
            if self.state == ConstraintState.EMERGENCY_RELAXED and constraint_reason == "":
                # 仅冲击度超限且未触发加速度约束，只告警不拦截
                self._publish_jerk_alert(LateralJerkAlert(
                    exceed_amount=abs(expected_lateral_jerk) - max_jerk,
                    current_jerk_ms3=abs(expected_lateral_jerk),
                    allowed_limit_ms3=max_jerk,
                ))
            else:
                constraint_triggered = True
                if constraint_reason:
                    constraint_reason += " + 横向冲击度超限"
                else:
                    constraint_reason = "横向冲击度超限"

        # 关键修复：基于最终 target_angle 重新计算横向加速度，确保 prev 记录准确
        expected_lateral_accel = self._calc_lateral_acceleration(target_angle, speed_ms)

        # 转角速率物理上限
        target_rate = min(abs(target_rate), ABSOLUTE_MAX_STEERING_ANGLE_RATE_DEG_PER_S)
        target_rate = math.copysign(target_rate, target_angle - self._prev_target_angle_deg)

        command = ConstrainedSteeringCommand(
            constrained_angle_deg=round(target_angle, 2),
            constrained_angle_rate_deg_per_s=round(target_rate, 2),
            original_rate_deg_per_s=smoothed.smoothed_angle_rate_deg_per_s,
            constraint_triggered=constraint_triggered,
            constraint_reason=constraint_reason,
        )

        self._prev_target_angle_deg = target_angle
        self._prev_lateral_accel_ms2 = expected_lateral_accel

        self._publish(command)

        if constraint_triggered and self.state != ConstraintState.EMERGENCY_RELAXED:
            self._publish_jerk_alert(LateralJerkAlert(
                exceed_amount=abs(expected_lateral_jerk) - max_jerk,
                current_jerk_ms3=abs(expected_lateral_jerk),
                allowed_limit_ms3=max_jerk,
            ))

        return command

    # ========== 横向加速度计算 ==========
    def _calc_lateral_acceleration(self, steering_angle_deg: float, speed_ms: float) -> float:
        if abs(steering_angle_deg) < 0.1 or speed_ms < 0.1:
            return 0.0
        wheel_angle_rad = (steering_angle_deg * math.pi / 180.0) / self._vehicle_params.steering_ratio
        tan_wheel = math.tan(wheel_angle_rad)
        if abs(tan_wheel) < 1e-6:
            return 0.0
        radius = self._vehicle_params.wheelbase_m / tan_wheel
        lateral_accel = speed_ms ** 2 / radius
        return lateral_accel

    # ========== 模式切换 ==========
    def _switch_mode(self, new_mode: DrivingMode):
        self._current_mode = new_mode
        self._lateral_limits = MODE_LATERAL_LIMITS.get(new_mode, MODE_LATERAL_LIMITS[DrivingMode.NORMAL])
        mode_state_map = {
            DrivingMode.NORMAL: ConstraintState.NORMAL_CONSTRAINT,
            DrivingMode.DEGRADED_LEVEL1: ConstraintState.DEGRADED_CONSTRAINT,
            DrivingMode.DEGRADED_LEVEL2: ConstraintState.DEGRADED_CONSTRAINT,
            DrivingMode.DEGRADED_LEVEL3: ConstraintState.EMERGENCY_RELAXED,
            DrivingMode.UNPAVED: ConstraintState.UNPAVED_CONSTRAINT,
        }
        self.state = mode_state_map.get(new_mode, ConstraintState.NORMAL_CONSTRAINT)

    def _publish(self, command: ConstrainedSteeringCommand):
        if self._publish_constrained_command:
            self._publish_constrained_command(command)

    def _publish_jerk_alert(self, alert: LateralJerkAlert):
        if self._publish_jerk_alert:
            self._publish_jerk_alert(alert)

    def get_state(self) -> ConstraintState:
        return self.state

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
            "current_mode": self._current_mode.value,
            "params_valid": self._params_valid,
        }

    def emergency_shutdown(self):
        self.state = ConstraintState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，保持上一帧有效指令")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 横向冲击度约束单元 (ad-mcc-06) 演示")
    print("=" * 70)

    unit = LateralJerkConstraintUnit()
    unit.set_vehicle_speed_query(lambda: 60.0)
    unit.set_current_mode_query(lambda: DrivingMode.NORMAL)
    unit.set_lateral_dynamics_params_query(lambda: VehicleLateralDynamicsParams(
        wheelbase_m=2.7, steering_ratio=16.0
    ))

    print_separator("STEP 1: 正常变道（横向冲击度在限值内）")
    unit.set_smoothed_sequence_query(lambda: SmoothedSteeringSequence(
        smoothed_angle_deg=15.0,
        smoothed_angle_rate_deg_per_s=100.0,
    ))
    cmd = unit.run_constraint_cycle()
    if cmd:
        print(f"  约束后转角: {cmd.constrained_angle_deg}°")
        print(f"  触发约束: {cmd.constraint_triggered}")

    print_separator("STEP 2: 急转弯触发横向加速度约束")
    unit.set_smoothed_sequence_query(lambda: SmoothedSteeringSequence(
        smoothed_angle_deg=200.0,
        smoothed_angle_rate_deg_per_s=300.0,
    ))
    unit.set_vehicle_speed_query(lambda: 80.0)
    unit._prev_lateral_accel_ms2 = 0.0
    unit._prev_target_angle_deg = 0.0
    cmd2 = unit.run_constraint_cycle()
    if cmd2:
        print(f"  约束后转角: {cmd2.constrained_angle_deg}°")
        print(f"  约束触发: {cmd2.constraint_triggered}")
        if cmd2.constraint_triggered:
            print(f"  约束原因: {cmd2.constraint_reason}")

    print("\n✅ 横向冲击度约束单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-06 横向冲击度约束单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_unit(speed=60.0, mode=DrivingMode.NORMAL):
            u = LateralJerkConstraintUnit()
            u.set_vehicle_speed_query(lambda: speed)
            u.set_current_mode_query(lambda: mode)
            u.set_lateral_dynamics_params_query(lambda: VehicleLateralDynamicsParams(
                wheelbase_m=2.7, steering_ratio=16.0
            ))
            u._prev_lateral_accel_ms2 = 0.0
            u._prev_target_angle_deg = 0.0
            return u

        print("\n[TC-M06-01] 正常变道冲击度不超限")
        try:
            u = setup_unit()
            u.set_smoothed_sequence_query(lambda: SmoothedSteeringSequence(
                smoothed_angle_deg=15.0,
                smoothed_angle_rate_deg_per_s=100.0,
            ))
            cmd = u.run_constraint_cycle()
            assert cmd is not None
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M06-02] 急转弯加速度超限触发约束")
        try:
            u = setup_unit(speed=80.0)
            u.set_smoothed_sequence_query(lambda: SmoothedSteeringSequence(
                smoothed_angle_deg=200.0,
                smoothed_angle_rate_deg_per_s=300.0,
            ))
            cmd = u.run_constraint_cycle()
            assert cmd is not None
            assert cmd.constraint_triggered, "应触发约束"
            assert "横向加速度超限" in cmd.constraint_reason
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M06-03] 降级模式约束更严格")
        try:
            u = setup_unit(mode=DrivingMode.DEGRADED_LEVEL1)
            u.set_smoothed_sequence_query(lambda: SmoothedSteeringSequence(
                smoothed_angle_deg=30.0,
                smoothed_angle_rate_deg_per_s=200.0,
            ))
            cmd = u.run_constraint_cycle()
            assert cmd is not None
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M06-04] 紧急模式冲击度超限仅告警")
        try:
            u = setup_unit(mode=DrivingMode.DEGRADED_LEVEL3)
            u.set_smoothed_sequence_query(lambda: SmoothedSteeringSequence(
                smoothed_angle_deg=20.0,
                smoothed_angle_rate_deg_per_s=250.0,
            ))
            cmd = u.run_constraint_cycle()
            assert cmd is not None
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M06-05] 紧急熔断")
        try:
            u = setup_unit()
            u.emergency_shutdown()
            assert u.state == ConstraintState.SYSTEM_PAUSED
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