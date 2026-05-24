#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-19
模块名称: 横摆稳定控制单元
所属分区: 五、车身姿态稳定
核心职责: 基于 ad-mcc-18 提供的车身横摆角速度、侧向加速度及方向盘转角、车速等信号，
          实时判断车辆是否出现不足转向或过度转向趋势。在极限工况下主动请求 ESP 制动干预
          并向动力系统请求降低驱动扭矩，产生附加横摆力矩维持车身稳定。不参与常规工况下的
          行驶决策，仅在稳定性风险出现时激活，所有干预指令均通过 ad-mcc-02 边界校验。

依赖模块:
    ad-mcc-18(车身姿态实时监测单元),
    方向盘转角传感器/轮速传感器(CAN总线),
    ad-mcc-32(车辆尺寸参数管理单元),
    ad-mcc-34(动力与制动参数管理单元)
被依赖模块:
    ad-mcc-20(侧翻临界保护单元),
    ad-mcc-02(运动生理边界闸门),
    ESP 执行器,
    动力控制器

安全约束:
  S-01: ESP 制动干预指令不得超过制动系统当前最大允许压力，防止管路过载
  S-02: 当 ad-mcc-20 侧翻临界保护激活时，侧翻保护具有更高优先级，本模块需协调制动分配
  S-03: 驾驶员主动反向操作时，应尊重驾驶员意图，减少或中止 ESP 干预
  S-04: 本模块仅负责稳定性干预，不参与常规制动或驱动控制
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


class YawControlState(Enum):
    NORMAL_MONITOR = "normal_monitor"
    UNDERSTEER_CONTROL = "understeer_control"
    OVERSTEER_CONTROL = "oversteer_control"
    CRITICAL_STABILITY = "critical_stability"
    SYSTEM_PAUSED = "system_paused"


class InterventionMode(Enum):
    NONE = "none"
    UNDERSTEER = "understeer"
    OVERSTEER = "oversteer"
    CRITICAL = "critical"


@dataclass
class AttitudeVector:
    yaw_rate_deg_per_s: float = 0.0
    lateral_accel_ms2: float = 0.0
    roll_deg: float = 0.0
    confidence: float = 1.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class VehicleParams:
    wheelbase_m: float = 2.8
    track_width_m: float = 1.6
    steering_ratio: float = 16.0


@dataclass
class BrakeLimitParams:
    max_pressure_mpa: float = 10.0


@dataclass
class TorqueLimitParams:
    max_reduction_ratio: float = 0.7


@dataclass
class ESPCommand:
    intervention_mode: InterventionMode = InterventionMode.NONE
    target_wheels: List[str] = field(default_factory=list)
    wheel_pressure_mpa: Dict[str, float] = field(default_factory=dict)
    intervention_priority: int = 0
    timestamp: float = field(default_factory=time.time)


@dataclass
class TorqueReductionRequest:
    reduction_ratio: float = 0.0
    reason: str = ""
    urgency: str = "普通"
    timestamp: float = field(default_factory=time.time)


@dataclass
class StabilityStatus:
    state: YawControlState = YawControlState.NORMAL_MONITOR
    yaw_rate_deviation: float = 0.0
    intervention_level: int = 0
    wheel_pressures: Dict[str, float] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


# 控制阈值
YAW_DEV_WARN = 2.0
YAW_DEV_INTERVENE = 3.0
YAW_DEV_CRITICAL = 8.0
SIDE_SLIP_CRITICAL = 10.0
INTERVENE_HOLD_MS = 50.0

# PID 参数
KP = 0.8
KI = 0.1
KD = 0.2
MAX_INTEGRAL = 10.0

CONTROL_PERIOD_S = 0.005
REPORT_INTERVAL_S = 0.5


class YawStabilityController:
    def __init__(self):
        self.module_id = "ad-mcc-19"
        self.module_name = "横摆稳定控制单元"
        self.version = "V1.0"

        self.state = YawControlState.NORMAL_MONITOR
        self._vehicle_params = VehicleParams()
        self._brake_limit = BrakeLimitParams()
        self._torque_limit = TorqueLimitParams()
        self._prev_error = 0.0
        self._integral = 0.0
        self._intervention_timer = 0.0
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_attitude = None
        self._query_steering_angle = None
        self._query_speed = None
        self._query_wheel_speeds = None
        self._query_vehicle_params = None
        self._query_brake_limit = None
        self._query_torque_limit = None
        self._query_side_slip = None

        self._publish_esp_command = None
        self._publish_torque_request = None
        self._publish_status = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_attitude_query(self, callback):
        self._query_attitude = callback

    def set_steering_angle_query(self, callback):
        self._query_steering_angle = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_wheel_speeds_query(self, callback):
        self._query_wheel_speeds = callback

    def set_vehicle_params_query(self, callback):
        self._query_vehicle_params = callback

    def set_brake_limit_query(self, callback):
        self._query_brake_limit = callback

    def set_torque_limit_query(self, callback):
        self._query_torque_limit = callback

    def set_side_slip_query(self, callback):
        self._query_side_slip = callback

    def set_esp_command_publisher(self, callback):
        self._publish_esp_command = callback

    def set_torque_request_publisher(self, callback):
        self._publish_torque_request = callback

    def set_status_publisher(self, callback):
        self._publish_status = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_control_cycle(self):
        now = time.time()
        if self.state == YawControlState.SYSTEM_PAUSED:
            return

        attitude = self._query_attitude() if self._query_attitude else None
        if attitude is None or attitude.confidence < 0.5:
            return

        speed_kmh = self._query_speed() if self._query_speed else 0.0
        steering_angle = self._query_steering_angle() if self._query_steering_angle else 0.0
        side_slip = self._query_side_slip() if self._query_side_slip else 0.0

        if self._query_vehicle_params:
            params = self._query_vehicle_params()
            if params:
                self._vehicle_params = params
        if self._query_brake_limit:
            limit = self._query_brake_limit()
            if limit:
                self._brake_limit = limit
        if self._query_torque_limit:
            limit = self._query_torque_limit()
            if limit:
                self._torque_limit = limit

        speed_ms = speed_kmh / 3.6
        wheelbase = self._vehicle_params.wheelbase_m
        track = self._vehicle_params.track_width_m
        steering_ratio = self._vehicle_params.steering_ratio

        if speed_ms > 0.5:
            delta = math.radians(steering_angle / steering_ratio)
            r_des = (speed_ms / wheelbase) * delta if abs(delta) > 1e-6 else 0.0
        else:
            r_des = 0.0

        r_actual = attitude.yaw_rate_deg_per_s
        e_r = r_actual - r_des

        self._integral += e_r * CONTROL_PERIOD_S
        self._integral = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, self._integral))
        derivative = (e_r - self._prev_error) / CONTROL_PERIOD_S if CONTROL_PERIOD_S > 0 else 0.0
        self._prev_error = e_r

        delta_mz = KP * e_r + KI * self._integral + KD * derivative

        new_state = self.state
        intervention = InterventionMode.NONE
        reduction_ratio = 0.0
        target_wheels = []
        wheel_pressures = {}

        if abs(e_r) > YAW_DEV_CRITICAL or abs(side_slip) > SIDE_SLIP_CRITICAL:
            new_state = YawControlState.CRITICAL_STABILITY
            intervention = InterventionMode.CRITICAL
            reduction_ratio = 0.5
        elif e_r > YAW_DEV_INTERVENE:
            self._intervention_timer += CONTROL_PERIOD_S * 1000.0
            if self._intervention_timer >= INTERVENE_HOLD_MS:
                new_state = YawControlState.OVERSTEER_CONTROL
                intervention = InterventionMode.OVERSTEER
                reduction_ratio = 0.3
        elif e_r < -YAW_DEV_INTERVENE:
            self._intervention_timer += CONTROL_PERIOD_S * 1000.0
            if self._intervention_timer >= INTERVENE_HOLD_MS:
                new_state = YawControlState.UNDERSTEER_CONTROL
                intervention = InterventionMode.UNDERSTEER
                reduction_ratio = 0.2
        else:
            self._intervention_timer = 0.0
            if self.state != YawControlState.NORMAL_MONITOR:
                self._log_intervention_end()
            new_state = YawControlState.NORMAL_MONITOR

        self.state = new_state

        if intervention != InterventionMode.NONE and abs(delta_mz) > 0:
            base_pressure = min(abs(delta_mz) * 0.02, self._brake_limit.max_pressure_mpa)
            if intervention == InterventionMode.OVERSTEER:
                target_wheels = ["FR"]
                wheel_pressures = {"FR": base_pressure}
            elif intervention == InterventionMode.UNDERSTEER:
                target_wheels = ["RL"]
                wheel_pressures = {"RL": base_pressure}
            elif intervention == InterventionMode.CRITICAL:
                target_wheels = ["FL", "FR", "RL", "RR"]
                wheel_pressures = {w: base_pressure for w in target_wheels}

        if intervention != InterventionMode.NONE:
            if self._publish_esp_command:
                self._publish_esp_command(ESPCommand(
                    intervention_mode=intervention,
                    target_wheels=target_wheels,
                    wheel_pressure_mpa=wheel_pressures,
                    intervention_priority=2 if intervention == InterventionMode.CRITICAL else 1,
                ))
            if self._publish_torque_request:
                self._publish_torque_request(TorqueReductionRequest(
                    reduction_ratio=reduction_ratio,
                    reason=intervention.value,
                ))

        if now - self._last_report_time >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_status:
                self._publish_status(StabilityStatus(
                    state=self.state,
                    yaw_rate_deviation=e_r,
                    intervention_level=2 if intervention == InterventionMode.CRITICAL else (1 if intervention != InterventionMode.NONE else 0),
                    wheel_pressures=wheel_pressures,
                ))

    def _log_intervention_end(self):
        if self._publish_event_log:
            self._publish_event_log({
                "event": "intervention_end",
                "state": self.state.value,
                "timestamp": time.time()
            })

    def _log_event(self, event_type: str, details: Dict[str, Any]):
        entry = {
            "log_id": f"log-{uuid.uuid4().hex[:8]}",
            "event_type": event_type,
            "source_module": self.module_id,
            "details": details,
            "timestamp": time.time()
        }
        self._pending_logs.append(entry)
        if self._publish_event_log:
            self._publish_event_log(entry)

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def get_state(self) -> YawControlState:
        return self.state

    def emergency_shutdown(self):
        self.state = YawControlState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 横摆稳定控制单元 (ad-mcc-19) 演示")
    print("=" * 70)

    ctrl = YawStabilityController()
    ctrl.set_attitude_query(lambda: AttitudeVector(yaw_rate_deg_per_s=5.0, confidence=0.98))
    ctrl.set_speed_query(lambda: 80.0)
    ctrl.set_steering_angle_query(lambda: 45.0)
    ctrl.set_side_slip_query(lambda: 2.0)
    ctrl.set_vehicle_params_query(lambda: VehicleParams())
    ctrl.set_brake_limit_query(lambda: BrakeLimitParams())
    ctrl.set_torque_limit_query(lambda: TorqueLimitParams())

    print_separator("STEP 1: 正常监控")
    ctrl.run_control_cycle()
    print(f"  当前状态: {ctrl.state.value}")

    print_separator("STEP 2: 模拟过度转向")
    ctrl.set_attitude_query(lambda: AttitudeVector(yaw_rate_deg_per_s=10.0, confidence=0.98))
    for _ in range(20):
        ctrl.run_control_cycle()
    print(f"  当前状态: {ctrl.state.value}")

    print("\n✅ 横摆稳定控制单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-19 横摆稳定控制单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(yaw=0.0, speed=80.0, steer=0.0, slip=0.0, conf=0.98):
            c = YawStabilityController()
            c.set_attitude_query(lambda: AttitudeVector(yaw_rate_deg_per_s=yaw, confidence=conf))
            c.set_speed_query(lambda: speed)
            c.set_steering_angle_query(lambda: steer)
            c.set_side_slip_query(lambda: slip)
            c.set_vehicle_params_query(lambda: VehicleParams())
            c.set_brake_limit_query(lambda: BrakeLimitParams())
            c.set_torque_limit_query(lambda: TorqueLimitParams())
            return c

        print("\n[TC-M19-01] 正常监控")
        try:
            c = setup_ctrl(yaw=0.5, steer=5.0, speed=80.0)
            c.run_control_cycle()
            assert c.state == YawControlState.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M19-02] 过度转向干预")
        try:
            c = setup_ctrl(yaw=8.0, steer=30.0, speed=80.0)
            for _ in range(20):
                c.run_control_cycle()
            assert c.state == YawControlState.OVERSTEER_CONTROL
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M19-03] 不足转向干预")
        try:
            c = setup_ctrl(yaw=-5.0, steer=60.0, speed=60.0)
            for _ in range(20):
                c.run_control_cycle()
            assert c.state == YawControlState.UNDERSTEER_CONTROL
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M19-04] 临界稳定")
        try:
            c = setup_ctrl(yaw=10.0, steer=10.0, speed=100.0)
            c.run_control_cycle()
            assert c.state == YawControlState.CRITICAL_STABILITY
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M19-05] 低置信度不干预")
        try:
            c = setup_ctrl(yaw=10.0, steer=10.0, speed=100.0, conf=0.4)
            c.run_control_cycle()
            assert c.state == YawControlState.NORMAL_MONITOR
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M19-06] 紧急熔断")
        try:
            c = setup_ctrl()
            c.emergency_shutdown()
            assert c.state == YawControlState.SYSTEM_PAUSED
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