#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-13
模块名称: 制动压力解算单元
所属分区: 四、制动控制集群
核心职责: 将 ECC 大脑通过 ad-mcc-01 下发的目标减速度意图（如巡航减速、紧急制动）转化为
          具体的制动主缸压力指令（MPa）。基于车辆制动系统参数、当前路面摩擦系数、车辆负载
          与目标停车距离，运用制动动力学模型，精确计算每一帧所需的制动压力。同时考虑再生制动
          与摩擦制动的协调，输出制动压力目标值至 ad-mcc-14 进行响应加速控制。
          不参与任何场景判断与驾驶决策。

依赖模块:
    ad-mcc-01(小脑总控调度核心，下发制动调度指令),
    ad-mcc-34(动力与制动参数管理单元，提供制动主缸规格、制动力分配曲线等),
    ad-mcc-17(再生制动优先协调单元，提供再生制动力分配比例),
    ad-44(独立世界模型库，通过 MemoryBus 提供路面摩擦系数),
    ad-mcc-38(执行日志记录单元，记录偏差事件)
被依赖模块:
    ad-mcc-14(制动响应加速单元，接收制动目标压力序列),
    ad-mcc-02(运动生理边界闸门，校验输出压力指令)

安全约束:
  S-01: 紧急制动指令为最高优先级，收到后必须立即输出最大制动压力，不得延迟
  S-02: 制动主缸压力不得超出制动系统标定的最大允许压力，防止硬件损坏
  S-03: 在低附着系数路面，必须限制最大减速度以防车轮抱死失控
  S-04: 制动压力解算结果必须经过 ad-mcc-02 边界校验方可最终执行
  S-05: 本模块仅负责制动压力的计算，不直接控制制动执行器
  S-06: 再生制动比例异常时必须回退至纯摩擦制动，并记录事件
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


class CalculationState(Enum):
    IDLE = "idle"
    CALCULATING = "calculating"
    MAX_BRAKE = "max_brake"
    DEGRADED_ESTIMATION = "degraded_estimation"
    SYSTEM_PAUSED = "system_paused"


class BrakeType(Enum):
    GENTLE = "日常缓刹"
    EMERGENCY = "紧急制动"


class ExecutionMode(Enum):
    NORMAL = "正常"
    DEGRADED_LEVEL1 = "一级降级"
    DEGRADED_LEVEL2 = "二级降级"
    DEGRADED_LEVEL3 = "三级降级"
    UNPAVED = "非铺装道路"


@dataclass
class BrakeCommand:
    msg_id: str = ""
    brake_type: BrakeType = BrakeType.GENTLE
    target_decel_ms2: float = 0.0
    target_stop_distance_m: float = 0.0
    mode_mark: ExecutionMode = ExecutionMode.NORMAL
    execution_timeout_s: float = 5.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class VehicleBrakeParams:
    master_cylinder_max_pressure_mpa: float = 10.0
    front_rear_brake_ratio: float = 0.7
    brake_booster_gain: float = 5.0
    pad_friction_coefficient: float = 0.4
    piston_area_m2: float = 0.002
    number_of_pads: int = 4
    tire_rolling_radius_m: float = 0.35
    vehicle_mass_kg: float = 1800.0
    max_brake_decel_ms2: float = 5.0


@dataclass
class EmergencyBrakeOverride:
    msg_id: str = ""
    target_decel_ms2: float = 9.0
    max_pressure_exempt: bool = True


@dataclass
class BrakePressureSequence:
    timestamp: float = field(default_factory=time.time)
    target_pressure_mpa: float = 0.0
    brake_type: BrakeType = BrakeType.GENTLE
    friction_pressure_mpa: float = 0.0
    regen_torque_nm: float = 0.0
    confidence: float = 0.95
    limiting_factor: str = ""


@dataclass
class StatusReport:
    state: CalculationState = CalculationState.IDLE
    target_pressure: float = 0.0
    limiting_factor: str = ""
    brake_system_status: str = "正常"


DEFAULT_BRAKE_PARAMS = VehicleBrakeParams()
LOW_MU_THRESHOLD = 0.2
LOW_MU_MAX_DECEL = 2.0
UNPAVED_MAX_DPRESSURE = 0.5
CONTROL_PERIOD_S = 0.01
GRAVITY = 9.81
AIR_DENSITY = 1.225


class BrakePressureCalculator:
    def __init__(self):
        self.module_id = "ad-mcc-13"
        self.module_name = "制动压力解算单元"
        self.version = "V1.0"

        self.state = CalculationState.IDLE
        self._params = DEFAULT_BRAKE_PARAMS
        self._params_valid = True
        self._prev_pressure = 0.0
        self._current_speed_ms = 0.0
        self._current_friction = 0.8
        self._regen_ratio = 0.0
        self._vehicle_mass = DEFAULT_BRAKE_PARAMS.vehicle_mass_kg

        self._pending_logs: List[Dict[str, Any]] = []

        self._query_brake_command = None
        self._query_vehicle_brake_params = None
        self._query_vehicle_speed = None
        self._query_road_friction = None
        self._query_regen_ratio = None
        self._query_vehicle_mass = None
        self._query_emergency_brake = None

        self._publish_brake_sequence = None
        self._publish_status_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_brake_command_query(self, callback):
        self._query_brake_command = callback

    def set_vehicle_brake_params_query(self, callback):
        self._query_vehicle_brake_params = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_road_friction_query(self, callback):
        self._query_road_friction = callback

    def set_regen_ratio_query(self, callback):
        self._query_regen_ratio = callback

    def set_vehicle_mass_query(self, callback):
        self._query_vehicle_mass = callback

    def set_emergency_brake_query(self, callback):
        self._query_emergency_brake = callback

    def set_brake_sequence_publisher(self, callback):
        self._publish_brake_sequence = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_calculation_cycle(self) -> Optional[BrakePressureSequence]:
        emergency = self._query_emergency_brake() if self._query_emergency_brake else None
        if emergency:
            self.state = CalculationState.MAX_BRAKE
            max_pressure = self._get_max_pressure()
            self._prev_pressure = max_pressure
            seq = BrakePressureSequence(
                target_pressure_mpa=max_pressure,
                brake_type=BrakeType.EMERGENCY,
                friction_pressure_mpa=max_pressure,
                regen_torque_nm=0.0,
                confidence=1.0,
                limiting_factor="紧急制动豁免"
            )
            if self._publish_brake_sequence:
                self._publish_brake_sequence(seq)
            self._log_event("EMERGENCY_BRAKE", {"max_pressure": max_pressure})
            return seq

        if self.state == CalculationState.SYSTEM_PAUSED:
            return None

        self._update_vehicle_state()
        self._validate_params()

        cmd = self._query_brake_command() if self._query_brake_command else None
        if cmd is None:
            return None

        self.state = CalculationState.CALCULATING if self._params_valid else CalculationState.DEGRADED_ESTIMATION

        if self._query_regen_ratio:
            self._regen_ratio = self._query_regen_ratio()
        else:
            self._regen_ratio = 0.0

        target_decel = cmd.target_decel_ms2
        limiting_factor = ""

        if cmd.brake_type == BrakeType.GENTLE and cmd.target_stop_distance_m > 0:
            speed_ms = self._current_speed_ms
            if speed_ms > 0:
                distance_decel = (speed_ms ** 2) / (2 * cmd.target_stop_distance_m)
                if distance_decel < target_decel:
                    target_decel = distance_decel
                    limiting_factor = "停车距离限制"

        max_decel_by_mode = self._get_mode_max_decel(cmd.mode_mark)
        if target_decel > max_decel_by_mode:
            target_decel = max_decel_by_mode
            limiting_factor = self._append_factor(limiting_factor, "模式限制")

        if self._current_friction < LOW_MU_THRESHOLD and target_decel > LOW_MU_MAX_DECEL:
            target_decel = LOW_MU_MAX_DECEL
            limiting_factor = self._append_factor(limiting_factor, "低摩擦系数限制")
            self._log_event("LOW_MU_LIMIT", {"decel": target_decel, "friction": self._current_friction})

        resistance = self._calc_resistance_force()
        total_brake_force = self._vehicle_mass * target_decel - resistance
        if total_brake_force < 0:
            total_brake_force = 0.0

        regen_ratio = self._regen_ratio
        if regen_ratio < 0 or regen_ratio > 1.0:
            regen_ratio = 0.0
            limiting_factor = self._append_factor(limiting_factor, "再生系统异常")
            self._log_event("REGEN_FAULT", {"original_ratio": self._regen_ratio})

        friction_force = total_brake_force * (1 - regen_ratio)
        regen_torque = total_brake_force * regen_ratio * self._params.tire_rolling_radius_m

        pressure = 0.0
        if friction_force > 0:
            denom = (self._params.pad_friction_coefficient * self._params.piston_area_m2 *
                     self._params.number_of_pads * self._params.brake_booster_gain)
            if denom > 0:
                pressure = friction_force * self._params.tire_rolling_radius_m / denom
                max_p = self._get_max_pressure()
                if pressure > max_p:
                    pressure = max_p
                    limiting_factor = self._append_factor(limiting_factor, "压力上限截断")
                    self._log_event("PRESSURE_CLAMP", {"pressure": pressure})

        if cmd.mode_mark == ExecutionMode.UNPAVED:
            dp = pressure - self._prev_pressure
            if abs(dp) > UNPAVED_MAX_DPRESSURE:
                pressure = self._prev_pressure + math.copysign(UNPAVED_MAX_DPRESSURE, dp)
                limiting_factor = self._append_factor(limiting_factor, "非铺装柔化")

        self._prev_pressure = pressure
        confidence = 0.7 if self.state == CalculationState.DEGRADED_ESTIMATION else 0.95

        seq = BrakePressureSequence(
            target_pressure_mpa=round(pressure, 3),
            brake_type=cmd.brake_type,
            friction_pressure_mpa=round(pressure, 3),
            regen_torque_nm=round(regen_torque, 2),
            confidence=confidence,
            limiting_factor=limiting_factor
        )

        if self._publish_brake_sequence:
            self._publish_brake_sequence(seq)

        if self._publish_status_report:
            self._publish_status_report(StatusReport(
                state=self.state,
                target_pressure=pressure,
                limiting_factor=limiting_factor,
                brake_system_status="正常" if self._params_valid else "降级"
            ))

        self._log_event("CALC_COMPLETE", {
            "decel": target_decel,
            "friction_force": friction_force,
            "regen_force": total_brake_force * regen_ratio,
            "pressure": pressure,
            "limiting": limiting_factor
        })

        if self._params_valid:
            self.state = CalculationState.IDLE
        return seq

    def _update_vehicle_state(self):
        if self._query_vehicle_speed:
            speed_kmh = self._query_vehicle_speed()
            self._current_speed_ms = speed_kmh / 3.6
        if self._query_road_friction:
            self._current_friction = self._query_road_friction()
        if self._query_vehicle_mass:
            self._vehicle_mass = self._query_vehicle_mass()
        else:
            self._vehicle_mass = self._params.vehicle_mass_kg

    def _validate_params(self):
        if self._query_vehicle_brake_params:
            params = self._query_vehicle_brake_params()
            if params and params.master_cylinder_max_pressure_mpa > 0 and params.piston_area_m2 > 0:
                self._params = params
                self._params_valid = True
                if self.state == CalculationState.DEGRADED_ESTIMATION:
                    self.state = CalculationState.IDLE
            else:
                self._params = DEFAULT_BRAKE_PARAMS
                self._params_valid = False
                self.state = CalculationState.DEGRADED_ESTIMATION
                self._log_event("PARAMS_DEGRADED", {"reason": "制动参数无效"})
        else:
            self._params = DEFAULT_BRAKE_PARAMS
            self._params_valid = False
            self.state = CalculationState.DEGRADED_ESTIMATION

    def _calc_resistance_force(self) -> float:
        cr = 0.04 if self._current_friction < 0.4 else 0.015
        f_roll = self._vehicle_mass * GRAVITY * cr
        f_air = 0.5 * AIR_DENSITY * 0.3 * 2.2 * self._current_speed_ms ** 2
        return f_roll + f_air

    def _get_max_pressure(self) -> float:
        return self._params.master_cylinder_max_pressure_mpa

    def _get_mode_max_decel(self, mode: ExecutionMode) -> float:
        limits = {
            ExecutionMode.NORMAL: 5.0,
            ExecutionMode.DEGRADED_LEVEL1: 4.5,
            ExecutionMode.DEGRADED_LEVEL2: 4.0,
            ExecutionMode.DEGRADED_LEVEL3: 9.0,
            ExecutionMode.UNPAVED: 3.0,
        }
        return limits.get(mode, 5.0)

    def _append_factor(self, existing: str, new: str) -> str:
        return f"{existing} + {new}" if existing else new

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

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def get_state(self) -> CalculationState:
        return self.state

    def get_statistics(self) -> Dict[str, Any]:
        return {
            "state": self.state.value,
            "params_valid": self._params_valid,
            "regen_ratio": self._regen_ratio,
            "prev_pressure": self._prev_pressure,
        }

    def emergency_shutdown(self):
        self.state = CalculationState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 制动压力解算单元 (ad-mcc-13) 演示")
    print("=" * 70)

    calc = BrakePressureCalculator()
    calc.set_vehicle_speed_query(lambda: 60.0)
    calc.set_road_friction_query(lambda: 0.8)
    calc.set_regen_ratio_query(lambda: 0.3)
    calc.set_vehicle_mass_query(lambda: 1800.0)
    calc.set_vehicle_brake_params_query(lambda: VehicleBrakeParams())

    print_separator("STEP 1: 正常缓刹")
    calc.set_brake_command_query(lambda: BrakeCommand(
        msg_id="CMD-001",
        brake_type=BrakeType.GENTLE,
        target_decel_ms2=2.0,
        mode_mark=ExecutionMode.NORMAL
    ))
    seq = calc.run_calculation_cycle()
    if seq:
        print(f"  目标压力: {seq.target_pressure_mpa} MPa")
        print(f"  再生扭矩: {seq.regen_torque_nm} Nm")
        print(f"  限制因素: {seq.limiting_factor}")

    print_separator("STEP 2: 紧急制动")
    calc.set_emergency_brake_query(lambda: EmergencyBrakeOverride())
    seq2 = calc.run_calculation_cycle()
    if seq2:
        print(f"  目标压力: {seq2.target_pressure_mpa} MPa (最大)")
        print(f"  制动类型: {seq2.brake_type.value}")

    print_separator("STEP 3: 停车距离限制")
    calc.set_emergency_brake_query(None)
    calc.set_vehicle_speed_query(lambda: 72.0)
    calc.set_brake_command_query(lambda: BrakeCommand(
        msg_id="CMD-002",
        brake_type=BrakeType.GENTLE,
        target_decel_ms2=5.0,
        target_stop_distance_m=50.0,
        mode_mark=ExecutionMode.NORMAL
    ))
    seq3 = calc.run_calculation_cycle()
    if seq3:
        print(f"  目标压力: {seq3.target_pressure_mpa} MPa")
        print(f"  限制因素: {seq3.limiting_factor}")

    print("\n✅ 制动压力解算单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-13 制动压力解算单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_calc(speed=60.0, friction=0.8, regen=0.3, mass=1800.0, params=None):
            c = BrakePressureCalculator()
            c.set_vehicle_speed_query(lambda: speed)
            c.set_road_friction_query(lambda: friction)
            c.set_regen_ratio_query(lambda: regen)
            c.set_vehicle_mass_query(lambda: mass)
            if params:
                c.set_vehicle_brake_params_query(lambda: params)
            else:
                c.set_vehicle_brake_params_query(lambda: VehicleBrakeParams())
            return c

        print("\n[TC-M13-01] 正常缓刹")
        try:
            c = setup_calc()
            c.set_brake_command_query(lambda: BrakeCommand(
                msg_id="T01", brake_type=BrakeType.GENTLE,
                target_decel_ms2=2.0, mode_mark=ExecutionMode.NORMAL
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.target_pressure_mpa > 0.5
            assert seq.brake_type == BrakeType.GENTLE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M13-02] 紧急制动")
        try:
            c = setup_calc()
            c.set_emergency_brake_query(lambda: EmergencyBrakeOverride())
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.target_pressure_mpa == 10.0
            assert seq.brake_type == BrakeType.EMERGENCY
            assert c.state == CalculationState.MAX_BRAKE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M13-03] 参数缺失降级估算")
        try:
            c = setup_calc(params=VehicleBrakeParams(master_cylinder_max_pressure_mpa=0.0))
            c.set_brake_command_query(lambda: BrakeCommand(
                msg_id="T03", brake_type=BrakeType.GENTLE,
                target_decel_ms2=2.0, mode_mark=ExecutionMode.NORMAL
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert c.state == CalculationState.DEGRADED_ESTIMATION
            assert seq.confidence < 0.8
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M13-04] 低摩擦系数限制")
        try:
            c = setup_calc(friction=0.15)
            c.set_brake_command_query(lambda: BrakeCommand(
                msg_id="T04", brake_type=BrakeType.GENTLE,
                target_decel_ms2=3.0, mode_mark=ExecutionMode.NORMAL
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.target_pressure_mpa < 3.0
            assert "低摩擦" in seq.limiting_factor
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M13-05] 再生制动异常")
        try:
            c = setup_calc(regen=1.5)
            c.set_brake_command_query(lambda: BrakeCommand(
                msg_id="T05", brake_type=BrakeType.GENTLE,
                target_decel_ms2=2.0, mode_mark=ExecutionMode.NORMAL
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.regen_torque_nm == 0.0
            assert "再生" in seq.limiting_factor
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M13-06] 停车距离优先")
        try:
            c = setup_calc(speed=72.0)
            c.set_brake_command_query(lambda: BrakeCommand(
                msg_id="T06", brake_type=BrakeType.GENTLE,
                target_decel_ms2=5.0, target_stop_distance_m=50.0,
                mode_mark=ExecutionMode.NORMAL
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert "停车距离" in seq.limiting_factor
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