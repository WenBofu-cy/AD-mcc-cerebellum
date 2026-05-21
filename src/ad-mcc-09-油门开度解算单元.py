#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-09
模块名称: 油门开度解算单元
所属分区: 三、动力控制集群
核心职责: 将 ECC 大脑通过 ad-mcc-01 下发的目标速度/加速度意图转化为具体的油门开度指令
          （0–100%）。基于车辆动力参数（最大功率、扭矩曲线、当前档位）、当前车速、路面
          坡度与摩擦系数，运用车辆纵向动力学模型，计算每一帧所需的油门踏板开度与期望加速度
          曲线。是连接 ECC 决策层与车辆底层驱动执行层之间的核心翻译模块。不参与任何场景判断
          与驾驶决策。

依赖模块:
    ad-mcc-01(小脑总控调度核心，下发动力调度指令),
    ad-mcc-34(动力与制动参数管理单元，提供最大功率/扭矩/加速特性曲线),
    ad-mcc-35(能源参数管理单元，提供电池/燃油当前状态),
    ad-mcc-02(运动生理边界闸门，校验输出指令)
被依赖模块:
    ad-mcc-10(纵向冲击度约束单元，接收原始油门开度序列进行冲击度约束)

安全约束:
  S-01: 紧急制动或碰撞后响应指令为最高优先级，收到后油门必须立即强制归零（FORCED_IDLE）
  S-02: 解算出的油门开度必须严格约束在当前驾驶模式允许的最大开度以内
  S-03: 车辆动力参数缺失时，必须使用保守默认值并明确标记“降级估算”
  S-04: 本模块仅输出目标油门开度序列，不直接操控驱动电机或节气门
  S-05: 下坡或减速场景下，油门开度必须归零，不得通过油门补偿来对抗制动系统
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


# ==================== 枚举定义 ====================

class CalculationState(Enum):
    """油门开度解算单元内部状态"""
    IDLE = "idle"
    CALCULATING = "calculating"
    DEGRADED_ESTIMATION = "degraded_estimation"
    FORCED_IDLE = "forced_idle"
    SYSTEM_PAUSED = "system_paused"


class ExecutionMode(Enum):
    """执行模式（与 ad-mcc-01 对齐）"""
    NORMAL = "正常"
    DEGRADED_LEVEL1 = "一级降级"
    DEGRADED_LEVEL2 = "二级降级"
    DEGRADED_LEVEL3 = "三级降级"
    UNPAVED = "非铺装道路"


# ==================== 数据结构 ====================

@dataclass
class ThrottleDispatchCommand:
    """动力调度指令（来自 ad-mcc-01）"""
    msg_id: str = ""
    command_type: str = "巡航"
    target_speed_kmh: float = 0.0
    target_acceleration_ms2: float = 0.0
    max_jerk_ms3: float = 3.0
    mode_mark: ExecutionMode = ExecutionMode.NORMAL
    execution_timeout_s: float = 5.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class VehiclePowerParams:
    """车辆动力参数（来自 ad-mcc-34）"""
    max_power_kw: float = 150.0
    max_torque_nm: float = 300.0
    acceleration_curve: Dict[str, float] = field(default_factory=dict)
    drive_type: str = "AWD"
    transmission_ratio: float = 1.0
    vehicle_mass_kg: float = 1800.0
    drag_coefficient: float = 0.28
    frontal_area_m2: float = 2.2
    transmission_efficiency: float = 0.92


@dataclass
class EnergyStatusParams:
    """能源状态参数（来自 ad-mcc-35）"""
    battery_soc_pct: float = 80.0
    fuel_level_pct: float = 0.0
    max_output_power_kw: float = 150.0
    regen_brake_level: str = "标准"


@dataclass
class EmergencyIdleCommand:
    """紧急制动/碰撞后响应指令"""
    msg_id: str = ""
    intent_type: str = "紧急制动"
    force_idle: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class ThrottleTargetSequence:
    """油门目标开度序列（发送至 ad-mcc-10）"""
    timestamp: float = field(default_factory=time.time)
    target_throttle_pct: float = 0.0
    expected_acceleration_ms2: float = 0.0
    calculation_method: str = "纵向动力学模型"
    confidence: float = 0.95


@dataclass
class CalculationStatusReport:
    """解算状态上报（发送至 ad-mcc-01）"""
    current_state: CalculationState = CalculationState.IDLE
    calculation_duration_ms: float = 0.0
    throttle_validity: str = "有效"
    current_limit_factor: str = ""


# ==================== 各驾驶模式油门限制 ====================

MODE_THROTTLE_LIMITS = {
    ExecutionMode.NORMAL: {"max_throttle_pct": 100.0, "response": "线性", "max_accel_ms2": 3.0},
    ExecutionMode.DEGRADED_LEVEL1: {"max_throttle_pct": 80.0, "response": "线性", "max_accel_ms2": 2.5},
    ExecutionMode.DEGRADED_LEVEL2: {"max_throttle_pct": 60.0, "response": "线性", "max_accel_ms2": 2.0},
    ExecutionMode.DEGRADED_LEVEL3: {"max_throttle_pct": 0.0, "response": "强制怠速", "max_accel_ms2": 0.0},
    ExecutionMode.UNPAVED: {"max_throttle_pct": 70.0, "response": "渐进", "max_accel_ms2": 1.8},
}

# 非铺装模式油门变化量上限（每帧百分比）
UNPAVED_MAX_THROTTLE_CHANGE_PCT = 5.0

# 降级估算默认值
DEFAULT_VEHICLE_MASS_KG = 1800.0
DEFAULT_MAX_POWER_KW = 100.0
DEFAULT_DRAG_COEFFICIENT = 0.30
DEFAULT_FRONTAL_AREA_M2 = 2.2
DEFAULT_TRANSMISSION_EFFICIENCY = 0.90

# 控制周期（秒）
CONTROL_PERIOD_S = 0.01  # 100Hz

# 物理常量
GRAVITY_MS2 = 9.81
AIR_DENSITY_KG_M3 = 1.225


# ==================== 主类定义 ====================

class ThrottleAngleCalculator:
    """
    油门开度解算单元
    
    职责:
    1. 将目标速度/加速度意图转化为油门开度指令（0–100%）
    2. 基于车辆纵向动力学模型计算需求功率与油门映射
    3. 根据驾驶模式限制油门开度上限
    4. 紧急制动时强制怠速
    5. 参数缺失时使用保守默认值降级估算
    """

    def __init__(self):
        self.module_id = "ad-mcc-09"
        self.module_name = "油门开度解算单元"
        self.version = "V1.0"

        self.state = CalculationState.IDLE

        # 车辆参数
        self._vehicle_params = VehiclePowerParams()
        self._params_valid = True

        # 当前状态
        self._current_speed_kmh: float = 0.0
        self._current_throttle_pct: float = 0.0
        self._current_grade_rad: float = 0.0
        self._current_friction: float = 0.8

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调
        self._query_throttle_command = None           # Callable[[], Optional[ThrottleDispatchCommand]]
        self._query_vehicle_power_params = None       # Callable[[], VehiclePowerParams]
        self._query_energy_status = None              # Callable[[], EnergyStatusParams]
        self._query_vehicle_speed = None              # Callable[[], float]
        self._query_current_throttle = None           # Callable[[], float]
        self._query_road_grade = None                 # Callable[[], float]
        self._query_road_friction = None              # Callable[[], float]
        self._query_emergency_idle = None             # Callable[[], Optional[EmergencyIdleCommand]]

        # 输出回调
        self._publish_throttle_sequence = None        # Callable[[ThrottleTargetSequence], None]
        self._publish_status_report = None            # Callable[[CalculationStatusReport], None]

        # 加载参数
        self._load_vehicle_params()

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_throttle_command_query(self, callback):
        self._query_throttle_command = callback

    def set_vehicle_power_params_query(self, callback):
        self._query_vehicle_power_params = callback

    def set_energy_status_query(self, callback):
        self._query_energy_status = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_current_throttle_query(self, callback):
        self._query_current_throttle = callback

    def set_road_grade_query(self, callback):
        self._query_road_grade = callback

    def set_road_friction_query(self, callback):
        self._query_road_friction = callback

    def set_emergency_idle_query(self, callback):
        self._query_emergency_idle = callback

    def set_throttle_sequence_publisher(self, callback):
        self._publish_throttle_sequence = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    # ========== 参数加载 ==========
    def _load_vehicle_params(self):
        """加载车辆动力参数"""
        if self._query_vehicle_power_params:
            params = self._query_vehicle_power_params()
            if params and params.vehicle_mass_kg > 0 and params.max_power_kw > 0:
                self._vehicle_params = params
                self._params_valid = True
                if self.state == CalculationState.DEGRADED_ESTIMATION:
                    self.state = CalculationState.IDLE
                return

        # 参数不可用，使用保守默认值
        self._params_valid = False
        self.state = CalculationState.DEGRADED_ESTIMATION
        self._vehicle_params = VehiclePowerParams(
            vehicle_mass_kg=DEFAULT_VEHICLE_MASS_KG,
            max_power_kw=DEFAULT_MAX_POWER_KW,
            drag_coefficient=DEFAULT_DRAG_COEFFICIENT,
            frontal_area_m2=DEFAULT_FRONTAL_AREA_M2,
            transmission_efficiency=DEFAULT_TRANSMISSION_EFFICIENCY,
        )
        self._log_event("PARAMS_DEGRADED", {"reason": "动力参数无效，使用默认值"})

    # ========== 主循环 ==========
    def run_calculation_cycle(self) -> Optional[ThrottleTargetSequence]:
        """
        执行一次油门解算周期（100Hz）
        
        Returns:
            油门目标开度序列，若无新指令则返回 None
        """
        # 最高优先级：紧急制动强制怠速
        emergency = self._query_emergency_idle() if self._query_emergency_idle else None
        if emergency and emergency.force_idle:
            self.state = CalculationState.FORCED_IDLE
            sequence = ThrottleTargetSequence(
                target_throttle_pct=0.0,
                expected_acceleration_ms2=0.0,
                calculation_method="紧急制动强制怠速",
                confidence=1.0
            )
            if self._publish_throttle_sequence:
                self._publish_throttle_sequence(sequence)
            return sequence

        if self.state == CalculationState.SYSTEM_PAUSED:
            return None

        # 更新车辆状态
        if self._query_vehicle_speed:
            self._current_speed_kmh = self._query_vehicle_speed()
        if self._query_current_throttle:
            self._current_throttle_pct = self._query_current_throttle()
        if self._query_road_grade:
            grade_pct = self._query_road_grade()
            self._current_grade_rad = math.atan(grade_pct / 100.0)
        if self._query_road_friction:
            friction = self._query_road_friction()
            self._current_friction = friction if friction > 0 else 0.8

        # 接收动力调度指令
        command = self._query_throttle_command() if self._query_throttle_command else None
        if command is None:
            return None

        # 参数校验
        self._load_vehicle_params()

        start_time = time.perf_counter()
        self.state = CalculationState.CALCULATING if self._params_valid else CalculationState.DEGRADED_ESTIMATION

        target_speed = command.target_speed_kmh
        target_accel = command.target_acceleration_ms2
        mode = command.mode_mark

        speed_ms = self._current_speed_kmh / 3.6

        # 获取当前模式油门限制
        mode_limits = MODE_THROTTLE_LIMITS.get(mode, MODE_THROTTLE_LIMITS[ExecutionMode.NORMAL])
        max_throttle_pct = mode_limits["max_throttle_pct"]
        max_accel_ms2 = mode_limits["max_accel_ms2"]

        # 目标加速度约束
        if target_accel > max_accel_ms2:
            target_accel = max_accel_ms2
            self._log_event("ACCEL_CLAMPED", {"original": command.target_acceleration_ms2, "clamped": target_accel})

        # 计算行驶阻力
        resistance_force = self._calc_resistance_force(speed_ms)

        # 计算需求牵引力
        required_tractive_force = resistance_force + self._vehicle_params.vehicle_mass_kg * target_accel

        # 计算需求功率 (kW)
        required_power_kw = required_tractive_force * speed_ms / 1000.0 / self._vehicle_params.transmission_efficiency

        # 功率限制
        energy_status = self._query_energy_status() if self._query_energy_status else EnergyStatusParams()
        max_available_power = min(self._vehicle_params.max_power_kw, energy_status.max_output_power_kw)

        # 油门开度映射
        if required_power_kw <= 0:
            target_throttle = 0.0
        else:
            power_ratio = min(required_power_kw / max_available_power, 1.0) if max_available_power > 0 else 0.0
            target_throttle = power_ratio * max_throttle_pct

        # 非铺装模式油门响应柔化
        if mode == ExecutionMode.UNPAVED:
            throttle_change = target_throttle - self._current_throttle_pct
            max_change = UNPAVED_MAX_THROTTLE_CHANGE_PCT
            if abs(throttle_change) > max_change:
                target_throttle = self._current_throttle_pct + math.copysign(max_change, throttle_change)

        # 计算期望加速度
        net_force = required_tractive_force - resistance_force
        expected_accel = net_force / self._vehicle_params.vehicle_mass_kg

        # 置信度
        confidence = 0.7 if self.state == CalculationState.DEGRADED_ESTIMATION else 0.95

        # 生成油门目标开度序列
        sequence = ThrottleTargetSequence(
            timestamp=time.time(),
            target_throttle_pct=round(target_throttle, 2),
            expected_acceleration_ms2=round(expected_accel, 3),
            calculation_method="纵向动力学模型",
            confidence=confidence
        )

        # 输出
        if self._publish_throttle_sequence:
            self._publish_throttle_sequence(sequence)

        # 状态上报
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        if self._publish_status_report:
            self._publish_status_report(CalculationStatusReport(
                current_state=self.state,
                calculation_duration_ms=round(elapsed_ms, 2),
                throttle_validity="有效",
                current_limit_factor=f"模式={mode.value}, 功率上限={max_available_power:.1f}kW"
            ))

        if self._params_valid and self.state != CalculationState.FORCED_IDLE:
            self.state = CalculationState.IDLE

        return sequence

    # ========== 行驶阻力计算 ==========
    def _calc_resistance_force(self, speed_ms: float) -> float:
        """
        计算行驶阻力
        F_total = F_roll + F_air + F_grade
        """
        mass = self._vehicle_params.vehicle_mass_kg
        grade = self._current_grade_rad
        friction = self._current_friction

        # 滚动阻力系数（低摩擦路面取高值）
        rolling_resistance_coeff = 0.04 if friction < 0.4 else 0.015

        # 滚动阻力
        F_roll = mass * GRAVITY_MS2 * rolling_resistance_coeff * math.cos(grade)

        # 空气阻力
        F_air = 0.5 * AIR_DENSITY_KG_M3 * self._vehicle_params.drag_coefficient * \
                self._vehicle_params.frontal_area_m2 * speed_ms * speed_ms

        # 坡度阻力
        F_grade = mass * GRAVITY_MS2 * math.sin(grade)

        return F_roll + F_air + F_grade

    # ========== 查询接口 ==========
    def get_state(self) -> CalculationState:
        return self.state

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
            "params_valid": self._params_valid,
        }

    def emergency_shutdown(self):
        self.state = CalculationState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，维持当前油门开度")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 油门开度解算单元 (ad-mcc-09) 演示")
    print("=" * 70)

    calc = ThrottleAngleCalculator()
    calc.set_vehicle_speed_query(lambda: 60.0)
    calc.set_current_throttle_query(lambda: 25.0)
    calc.set_road_grade_query(lambda: 0.0)
    calc.set_road_friction_query(lambda: 0.8)
    calc.set_vehicle_power_params_query(lambda: VehiclePowerParams(
        vehicle_mass_kg=1800.0, max_power_kw=150.0
    ))
    calc.set_energy_status_query(lambda: EnergyStatusParams(max_output_power_kw=150.0))

    print_separator("STEP 1: 正常巡航指令")
    calc.set_throttle_command_query(lambda: ThrottleDispatchCommand(
        msg_id="CMD-001",
        command_type="巡航",
        target_speed_kmh=100.0,
        target_acceleration_ms2=0.5,
        mode_mark=ExecutionMode.NORMAL,
    ))
    seq = calc.run_calculation_cycle()
    if seq:
        print(f"  目标油门开度: {seq.target_throttle_pct}%")
        print(f"  期望加速度: {seq.expected_acceleration_ms2} m/s²")
        print(f"  解算方法: {seq.calculation_method}")
        print(f"  置信度: {seq.confidence}")

    print_separator("STEP 2: 紧急制动强制怠速")
    calc.set_emergency_idle_query(lambda: EmergencyIdleCommand(
        msg_id="EMG-001",
        force_idle=True,
    ))
    seq2 = calc.run_calculation_cycle()
    if seq2:
        print(f"  目标油门开度: {seq2.target_throttle_pct}%")
        print(f"  解算方法: {seq2.calculation_method}")

    print("\n✅ 油门开度解算单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-09 油门开度解算单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_calc(speed=60.0, throttle=25.0, grade=0.0, friction=0.8):
            c = ThrottleAngleCalculator()
            c.set_vehicle_speed_query(lambda: speed)
            c.set_current_throttle_query(lambda: throttle)
            c.set_road_grade_query(lambda: grade)
            c.set_road_friction_query(lambda: friction)
            c.set_vehicle_power_params_query(lambda: VehiclePowerParams(
                vehicle_mass_kg=1800.0, max_power_kw=150.0
            ))
            c.set_energy_status_query(lambda: EnergyStatusParams(max_output_power_kw=150.0))
            return c

        # TC-M09-01: 正常巡航指令
        print("\n[TC-M09-01] 正常巡航指令解算")
        try:
            c = setup_calc()
            c.set_throttle_command_query(lambda: ThrottleDispatchCommand(
                msg_id="T01", command_type="巡航",
                target_speed_kmh=100.0, target_acceleration_ms2=0.5,
                mode_mark=ExecutionMode.NORMAL
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert 10.0 <= seq.target_throttle_pct <= 50.0
            assert seq.confidence >= 0.9
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M09-02: 急加速指令
        print("\n[TC-M09-02] 急加速指令")
        try:
            c = setup_calc(speed=30.0)
            c.set_throttle_command_query(lambda: ThrottleDispatchCommand(
                msg_id="T02", target_acceleration_ms2=2.5,
                mode_mark=ExecutionMode.NORMAL
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.target_throttle_pct >= 30.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M09-03: 降级模式油门限制
        print("\n[TC-M09-03] 一级降级模式下油门开度不超过80%")
        try:
            c = setup_calc()
            c.set_throttle_command_query(lambda: ThrottleDispatchCommand(
                msg_id="T03", target_acceleration_ms2=3.0,
                mode_mark=ExecutionMode.DEGRADED_LEVEL1
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.target_throttle_pct <= 80.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M09-04: 下坡路段油门归零
        print("\n[TC-M09-04] 下坡路段油门归零")
        try:
            c = setup_calc(speed=65.0, grade=-5.0)
            c.set_throttle_command_query(lambda: ThrottleDispatchCommand(
                msg_id="T04", target_speed_kmh=50.0,
                mode_mark=ExecutionMode.NORMAL
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.target_throttle_pct == 0.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M09-05: 参数缺失降级估算
        print("\n[TC-M09-05] 动力参数缺失降级估算")
        try:
            c = ThrottleAngleCalculator()
            c.set_vehicle_speed_query(lambda: 60.0)
            c.set_current_throttle_query(lambda: 25.0)
            c.set_road_grade_query(lambda: 0.0)
            c.set_road_friction_query(lambda: 0.8)
            # 返回无效参数
            c.set_vehicle_power_params_query(lambda: VehiclePowerParams(
                vehicle_mass_kg=0.0, max_power_kw=0.0
            ))
            c.set_energy_status_query(lambda: EnergyStatusParams(max_output_power_kw=150.0))
            c.set_throttle_command_query(lambda: ThrottleDispatchCommand(
                msg_id="T05", target_acceleration_ms2=0.5,
                mode_mark=ExecutionMode.NORMAL
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

        # TC-M09-06: 紧急制动强制怠速
        print("\n[TC-M09-06] 紧急制动强制怠速")
        try:
            c = setup_calc()
            c.set_emergency_idle_query(lambda: EmergencyIdleCommand(
                msg_id="EMG", force_idle=True
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.target_throttle_pct == 0.0
            assert c.state == CalculationState.FORCED_IDLE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M09-07: 紧急熔断
        print("\n[TC-M09-07] 紧急熔断")
        try:
            c = setup_calc()
            c.emergency_shutdown()
            assert c.state == CalculationState.SYSTEM_PAUSED
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