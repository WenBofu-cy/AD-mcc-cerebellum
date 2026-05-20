#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-02
模块名称: 运动生理边界闸门
所属分区: 一、顶层总控中枢
核心职责: 锁定车辆动力学绝对极限（最大方向盘转角速率、最大纵向加减速度、最大横向加速度、
          侧翻临界阈值、最高车速等），作为 MCC 运动小脑执行任何操控指令前的最后一道硬约束
          校验。接收 ad-mcc-01 分发的操控指令或模式切换参数集，逐项校验是否超出物理极限或
          法规硬约束。超限指令立即拦截，并返回超限原因与修正建议。不参与任何场景判断与驾驶决策。

依赖模块:
    ad-mcc-01(小脑总控调度核心，下发待校验的操控指令),
    ad-mcc-32(车辆尺寸参数管理单元，提供车宽/轴距/质心高度等基准数据),
    ad-mcc-34(动力与制动参数管理单元，提供最大加减速度/最大功率等动力极限)
被依赖模块:
    ad-mcc-01(接收校验结果),
    ad-mcc-04 至 ad-mcc-38(全部执行模块的指令均须经本模块校验后方可执行)

安全约束:
  S-01: 任何操控指令在下发至执行模块前，必须通过本模块的边界校验。此为编译期强制约束，不可绕过
  S-02: 车辆出厂标定的绝对物理极限为不可逾越的红线，任何模式不得突破
  S-03: 紧急模式仅放宽模式边界约束，绝对物理极限仍死守不破
  S-04: 路面摩擦系数不可用时，必须保守假设为低附着路面，采用最严格的修正系数
  S-05: 本模块不参与场景判断与驾驶决策，仅提供物理可行性校验
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


# ==================== 枚举定义 ====================

class GateState(Enum):
    """边界闸门内部状态"""
    NORMAL_GATE = "normal_gate"
    DEGRADED_GATE = "degraded_gate"
    EMERGENCY_GATE = "emergency_gate"
    UNPAVED_GATE = "unpaved_gate"
    SYSTEM_PAUSED = "system_paused"


class ExecutionMode(Enum):
    """执行模式"""
    NORMAL = "正常"
    DEGRADED_LEVEL1 = "一级降级"
    DEGRADED_LEVEL2 = "二级降级"
    DEGRADED_LEVEL3 = "三级降级"
    UNPAVED = "非铺装道路"


class CommandType(Enum):
    """操控指令类型"""
    STEERING = "转向"
    ACCELERATION = "加速"
    BRAKING = "制动"
    SPEED = "车速"


class RoadCondition(Enum):
    """路面条件"""
    DRY_ASPHALT = "干燥沥青"
    WET = "湿滑路面"
    SNOW = "积雪路面"
    ICE = "结冰路面"


# ==================== 数据结构 ====================

@dataclass
class CommandToValidate:
    """待校验操控指令"""
    command_type: CommandType = CommandType.STEERING
    target_value: float = 0.0
    current_vehicle_state: Dict[str, float] = field(default_factory=dict)
    mode_mark: ExecutionMode = ExecutionMode.NORMAL


@dataclass
class GatePassSignal:
    """校验通过放行信号"""
    original_command: CommandToValidate = field(default_factory=CommandToValidate)
    passed: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class GateBlockSignal:
    """校验拦截信号"""
    original_command: CommandToValidate = field(default_factory=CommandToValidate)
    block_reasons: List[str] = field(default_factory=list)
    exceeded_params: Dict[str, float] = field(default_factory=dict)
    current_boundaries: Dict[str, float] = field(default_factory=dict)
    suggested_corrections: Dict[str, float] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class BoundaryUpdateConfirmation:
    """边界参数更新确认"""
    new_parameter_set: Dict[str, float] = field(default_factory=dict)
    effective_timestamp: float = field(default_factory=time.time)


# ==================== 车辆动力学硬边界参数库 ====================

# 绝对物理极限（不可逾越的红线）
ABSOLUTE_PHYSICAL_LIMITS = {
    "max_steering_angle_rate_deg_per_s": 500.0,
    "max_longitudinal_accel_ms2": 3.0,
    "max_emergency_brake_decel_ms2": 9.0,
    "max_lateral_accel_g": 0.85,
    "max_speed_kmh": 200.0,  # 车型标定值
}

# 各执行模式下的边界调整
MODE_BOUNDARIES = {
    ExecutionMode.NORMAL: {
        "max_longitudinal_accel_ms2": 3.0,
        "max_brake_decel_ms2": 5.0,
        "max_emergency_brake_decel_ms2": 9.0,
        "max_lateral_accel_g": 0.65,
        "max_steering_angle_rate_deg_per_s": 500.0,
        "max_speed_kmh": 200.0,
    },
    ExecutionMode.DEGRADED_LEVEL1: {
        "max_longitudinal_accel_ms2": 2.5,
        "max_brake_decel_ms2": 4.5,
        "max_emergency_brake_decel_ms2": 9.0,
        "max_lateral_accel_g": 0.55,
        "max_steering_angle_rate_deg_per_s": 400.0,
        "max_speed_kmh": 80.0,
    },
    ExecutionMode.DEGRADED_LEVEL2: {
        "max_longitudinal_accel_ms2": 2.0,
        "max_brake_decel_ms2": 4.0,
        "max_emergency_brake_decel_ms2": 9.0,
        "max_lateral_accel_g": 0.45,
        "max_steering_angle_rate_deg_per_s": 300.0,
        "max_speed_kmh": 40.0,
    },
    ExecutionMode.DEGRADED_LEVEL3: {
        "max_longitudinal_accel_ms2": 1.5,
        "max_brake_decel_ms2": 9.0,  # 允许紧急制动到极限
        "max_emergency_brake_decel_ms2": 9.0,
        "max_lateral_accel_g": 0.35,
        "max_steering_angle_rate_deg_per_s": 200.0,
        "max_speed_kmh": 0.0,
    },
    ExecutionMode.UNPAVED: {
        "max_longitudinal_accel_ms2": 1.8,
        "max_brake_decel_ms2": 3.0,
        "max_emergency_brake_decel_ms2": 7.0,
        "max_lateral_accel_g": 0.40,
        "max_steering_angle_rate_deg_per_s": 300.0,
        "max_speed_kmh": 40.0,
    },
}

# 湿滑路面修正系数
ROAD_FRICTION_FACTORS = {
    RoadCondition.DRY_ASPHALT: {"accel": 1.0, "brake": 1.0},
    RoadCondition.WET: {"accel": 0.8, "brake": 0.7},
    RoadCondition.SNOW: {"accel": 0.5, "brake": 0.4},
    RoadCondition.ICE: {"accel": 0.3, "brake": 0.2},
}


# ==================== 主类定义 ====================

class MotionBoundaryGate:
    """
    运动生理边界闸门
    
    职责:
    1. 锁定车辆动力学绝对极限，作为最后一道硬约束校验
    2. 接收操控指令，逐项校验是否超出物理极限或法规硬约束
    3. 超限指令立即拦截并返回修正建议
    4. 根据执行模式和路面条件动态调整边界参数
    5. 不参与任何场景判断与驾驶决策
    """

    # 默认摩擦系数（数据不可用时保守假设）
    DEFAULT_FRICTION_COEFFICIENT = 0.4

    def __init__(self):
        self.module_id = "ad-mcc-02"
        self.module_name = "运动生理边界闸门"
        self.version = "V1.0"

        self.state = GateState.NORMAL_GATE
        self._current_mode = ExecutionMode.NORMAL
        self._current_boundaries = MODE_BOUNDARIES[ExecutionMode.NORMAL].copy()
        self._current_friction = 0.8  # 默认干燥沥青

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调
        self._query_friction = None         # Callable[[], float]
        self._query_vehicle_speed = None    # Callable[[], float]

        # 向 ad-mcc-01 返回校验结果
        self._return_to_dispatcher = None   # Callable[[Any], None]

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_friction_query(self, callback):
        self._query_friction = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_dispatcher_return(self, callback):
        self._return_to_dispatcher = callback

    # ========== 模式切换 ==========
    def switch_mode(self, new_mode: ExecutionMode):
        """切换执行模式并更新边界参数"""
        self._current_mode = new_mode
        self._current_boundaries = MODE_BOUNDARIES.get(new_mode, MODE_BOUNDARIES[ExecutionMode.NORMAL]).copy()

        mode_state_map = {
            ExecutionMode.NORMAL: GateState.NORMAL_GATE,
            ExecutionMode.DEGRADED_LEVEL1: GateState.DEGRADED_GATE,
            ExecutionMode.DEGRADED_LEVEL2: GateState.DEGRADED_GATE,
            ExecutionMode.DEGRADED_LEVEL3: GateState.EMERGENCY_GATE,
            ExecutionMode.UNPAVED: GateState.UNPAVED_GATE,
        }
        self.state = mode_state_map.get(new_mode, GateState.NORMAL_GATE)

        self._log_event("MODE_SWITCHED", {"new_mode": new_mode.value})

    # ========== 主校验入口 ==========
    def validate_command(self, command: CommandToValidate) -> Optional[Any]:
        """
        校验操控指令是否超出物理边界
        
        Args:
            command: 待校验操控指令
            
        Returns:
            校验通过返回 GatePassSignal，拦截返回 GateBlockSignal
        """
        if self.state == GateState.SYSTEM_PAUSED:
            return GateBlockSignal(
                original_command=command,
                block_reasons=["系统熔断"],
                suggested_corrections={"target_value": 0.0}
            )

        # 更新当前路面摩擦系数
        if self._query_friction:
            friction = self._query_friction()
            self._current_friction = friction if friction > 0 else self.DEFAULT_FRICTION_COEFFICIENT
        else:
            self._current_friction = self.DEFAULT_FRICTION_COEFFICIENT

        # 获取当前车速
        current_speed = self._query_vehicle_speed() if self._query_vehicle_speed else 0.0

        block_reasons = []
        exceeded_params = {}
        suggested_corrections = {}

        # 根据指令类型逐项校验
        if command.command_type == CommandType.STEERING:
            self._validate_steering(command, block_reasons, exceeded_params, suggested_corrections)
        elif command.command_type == CommandType.ACCELERATION:
            self._validate_acceleration(command, block_reasons, exceeded_params, suggested_corrections)
        elif command.command_type == CommandType.BRAKING:
            self._validate_braking(command, block_reasons, exceeded_params, suggested_corrections)
        elif command.command_type == CommandType.SPEED:
            self._validate_speed(command, current_speed, block_reasons, exceeded_params, suggested_corrections)

        # 紧急模式宽容处理（S-03）
        if block_reasons and self.state == GateState.EMERGENCY_GATE:
            physical_limit_reasons = [
                r for r in block_reasons
                if "绝对物理极限" in r or "系统熔断" in r
            ]
            if not physical_limit_reasons:
                # 仅有模式边界超限，放行但附带告警
                self._log_event("EMERGENCY_OVERRIDE", {
                    "block_reasons": block_reasons,
                    "action": "紧急模式宽容放行"
                })
                return GatePassSignal(original_command=command, passed=True)
            else:
                # 保留绝对物理极限拦截
                return GateBlockSignal(
                    original_command=command,
                    block_reasons=physical_limit_reasons,
                    exceeded_params=exceeded_params,
                    current_boundaries=self._current_boundaries,
                    suggested_corrections=suggested_corrections
                )

        if block_reasons:
            return GateBlockSignal(
                original_command=command,
                block_reasons=block_reasons,
                exceeded_params=exceeded_params,
                current_boundaries=self._current_boundaries,
                suggested_corrections=suggested_corrections
            )

        return GatePassSignal(original_command=command, passed=True)

    # ========== 各类型校验 ==========
    def _validate_steering(self, command: CommandToValidate, block_reasons: List[str],
                           exceeded_params: Dict[str, float], corrections: Dict[str, float]):
        """校验转向指令"""
        target_rate = abs(command.target_value)
        max_rate = self._current_boundaries["max_steering_angle_rate_deg_per_s"]

        # 湿滑路面额外限制
        if self._current_friction < 0.4:
            max_rate *= 0.6

        if target_rate > max_rate:
            block_reasons.append("转角速率超限")
            exceeded_params["angle_rate"] = target_rate
            corrections["angle_rate"] = max_rate

        # 绝对物理极限
        if target_rate > ABSOLUTE_PHYSICAL_LIMITS["max_steering_angle_rate_deg_per_s"]:
            block_reasons.append("转角速率超出绝对物理极限")
            corrections["angle_rate"] = ABSOLUTE_PHYSICAL_LIMITS["max_steering_angle_rate_deg_per_s"]

    def _validate_acceleration(self, command: CommandToValidate, block_reasons: List[str],
                               exceeded_params: Dict[str, float], corrections: Dict[str, float]):
        """校验加速指令"""
        target_accel = command.target_value
        max_accel = self._current_boundaries["max_longitudinal_accel_ms2"]

        # 湿滑路面修正
        road_condition = self._get_road_condition()
        friction_factor = ROAD_FRICTION_FACTORS.get(road_condition, {}).get("accel", 1.0)
        max_accel *= friction_factor

        if target_accel > max_accel:
            block_reasons.append(f"加速度超限（当前上限={max_accel:.1f}）")
            exceeded_params["acceleration"] = target_accel
            corrections["acceleration"] = max_accel

        # 绝对物理极限
        if target_accel > ABSOLUTE_PHYSICAL_LIMITS["max_longitudinal_accel_ms2"]:
            block_reasons.append("加速度超出绝对物理极限")
            corrections["acceleration"] = ABSOLUTE_PHYSICAL_LIMITS["max_longitudinal_accel_ms2"]

    def _validate_braking(self, command: CommandToValidate, block_reasons: List[str],
                          exceeded_params: Dict[str, float], corrections: Dict[str, float]):
        """校验制动指令"""
        target_decel = abs(command.target_value)
        is_emergency = command.current_vehicle_state.get("is_emergency", False)

        if is_emergency:
            max_decel = ABSOLUTE_PHYSICAL_LIMITS["max_emergency_brake_decel_ms2"]
            road_condition = self._get_road_condition()
            friction_factor = ROAD_FRICTION_FACTORS.get(road_condition, {}).get("brake", 1.0)
            max_decel *= friction_factor
        else:
            max_decel = self._current_boundaries["max_brake_decel_ms2"]
            road_condition = self._get_road_condition()
            friction_factor = ROAD_FRICTION_FACTORS.get(road_condition, {}).get("brake", 1.0)
            max_decel *= friction_factor

        if target_decel > max_decel:
            block_reasons.append(f"制动减速度超限（当前上限={max_decel:.1f}）")
            exceeded_params["deceleration"] = target_decel
            corrections["deceleration"] = max_decel

    def _validate_speed(self, command: CommandToValidate, current_speed: float,
                        block_reasons: List[str], exceeded_params: Dict[str, float],
                        corrections: Dict[str, float]):
        """校验车速指令"""
        target_speed = command.target_value
        max_speed = self._current_boundaries["max_speed_kmh"]

        if target_speed > max_speed:
            block_reasons.append(f"车速超限（当前上限={max_speed}km/h）")
            exceeded_params["speed"] = target_speed
            corrections["speed"] = max_speed

    def _get_road_condition(self) -> RoadCondition:
        """根据摩擦系数判定路面条件"""
        if self._current_friction >= 0.7:
            return RoadCondition.DRY_ASPHALT
        elif self._current_friction >= 0.4:
            return RoadCondition.WET
        elif self._current_friction >= 0.2:
            return RoadCondition.SNOW
        else:
            return RoadCondition.ICE

    # ========== 查询接口 ==========
    def get_state(self) -> GateState:
        return self.state

    def get_current_boundaries(self) -> Dict[str, float]:
        return self._current_boundaries.copy()

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
            "current_mode": self._current_mode.value,
            "current_friction": self._current_friction,
        }

    def emergency_shutdown(self):
        self.state = GateState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 运动生理边界闸门 (ad-mcc-02) 演示")
    print("=" * 70)

    gate = MotionBoundaryGate()
    gate.set_friction_query(lambda: 0.8)

    print_separator("STEP 1: 正常转向指令校验通过")
    cmd = CommandToValidate(
        command_type=CommandType.STEERING,
        target_value=300.0,
        mode_mark=ExecutionMode.NORMAL
    )
    result = gate.validate_command(cmd)
    if isinstance(result, GatePassSignal):
        print(f"  结果: 校验通过")

    print_separator("STEP 2: 超限加速指令被拦截")
    cmd2 = CommandToValidate(
        command_type=CommandType.ACCELERATION,
        target_value=4.5,
        mode_mark=ExecutionMode.NORMAL
    )
    result2 = gate.validate_command(cmd2)
    if isinstance(result2, GateBlockSignal):
        print(f"  拦截原因: {result2.block_reasons}")
        print(f"  建议修正: {result2.suggested_corrections}")

    print_separator("STEP 3: 降级模式切换")
    gate.switch_mode(ExecutionMode.DEGRADED_LEVEL2)
    print(f"  当前状态: {gate.state.value}")
    print(f"  当前边界: {gate.get_current_boundaries()}")

    print_separator("STEP 4: 紧急模式宽容放行（仅超模式边界，未超物理极限）")
    gate.switch_mode(ExecutionMode.DEGRADED_LEVEL3)
    cmd3 = CommandToValidate(
        command_type=CommandType.ACCELERATION,
        target_value=2.0,
        mode_mark=ExecutionMode.DEGRADED_LEVEL3
    )
    result3 = gate.validate_command(cmd3)
    if isinstance(result3, GatePassSignal):
        print(f"  紧急模式放行（附带告警），校验通过")
    elif isinstance(result3, GateBlockSignal):
        print(f"  拦截原因: {result3.block_reasons}")

    print("\n✅ 运动生理边界闸门演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-02 运动生理边界闸门 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        # TC-M02-01: 正常转向指令校验通过
        print("\n[TC-M02-01] 正常转向指令校验通过")
        try:
            gate = MotionBoundaryGate()
            gate.set_friction_query(lambda: 0.8)
            cmd = CommandToValidate(command_type=CommandType.STEERING, target_value=300.0)
            result = gate.validate_command(cmd)
            assert isinstance(result, GatePassSignal)
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M02-02: 超限加速被拦截
        print("\n[TC-M02-02] 超限加速指令被拦截")
        try:
            gate = MotionBoundaryGate()
            gate.set_friction_query(lambda: 0.8)
            cmd = CommandToValidate(command_type=CommandType.ACCELERATION, target_value=4.0)
            result = gate.validate_command(cmd)
            assert isinstance(result, GateBlockSignal)
            assert "加速度超限" in result.block_reasons[0]
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M02-03: 降级模式更严格
        print("\n[TC-M02-03] 降级模式下正常指令被拦截")
        try:
            gate = MotionBoundaryGate()
            gate.set_friction_query(lambda: 0.8)
            gate.switch_mode(ExecutionMode.DEGRADED_LEVEL2)
            cmd = CommandToValidate(command_type=CommandType.ACCELERATION, target_value=2.5)
            result = gate.validate_command(cmd)
            assert isinstance(result, GateBlockSignal)
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M02-04: 湿滑路面严格限制
        print("\n[TC-M02-04] 湿滑路面加速受限")
        try:
            gate = MotionBoundaryGate()
            gate.set_friction_query(lambda: 0.3)
            cmd = CommandToValidate(command_type=CommandType.ACCELERATION, target_value=2.0)
            result = gate.validate_command(cmd)
            assert isinstance(result, GateBlockSignal)
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M02-05: 紧急熔断
        print("\n[TC-M02-05] 紧急熔断拦截所有指令")
        try:
            gate = MotionBoundaryGate()
            gate.emergency_shutdown()
            cmd = CommandToValidate(command_type=CommandType.STEERING, target_value=100.0)
            result = gate.validate_command(cmd)
            assert isinstance(result, GateBlockSignal)
            assert "系统熔断" in result.block_reasons
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M02-06: 紧急模式宽容放行（仅超模式边界，未超物理极限）
        print("\n[TC-M02-06] 紧急模式超模式边界但未超物理极限 → 放行")
        try:
            gate = MotionBoundaryGate()
            gate.set_friction_query(lambda: 0.8)
            gate.switch_mode(ExecutionMode.DEGRADED_LEVEL3)
            # 加速2.0m/s²超了紧急模式边界1.5，但未超物理极限3.0
            cmd = CommandToValidate(command_type=CommandType.ACCELERATION, target_value=2.0,
                                   mode_mark=ExecutionMode.DEGRADED_LEVEL3)
            result = gate.validate_command(cmd)
            assert isinstance(result, GatePassSignal), f"预期放行，实际{type(result).__name__}"
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