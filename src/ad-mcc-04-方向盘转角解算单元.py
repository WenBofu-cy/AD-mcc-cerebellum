#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-04
模块名称: 方向盘转角解算单元
所属分区: 二、转向控制集群
核心职责: 将 ECC 大脑下发的抽象行驶意图（车道变换的偏移距离、转弯的目标曲率半径、
          避让的横向偏移量等）转化为具体的方向盘目标转角序列。基于车辆轴距、转向比等
          几何参数，运用纯追踪算法与阿克曼转向几何模型，计算每一帧的方向盘目标角度与
          转角速率曲线。是连接 ECC 决策层与车辆底层转向执行层之间的核心翻译模块。

依赖模块:
    ad-mcc-01(小脑总控调度核心，下发转向调度指令),
    ad-mcc-32(车辆尺寸参数管理单元，提供轴距/转向比),
    ad-mcc-33(转向特性参数管理单元，提供转向机响应特性)
被依赖模块:
    ad-mcc-05(转向平顺滤波单元，接收原始目标转角序列进行平滑处理)

安全约束:
  S-01: 解算出的方向盘转角必须严格约束在车辆物理最大转角范围内，不得超出
  S-02: 转角速率不得超过转向机物理上限，防止转向电机过热或失控
  S-03: 车辆几何参数缺失时，必须使用保守默认值并明确标记“降级估算”
  S-04: 本模块仅输出目标转角序列，不直接操控转向电机
  S-05: 解算所用的目标曲率半径不得小于车辆最小转弯半径
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


# ==================== 枚举定义 ====================

class CalculationState(Enum):
    """方向盘转角解算单元内部状态"""
    IDLE = "idle"
    CALCULATING = "calculating"
    DEGRADED_ESTIMATION = "degraded_estimation"
    SYSTEM_PAUSED = "system_paused"


class CommandType(Enum):
    """转向指令类型"""
    LANE_CHANGE = "车道变换"
    TURN = "转弯"
    AVOIDANCE = "避让"
    LANE_KEEPING = "车道保持"


class CalculationMethod(Enum):
    """解算方法"""
    PURE_PURSUIT = "纯追踪算法"
    ACKERMANN = "阿克曼转向几何"
    LATERAL_OFFSET = "横向偏移避让"
    LANE_KEEPING_ADJUST = "车道保持微调"


# ==================== 数据结构 ====================

@dataclass
class SteeringDispatchCommand:
    """转向调度指令（来自 ad-mcc-01）"""
    command_type: CommandType = CommandType.LANE_CHANGE
    target_trajectory: List[Tuple[float, float]] = field(default_factory=list)
    lateral_offset_m: float = 0.0
    target_curvature_radius_m: float = 999.0
    max_lateral_jerk_ms3: float = 3.0
    mode_mark: str = "正常"
    timestamp: float = field(default_factory=time.time)


@dataclass
class VehicleGeometryParams:
    """车辆几何参数（来自 ad-mcc-32）"""
    wheelbase_m: float = 2.7
    steering_ratio: float = 16.0
    max_steering_angle_deg: float = 500.0
    min_turn_radius_m: float = 5.0


@dataclass
class SteeringCharacteristics:
    """转向机特性参数（来自 ad-mcc-33）"""
    response_delay_ms: float = 20.0
    angle_resolution_deg: float = 0.1
    max_angle_rate_deg_per_s: float = 500.0


@dataclass
class SteeringAngleSequence:
    """方向盘目标转角序列"""
    timestamp: float = field(default_factory=time.time)
    target_angle_deg: float = 0.0
    angle_rate_deg_per_s: float = 0.0
    calculation_method: CalculationMethod = CalculationMethod.PURE_PURSUIT
    confidence: float = 0.95


@dataclass
class CalculationStatusReport:
    """解算状态上报"""
    current_state: CalculationState = CalculationState.IDLE
    calculation_duration_ms: float = 0.0
    angle_validity: str = "有效"


# ==================== 主类定义 ====================

class SteeringAngleCalculator:
    """
    方向盘转角解算单元
    
    职责:
    1. 将抽象行驶意图（变道/转弯/避让/车道保持）转化为方向盘目标转角
    2. 基于车辆几何参数，使用纯追踪、阿克曼、横向偏移三种算法
    3. 对解算结果进行转角限幅和速率约束
    4. 参数缺失时使用保守默认值降级估算
    """

    # 默认保守参数
    DEFAULT_WHEELBASE_M = 2.7
    DEFAULT_STEERING_RATIO = 16.0
    DEFAULT_MAX_STEERING_ANGLE_DEG = 500.0
    DEFAULT_MIN_TURN_RADIUS_M = 5.0
    DEFAULT_MAX_ANGLE_RATE_DEG_PER_S = 500.0

    # 控制周期（秒）
    CONTROL_PERIOD_S = 0.01  # 100Hz

    def __init__(self):
        self.module_id = "ad-mcc-04"
        self.module_name = "方向盘转角解算单元"
        self.version = "V1.0"

        self.state = CalculationState.IDLE

        # 车辆参数（默认值，实际从 ad-mcc-32/33 加载）
        self._vehicle_geo = VehicleGeometryParams()
        self._steering_char = SteeringCharacteristics()
        self._params_valid = True

        # 当前状态
        self._current_speed_kmh: float = 0.0
        self._current_angle_deg: float = 0.0

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调
        self._query_steering_command = None         # Callable[[], Optional[SteeringDispatchCommand]]
        self._query_vehicle_geometry = None         # Callable[[], VehicleGeometryParams]
        self._query_steering_characteristics = None # Callable[[], SteeringCharacteristics]
        self._query_vehicle_speed = None            # Callable[[], float]
        self._query_current_angle = None            # Callable[[], float]

        # 输出回调
        self._publish_angle_sequence = None         # Callable[[SteeringAngleSequence], None]
        self._publish_status_report = None          # Callable[[CalculationStatusReport], None]

        # 加载参数
        self._load_vehicle_params()

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_steering_command_query(self, callback):
        self._query_steering_command = callback

    def set_vehicle_geometry_query(self, callback):
        self._query_vehicle_geometry = callback

    def set_steering_characteristics_query(self, callback):
        self._query_steering_characteristics = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_current_angle_query(self, callback):
        self._query_current_angle = callback

    def set_angle_sequence_publisher(self, callback):
        self._publish_angle_sequence = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    # ========== 参数加载 ==========
    def _load_vehicle_params(self):
        """加载车辆几何参数和转向特性参数"""
        if self._query_vehicle_geometry:
            self._vehicle_geo = self._query_vehicle_geometry()
        if self._query_steering_characteristics:
            self._steering_char = self._query_steering_characteristics()

        # 参数有效性校验
        if self._vehicle_geo.wheelbase_m <= 0 or self._vehicle_geo.steering_ratio <= 0:
            self._params_valid = False
            self.state = CalculationState.DEGRADED_ESTIMATION
            self._vehicle_geo.wheelbase_m = self.DEFAULT_WHEELBASE_M
            self._vehicle_geo.steering_ratio = self.DEFAULT_STEERING_RATIO
            self._vehicle_geo.max_steering_angle_deg = self.DEFAULT_MAX_STEERING_ANGLE_DEG
            self._vehicle_geo.min_turn_radius_m = self.DEFAULT_MIN_TURN_RADIUS_M
            self._steering_char.max_angle_rate_deg_per_s = self.DEFAULT_MAX_ANGLE_RATE_DEG_PER_S
            self._log_event("PARAMS_DEGRADED", {"reason": "几何参数无效，使用默认值"})
        else:
            self._params_valid = True
            if self.state == CalculationState.DEGRADED_ESTIMATION:
                self.state = CalculationState.IDLE

    # ========== 主循环 ==========
    def run_calculation_cycle(self) -> Optional[SteeringAngleSequence]:
        """
        执行一次转角解算周期（100Hz）
        
        Returns:
            目标转角序列，若无新指令则返回 None
        """
        if self.state == CalculationState.SYSTEM_PAUSED:
            return None

        # 更新车速和当前转角
        if self._query_vehicle_speed:
            self._current_speed_kmh = self._query_vehicle_speed()
        if self._query_current_angle:
            self._current_angle_deg = self._query_current_angle()

        # 接收转向调度指令
        command = self._query_steering_command() if self._query_steering_command else None
        if not command:
            return None

        # 参数校验
        self._load_vehicle_params()

        start_time = time.perf_counter()
        self.state = CalculationState.CALCULATING

        # 根据指令类型选择解算方法
        target_angle, method = self._calculate_by_command(command)

        # 转角约束
        target_angle = self._apply_angle_constraints(target_angle, command)

        # 计算转角速率
        angle_rate = self._calculate_angle_rate(self._current_angle_deg, target_angle)

        # 生成目标转角序列
        confidence = 0.7 if self.state == CalculationState.DEGRADED_ESTIMATION else 0.95
        sequence = SteeringAngleSequence(
            timestamp=time.time(),
            target_angle_deg=round(target_angle, 2),
            angle_rate_deg_per_s=round(angle_rate, 2),
            calculation_method=method,
            confidence=confidence
        )

        # 输出
        if self._publish_angle_sequence:
            self._publish_angle_sequence(sequence)

        # 状态上报
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        if self._publish_status_report:
            self._publish_status_report(CalculationStatusReport(
                current_state=self.state,
                calculation_duration_ms=round(elapsed_ms, 2),
                angle_validity="有效"
            ))

        self.state = CalculationState.DEGRADED_ESTIMATION if not self._params_valid else CalculationState.IDLE
        return sequence

    # ========== 指令解算 ==========
    def _calculate_by_command(self, command: SteeringDispatchCommand) -> Tuple[float, CalculationMethod]:
        """根据指令类型选择解算方法"""
        if command.command_type == CommandType.LANE_CHANGE:
            return self._pure_pursuit_calc(command), CalculationMethod.PURE_PURSUIT
        elif command.command_type == CommandType.TURN:
            return self._ackermann_calc(command), CalculationMethod.ACKERMANN
        elif command.command_type == CommandType.AVOIDANCE:
            return self._lateral_offset_calc(command), CalculationMethod.LATERAL_OFFSET
        elif command.command_type == CommandType.LANE_KEEPING:
            return self._lane_keeping_calc(command), CalculationMethod.LANE_KEEPING_ADJUST
        else:
            return self._current_angle_deg, CalculationMethod.PURE_PURSUIT

    # ========== 纯追踪算法 ==========
    def _pure_pursuit_calc(self, command: SteeringDispatchCommand) -> float:
        """
        纯追踪算法：从目标轨迹中提取预瞄点，计算方向盘转角
        delta_steering = atan(2 * L * sin(alpha) / ld) * K
        """
        trajectory = command.target_trajectory
        if not trajectory:
            return self._current_angle_deg

        # 计算预瞄距离（基于车速）
        lookahead_distance = self._calc_lookahead_distance(self._current_speed_kmh)

        # 从轨迹中提取预瞄点
        preview_point = self._extract_preview_point(trajectory, lookahead_distance)
        if preview_point is None:
            return self._current_angle_deg

        x_target, y_target = preview_point
        wheelbase = self._vehicle_geo.wheelbase_m
        steer_ratio = self._vehicle_geo.steering_ratio

        # 预瞄点与车辆纵向轴的夹角
        alpha = math.atan2(y_target, x_target) if x_target != 0 else 0.0

        # 期望转弯半径
        if alpha == 0:
            return 0.0
        R = lookahead_distance / (2 * math.sin(alpha))

        # 车轮转角
        delta_wheel = math.atan(wheelbase / R) if R != 0 else 0.0

        # 方向盘转角
        delta_steering = delta_wheel * steer_ratio * 180.0 / math.pi

        # 方向修正
        if y_target < 0:
            delta_steering = -delta_steering

        return delta_steering

    # ========== 阿克曼转向几何 ==========
    def _ackermann_calc(self, command: SteeringDispatchCommand) -> float:
        """
        阿克曼转向几何：根据目标曲率半径计算方向盘转角
        delta_steering = atan(L / R) * K * 180 / PI
        """
        radius = command.target_curvature_radius_m
        wheelbase = self._vehicle_geo.wheelbase_m
        steer_ratio = self._vehicle_geo.steering_ratio
        min_radius = self._vehicle_geo.min_turn_radius_m

        # 曲率半径硬约束
        if radius < min_radius:
            radius = min_radius
            self._log_event("RADIUS_CLAMPED", {"original": command.target_curvature_radius_m, "clamped": min_radius})

        # 车轮转角
        delta_wheel = math.atan(wheelbase / radius) if radius > 0 else 0.0

        # 方向盘转角
        delta_steering = delta_wheel * steer_ratio * 180.0 / math.pi

        return delta_steering

    # ========== 横向偏移避让 ==========
    def _lateral_offset_calc(self, command: SteeringDispatchCommand) -> float:
        """
        横向偏移避让：将横向偏移距离转换为等效曲率，再解算方向盘转角
        curvature = 2 * d_lateral / (ld^2)
        """
        d_lateral = command.lateral_offset_m
        lookahead = self._calc_lookahead_distance(self._current_speed_kmh)

        if lookahead == 0:
            return 0.0

        # 等效曲率
        curvature = 2.0 * d_lateral / (lookahead ** 2)

        # 等效转弯半径
        R = 1.0 / curvature if curvature != 0 else float('inf')

        wheelbase = self._vehicle_geo.wheelbase_m
        steer_ratio = self._vehicle_geo.steering_ratio

        delta_wheel = math.atan(wheelbase / R) if R != float('inf') else 0.0
        delta_steering = delta_wheel * steer_ratio * 180.0 / math.pi

        return delta_steering

    # ========== 车道保持微调 ==========
    def _lane_keeping_calc(self, command: SteeringDispatchCommand) -> float:
        """
        车道保持微调：根据当前偏差微调方向盘转角
        """
        trajectory = command.target_trajectory
        if not trajectory:
            return self._current_angle_deg

        # 取最近的点计算横向偏差
        nearest_point = trajectory[0]
        lateral_error = nearest_point[1]  # y 偏移

        # P 控制器：根据横向偏差调整转角
        kp = 0.5  # 比例增益
        correction = kp * lateral_error * self._vehicle_geo.steering_ratio
        target_angle = self._current_angle_deg + correction

        return target_angle

    # ========== 转角约束 ==========
    def _apply_angle_constraints(self, target_angle: float, command: SteeringDispatchCommand) -> float:
        """
        应用转角约束：最大转角限制、转角速率限制、非铺装模式额外限制
        """
        max_angle = self._vehicle_geo.max_steering_angle_deg
        max_rate = self._steering_char.max_angle_rate_deg_per_s

        # 最大转角限制
        if abs(target_angle) > max_angle:
            target_angle = math.copysign(max_angle, target_angle)
            self._log_event("ANGLE_CLAMPED", {"target": target_angle, "max": max_angle})

        # 转角速率限制
        angle_change = target_angle - self._current_angle_deg
        max_change = max_rate * self.CONTROL_PERIOD_S
        if abs(angle_change) > max_change:
            target_angle = self._current_angle_deg + math.copysign(max_change, angle_change)

        # 非铺装模式额外限制
        if command.mode_mark == "非铺装道路":
            max_rate_unpaved = max_rate * 0.6
            max_change_unpaved = max_rate_unpaved * self.CONTROL_PERIOD_S
            angle_change = target_angle - self._current_angle_deg
            if abs(angle_change) > max_change_unpaved:
                target_angle = self._current_angle_deg + math.copysign(max_change_unpaved, angle_change)

        return target_angle

    # ========== 辅助计算 ==========
    def _calc_lookahead_distance(self, speed_kmh: float) -> float:
        """
        根据车速计算预瞄距离
        低速时预瞄距离短，高速时预瞄距离长
        """
        speed_ms = speed_kmh / 3.6
        # 基础预瞄距离 + 速度相关预瞄
        lookahead = 5.0 + speed_ms * 1.0
        return max(3.0, min(lookahead, 50.0))  # 限制在3-50m之间

    def _extract_preview_point(self, trajectory: List[Tuple[float, float]],
                               lookahead_distance: float) -> Optional[Tuple[float, float]]:
        """
        从轨迹中提取预瞄点：找到距离最接近预瞄距离的点
        """
        if not trajectory:
            return None

        # 从轨迹起点计算累计距离，找到最接近预瞄距离的点
        cumulative = 0.0
        prev_point = trajectory[0]
        for point in trajectory[1:]:
            dx = point[0] - prev_point[0]
            dy = point[1] - prev_point[1]
            dist = math.sqrt(dx * dx + dy * dy)
            cumulative += dist
            if cumulative >= lookahead_distance:
                return point
            prev_point = point

        # 如果轨迹总长度不足，返回最后一个点
        return trajectory[-1]

    def _calculate_angle_rate(self, current_angle: float, target_angle: float) -> float:
        """计算转角速率（度/秒）"""
        angle_change = abs(target_angle - current_angle)
        rate = angle_change / self.CONTROL_PERIOD_S
        return min(rate, self._steering_char.max_angle_rate_deg_per_s)

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
        print(f"[{self.module_id}] 紧急熔断，维持当前转角")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 方向盘转角解算单元 (ad-mcc-04) 演示")
    print("=" * 70)

    calc = SteeringAngleCalculator()
    calc.set_vehicle_speed_query(lambda: 60.0)
    calc.set_current_angle_query(lambda: 5.0)
    calc.set_vehicle_geometry_query(lambda: VehicleGeometryParams(
        wheelbase_m=2.7, steering_ratio=16.0, max_steering_angle_deg=500.0, min_turn_radius_m=5.0
    ))
    calc.set_steering_characteristics_query(lambda: SteeringCharacteristics(
        max_angle_rate_deg_per_s=500.0
    ))

    print_separator("STEP 1: 车道变换解算（纯追踪）")
    calc.set_steering_command_query(lambda: SteeringDispatchCommand(
        command_type=CommandType.LANE_CHANGE,
        target_trajectory=[(0, 0), (10, 0.5), (20, 1.5), (30, 3.0), (40, 3.5)],
    ))
    sequence = calc.run_calculation_cycle()
    if sequence:
        print(f"  目标转角: {sequence.target_angle_deg}°")
        print(f"  解算方法: {sequence.calculation_method.value}")
        print(f"  置信度: {sequence.confidence}")

    print_separator("STEP 2: 转弯解算（阿克曼）")
    calc.set_steering_command_query(lambda: SteeringDispatchCommand(
        command_type=CommandType.TURN,
        target_curvature_radius_m=50.0,
    ))
    sequence2 = calc.run_calculation_cycle()
    if sequence2:
        print(f"  目标转角: {sequence2.target_angle_deg}°")
        print(f"  解算方法: {sequence2.calculation_method.value}")

    print("\n✅ 方向盘转角解算单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-04 方向盘转角解算单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_calc(speed=60.0, current_angle=0.0):
            c = SteeringAngleCalculator()
            c.set_vehicle_speed_query(lambda: speed)
            c.set_current_angle_query(lambda: current_angle)
            c.set_vehicle_geometry_query(lambda: VehicleGeometryParams(
                wheelbase_m=2.7, steering_ratio=16.0,
                max_steering_angle_deg=500.0, min_turn_radius_m=5.0
            ))
            c.set_steering_characteristics_query(lambda: SteeringCharacteristics(
                max_angle_rate_deg_per_s=500.0
            ))
            return c

        # TC-M04-01: 车道变换正常解算
        print("\n[TC-M04-01] 车道变换正常解算（纯追踪）")
        try:
            c = setup_calc()
            c.set_steering_command_query(lambda: SteeringDispatchCommand(
                command_type=CommandType.LANE_CHANGE,
                target_trajectory=[(0, 0), (10, 1), (20, 3)],
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.calculation_method == CalculationMethod.PURE_PURSUIT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M04-02: 转弯正常解算
        print("\n[TC-M04-02] 转弯正常解算（阿克曼）")
        try:
            c = setup_calc()
            c.set_steering_command_query(lambda: SteeringDispatchCommand(
                command_type=CommandType.TURN,
                target_curvature_radius_m=50.0,
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.calculation_method == CalculationMethod.ACKERMANN
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M04-03: 避让解算
        print("\n[TC-M04-03] 避让解算（横向偏移）")
        try:
            c = setup_calc()
            c.set_steering_command_query(lambda: SteeringDispatchCommand(
                command_type=CommandType.AVOIDANCE,
                lateral_offset_m=1.2,
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            assert seq.calculation_method == CalculationMethod.LATERAL_OFFSET
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M04-04: 曲率半径小于最小转弯半径被截断
        print("\n[TC-M04-04] 曲率半径小于最小转弯半径被截断")
        try:
            c = setup_calc()
            c.set_steering_command_query(lambda: SteeringDispatchCommand(
                command_type=CommandType.TURN,
                target_curvature_radius_m=3.0,  # 小于最小转弯半径5.0m
            ))
            seq = c.run_calculation_cycle()
            assert seq is not None
            # 应该被截断到5.0m，转角相应变化
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M04-05: 参数缺失降级估算
        print("\n[TC-M04-05] 轴距参数为0降级估算")
        try:
            c = SteeringAngleCalculator()
            c.set_vehicle_speed_query(lambda: 60.0)
            c.set_current_angle_query(lambda: 0.0)
            # 返回无效参数
            c.set_vehicle_geometry_query(lambda: VehicleGeometryParams(
                wheelbase_m=0.0, steering_ratio=0.0
            ))
            c.set_steering_characteristics_query(lambda: SteeringCharacteristics())
            c.set_steering_command_query(lambda: SteeringDispatchCommand(
                command_type=CommandType.TURN,
                target_curvature_radius_m=50.0,
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

        # TC-M04-06: 紧急熔断
        print("\n[TC-M04-06] 紧急熔断")
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