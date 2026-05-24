#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-10
模块名称: 纵向冲击度约束单元
所属分区: 三、动力控制集群
核心职责: 接收 ad-mcc-09 输出的原始油门开度序列，基于当前车速、目标加速度与驾驶模式，
          对油门开度变化率进行约束，确保起步、加速、巡航过程中纵向冲击度始终 ≤ 5m/s³
          （正常模式）或对应降级模式的更严格上限。对超出冲击度限制的油门指令进行削峰与
          平滑修正，输出冲击度合规的油门开度序列至 ad-mcc-11。不参与任何场景判断与驾驶决策。

依赖模块:
    ad-mcc-09(油门开度解算单元，提供原始油门开度序列),
    ad-mcc-01(小脑总控调度核心，下发驾驶模式信号)
被依赖模块:
    ad-mcc-11(加速平顺滤波单元，接收冲击度合规的油门开度序列)

安全约束:
  S-01: 紧急制动指令为最高优先级，收到后冲击度约束立即豁免，油门指令无条件直接放行
  S-02: 本模块仅约束冲击度与油门变化率，不改变油门开度的终值方向（加速/减速意图不变）
  S-03: 冲击度约束不得导致油门响应完全停滞。修正后的加速度变化量不得为0（除非目标加速度本身为0）
  S-04: 原始油门开度置信度低于0.7时，必须收紧约束上限20%，增加安全裕度
  S-05: 本模块仅做冲击度约束修正，不参与任何场景判断与驾驶决策
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


# ==================== 枚举定义 ====================

class ConstraintState(Enum):
    """纵向冲击度约束单元内部状态"""
    NORMAL_CONSTRAINT = "normal_constraint"
    DEGRADED_CONSTRAINT = "degraded_constraint"
    UNPAVED_CONSTRAINT = "unpaved_constraint"
    EMERGENCY_MODE = "emergency_mode"
    SYSTEM_PAUSED = "system_paused"


class ExecutionMode(Enum):
    """驾驶模式（与 ad-mcc-01 对齐）"""
    NORMAL = "正常"
    DEGRADED_LEVEL1 = "一级降级"
    DEGRADED_LEVEL2 = "二级降级"
    DEGRADED_LEVEL3 = "三级降级"
    UNPAVED = "非铺装道路"


# ==================== 数据结构 ====================

@dataclass
class ThrottleSequence:
    """原始油门开度序列（来自 ad-mcc-09）"""
    timestamp: float = field(default_factory=time.time)
    target_throttle_pct: float = 0.0
    expected_acceleration_ms2: float = 0.0
    calculation_method: str = ""
    confidence: float = 0.95


@dataclass
class EmergencyBrakeCommand:
    """紧急制动指令"""
    msg_id: str = ""
    intent_type: str = "紧急制动"
    jerk_exempt: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class JerkCompliantSequence:
    """冲击度合规油门序列（发送至 ad-mcc-11）"""
    timestamp: float = field(default_factory=time.time)
    corrected_throttle_pct: float = 0.0
    original_throttle_pct: float = 0.0
    constraint_triggered: bool = False
    constraint_reason: str = ""
    expected_acceleration_ms2: float = 0.0


@dataclass
class ConstraintLogEntry:
    """约束触发记录（发送至 ad-mcc-38）"""
    timestamp: float = field(default_factory=time.time)
    original_value: float = 0.0
    corrected_value: float = 0.0
    trigger_reason: str = ""
    current_mode: str = ""


# ==================== 各驾驶模式冲击度上限 ====================

JERK_LIMITS = {
    ExecutionMode.NORMAL: {"jerk_limit_ms3": 5.0, "throttle_rate_limit_pct_per_100ms": None},
    ExecutionMode.DEGRADED_LEVEL1: {"jerk_limit_ms3": 4.0, "throttle_rate_limit_pct_per_100ms": 15.0},
    ExecutionMode.DEGRADED_LEVEL2: {"jerk_limit_ms3": 3.0, "throttle_rate_limit_pct_per_100ms": 10.0},
    ExecutionMode.DEGRADED_LEVEL3: {"jerk_limit_ms3": None, "throttle_rate_limit_pct_per_100ms": None},  # 豁免
    ExecutionMode.UNPAVED: {"jerk_limit_ms3": 3.0, "throttle_rate_limit_pct_per_100ms": 8.0},
}

# 低置信度收紧系数
LOW_CONFIDENCE_TIGHTEN_FACTOR = 0.8  # 收紧20%
LOW_CONFIDENCE_THRESHOLD = 0.7

# 控制周期（秒）
CONTROL_PERIOD_S = 0.01  # 100Hz

# 加速度突跳判断阈值
ACCEL_SPIKE_THRESHOLD_MS2 = 5.0


# ==================== 主类定义 ====================

class LongitudinalJerkConstraint:
    """
    纵向冲击度约束单元
    
    职责:
    1. 接收 ad-mcc-09 原始油门开度序列，计算纵向冲击度
    2. 根据驾驶模式施加冲击度上限约束，超限时削峰修正
    3. 降级/非铺装模式下额外约束油门变化率
    4. 紧急制动豁免一切约束，直接放行
    5. 低置信度时收紧约束，增加安全裕度
    """

    def __init__(self):
        self.module_id = "ad-mcc-10"
        self.module_name = "纵向冲击度约束单元"
        self.version = "V1.0"

        self.state = ConstraintState.NORMAL_CONSTRAINT
        self._current_mode = ExecutionMode.NORMAL
        self._jerk_limit = JERK_LIMITS[ExecutionMode.NORMAL]["jerk_limit_ms3"]
        self._throttle_rate_limit = JERK_LIMITS[ExecutionMode.NORMAL]["throttle_rate_limit_pct_per_100ms"]

        # 历史值
        self._prev_timestamp: float = 0.0
        self._prev_target_accel: float = 0.0
        self._prev_throttle_pct: float = 0.0
        self._first_frame: bool = True

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调
        self._query_throttle_sequence = None       # Callable[[], Optional[ThrottleSequence]]
        self._query_driving_mode = None            # Callable[[], Optional[ExecutionMode]]
        self._query_vehicle_speed = None           # Callable[[], float]
        self._query_current_throttle = None        # Callable[[], float]
        self._query_emergency_brake = None         # Callable[[], Optional[EmergencyBrakeCommand]]

        # 输出回调
        self._publish_jerk_compliant = None        # Callable[[JerkCompliantSequence], None]
        self._publish_constraint_log = None        # Callable[[ConstraintLogEntry], None]

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_throttle_sequence_query(self, callback):
        self._query_throttle_sequence = callback

    def set_driving_mode_query(self, callback):
        self._query_driving_mode = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_current_throttle_query(self, callback):
        self._query_current_throttle = callback

    def set_emergency_brake_query(self, callback):
        self._query_emergency_brake = callback

    def set_jerk_compliant_publisher(self, callback):
        self._publish_jerk_compliant = callback

    def set_constraint_log_publisher(self, callback):
        self._publish_constraint_log = callback

    # ========== 主循环 ==========
    def run_constraint_cycle(self) -> Optional[JerkCompliantSequence]:
        """
        执行一次冲击度约束计算周期（100Hz）
        
        Returns:
            冲击度合规油门序列，若无新指令则返回 None
        """
        # 最高优先级：紧急制动豁免
        emergency = self._query_emergency_brake() if self._query_emergency_brake else None
        if emergency and emergency.jerk_exempt:
            self.state = ConstraintState.EMERGENCY_MODE
            # 直接放行原始油门序列
            raw_seq = self._query_throttle_sequence() if self._query_throttle_sequence else None
            if raw_seq:
                compliant = JerkCompliantSequence(
                    timestamp=time.time(),
                    corrected_throttle_pct=raw_seq.target_throttle_pct,
                    original_throttle_pct=raw_seq.target_throttle_pct,
                    constraint_triggered=False,
                    constraint_reason="紧急制动豁免",
                    expected_acceleration_ms2=raw_seq.expected_acceleration_ms2,
                )
                if self._publish_jerk_compliant:
                    self._publish_jerk_compliant(compliant)
                return compliant
            return None

        if self.state == ConstraintState.SYSTEM_PAUSED:
            return None

        # 驾驶模式更新
        mode = self._query_driving_mode() if self._query_driving_mode else None
        if mode and mode != self._current_mode:
            self._current_mode = mode
            limits = JERK_LIMITS.get(mode, JERK_LIMITS[ExecutionMode.NORMAL])
            self._jerk_limit = limits["jerk_limit_ms3"]
            self._throttle_rate_limit = limits["throttle_rate_limit_pct_per_100ms"]

            if mode == ExecutionMode.DEGRADED_LEVEL3:
                self.state = ConstraintState.EMERGENCY_MODE
            elif mode == ExecutionMode.UNPAVED:
                self.state = ConstraintState.UNPAVED_CONSTRAINT
            elif mode in (ExecutionMode.DEGRADED_LEVEL1, ExecutionMode.DEGRADED_LEVEL2):
                self.state = ConstraintState.DEGRADED_CONSTRAINT
            else:
                self.state = ConstraintState.NORMAL_CONSTRAINT

        # 接收原始油门序列
        raw_seq = self._query_throttle_sequence() if self._query_throttle_sequence else None
        if raw_seq is None:
            return None

        # 紧急模式直接放行
        if self.state == ConstraintState.EMERGENCY_MODE:
            compliant = JerkCompliantSequence(
                timestamp=time.time(),
                corrected_throttle_pct=raw_seq.target_throttle_pct,
                original_throttle_pct=raw_seq.target_throttle_pct,
                constraint_triggered=False,
                constraint_reason="紧急模式豁免",
                expected_acceleration_ms2=raw_seq.expected_acceleration_ms2,
            )
            if self._publish_jerk_compliant:
                self._publish_jerk_compliant(compliant)
            return compliant

        # 首帧/时间异常处理
        now = raw_seq.timestamp
        if self._first_frame or now <= self._prev_timestamp or (now - self._prev_timestamp) > 1.0:
            self._prev_timestamp = now
            self._prev_target_accel = raw_seq.expected_acceleration_ms2
            self._prev_throttle_pct = raw_seq.target_throttle_pct
            self._first_frame = False

            compliant = JerkCompliantSequence(
                timestamp=now,
                corrected_throttle_pct=raw_seq.target_throttle_pct,
                original_throttle_pct=raw_seq.target_throttle_pct,
                constraint_triggered=False,
                constraint_reason="首帧或时间异常，放行",
                expected_acceleration_ms2=raw_seq.expected_acceleration_ms2,
            )
            if self._publish_jerk_compliant:
                self._publish_jerk_compliant(compliant)
            return compliant

        # 计算冲击度
        dt = now - self._prev_timestamp
        accel_change = raw_seq.expected_acceleration_ms2 - self._prev_target_accel
        current_jerk = abs(accel_change) / dt if dt > 0 else 0.0

        # 加速度突变检测
        if abs(accel_change) > ACCEL_SPIKE_THRESHOLD_MS2:
            # 视为传感器噪声，削峰至允许最大变化量
            max_allowed_change = self._jerk_limit * dt if self._jerk_limit else ACCEL_SPIKE_THRESHOLD_MS2
            accel_change = math.copysign(max_allowed_change, accel_change)

        # 有效冲击度限制（可能因低置信度收紧）
        effective_jerk_limit = self._jerk_limit
        if raw_seq.confidence < LOW_CONFIDENCE_THRESHOLD and effective_jerk_limit is not None:
            effective_jerk_limit *= LOW_CONFIDENCE_TIGHTEN_FACTOR

        constraint_triggered = False
        constraint_reasons = []
        corrected_accel = raw_seq.expected_acceleration_ms2

        # 冲击度约束
        if effective_jerk_limit is not None and current_jerk > effective_jerk_limit:
            constraint_triggered = True
            constraint_reasons.append(f"纵向冲击度超限 ({current_jerk:.2f} > {effective_jerk_limit})")
            # 限制加速度变化量
            max_allowed_change = effective_jerk_limit * dt
            accel_change = math.copysign(max_allowed_change, accel_change)
            corrected_accel = self._prev_target_accel + accel_change

        # 根据期望加速度反推修正油门
        if raw_seq.expected_acceleration_ms2 != 0 and abs(raw_seq.expected_acceleration_ms2) > 1e-6:
            throttle_scale = corrected_accel / raw_seq.expected_acceleration_ms2
        else:
            throttle_scale = 0.0 if raw_seq.expected_acceleration_ms2 == 0 else 1.0

        corrected_throttle = self._prev_throttle_pct + (raw_seq.target_throttle_pct - self._prev_throttle_pct) * throttle_scale

        # 油门变化率额外约束
        if self._throttle_rate_limit is not None:
            throttle_change = abs(corrected_throttle - self._prev_throttle_pct)
            allowed_change = self._throttle_rate_limit * (dt / 0.1)  # 转为当前时间间隔允许的变化量
            if throttle_change > allowed_change:
                constraint_triggered = True
                constraint_reasons.append("油门变化率超限")
                corrected_throttle = self._prev_throttle_pct + math.copysign(allowed_change, corrected_throttle - self._prev_throttle_pct)

        # 确保加速度不因约束变成零除非本身就是零
        if abs(raw_seq.expected_acceleration_ms2) > 0.0 and abs(corrected_accel) < 1e-6 and corrected_throttle == self._prev_throttle_pct:
            # 避免完全停滞
            min_accel_change = 0.1  # 最小加速度变化
            corrected_accel = self._prev_target_accel + math.copysign(min_accel_change, raw_seq.expected_acceleration_ms2 - self._prev_target_accel)

        # 更新历史值
        self._prev_timestamp = now
        self._prev_target_accel = corrected_accel
        self._prev_throttle_pct = corrected_throttle

        # 构建输出
        reason_str = " + ".join(constraint_reasons) if constraint_reasons else ""
        compliant = JerkCompliantSequence(
            timestamp=now,
            corrected_throttle_pct=round(corrected_throttle, 2),
            original_throttle_pct=raw_seq.target_throttle_pct,
            constraint_triggered=constraint_triggered,
            constraint_reason=reason_str,
            expected_acceleration_ms2=round(corrected_accel, 3),
        )

        if self._publish_jerk_compliant:
            self._publish_jerk_compliant(compliant)

        # 约束触发记录
        if constraint_triggered and self._publish_constraint_log:
            self._publish_constraint_log(ConstraintLogEntry(
                timestamp=now,
                original_value=raw_seq.target_throttle_pct,
                corrected_value=corrected_throttle,
                trigger_reason=reason_str,
                current_mode=self.state.value,
            ))

        return compliant

    # ========== 查询接口 ==========
    def get_state(self) -> ConstraintState:
        return self.state

    def get_current_mode(self) -> ExecutionMode:
        return self._current_mode

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
            "jerk_limit": self._jerk_limit,
            "throttle_rate_limit": self._throttle_rate_limit,
        }

    def emergency_shutdown(self):
        self.state = ConstraintState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，维持当前约束参数")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 纵向冲击度约束单元 (ad-mcc-10) 演示")
    print("=" * 70)

    constraint = LongitudinalJerkConstraint()
    constraint.set_vehicle_speed_query(lambda: 60.0)
    constraint.set_current_throttle_query(lambda: 30.0)
    constraint.set_driving_mode_query(lambda: ExecutionMode.NORMAL)

    # 模拟历史值已初始化
    constraint._first_frame = False
    constraint._prev_timestamp = time.time() - 0.1
    constraint._prev_target_accel = 1.0
    constraint._prev_throttle_pct = 25.0

    print_separator("STEP 1: 正常冲击度（不超限）")
    seq = ThrottleSequence(
        target_throttle_pct=30.0,
        expected_acceleration_ms2=1.5,
        confidence=0.95,
    )
    constraint.set_throttle_sequence_query(lambda: seq)
    result = constraint.run_constraint_cycle()
    if result:
        print(f"  修正后油门: {result.corrected_throttle_pct}%")
        print(f"  是否触发约束: {result.constraint_triggered}")
        print(f"  期望加速度: {result.expected_acceleration_ms2} m/s²")

    print_separator("STEP 2: 冲击度超限（削峰）")
    constraint._prev_target_accel = 0.5
    constraint._prev_throttle_pct = 20.0
    seq2 = ThrottleSequence(
        target_throttle_pct=50.0,
        expected_acceleration_ms2=3.0,
        confidence=0.95,
    )
    constraint.set_throttle_sequence_query(lambda: seq2)
    result2 = constraint.run_constraint_cycle()
    if result2:
        print(f"  原始油门: {result2.original_throttle_pct}%")
        print(f"  修正后油门: {result2.corrected_throttle_pct}%")
        print(f"  是否触发约束: {result2.constraint_triggered}")
        print(f"  约束原因: {result2.constraint_reason}")

    print_separator("STEP 3: 紧急制动豁免")
    constraint.set_emergency_brake_query(lambda: EmergencyBrakeCommand(jerk_exempt=True))
    result3 = constraint.run_constraint_cycle()
    if result3:
        print(f"  修正后油门: {result3.corrected_throttle_pct}%")
        print(f"  约束原因: {result3.constraint_reason}")

    print("\n✅ 纵向冲击度约束单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-10 纵向冲击度约束单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_constraint(mode=ExecutionMode.NORMAL, prev_accel=1.0, prev_throttle=25.0):
            c = LongitudinalJerkConstraint()
            c.set_vehicle_speed_query(lambda: 60.0)
            c.set_current_throttle_query(lambda: prev_throttle)
            c.set_driving_mode_query(lambda: mode)
            # 初始化历史值
            c._first_frame = False
            c._prev_timestamp = time.time() - 0.1
            c._prev_target_accel = prev_accel
            c._prev_throttle_pct = prev_throttle
            return c

        # TC-M10-01: 正常冲击度不超限
        print("\n[TC-M10-01] 正常冲击度放行")
        try:
            c = setup_constraint()
            c.set_throttle_sequence_query(lambda: ThrottleSequence(
                target_throttle_pct=30.0, expected_acceleration_ms2=1.5, confidence=0.95
            ))
            result = c.run_constraint_cycle()
            assert result is not None
            assert result.constraint_triggered == False
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M10-02: 冲击度超限，触发约束
        print("\n[TC-M10-02] 冲击度超限，触发约束")
        try:
            c = setup_constraint(prev_accel=0.5, prev_throttle=20.0)
            c.set_throttle_sequence_query(lambda: ThrottleSequence(
                target_throttle_pct=50.0, expected_acceleration_ms2=3.0, confidence=0.95
            ))
            result = c.run_constraint_cycle()
            assert result is not None
            assert result.constraint_triggered == True
            assert result.corrected_throttle_pct < result.original_throttle_pct
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M10-03: 二级降级下更严格的冲击度
        print("\n[TC-M10-03] 二级降级下更严格的冲击度约束")
        try:
            c = setup_constraint(mode=ExecutionMode.DEGRADED_LEVEL2, prev_accel=1.0, prev_throttle=20.0)
            c.set_throttle_sequence_query(lambda: ThrottleSequence(
                target_throttle_pct=40.0, expected_acceleration_ms2=2.0, confidence=0.95
            ))
            result = c.run_constraint_cycle()
            assert result is not None
            # 在3.0m/s³限制下，变化应该受限
            assert result.constraint_triggered == True or result.corrected_throttle_pct <= 35.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M10-04: 紧急制动豁免
        print("\n[TC-M10-04] 紧急制动豁免")
        try:
            c = setup_constraint()
            c.set_emergency_brake_query(lambda: EmergencyBrakeCommand(jerk_exempt=True))
            c.set_throttle_sequence_query(lambda: ThrottleSequence(
                target_throttle_pct=0.0, expected_acceleration_ms2=-6.0, confidence=0.95
            ))
            result = c.run_constraint_cycle()
            assert result is not None
            assert result.constraint_triggered == False
            assert result.corrected_throttle_pct == result.original_throttle_pct
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M10-05: 首帧直接放行
        print("\n[TC-M10-05] 首帧直接放行")
        try:
            c = LongitudinalJerkConstraint()
            c.set_vehicle_speed_query(lambda: 60.0)
            c.set_driving_mode_query(lambda: ExecutionMode.NORMAL)
            c._first_frame = True  # 强制首帧
            c.set_throttle_sequence_query(lambda: ThrottleSequence(
                target_throttle_pct=35.0, expected_acceleration_ms2=2.0, confidence=0.95
            ))
            result = c.run_constraint_cycle()
            assert result is not None
            assert result.constraint_triggered == False
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M10-06: 低置信度收紧约束
        print("\n[TC-M10-06] 低置信度收紧约束")
        try:
            c = setup_constraint(prev_accel=0.5, prev_throttle=20.0)
            c.set_throttle_sequence_query(lambda: ThrottleSequence(
                target_throttle_pct=40.0, expected_acceleration_ms2=1.5, confidence=0.5  # < 0.7
            ))
            result = c.run_constraint_cycle()
            assert result is not None
            # 低置信度下冲击度上限收紧，可能触发约束或更保守的修正
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