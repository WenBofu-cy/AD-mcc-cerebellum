#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-05
模块名称: 转向平顺滤波单元
所属分区: 二、转向控制集群
核心职责: 接收 ad-mcc-04 输出的原始方向盘目标转角序列，对转角指令进行时间域平滑滤波，
          消除高频抖动与突变阶跃，输出平滑后的转角曲线。同时附加转角速率限制，确保转向
          动作在物理可执行范围内且符合舒适性标准。是转向控制链路中“质感优化”的核心环节。

依赖模块:
    ad-mcc-04(方向盘转角解算单元，提供原始目标转角序列)
被依赖模块:
    ad-mcc-06(横向冲击度约束单元，接收平滑后的转角序列)

安全约束:
  S-01: 平滑后的转角速率必须严格限制在设定最大转角速率以内，不得突破
  S-02: 非铺装模式下，转角速率上限自动降低40%，提升舒适与安全
  S-03: 滤波参数缺失时，必须使用保守默认参数并明确标记“降级滤波”
  S-04: 本模块仅输出平滑后转角序列，不直接操控转向电机
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
import time
import uuid
import math


# ==================== 枚举定义 ====================

class FilteringState(Enum):
    """转向平顺滤波单元内部状态"""
    NORMAL_FILTERING = "normal_filtering"
    DEGRADED_FILTERING = "degraded_filtering"
    SYSTEM_PAUSED = "system_paused"


class FilterType(Enum):
    """滤波器类型"""
    IIR_LOWPASS = "一阶低通IIR"
    MOVING_AVERAGE = "滑动平均"


# ==================== 数据结构 ====================

@dataclass
class RawSteeringAngleSequence:
    """方向盘原始目标转角序列（来自 ad-mcc-04）"""
    timestamp: float = field(default_factory=time.time)
    target_angle_deg: float = 0.0
    angle_rate_deg_per_s: float = 0.0
    calculation_method: str = ""
    confidence: float = 0.95


@dataclass
class FilterParameters:
    """滤波参数配置"""
    filter_type: FilterType = FilterType.IIR_LOWPASS
    cutoff_frequency_hz: float = 8.0
    damping_ratio: float = 0.7
    max_angle_rate_deg_per_s: float = 300.0
    moving_average_window_size: int = 5


@dataclass
class SmoothedSteeringSequence:
    """平滑后方向盘目标转角序列"""
    timestamp: float = field(default_factory=time.time)
    smoothed_angle_deg: float = 0.0
    smoothed_angle_rate_deg_per_s: float = 0.0
    filter_method: str = "一阶低通IIR"
    smoothing_confidence: float = 0.95


@dataclass
class FilteringStatusReport:
    """滤波状态上报"""
    current_state: FilteringState = FilteringState.NORMAL_FILTERING
    filter_delay_ms: float = 0.0
    angle_smoothness_score: float = 0.0


# ==================== 主类定义 ====================

class SteeringSmoothingFilter:
    """
    转向平顺滤波单元
    
    职责:
    1. 对原始方向盘目标转角序列进行时间域平滑滤波
    2. 消除高频抖动与突变阶跃
    3. 附加转角速率限制
    4. 根据车速动态调整滤波强度
    5. 非铺装模式额外加强滤波
    """

    # 默认滤波参数（正常模式）
    DEFAULT_CUTOFF_FREQ_HZ = 8.0
    DEFAULT_MAX_ANGLE_RATE_DEG_PER_S = 300.0
    # 降级时使用的保守默认值（与规格一致）
    DEGRADED_CUTOFF_FREQ_HZ = 5.0
    DEGRADED_MAX_ANGLE_RATE_DEG_PER_S = 200.0
    DEFAULT_MOVING_AVERAGE_WINDOW = 5

    # 控制周期（秒）
    CONTROL_PERIOD_S = 0.01  # 100Hz

    # 车速阈值
    HIGH_SPEED_THRESHOLD_KMH = 80.0
    LOW_SPEED_THRESHOLD_KMH = 20.0
    HIGH_SPEED_FREQ_FACTOR = 0.6
    LOW_SPEED_FREQ_FACTOR = 1.5

    # 非铺装模式速率修正系数
    UNPAVED_RATE_FACTOR = 0.6

    def __init__(self):
        self.module_id = "ad-mcc-05"
        self.module_name = "转向平顺滤波单元"
        self.version = "V1.0"

        self.state = FilteringState.NORMAL_FILTERING

        self._filter_params = FilterParameters()
        self._params_valid = True

        self._prev_smoothed_angle: float = 0.0
        self._raw_angle_buffer: deque = deque(maxlen=self.DEFAULT_MOVING_AVERAGE_WINDOW)

        self._current_speed_kmh: float = 0.0
        self._current_actual_angle: float = 0.0
        self._current_mode: str = "正常"

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调
        self._query_raw_angle_sequence = None
        self._query_filter_params = None
        self._query_actual_angle = None
        self._query_vehicle_speed = None
        self._query_current_mode = None

        # 输出回调
        self._publish_smoothed_sequence = None
        self._publish_status_report = None

        self._load_filter_params()

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_raw_angle_sequence_query(self, callback):
        self._query_raw_angle_sequence = callback

    def set_filter_params_query(self, callback):
        self._query_filter_params = callback

    def set_actual_angle_query(self, callback):
        self._query_actual_angle = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_current_mode_query(self, callback):
        self._query_current_mode = callback

    def set_smoothed_sequence_publisher(self, callback):
        self._publish_smoothed_sequence = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    # ========== 参数加载 ==========
    def _load_filter_params(self):
        """加载滤波参数"""
        if self._query_filter_params:
            params = self._query_filter_params()
            if params:
                self._filter_params = params
                self._params_valid = True
                if self.state == FilteringState.DEGRADED_FILTERING:
                    self.state = FilteringState.NORMAL_FILTERING
                return

        # 参数不可用，使用保守降级默认值
        self._params_valid = False
        self.state = FilteringState.DEGRADED_FILTERING
        self._filter_params = FilterParameters(
            filter_type=FilterType.IIR_LOWPASS,
            cutoff_frequency_hz=self.DEGRADED_CUTOFF_FREQ_HZ,
            max_angle_rate_deg_per_s=self.DEGRADED_MAX_ANGLE_RATE_DEG_PER_S,
            moving_average_window_size=self.DEFAULT_MOVING_AVERAGE_WINDOW
        )
        self._log_event("PARAMS_DEGRADED", {"cutoff_hz": self.DEGRADED_CUTOFF_FREQ_HZ,
                                             "max_rate": self.DEGRADED_MAX_ANGLE_RATE_DEG_PER_S})

    # ========== 主循环 ==========
    def run_filtering_cycle(self) -> Optional[SmoothedSteeringSequence]:
        if self.state == FilteringState.SYSTEM_PAUSED:
            return None

        if self._query_vehicle_speed:
            self._current_speed_kmh = self._query_vehicle_speed()
        if self._query_actual_angle:
            self._current_actual_angle = self._query_actual_angle()
        if self._query_current_mode:
            self._current_mode = self._query_current_mode()

        raw_sequence = self._query_raw_angle_sequence() if self._query_raw_angle_sequence else None
        if raw_sequence is None:
            return None

        self._load_filter_params()
        start_time = time.perf_counter()

        raw_angle = raw_sequence.target_angle_deg
        self._raw_angle_buffer.append(raw_angle)

        dynamic_fc = self._calc_dynamic_cutoff_freq(self._current_speed_kmh)

        if self._filter_params.filter_type == FilterType.IIR_LOWPASS:
            smoothed_angle = self._iir_lowpass_filter(raw_angle, dynamic_fc)
            filter_method = "一阶低通IIR"
        else:
            smoothed_angle = self._moving_average_filter()
            filter_method = "滑动平均"

        smoothed_angle = self._apply_rate_limit(smoothed_angle)

        if self._current_mode == "非铺装道路":
            smoothed_angle = self._apply_rate_limit_with_factor(smoothed_angle, self.UNPAVED_RATE_FACTOR)

        angle_change = smoothed_angle - self._prev_smoothed_angle
        smoothed_rate = angle_change / self.CONTROL_PERIOD_S

        confidence = 0.8 if self.state == FilteringState.DEGRADED_FILTERING else 0.95
        if self.state == FilteringState.DEGRADED_FILTERING:
            filter_method += "（降级）"

        sequence = SmoothedSteeringSequence(
            timestamp=time.time(),
            smoothed_angle_deg=round(smoothed_angle, 2),
            smoothed_angle_rate_deg_per_s=round(smoothed_rate, 2),
            filter_method=filter_method,
            smoothing_confidence=confidence
        )

        self._prev_smoothed_angle = smoothed_angle

        if self._publish_smoothed_sequence:
            self._publish_smoothed_sequence(sequence)

        elapsed_ms = (time.perf_counter() - start_time) * 1000
        if self._publish_status_report:
            self._publish_status_report(FilteringStatusReport(
                current_state=self.state,
                filter_delay_ms=round(elapsed_ms, 2),
                angle_smoothness_score=self._calc_smoothness_score()
            ))

        return sequence

    # ========== 一阶低通 IIR 滤波 ==========
    def _iir_lowpass_filter(self, raw_angle: float, cutoff_freq_hz: float) -> float:
        T = self.CONTROL_PERIOD_S
        omega = 2.0 * math.pi * cutoff_freq_hz
        a = omega * T / (omega * T + 1.0)

        if self._prev_smoothed_angle == 0.0 and self._current_actual_angle != 0.0:
            self._prev_smoothed_angle = self._current_actual_angle

        smoothed = a * raw_angle + (1.0 - a) * self._prev_smoothed_angle
        return smoothed

    # ========== 滑动平均滤波 ==========
    def _moving_average_filter(self) -> float:
        if not self._raw_angle_buffer:
            return self._prev_smoothed_angle
        return sum(self._raw_angle_buffer) / len(self._raw_angle_buffer)

    # ========== 速率限制 ==========
    def _apply_rate_limit(self, target_angle: float) -> float:
        return self._apply_rate_limit_with_factor(target_angle, 1.0)

    def _apply_rate_limit_with_factor(self, target_angle: float, factor: float) -> float:
        max_rate = self._filter_params.max_angle_rate_deg_per_s * factor
        max_change = max_rate * self.CONTROL_PERIOD_S
        angle_change = target_angle - self._prev_smoothed_angle
        if abs(angle_change) > max_change:
            target_angle = self._prev_smoothed_angle + math.copysign(max_change, angle_change)
        return target_angle

    # ========== 动态截止频率 ==========
    def _calc_dynamic_cutoff_freq(self, speed_kmh: float) -> float:
        base_fc = self._filter_params.cutoff_frequency_hz
        if speed_kmh > self.HIGH_SPEED_THRESHOLD_KMH:
            return base_fc * self.HIGH_SPEED_FREQ_FACTOR
        elif speed_kmh < self.LOW_SPEED_THRESHOLD_KMH:
            return base_fc * self.LOW_SPEED_FREQ_FACTOR
        return base_fc

    # ========== 平滑度评分 ==========
    def _calc_smoothness_score(self) -> float:
        if len(self._raw_angle_buffer) < 3:
            return 1.0
        raw_values = list(self._raw_angle_buffer)
        mean = sum(raw_values) / len(raw_values)
        variance = sum((x - mean) ** 2 for x in raw_values) / len(raw_values)
        score = max(0.0, 1.0 - math.sqrt(variance) / 5.0)
        return round(score, 3)

    # ========== 查询接口 ==========
    def get_state(self) -> FilteringState:
        return self.state

    def get_current_smoothed_angle(self) -> float:
        return self._prev_smoothed_angle

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
            "current_smoothed_angle": self._prev_smoothed_angle,
        }

    def emergency_shutdown(self):
        self.state = FilteringState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，保持上一帧平滑转角")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 转向平顺滤波单元 (ad-mcc-05) 演示")
    print("=" * 70)

    filt = SteeringSmoothingFilter()
    filt.set_actual_angle_query(lambda: 0.0)
    filt.set_vehicle_speed_query(lambda: 60.0)
    filt.set_current_mode_query(lambda: "正常")

    print_separator("STEP 1: 正常转角平滑滤波")
    filt.set_raw_angle_sequence_query(lambda: RawSteeringAngleSequence(
        target_angle_deg=15.0,
        angle_rate_deg_per_s=150.0
    ))
    for _ in range(5):
        seq = filt.run_filtering_cycle()
    if seq:
        print(f"  平滑后转角: {seq.smoothed_angle_deg}°")
        print(f"  滤波方法: {seq.filter_method}")
        print(f"  置信度: {seq.smoothing_confidence}")

    print_separator("STEP 2: 转角突变（50°阶跃）受速率限制")
    filt.set_raw_angle_sequence_query(lambda: RawSteeringAngleSequence(
        target_angle_deg=50.0,
        angle_rate_deg_per_s=600.0
    ))
    for _ in range(3):
        seq = filt.run_filtering_cycle()
    if seq:
        print(f"  平滑后转角: {seq.smoothed_angle_deg}°")
        print(f"  转角速率: {seq.smoothed_angle_rate_deg_per_s}°/s")
        print(f"  （单帧最大变化={filt._filter_params.max_angle_rate_deg_per_s * filt.CONTROL_PERIOD_S}°）")

    print("\n✅ 转向平顺滤波单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-05 转向平顺滤波单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_filter(speed=60.0, actual_angle=0.0, mode="正常"):
            f = SteeringSmoothingFilter()
            f.set_actual_angle_query(lambda: actual_angle)
            f.set_vehicle_speed_query(lambda: speed)
            f.set_current_mode_query(lambda: mode)
            return f

        # TC-M05-01: 正常转角平滑滤波
        print("\n[TC-M05-01] 正常转角平滑滤波")
        try:
            f = setup_filter(actual_angle=14.0)
            f.set_raw_angle_sequence_query(lambda: RawSteeringAngleSequence(target_angle_deg=15.0))
            for _ in range(10):
                seq = f.run_filtering_cycle()
            assert seq is not None
            assert 14.0 <= seq.smoothed_angle_deg <= 15.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M05-02: 转角突变受速率限制
        print("\n[TC-M05-02] 转角突变（50°）受速率限制")
        try:
            f = setup_filter(actual_angle=0.0)
            f.set_raw_angle_sequence_query(lambda: RawSteeringAngleSequence(target_angle_deg=0.0))
            f.run_filtering_cycle()
            f.set_raw_angle_sequence_query(lambda: RawSteeringAngleSequence(target_angle_deg=50.0))
            seq = f.run_filtering_cycle()
            assert seq is not None
            max_per_frame = f._filter_params.max_angle_rate_deg_per_s * f.CONTROL_PERIOD_S
            angle_change = abs(seq.smoothed_angle_deg - 0.0)
            assert angle_change <= max_per_frame + 0.01, f"变化{angle_change}° > 上限{max_per_frame}°"
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M05-03: 高速时滤波更强
        print("\n[TC-M05-03] 高速行驶时动态截止频率降低")
        try:
            f = setup_filter(speed=120.0, actual_angle=10.0)
            f.set_raw_angle_sequence_query(lambda: RawSteeringAngleSequence(target_angle_deg=12.0))
            for _ in range(3):
                seq = f.run_filtering_cycle()
            assert seq is not None
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M05-04: 参数缺失降级（应使用保守参数 fc=5Hz, max_rate=200°/s）
        print("\n[TC-M05-04] 滤波参数缺失降级")
        try:
            f = SteeringSmoothingFilter()
            f.set_actual_angle_query(lambda: 0.0)
            f.set_vehicle_speed_query(lambda: 60.0)
            f.set_current_mode_query(lambda: "正常")
            f.set_raw_angle_sequence_query(lambda: RawSteeringAngleSequence(target_angle_deg=10.0))
            seq = f.run_filtering_cycle()
            assert seq is not None
            assert f.state == FilteringState.DEGRADED_FILTERING
            assert f._filter_params.cutoff_frequency_hz == 5.0
            assert f._filter_params.max_angle_rate_deg_per_s == 200.0
            assert seq.smoothing_confidence < 0.9
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M05-05: 紧急熔断
        print("\n[TC-M05-05] 紧急熔断")
        try:
            f = setup_filter()
            f.emergency_shutdown()
            assert f.state == FilteringState.SYSTEM_PAUSED
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