#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-11
模块名称: 加速平顺滤波单元
所属分区: 三、动力控制集群
核心职责: 接收 ad-mcc-10 输出的冲击度合规油门开度序列，对油门指令进行低通滤波与平滑处理，
          消除残余的高频波动与微幅抖动，确保油门开度变化曲线连续、圆滑、无阶跃感。
          输出最终的油门踏板目标开度指令至车辆底层驱动接口（或 ad-mcc-12 供偏差监控）。
          不参与任何场景判断与驾驶决策。

依赖模块:
    ad-mcc-10(纵向冲击度约束单元，提供冲击度合规油门序列),
    ad-mcc-01(小脑总控调度核心，下发驾驶模式信号)
被依赖模块:
    ad-mcc-12(动力执行偏差监控单元，接收最终油门指令作为目标值),
    车辆底层油门执行器接口

安全约束:
  S-01: 紧急冻结指令为最高优先级，一旦激活，必须锁定当前油门或指定值，严禁响应任何新油门指令
  S-02: 滤波后的油门开度必须严格在 [0, 100] 范围内，不得输出负值或超限值
  S-03: 滤波算法不得引入超过 100ms 的延迟（从输入到输出），确保油门响应及时性
  S-04: 在三级降级或碰撞后，油门必须归零或锁定为 0，不允许滤波算法输出非零值
  S-05: 本模块仅负责油门信号的平滑处理，不改变驾驶意图的方向（加速/减速）
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math
from collections import deque


# ==================== 枚举定义 ====================

class FilterState(Enum):
    """加速平顺滤波单元内部状态"""
    NORMAL_FILTER = "normal_filter"
    SOFT_FILTER = "soft_filter"
    FAST_RESPONSE = "fast_response"
    HOLD = "hold"
    SYSTEM_PAUSED = "system_paused"


class ExecutionMode(Enum):
    """驾驶模式（与 ad-mcc-01 对齐）"""
    NORMAL = "正常"
    DEGRADED_LEVEL1 = "一级降级"
    DEGRADED_LEVEL2 = "二级降级"
    DEGRADED_LEVEL3 = "三级降级"
    UNPAVED = "非铺装道路"


class DrivingStyle(Enum):
    """用户驾驶风格"""
    COMFORT = "平顺舒适"
    STANDARD = "标准通勤"
    EFFICIENT = "高效通行"


# ==================== 数据结构 ====================

@dataclass
class JerkCompliantSequence:
    """冲击度合规油门序列（来自 ad-mcc-10）"""
    timestamp: float = field(default_factory=time.time)
    corrected_throttle_pct: float = 0.0
    original_throttle_pct: float = 0.0
    constraint_triggered: bool = False
    constraint_reason: str = ""
    expected_acceleration_ms2: float = 0.0


@dataclass
class EmergencyFreezeCommand:
    """紧急状态冻结指令"""
    msg_id: str = ""
    freeze_active: bool = True
    target_throttle_lock_pct: Optional[float] = None  # None 表示保持当前滤波值
    reason: str = ""


@dataclass
class FinalThrottleCommand:
    """最终油门目标指令（发送至 ad-mcc-12 及底层执行器）"""
    timestamp: float = field(default_factory=time.time)
    filtered_throttle_pct: float = 0.0
    original_throttle_pct: float = 0.0
    filter_method: str = ""
    filter_alpha: float = 0.0


@dataclass
class FilterStatusReport:
    """滤波状态上报（发送至 ad-mcc-01）"""
    current_filter_mode: str = ""
    current_cutoff_freq_hz: float = 0.0
    current_alpha: float = 0.0
    filter_delay_ms: float = 0.0


# ==================== 滤波参数表 ====================

# 结构: (alpha, max_frame_change_pct, filter_mode_name)
FILTER_CONFIGS = {
    (ExecutionMode.NORMAL, DrivingStyle.STANDARD): (0.2, None, "一阶低通 (标准)"),
    (ExecutionMode.NORMAL, DrivingStyle.COMFORT): (0.35, 3.0, "一阶低通 (舒适)"),
    (ExecutionMode.NORMAL, DrivingStyle.EFFICIENT): (0.1, None, "一阶低通 (高效)"),
    (ExecutionMode.DEGRADED_LEVEL1, None): (0.3, 4.0, "一阶低通 (一级降级)"),
    (ExecutionMode.DEGRADED_LEVEL2, None): (0.4, 2.5, "一阶低通 (二级降级)"),
    (ExecutionMode.UNPAVED, None): (0.5, 2.0, "一阶低通+移动平均 (非铺装)"),
}

# 非铺装模式移动平均窗口大小
UNPAVED_MA_WINDOW = 3
# 控制周期
CONTROL_PERIOD_S = 0.01
# 最大滤波延迟上限 (秒)
MAX_FILTER_DELAY_S = 0.1
# 油门指令超时衰减率 (每秒衰减 5%)
THROTTLE_DECAY_RATE_PER_S = 5.0


# ==================== 主类定义 ====================

class ThrottleSmoothFilter:
    """
    加速平顺滤波单元
    
    职责:
    1. 一阶低通滤波消除油门高频抖动
    2. 根据驾驶模式和用户风格动态调整滤波强度
    3. 非铺装模式下启用移动平均
    4. 紧急冻结时锁定油门输出（包括三级降级强制归零）
    """

    def __init__(self):
        self.module_id = "ad-mcc-11"
        self.module_name = "加速平顺滤波单元"
        self.version = "V1.0"

        self.state = FilterState.NORMAL_FILTER
        self._current_mode = ExecutionMode.NORMAL
        self._current_style = DrivingStyle.STANDARD

        # 滤波参数
        self._alpha = 0.2
        self._max_frame_change_pct = None
        self._filter_mode_name = "一阶低通 (标准)"

        # 历史值
        self._prev_filtered = 0.0
        # 非铺装窗口
        self._raw_window = deque(maxlen=UNPAVED_MA_WINDOW)
        # 冻结值
        self._frozen_value = 0.0
        # 输入超时计时
        self._last_input_time = time.time()
        # 状态上报计时
        self._last_status_report_time = time.time()

        self._pending_logs: List[Dict[str, Any]] = []

        # 回调注入
        self._query_jerk_compliant_seq = None     # Callable[[], Optional[JerkCompliantSequence]]
        self._query_driving_mode = None           # Callable[[], ExecutionMode]
        self._query_driving_style = None          # Callable[[], DrivingStyle]
        self._query_emergency_freeze = None       # Callable[[], Optional[EmergencyFreezeCommand]]

        self._publish_final_throttle = None       # Callable[[FinalThrottleCommand], None]
        self._publish_status_report = None        # Callable[[FilterStatusReport], None]

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_jerk_compliant_query(self, callback):
        self._query_jerk_compliant_seq = callback

    def set_driving_mode_query(self, callback):
        self._query_driving_mode = callback

    def set_driving_style_query(self, callback):
        self._query_driving_style = callback

    def set_emergency_freeze_query(self, callback):
        self._query_emergency_freeze = callback

    def set_final_throttle_publisher(self, callback):
        self._publish_final_throttle = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    # ========== 主循环 ==========
    def run_filter_cycle(self) -> Optional[FinalThrottleCommand]:
        """
        执行一次油门滤波周期（100Hz）
        """
        now = time.time()

        # 紧急熔断
        if self.state == FilterState.SYSTEM_PAUSED:
            return None

        # 处理冻结状态（紧急冻结或三级降级）
        if self.state == FilterState.HOLD:
            output = FinalThrottleCommand(
                timestamp=now,
                filtered_throttle_pct=self._frozen_value,
                original_throttle_pct=self._frozen_value,
                filter_method="冻结保持",
                filter_alpha=0.0
            )
            if self._publish_final_throttle:
                self._publish_final_throttle(output)
            return output

        # 接收紧急冻结指令，切换至 HOLD 状态
        emergency = self._query_emergency_freeze() if self._query_emergency_freeze else None
        if emergency and emergency.freeze_active:
            self.state = FilterState.HOLD
            if emergency.target_throttle_lock_pct is not None:
                self._frozen_value = emergency.target_throttle_lock_pct
            else:
                self._frozen_value = self._prev_filtered  # 保持当前值
            # 立即输出冻结值
            output = FinalThrottleCommand(
                timestamp=now,
                filtered_throttle_pct=self._frozen_value,
                original_throttle_pct=self._frozen_value,
                filter_method="冻结保持",
                filter_alpha=0.0
            )
            if self._publish_final_throttle:
                self._publish_final_throttle(output)
            return output

        # 更新驾驶模式与风格
        mode = self._query_driving_mode() if self._query_driving_mode else ExecutionMode.NORMAL
        style = self._query_driving_style() if self._query_driving_style else DrivingStyle.STANDARD

        if mode != self._current_mode or style != self._current_style:
            self._current_mode = mode
            self._current_style = style
            self._update_filter_params()
            # 如果参数更新后进入 HOLD（如三级降级），本帧即输出冻结值
            if self.state == FilterState.HOLD:
                output = FinalThrottleCommand(
                    timestamp=now,
                    filtered_throttle_pct=self._frozen_value,
                    original_throttle_pct=self._frozen_value,
                    filter_method="冻结保持",
                    filter_alpha=0.0
                )
                if self._publish_final_throttle:
                    self._publish_final_throttle(output)
                return output

        # 接收冲击度合规序列
        raw_seq = self._query_jerk_compliant_seq() if self._query_jerk_compliant_seq else None
        if raw_seq is None:
            # 输入超时处理
            if now - self._last_input_time > 0.2:
                decay_amount = THROTTLE_DECAY_RATE_PER_S * CONTROL_PERIOD_S
                self._prev_filtered = max(0.0, self._prev_filtered - decay_amount)
                output = FinalThrottleCommand(
                    timestamp=now,
                    filtered_throttle_pct=self._prev_filtered,
                    original_throttle_pct=self._prev_filtered,
                    filter_method="输入超时衰减",
                    filter_alpha=0.0
                )
                if self._publish_final_throttle:
                    self._publish_final_throttle(output)
                return output
            # 无新输入但未超时，不输出
            return None

        self._last_input_time = now
        raw_throttle = raw_seq.corrected_throttle_pct
        original_throttle = raw_seq.original_throttle_pct

        # 非铺装模式：更新移动平均窗口
        if self._current_mode == ExecutionMode.UNPAVED:
            self._raw_window.append(raw_throttle)
            if len(self._raw_window) == UNPAVED_MA_WINDOW:
                ma_value = sum(self._raw_window) / UNPAVED_MA_WINDOW
                filtered = self._alpha * ma_value + (1 - self._alpha) * self._prev_filtered
            else:
                filtered = self._alpha * raw_throttle + (1 - self._alpha) * self._prev_filtered
        else:
            # 标准一阶低通滤波
            filtered = self._alpha * raw_throttle + (1 - self._alpha) * self._prev_filtered

        # 帧间变化率限制
        if self._max_frame_change_pct is not None:
            change = filtered - self._prev_filtered
            if abs(change) > self._max_frame_change_pct:
                filtered = self._prev_filtered + math.copysign(self._max_frame_change_pct, change)

        # 边界裁剪
        filtered = max(0.0, min(100.0, filtered))

        # 更新历史值
        self._prev_filtered = filtered

        # 输出
        output = FinalThrottleCommand(
            timestamp=now,
            filtered_throttle_pct=round(filtered, 2),
            original_throttle_pct=original_throttle,
            filter_method=self._filter_mode_name,
            filter_alpha=self._alpha
        )

        if self._publish_final_throttle:
            self._publish_final_throttle(output)

        # 周期性状态上报
        if now - self._last_status_report_time >= 1.0:
            self._last_status_report_time = now
            if self._publish_status_report:
                cutoff_freq = self._alpha / (2 * math.pi * CONTROL_PERIOD_S)
                report = FilterStatusReport(
                    current_filter_mode=self._filter_mode_name,
                    current_cutoff_freq_hz=cutoff_freq,
                    current_alpha=self._alpha,
                    filter_delay_ms=self._alpha * 1000  # 一阶滤波延迟近似
                )
                self._publish_status_report(report)

        return output

    def _update_filter_params(self):
        """根据当前模式和风格更新滤波参数"""
        # 三级降级：强制归零，状态设为 HOLD
        if self._current_mode == ExecutionMode.DEGRADED_LEVEL3:
            self.state = FilterState.HOLD
            self._frozen_value = 0.0
            self._alpha = 0.0
            self._max_frame_change_pct = None
            self._filter_mode_name = "冻结保持 (三级降级)"
            return

        # 查找参数配置
        config = None
        if self._current_mode in [ExecutionMode.DEGRADED_LEVEL1, ExecutionMode.DEGRADED_LEVEL2, ExecutionMode.UNPAVED]:
            config = FILTER_CONFIGS.get((self._current_mode, None))
        else:
            config = FILTER_CONFIGS.get((self._current_mode, self._current_style))
            if config is None:
                config = FILTER_CONFIGS.get((ExecutionMode.NORMAL, DrivingStyle.STANDARD))

        if config:
            self._alpha, self._max_frame_change_pct, self._filter_mode_name = config

        # 更新状态（非 HOLD 情况）
        if self._current_mode == ExecutionMode.UNPAVED:
            self.state = FilterState.SOFT_FILTER
        elif self._current_style == DrivingStyle.COMFORT:
            self.state = FilterState.SOFT_FILTER
        elif self._current_style == DrivingStyle.EFFICIENT:
            self.state = FilterState.FAST_RESPONSE
        else:
            self.state = FilterState.NORMAL_FILTER

    # ========== 查询接口 ==========
    def get_state(self) -> FilterState:
        return self.state

    def get_current_filtered_value(self) -> float:
        return self._prev_filtered

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
            "current_style": self._current_style.value,
            "alpha": self._alpha,
            "prev_filtered": self._prev_filtered,
        }

    def emergency_shutdown(self):
        self.state = FilterState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，保持当前滤波值")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 加速平顺滤波单元 (ad-mcc-11) 演示")
    print("=" * 70)

    filt = ThrottleSmoothFilter()
    filt.set_driving_mode_query(lambda: ExecutionMode.NORMAL)
    filt.set_driving_style_query(lambda: DrivingStyle.STANDARD)

    # 初始化历史滤波值为25%
    filt._prev_filtered = 25.0

    print_separator("STEP 1: 标准滤波 (alpha=0.2)，从 25% 逐步响应到 30%")
    for i in range(5):
        seq = JerkCompliantSequence(
            corrected_throttle_pct=30.0,
            original_throttle_pct=30.0,
            expected_acceleration_ms2=1.0
        )
        filt.set_jerk_compliant_query(lambda s=seq: s)
        result = filt.run_filter_cycle()
        if result:
            print(f"  帧{i+1}: 原始={seq.corrected_throttle_pct}%, 滤波后={result.filtered_throttle_pct}%")

    print_separator("STEP 2: 舒适模式 (alpha=0.35，限制 3%/帧)")
    filt.set_driving_style_query(lambda: DrivingStyle.COMFORT)
    filt._update_filter_params()
    filt._prev_filtered = 20.0
    seq2 = JerkCompliantSequence(corrected_throttle_pct=50.0, original_throttle_pct=50.0)
    filt.set_jerk_compliant_query(lambda s=seq2: s)
    for i in range(5):
        result = filt.run_filter_cycle()
        if result:
            print(f"  帧{i+1}: 目标50%，滤波后={result.filtered_throttle_pct}% (方法: {result.filter_method})")

    print_separator("STEP 3: 三级降级强制归零")
    filt.set_driving_mode_query(lambda: ExecutionMode.DEGRADED_LEVEL3)
    filt._update_filter_params()
    result3 = filt.run_filter_cycle()
    if result3:
        print(f"  冻结输出: {result3.filtered_throttle_pct}%, 方法: {result3.filter_method}")

    print("\n✅ 加速平顺滤波单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-11 加速平顺滤波单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_filter(mode=ExecutionMode.NORMAL, style=DrivingStyle.STANDARD, prev_filtered=25.0):
            f = ThrottleSmoothFilter()
            f.set_driving_mode_query(lambda: mode)
            f.set_driving_style_query(lambda: style)
            f._prev_filtered = prev_filtered
            f._update_filter_params()
            return f

        # TC-M11-01: 标准滤波
        print("\n[TC-M11-01] 标准滤波 α=0.2，从25%到30%")
        try:
            f = setup_filter()
            f._alpha = 0.2
            f.set_jerk_compliant_query(lambda: JerkCompliantSequence(corrected_throttle_pct=30.0))
            result = f.run_filter_cycle()
            assert result is not None
            expected = 0.2 * 30.0 + 0.8 * 25.0  # 26.0
            assert abs(result.filtered_throttle_pct - expected) < 0.01
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M11-02: 舒适模式 + 变化率限制
        print("\n[TC-M11-02] 舒适模式 帧变化限制3%，从20%到大步长50%")
        try:
            f = setup_filter(style=DrivingStyle.COMFORT, prev_filtered=20.0)
            f._alpha = 0.35
            f._max_frame_change_pct = 3.0
            f.set_jerk_compliant_query(lambda: JerkCompliantSequence(corrected_throttle_pct=50.0))
            result = f.run_filter_cycle()
            assert result is not None
            assert result.filtered_throttle_pct <= 23.01
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M11-03: 高效模式 α=0.1
        print("\n[TC-M11-03] 高效模式 α=0.1，从35%到40%")
        try:
            f = setup_filter(style=DrivingStyle.EFFICIENT, prev_filtered=35.0)
            f._alpha = 0.1
            f.set_jerk_compliant_query(lambda: JerkCompliantSequence(corrected_throttle_pct=40.0))
            result = f.run_filter_cycle()
            assert result is not None
            expected = 0.1 * 40.0 + 0.9 * 35.0  # 35.5
            assert abs(result.filtered_throttle_pct - expected) < 0.01
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M11-04: 紧急冻结保持
        print("\n[TC-M11-04] 冻结保持指令，锁定值15%")
        try:
            f = setup_filter()
            f.set_emergency_freeze_query(lambda: EmergencyFreezeCommand(freeze_active=True, target_throttle_lock_pct=15.0))
            f.set_jerk_compliant_query(lambda: JerkCompliantSequence(corrected_throttle_pct=80.0))
            result = f.run_filter_cycle()
            assert result is not None
            assert result.filtered_throttle_pct == 15.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M11-05: 边界裁剪
        print("\n[TC-M11-05] 输入超100% 裁剪至100%")
        try:
            f = setup_filter(prev_filtered=90.0)
            f._alpha = 1.0
            f.set_jerk_compliant_query(lambda: JerkCompliantSequence(corrected_throttle_pct=110.0))
            result = f.run_filter_cycle()
            assert result is not None
            assert result.filtered_throttle_pct == 100.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M11-06: 非铺装移动平均
        print("\n[TC-M11-06] 非铺装模式，窗口3帧平均")
        try:
            f = setup_filter(mode=ExecutionMode.UNPAVED, prev_filtered=20.0)
            f._alpha = 0.5
            f._raw_window = deque([30.0, 35.0], maxlen=3)
            f.set_jerk_compliant_query(lambda: JerkCompliantSequence(corrected_throttle_pct=40.0))
            result = f.run_filter_cycle()
            assert result is not None
            expected = 0.5 * 35.0 + 0.5 * 20.0  # 27.5
            assert abs(result.filtered_throttle_pct - expected) < 0.01
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M11-07 (新增): 三级降级强制归零
        print("\n[TC-M11-07] 三级降级强制归零")
        try:
            f = setup_filter(mode=ExecutionMode.DEGRADED_LEVEL3, prev_filtered=45.0)
            f.set_jerk_compliant_query(lambda: JerkCompliantSequence(corrected_throttle_pct=30.0))
            result = f.run_filter_cycle()
            assert result is not None
            assert result.filtered_throttle_pct == 0.0
            assert f.state == FilterState.HOLD
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