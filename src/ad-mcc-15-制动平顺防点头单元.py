#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-15
模块名称: 制动平顺防点头单元
所属分区: 四、制动控制集群
核心职责: 接收 ad-mcc-14 输出的实时制动压力指令，在车辆即将完全停止（车速 < 5 km/h）时，
          主动介入制动压力的精细调节。通过动态降低制动压力变化速率、在停止瞬间提前微量释放
          压力并立即恢复，消除传统制动过程中因惯性导致的“点头”现象。使得车辆停止过程柔和平稳，
          提升乘坐舒适性。不参与制动时机的决策，仅对制动末端压力曲线进行平顺化处理。

依赖模块:
    ad-mcc-14(制动响应加速单元，提供实时制动压力指令),
    车辆轮速/车速传感器(CAN总线),
    ad-mcc-34(动力与制动参数管理单元，提供悬架特性参数)
被依赖模块:
    ad-mcc-16(制动执行偏差监控单元，接收最终的制动压力指令作为目标值),
    实际制动执行器接口

安全约束:
  S-01: 紧急制动情况下，防点头功能必须立即旁路，无条件直通原始制动压力，确保制动距离不受影响
  S-02: 末端释放阶段的压力降低幅度不得超过当前目标压力的 20%，且总降压时间不得超过 100ms，
        防止制动力不足导致溜车
  S-03: 停止保压期间，若检测到车辆有微小移动（车速 > 0.2 km/h），应立即增加压力至
        最大驻车压力（3.0 MPa）防止溜车
  S-04: 本模块仅在制动末端微调压力，不参与正常行驶和紧急制动的压力决策
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


class AntiDiveState(Enum):
    NORMAL_PASSTHROUGH = "normal_passthrough"
    PREDICT_STOP = "predict_stop"
    END_RELEASE = "end_release"
    STOP_HOLD = "stop_hold"
    SYSTEM_PAUSED = "system_paused"


class BrakeType(Enum):
    GENTLE = "日常缓刹"
    EMERGENCY = "紧急制动"


@dataclass
class RealTimeBrakeCommand:
    timestamp: float = field(default_factory=time.time)
    current_target_pressure_mpa: float = 0.0
    pressure_rate_mpa_per_s: float = 0.0
    brake_type: BrakeType = BrakeType.GENTLE
    build_phase: str = ""


@dataclass
class SmoothBrakeCommand:
    timestamp: float = field(default_factory=time.time)
    corrected_pressure_mpa: float = 0.0
    original_pressure_mpa: float = 0.0
    anti_dive_state: AntiDiveState = AntiDiveState.NORMAL_PASSTHROUGH
    correction_amount: float = 0.0


@dataclass
class AntiDiveStatus:
    current_state: AntiDiveState = AntiDiveState.NORMAL_PASSTHROUGH
    estimated_stop_time: float = 0.0
    pressure_correction_factor: float = 1.0


# 控制参数
PRE_STOP_SPEED_THRESHOLD_KMH = 5.0
END_RELEASE_SPEED_THRESHOLD_KMH = 1.0
DECEL_THRESHOLD_MS2 = 0.5
PRESSURE_THRESHOLD_MPA = 1.0
STANDSTILL_DURATION_S = 0.5
ROLL_BACK_SPEED_KMH = 0.2
EMERGENCY_HOLD_PRESSURE_MPA = 3.0
DEFAULT_HOLD_PRESSURE_MPA = 2.0

# 末端释放参数
RELEASE_DROP_RATIO = 0.15
RELEASE_DURATION_MS = 150.0
RELEASE_HALF_DURATION_MS = 75.0
MAX_RELEASE_DROP_PCT = 0.20
MAX_RELEASE_DROP_TIME_MS = 100.0

# 预判滤波
PREDICT_ALPHA = 0.4
RATE_SCALE_FACTOR = 0.6

# 控制周期 (200Hz)
CONTROL_PERIOD_S = 0.005
SIGNAL_TIMEOUT_S = 0.2


class AntiDiveController:
    def __init__(self):
        self.module_id = "ad-mcc-15"
        self.module_name = "制动平顺防点头单元"
        self.version = "V1.0"

        self.state = AntiDiveState.NORMAL_PASSTHROUGH
        self._smoothed_target = 0.0
        self._prev_smoothed = 0.0
        self._hold_pressure = DEFAULT_HOLD_PRESSURE_MPA
        self._release_start_time = 0.0
        self._pre_release_pressure = 0.0
        self._standstill_timer = 0.0
        self._last_speed = 0.0
        self._speed_lost_timer = 0.0

        self._pending_logs: List[Dict[str, Any]] = []

        # 回调注入
        self._query_brake_command = None
        self._query_vehicle_speed = None
        self._query_longitudinal_decel = None
        self._query_stop_line_distance = None
        self._query_suspension_params = None

        self._publish_smooth_command = None
        self._publish_status_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_brake_command_query(self, callback):
        self._query_brake_command = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_longitudinal_decel_query(self, callback):
        self._query_longitudinal_decel = callback

    def set_stop_line_distance_query(self, callback):
        self._query_stop_line_distance = callback

    def set_suspension_params_query(self, callback):
        self._query_suspension_params = callback

    def set_smooth_command_publisher(self, callback):
        self._publish_smooth_command = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_control_cycle(self) -> Optional[SmoothBrakeCommand]:
        now = time.time()

        if self.state == AntiDiveState.SYSTEM_PAUSED:
            return None

        # 获取实时制动指令
        brake_cmd = self._query_brake_command() if self._query_brake_command else None
        if brake_cmd is None:
            # 无制动指令时的处理
            if self.state == AntiDiveState.STOP_HOLD:
                # 继续保压
                output = self._hold_pressure
                state = self.state
                original = output
            else:
                # 其他状态回到直通，输出0
                self.state = AntiDiveState.NORMAL_PASSTHROUGH
                return None
        else:
            original = brake_cmd.current_target_pressure_mpa
            brake_type = brake_cmd.brake_type

            # 紧急制动立即旁路
            if brake_type == BrakeType.EMERGENCY:
                self.state = AntiDiveState.NORMAL_PASSTHROUGH
                output = original
                state = self.state
                self._reset_filter()
            else:
                # 获取车辆状态
                speed = self._get_speed()
                decel = self._get_decel()

                # 车速信号超时处理
                if speed is None:
                    self._speed_lost_timer += CONTROL_PERIOD_S
                    if self._speed_lost_timer > SIGNAL_TIMEOUT_S:
                        self.state = AntiDiveState.NORMAL_PASSTHROUGH
                        output = original
                        state = self.state
                    else:
                        speed = self._last_speed
                else:
                    self._speed_lost_timer = 0.0
                    self._last_speed = speed

                # 状态判定
                if speed is not None and speed == 0.0:
                    self._standstill_timer += CONTROL_PERIOD_S
                else:
                    self._standstill_timer = 0.0

                if self._standstill_timer >= STANDSTILL_DURATION_S and original > 0.5:
                    self.state = AntiDiveState.STOP_HOLD
                    self._hold_pressure = original
                elif speed is not None and speed < END_RELEASE_SPEED_THRESHOLD_KMH and original > 0.5 and self.state == AntiDiveState.PREDICT_STOP:
                    self.state = AntiDiveState.END_RELEASE
                    self._release_start_time = now
                    self._pre_release_pressure = self._smoothed_target
                elif (speed is not None and speed < PRE_STOP_SPEED_THRESHOLD_KMH and
                      decel is not None and decel > DECEL_THRESHOLD_MS2 and
                      original > PRESSURE_THRESHOLD_MPA):
                    if self.state not in (AntiDiveState.END_RELEASE, AntiDiveState.STOP_HOLD):
                        self.state = AntiDiveState.PREDICT_STOP
                else:
                    self.state = AntiDiveState.NORMAL_PASSTHROUGH

                # 执行相应状态的压力处理
                if self.state == AntiDiveState.NORMAL_PASSTHROUGH:
                    output = original
                    self._reset_filter()
                elif self.state == AntiDiveState.PREDICT_STOP:
                    output = self._handle_predict(original, brake_cmd.pressure_rate_mpa_per_s)
                elif self.state == AntiDiveState.END_RELEASE:
                    output = self._handle_release(now, original)
                elif self.state == AntiDiveState.STOP_HOLD:
                    output = self._handle_hold(speed, original)
                else:
                    output = original

                state = self.state

        # 边界裁剪
        output = max(0.0, min(output, 12.0))  # 制动系统最大压力假设 12 MPa
        correction = output - original

        # 输出平顺化指令
        cmd = SmoothBrakeCommand(
            timestamp=now,
            corrected_pressure_mpa=round(output, 4),
            original_pressure_mpa=round(original, 4),
            anti_dive_state=state,
            correction_amount=round(correction, 4)
        )
        if self._publish_smooth_command:
            self._publish_smooth_command(cmd)

        # 状态变更上报
        # 简化：如果状态有变化就上报（此处省略状态变化跟踪，可直接在调用方控制）
        # 这里仅做演示性状态上报，实际可记录日志
        if self._publish_status_report and state != getattr(self, '_last_reported_state', None):
            self._publish_status_report(AntiDiveStatus(
                current_state=state,
                pressure_correction_factor=0.6 if state == AntiDiveState.PREDICT_STOP else 1.0
            ))
            self._last_reported_state = state

        return cmd

    def _handle_predict(self, target: float, rate: float) -> float:
        # 低通滤波
        self._smoothed_target = PREDICT_ALPHA * target + (1 - PREDICT_ALPHA) * self._prev_smoothed
        # 限制压力变化速率至正常的 60%
        max_step = rate * RATE_SCALE_FACTOR * CONTROL_PERIOD_S
        if abs(self._smoothed_target - self._prev_smoothed) > max_step:
            self._smoothed_target = self._prev_smoothed + math.copysign(max_step,
                                                                        self._smoothed_target - self._prev_smoothed)
        self._prev_smoothed = self._smoothed_target
        return self._smoothed_target

    def _handle_release(self, now: float, original: float) -> float:
        elapsed = (now - self._release_start_time) * 1000.0
        if elapsed < RELEASE_HALF_DURATION_MS:
            # 降压阶段：线性降至 85% 的 pre_release_pressure
            ratio = elapsed / RELEASE_HALF_DURATION_MS
            pressure = self._pre_release_pressure * (1.0 - RELEASE_DROP_RATIO * ratio)
        elif elapsed < RELEASE_DURATION_MS:
            # 恢复阶段：线性升至 hold_pressure
            ratio = (elapsed - RELEASE_HALF_DURATION_MS) / RELEASE_HALF_DURATION_MS
            start_pressure = self._pre_release_pressure * (1.0 - RELEASE_DROP_RATIO)
            pressure = start_pressure + (self._hold_pressure - start_pressure) * ratio
        else:
            # 释放完成，转入保压
            pressure = self._hold_pressure
            self.state = AntiDiveState.STOP_HOLD
            self._smoothed_target = pressure
            self._prev_smoothed = pressure
        return pressure

    def _handle_hold(self, speed: Optional[float], original: float) -> float:
        # 溜车检测：车速 > 0.2 km/h 且无新的大幅加速意图
        if speed is not None and speed > ROLL_BACK_SPEED_KMH:
            # 提升至最大驻车压力
            self._hold_pressure = EMERGENCY_HOLD_PRESSURE_MPA
            return self._hold_pressure
        # 如果目标压力显著增加（驾驶员起步），退出保压
        if original > self._hold_pressure + 1.0:
            self.state = AntiDiveState.NORMAL_PASSTHROUGH
            self._reset_filter()
            return original
        # 保持驻车压力
        return self._hold_pressure

    def _get_speed(self) -> Optional[float]:
        if self._query_vehicle_speed:
            return self._query_vehicle_speed()
        return None

    def _get_decel(self) -> Optional[float]:
        if self._query_longitudinal_decel:
            return self._query_longitudinal_decel()
        return None

    def _reset_filter(self):
        self._smoothed_target = 0.0
        self._prev_smoothed = 0.0

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

    def get_state(self) -> AntiDiveState:
        return self.state

    def emergency_shutdown(self):
        self.state = AntiDiveState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 制动平顺防点头单元 (ad-mcc-15) 演示")
    print("=" * 70)

    ctrl = AntiDiveController()
    speed = [60.0]
    decel = [0.0]
    ctrl.set_vehicle_speed_query(lambda: speed[0])
    ctrl.set_longitudinal_decel_query(lambda: decel[0])

    print_separator("STEP 1: 高速直通")
    ctrl.set_brake_command_query(lambda: RealTimeBrakeCommand(
        current_target_pressure_mpa=2.0,
        pressure_rate_mpa_per_s=20.0,
        brake_type=BrakeType.GENTLE
    ))
    cmd = ctrl.run_control_cycle()
    if cmd:
        print(f"  状态: {cmd.anti_dive_state.value}, 输出压力: {cmd.corrected_pressure_mpa:.3f} MPa")

    print_separator("STEP 2: 预判停车 (3 km/h)")
    speed[0] = 3.0
    decel[0] = 1.2
    for i in range(20):
        cmd = ctrl.run_control_cycle()
    if cmd:
        print(f"  状态: {cmd.anti_dive_state.value}, 输出压力: {cmd.corrected_pressure_mpa:.3f} MPa")

    print_separator("STEP 3: 末端释放 (0.5 km/h)")
    speed[0] = 0.5
    for i in range(40):
        cmd = ctrl.run_control_cycle()
        if cmd and i % 10 == 0:
            print(f"  t={i*5}ms 状态: {cmd.anti_dive_state.value}, 压力: {cmd.corrected_pressure_mpa:.3f} MPa")

    print("\n✅ 制动平顺防点头单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-15 制动平顺防点头单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(speed=60.0, decel=0.0):
            c = AntiDiveController()
            c.set_vehicle_speed_query(lambda: speed)
            c.set_longitudinal_decel_query(lambda: decel)
            c.set_brake_command_query(lambda: RealTimeBrakeCommand(
                current_target_pressure_mpa=2.0,
                pressure_rate_mpa_per_s=20.0,
                brake_type=BrakeType.GENTLE
            ))
            return c

        # TC-M15-01: 高速直通
        print("\n[TC-M15-01] 高速直通 (60 km/h)")
        try:
            c = setup_ctrl(speed=60.0)
            cmd = c.run_control_cycle()
            assert cmd is not None
            assert cmd.anti_dive_state == AntiDiveState.NORMAL_PASSTHROUGH
            assert cmd.corrected_pressure_mpa == 2.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M15-02: 预判停车
        print("\n[TC-M15-02] 预判停车 (3 km/h, -1.2 m/s²)")
        try:
            c = setup_ctrl(speed=3.0, decel=1.2)
            for _ in range(5):
                cmd = c.run_control_cycle()
            assert cmd is not None
            assert cmd.anti_dive_state == AntiDiveState.PREDICT_STOP
            # 压力应被滤波/限速，与原始值有差异
            assert cmd.corrected_pressure_mpa != 2.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M15-03: 末端释放
        print("\n[TC-M15-03] 末端释放 (0.5 km/h)")
        try:
            c = setup_ctrl(speed=0.5, decel=0.6)
            # 先强制进入预判状态
            c.state = AntiDiveState.PREDICT_STOP
            c._smoothed_target = 2.0
            c._prev_smoothed = 2.0
            cmd = c.run_control_cycle()
            assert cmd.anti_dive_state == AntiDiveState.END_RELEASE
            # 压力应低于原始
            assert cmd.corrected_pressure_mpa < 2.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M15-04: 紧急制动旁路
        print("\n[TC-M15-04] 紧急制动旁路")
        try:
            c = setup_ctrl(speed=3.0, decel=6.0)
            c.set_brake_command_query(lambda: RealTimeBrakeCommand(
                current_target_pressure_mpa=9.0,
                pressure_rate_mpa_per_s=150.0,
                brake_type=BrakeType.EMERGENCY
            ))
            cmd = c.run_control_cycle()
            assert cmd.anti_dive_state == AntiDiveState.NORMAL_PASSTHROUGH
            assert cmd.corrected_pressure_mpa == 9.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M15-05: 溜车防护
        print("\n[TC-M15-05] 停止保压时溜车，增压至3.0 MPa")
        try:
            c = setup_ctrl(speed=0.3, decel=0.0)
            c.state = AntiDiveState.STOP_HOLD
            c._hold_pressure = 2.0
            c.set_brake_command_query(lambda: RealTimeBrakeCommand(
                current_target_pressure_mpa=2.0,
                pressure_rate_mpa_per_s=0.0,
                brake_type=BrakeType.GENTLE
            ))
            cmd = c.run_control_cycle()
            assert cmd.corrected_pressure_mpa == EMERGENCY_HOLD_PRESSURE_MPA
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M15-06: 起步退出保压
        print("\n[TC-M15-06] 起步时退出保压")
        try:
            c = setup_ctrl(speed=0.0, decel=0.0)
            c.state = AntiDiveState.STOP_HOLD
            c._hold_pressure = 2.0
            c.set_brake_command_query(lambda: RealTimeBrakeCommand(
                current_target_pressure_mpa=3.5,
                pressure_rate_mpa_per_s=20.0,
                brake_type=BrakeType.GENTLE
            ))
            cmd = c.run_control_cycle()
            assert cmd.anti_dive_state == AntiDiveState.NORMAL_PASSTHROUGH
            assert cmd.corrected_pressure_mpa == 3.5
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