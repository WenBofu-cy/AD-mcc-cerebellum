#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-14
模块名称: 制动响应加速单元
所属分区: 四、制动控制集群
核心职责: 接收 ad-mcc-13 输出的制动目标压力序列，根据制动类型（日常缓刹/紧急制动）
          和当前实际制动压力，动态控制制动压力的建立速率与波形。紧急制动时实现极速建压
          （<100ms 达到目标压力的90%），日常制动时控制压力平缓上升，消除突兀感。同时管理
          制动管路的预填充与压力保持，输出实时压力指令至下游执行模块。
          不参与任何场景判断与驾驶决策。

依赖模块:
    ad-mcc-13(制动压力解算单元，提供制动目标压力序列),
    制动压力传感器(CAN总线),
    ad-mcc-34(动力与制动参数管理单元，提供制动系统建压特性参数)
被依赖模块:
    ad-mcc-15(制动平顺防点头单元，接收平滑后的压力指令曲线),
    ad-mcc-02(运动生理边界闸门，校验输出压力指令)

安全约束:
  S-01: 紧急制动建压时间从指令到达到实际压力达目标90%不得超过150ms，超时强制上报告警
  S-02: 输出压力指令禁止超过制动系统物理最大压力，防止管路爆裂
  S-03: ABS/ESP 激活时，快速泄压指令优先级高于一切建压指令
  S-04: 建压过程中不得出现压力回落（除非收到泄压指令），防止制动点头或制动力波动
  S-05: 本模块仅负责制动压力的动态响应控制，不参与制动时机的决策
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


class BuildState(Enum):
    IDLE_DEPRESSURIZED = "idle_depressurized"
    GENTLE_BUILDUP = "gentle_buildup"
    EMERGENCY_BUILDUP = "emergency_buildup"
    HOLDING = "holding"
    GENTLE_RELEASE = "gentle_release"
    FAST_RELEASE = "fast_release"
    SYSTEM_PAUSED = "system_paused"


class BrakeType(Enum):
    GENTLE = "日常缓刹"
    EMERGENCY = "紧急制动"


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
class BuildCharacteristics:
    gentle_build_rate_mpa_per_s: float = 20.0
    emergency_build_rate_mpa_per_s: float = 150.0
    prefill_pressure_mpa: float = 0.5
    prefill_duration_ms: float = 50.0
    max_overshoot_mpa: float = 0.2
    build_timeout_ms: float = 500.0
    emergency_timeout_ms: float = 150.0
    release_rate_mpa_per_s: float = 15.0
    fast_release_rate_mpa_per_s: float = 80.0
    unpaved_build_rate_mpa_per_s: float = 12.0
    unpaved_release_rate_mpa_per_s: float = 10.0


@dataclass
class FastReleaseCommand:
    active: bool = True
    residual_pressure_mpa: float = 0.0


@dataclass
class RealTimeBrakeCommand:
    timestamp: float = field(default_factory=time.time)
    current_target_pressure_mpa: float = 0.0
    pressure_rate_mpa_per_s: float = 0.0
    brake_type: BrakeType = BrakeType.GENTLE
    build_phase: BuildState = BuildState.IDLE_DEPRESSURIZED
    expected_arrival_time: float = 0.0


@dataclass
class BuildStatusReport:
    current_state: BuildState = BuildState.IDLE_DEPRESSURIZED
    actual_pressure_mpa: float = 0.0
    target_pressure_mpa: float = 0.0
    build_duration_ms: float = 0.0
    overshoot_mpa: float = 0.0


@dataclass
class EmergencyBuildTimeoutAlert:
    target_pressure_mpa: float = 0.0
    actual_pressure_mpa: float = 0.0
    elapsed_ms: float = 0.0
    reason: str = ""


CONTROL_PERIOD_S = 0.005
PRE_BUILD_HOLD_THRESHOLD_MPA = 0.1
GENTLE_APPROACH_SLOPE = 0.3


class BrakePressureResponse:
    def __init__(self):
        self.module_id = "ad-mcc-14"
        self.module_name = "制动响应加速单元"
        self.version = "V1.0"

        self.state = BuildState.IDLE_DEPRESSURIZED
        self._current_target = 0.0
        self._build_start_time = 0.0
        self._prefill_done = False
        self._prefill_duration = 0.0
        self._last_output_pressure = 0.0
        self._brake_type = BrakeType.GENTLE
        self._build_characteristics = BuildCharacteristics()
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_brake_sequence = None
        self._query_actual_pressure = None
        self._query_build_chars = None
        self._query_fast_release = None

        self._publish_realtime_command = None
        self._publish_status_report = None
        self._publish_emergency_timeout = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_brake_sequence_query(self, callback):
        self._query_brake_sequence = callback

    def set_actual_pressure_query(self, callback):
        self._query_actual_pressure = callback

    def set_build_chars_query(self, callback):
        self._query_build_chars = callback

    def set_fast_release_query(self, callback):
        self._query_fast_release = callback

    def set_realtime_command_publisher(self, callback):
        self._publish_realtime_command = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_emergency_timeout_publisher(self, callback):
        self._publish_emergency_timeout = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_response_cycle(self) -> Optional[RealTimeBrakeCommand]:
        now = time.time()

        fast_release = self._query_fast_release() if self._query_fast_release else None
        if fast_release and fast_release.active:
            self.state = BuildState.FAST_RELEASE
            self._current_target = fast_release.residual_pressure_mpa
            self._prefill_done = False
            actual = self._get_actual_pressure()
            rate = self._build_characteristics.fast_release_rate_mpa_per_s
            step = rate * CONTROL_PERIOD_S
            output = max(actual - step, self._current_target)
            output = round(output, 4)
            self._last_output_pressure = output
            cmd = RealTimeBrakeCommand(
                timestamp=now,
                current_target_pressure_mpa=output,
                pressure_rate_mpa_per_s=-rate,
                brake_type=BrakeType.GENTLE,
                build_phase=self.state,
            )
            self._publish_if_possible(cmd)
            if actual <= self._current_target + 0.05:
                self.state = BuildState.IDLE_DEPRESSURIZED
                self._current_target = 0.0
            return cmd

        if self.state == BuildState.SYSTEM_PAUSED:
            return None

        seq = self._query_brake_sequence() if self._query_brake_sequence else None
        if seq is None:
            return self._maintain_current_pressure()

        target = seq.target_pressure_mpa
        brake_type = seq.brake_type
        self._brake_type = brake_type
        actual = self._get_actual_pressure()

        if self._query_build_chars:
            chars = self._query_build_chars()
            if chars:
                self._build_characteristics = chars

        if target <= 0.05:
            if self.state in (BuildState.EMERGENCY_BUILDUP, BuildState.FAST_RELEASE):
                self.state = BuildState.FAST_RELEASE
                self._current_target = 0.0
            else:
                self.state = BuildState.GENTLE_RELEASE
                self._current_target = 0.0
        elif brake_type == BrakeType.EMERGENCY or target > 7.0:
            self.state = BuildState.EMERGENCY_BUILDUP
            self._current_target = target
            self._build_start_time = now
            self._prefill_done = False
            self._prefill_duration = 0.0
        else:
            if target > self._current_target:
                self.state = BuildState.GENTLE_BUILDUP
                if self._current_target < PRE_BUILD_HOLD_THRESHOLD_MPA:
                    self._build_start_time = now
                    self._prefill_done = False
                    self._prefill_duration = 0.0
                self._current_target = target
            elif target < self._current_target and target > 0:
                self._current_target = target
                if self.state != BuildState.GENTLE_RELEASE:
                    self.state = BuildState.GENTLE_RELEASE

        output = self._execute_control(actual, target)
        output = max(0.0, min(output, self._get_max_pressure()))
        self._last_output_pressure = output

        cmd = RealTimeBrakeCommand(
            timestamp=now,
            current_target_pressure_mpa=round(output, 4),
            pressure_rate_mpa_per_s=self._get_current_rate(),
            brake_type=self._brake_type,
            build_phase=self.state,
        )
        if self._publish_realtime_command:
            self._publish_realtime_command(cmd)

        self._check_timeout(now)
        return cmd

    def _execute_control(self, actual: float, target: float) -> float:
        chars = self._build_characteristics
        if self.state == BuildState.IDLE_DEPRESSURIZED:
            return 0.0

        if self.state in (BuildState.GENTLE_BUILDUP, BuildState.EMERGENCY_BUILDUP):
            if not self._prefill_done and actual < chars.prefill_pressure_mpa:
                self._prefill_duration += CONTROL_PERIOD_S * 1000.0
                if self._prefill_duration >= chars.prefill_duration_ms:
                    self._prefill_done = True
                return chars.prefill_pressure_mpa

            if self.state == BuildState.EMERGENCY_BUILDUP:
                rate = chars.emergency_build_rate_mpa_per_s
                overshoot = 0.5
            else:
                rate = chars.gentle_build_rate_mpa_per_s
                overshoot = chars.max_overshoot_mpa

            step = rate * CONTROL_PERIOD_S
            output = min(actual + step, target + overshoot)

            if target - output < 0.3:
                output = output + (target - output) * GENTLE_APPROACH_SLOPE

            if output < actual:
                output = actual

            if actual >= target - 0.1 and output >= target - 0.1:
                self.state = BuildState.HOLDING
                duration = (time.time() - self._build_start_time) * 1000.0
                self._report_build_complete(duration, actual - target)

            return output

        elif self.state == BuildState.HOLDING:
            if abs(actual - target) > 0.2:
                return actual + math.copysign(0.1, target - actual)
            return target

        elif self.state == BuildState.GENTLE_RELEASE:
            step = chars.release_rate_mpa_per_s * CONTROL_PERIOD_S
            output = max(actual - step, target)
            if actual <= 0.05:
                self.state = BuildState.IDLE_DEPRESSURIZED
            return output

        elif self.state == BuildState.FAST_RELEASE:
            step = chars.fast_release_rate_mpa_per_s * CONTROL_PERIOD_S
            output = max(actual - step, target)
            if actual <= target + 0.05:
                self.state = BuildState.IDLE_DEPRESSURIZED
            return output

        return 0.0

    def _maintain_current_pressure(self) -> Optional[RealTimeBrakeCommand]:
        if self.state == BuildState.HOLDING:
            actual = self._get_actual_pressure()
            target = self._current_target
            if abs(actual - target) > 0.2:
                output = actual + math.copysign(0.1, target - actual)
            else:
                output = target
            output = max(0.0, min(output, self._get_max_pressure()))
            self._last_output_pressure = output
            cmd = RealTimeBrakeCommand(
                current_target_pressure_mpa=round(output, 4),
                pressure_rate_mpa_per_s=0.0,
                brake_type=self._brake_type,
                build_phase=self.state,
            )
            if self._publish_realtime_command:
                self._publish_realtime_command(cmd)
            return cmd
        return None

    def _check_timeout(self, now: float):
        if self.state not in (BuildState.EMERGENCY_BUILDUP, BuildState.GENTLE_BUILDUP):
            return
        elapsed = (now - self._build_start_time) * 1000.0
        chars = self._build_characteristics
        if self.state == BuildState.EMERGENCY_BUILDUP and elapsed > chars.emergency_timeout_ms:
            actual = self._get_actual_pressure()
            alert = EmergencyBuildTimeoutAlert(
                target_pressure_mpa=self._current_target,
                actual_pressure_mpa=actual,
                elapsed_ms=elapsed,
                reason="紧急建压超时"
            )
            if self._publish_emergency_timeout:
                self._publish_emergency_timeout(alert)
            self._log_event("EMERGENCY_TIMEOUT", {"target": self._current_target, "actual": actual})
        elif self.state == BuildState.GENTLE_BUILDUP and elapsed > chars.build_timeout_ms:
            actual = self._get_actual_pressure()
            self._log_event("BUILD_TIMEOUT", {"target": self._current_target, "actual": actual})

    def _report_build_complete(self, duration_ms: float, overshoot: float):
        if self._publish_status_report:
            self._publish_status_report(BuildStatusReport(
                current_state=self.state,
                actual_pressure_mpa=self._get_actual_pressure(),
                target_pressure_mpa=self._current_target,
                build_duration_ms=duration_ms,
                overshoot_mpa=max(0.0, overshoot)
            ))

    def _get_actual_pressure(self) -> float:
        if self._query_actual_pressure:
            return self._query_actual_pressure()
        return 0.0

    def _get_current_rate(self) -> float:
        chars = self._build_characteristics
        if self.state == BuildState.EMERGENCY_BUILDUP:
            return chars.emergency_build_rate_mpa_per_s
        elif self.state == BuildState.GENTLE_BUILDUP:
            return chars.gentle_build_rate_mpa_per_s
        elif self.state == BuildState.FAST_RELEASE:
            return -chars.fast_release_rate_mpa_per_s
        elif self.state == BuildState.GENTLE_RELEASE:
            return -chars.release_rate_mpa_per_s
        return 0.0

    def _get_max_pressure(self) -> float:
        return 10.0

    def _publish_if_possible(self, cmd: RealTimeBrakeCommand):
        if self._publish_realtime_command:
            self._publish_realtime_command(cmd)

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

    def get_state(self) -> BuildState:
        return self.state

    def emergency_shutdown(self):
        self.state = BuildState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 制动响应加速单元 (ad-mcc-14) 演示")
    print("=" * 70)

    resp = BrakePressureResponse()
    actual_pressure = [0.0]
    def actual_query():
        return actual_pressure[0]
    resp.set_actual_pressure_query(actual_query)
    resp.set_build_chars_query(lambda: BuildCharacteristics())

    print_separator("STEP 1: 日常缓刹至 3.0 MPa")
    resp.set_brake_sequence_query(lambda: BrakePressureSequence(
        target_pressure_mpa=3.0, brake_type=BrakeType.GENTLE
    ))
    for i in range(30):
        cmd = resp.run_response_cycle()
        if cmd:
            actual_pressure[0] = cmd.current_target_pressure_mpa
        if i % 10 == 0 and cmd:
            print(f"  t={i*5}ms 输出压力={cmd.current_target_pressure_mpa:.2f} MPa 状态={cmd.build_phase.value}")

    print_separator("STEP 2: 紧急制动至 9.0 MPa")
    resp.set_brake_sequence_query(lambda: BrakePressureSequence(
        target_pressure_mpa=9.0, brake_type=BrakeType.EMERGENCY
    ))
    for i in range(20):
        cmd = resp.run_response_cycle()
        if cmd:
            actual_pressure[0] = cmd.current_target_pressure_mpa
        if i % 5 == 0 and cmd:
            print(f"  t={i*5}ms 输出压力={cmd.current_target_pressure_mpa:.2f} MPa 状态={cmd.build_phase.value}")

    print("\n✅ 制动响应加速单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-14 制动响应加速单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_response():
            r = BrakePressureResponse()
            actual = [0.0]
            r.set_actual_pressure_query(lambda: actual[0])
            r.set_build_chars_query(lambda: BuildCharacteristics())
            return r, actual

        print("\n[TC-M14-01] 日常缓刹建压")
        try:
            r, act = setup_response()
            r.set_brake_sequence_query(lambda: BrakePressureSequence(
                target_pressure_mpa=3.0, brake_type=BrakeType.GENTLE
            ))
            cmd = None
            for _ in range(80):
                cmd = r.run_response_cycle()
                if cmd:
                    act[0] = cmd.current_target_pressure_mpa
            assert cmd is not None
            assert act[0] >= 2.9
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M14-02] 紧急建压")
        try:
            r, act = setup_response()
            r.set_brake_sequence_query(lambda: BrakePressureSequence(
                target_pressure_mpa=9.0, brake_type=BrakeType.EMERGENCY
            ))
            cmd = None
            for _ in range(30):
                cmd = r.run_response_cycle()
                if cmd:
                    act[0] = cmd.current_target_pressure_mpa
            assert act[0] >= 8.1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M14-03] 保压状态")
        try:
            r, act = setup_response()
            act[0] = 3.0
            r._current_target = 3.0
            r.state = BuildState.HOLDING
            r._brake_type = BrakeType.GENTLE
            r.set_brake_sequence_query(lambda: BrakePressureSequence(
                target_pressure_mpa=3.0, brake_type=BrakeType.GENTLE
            ))
            cmd = r.run_response_cycle()
            assert cmd is not None and abs(cmd.current_target_pressure_mpa - 3.0) < 0.5
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M14-04] 日常泄压")
        try:
            r, act = setup_response()
            act[0] = 3.0
            r.state = BuildState.GENTLE_BUILDUP
            r._current_target = 3.0
            r.set_brake_sequence_query(lambda: BrakePressureSequence(
                target_pressure_mpa=0.0, brake_type=BrakeType.GENTLE
            ))
            for _ in range(50):
                cmd = r.run_response_cycle()
                if cmd:
                    act[0] = cmd.current_target_pressure_mpa
            assert act[0] <= 0.5
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M14-05] ABS快速泄压")
        try:
            r, act = setup_response()
            act[0] = 5.0
            r.set_fast_release_query(lambda: FastReleaseCommand(active=True, residual_pressure_mpa=1.0))
            cmd = r.run_response_cycle()
            assert cmd is not None and cmd.current_target_pressure_mpa < 5.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M14-06] 紧急建压超时告警")
        try:
            r, act = setup_response()
            r.set_actual_pressure_query(lambda: 1.0)
            r.set_brake_sequence_query(lambda: BrakePressureSequence(
                target_pressure_mpa=9.0, brake_type=BrakeType.EMERGENCY
            ))
            alert_triggered = False
            r.set_emergency_timeout_publisher(lambda a: None)
            original_publish = r._publish_emergency_timeout
            def trap_alert(a):
                nonlocal alert_triggered
                alert_triggered = True
                if original_publish:
                    original_publish(a)
            r.set_emergency_timeout_publisher(trap_alert)
            for _ in range(40):
                r.run_response_cycle()
            assert alert_triggered
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