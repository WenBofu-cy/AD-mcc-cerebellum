#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-36
模块名称: 执行闭环反馈单元
所属分区: 十、执行反馈与日志
核心职责: 汇总各执行模块（转向、动力、制动、姿态、灯光、档位等）每次操控动作的完整执行结果，
          包括偏差报告、完成信号、响应延迟与目标达成判定。将分散的偏差数据整合为统一的
          “动作闭环回执”，上报至 ad-mcc-01 供上层决策与元认知评估，同时推送至 ad-mcc-37 供
          运动质量评估使用。不参与任何操控决策，仅负责执行结果的汇总、比对与闭环反馈。

依赖模块:
    ad-mcc-07(转向执行偏差监控单元),
    ad-mcc-12(动力执行偏差监控单元),
    ad-mcc-16(制动执行偏差监控单元),
    ad-mcc-18(车身姿态实时监测单元),
    ad-mcc-22(转向灯自动控制单元),
    ad-mcc-26(档位切换管控单元)
被依赖模块:
    ad-mcc-01(小脑总控调度核心),
    ad-mcc-37(运动质量评估单元),
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: 关键模块（转向、制动）反馈缺失或偏差严重超标时，必须判定为“未达成”，不得虚报达成
  S-02: 闭环回执中的数据必须与各模块上报的原始偏差数据一致，不得修饰或篡改
  S-03: 超时未反馈的模块必须明确标记，并在闭环回执中注明
  S-04: 本模块仅负责执行结果的汇总与判定，不参与任何操控指令的生成或修改
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class FeedbackState(Enum):
    IDLE = "idle"
    COLLECTING = "collecting"
    JUDGING = "judging"
    SYSTEM_PAUSED = "system_paused"


class CommandType(Enum):
    STEERING = "steering"
    THROTTLE = "throttle"
    BRAKE = "brake"
    ATTITUDE = "attitude"
    LIGHT = "light"
    GEAR = "gear"
    COMPOSITE = "composite"


@dataclass
class ActionCommandNotice:
    command_id: str = ""
    command_type: CommandType = CommandType.COMPOSITE
    target_summary: Dict[str, float] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class SteeringDeviation:
    command_id: str = ""
    angle_deviation_deg: float = 0.0
    rate_deviation_pct: float = 0.0
    response_latency_ms: float = 0.0
    online: bool = True


@dataclass
class ThrottleDeviation:
    command_id: str = ""
    speed_deviation_kmh: float = 0.0
    response_latency_ms: float = 0.0
    online: bool = True


@dataclass
class BrakeDeviation:
    command_id: str = ""
    pressure_deviation_mpa: float = 0.0
    response_latency_ms: float = 0.0
    online: bool = True


@dataclass
class AttitudeDeviation:
    command_id: str = ""
    yaw_rate_deviation: float = 0.0
    roll_deviation: float = 0.0
    online: bool = True


@dataclass
class LightDeviation:
    command_id: str = ""
    state_match: bool = True
    online: bool = True


@dataclass
class GearDeviation:
    command_id: str = ""
    actual_gear: str = ""
    shift_duration_ms: float = 0.0
    online: bool = True


@dataclass
class ClosedLoopAck:
    command_id: str = ""
    target_achieved: str = ""
    deviation_summary: Dict[str, Any] = field(default_factory=dict)
    overall_rating: str = ""
    max_response_latency_ms: float = 0.0
    online_modules_count: int = 0
    total_modules_count: int = 0
    timestamp: float = field(default_factory=time.time)


@dataclass
class DeviationDataPackage:
    command_id: str = ""
    deviations: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class TimeoutAlert:
    command_id: str = ""
    timeout_modules: List[str] = field(default_factory=list)
    wait_duration_ms: float = 0.0
    timestamp: float = field(default_factory=time.time)


TOLERANCE = {
    "steering_angle": 3.0,
    "steering_angle_unpaved": 5.0,
    "speed": 3.0,
    "brake_pressure": 0.5,
    "brake_pressure_emergency": 1.0,
    "yaw_rate": 3.0,
    "response_latency_max": 200.0,
}

TIMEOUT_MS = 500.0
CONTROL_PERIOD_S = 0.01


class ClosedLoopFeedback:
    def __init__(self):
        self.module_id = "ad-mcc-36"
        self.module_name = "执行闭环反馈单元"
        self.version = "V1.0"

        self.state = FeedbackState.IDLE
        self._active_command_id: Optional[str] = None
        self._active_command_type: Optional[CommandType] = None
        self._required_modules: List[str] = []
        self._collected_data: Dict[str, Any] = {}
        self._start_time: float = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_command_notice = None
        self._query_steering_dev = None
        self._query_throttle_dev = None
        self._query_brake_dev = None
        self._query_attitude_dev = None
        self._query_light_dev = None
        self._query_gear_dev = None

        self._publish_ack = None
        self._publish_deviation_package = None
        self._publish_event_log = None
        self._publish_timeout_alert = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_command_notice_query(self, callback):
        self._query_command_notice = callback

    def set_steering_dev_query(self, callback):
        self._query_steering_dev = callback

    def set_throttle_dev_query(self, callback):
        self._query_throttle_dev = callback

    def set_brake_dev_query(self, callback):
        self._query_brake_dev = callback

    def set_attitude_dev_query(self, callback):
        self._query_attitude_dev = callback

    def set_light_dev_query(self, callback):
        self._query_light_dev = callback

    def set_gear_dev_query(self, callback):
        self._query_gear_dev = callback

    def set_ack_publisher(self, callback):
        self._publish_ack = callback

    def set_deviation_package_publisher(self, callback):
        self._publish_deviation_package = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def set_timeout_alert_publisher(self, callback):
        self._publish_timeout_alert = callback

    def run_feedback_cycle(self):
        now = time.time()
        if self.state == FeedbackState.SYSTEM_PAUSED:
            return

        notice = self._query_command_notice() if self._query_command_notice else None
        if notice and self.state == FeedbackState.IDLE:
            self._active_command_id = notice.command_id
            self._active_command_type = notice.command_type
            self._required_modules = self._get_required_modules(notice.command_type)
            self._collected_data = {}
            self._start_time = now
            self.state = FeedbackState.COLLECTING
            return

        if self.state != FeedbackState.COLLECTING:
            return

        self._collect_module_data("steering", self._query_steering_dev)
        self._collect_module_data("throttle", self._query_throttle_dev)
        self._collect_module_data("brake", self._query_brake_dev)
        self._collect_module_data("attitude", self._query_attitude_dev)
        self._collect_module_data("light", self._query_light_dev)
        self._collect_module_data("gear", self._query_gear_dev)

        collected_modules = set(self._collected_data.keys())
        required_set = set(self._required_modules)
        if collected_modules >= required_set:
            self.state = FeedbackState.JUDGING
        elif now - self._start_time > TIMEOUT_MS / 1000.0:
            missing = list(required_set - collected_modules)
            if self._publish_timeout_alert:
                self._publish_timeout_alert(TimeoutAlert(
                    command_id=self._active_command_id,
                    timeout_modules=missing,
                    wait_duration_ms=TIMEOUT_MS,
                ))
            self.state = FeedbackState.JUDGING

        if self.state == FeedbackState.JUDGING:
            self._perform_judgment(now)

    def _collect_module_data(self, module_name: str, query_func):
        if module_name not in self._required_modules:
            return
        if module_name in self._collected_data:
            return
        if query_func:
            data = query_func()
            if data and getattr(data, 'command_id', '') == self._active_command_id:
                self._collected_data[module_name] = data

    def _get_required_modules(self, cmd_type: CommandType) -> List[str]:
        mapping = {
            CommandType.STEERING: ["steering"],
            CommandType.THROTTLE: ["throttle"],
            CommandType.BRAKE: ["brake"],
            CommandType.ATTITUDE: ["attitude"],
            CommandType.LIGHT: ["light"],
            CommandType.GEAR: ["gear"],
            CommandType.COMPOSITE: ["steering", "throttle", "brake", "attitude", "light", "gear"],
        }
        return mapping.get(cmd_type, [])

    def _perform_judgment(self, now: float):
        deviations = {}
        max_latency = 0.0
        all_achieved = True
        critical_failed = False

        collected_modules = set(self._collected_data.keys())
        required_set = set(self._required_modules)
        missing_modules = required_set - collected_modules

        for module in missing_modules:
            deviations[module] = {"status": "超时未反馈"}
            if module in ("steering", "brake"):
                critical_failed = True
                all_achieved = False
            else:
                all_achieved = False

        if "steering" in self._required_modules and "steering" not in missing_modules:
            dev = self._collected_data.get("steering")
            if dev and dev.online:
                angle_dev = abs(dev.angle_deviation_deg)
                latency = dev.response_latency_ms
                deviations["steering"] = {
                    "angle_deviation": angle_dev,
                    "rate_deviation": dev.rate_deviation_pct,
                    "latency_ms": latency,
                }
                max_latency = max(max_latency, latency)
                if angle_dev > TOLERANCE["steering_angle"] or latency > TOLERANCE["response_latency_max"]:
                    all_achieved = False
                    critical_failed = True
            else:
                all_achieved = False
                critical_failed = True
                deviations["steering"] = {"status": "模块离线"}

        if "throttle" in self._required_modules and "throttle" not in missing_modules:
            dev = self._collected_data.get("throttle")
            if dev and dev.online:
                speed_dev = abs(dev.speed_deviation_kmh)
                latency = dev.response_latency_ms
                deviations["throttle"] = {
                    "speed_deviation": speed_dev,
                    "latency_ms": latency,
                }
                max_latency = max(max_latency, latency)
                if speed_dev > TOLERANCE["speed"] or latency > TOLERANCE["response_latency_max"]:
                    all_achieved = False
            else:
                all_achieved = False
                deviations["throttle"] = {"status": "模块离线"}

        if "brake" in self._required_modules and "brake" not in missing_modules:
            dev = self._collected_data.get("brake")
            if dev and dev.online:
                pressure_dev = abs(dev.pressure_deviation_mpa)
                latency = dev.response_latency_ms
                deviations["brake"] = {
                    "pressure_deviation": pressure_dev,
                    "latency_ms": latency,
                }
                max_latency = max(max_latency, latency)
                if pressure_dev > TOLERANCE["brake_pressure"] or latency > TOLERANCE["response_latency_max"]:
                    all_achieved = False
                    critical_failed = True
            else:
                all_achieved = False
                critical_failed = True
                deviations["brake"] = {"status": "模块离线"}

        if "attitude" in self._required_modules and "attitude" not in missing_modules:
            dev = self._collected_data.get("attitude")
            if dev and dev.online:
                yaw_dev = abs(dev.yaw_rate_deviation)
                deviations["attitude"] = {"yaw_rate_deviation": yaw_dev}
                if yaw_dev > TOLERANCE["yaw_rate"]:
                    all_achieved = False
            else:
                all_achieved = False
                deviations["attitude"] = {"status": "模块离线"}

        if "light" in self._required_modules and "light" not in missing_modules:
            dev = self._collected_data.get("light")
            if dev and dev.online:
                deviations["light"] = {"state_match": dev.state_match}
                if not dev.state_match:
                    all_achieved = False
            else:
                all_achieved = False
                deviations["light"] = {"status": "模块离线"}

        if "gear" in self._required_modules and "gear" not in missing_modules:
            dev = self._collected_data.get("gear")
            if dev and dev.online:
                deviations["gear"] = {"actual_gear": dev.actual_gear, "shift_duration": dev.shift_duration_ms}
            else:
                deviations["gear"] = {"status": "模块离线"}

        if critical_failed:
            overall = "未达成"
        elif not all_achieved:
            overall = "部分达成"
        else:
            overall = "完全达成"

        online_count = sum(1 for m in self._required_modules if m in self._collected_data and self._collected_data[m].online)

        if self._publish_ack:
            self._publish_ack(ClosedLoopAck(
                command_id=self._active_command_id,
                target_achieved=overall,
                deviation_summary=deviations,
                overall_rating=overall,
                max_response_latency_ms=max_latency,
                online_modules_count=online_count,
                total_modules_count=len(self._required_modules),
            ))

        if self._publish_deviation_package:
            self._publish_deviation_package(DeviationDataPackage(
                command_id=self._active_command_id,
                deviations=deviations,
            ))

        if self._publish_event_log:
            self._publish_event_log({
                "event": "closed_loop",
                "command_id": self._active_command_id,
                "result": overall,
                "latency": max_latency,
            })

        self.state = FeedbackState.IDLE
        self._active_command_id = None
        self._collected_data = {}

    def get_state(self) -> FeedbackState:
        return self.state

    def emergency_shutdown(self):
        self.state = FeedbackState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 执行闭环反馈单元 (ad-mcc-36) 演示")
    print("=" * 70)

    fb = ClosedLoopFeedback()
    fb.set_command_notice_query(lambda: ActionCommandNotice(
        command_id="CMD-001",
        command_type=CommandType.STEERING,
    ))
    fb.set_steering_dev_query(lambda: SteeringDeviation(
        command_id="CMD-001",
        angle_deviation_deg=1.5,
        response_latency_ms=50.0,
        online=True,
    ))

    print_separator("STEP 1: 收到转向指令，收集反馈")
    for _ in range(3):
        fb.run_feedback_cycle()
    print(f"  状态: {fb.state.value}")

    print("\n✅ 执行闭环反馈单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-36 执行闭环反馈单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_fb():
            f = ClosedLoopFeedback()
            return f

        print("\n[TC-M36-01] 完全达成")
        try:
            f = setup_fb()
            ack_result = None
            def trap_ack(ack):
                nonlocal ack_result
                ack_result = ack
            f.set_ack_publisher(trap_ack)
            f.set_command_notice_query(lambda: ActionCommandNotice(
                command_id="C01", command_type=CommandType.STEERING
            ))
            f.set_steering_dev_query(lambda: SteeringDeviation(
                command_id="C01", angle_deviation_deg=1.0, response_latency_ms=80.0, online=True
            ))
            for _ in range(5):
                f.run_feedback_cycle()
            assert ack_result is not None and ack_result.target_achieved == "完全达成"
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M36-02] 转向偏差超标")
        try:
            f = setup_fb()
            ack_result = None
            def trap_ack(ack):
                nonlocal ack_result
                ack_result = ack
            f.set_ack_publisher(trap_ack)
            f.set_command_notice_query(lambda: ActionCommandNotice(
                command_id="C02", command_type=CommandType.STEERING
            ))
            f.set_steering_dev_query(lambda: SteeringDeviation(
                command_id="C02", angle_deviation_deg=5.5, response_latency_ms=90.0, online=True
            ))
            for _ in range(5):
                f.run_feedback_cycle()
            assert ack_result is not None and ack_result.target_achieved == "未达成"
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M36-03] 制动模块超时")
        try:
            f = setup_fb()
            ack_result = None
            def trap_ack(ack):
                nonlocal ack_result
                ack_result = ack
            f.set_ack_publisher(trap_ack)
            f.set_command_notice_query(lambda: ActionCommandNotice(
                command_id="C03", command_type=CommandType.BRAKE
            ))
            f.set_brake_dev_query(lambda: None)
            f._start_time = time.time() - 1.0
            f.state = FeedbackState.COLLECTING
            f._active_command_id = "C03"
            f._active_command_type = CommandType.BRAKE
            f._required_modules = ["brake"]
            f.run_feedback_cycle()
            assert ack_result is not None and ack_result.target_achieved == "未达成"
            assert "brake" in ack_result.deviation_summary
            assert ack_result.deviation_summary["brake"]["status"] == "超时未反馈"
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M36-04] 档位切换正常")
        try:
            f = setup_fb()
            ack_result = None
            def trap_ack(ack):
                nonlocal ack_result
                ack_result = ack
            f.set_ack_publisher(trap_ack)
            f.set_command_notice_query(lambda: ActionCommandNotice(
                command_id="C04", command_type=CommandType.GEAR
            ))
            f.set_gear_dev_query(lambda: GearDeviation(
                command_id="C04", actual_gear="D", shift_duration_ms=300.0, online=True
            ))
            for _ in range(5):
                f.run_feedback_cycle()
            assert ack_result is not None and ack_result.target_achieved == "完全达成"
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M36-05] 旧指令数据丢弃")
        try:
            f = setup_fb()
            f.set_command_notice_query(lambda: ActionCommandNotice(
                command_id="C05", command_type=CommandType.STEERING
            ))
            f.state = FeedbackState.COLLECTING
            f._active_command_id = "C05"
            f._required_modules = ["steering"]
            f._collected_data = {}
            old_data = SteeringDeviation(command_id="C04_OLD", angle_deviation_deg=0.5, online=True)
            f.set_steering_dev_query(lambda: old_data)
            f.run_feedback_cycle()
            assert "steering" not in f._collected_data
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M36-06] 紧急熔断")
        try:
            f = setup_fb()
            f.emergency_shutdown()
            assert f.state == FeedbackState.SYSTEM_PAUSED
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