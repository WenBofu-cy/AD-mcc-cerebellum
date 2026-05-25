#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-22
模块名称: 转向灯自动控制单元
所属分区: 六、灯光与外设管理
核心职责: 根据 ECC 大脑下发的变道/转弯意图及方向盘转角信号，自动控制转向灯的开启与关闭。
          确保转向灯在变道或转弯前至少提前 3 秒开启，并在动作完成后自动关闭。同时处理转向灯
          与双闪灯的优先级协调，避免双闪覆盖转向意图。不参与任何驾驶决策，仅执行灯光控制逻辑。

依赖模块:
    ad-mcc-01(小脑总控调度核心，下发转向/变道意图),
    方向盘转角传感器(CAN总线),
    轮速传感器(CAN总线),
    ad-mcc-23(双闪与刹车灯控制单元，协调优先级)
被依赖模块:
    车身灯光控制器,
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: 转向灯开启必须符合道路交通安全法规，变道/转弯前至少 3 秒开启
  S-02: 双闪激活时，非紧急情况下转向灯不得覆盖双闪
  S-03: 紧急避让等安全优先场景可临时覆盖双闪
  S-04: 本模块仅控制转向灯，不参与转向决策或执行
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class TurnSignalState(Enum):
    IDLE_OFF = "idle_off"
    LEFT_ACTIVE = "left_active"
    RIGHT_ACTIVE = "right_active"
    DELAY_OFF = "delay_off"
    HAZARD_OVERRIDE = "hazard_override"
    SYSTEM_PAUSED = "system_paused"


class TurnIntentType(Enum):
    LEFT_TURN = "左转"
    RIGHT_TURN = "右转"
    LEFT_LANE_CHANGE = "左变道"
    RIGHT_LANE_CHANGE = "右变道"
    EMERGENCY_SWERVE = "紧急避让"


@dataclass
class TurnIntent:
    intent_type: TurnIntentType = TurnIntentType.LEFT_TURN
    target_lane: int = 0
    expected_execution_time: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class HazardSignal:
    hazard_active: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class TurnSignalCommand:
    left_light: bool = False
    right_light: bool = False
    source: str = ""
    duration_estimate: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class TurnSignalStatus:
    state: TurnSignalState = TurnSignalState.IDLE_OFF
    remaining_delay: float = 0.0
    override_reason: str = ""
    timestamp: float = field(default_factory=time.time)


STEERING_THRESHOLD_DEG = 30.0
RETURN_THRESHOLD_DEG = 10.0
DELAY_DURATION_S = 1.0
STANDSTILL_DURATION_S = 3.0
CONTROL_PERIOD_S = 0.02


class TurnSignalController:
    def __init__(self):
        self.module_id = "ad-mcc-22"
        self.module_name = "转向灯自动控制单元"
        self.version = "V1.0"

        self.state = TurnSignalState.IDLE_OFF
        self._delay_timer = 0.0
        self._standstill_timer = 0.0
        self._emergency_override = False
        self._last_steering = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_turn_intent = None
        self._query_steering_angle = None
        self._query_speed = None
        self._query_hazard_signal = None

        self._publish_light_command = None
        self._publish_status = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_turn_intent_query(self, callback):
        self._query_turn_intent = callback

    def set_steering_angle_query(self, callback):
        self._query_steering_angle = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_hazard_signal_query(self, callback):
        self._query_hazard_signal = callback

    def set_light_command_publisher(self, callback):
        self._publish_light_command = callback

    def set_status_publisher(self, callback):
        self._publish_status = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_control_cycle(self):
        now = time.time()
        if self.state == TurnSignalState.SYSTEM_PAUSED:
            return

        steering = self._query_steering_angle() if self._query_steering_angle else 0.0
        speed = self._query_speed() if self._query_speed else 0.0
        hazard = self._query_hazard_signal() if self._query_hazard_signal else HazardSignal()
        intent = self._query_turn_intent() if self._query_turn_intent else None

        # 提前检测紧急避让意图，确保本帧即可覆盖双闪
        if intent is not None and intent.intent_type == TurnIntentType.EMERGENCY_SWERVE:
            self._emergency_override = True
        else:
            self._emergency_override = False

        if hazard.hazard_active and not self._emergency_override:
            if self.state not in (TurnSignalState.HAZARD_OVERRIDE, TurnSignalState.SYSTEM_PAUSED):
                self._turn_off_all("双闪覆盖")
                self.state = TurnSignalState.HAZARD_OVERRIDE
            return
        elif not hazard.hazard_active and self.state == TurnSignalState.HAZARD_OVERRIDE:
            self.state = TurnSignalState.IDLE_OFF

        target_side = None
        is_emergency = False
        if intent is not None:
            if intent.intent_type == TurnIntentType.EMERGENCY_SWERVE:
                is_emergency = True
            if intent.intent_type in (TurnIntentType.LEFT_TURN, TurnIntentType.LEFT_LANE_CHANGE):
                target_side = "left"
            elif intent.intent_type in (TurnIntentType.RIGHT_TURN, TurnIntentType.RIGHT_LANE_CHANGE):
                target_side = "right"
            elif intent.intent_type == TurnIntentType.EMERGENCY_SWERVE:
                if steering > STEERING_THRESHOLD_DEG:
                    target_side = "right"
                elif steering < -STEERING_THRESHOLD_DEG:
                    target_side = "left"

        if target_side is None:
            if steering > STEERING_THRESHOLD_DEG and speed > 10.0:
                target_side = "right"
            elif steering < -STEERING_THRESHOLD_DEG and speed > 10.0:
                target_side = "left"

        if target_side == "left":
            if self.state != TurnSignalState.LEFT_ACTIVE:
                if self.state == TurnSignalState.HAZARD_OVERRIDE and is_emergency:
                    self._emergency_override = True
                self._activate_left()
        elif target_side == "right":
            if self.state != TurnSignalState.RIGHT_ACTIVE:
                if self.state == TurnSignalState.HAZARD_OVERRIDE and is_emergency:
                    self._emergency_override = True
                self._activate_right()
        else:
            if self.state in (TurnSignalState.LEFT_ACTIVE, TurnSignalState.RIGHT_ACTIVE):
                self.state = TurnSignalState.DELAY_OFF
                self._delay_timer = now
            elif self.state == TurnSignalState.DELAY_OFF:
                if now - self._delay_timer >= DELAY_DURATION_S:
                    self._turn_off_all("转向完成")
                    self.state = TurnSignalState.IDLE_OFF
                    self._emergency_override = False

        if self.state in (TurnSignalState.LEFT_ACTIVE, TurnSignalState.RIGHT_ACTIVE, TurnSignalState.DELAY_OFF):
            if speed == 0.0:
                self._standstill_timer += CONTROL_PERIOD_S
                if self._standstill_timer >= STANDSTILL_DURATION_S:
                    self._turn_off_all("停车")
                    self.state = TurnSignalState.IDLE_OFF
                    self._emergency_override = False
            else:
                self._standstill_timer = 0.0

    def _activate_left(self):
        self.state = TurnSignalState.LEFT_ACTIVE
        self._send_light_command(left=True, right=False)
        self._log_event("LEFT_ON", {})

    def _activate_right(self):
        self.state = TurnSignalState.RIGHT_ACTIVE
        self._send_light_command(left=False, right=True)
        self._log_event("RIGHT_ON", {})

    def _turn_off_all(self, reason):
        self._send_light_command(left=False, right=False)
        self._log_event("TURN_OFF", {"reason": reason})

    def _send_light_command(self, left, right):
        if self._publish_light_command:
            self._publish_light_command(TurnSignalCommand(
                left_light=left,
                right_light=right,
                source="转向灯自动控制"
            ))

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

    def get_state(self) -> TurnSignalState:
        return self.state

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def emergency_shutdown(self):
        self.state = TurnSignalState.SYSTEM_PAUSED
        self._turn_off_all("紧急熔断")
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 转向灯自动控制单元 (ad-mcc-22) 演示")
    print("=" * 70)

    ctrl = TurnSignalController()
    ctrl.set_speed_query(lambda: 40.0)
    ctrl.set_steering_angle_query(lambda: 5.0)
    ctrl.set_hazard_signal_query(lambda: HazardSignal())

    print_separator("STEP 1: ECC 左转意图")
    ctrl.set_turn_intent_query(lambda: TurnIntent(intent_type=TurnIntentType.LEFT_TURN))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 2: 方向盘回正，延时关闭")
    ctrl.set_steering_angle_query(lambda: 2.0)
    ctrl.set_turn_intent_query(lambda: None)
    for _ in range(60):
        ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 3: 双闪覆盖")
    ctrl.set_hazard_signal_query(lambda: HazardSignal(hazard_active=True))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print("\n✅ 转向灯自动控制单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-22 转向灯自动控制单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(speed=40.0, steering=5.0, hazard=False, intent=None):
            c = TurnSignalController()
            c.set_speed_query(lambda: speed)
            c.set_steering_angle_query(lambda: steering)
            c.set_hazard_signal_query(lambda: HazardSignal(hazard_active=hazard))
            c.set_turn_intent_query(lambda: intent)
            return c

        print("\n[TC-M22-01] ECC左转意图开启左灯")
        try:
            c = setup_ctrl(intent=TurnIntent(intent_type=TurnIntentType.LEFT_TURN))
            c.run_control_cycle()
            assert c.state == TurnSignalState.LEFT_ACTIVE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M22-02] 方向盘右转自动开启右灯")
        try:
            c = setup_ctrl(steering=35.0)
            c.run_control_cycle()
            assert c.state == TurnSignalState.RIGHT_ACTIVE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M22-03] 回正后延时关闭")
        try:
            c = setup_ctrl(steering=2.0)
            c.state = TurnSignalState.LEFT_ACTIVE
            c._delay_timer = time.time() - 1.1
            c.run_control_cycle()
            assert c.state == TurnSignalState.IDLE_OFF
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M22-04] 双闪激活覆盖转向灯")
        try:
            c = setup_ctrl(hazard=True, intent=TurnIntent(intent_type=TurnIntentType.LEFT_TURN))
            c.run_control_cycle()
            assert c.state == TurnSignalState.HAZARD_OVERRIDE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M22-05] 双闪解除恢复待机")
        try:
            c = setup_ctrl(hazard=False)
            c.state = TurnSignalState.HAZARD_OVERRIDE
            c.run_control_cycle()
            assert c.state == TurnSignalState.IDLE_OFF
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M22-06] 停车3秒自动关闭")
        try:
            c = setup_ctrl(speed=0.0)
            c.state = TurnSignalState.RIGHT_ACTIVE
            c._standstill_timer = 3.1
            c.run_control_cycle()
            assert c.state == TurnSignalState.IDLE_OFF
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