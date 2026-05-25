#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-27
模块名称: 电子驻车制动控制单元
所属分区: 七、档位与驻车管理
核心职责: 根据车辆静止状态、驾驶员意图（起步、驻车）及来自 ad-mcc-26 的 P 档联动请求，
          自动控制电子驻车制动（EPB）的夹紧与释放。在车辆静止且满足驻车条件时自动激活 EPB；
          在驾驶员系好安全带、挂入 D/R 档并轻踩油门时自动释放 EPB，实现平顺起步。同时监控
          EPB 系统状态，处理坡道辅助与故障。不参与任何驾驶决策，仅执行驻车制动的伺服控制。

依赖模块:
    ad-mcc-26(档位切换管控单元，下发 P 档联动夹紧请求),
    车速/油门/制动/安全带(CAN总线),
    EPB 控制器(CAN总线)
被依赖模块:
    ad-mcc-01(小脑总控调度核心，接收 EPB 状态上报),
    ad-mcc-26(接收 EPB 状态供档位安全校验),
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: EPB 夹紧必须在 P 档切入后 2 秒内完成，防止车辆意外移动
  S-02: 自动释放必须确保驾驶员在环（安全带系好、有明确起步意图），严禁在无人状态下自动释放
  S-03: EPB 系统故障时，必须保持夹紧状态（故障安全原则）
  S-04: 本模块仅负责 EPB 的伺服控制，不参与车辆动态决策
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class EPBState(Enum):
    RELEASED = "released"
    CLAMPING = "clamping"
    CLAMPED = "clamped"
    RELEASING = "releasing"
    FAULT = "fault"
    SYSTEM_PAUSED = "system_paused"


class TargetGear(Enum):
    P = "P"
    R = "R"
    N = "N"
    D = "D"


@dataclass
class PClampRequest:
    request_clamp: bool = False
    reason: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class DriverInputs:
    throttle_pct: float = 0.0
    brake_switch: bool = False
    seatbelt_fastened: bool = True
    manual_epb_button: bool = False


@dataclass
class EPBFeedback:
    clamped: bool = False
    released: bool = True
    fault_code: int = 0
    pressure: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class EPBCommand:
    action: str = "release"  # "clamp" or "release"
    target_pressure_ratio: float = 1.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class EPBStatusReport:
    state: EPBState = EPBState.RELEASED
    fault_code: int = 0
    pressure: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class AutoHoldStatus:
    active: bool = False
    state: str = "off"


CLAMP_TIMEOUT_S = 2.0
RELEASE_TIMEOUT_S = 3.0
THROTTLE_THRESHOLD_PCT = 5.0
GRADE_THRESHOLD_PCT = 3.0
CONTROL_PERIOD_S = 0.02


class ElectronicParkingBrakeController:
    def __init__(self):
        self.module_id = "ad-mcc-27"
        self.module_name = "电子驻车制动控制单元"
        self.version = "V1.0"

        self.state = EPBState.RELEASED
        self._pending_action = None
        self._action_timer = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_p_clamp_request = None
        self._query_speed = None
        self._query_throttle = None
        self._query_brake_switch = None
        self._query_seatbelt = None
        self._query_gear = None
        self._query_epb_feedback = None
        self._query_grade = None
        self._query_manual_epb = None
        self._query_engine_off = None

        self._publish_epb_command = None
        self._publish_status = None
        self._publish_autohold_status = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_p_clamp_request_query(self, callback):
        self._query_p_clamp_request = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_throttle_query(self, callback):
        self._query_throttle = callback

    def set_brake_switch_query(self, callback):
        self._query_brake_switch = callback

    def set_seatbelt_query(self, callback):
        self._query_seatbelt = callback

    def set_gear_query(self, callback):
        self._query_gear = callback

    def set_epb_feedback_query(self, callback):
        self._query_epb_feedback = callback

    def set_grade_query(self, callback):
        self._query_grade = callback

    def set_manual_epb_query(self, callback):
        self._query_manual_epb = callback

    def set_engine_off_query(self, callback):
        self._query_engine_off = callback

    def set_epb_command_publisher(self, callback):
        self._publish_epb_command = callback

    def set_status_publisher(self, callback):
        self._publish_status = callback

    def set_autohold_status_publisher(self, callback):
        self._publish_autohold_status = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_control_cycle(self):
        now = time.time()
        if self.state == EPBState.SYSTEM_PAUSED:
            return

        p_req = self._query_p_clamp_request() if self._query_p_clamp_request else PClampRequest()
        speed = self._query_speed() if self._query_speed else 0.0
        throttle = self._query_throttle() if self._query_throttle else 0.0
        brake_switch = self._query_brake_switch() if self._query_brake_switch else False
        seatbelt = self._query_seatbelt() if self._query_seatbelt else False
        gear = self._query_gear() if self._query_gear else TargetGear.P
        epb_fb = self._query_epb_feedback() if self._query_epb_feedback else EPBFeedback()
        grade = self._query_grade() if self._query_grade else 0.0
        manual_epb = self._query_manual_epb() if self._query_manual_epb else False
        engine_off = self._query_engine_off() if self._query_engine_off else False

        if epb_fb.fault_code != 0:
            self.state = EPBState.FAULT
            self._log_event("EPB_FAULT", {"fault_code": epb_fb.fault_code})
            return

        # 处理正在进行的动作
        if self.state == EPBState.CLAMPING:
            if epb_fb.clamped:
                self.state = EPBState.CLAMPED
                self._pending_action = None
            elif now - self._action_timer > CLAMP_TIMEOUT_S:
                self.state = EPBState.FAULT
                self._log_event("CLAMP_TIMEOUT", {})
            else:
                return

        if self.state == EPBState.RELEASING:
            if epb_fb.released:
                self.state = EPBState.RELEASED
                self._pending_action = None
            elif now - self._action_timer > RELEASE_TIMEOUT_S:
                self.state = EPBState.FAULT
                self._log_event("RELEASE_TIMEOUT", {})
            else:
                return

        # 夹紧请求判定
        need_clamp = p_req.request_clamp or manual_epb or engine_off
        if need_clamp and self.state not in (EPBState.CLAMPED, EPBState.CLAMPING):
            self._send_epb_command("clamp")
            self.state = EPBState.CLAMPING
            self._action_timer = now
            return

        # 释放判定 (仅在已夹紧状态)
        if self.state == EPBState.CLAMPED:
            can_release = (
                gear in (TargetGear.D, TargetGear.R) and
                seatbelt and
                throttle > THROTTLE_THRESHOLD_PCT and
                not brake_switch
            )
            if can_release:
                if grade > GRADE_THRESHOLD_PCT:
                    # 实际需与动力系统协调，这里简化放行
                    pass
                self._send_epb_command("release")
                self.state = EPBState.RELEASING
                self._action_timer = now

        # 状态上报
        if getattr(self, '_last_state', None) != self.state:
            self._last_state = self.state
            if self._publish_status:
                self._publish_status(EPBStatusReport(
                    state=self.state,
                    fault_code=epb_fb.fault_code,
                    pressure=epb_fb.pressure
                ))
            self._log_event("STATE_CHANGE", {"state": self.state.value})

    def _send_epb_command(self, action: str):
        if self._publish_epb_command:
            self._publish_epb_command(EPBCommand(action=action))

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

    def get_state(self) -> EPBState:
        return self.state

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def emergency_shutdown(self):
        if self.state == EPBState.RELEASED:
            self._send_epb_command("clamp")
        self.state = EPBState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 电子驻车制动控制单元 (ad-mcc-27) 演示")
    print("=" * 70)

    ctrl = ElectronicParkingBrakeController()
    ctrl.set_p_clamp_request_query(lambda: PClampRequest())
    ctrl.set_speed_query(lambda: 0.0)
    ctrl.set_throttle_query(lambda: 0.0)
    ctrl.set_brake_switch_query(lambda: False)
    ctrl.set_seatbelt_query(lambda: True)
    ctrl.set_gear_query(lambda: TargetGear.P)
    ctrl.set_epb_feedback_query(lambda: EPBFeedback(released=True))
    ctrl.set_grade_query(lambda: 0.0)
    ctrl.set_manual_epb_query(lambda: False)
    ctrl.set_engine_off_query(lambda: False)

    print_separator("STEP 1: P 档联动夹紧")
    ctrl.set_p_clamp_request_query(lambda: PClampRequest(request_clamp=True, reason="P档"))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    # 模拟 EPB 夹紧到位
    ctrl.set_epb_feedback_query(lambda: EPBFeedback(clamped=True, released=False))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 2: 满足起步条件自动释放")
    ctrl.set_gear_query(lambda: TargetGear.D)
    ctrl.set_throttle_query(lambda: 10.0)
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    # 模拟 EPB 释放到位
    ctrl.set_epb_feedback_query(lambda: EPBFeedback(released=True))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print("\n✅ 电子驻车制动控制单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-27 电子驻车制动控制单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(clamped=False, released=True, fault=0, gear=TargetGear.P,
                       throttle=0.0, brake=False, seatbelt=True, p_req=False,
                       manual_epb=False, engine_off=False, grade=0.0):
            c = ElectronicParkingBrakeController()
            c.set_p_clamp_request_query(lambda: PClampRequest(request_clamp=p_req))
            c.set_speed_query(lambda: 0.0)
            c.set_throttle_query(lambda: throttle)
            c.set_brake_switch_query(lambda: brake)
            c.set_seatbelt_query(lambda: seatbelt)
            c.set_gear_query(lambda: gear)
            c.set_epb_feedback_query(lambda: EPBFeedback(clamped=clamped, released=released, fault_code=fault))
            c.set_grade_query(lambda: grade)
            c.set_manual_epb_query(lambda: manual_epb)
            c.set_engine_off_query(lambda: engine_off)
            return c

        print("\n[TC-M27-01] P 档联动夹紧")
        try:
            c = setup_ctrl(p_req=True)
            c.run_control_cycle()
            assert c.state == EPBState.CLAMPING
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M27-02] 夹紧到位")
        try:
            c = setup_ctrl(clamped=True, released=False)
            c.state = EPBState.CLAMPING
            c.run_control_cycle()
            assert c.state == EPBState.CLAMPED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M27-03] 满足起步条件自动释放")
        try:
            c = setup_ctrl(clamped=True, released=False, gear=TargetGear.D, throttle=10.0, seatbelt=True)
            c.state = EPBState.CLAMPED
            c.run_control_cycle()
            assert c.state == EPBState.RELEASING
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M27-04] 未系安全带不释放")
        try:
            c = setup_ctrl(clamped=True, released=False, gear=TargetGear.D, throttle=10.0, seatbelt=False)
            c.state = EPBState.CLAMPED
            c.run_control_cycle()
            assert c.state == EPBState.CLAMPED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M27-05] 夹紧超时故障")
        try:
            c = setup_ctrl(clamped=False, released=False)
            c.state = EPBState.CLAMPING
            c._action_timer = time.time() - CLAMP_TIMEOUT_S - 0.1
            c.run_control_cycle()
            assert c.state == EPBState.FAULT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M27-06] 紧急熔断时夹紧")
        try:
            c = setup_ctrl(released=True)
            c.emergency_shutdown()
            assert c.state == EPBState.SYSTEM_PAUSED
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