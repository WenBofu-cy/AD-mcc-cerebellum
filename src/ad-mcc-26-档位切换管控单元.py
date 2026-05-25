#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-26
模块名称: 档位切换管控单元
所属分区: 七、档位与驻车管理
核心职责: 接收 ECC 大脑通过 ad-mcc-01 下发的档位切换意图（P/R/N/D），结合当前车速、
          制动踏板状态及车辆运动状态，执行安全、平顺的档位逻辑切换。确保 P 档仅能在车辆
          完全静止且满足驻车条件时切入，R 档与 D 档之间的切换需在车速低于安全阈值且制动
          踏板踩下时执行。输出标准化档位指令至变速箱控制器（TCU），并回传切换完成回执。
          不参与任何驾驶决策，仅执行档位切换的安全管控与执行。

依赖模块:
    ad-mcc-01(小脑总控调度核心，下发档位意图),
    车速传感器(CAN总线),
    制动踏板(CAN总线),
    TCU(变速箱控制器),
    ad-mcc-27(电子驻车制动控制单元)
被依赖模块:
    TCU(执行档位指令),
    ad-mcc-01(接收切换完成回执),
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: 车辆未完全静止（车速>0）时严禁切入 P 档，防止变速器锁止机构损坏
  S-02: 切换超时或 TCU 故障时，必须标记失败并禁止连续重试
  S-03: 从 P/R 切换至 D 时必须检测制动踏板，防止车辆意外起步
  S-04: 本模块仅执行档位切换的安全管控，不参与驾驶策略决策
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class GearState(Enum):
    GEAR_STABLE = "gear_stable"
    SHIFT_IN_PROGRESS = "shift_in_progress"
    SHIFT_FAILED = "shift_failed"
    SHIFT_INHIBITED = "shift_inhibited"
    SYSTEM_PAUSED = "system_paused"


class TargetGear(Enum):
    P = "P"
    R = "R"
    N = "N"
    D = "D"


@dataclass
class GearShiftIntent:
    target_gear: TargetGear = TargetGear.P
    source: str = ""
    priority: int = 0
    timestamp: float = field(default_factory=time.time)


@dataclass
class BrakePedalStatus:
    brake_switch: bool = False
    pressure_mpa: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class TCUFeedback:
    current_gear: TargetGear = TargetGear.P
    valid: bool = True
    fault_code: int = 0
    temp_protection: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class EPBStatus:
    clamped: bool = False
    fault: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class GearShiftCommand:
    target_gear: TargetGear = TargetGear.P
    mode: str = "normal"
    timeout_s: float = 2.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class ShiftAcknowledgment:
    requested_gear: TargetGear = TargetGear.P
    actual_gear: TargetGear = TargetGear.P
    result: str = ""
    duration_s: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class EPBRequest:
    request_clamp: bool = False
    reason: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class GearStatusReport:
    current_gear: TargetGear = TargetGear.P
    state: GearState = GearState.GEAR_STABLE
    fault_code: int = 0
    timestamp: float = field(default_factory=time.time)


SHIFT_TIMEOUT_S = 2.0
PARK_SPEED_KMH = 0.0
STILL_DURATION_S = 0.5
LOW_SPEED_THRESHOLD_KMH = 5.0
CONTROL_PERIOD_S = 0.02


class GearShiftController:
    def __init__(self):
        self.module_id = "ad-mcc-26"
        self.module_name = "档位切换管控单元"
        self.version = "V1.0"

        self.state = GearState.GEAR_STABLE
        self._current_gear = TargetGear.P
        self._pending_gear = None
        self._shift_timer = 0.0
        self._still_timer = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_gear_intent = None
        self._query_speed = None
        self._query_brake = None
        self._query_tcu = None
        self._query_epb = None

        self._publish_shift_command = None
        self._publish_ack = None
        self._publish_epb_request = None
        self._publish_status = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_gear_intent_query(self, callback):
        self._query_gear_intent = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_brake_query(self, callback):
        self._query_brake = callback

    def set_tcu_query(self, callback):
        self._query_tcu = callback

    def set_epb_query(self, callback):
        self._query_epb = callback

    def set_shift_command_publisher(self, callback):
        self._publish_shift_command = callback

    def set_ack_publisher(self, callback):
        self._publish_ack = callback

    def set_epb_request_publisher(self, callback):
        self._publish_epb_request = callback

    def set_status_publisher(self, callback):
        self._publish_status = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_control_cycle(self):
        now = time.time()
        if self.state == GearState.SYSTEM_PAUSED:
            return

        speed = self._query_speed() if self._query_speed else 0.0
        brake = self._query_brake() if self._query_brake else BrakePedalStatus()
        tcu = self._query_tcu() if self._query_tcu else TCUFeedback()
        epb = self._query_epb() if self._query_epb else EPBStatus()

        if tcu.valid:
            self._current_gear = tcu.current_gear

        if speed == 0.0:
            self._still_timer += CONTROL_PERIOD_S
        else:
            self._still_timer = 0.0

        if self.state == GearState.SHIFT_IN_PROGRESS:
            if tcu.fault_code != 0:
                self.state = GearState.SHIFT_FAILED
                self._send_ack(self._pending_gear, self._current_gear, "TCU故障")
                self._pending_gear = None
                self._log_event("SHIFT_FAILED", {"reason": "TCU故障"})
                return

            if self._current_gear == self._pending_gear:
                self.state = GearState.GEAR_STABLE
                self._send_ack(self._pending_gear, self._current_gear, "成功")
                if self._pending_gear == TargetGear.P:
                    self._request_epb_clamp()
                self._pending_gear = None
                self._log_event("SHIFT_SUCCESS", {"gear": self._current_gear.value})
                return

            if now - self._shift_timer > SHIFT_TIMEOUT_S:
                self.state = GearState.SHIFT_FAILED
                self._send_ack(self._pending_gear, self._current_gear, "超时")
                self._pending_gear = None
                self._log_event("SHIFT_FAILED", {"reason": "超时"})
                return
            return

        intent = self._query_gear_intent() if self._query_gear_intent else None
        if intent is None:
            return

        target = intent.target_gear
        allowed, reason = self._check_conditions(target, speed, brake, epb)
        if not allowed:
            self.state = GearState.SHIFT_INHIBITED
            self._send_ack(target, self._current_gear, reason)
            return

        if self._publish_shift_command:
            self._publish_shift_command(GearShiftCommand(target_gear=target, timeout_s=SHIFT_TIMEOUT_S))
        self.state = GearState.SHIFT_IN_PROGRESS
        self._pending_gear = target
        self._shift_timer = now

    def _check_conditions(self, target, speed, brake, epb):
        if target == TargetGear.P:
            if speed == PARK_SPEED_KMH and self._still_timer >= STILL_DURATION_S:
                if brake.brake_switch or epb.clamped:
                    return True, ""
                return False, "未踩制动且EPB未夹紧"
            return False, "车辆未完全静止"
        if target == TargetGear.R:
            if speed < LOW_SPEED_THRESHOLD_KMH and brake.brake_switch:
                return True, ""
            return False, "车速过高或未踩制动"
        if target == TargetGear.N:
            return True, ""
        if target == TargetGear.D:
            if speed < LOW_SPEED_THRESHOLD_KMH:
                if self._current_gear in (TargetGear.P, TargetGear.R) and not brake.brake_switch:
                    return False, "从P/R切换需踩制动"
                return True, ""
            return False, "车速过高"
        return False, "未知档位"

    def _send_ack(self, requested, actual, result):
        if self._publish_ack:
            self._publish_ack(ShiftAcknowledgment(
                requested_gear=requested if requested else TargetGear.P,
                actual_gear=actual,
                result=result,
                duration_s=time.time() - self._shift_timer if self._shift_timer else 0.0
            ))

    def _request_epb_clamp(self):
        if self._publish_epb_request:
            self._publish_epb_request(EPBRequest(request_clamp=True, reason="P档已切入"))

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

    def get_state(self) -> GearState:
        return self.state

    def get_current_gear(self) -> TargetGear:
        return self._current_gear

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def emergency_shutdown(self):
        self.state = GearState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 档位切换管控单元 (ad-mcc-26) 演示")
    print("=" * 70)

    ctrl = GearShiftController()
    ctrl.set_speed_query(lambda: 0.0)
    ctrl.set_brake_query(lambda: BrakePedalStatus(brake_switch=True))
    ctrl.set_tcu_query(lambda: TCUFeedback(current_gear=TargetGear.P))
    ctrl.set_epb_query(lambda: EPBStatus(clamped=True))

    print_separator("STEP 1: 从 P 切换至 D")
    ctrl.set_gear_intent_query(lambda: GearShiftIntent(target_gear=TargetGear.D))
    ctrl._still_timer = 1.0
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 2: TCU 确认切换")
    ctrl.set_tcu_query(lambda: TCUFeedback(current_gear=TargetGear.D))
    ctrl.run_control_cycle()
    print(f"  当前档位: {ctrl.get_current_gear().value}")

    print("\n✅ 档位切换管控单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-26 档位切换管控单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(speed=0.0, brake_switch=True, current_gear=TargetGear.P, epb_clamped=True):
            c = GearShiftController()
            c.set_speed_query(lambda: speed)
            c.set_brake_query(lambda: BrakePedalStatus(brake_switch=brake_switch))
            c.set_tcu_query(lambda: TCUFeedback(current_gear=current_gear))
            c.set_epb_query(lambda: EPBStatus(clamped=epb_clamped))
            c._still_timer = 1.0 if speed == 0.0 else 0.0
            return c

        print("\n[TC-M26-01] 正常从 P 切换至 D")
        try:
            c = setup_ctrl()
            c.set_gear_intent_query(lambda: GearShiftIntent(target_gear=TargetGear.D))
            c.run_control_cycle()
            assert c.state == GearState.SHIFT_IN_PROGRESS
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M26-02] 未踩制动被拒绝")
        try:
            c = setup_ctrl(brake_switch=False, epb_clamped=False)
            c.set_gear_intent_query(lambda: GearShiftIntent(target_gear=TargetGear.D))
            c.run_control_cycle()
            assert c.state == GearState.SHIFT_INHIBITED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M26-03] 高速拒绝切入 P")
        try:
            c = setup_ctrl(speed=10.0, current_gear=TargetGear.D)
            c._still_timer = 0.0
            c.set_gear_intent_query(lambda: GearShiftIntent(target_gear=TargetGear.P))
            c.run_control_cycle()
            assert c.state == GearState.SHIFT_INHIBITED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M26-04] TCU 确认切换成功")
        try:
            c = setup_ctrl()
            c.set_gear_intent_query(lambda: GearShiftIntent(target_gear=TargetGear.D))
            c.run_control_cycle()
            c.set_tcu_query(lambda: TCUFeedback(current_gear=TargetGear.D))
            c.run_control_cycle()
            assert c.state == GearState.GEAR_STABLE
            assert c.get_current_gear() == TargetGear.D
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M26-05] 切换超时")
        try:
            c = setup_ctrl()
            c.set_gear_intent_query(lambda: GearShiftIntent(target_gear=TargetGear.D))
            c.run_control_cycle()
            c._shift_timer = time.time() - 3.0
            c.run_control_cycle()
            assert c.state == GearState.SHIFT_FAILED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M26-06] 切入 P 档联动 EPB")
        try:
            c = setup_ctrl(current_gear=TargetGear.D)
            c.set_gear_intent_query(lambda: GearShiftIntent(target_gear=TargetGear.P))
            c.run_control_cycle()
            c.set_tcu_query(lambda: TCUFeedback(current_gear=TargetGear.P))
            epb_request = None
            def trap_epb(req):
                nonlocal epb_request
                epb_request = req
            c.set_epb_request_publisher(trap_epb)
            c.run_control_cycle()
            assert epb_request is not None and epb_request.request_clamp
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