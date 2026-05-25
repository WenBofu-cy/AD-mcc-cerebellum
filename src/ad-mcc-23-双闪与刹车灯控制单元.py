#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-23
模块名称: 双闪与刹车灯控制单元
所属分区: 六、灯光与外设管理
核心职责: 根据制动类型（日常缓刹/紧急制动）、当前降级等级及停车状态，自动控制双闪灯和
          刹车灯的开启与闪烁模式。紧急制动时自动高频闪烁刹车灯以警示后车，并在车速降至
          安全范围后激活双闪；降级停车时强制开启双闪，直至系统恢复。同时向 ad-mcc-22 提供
          双闪激活信号以协调转向灯优先级。不参与任何驾驶决策，仅执行灯光控制逻辑。

依赖模块:
    ad-mcc-14(制动响应加速单元，提供制动类型与建压状态),
    车速传感器(CAN总线),
    ad-mcc-01(小脑总控调度核心，下发降级等级信号)
被依赖模块:
    ad-mcc-22(转向灯自动控制单元，接收双闪激活信号),
    车身灯光控制器,
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: 紧急制动时刹车灯必须高频闪烁，频率不得低于 4Hz，确保对后车的警示效果
  S-02: 降级停车时双闪必须强制开启，不得被转向灯或手动操作覆盖
  S-03: 手动双闪指令优先级高于自动控制，本模块在手动激活时应暂停自动控制
  S-04: 本模块仅负责灯光控制，不参与制动决策或车辆动力学干预
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


class LightControlState(Enum):
    NORMAL_STANDBY = "normal_standby"
    EMERGENCY_BRAKE_FLASH = "emergency_brake_flash"
    DEGRADED_HAZARD = "degraded_hazard"
    HAZARD_HOLD = "hazard_hold"
    SYSTEM_PAUSED = "system_paused"


class BrakeType(Enum):
    GENTLE = "日常缓刹"
    EMERGENCY = "紧急制动"


class DegradationLevel(Enum):
    NORMAL = 0
    LEVEL1 = 1
    LEVEL2 = 2
    LEVEL3 = 3


@dataclass
class BrakeStatus:
    brake_type: BrakeType = BrakeType.GENTLE
    target_pressure_mpa: float = 0.0
    build_phase: str = "idle"
    emergency_flag: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class TurnSignalStatusInfo:
    state: str = "idle_off"


@dataclass
class HazardSignal:
    hazard_active: bool = False
    reason: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class LightCommand:
    hazard_light: bool = False
    brake_light: bool = False
    brake_flash_mode: str = "none"
    flash_frequency_hz: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class LightStatusReport:
    state: LightControlState = LightControlState.NORMAL_STANDBY
    hazard_active: bool = False
    brake_flash_mode: str = "none"
    brake_type: str = ""
    timestamp: float = field(default_factory=time.time)


CONTROL_PERIOD_S = 0.01
EMERGENCY_FLASH_FREQ_HZ = 5.0
BRAKE_LIGHT_ON_PRESSURE_MPA = 0.2
BRAKE_LIGHT_OFF_DELAY_S = 0.3
HAZARD_ACTIVATE_SPEED_KMH = 30.0
HAZARD_LOW_SPEED_KMH = 5.0
STANDSTILL_DURATION_S = 1.0


class HazardBrakeLightController:
    def __init__(self):
        self.module_id = "ad-mcc-23"
        self.module_name = "双闪与刹车灯控制单元"
        self.version = "V1.0"

        self.state = LightControlState.NORMAL_STANDBY
        self._hazard_active = False
        self._flash_mode = "none"
        self._brake_light_off_timer = 0.0
        self._standstill_timer = 0.0
        self._emergency_ended_timer = 0.0
        self._manual_hazard_override = False
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_brake_status = None
        self._query_speed = None
        self._query_degradation_level = None
        self._query_turn_signal_status = None
        self._query_manual_hazard = None

        self._publish_hazard_signal = None
        self._publish_light_command = None
        self._publish_status_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_brake_status_query(self, callback):
        self._query_brake_status = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_degradation_level_query(self, callback):
        self._query_degradation_level = callback

    def set_turn_signal_status_query(self, callback):
        self._query_turn_signal_status = callback

    def set_manual_hazard_query(self, callback):
        self._query_manual_hazard = callback

    def set_hazard_signal_publisher(self, callback):
        self._publish_hazard_signal = callback

    def set_light_command_publisher(self, callback):
        self._publish_light_command = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_control_cycle(self):
        now = time.time()
        if self.state == LightControlState.SYSTEM_PAUSED:
            return

        brake = self._query_brake_status() if self._query_brake_status else BrakeStatus()
        speed = self._query_speed() if self._query_speed else 0.0
        degradation = self._query_degradation_level() if self._query_degradation_level else DegradationLevel.NORMAL
        manual_hazard = self._query_manual_hazard() if self._query_manual_hazard else False

        if manual_hazard:
            self._manual_hazard_override = True
            self._hazard_active = True
            self._flash_mode = "none"
            self._send_light_command(hazard=True, brake_light=False, flash_mode="none")
            self._publish_hazard_if_needed(True, "手动双闪")
            return
        else:
            if self._manual_hazard_override:
                self._manual_hazard_override = False
                self._hazard_active = False

        prev_state = self.state
        prev_hazard = self._hazard_active

        if degradation.value >= 2:
            if speed == 0.0:
                self._standstill_timer += CONTROL_PERIOD_S
                if self._standstill_timer >= STANDSTILL_DURATION_S:
                    self.state = LightControlState.DEGRADED_HAZARD
                    self._hazard_active = True
                    self._flash_mode = "none"
            else:
                self._standstill_timer = 0.0
        else:
            if self.state == LightControlState.DEGRADED_HAZARD:
                self.state = LightControlState.NORMAL_STANDBY
                self._hazard_active = False
                self._flash_mode = "none"

        if brake.brake_type == BrakeType.EMERGENCY and brake.emergency_flag:
            self.state = LightControlState.EMERGENCY_BRAKE_FLASH
            self._flash_mode = "5Hz闪烁"
            if speed < HAZARD_LOW_SPEED_KMH:
                self._hazard_active = True
            elif speed < HAZARD_ACTIVATE_SPEED_KMH:
                self._hazard_active = True
            else:
                self._hazard_active = False
        elif self.state == LightControlState.EMERGENCY_BRAKE_FLASH:
            if speed < HAZARD_LOW_SPEED_KMH and brake.target_pressure_mpa < 0.2:
                self.state = LightControlState.HAZARD_HOLD
                self._hazard_active = True
                self._flash_mode = "none"
            else:
                self.state = LightControlState.NORMAL_STANDBY
                self._hazard_active = False
                self._flash_mode = "none"

        brake_light = False
        if self.state == LightControlState.NORMAL_STANDBY:
            if brake.target_pressure_mpa > BRAKE_LIGHT_ON_PRESSURE_MPA or brake.build_phase not in ("idle", "idle_depressurized"):
                brake_light = True
                self._brake_light_off_timer = 0.0
            elif self._brake_light_off_timer < BRAKE_LIGHT_OFF_DELAY_S:
                self._brake_light_off_timer += CONTROL_PERIOD_S
                brake_light = True
            else:
                brake_light = False
        elif self.state == LightControlState.EMERGENCY_BRAKE_FLASH:
            flash_period = 1.0 / EMERGENCY_FLASH_FREQ_HZ
            brake_light = (math.fmod(now, flash_period) < flash_period * 0.5)
        elif self.state in (LightControlState.DEGRADED_HAZARD, LightControlState.HAZARD_HOLD):
            brake_light = brake.target_pressure_mpa > 0.2

        self._send_light_command(
            hazard=self._hazard_active,
            brake_light=brake_light,
            flash_mode=self._flash_mode,
            flash_freq=EMERGENCY_FLASH_FREQ_HZ if self._flash_mode != "none" else 0.0
        )

        if self._hazard_active != prev_hazard:
            self._publish_hazard_if_needed(self._hazard_active, "自动控制")

        if self.state != prev_state or self._hazard_active != prev_hazard:
            if self._publish_status_report:
                self._publish_status_report(LightStatusReport(
                    state=self.state,
                    hazard_active=self._hazard_active,
                    brake_flash_mode=self._flash_mode,
                    brake_type=brake.brake_type.value,
                ))
            if self.state != prev_state and self._publish_event_log:
                self._publish_event_log({
                    "event": "light_state_change",
                    "from": prev_state.value,
                    "to": self.state.value,
                    "hazard": self._hazard_active,
                    "timestamp": now,
                })

    def _send_light_command(self, hazard, brake_light, flash_mode, flash_freq=0.0):
        if self._publish_light_command:
            self._publish_light_command(LightCommand(
                hazard_light=hazard,
                brake_light=brake_light,
                brake_flash_mode=flash_mode,
                flash_frequency_hz=flash_freq,
            ))

    def _publish_hazard_if_needed(self, active, reason):
        if self._publish_hazard_signal:
            self._publish_hazard_signal(HazardSignal(
                hazard_active=active,
                reason=reason,
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

    def get_state(self) -> LightControlState:
        return self.state

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def emergency_shutdown(self):
        self.state = LightControlState.SYSTEM_PAUSED
        self._send_light_command(hazard=False, brake_light=False, flash_mode="none")
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 双闪与刹车灯控制单元 (ad-mcc-23) 演示")
    print("=" * 70)

    ctrl = HazardBrakeLightController()
    ctrl.set_speed_query(lambda: 80.0)
    ctrl.set_degradation_level_query(lambda: DegradationLevel.NORMAL)
    ctrl.set_manual_hazard_query(lambda: False)

    print_separator("STEP 1: 日常制动")
    ctrl.set_brake_status_query(lambda: BrakeStatus(
        brake_type=BrakeType.GENTLE,
        target_pressure_mpa=2.0,
        emergency_flag=False
    ))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 2: 紧急制动触发")
    ctrl.set_brake_status_query(lambda: BrakeStatus(
        brake_type=BrakeType.EMERGENCY,
        target_pressure_mpa=8.0,
        emergency_flag=True
    ))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 3: 紧急制动后低速双闪")
    ctrl.set_speed_query(lambda: 3.0)
    ctrl.set_brake_status_query(lambda: BrakeStatus(
        brake_type=BrakeType.GENTLE,
        target_pressure_mpa=0.0,
        emergency_flag=False
    ))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print("\n✅ 双闪与刹车灯控制单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-23 双闪与刹车灯控制单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(speed=80.0, brake_type=BrakeType.GENTLE, pressure=2.0,
                       emergency=False, degradation=DegradationLevel.NORMAL, manual_hazard=False):
            c = HazardBrakeLightController()
            c.set_speed_query(lambda: speed)
            c.set_brake_status_query(lambda: BrakeStatus(
                brake_type=brake_type,
                target_pressure_mpa=pressure,
                emergency_flag=emergency
            ))
            c.set_degradation_level_query(lambda: degradation)
            c.set_manual_hazard_query(lambda: manual_hazard)
            return c

        print("\n[TC-M23-01] 日常制动刹车灯常亮")
        try:
            c = setup_ctrl()
            c.run_control_cycle()
            assert c.state == LightControlState.NORMAL_STANDBY
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M23-02] 紧急制动高频闪烁")
        try:
            c = setup_ctrl(brake_type=BrakeType.EMERGENCY, emergency=True, pressure=8.0)
            c.run_control_cycle()
            assert c.state == LightControlState.EMERGENCY_BRAKE_FLASH
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M23-03] 紧急制动低速激活双闪")
        try:
            c = setup_ctrl(speed=4.0, brake_type=BrakeType.EMERGENCY, emergency=True, pressure=8.0)
            c.run_control_cycle()
            assert c.state == LightControlState.EMERGENCY_BRAKE_FLASH
            assert c._hazard_active
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M23-04] 降级停车强制双闪")
        try:
            c = setup_ctrl(speed=0.0, degradation=DegradationLevel.LEVEL2)
            c._standstill_timer = 1.5
            c.run_control_cycle()
            assert c.state == LightControlState.DEGRADED_HAZARD
            assert c._hazard_active
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M23-05] 降级恢复关闭双闪")
        try:
            c = setup_ctrl(speed=0.0, degradation=DegradationLevel.NORMAL)
            c.state = LightControlState.DEGRADED_HAZARD
            c._hazard_active = True
            c.run_control_cycle()
            assert c.state == LightControlState.NORMAL_STANDBY
            assert not c._hazard_active
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M23-06] 紧急熔断")
        try:
            c = setup_ctrl()
            c.emergency_shutdown()
            assert c.state == LightControlState.SYSTEM_PAUSED
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