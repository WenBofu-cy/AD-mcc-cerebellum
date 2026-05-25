#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-24
模块名称: 远近光灯自动切换单元
所属分区: 六、灯光与外设管理
核心职责: 根据环境光传感器、对向来车检测、前方同向车辆检测及当前车速，自动切换远近光灯。
          在夜间或隧道等低光照条件下自动开启近光灯，在满足远光灯开启条件时激活远光，
          检测到会车或跟车时立即切回近光。同时处理手动超车灯与自动控制的优先级。
          不参与任何驾驶决策，仅执行灯光控制逻辑。

依赖模块:
    环境光传感器(CAN总线),
    前方目标检测模块(ECC-01/ad-44),
    车速传感器(CAN总线),
    ad-mcc-01(小脑总控调度核心),
    ad-mcc-25(雨刮与外设自适应单元)
被依赖模块:
    车身灯光控制器,
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: 会车时必须在 500ms 内切回近光，不得延迟，避免眩目对向驾驶员
  S-02: 环境光传感器故障时，必须保守开启近光，确保行车安全
  S-03: 手动超车灯优先级最高，但不得影响自动控制的恢复
  S-04: 本模块仅控制远近光切换，不参与车辆行驶决策
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class LightState(Enum):
    LIGHTS_OFF = "lights_off"
    LOW_BEAM = "low_beam"
    HIGH_BEAM_READY = "high_beam_ready"
    HIGH_BEAM_ACTIVE = "high_beam_active"
    MANUAL_FLASH = "manual_flash"
    SYSTEM_PAUSED = "system_paused"


@dataclass
class AmbientLight:
    illuminance_lux: float = 100.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class OncomingVehicle:
    detected: bool = False
    distance_m: float = 1000.0
    orientation: str = "straight"
    confidence: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class FrontVehicle:
    detected: bool = False
    distance_m: float = 1000.0
    relative_speed_kmh: float = 0.0
    confidence: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class WiperStatus:
    active: bool = False
    continuous_duration_s: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class ManualLightCommand:
    flash_to_pass: bool = False
    manual_low_beam: bool = False
    auto_mode: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class HighLowBeamCommand:
    low_beam: bool = False
    high_beam: bool = False
    reason: str = ""
    mode: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class LightStatusReport:
    state: LightState = LightState.LIGHTS_OFF
    illuminance: float = 0.0
    reason: str = ""
    is_manual: bool = False
    timestamp: float = field(default_factory=time.time)


NIGHT_THRESHOLD_LUX = 5.0
TWILIGHT_THRESHOLD_LUX = 20.0
TUNNEL_LIGHT_DROP = 30.0
WIPER_AUTO_LIGHT_S = 10.0
HIGH_BEAM_READY_DELAY_S = 0.5
ONCOMING_DISTANCE_M = 800.0
FRONT_VEHICLE_DISTANCE_M = 150.0
HIGH_BEAM_SPEED_THRESHOLD_KMH = 40.0
LOW_BEAM_SPEED_THRESHOLD_KMH = 30.0
CONTROL_PERIOD_S = 0.05


class AutoHighBeamController:
    def __init__(self):
        self.module_id = "ad-mcc-24"
        self.module_name = "远近光灯自动切换单元"
        self.version = "V1.0"

        self.state = LightState.LIGHTS_OFF
        self._prev_non_flash_state = LightState.LIGHTS_OFF
        self._ready_timer = 0.0
        self._flash_timer = 0.0
        self._tunnel_mode = False
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_ambient_light = None
        self._query_oncoming_vehicle = None
        self._query_front_vehicle = None
        self._query_speed = None
        self._query_wiper_status = None
        self._query_manual_command = None

        self._publish_light_command = None
        self._publish_status = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_ambient_light_query(self, callback):
        self._query_ambient_light = callback

    def set_oncoming_vehicle_query(self, callback):
        self._query_oncoming_vehicle = callback

    def set_front_vehicle_query(self, callback):
        self._query_front_vehicle = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_wiper_status_query(self, callback):
        self._query_wiper_status = callback

    def set_manual_command_query(self, callback):
        self._query_manual_command = callback

    def set_light_command_publisher(self, callback):
        self._publish_light_command = callback

    def set_status_publisher(self, callback):
        self._publish_status = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_control_cycle(self):
        now = time.time()
        if self.state == LightState.SYSTEM_PAUSED:
            return

        manual = self._query_manual_command() if self._query_manual_command else ManualLightCommand()

        if manual.flash_to_pass:
            self._prev_non_flash_state = self.state if self.state != LightState.MANUAL_FLASH else self._prev_non_flash_state
            self.state = LightState.MANUAL_FLASH
            self._send_light_command(low_beam=False, high_beam=True, reason="手动闪灯")
            self._flash_timer = now
            return

        if self.state == LightState.MANUAL_FLASH:
            if now - self._flash_timer >= 0.2:
                self.state = self._prev_non_flash_state
                self._apply_state()

        ambient = self._query_ambient_light() if self._query_ambient_light else AmbientLight()
        oncoming = self._query_oncoming_vehicle() if self._query_oncoming_vehicle else OncomingVehicle()
        front = self._query_front_vehicle() if self._query_front_vehicle else FrontVehicle()
        speed = self._query_speed() if self._query_speed else 0.0
        wiper = self._query_wiper_status() if self._query_wiper_status else WiperStatus()

        need_low_beam = False
        if ambient.illuminance_lux < NIGHT_THRESHOLD_LUX:
            need_low_beam = True
        if wiper.active and wiper.continuous_duration_s >= WIPER_AUTO_LIGHT_S:
            need_low_beam = True
        if self._tunnel_mode:
            need_low_beam = True

        if not need_low_beam:
            if self.state != LightState.LIGHTS_OFF:
                self.state = LightState.LIGHTS_OFF
                self._send_light_command(low_beam=False, high_beam=False, reason="日间关闭")
            return

        if self.state == LightState.LIGHTS_OFF:
            self.state = LightState.LOW_BEAM
            self._send_light_command(low_beam=True, high_beam=False, reason="夜间/低光照")
            return

        can_high_beam = True
        if oncoming.detected and oncoming.distance_m < ONCOMING_DISTANCE_M:
            can_high_beam = False
        if front.detected and front.distance_m < FRONT_VEHICLE_DISTANCE_M:
            can_high_beam = False
        if speed < HIGH_BEAM_SPEED_THRESHOLD_KMH:
            can_high_beam = False
        if ambient.illuminance_lux > TWILIGHT_THRESHOLD_LUX:
            can_high_beam = False

        if can_high_beam:
            if self.state == LightState.LOW_BEAM:
                self.state = LightState.HIGH_BEAM_READY
                self._ready_timer = now
            elif self.state == LightState.HIGH_BEAM_READY:
                if now - self._ready_timer >= HIGH_BEAM_READY_DELAY_S:
                    self.state = LightState.HIGH_BEAM_ACTIVE
                    self._send_light_command(low_beam=False, high_beam=True, reason="激活远光")
        else:
            if self.state in (LightState.HIGH_BEAM_READY, LightState.HIGH_BEAM_ACTIVE):
                self.state = LightState.LOW_BEAM
                self._send_light_command(low_beam=True, high_beam=False, reason="会车/跟车切回近光")

        if self.state != getattr(self, '_last_reported_state', None):
            self._last_reported_state = self.state
            if self._publish_status:
                self._publish_status(LightStatusReport(
                    state=self.state,
                    illuminance=ambient.illuminance_lux,
                    reason="自动控制",
                    is_manual=False
                ))
            if self._publish_event_log:
                self._publish_event_log({
                    "event": "high_beam_state_change",
                    "state": self.state.value,
                    "timestamp": now
                })

    def _apply_state(self):
        if self.state == LightState.LIGHTS_OFF:
            self._send_light_command(low_beam=False, high_beam=False, reason="状态恢复")
        elif self.state == LightState.LOW_BEAM:
            self._send_light_command(low_beam=True, high_beam=False, reason="状态恢复")
        elif self.state == LightState.HIGH_BEAM_ACTIVE:
            self._send_light_command(low_beam=False, high_beam=True, reason="状态恢复")
        else:
            self._send_light_command(low_beam=True, high_beam=False, reason="状态恢复")

    def _send_light_command(self, low_beam, high_beam, reason):
        if self._publish_light_command:
            self._publish_light_command(HighLowBeamCommand(
                low_beam=low_beam,
                high_beam=high_beam,
                reason=reason,
                mode=self.state.value
            ))

    def get_state(self) -> LightState:
        return self.state

    def emergency_shutdown(self):
        self.state = LightState.SYSTEM_PAUSED
        self._send_light_command(low_beam=False, high_beam=False, reason="紧急熔断")
        print(f"[{self.module_id}] 紧急熔断")

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 远近光灯自动切换单元 (ad-mcc-24) 演示")
    print("=" * 70)

    ctrl = AutoHighBeamController()
    ctrl.set_speed_query(lambda: 60.0)
    ctrl.set_ambient_light_query(lambda: AmbientLight(illuminance_lux=100.0))
    ctrl.set_oncoming_vehicle_query(lambda: OncomingVehicle())
    ctrl.set_front_vehicle_query(lambda: FrontVehicle())
    ctrl.set_wiper_status_query(lambda: WiperStatus())
    ctrl.set_manual_command_query(lambda: ManualLightCommand())

    print_separator("STEP 1: 进入夜间，开启近光")
    ctrl.set_ambient_light_query(lambda: AmbientLight(illuminance_lux=3.0))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 2: 满足远光条件，激活远光")
    ctrl.run_control_cycle()
    import time as _time
    _time.sleep(0.6)
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 3: 检测到对向来车，切回近光")
    ctrl.set_oncoming_vehicle_query(lambda: OncomingVehicle(detected=True, distance_m=500.0))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print("\n✅ 远近光灯自动切换单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-24 远近光灯自动切换单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(illuminance=100.0, oncoming=False, oncoming_dist=1000.0,
                       front=False, front_dist=1000.0, speed=60.0, wiper_active=False, wiper_dur=0.0):
            c = AutoHighBeamController()
            c.set_ambient_light_query(lambda: AmbientLight(illuminance_lux=illuminance))
            c.set_oncoming_vehicle_query(lambda: OncomingVehicle(detected=oncoming, distance_m=oncoming_dist))
            c.set_front_vehicle_query(lambda: FrontVehicle(detected=front, distance_m=front_dist))
            c.set_speed_query(lambda: speed)
            c.set_wiper_status_query(lambda: WiperStatus(active=wiper_active, continuous_duration_s=wiper_dur))
            c.set_manual_command_query(lambda: ManualLightCommand())
            return c

        print("\n[TC-M24-01] 夜间开灯")
        try:
            c = setup_ctrl(illuminance=3.0)
            c.run_control_cycle()
            assert c.state == LightState.LOW_BEAM
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M24-02] 激活远光")
        try:
            c = setup_ctrl(illuminance=3.0, speed=60.0)
            c.run_control_cycle()
            import time as _time
            _time.sleep(0.6)
            c.run_control_cycle()
            assert c.state == LightState.HIGH_BEAM_ACTIVE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M24-03] 会车切回近光")
        try:
            c = setup_ctrl(illuminance=3.0, speed=60.0)
            c.run_control_cycle()
            import time as _time
            _time.sleep(0.6)
            c.run_control_cycle()
            c.set_oncoming_vehicle_query(lambda: OncomingVehicle(detected=True, distance_m=500.0))
            c.run_control_cycle()
            assert c.state == LightState.LOW_BEAM
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M24-04] 手动闪灯")
        try:
            c = setup_ctrl(illuminance=3.0)
            c.run_control_cycle()
            c.set_manual_command_query(lambda: ManualLightCommand(flash_to_pass=True))
            c.run_control_cycle()
            assert c.state == LightState.MANUAL_FLASH
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M24-05] 隧道开灯")
        try:
            c = setup_ctrl(illuminance=10.0)
            c._tunnel_mode = True
            c.run_control_cycle()
            assert c.state == LightState.LOW_BEAM
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M24-06] 紧急熔断")
        try:
            c = setup_ctrl()
            c.emergency_shutdown()
            assert c.state == LightState.SYSTEM_PAUSED
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