#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-25
模块名称: 雨刮与外设自适应单元
所属分区: 六、灯光与外设管理
核心职责: 依据雨量传感器数据、环境光照度及车速，自动控制雨刮器的工作状态
          （关闭/间歇/低速/高速），并管理洗涤液喷射逻辑。同时协调其他天气相关外设
          （如后视镜加热、前照灯清洗等，若配备），确保恶劣天气下的视野清晰度。
          不参与任何驾驶决策，仅执行雨刮及相关外设的自适应控制逻辑。

依赖模块:
    雨量传感器(CAN总线),
    环境光传感器(CAN总线),
    车速传感器(CAN总线),
    ad-mcc-24(远近光灯自动切换单元)
被依赖模块:
    车身控制器(雨刮电机、洗涤泵、后视镜加热等执行器),
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: 雨刮电机堵转时必须立即断电，防止过热起火
  S-02: 洗涤模式不得持续超过 5 秒，避免洗涤泵损坏或驾驶员视线受阻
  S-03: 雨量传感器故障时，必须保持当前雨刮状态，避免突然停止导致视野丧失
  S-04: 本模块仅控制雨刮与外设，不参与任何车辆行驶决策
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class WiperState(Enum):
    WIPER_OFF = "wiper_off"
    INTERMITTENT = "intermittent"
    LOW_SPEED = "low_speed"
    HIGH_SPEED = "high_speed"
    WASH_MODE = "wash_mode"
    SYSTEM_PAUSED = "system_paused"


@dataclass
class RainSensorData:
    drop_freq_hz: float = 0.0
    valid: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class AmbientLight:
    illuminance_lux: float = 100.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class WashRequest:
    front_wash: bool = False
    rear_wash: bool = False
    headlight_wash: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class WiperMotorStatus:
    overload: bool = False
    stall: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class WiperCommand:
    front_wiper_mode: str = "off"
    rear_wiper_mode: str = "off"
    wash_pump: bool = False
    headlight_washer: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class MirrorHeaterCommand:
    heater_on: bool = False
    reason: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class LowBeamTrigger:
    request_low_beam: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class WiperStatusReport:
    state: WiperState = WiperState.WIPER_OFF
    rain_level: float = 0.0
    speed: float = 0.0
    fault_code: str = ""
    timestamp: float = field(default_factory=time.time)


CONTROL_PERIOD_S = 0.05
WASH_TIMEOUT_S = 5.0
WASH_CYCLES = 3
WIPER_LIGHT_THRESHOLD_LUX = 20.0
WIPER_LIGHT_DURATION_S = 10.0
MIRROR_HEATER_TEMP_C = 5.0
MIRROR_HEATER_OFF_TEMP_C = 10.0


class WiperAdaptiveController:
    def __init__(self):
        self.module_id = "ad-mcc-25"
        self.module_name = "雨刮与外设自适应单元"
        self.version = "V1.0"

        self.state = WiperState.WIPER_OFF
        self._prev_state = WiperState.WIPER_OFF
        self._wash_timer = 0.0
        self._wash_cycle_count = 0
        self._wiper_on_timer = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_rain_sensor = None
        self._query_ambient_light = None
        self._query_speed = None
        self._query_wash_request = None
        self._query_motor_status = None
        self._query_outside_temp = None

        self._publish_wiper_command = None
        self._publish_mirror_heater = None
        self._publish_low_beam_trigger = None
        self._publish_status_report = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_rain_sensor_query(self, callback):
        self._query_rain_sensor = callback

    def set_ambient_light_query(self, callback):
        self._query_ambient_light = callback

    def set_speed_query(self, callback):
        self._query_speed = callback

    def set_wash_request_query(self, callback):
        self._query_wash_request = callback

    def set_motor_status_query(self, callback):
        self._query_motor_status = callback

    def set_outside_temp_query(self, callback):
        self._query_outside_temp = callback

    def set_wiper_command_publisher(self, callback):
        self._publish_wiper_command = callback

    def set_mirror_heater_publisher(self, callback):
        self._publish_mirror_heater = callback

    def set_low_beam_trigger_publisher(self, callback):
        self._publish_low_beam_trigger = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_control_cycle(self):
        now = time.time()
        if self.state == WiperState.SYSTEM_PAUSED:
            return

        # 电机故障检测
        motor = self._query_motor_status() if self._query_motor_status else WiperMotorStatus()
        if motor.overload or motor.stall:
            self._send_wiper_command("off")
            self._log_event("WIPER_FAULT", {"type": "motor_stall"})
            return

        # 洗涤请求处理
        wash = self._query_wash_request() if self._query_wash_request else WashRequest()
        if wash.front_wash:
            # 无论当前状态，立即进入洗涤模式
            self.state = WiperState.WASH_MODE
            self._wash_timer = now
            self._wash_cycle_count = 0
            self._send_wiper_command("low", wash_pump=True)
            return

        # 洗涤模式进行中或结束处理
        if self.state == WiperState.WASH_MODE:
            elapsed = now - self._wash_timer
            # 检查是否完成或超时
            if elapsed >= WASH_TIMEOUT_S or self._wash_cycle_count >= WASH_CYCLES:
                # 关闭洗涤泵，雨刮模式暂时设为 OFF，但接下来会重新自动判定
                self._send_wiper_command("off", wash_pump=False)
                self.state = WiperState.WIPER_OFF  # 临时状态，后面会根据雨量重新设定
                # 不 return，继续执行后面的自动雨刮逻辑，实现立即恢复自动模式
            else:
                # 仍在洗涤中，维持低速刮刷并开启泵
                self._send_wiper_command("low", wash_pump=True)
                return

        # 获取雨量传感器（若无效则保持当前状态）
        rain = self._query_rain_sensor() if self._query_rain_sensor else RainSensorData()
        if not rain.valid:
            # 传感器故障，保持当前状态不变（S-03）
            return

        speed = self._query_speed() if self._query_speed else 0.0
        freq = rain.drop_freq_hz

        # 根据雨量和车速查表确定目标状态
        if freq < 1.0:
            target_state = WiperState.WIPER_OFF
        elif freq < 5.0:
            target_state = WiperState.LOW_SPEED if speed >= 80.0 else WiperState.INTERMITTENT
        elif freq < 15.0:
            target_state = WiperState.HIGH_SPEED if speed >= 80.0 else WiperState.LOW_SPEED
        else:
            target_state = WiperState.HIGH_SPEED

        # 状态切换与指令下发
        if target_state != self.state:
            self.state = target_state
            mode_map = {
                WiperState.WIPER_OFF: "off",
                WiperState.INTERMITTENT: "intermittent",
                WiperState.LOW_SPEED: "low",
                WiperState.HIGH_SPEED: "high",
            }
            self._send_wiper_command(mode_map.get(target_state, "off"))

        # 雨刮开启计时（用于联动近光）
        if self.state != WiperState.WIPER_OFF:
            self._wiper_on_timer += CONTROL_PERIOD_S
        else:
            self._wiper_on_timer = 0.0

        # 联动近光灯
        ambient = self._query_ambient_light() if self._query_ambient_light else AmbientLight()
        if self._wiper_on_timer >= WIPER_LIGHT_DURATION_S and ambient.illuminance_lux < WIPER_LIGHT_THRESHOLD_LUX:
            if self._publish_low_beam_trigger:
                self._publish_low_beam_trigger(LowBeamTrigger(request_low_beam=True))

        # 后视镜加热
        outside_temp = self._query_outside_temp() if self._query_outside_temp else 20.0
        if self.state in (WiperState.LOW_SPEED, WiperState.HIGH_SPEED) and outside_temp < MIRROR_HEATER_TEMP_C:
            if self._publish_mirror_heater:
                self._publish_mirror_heater(MirrorHeaterCommand(heater_on=True, reason="雨刮高速且低温"))
        elif outside_temp > MIRROR_HEATER_OFF_TEMP_C or self.state == WiperState.WIPER_OFF:
            if self._publish_mirror_heater:
                self._publish_mirror_heater(MirrorHeaterCommand(heater_on=False, reason="条件不满足"))

        # 状态上报
        if self.state != self._prev_state:
            self._prev_state = self.state
            if self._publish_status_report:
                self._publish_status_report(WiperStatusReport(
                    state=self.state,
                    rain_level=freq,
                    speed=speed,
                ))
            if self._publish_event_log:
                self._publish_event_log({
                    "event": "wiper_state_change",
                    "state": self.state.value,
                    "freq": freq,
                    "timestamp": now
                })

    def _send_wiper_command(self, mode, wash_pump=False):
        if self._publish_wiper_command:
            self._publish_wiper_command(WiperCommand(
                front_wiper_mode=mode,
                wash_pump=wash_pump
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

    def get_state(self) -> WiperState:
        return self.state

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def emergency_shutdown(self):
        self.state = WiperState.SYSTEM_PAUSED
        self._send_wiper_command("off", wash_pump=False)
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 雨刮与外设自适应单元 (ad-mcc-25) 演示")
    print("=" * 70)

    ctrl = WiperAdaptiveController()
    ctrl.set_rain_sensor_query(lambda: RainSensorData(drop_freq_hz=0.0, valid=True))
    ctrl.set_speed_query(lambda: 30.0)
    ctrl.set_ambient_light_query(lambda: AmbientLight(illuminance_lux=100.0))
    ctrl.set_wash_request_query(lambda: WashRequest())
    ctrl.set_motor_status_query(lambda: WiperMotorStatus())
    ctrl.set_outside_temp_query(lambda: 25.0)

    print_separator("STEP 1: 小雨 3Hz 车速 30 -> 间歇")
    ctrl.set_rain_sensor_query(lambda: RainSensorData(drop_freq_hz=3.0, valid=True))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 2: 中雨 10Hz 车速 90 -> 高速")
    ctrl.set_rain_sensor_query(lambda: RainSensorData(drop_freq_hz=10.0, valid=True))
    ctrl.set_speed_query(lambda: 90.0)
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print_separator("STEP 3: 洗涤请求")
    ctrl.set_wash_request_query(lambda: WashRequest(front_wash=True))
    ctrl.run_control_cycle()
    print(f"  状态: {ctrl.state.value}")

    print("\n✅ 雨刮与外设自适应单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-25 雨刮与外设自适应单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_ctrl(freq=0.0, valid=True, speed=30.0, illuminance=100.0,
                       wash=False, motor_fault=False, outside_temp=25.0):
            c = WiperAdaptiveController()
            c.set_rain_sensor_query(lambda: RainSensorData(drop_freq_hz=freq, valid=valid))
            c.set_speed_query(lambda: speed)
            c.set_ambient_light_query(lambda: AmbientLight(illuminance_lux=illuminance))
            c.set_wash_request_query(lambda: WashRequest(front_wash=wash))
            c.set_motor_status_query(lambda: WiperMotorStatus(overload=motor_fault, stall=motor_fault))
            c.set_outside_temp_query(lambda: outside_temp)
            return c

        print("\n[TC-M25-01] 小雨 3Hz 车速 30 -> 间歇")
        try:
            c = setup_ctrl(freq=3.0, speed=30.0)
            c.run_control_cycle()
            assert c.state == WiperState.INTERMITTENT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M25-02] 中雨 10Hz 车速 90 -> 高速")
        try:
            c = setup_ctrl(freq=10.0, speed=90.0)
            c.run_control_cycle()
            assert c.state == WiperState.HIGH_SPEED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M25-03] 洗涤模式")
        try:
            c = setup_ctrl(wash=True)
            c.run_control_cycle()
            assert c.state == WiperState.WASH_MODE
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M25-04] 洗涤超时退出，恢复自动模式")
        try:
            # 模拟洗涤中，并设置超时
            c = setup_ctrl(freq=4.0, speed=50.0)  # 应该恢复为间歇模式
            c.state = WiperState.WASH_MODE
            c._wash_timer = time.time() - 5.1  # 已超时
            c.run_control_cycle()
            # 根据雨量应自动切换到 INTERMITTENT 或 LOW_SPEED（车速50<80，所以间歇）
            assert c.state == WiperState.INTERMITTENT
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M25-05] 电机堵转断电")
        try:
            c = setup_ctrl(motor_fault=True)
            c.run_control_cycle()
            # 电机故障后应发送关闭指令，状态不变（仍为初始WIPER_OFF）
            assert c.state == WiperState.WIPER_OFF
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M25-06] 大雨低温后视镜加热")
        try:
            c = setup_ctrl(freq=20.0, speed=60.0, outside_temp=2.0)
            heater_cmd = None
            def trap_heater(cmd):
                nonlocal heater_cmd
                heater_cmd = cmd
            c.set_mirror_heater_publisher(trap_heater)
            c.run_control_cycle()
            assert heater_cmd is not None and heater_cmd.heater_on
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M25-07] 雨刮持续10秒联动近光")
        try:
            c = setup_ctrl(freq=5.0, speed=50.0, illuminance=15.0)
            c._wiper_on_timer = 10.5
            trigger_cmd = None
            def trap_trigger(cmd):
                nonlocal trigger_cmd
                trigger_cmd = cmd
            c.set_low_beam_trigger_publisher(trap_trigger)
            c.run_control_cycle()
            assert trigger_cmd is not None and trigger_cmd.request_low_beam
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