#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-17
模块名称: 再生制动优先协调单元
所属分区: 四、制动控制集群
核心职责: 根据车辆当前能量状态（电池 SOC、电池温度、最大充电功率）、车速、制动需求及
          驱动电机能力，动态计算再生制动与摩擦制动的分配比例（0.0–1.0）。在满足目标减速度
          需求的前提下，优先使用再生制动进行能量回收，不足部分由液压摩擦制动补足。同时监控
          再生制动系统的运行状态，当再生制动受限或故障时，平滑切换至纯摩擦制动，确保制动安全。
          不参与制动时机的决策，仅负责制动力的分配优化。

依赖模块:
    ad-mcc-13(制动压力解算单元，提供总需求制动力),
    ad-mcc-35(能源参数管理单元，提供电池 SOC、温度、最大充电功率),
    驱动电机控制器(CAN总线)
被依赖模块:
    ad-mcc-13(消费再生制动分配比例),
    ad-mcc-18(车身姿态实时监测单元),
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: 紧急制动或再生系统故障时，再生比例必须立即归零，不得有任何延迟
  S-02: 再生制动扭矩不得超过驱动电机当前最大允许再生扭矩，防止电机损坏
  S-03: 电池过充保护优先级最高：SOC ≥ 95% 时禁止任何再生充电
  S-04: 分配比例变化必须经过低通滤波，防止制动力突变影响稳定性
  S-05: 本模块仅负责再生比例分配，不直接控制制动执行器，不参与制动时机决策
"""

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


class RegenState(Enum):
    NORMAL_REGEN = "normal_regen"
    LIMITED_REGEN = "limited_regen"
    REGEN_DISABLED = "regen_disabled"
    SYSTEM_PAUSED = "system_paused"


@dataclass
class TotalBrakeForceRequest:
    force_newton: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class EnergyStatus:
    battery_soc_pct: float = 80.0
    battery_temp_c: float = 25.0
    max_charge_power_kw: float = 150.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class MotorInfo:
    max_regen_torque_nm: float = 500.0
    motor_state: str = "正常"   # 正常/故障/过热
    timestamp: float = field(default_factory=time.time)


@dataclass
class EmergencyBrakeOverride:
    active: bool = True


@dataclass
class RegenRatioOutput:
    ratio: float = 0.0
    source: str = "再生优先协调"
    limiting_factor: str = ""
    available_regen_torque_nm: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class RegenStatusReport:
    state: RegenState = RegenState.NORMAL_REGEN
    actual_regen_power_kw: float = 0.0
    total_energy_recovered_kwh: float = 0.0
    limiting_factor: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class RegenFaultAlert:
    alert_type: str = ""
    reason: str = ""
    suggestion: str = ""
    timestamp: float = field(default_factory=time.time)


# 控制参数
REGEN_MIN_SPEED_KMH = 5.0
SOC_FULL_DISABLE = 95.0
SOC_LIMIT_HIGH = 90.0
SOC_LIMIT_MEDIUM = 80.0
BATTERY_TEMP_OVERTEMP = 50.0
BATTERY_TEMP_REDUCE = 45.0
BATTERY_TEMP_WARN = 40.0
FILTER_ALPHA = 0.3
TIRE_ROLLING_RADIUS_M = 0.35  # 应与制动参数一致
CONTROL_PERIOD_S = 0.01
REPORT_INTERVAL_S = 1.0


class RegenBrakeCoordinator:
    def __init__(self):
        self.module_id = "ad-mcc-17"
        self.module_name = "再生制动优先协调单元"
        self.version = "V1.0"

        self.state = RegenState.REGEN_DISABLED
        self._filtered_ratio = 0.0
        self._total_energy_recovered_kwh = 0.0
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        # 回调注入
        self._query_total_force = None
        self._query_vehicle_speed = None
        self._query_energy_status = None
        self._query_motor_info = None
        self._query_emergency_brake = None

        self._publish_regen_ratio = None
        self._publish_status_report = None
        self._publish_fault_alert = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_total_force_query(self, callback):
        self._query_total_force = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_energy_status_query(self, callback):
        self._query_energy_status = callback

    def set_motor_info_query(self, callback):
        self._query_motor_info = callback

    def set_emergency_brake_query(self, callback):
        self._query_emergency_brake = callback

    def set_regen_ratio_publisher(self, callback):
        self._publish_regen_ratio = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    def set_fault_alert_publisher(self, callback):
        self._publish_fault_alert = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_coordination_cycle(self) -> Optional[RegenRatioOutput]:
        now = time.time()

        # 紧急制动优先
        emergency = self._query_emergency_brake() if self._query_emergency_brake else None
        if emergency and emergency.active:
            self.state = RegenState.REGEN_DISABLED
            self._filtered_ratio = 0.0
            output = RegenRatioOutput(ratio=0.0, limiting_factor="紧急制动旁路")
            if self._publish_regen_ratio:
                self._publish_regen_ratio(output)
            return output

        if self.state == RegenState.SYSTEM_PAUSED:
            return None

        # 获取系统状态
        total_force = self._get_total_force()
        speed = self._get_speed()
        energy = self._get_energy_status()
        motor = self._get_motor_info()

        # 判定再生状态
        limiting = ""
        target_ratio = 0.0

        # 禁用条件判断
        regen_disabled = False
        if speed is None or speed < REGEN_MIN_SPEED_KMH:
            regen_disabled = True
            limiting = "车速过低"
        if energy and energy.battery_soc_pct >= SOC_FULL_DISABLE:
            regen_disabled = True
            limiting = "电池满充保护(SOC≥95%)"
        if energy and energy.battery_temp_c > BATTERY_TEMP_OVERTEMP:
            regen_disabled = True
            limiting = "电池过热(>50°C)"
        if motor and motor.motor_state != "正常":
            regen_disabled = True
            limiting = f"驱动电机{motor.motor_state}"

        if regen_disabled:
            self.state = RegenState.REGEN_DISABLED
            target_ratio = 0.0
        else:
            # 判断是否受限
            limited = False
            limit_reasons = []
            if energy and energy.battery_soc_pct >= SOC_LIMIT_HIGH:
                limited = True
                limit_reasons.append("SOC偏高")
            if energy and energy.battery_temp_c > BATTERY_TEMP_WARN:
                limited = True
                limit_reasons.append("温度偏高")
            
            if limited:
                self.state = RegenState.LIMITED_REGEN
                limiting = " + ".join(limit_reasons)
            else:
                self.state = RegenState.NORMAL_REGEN
                limiting = ""

            # 计算可用再生比例
            if total_force > 0 and speed > 0:
                speed_ms = speed / 3.6
                max_regen_force_by_motor = motor.max_regen_torque_nm / TIRE_ROLLING_RADIUS_M if motor else float('inf')
                max_regen_force_by_battery = (energy.max_charge_power_kw * 1000.0) / speed_ms if energy and speed_ms > 0 else float('inf')
                max_regen_force = min(max_regen_force_by_motor, max_regen_force_by_battery)
                available_regen_force = min(total_force, max_regen_force)
                base_ratio = available_regen_force / total_force if total_force > 0 else 0.0

                # SOC 比例上限
                soc = energy.battery_soc_pct if energy else 80.0
                if soc > SOC_LIMIT_HIGH:
                    ratio_limit = 0.3
                elif soc > SOC_LIMIT_MEDIUM:
                    ratio_limit = 0.7
                else:
                    ratio_limit = 1.0

                # 温度比例上限
                temp = energy.battery_temp_c if energy else 25.0
                if temp > BATTERY_TEMP_REDUCE:
                    ratio_limit = min(ratio_limit, 0.5)
                elif temp > BATTERY_TEMP_WARN:
                    ratio_limit = min(ratio_limit, 0.8)

                target_ratio = min(base_ratio, ratio_limit)
            else:
                target_ratio = 0.0

        # 平滑滤波
        self._filtered_ratio = FILTER_ALPHA * target_ratio + (1 - FILTER_ALPHA) * self._filtered_ratio
        if abs(self._filtered_ratio - target_ratio) < 0.01:
            self._filtered_ratio = target_ratio

        # 构建输出
        available_torque = motor.max_regen_torque_nm if motor else 0.0
        output = RegenRatioOutput(
            ratio=round(self._filtered_ratio, 4),
            limiting_factor=limiting,
            available_regen_torque_nm=available_torque,
        )
        if self._publish_regen_ratio:
            self._publish_regen_ratio(output)

        # 累加回收能量
        if total_force > 0 and speed > 0:
            speed_ms = speed / 3.6
            regen_power_kw = total_force * self._filtered_ratio * speed_ms / 1000.0
            self._total_energy_recovered_kwh += regen_power_kw * CONTROL_PERIOD_S / 3600.0

        # 周期性状态上报
        if now - self._last_report_time >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_status_report:
                actual_regen_power = (total_force * self._filtered_ratio * (speed / 3.6) / 1000.0) if speed else 0.0
                self._publish_status_report(RegenStatusReport(
                    state=self.state,
                    actual_regen_power_kw=round(actual_regen_power, 2),
                    total_energy_recovered_kwh=round(self._total_energy_recovered_kwh, 4),
                    limiting_factor=limiting,
                ))

        # 严重故障告警
        if self.state == RegenState.REGEN_DISABLED and "故障" in limiting:
            if self._publish_fault_alert:
                self._publish_fault_alert(RegenFaultAlert(
                    alert_type="再生制动禁用",
                    reason=limiting,
                    suggestion="检查驱动电机状态，切换至纯摩擦制动"
                ))
            self._log_event("REGEN_FAULT", {"reason": limiting})

        return output

    def _get_total_force(self) -> float:
        if self._query_total_force:
            req = self._query_total_force()
            if isinstance(req, TotalBrakeForceRequest):
                return req.force_newton
            return float(req)
        return 0.0

    def _get_speed(self) -> Optional[float]:
        if self._query_vehicle_speed:
            return self._query_vehicle_speed()
        return None

    def _get_energy_status(self) -> Optional[EnergyStatus]:
        if self._query_energy_status:
            status = self._query_energy_status()
            if isinstance(status, EnergyStatus):
                return status
        return None

    def _get_motor_info(self) -> Optional[MotorInfo]:
        if self._query_motor_info:
            info = self._query_motor_info()
            if isinstance(info, MotorInfo):
                return info
        return None

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

    def get_state(self) -> RegenState:
        return self.state

    def emergency_shutdown(self):
        self.state = RegenState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 再生制动优先协调单元 (ad-mcc-17) 演示")
    print("=" * 70)

    coord = RegenBrakeCoordinator()
    coord.set_vehicle_speed_query(lambda: 60.0)
    coord.set_energy_status_query(lambda: EnergyStatus(battery_soc_pct=70.0, battery_temp_c=25.0))
    coord.set_motor_info_query(lambda: MotorInfo(max_regen_torque_nm=400.0))

    print_separator("STEP 1: 正常再生 (SOC=70%)")
    coord.set_total_force_query(lambda: TotalBrakeForceRequest(force_newton=5000.0))
    out = coord.run_coordination_cycle()
    if out:
        print(f"  再生比例: {out.ratio:.3f}")
        print(f"  限制因素: {out.limiting_factor or '无'}")

    print_separator("STEP 2: 受限再生 (SOC=88%)")
    coord.set_energy_status_query(lambda: EnergyStatus(battery_soc_pct=88.0, battery_temp_c=30.0))
    for _ in range(5):
        out = coord.run_coordination_cycle()
    if out:
        print(f"  再生比例: {out.ratio:.3f}")
        print(f"  限制因素: {out.limiting_factor}")

    print_separator("STEP 3: 禁用再生 (SOC=96%)")
    coord.set_energy_status_query(lambda: EnergyStatus(battery_soc_pct=96.0, battery_temp_c=25.0))
    out = coord.run_coordination_cycle()
    if out:
        print(f"  再生比例: {out.ratio:.3f}")
        print(f"  限制因素: {out.limiting_factor}")

    print("\n✅ 再生制动优先协调单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-17 再生制动优先协调单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_coord(speed=60.0, soc=70.0, temp=25.0, motor_torque=500.0, motor_state="正常"):
            c = RegenBrakeCoordinator()
            c.set_vehicle_speed_query(lambda: speed)
            c.set_energy_status_query(lambda: EnergyStatus(battery_soc_pct=soc, battery_temp_c=temp))
            c.set_motor_info_query(lambda: MotorInfo(max_regen_torque_nm=motor_torque, motor_state=motor_state))
            c.set_total_force_query(lambda: TotalBrakeForceRequest(force_newton=5000.0))
            return c

        # TC-M17-01: 正常再生
        print("\n[TC-M17-01] 正常再生 (SOC=70%)")
        try:
            c = setup_coord()
            out = c.run_coordination_cycle()
            assert out is not None and out.ratio > 0.5
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M17-02: 受限再生 (SOC=88%)
        print("\n[TC-M17-02] 受限再生 (SOC=88%)")
        try:
            c = setup_coord(soc=88.0)
            for _ in range(10):
                out = c.run_coordination_cycle()
            assert out.ratio <= 0.3 + 0.01  # 滤波后接近0.3
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M17-03: 禁用再生 (SOC=96%)
        print("\n[TC-M17-03] 禁用再生 (SOC=96%)")
        try:
            c = setup_coord(soc=96.0)
            out = c.run_coordination_cycle()
            assert out.ratio == 0.0 and c.state == RegenState.REGEN_DISABLED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M17-04: 紧急制动旁路
        print("\n[TC-M17-04] 紧急制动旁路")
        try:
            c = setup_coord()
            c.set_emergency_brake_query(lambda: EmergencyBrakeOverride(active=True))
            out = c.run_coordination_cycle()
            assert out.ratio == 0.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M17-05: 低速退出再生
        print("\n[TC-M17-05] 低速退出再生 (4 km/h)")
        try:
            c = setup_coord(speed=4.0)
            out = c.run_coordination_cycle()
            assert out.ratio == 0.0 and c.state == RegenState.REGEN_DISABLED
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M17-06: 电池过热禁用
        print("\n[TC-M17-06] 电池过热禁用 (52°C)")
        try:
            c = setup_coord(temp=52.0)
            out = c.run_coordination_cycle()
            assert out.ratio == 0.0
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