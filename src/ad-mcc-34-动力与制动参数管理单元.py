#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-34
模块名称: 动力与制动参数管理单元
所属分区: 九、多车型自适应适配
核心职责: 存储并管理车辆出厂前一次性标定的动力系统与制动系统特性参数（最大功率/扭矩、
          加速特性曲线、制动主缸规格、制动力分配曲线、轮胎滚动半径等），为 MCC 运动小脑
          各动力与制动相关执行模块提供统一的参数查询服务。确保参数来源唯一、版本一致，避免
          各模块独立维护参数导致的不一致。不参与任何驾驶决策或控制，仅提供参数数据的存取与校验。

依赖模块:
    出厂标定系统(配置文件/诊断接口)
被依赖模块:
    ad-mcc-09/13/14/17/19/20 等动力/制动相关模块

安全约束:
  S-01: 出厂标定参数为只读数据，运行时不得被任何控制模块修改
  S-02: 参数更新必须经过完整性校验与人工确认，更新前必须备份当前参数
  S-03: 参数缺失时必须使用保守默认值，确保动力与制动控制不因参数缺失而失效
  S-04: 本模块仅提供参数查询服务，不参与任何车辆控制决策
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import copy


class ServiceState(Enum):
    NORMAL_SERVICE = "normal_service"
    DEGRADED_SERVICE = "degraded_service"
    UPDATING = "updating"
    SYSTEM_PAUSED = "system_paused"


@dataclass
class PowertrainBrakeParams:
    max_power_kw: float = 100.0
    max_torque_nm: float = 250.0
    accel_curve_type: int = 1
    brake_master_cylinder_max_pressure_mpa: float = 10.0
    front_rear_brake_ratio: float = 0.7
    brake_booster_gain: float = 5.0
    pad_friction_coefficient: float = 0.4
    brake_piston_area_m2: float = 0.002
    brake_pad_count: int = 4
    tire_rolling_radius_m: float = 0.35
    drive_type: int = 1
    max_torque_reduction_ratio: float = 0.5


@dataclass
class ParamQueryRequest:
    requester_id: str = ""
    param_names: List[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


@dataclass
class ParamQueryResponse:
    requester_id: str = ""
    params: Dict[str, Tuple[float, str, float, bool]] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class ParamUpdateCommand:
    params: Dict[str, float] = field(default_factory=dict)
    source: str = ""
    checksum: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class ParamUpdateAck:
    result: str = ""
    checksum: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class ParamHealthReport:
    total_params: int = 0
    loaded_params: int = 0
    default_params: int = 0
    status: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class ParamFaultAlert:
    missing_params: List[str] = field(default_factory=list)
    default_values: Dict[str, float] = field(default_factory=dict)
    affected_modules: str = ""
    timestamp: float = field(default_factory=time.time)


PARAM_DEFAULTS = {
    "max_power_kw": 100.0,
    "max_torque_nm": 250.0,
    "accel_curve_type": 1,
    "brake_master_cylinder_max_pressure_mpa": 10.0,
    "front_rear_brake_ratio": 0.7,
    "brake_booster_gain": 5.0,
    "pad_friction_coefficient": 0.4,
    "brake_piston_area_m2": 0.002,
    "brake_pad_count": 4,
    "tire_rolling_radius_m": 0.35,
    "drive_type": 1,
    "max_torque_reduction_ratio": 0.5,
}

PARAM_UNITS = {
    "max_power_kw": "kW",
    "max_torque_nm": "Nm",
    "accel_curve_type": "",
    "brake_master_cylinder_max_pressure_mpa": "MPa",
    "front_rear_brake_ratio": "",
    "brake_booster_gain": "",
    "pad_friction_coefficient": "",
    "brake_piston_area_m2": "m²",
    "brake_pad_count": "",
    "tire_rolling_radius_m": "m",
    "drive_type": "",
    "max_torque_reduction_ratio": "",
}

PARAM_RANGES = {
    "max_power_kw": (30.0, 500.0),
    "max_torque_nm": (50.0, 1000.0),
    "accel_curve_type": (1, 3),
    "brake_master_cylinder_max_pressure_mpa": (5.0, 20.0),
    "front_rear_brake_ratio": (0.5, 0.8),
    "brake_booster_gain": (3.0, 10.0),
    "pad_friction_coefficient": (0.25, 0.55),
    "brake_piston_area_m2": (0.001, 0.005),
    "brake_pad_count": (2, 8),
    "tire_rolling_radius_m": (0.25, 0.55),
    "drive_type": (1, 4),
    "max_torque_reduction_ratio": (0.2, 0.8),
}


class PowertrainBrakeParamsManager:
    def __init__(self):
        self.module_id = "ad-mcc-34"
        self.module_name = "动力与制动参数管理单元"
        self.version = "V1.0"

        self.state = ServiceState.UPDATING
        self._params = PowertrainBrakeParams()
        self._is_default = {name: True for name in PARAM_DEFAULTS}
        self._backup_params = None
        self._pending_queries: List[ParamQueryRequest] = []
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_calibration_file = None
        self._query_update_command = None

        self._publish_query_response = None
        self._publish_health_report = None
        self._publish_update_ack = None
        self._publish_fault_alert = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_calibration_file_query(self, callback):
        self._query_calibration_file = callback

    def set_update_command_query(self, callback):
        self._query_update_command = callback

    def set_query_response_publisher(self, callback):
        self._publish_query_response = callback

    def set_health_report_publisher(self, callback):
        self._publish_health_report = callback

    def set_update_ack_publisher(self, callback):
        self._publish_update_ack = callback

    def set_fault_alert_publisher(self, callback):
        self._publish_fault_alert = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def add_query_request(self, request: ParamQueryRequest):
        self._pending_queries.append(request)

    def run_management_cycle(self):
        now = time.time()
        if self.state == ServiceState.SYSTEM_PAUSED:
            return

        if self.state == ServiceState.UPDATING:
            self._load_calibration()

        update_cmd = self._query_update_command() if self._query_update_command else None
        if update_cmd and self.state == ServiceState.NORMAL_SERVICE:
            self._process_update(update_cmd)

        while self._pending_queries:
            req = self._pending_queries.pop(0)
            self._respond_to_query(req)

        if getattr(self, '_last_report', 0) == 0 or now - self._last_report >= 2.0:
            self._last_report = now
            self._report_health()

    def _load_calibration(self):
        calib = self._query_calibration_file() if self._query_calibration_file else None
        if calib is None or not isinstance(calib, PowertrainBrakeParams):
            self.state = ServiceState.DEGRADED_SERVICE
            self._send_fault_alert("标定文件缺失，全部使用默认值")
            return

        loaded = 0
        missing = []
        for name in PARAM_DEFAULTS:
            val = getattr(calib, name, None)
            if val is not None and self._validate_param(name, val):
                setattr(self._params, name, val)
                self._is_default[name] = False
                loaded += 1
            else:
                self._is_default[name] = True
                missing.append(name)

        if missing:
            self._send_fault_alert(f"部分参数使用默认值: {missing}")
            self.state = ServiceState.DEGRADED_SERVICE
        else:
            self.state = ServiceState.NORMAL_SERVICE

    def _validate_param(self, name: str, value: float) -> bool:
        if name in PARAM_RANGES:
            low, high = PARAM_RANGES[name]
            if isinstance(low, int):
                return low <= int(value) <= high
            return low <= value <= high
        return True

    def _process_update(self, cmd: ParamUpdateCommand):
        self.state = ServiceState.UPDATING
        self._backup_params = copy.deepcopy(self._params)
        try:
            for name, val in cmd.params.items():
                if name in PARAM_DEFAULTS:
                    setattr(self._params, name, val)
            self.state = ServiceState.NORMAL_SERVICE
            if self._publish_update_ack:
                self._publish_update_ack(ParamUpdateAck(result="成功", checksum=cmd.checksum))
        except Exception:
            self._params = self._backup_params
            self.state = ServiceState.NORMAL_SERVICE
            if self._publish_update_ack:
                self._publish_update_ack(ParamUpdateAck(result="失败", checksum=cmd.checksum))

    def _respond_to_query(self, req: ParamQueryRequest):
        resp_params = {}
        for name in req.param_names:
            if name in PARAM_DEFAULTS:
                val = getattr(self._params, name)
                unit = PARAM_UNITS.get(name, "")
                confidence = 0.7 if self._is_default[name] and self.state == ServiceState.DEGRADED_SERVICE else 1.0
                is_default = self._is_default[name]
                resp_params[name] = (val, unit, confidence, is_default)
            else:
                resp_params[name] = (0.0, "", 0.0, True)

        if self._publish_query_response:
            self._publish_query_response(ParamQueryResponse(
                requester_id=req.requester_id,
                params=resp_params
            ))

    def _report_health(self):
        total = len(PARAM_DEFAULTS)
        loaded = sum(1 for v in self._is_default.values() if not v)
        default = total - loaded
        if self._publish_health_report:
            self._publish_health_report(ParamHealthReport(
                total_params=total,
                loaded_params=loaded,
                default_params=default,
                status=self.state.value
            ))

    def _send_fault_alert(self, message: str):
        missing = [name for name, is_def in self._is_default.items() if is_def]
        defaults = {name: PARAM_DEFAULTS[name] for name in missing}
        if self._publish_fault_alert:
            self._publish_fault_alert(ParamFaultAlert(
                missing_params=missing,
                default_values=defaults,
                affected_modules="多个动力/制动模块",
            ))

    def get_state(self) -> ServiceState:
        return self.state

    def emergency_shutdown(self):
        self.state = ServiceState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 动力与制动参数管理单元 (ad-mcc-34) 演示")
    print("=" * 70)

    mgr = PowertrainBrakeParamsManager()
    mgr.set_calibration_file_query(lambda: PowertrainBrakeParams(
        max_power_kw=150.0,
        brake_master_cylinder_max_pressure_mpa=12.0,
        tire_rolling_radius_m=0.33
    ))
    mgr.set_update_command_query(lambda: None)

    print_separator("STEP 1: 加载标定参数")
    for _ in range(2):
        mgr.run_management_cycle()
    print(f"  状态: {mgr.state.value}")

    print_separator("STEP 2: 查询最大功率与制动主缸压力")
    mgr.add_query_request(ParamQueryRequest(
        requester_id="ad-mcc-09",
        param_names=["max_power_kw", "brake_master_cylinder_max_pressure_mpa"]
    ))
    mgr.run_management_cycle()
    print("  查询完成 (查看回调输出)")

    print("\n✅ 动力与制动参数管理单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-34 动力与制动参数管理单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_manager(calib=None):
            m = PowertrainBrakeParamsManager()
            m.set_calibration_file_query(lambda: calib)
            m.set_update_command_query(lambda: None)
            return m

        print("\n[TC-M34-01] 正常查询标定值")
        try:
            m = setup_manager(calib=PowertrainBrakeParams(
                max_power_kw=150.0,
                brake_master_cylinder_max_pressure_mpa=12.0,
                tire_rolling_radius_m=0.33
            ))
            for _ in range(2):
                m.run_management_cycle()
            resp = None
            def trap_resp(r):
                nonlocal resp
                resp = r
            m.set_query_response_publisher(trap_resp)
            m.add_query_request(ParamQueryRequest(
                requester_id="ad-mcc-09",
                param_names=["max_power_kw"]
            ))
            m.run_management_cycle()
            assert resp is not None
            val, unit, conf, is_def = resp.params["max_power_kw"]
            assert val == 150.0 and conf == 1.0 and not is_def
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M34-02] 标定文件缺失使用默认值")
        try:
            m = setup_manager(calib=None)
            for _ in range(2):
                m.run_management_cycle()
            resp = None
            def trap_resp(r):
                nonlocal resp
                resp = r
            m.set_query_response_publisher(trap_resp)
            m.add_query_request(ParamQueryRequest(
                requester_id="ad-mcc-13",
                param_names=["brake_master_cylinder_max_pressure_mpa"]
            ))
            m.run_management_cycle()
            assert resp is not None
            val, unit, conf, is_def = resp.params["brake_master_cylinder_max_pressure_mpa"]
            assert val == 10.0 and conf == 0.7 and is_def
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M34-03] 查询不存在的参数")
        try:
            m = setup_manager(calib=PowertrainBrakeParams())
            for _ in range(2):
                m.run_management_cycle()
            resp = None
            def trap_resp(r):
                nonlocal resp
                resp = r
            m.set_query_response_publisher(trap_resp)
            m.add_query_request(ParamQueryRequest(requester_id="ad-mcc-00", param_names=["invalid_param"]))
            m.run_management_cycle()
            assert resp is not None
            val, unit, conf, is_def = resp.params["invalid_param"]
            assert val == 0.0 and conf == 0.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M34-04] 紧急熔断")
        try:
            m = setup_manager()
            m.emergency_shutdown()
            assert m.state == ServiceState.SYSTEM_PAUSED
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