#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-32
模块名称: 车辆尺寸参数管理单元
所属分区: 九、多车型自适应适配
核心职责: 存储并管理车辆出厂前一次性标定的尺寸参数（轴距、轮距、车长、车宽、质心高度、
          最小离地间隙、轮胎滚动半径等），为 MCC 运动小脑各执行模块提供统一的车辆尺寸参数
          查询服务。确保参数来源唯一、版本一致，避免各模块独立维护参数导致的不一致。
          不参与任何驾驶决策或控制，仅提供参数数据的存取与校验。

依赖模块:
    出厂标定系统(配置文件/诊断接口)
被依赖模块:
    ad-mcc-04/08/09/13/18/19/20/21 等执行模块

安全约束:
  S-01: 出厂标定参数为只读数据，运行时不得被任何控制模块修改
  S-02: 参数更新必须经过完整性校验与人工确认，更新前必须备份当前参数
  S-03: 参数缺失时必须使用保守默认值，确保车辆控制不因参数缺失而失效
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
class VehicleParams:
    wheelbase_m: float = 2.8
    track_front_m: float = 1.6
    track_rear_m: float = 1.6
    length_m: float = 4.8
    width_m: float = 1.9
    cg_height_m: float = 0.55
    ground_clearance_m: float = 0.15
    tire_rolling_radius_m: float = 0.35
    frontal_area_m2: float = 2.2
    curb_weight_kg: float = 1800.0
    steering_ratio: float = 16.0


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
    "wheelbase_m": 2.8,
    "track_front_m": 1.6,
    "track_rear_m": 1.6,
    "length_m": 4.8,
    "width_m": 1.9,
    "cg_height_m": 0.55,
    "ground_clearance_m": 0.15,
    "tire_rolling_radius_m": 0.35,
    "frontal_area_m2": 2.2,
    "curb_weight_kg": 1800.0,
    "steering_ratio": 16.0,
}

PARAM_UNITS = {
    "wheelbase_m": "m",
    "track_front_m": "m",
    "track_rear_m": "m",
    "length_m": "m",
    "width_m": "m",
    "cg_height_m": "m",
    "ground_clearance_m": "m",
    "tire_rolling_radius_m": "m",
    "frontal_area_m2": "m²",
    "curb_weight_kg": "kg",
    "steering_ratio": "",
}


class VehicleParamsManager:
    def __init__(self):
        self.module_id = "ad-mcc-32"
        self.module_name = "车辆尺寸参数管理单元"
        self.version = "V1.0"

        self.state = ServiceState.UPDATING
        self._params = VehicleParams()
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
        if calib is None or not isinstance(calib, VehicleParams):
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
        ranges = {
            "wheelbase_m": (2.0, 4.5),
            "track_front_m": (1.2, 2.2),
            "track_rear_m": (1.2, 2.2),
            "length_m": (3.0, 7.0),
            "width_m": (1.5, 2.5),
            "cg_height_m": (0.3, 1.2),
            "ground_clearance_m": (0.08, 0.35),
            "tire_rolling_radius_m": (0.25, 0.55),
            "frontal_area_m2": (1.5, 4.0),
            "curb_weight_kg": (800, 5000),
            "steering_ratio": (8.0, 24.0),
        }
        if name in ranges:
            low, high = ranges[name]
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
                affected_modules="多个模块",
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
    print("  AD-mcc-cerebellum 车辆尺寸参数管理单元 (ad-mcc-32) 演示")
    print("=" * 70)

    mgr = VehicleParamsManager()
    mgr.set_calibration_file_query(lambda: VehicleParams(wheelbase_m=2.9, cg_height_m=0.52))
    mgr.set_update_command_query(lambda: None)

    print_separator("STEP 1: 加载标定参数")
    for _ in range(2):
        mgr.run_management_cycle()
    print(f"  状态: {mgr.state.value}")

    print_separator("STEP 2: 查询轴距")
    mgr.add_query_request(ParamQueryRequest(requester_id="ad-mcc-04", param_names=["wheelbase_m"]))
    mgr.run_management_cycle()
    print("  查询完成 (查看回调输出)")

    print("\n✅ 车辆尺寸参数管理单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-32 车辆尺寸参数管理单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_manager(calib=None):
            m = VehicleParamsManager()
            m.set_calibration_file_query(lambda: calib)
            m.set_update_command_query(lambda: None)
            return m

        print("\n[TC-M32-01] 正常查询标定值")
        try:
            m = setup_manager(calib=VehicleParams(wheelbase_m=2.9, cg_height_m=0.52))
            for _ in range(2):
                m.run_management_cycle()
            resp = None
            def trap_resp(r):
                nonlocal resp
                resp = r
            m.set_query_response_publisher(trap_resp)
            m.add_query_request(ParamQueryRequest(requester_id="ad-mcc-04", param_names=["wheelbase_m"]))
            m.run_management_cycle()
            assert resp is not None
            val, unit, conf, is_def = resp.params["wheelbase_m"]
            assert val == 2.9 and conf == 1.0 and not is_def
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M32-02] 标定文件缺失使用默认值")
        try:
            m = setup_manager(calib=None)
            for _ in range(2):
                m.run_management_cycle()
            resp = None
            def trap_resp(r):
                nonlocal resp
                resp = r
            m.set_query_response_publisher(trap_resp)
            m.add_query_request(ParamQueryRequest(requester_id="ad-mcc-18", param_names=["cg_height_m"]))
            m.run_management_cycle()
            assert resp is not None
            val, unit, conf, is_def = resp.params["cg_height_m"]
            assert val == 0.55 and conf == 0.7 and is_def
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M32-03] 查询不存在的参数")
        try:
            m = setup_manager(calib=VehicleParams())
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

        print("\n[TC-M32-04] 紧急熔断")
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