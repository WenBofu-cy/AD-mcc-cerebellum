#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-03
模块名称: 全身运动状态归集中心
所属分区: 一、顶层总控中枢
核心职责: 实时汇总各执行模块（转向、动力、制动、姿态、灯光、档位等）的协同进度、
          执行偏差、平衡状态与动作流畅度。整合为统一的“全身运动状态快照”，生成
          运动质量评估报告与体态异常标记，周期性上报 ECC 大脑及 MLNF-Mem 记忆中枢。
          为 ECC 提供运动执行层面的完整闭环反馈，支撑元认知、决策追溯与运动调优。

依赖模块:
    ad-mcc-04 至 ad-mcc-38（各执行模块，周期性上报执行状态与偏差）,
    ad-mcc-37（运动质量评估单元，提供运动质量得分与优化建议）
被依赖模块:
    ad-mcc-01（小脑总控调度核心，接收状态汇总用于调度决策）,
    ECC-12 资源调度模块（通过 CerebellumBus 接收全身运动状态快照）,
    MLNF-Mem 记忆中枢（通过 MemoryBus 接收运动状态快照用于经验沉淀）

安全约束:
  S-01: 本模块仅做状态归集与上报，不参与任何操控决策。异常标记仅作为建议，不直接触发执行动作
  S-02: 模块离线时，必须使用最后有效值填充缺失数据并明确标记在线=False，不可伪造在线状态
  S-03: 全身运动状态快照中的数据须与各执行模块上报的原始数据保持一致，不得篡改或修饰
  S-04: 数据完整性为“严重缺失”时，快照中须明确标注，供 ECC 决策时参考
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid


# ==================== 枚举定义 ====================

class GatheringState(Enum):
    """状态归集中心内部状态"""
    NORMAL_GATHERING = "normal_gathering"
    PARTIAL_OFFLINE = "partial_offline"
    SEVERE_DEGRADED = "severe_degraded"
    SYSTEM_PAUSED = "system_paused"


class DataIntegrity(Enum):
    """数据完整性"""
    COMPLETE = "完整"
    PARTIAL_MISSING = "部分缺失"
    SEVERE_MISSING = "严重缺失"


class AnomalySeverity(Enum):
    """异常严重等级"""
    NORMAL = "正常"
    WARNING = "一般"
    CRITICAL = "严重"


class ModuleOnlineStatus(Enum):
    """模块在线状态"""
    ONLINE = "在线"
    OFFLINE = "离线"


# ==================== 数据结构 ====================

@dataclass
class SteeringExecutionStatus:
    """转向执行状态"""
    target_angle_deg: float = 0.0
    actual_angle_deg: float = 0.0
    angle_deviation_deg: float = 0.0
    angle_rate_deg_per_s: float = 0.0
    execution_delay_ms: float = 0.0
    online: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class ThrottleExecutionStatus:
    """动力执行状态"""
    target_speed_kmh: float = 0.0
    actual_speed_kmh: float = 0.0
    speed_deviation_kmh: float = 0.0
    throttle_pct: float = 0.0
    longitudinal_jerk_ms3: float = 0.0
    execution_delay_ms: float = 0.0
    online: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class BrakeExecutionStatus:
    """制动执行状态"""
    target_deceleration_ms2: float = 0.0
    actual_deceleration_ms2: float = 0.0
    decel_deviation_pct: float = 0.0
    brake_pressure_mpa: float = 0.0
    brake_type: str = "日常缓刹"
    execution_delay_ms: float = 0.0
    online: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class PoseExecutionStatus:
    """姿态执行状态"""
    target_yaw_rate_rads: float = 0.0
    actual_yaw_rate_rads: float = 0.0
    roll_angle_deg: float = 0.0
    pitch_angle_deg: float = 0.0
    rollover_risk_level: str = "低"
    online: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class LightExecutionStatus:
    """灯光执行状态"""
    turn_signal: str = "off"
    hazard_lights: bool = False
    headlights: str = "auto"
    brake_lights: str = "normal"
    online: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class GearExecutionStatus:
    """档位执行状态"""
    current_gear: str = "D"
    target_gear: str = "D"
    switching_in_progress: bool = False
    online: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class MotionQualityReport:
    """运动质量评估报告（来自 ad-mcc-37）"""
    quality_score: float = 0.0
    longitudinal_jerk_rms: float = 0.0
    lateral_jerk_rms: float = 0.0
    deviation_accumulation: float = 0.0
    optimization_suggestions: List[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


@dataclass
class FullBodyMotionSnapshot:
    """全身运动状态快照"""
    snapshot_timestamp: float = field(default_factory=time.time)
    snapshot_sequence_num: int = 0
    steering: SteeringExecutionStatus = field(default_factory=SteeringExecutionStatus)
    throttle: ThrottleExecutionStatus = field(default_factory=ThrottleExecutionStatus)
    brake: BrakeExecutionStatus = field(default_factory=BrakeExecutionStatus)
    pose: PoseExecutionStatus = field(default_factory=PoseExecutionStatus)
    light: LightExecutionStatus = field(default_factory=LightExecutionStatus)
    gear: GearExecutionStatus = field(default_factory=GearExecutionStatus)
    data_integrity: DataIntegrity = DataIntegrity.COMPLETE
    module_online_ratio: float = 1.0


@dataclass
class AnomalyMarker:
    """体态异常标记"""
    anomaly_type: str = ""
    anomaly_module: str = ""
    severity: AnomalySeverity = AnomalySeverity.NORMAL
    suggested_action: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class ModuleOnlineList:
    """模块在线状态清单"""
    modules: Dict[str, ModuleOnlineStatus] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


# ==================== 主类定义 ====================

class FullBodyMotionGatherer:
    """
    全身运动状态归集中心
    
    职责:
    1. 实时汇总各执行模块的协同进度、执行偏差与平衡状态
    2. 整合为统一的全身运动状态快照
    3. 周期性上报 ECC 大脑及 MLNF-Mem 记忆中枢
    4. 检测体态异常并标记，转发运动质量评估报告
    """

    # 模块心跳超时判定周期数（连续3个上报周期无响应视为离线）
    HEARTBEAT_TIMEOUT_CYCLES = 3
    # 快照生成间隔（秒）
    SNAPSHOT_INTERVAL_S = 0.1  # 100ms
    # 在线状态上报间隔（秒）
    ONLINE_REPORT_INTERVAL_S = 1.0
    # 关键模块列表（离线会触发严重降级）
    CRITICAL_MODULES = ["转向", "制动"]

    # 异常阈值
    STEERING_DEVIATION_THRESHOLD_DEG = 5.0
    SPEED_DEVIATION_THRESHOLD_KMH = 3.0
    DECEL_DEVIATION_THRESHOLD_PCT = 20.0
    ROLLOVER_RISK_HIGH = "高"

    def __init__(self):
        self.module_id = "ad-mcc-03"
        self.module_name = "全身运动状态归集中心"
        self.version = "V1.0"

        self.state = GatheringState.NORMAL_GATHERING
        self._snapshot_seq_num: int = 0

        # 各模块最新状态缓存
        self._steering_status: Optional[SteeringExecutionStatus] = None
        self._throttle_status: Optional[ThrottleExecutionStatus] = None
        self._brake_status: Optional[BrakeExecutionStatus] = None
        self._pose_status: Optional[PoseExecutionStatus] = None
        self._light_status: Optional[LightExecutionStatus] = None
        self._gear_status: Optional[GearExecutionStatus] = None
        self._quality_report: Optional[MotionQualityReport] = None

        # 模块心跳跟踪（模块名 → 最后心跳时间戳）
        self._heartbeats: Dict[str, float] = {}
        # 模块在线状态（模块名 → 是否在线）
        self._online_status: Dict[str, bool] = {}
        # 最后有效值备份（模块名 → 状态对象）
        self._last_valid_status: Dict[str, Any] = {}

        # 时间管理
        self._last_snapshot_time: float = 0.0
        self._last_online_report_time: float = 0.0

        # 统计
        self._total_snapshots: int = 0
        self._total_anomalies_detected: int = 0

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调（接收各模块状态）
        self._query_steering_status = None          # Callable[[], Optional[SteeringExecutionStatus]]
        self._query_throttle_status = None          # Callable[[], Optional[ThrottleExecutionStatus]]
        self._query_brake_status = None             # Callable[[], Optional[BrakeExecutionStatus]]
        self._query_pose_status = None              # Callable[[], Optional[PoseExecutionStatus]]
        self._query_light_status = None             # Callable[[], Optional[LightExecutionStatus]]
        self._query_gear_status = None              # Callable[[], Optional[GearExecutionStatus]]
        self._query_quality_report = None           # Callable[[], Optional[MotionQualityReport]]

        # 外部发布回调
        self._publish_to_dispatcher = None          # Callable[[Any], None]  向 ad-mcc-01
        self._publish_to_cerebellum = None          # Callable[[str, Any], None]  向 ECC-12
        self._publish_to_memory = None              # Callable[[str, Any], None]  向 MLNF-Mem

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_steering_status_query(self, callback):
        self._query_steering_status = callback

    def set_throttle_status_query(self, callback):
        self._query_throttle_status = callback

    def set_brake_status_query(self, callback):
        self._query_brake_status = callback

    def set_pose_status_query(self, callback):
        self._query_pose_status = callback

    def set_light_status_query(self, callback):
        self._query_light_status = callback

    def set_gear_status_query(self, callback):
        self._query_gear_status = callback

    def set_quality_report_query(self, callback):
        self._query_quality_report = callback

    def set_dispatcher_publisher(self, callback):
        self._publish_to_dispatcher = callback

    def set_cerebellum_publisher(self, callback):
        self._publish_to_cerebellum = callback

    def set_memory_publisher(self, callback):
        self._publish_to_memory = callback

    # ========== 主循环 ==========
    def run_gathering_cycle(self) -> Optional[FullBodyMotionSnapshot]:
        """
        执行一次状态归集周期（高频调用，100Hz）
        
        Returns:
            本次生成的全身快照，若未到生成周期则返回 None
        """
        if self.state == GatheringState.SYSTEM_PAUSED:
            return None

        now = time.time()

        # 接收各模块状态更新
        self._update_module_status("转向", self._query_steering_status, '_steering_status')
        self._update_module_status("动力", self._query_throttle_status, '_throttle_status')
        self._update_module_status("制动", self._query_brake_status, '_brake_status')
        self._update_module_status("姿态", self._query_pose_status, '_pose_status')
        self._update_module_status("灯光", self._query_light_status, '_light_status')
        self._update_module_status("档位", self._query_gear_status, '_gear_status')

        # 接收运动质量评估报告并转发
        quality = self._query_quality_report() if self._query_quality_report else None
        if quality:
            self._quality_report = quality
            self._forward_quality_report(quality)

        # 模块心跳超时检查
        self._check_heartbeat_timeout(now)

        # 周期性快照生成（100ms）
        if now - self._last_snapshot_time >= self.SNAPSHOT_INTERVAL_S:
            self._last_snapshot_time = now
            self._snapshot_seq_num += 1
            self._total_snapshots += 1

            snapshot = self._build_snapshot()
            self._publish_snapshot(snapshot)
            self._detect_anomalies(snapshot)
            return snapshot

        # 周期性在线状态上报（1s）
        if now - self._last_online_report_time >= self.ONLINE_REPORT_INTERVAL_S:
            self._last_online_report_time = now
            self._report_online_status()

        return None

    # ========== 模块状态更新 ==========
    def _update_module_status(self, module_name: str, query_func, attr_name: str):
        """通用模块状态更新方法"""
        if query_func is None:
            return

        status = query_func()
        if status is not None:
            setattr(self, attr_name, status)
            self._heartbeats[module_name] = time.time()
            self._online_status[module_name] = True
            # 备份最后有效值
            self._last_valid_status[module_name] = status
        # 若为 None 则不更新状态，等待心跳超时检查处理

    # ========== 心跳超时检查 ==========
    def _check_heartbeat_timeout(self, now: float):
        """检查各模块心跳是否超时"""
        all_modules = ["转向", "动力", "制动", "姿态", "灯光", "档位"]
        critical_offline = False
        any_offline = False

        for mod in all_modules:
            last_hb = self._heartbeats.get(mod, 0.0)
            # 默认上报周期按100Hz算，3个周期 = 0.03s，实际根据模块调整，此处用0.1s作为示例
            if now - last_hb > 0.5:  # 实际应根据各模块上报周期动态计算
                if self._online_status.get(mod, True):
                    self._online_status[mod] = False
                    # 使用最后有效值填充
                    self._fill_last_valid(mod)
                any_offline = True
                if mod in self.CRITICAL_MODULES:
                    critical_offline = True
            else:
                self._online_status[mod] = True

        if critical_offline:
            self.state = GatheringState.SEVERE_DEGRADED
        elif any_offline:
            self.state = GatheringState.PARTIAL_OFFLINE
        else:
            self.state = GatheringState.NORMAL_GATHERING

    def _fill_last_valid(self, module_name: str):
        """用最后有效值填充离线模块的状态"""
        last_valid = self._last_valid_status.get(module_name)
        if last_valid is None:
            return
        # 复制最后有效值并标记在线=False
        if module_name == "转向":
            self._steering_status = SteeringExecutionStatus(
                **vars(last_valid), online=False, timestamp=time.time()
            )
        elif module_name == "动力":
            self._throttle_status = ThrottleExecutionStatus(
                **vars(last_valid), online=False, timestamp=time.time()
            )
        elif module_name == "制动":
            self._brake_status = BrakeExecutionStatus(
                **vars(last_valid), online=False, timestamp=time.time()
            )
        elif module_name == "姿态":
            self._pose_status = PoseExecutionStatus(
                **vars(last_valid), online=False, timestamp=time.time()
            )
        elif module_name == "灯光":
            self._light_status = LightExecutionStatus(
                **vars(last_valid), online=False, timestamp=time.time()
            )
        elif module_name == "档位":
            self._gear_status = GearExecutionStatus(
                **vars(last_valid), online=False, timestamp=time.time()
            )

    # ========== 快照构建 ==========
    def _build_snapshot(self) -> FullBodyMotionSnapshot:
        """构建全身运动状态快照"""
        # 使用默认值兜底，确保快照始终完整
        steering = self._steering_status or SteeringExecutionStatus(online=False)
        throttle = self._throttle_status or ThrottleExecutionStatus(online=False)
        brake = self._brake_status or BrakeExecutionStatus(online=False)
        pose = self._pose_status or PoseExecutionStatus(online=False)
        light = self._light_status or LightExecutionStatus(online=False)
        gear = self._gear_status or GearExecutionStatus(online=False)

        # 计算模块在线率
        modules = ["转向", "动力", "制动", "姿态", "灯光", "档位"]
        online_count = sum(1 for m in modules if self._online_status.get(m, False))
        online_ratio = online_count / len(modules)

        # 数据完整性判定
        if online_ratio >= 1.0:
            integrity = DataIntegrity.COMPLETE
        elif online_ratio >= 0.7:
            integrity = DataIntegrity.PARTIAL_MISSING
        else:
            integrity = DataIntegrity.SEVERE_MISSING

        snapshot = FullBodyMotionSnapshot(
            snapshot_timestamp=time.time(),
            snapshot_sequence_num=self._snapshot_seq_num,
            steering=steering,
            throttle=throttle,
            brake=brake,
            pose=pose,
            light=light,
            gear=gear,
            data_integrity=integrity,
            module_online_ratio=round(online_ratio, 3)
        )

        return snapshot

    # ========== 快照发布 ==========
    def _publish_snapshot(self, snapshot: FullBodyMotionSnapshot):
        """发布全身快照到内部调度总线、CerebellumBus 和 MemoryBus"""
        if self._publish_to_dispatcher:
            self._publish_to_dispatcher(snapshot)
        if self._publish_to_cerebellum:
            self._publish_to_cerebellum("full_body_motion_snapshot", snapshot)
        if self._publish_to_memory:
            self._publish_to_memory("full_body_motion_snapshot", snapshot)

    # ========== 质量报告转发 ==========
    def _forward_quality_report(self, report: MotionQualityReport):
        """转发运动质量评估报告"""
        if self._publish_to_cerebellum:
            self._publish_to_cerebellum("motion_quality_report", report)
        if self._publish_to_memory:
            self._publish_to_memory("motion_quality_report", report)

    # ========== 异常检测 ==========
    def _detect_anomalies(self, snapshot: FullBodyMotionSnapshot):
        """检测体态异常并标记"""
        anomalies = []

        # 转向偏差检测
        if snapshot.steering.online and abs(snapshot.steering.angle_deviation_deg) > self.STEERING_DEVIATION_THRESHOLD_DEG:
            anomalies.append(AnomalyMarker(
                anomaly_type="转向偏差超限",
                anomaly_module="转向",
                severity=AnomalySeverity.WARNING,
                suggested_action="检查转向执行器"
            ))

        # 速度偏差检测
        if snapshot.throttle.online and abs(snapshot.throttle.speed_deviation_kmh) > self.SPEED_DEVIATION_THRESHOLD_KMH:
            anomalies.append(AnomalyMarker(
                anomaly_type="车速偏差超限",
                anomaly_module="动力",
                severity=AnomalySeverity.WARNING,
                suggested_action="检查驱动系统"
            ))

        # 制动偏差检测
        if snapshot.brake.online and abs(snapshot.brake.decel_deviation_pct) > self.DECEL_DEVIATION_THRESHOLD_PCT:
            severity = AnomalySeverity.CRITICAL if abs(snapshot.brake.decel_deviation_pct) > 30.0 else AnomalySeverity.WARNING
            anomalies.append(AnomalyMarker(
                anomaly_type="制动减速度偏差超限",
                anomaly_module="制动",
                severity=severity,
                suggested_action="触发降级或紧急停车" if severity == AnomalySeverity.CRITICAL else "检查制动系统"
            ))

        # 侧翻风险检测
        if snapshot.pose.online and snapshot.pose.rollover_risk_level == self.ROLLOVER_RISK_HIGH:
            anomalies.append(AnomalyMarker(
                anomaly_type="侧翻风险高",
                anomaly_module="姿态",
                severity=AnomalySeverity.CRITICAL,
                suggested_action="立即减速并稳定车身"
            ))

        # 发布异常标记
        for anomaly in anomalies:
            self._total_anomalies_detected += 1
            if self._publish_to_dispatcher:
                self._publish_to_dispatcher(anomaly)
            if self._publish_to_cerebellum:
                self._publish_to_cerebellum("motion_anomaly_marker", anomaly)

    # ========== 在线状态上报 ==========
    def _report_online_status(self):
        """上报模块在线状态清单"""
        modules_status = {mod: (ModuleOnlineStatus.ONLINE if self._online_status.get(mod, False) else ModuleOnlineStatus.OFFLINE)
                          for mod in ["转向", "动力", "制动", "姿态", "灯光", "档位"]}
        report = ModuleOnlineList(modules=modules_status)
        if self._publish_to_dispatcher:
            self._publish_to_dispatcher(report)

    # ========== 查询接口 ==========
    def get_state(self) -> GatheringState:
        return self.state

    def get_latest_snapshot(self) -> Optional[FullBodyMotionSnapshot]:
        """获取最近一次快照（如有）"""
        # 为简化，直接返回最近构建的快照（需自行缓存）
        return None  # 实际项目可缓存

    # ========== 日志与统计 ==========
    def _log_event(self, event_type: str, details: Dict[str, Any]) -> None:
        self._pending_logs.append({
            "log_id": f"log-{uuid.uuid4().hex[:8]}",
            "event_type": event_type,
            "source_module": self.module_id,
            "details": details,
            "timestamp": time.time()
        })

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def get_statistics(self) -> Dict[str, Any]:
        return {
            "state": self.state.value,
            "total_snapshots": self._total_snapshots,
            "total_anomalies_detected": self._total_anomalies_detected,
            "online_modules": sum(1 for v in self._online_status.values() if v),
        }

    def emergency_shutdown(self):
        self.state = GatheringState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，保留最后有效快照")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 全身运动状态归集中心 (ad-mcc-03) 演示")
    print("=" * 70)

    gatherer = FullBodyMotionGatherer()

    # 模拟回调
    def mock_steering():
        return SteeringExecutionStatus(
            target_angle_deg=15.0,
            actual_angle_deg=14.5,
            angle_deviation_deg=0.5,
            angle_rate_deg_per_s=120.0,
            online=True
        )

    def mock_throttle():
        return ThrottleExecutionStatus(
            target_speed_kmh=80.0,
            actual_speed_kmh=79.0,
            speed_deviation_kmh=1.0,
            throttle_pct=30.0,
            online=True
        )

    def mock_brake():
        return BrakeExecutionStatus(
            target_deceleration_ms2=0.0,
            actual_deceleration_ms2=0.0,
            decel_deviation_pct=0.0,
            brake_pressure_mpa=0.0,
            online=True
        )

    def mock_pose():
        return PoseExecutionStatus(
            target_yaw_rate_rads=0.0,
            actual_yaw_rate_rads=0.0,
            roll_angle_deg=1.0,
            pitch_angle_deg=0.5,
            rollover_risk_level="低",
            online=True
        )

    def mock_light():
        return LightExecutionStatus(online=True)

    def mock_gear():
        return GearExecutionStatus(online=True)

    gatherer.set_steering_status_query(mock_steering)
    gatherer.set_throttle_status_query(mock_throttle)
    gatherer.set_brake_status_query(mock_brake)
    gatherer.set_pose_status_query(mock_pose)
    gatherer.set_light_status_query(mock_light)
    gatherer.set_gear_status_query(mock_gear)

    print_separator("STEP 1: 正常状态归集并生成快照")
    # 首次调用可能立即生成快照
    snapshot = gatherer.run_gathering_cycle()
    if snapshot:
        print(f"  快照序列号: {snapshot.snapshot_sequence_num}")
        print(f"  数据完整性: {snapshot.data_integrity.value}")
        print(f"  模块在线率: {snapshot.module_online_ratio}")

    print_separator("STEP 2: 模拟转向偏差超限")
    gatherer.set_steering_status_query(lambda: SteeringExecutionStatus(
        target_angle_deg=15.0,
        actual_angle_deg=8.0,
        angle_deviation_deg=7.0,
        online=True
    ))
    snapshot = gatherer.run_gathering_cycle()
    if snapshot:
        print(f"  转向偏差: {snapshot.steering.angle_deviation_deg}°")

    print("\n✅ 全身运动状态归集中心演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-03 全身运动状态归集中心 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_gatherer():
            g = FullBodyMotionGatherer()
            g.set_steering_status_query(lambda: SteeringExecutionStatus(
                target_angle_deg=10.0, actual_angle_deg=9.8,
                angle_deviation_deg=0.2, online=True
            ))
            g.set_throttle_status_query(lambda: ThrottleExecutionStatus(
                target_speed_kmh=60.0, actual_speed_kmh=59.5,
                speed_deviation_kmh=0.5, online=True
            ))
            g.set_brake_status_query(lambda: BrakeExecutionStatus(
                target_deceleration_ms2=0.0, actual_deceleration_ms2=0.0,
                decel_deviation_pct=0.0, online=True
            ))
            g.set_pose_status_query(lambda: PoseExecutionStatus(
                rollover_risk_level="低", online=True
            ))
            g.set_light_status_query(lambda: LightExecutionStatus(online=True))
            g.set_gear_status_query(lambda: GearExecutionStatus(online=True))
            return g

        # TC-M03-01: 正常生成全身快照
        print("\n[TC-M03-01] 全模块在线生成完整快照")
        try:
            g = setup_gatherer()
            snapshot = g.run_gathering_cycle()
            # 等待100ms生成快照，再调用一次
            import time
            time.sleep(0.11)
            snapshot = g.run_gathering_cycle()
            assert snapshot is not None
            assert snapshot.data_integrity == DataIntegrity.COMPLETE
            assert snapshot.module_online_ratio == 1.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M03-02: 转向偏差超限生成异常标记
        print("\n[TC-M03-02] 转向偏差超限生成异常标记")
        try:
            g = setup_gatherer()
            g.set_steering_status_query(lambda: SteeringExecutionStatus(
                target_angle_deg=15.0, actual_angle_deg=8.0,
                angle_deviation_deg=7.0, online=True
            ))
            time.sleep(0.11)
            snapshot = g.run_gathering_cycle()
            assert snapshot is not None
            assert g._total_anomalies_detected >= 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M03-03: 动力模块超时离线使用最后有效值
        print("\n[TC-M03-03] 动力模块超时离线使用最后有效值")
        try:
            g = setup_gatherer()
            # 先正常调用一次以设置心跳
            g.run_gathering_cycle()
            # 伪造离线：将心跳时间设到很久以前
            g._heartbeats["动力"] = 0.0
            # 设置查询返回 None（模拟无响应）
            g.set_throttle_status_query(lambda: None)
            time.sleep(0.11)
            snapshot = g.run_gathering_cycle()
            assert snapshot is not None
            assert snapshot.throttle.online == False
            assert snapshot.throttle.speed_deviation_kmh == 0.5  # 最后有效值
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M03-04: 紧急熔断
        print("\n[TC-M03-04] 紧急熔断")
        try:
            g = setup_gatherer()
            g.emergency_shutdown()
            assert g.state == GatheringState.SYSTEM_PAUSED
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