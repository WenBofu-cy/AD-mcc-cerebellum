#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-01
模块名称: 小脑总控调度核心
所属分区: 一、顶层总控中枢
核心职责: MCC 运动小脑最高统筹单元。接收 ECC 大脑下发的标准化行驶意图指令，解析指令
          类型与优先级，统一分发至转向、动力、制动、姿态、灯光、档位等各执行集群。
          汇总各子模块的执行状态反馈，形成运动闭环回执上报 ECC。调度不同执行模式
          （正常/降级/紧急/非铺装）之间的平滑切换。不参与任何场景判断与驾驶决策。

依赖模块:
    ECC-16(意图下发接口单元，通过 CerebellumBus 下发标准化行驶意图指令)
被依赖模块:
    ad-mcc-02(运动生理边界闸门，接收初步校验后的指令),
    ad-mcc-03(全身运动状态归集中心，接收状态汇总触发),
    ad-mcc-04 至 ad-mcc-38(全部执行模块，接收调度指令)

安全约束:
  S-01: MCC-01 为 ECC 大脑下发操控指令的唯一入口，任何模块不得绕过本模块直接操控车辆硬件
  S-02: 紧急制动或碰撞后响应指令为最高优先级，可立即中断当前任何正在执行的非安全指令
  S-03: 冷启动自检未通过时，MCC-01 必须处于维护锁定状态，拒绝执行任何操控指令
  S-04: 本模块不参与任何场景判断与驾驶决策，仅负责指令解析、分发与执行反馈汇总
  S-05: 执行模式切换时，各模块参数集必须通过 ad-mcc-02 运动生理边界闸门校验后方可生效
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
import time
import uuid


# ==================== 枚举定义 ====================

class DispatchState(Enum):
    """总控调度核心内部状态"""
    NORMAL_EXEC = "normal_exec"
    DEGRADED_EXEC = "degraded_exec"
    EMERGENCY_EXEC = "emergency_exec"
    UNPAVED_EXEC = "unpaved_exec"
    PARSING = "parsing"
    MAINTENANCE_LOCK = "maintenance_lock"
    SYSTEM_PAUSED = "system_paused"


class IntentType(Enum):
    """行驶意图类型（与 ECC 对齐）"""
    CRUISE = "CRUISE"
    LANE_CHANGE = "LANE_CHANGE"
    BRAKE = "BRAKE"
    TURN = "TURN"
    PARK = "PARK"
    CHARGING = "CHARGING"


class ExecutionMode(Enum):
    """执行模式"""
    NORMAL = "正常"
    DEGRADED_LEVEL1 = "一级降级"
    DEGRADED_LEVEL2 = "二级降级"
    DEGRADED_LEVEL3 = "三级降级"
    UNPAVED = "非铺装道路"


class MessagePriority(Enum):
    """消息优先级"""
    CRITICAL = "critical"
    HIGH = "high"
    NORMAL = "normal"


# ==================== 数据结构 ====================

@dataclass
class DrivingIntentCommand:
    """标准化行驶意图指令（来自 ECC-16）"""
    msg_id: str = ""
    priority: MessagePriority = MessagePriority.HIGH
    intent_type: IntentType = IntentType.CRUISE
    target_lane: int = 1
    target_speed_kmh: float = 0.0
    target_deceleration_ms2: float = 0.0
    target_trajectory: List[Tuple[float, float]] = field(default_factory=list)
    constraint_params: Dict[str, Any] = field(default_factory=dict)
    light_commands: Dict[str, Any] = field(default_factory=dict)
    gear_command: str = ""
    is_emergency: bool = False
    execution_timeout_s: float = 5.0
    checksum: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class EmergencyCommand:
    """紧急制动/碰撞后响应指令"""
    msg_id: str = ""
    intent_type: IntentType = IntentType.BRAKE
    target_speed_kmh: float = 0.0
    target_deceleration_ms2: float = 8.0
    is_emergency: bool = True
    hazard_lights: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass
class SteeringDispatchCommand:
    """转向调度指令"""
    target_angle_deg: float = 0.0
    max_angle_rate_deg_per_s: float = 300.0
    mode_mark: ExecutionMode = ExecutionMode.NORMAL


@dataclass
class ThrottleDispatchCommand:
    """动力调度指令"""
    target_speed_kmh: float = 0.0
    target_acceleration_ms2: float = 0.0
    max_jerk_ms3: float = 3.0
    mode_mark: ExecutionMode = ExecutionMode.NORMAL


@dataclass
class BrakeDispatchCommand:
    """制动调度指令"""
    target_deceleration_ms2: float = 0.0
    brake_type: str = "日常缓刹"
    brake_mode: str = "COMFORT"
    is_emergency: bool = False
    mode_mark: ExecutionMode = ExecutionMode.NORMAL


@dataclass
class PoseDispatchCommand:
    """姿态调度指令"""
    target_yaw_rate_rads: float = 0.0
    max_lateral_accel_ms2: float = 2.5
    mode_mark: ExecutionMode = ExecutionMode.NORMAL


@dataclass
class LightDispatchCommand:
    """灯光调度指令"""
    turn_signal: str = "off"
    hazard_lights: bool = False
    headlights: str = "auto"
    brake_lights: str = "normal"


@dataclass
class GearDispatchCommand:
    """档位调度指令"""
    target_gear: str = "D"
    switch_condition: str = ""


@dataclass
class ModeSwitchSignal:
    """执行模式切换信号"""
    new_mode: ExecutionMode = ExecutionMode.NORMAL
    switch_reason: str = ""
    parameter_set: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class MotionClosedLoopReceipt:
    """运动闭环回执"""
    command_id: str = ""
    execution_result: str = "完成"
    deviation_summary: Dict[str, float] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


# ==================== 主类定义 ====================

class CerebellumDispatchCore:
    """
    小脑总控调度核心
    
    职责:
    1. 接收 ECC 大脑下发的标准化行驶意图指令
    2. 解析指令类型与优先级，统一分发至各执行集群
    3. 汇总各子模块执行状态反馈，形成运动闭环回执
    4. 调度不同执行模式（正常/降级/紧急/非铺装）之间的平滑切换
    5. 不参与任何场景判断与驾驶决策
    """

    # 指令队列最大长度
    MAX_QUEUE_SIZE = 10
    # 压缩保留帧数
    COMPRESS_RETAIN_FRAMES = 2
    # 状态汇总间隔（秒）
    STATUS_REPORT_INTERVAL_S = 0.01  # 10ms

    def __init__(self):
        self.module_id = "ad-mcc-01"
        self.module_name = "小脑总控调度核心"
        self.version = "V1.0"

        # 内部状态
        self.state = DispatchState.MAINTENANCE_LOCK
        self._current_mode = ExecutionMode.NORMAL
        self._command_queue: deque = deque()
        self._last_status_report_time: float = 0.0

        # 统计
        self._total_commands_processed = 0
        self._total_emergency_commands = 0
        self._total_rejected_in_lock = 0

        # 待写入日志
        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调（通过 CerebellumBus 接收）
        self._query_driving_intent = None        # Callable[[], Optional[DrivingIntentCommand]]
        self._query_emergency_command = None     # Callable[[], Optional[EmergencyCommand]]
        self._query_mode_signal = None           # Callable[[], Optional[ExecutionMode]]
        self._query_self_check_result = None     # Callable[[], Optional[Dict[str, Any]]]
        self._query_module_statuses = None       # Callable[[], List[Dict[str, Any]]]

        # 内部调度回调（向各执行模块发送）
        self._send_to_steering = None            # Callable[[SteeringDispatchCommand], None]
        self._send_to_throttle = None            # Callable[[ThrottleDispatchCommand], None]
        self._send_to_brake = None               # Callable[[BrakeDispatchCommand], None]
        self._send_to_pose = None                # Callable[[PoseDispatchCommand], None]
        self._send_to_light = None               # Callable[[LightDispatchCommand], None]
        self._send_to_gear = None                # Callable[[GearDispatchCommand], None]
        self._send_mode_switch = None            # Callable[[ModeSwitchSignal], None]

        # 向 CerebellumBus 发送回执
        self._publish_to_bus = None              # Callable[[str, Any], None]

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成（维护锁定状态）")

    # ========== 回调注入 ==========
    def set_driving_intent_query(self, callback):
        self._query_driving_intent = callback

    def set_emergency_command_query(self, callback):
        self._query_emergency_command = callback

    def set_mode_signal_query(self, callback):
        self._query_mode_signal = callback

    def set_self_check_result_query(self, callback):
        self._query_self_check_result = callback

    def set_module_statuses_query(self, callback):
        self._query_module_statuses = callback

    def set_steering_sender(self, callback):
        self._send_to_steering = callback

    def set_throttle_sender(self, callback):
        self._send_to_throttle = callback

    def set_brake_sender(self, callback):
        self._send_to_brake = callback

    def set_pose_sender(self, callback):
        self._send_to_pose = callback

    def set_light_sender(self, callback):
        self._send_to_light = callback

    def set_gear_sender(self, callback):
        self._send_to_gear = callback

    def set_mode_switch_sender(self, callback):
        self._send_mode_switch = callback

    def set_bus_publisher(self, callback):
        self._publish_to_bus = callback

    # ========== 主调度循环 ==========
    def run_dispatch_cycle(self) -> Optional[MotionClosedLoopReceipt]:
        """
        执行一次调度周期（高频调用，500Hz）
        
        Returns:
            运动闭环回执，若未完成则返回 None
        """
        now = time.time()

        # 最高优先级：紧急指令
        emergency = self._query_emergency_command() if self._query_emergency_command else None
        if emergency:
            return self._handle_emergency(emergency)

        # 紧急熔断处理
        if self.state == DispatchState.SYSTEM_PAUSED:
            return None

        # 冷启动自检结果处理
        if self.state == DispatchState.MAINTENANCE_LOCK:
            self_check = self._query_self_check_result() if self._query_self_check_result else None
            if self_check and self_check.get("overall_passed", False):
                self.state = DispatchState.NORMAL_EXEC
                self._current_mode = ExecutionMode.NORMAL
                self._log_event("SELF_CHECK_PASSED", {})
            else:
                return None

        # 驾驶模式切换
        mode_signal = self._query_mode_signal() if self._query_mode_signal else None
        if mode_signal and mode_signal != self._current_mode:
            self._handle_mode_switch(mode_signal)

        # 接收 ECC 行驶意图指令
        intent = self._query_driving_intent() if self._query_driving_intent else None
        if intent:
            # 维护锁定状态拒绝执行
            if self.state == DispatchState.MAINTENANCE_LOCK:
                self._total_rejected_in_lock += 1
                self._log_event("REJECTED_IN_LOCK", {"msg_id": intent.msg_id})
                return None

            # 加入指令队列
            self._enqueue_command(intent)
            # 消费队列
            return self._consume_queue()

        # 周期性状态汇总
        if now - self._last_status_report_time >= self.STATUS_REPORT_INTERVAL_S:
            self._last_status_report_time = now
            self._report_status()

        return None

    # ========== 紧急指令处理 ==========
    def _handle_emergency(self, emergency: EmergencyCommand) -> MotionClosedLoopReceipt:
        """处理紧急制动/碰撞后响应指令（最高优先级）"""
        self.state = DispatchState.EMERGENCY_EXEC
        self._command_queue.clear()
        self._total_emergency_commands += 1

        self._log_event("EMERGENCY_COMMAND", {"msg_id": emergency.msg_id})

        # 立即下发制动指令
        if self._send_to_brake:
            self._send_to_brake(BrakeDispatchCommand(
                target_deceleration_ms2=emergency.target_deceleration_ms2,
                brake_type="紧急制动",
                is_emergency=True,
                mode_mark=ExecutionMode.DEGRADED_LEVEL3
            ))

        # 灯光指令
        if self._send_to_light:
            self._send_to_light(LightDispatchCommand(
                hazard_lights=True,
                brake_lights="flashing"
            ))

        receipt = MotionClosedLoopReceipt(
            command_id=emergency.msg_id,
            execution_result="紧急指令已执行"
        )

        if self._publish_to_bus:
            self._publish_to_bus("motion_closed_loop_receipt", receipt)

        return receipt

    # ========== 模式切换 ==========
    def _handle_mode_switch(self, new_mode: ExecutionMode):
        """处理驾驶模式切换"""
        self._current_mode = new_mode

        mode_state_map = {
            ExecutionMode.NORMAL: DispatchState.NORMAL_EXEC,
            ExecutionMode.DEGRADED_LEVEL1: DispatchState.DEGRADED_EXEC,
            ExecutionMode.DEGRADED_LEVEL2: DispatchState.DEGRADED_EXEC,
            ExecutionMode.DEGRADED_LEVEL3: DispatchState.EMERGENCY_EXEC,
            ExecutionMode.UNPAVED: DispatchState.UNPAVED_EXEC,
        }
        self.state = mode_state_map.get(new_mode, DispatchState.NORMAL_EXEC)

        signal = ModeSwitchSignal(
            new_mode=new_mode,
            switch_reason="驾驶模式变更",
            parameter_set={"mode": new_mode.value}
        )

        if self._send_mode_switch:
            self._send_mode_switch(signal)

        self._log_event("MODE_SWITCHED", {"new_mode": new_mode.value})

    # ========== 指令队列管理 ==========
    def _enqueue_command(self, intent: DrivingIntentCommand):
        """按优先级将指令加入队列"""
        if self.state in [DispatchState.DEGRADED_EXEC, DispatchState.EMERGENCY_EXEC]:
            if len(self._command_queue) >= self.MAX_QUEUE_SIZE:
                # 压缩队列：仅保留最新2帧
                while len(self._command_queue) > self.COMPRESS_RETAIN_FRAMES:
                    self._command_queue.popleft()
                self._log_event("QUEUE_COMPRESSED", {})

        # CRITICAL 优先级指令插队到头部
        if intent.priority == MessagePriority.CRITICAL:
            self._command_queue.appendleft(intent)
        else:
            self._command_queue.append(intent)

    def _consume_queue(self) -> Optional[MotionClosedLoopReceipt]:
        """消费指令队列"""
        if not self._command_queue:
            return None

        if self.state == DispatchState.SYSTEM_PAUSED:
            return None

        intent = self._command_queue.popleft()
        self.state = DispatchState.PARSING
        self._total_commands_processed += 1

        # 解析指令类型并分发
        self._dispatch_by_intent(intent)

        receipt = MotionClosedLoopReceipt(
            command_id=intent.msg_id,
            execution_result="完成"
        )

        if self._publish_to_bus:
            self._publish_to_bus("motion_closed_loop_receipt", receipt)

        # 恢复到当前模式状态
        self._restore_current_mode_state()

        return receipt

    def _dispatch_by_intent(self, intent: DrivingIntentCommand):
        """根据意图类型分发指令到各执行集群"""
        if intent.intent_type == IntentType.CRUISE:
            # 匀速巡航
            self._send_to_throttle(ThrottleDispatchCommand(
                target_speed_kmh=intent.target_speed_kmh,
                max_jerk_ms3=intent.constraint_params.get("max_jerk_ms3", 3.0),
                mode_mark=self._current_mode
            ))

        elif intent.intent_type == IntentType.LANE_CHANGE:
            # 车道变换
            self._send_to_light(LightDispatchCommand(
                turn_signal=intent.constraint_params.get("turn_signal", "left")
            ))
            # 转向指令（具体转角由 ad-mcc-04 解算）
            self._send_to_steering(SteeringDispatchCommand(
                target_angle_deg=intent.constraint_params.get("target_angle_deg", 0.0),
                mode_mark=self._current_mode
            ))
            self._send_to_throttle(ThrottleDispatchCommand(
                target_speed_kmh=intent.target_speed_kmh,
                mode_mark=self._current_mode
            ))

        elif intent.intent_type == IntentType.BRAKE:
            # 制动
            self._send_to_brake(BrakeDispatchCommand(
                target_deceleration_ms2=intent.target_deceleration_ms2,
                brake_type="紧急制动" if intent.is_emergency else "日常缓刹",
                brake_mode=intent.constraint_params.get("brake_mode", "COMFORT"),
                is_emergency=intent.is_emergency,
                mode_mark=self._current_mode
            ))
            if intent.is_emergency:
                self._send_to_light(LightDispatchCommand(
                    hazard_lights=True,
                    brake_lights="flashing"
                ))

        elif intent.intent_type == IntentType.TURN:
            # 转弯
            self._send_to_light(LightDispatchCommand(
                turn_signal=intent.constraint_params.get("turn_signal", "left")
            ))
            self._send_to_steering(SteeringDispatchCommand(
                target_angle_deg=intent.constraint_params.get("target_angle_deg", 0.0),
                max_angle_rate_deg_per_s=intent.constraint_params.get("max_angle_rate", 200.0),
                mode_mark=self._current_mode
            ))
            self._send_to_throttle(ThrottleDispatchCommand(
                target_speed_kmh=intent.target_speed_kmh,
                mode_mark=self._current_mode
            ))

        elif intent.intent_type == IntentType.PARK:
            # 停车
            self._send_to_brake(BrakeDispatchCommand(
                target_deceleration_ms2=2.0,
                brake_type="日常缓刹",
                brake_mode="COMFORT",
                mode_mark=self._current_mode
            ))
            if self._send_to_gear:
                self._send_to_gear(GearDispatchCommand(
                    target_gear="P",
                    switch_condition="完全静止后"
                ))

        elif intent.intent_type == IntentType.CHARGING:
            # 补能导航
            self._send_to_throttle(ThrottleDispatchCommand(
                target_speed_kmh=intent.target_speed_kmh,
                mode_mark=self._current_mode
            ))

    def _restore_current_mode_state(self):
        """恢复到当前模式状态"""
        mode_state_map = {
            ExecutionMode.NORMAL: DispatchState.NORMAL_EXEC,
            ExecutionMode.DEGRADED_LEVEL1: DispatchState.DEGRADED_EXEC,
            ExecutionMode.DEGRADED_LEVEL2: DispatchState.DEGRADED_EXEC,
            ExecutionMode.DEGRADED_LEVEL3: DispatchState.EMERGENCY_EXEC,
            ExecutionMode.UNPAVED: DispatchState.UNPAVED_EXEC,
        }
        self.state = mode_state_map.get(self._current_mode, DispatchState.NORMAL_EXEC)

    # ========== 状态汇总 ==========
    def _report_status(self):
        """周期性状态汇总上报"""
        status = {
            "current_mode": self._current_mode.value,
            "state": self.state.value,
            "queue_size": len(self._command_queue),
            "total_processed": self._total_commands_processed,
            "total_emergency": self._total_emergency_commands,
        }
        if self._publish_to_bus:
            self._publish_to_bus("mcc_status_report", status)

    # ========== 查询接口 ==========
    def get_state(self) -> DispatchState:
        return self.state

    def get_current_mode(self) -> ExecutionMode:
        return self._current_mode

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
            "current_mode": self._current_mode.value,
            "total_commands_processed": self._total_commands_processed,
            "total_emergency_commands": self._total_emergency_commands,
            "total_rejected_in_lock": self._total_rejected_in_lock,
            "queue_size": len(self._command_queue),
        }

    def emergency_shutdown(self):
        self.state = DispatchState.SYSTEM_PAUSED
        self._command_queue.clear()
        print(f"[{self.module_id}] 紧急熔断，清空指令队列")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 小脑总控调度核心 (ad-mcc-01) 演示")
    print("=" * 70)

    mcc = CerebellumDispatchCore()

    # 模拟自检通过
    mcc.set_self_check_result_query(lambda: {"overall_passed": True})

    print_separator("STEP 1: 自检通过，解锁")
    mcc.run_dispatch_cycle()
    print(f"  状态: {mcc.state.value}")

    print_separator("STEP 2: 接收巡航指令")
    mcc.set_driving_intent_query(lambda: DrivingIntentCommand(
        msg_id="MSG-001",
        intent_type=IntentType.CRUISE,
        target_speed_kmh=100.0,
        target_lane=2,
        constraint_params={"max_jerk_ms3": 3.0}
    ))
    receipt = mcc.run_dispatch_cycle()
    if receipt:
        print(f"  指令ID: {receipt.command_id}, 结果: {receipt.execution_result}")

    print_separator("STEP 3: 紧急制动抢占")
    mcc.set_emergency_command_query(lambda: EmergencyCommand(
        msg_id="EMG-001",
        target_deceleration_ms2=8.0
    ))
    mcc.run_dispatch_cycle()
    print(f"  状态: {mcc.state.value}")
    print(f"  统计: {mcc.get_statistics()}")

    print("\n✅ 小脑总控调度核心演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-01 小脑总控调度核心 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        # TC-M01-01: 自检通过后解锁
        print("\n[TC-M01-01] 自检通过后从维护锁定转为正常执行")
        try:
            mcc = CerebellumDispatchCore()
            mcc.set_self_check_result_query(lambda: {"overall_passed": True})
            mcc.run_dispatch_cycle()
            assert mcc.state == DispatchState.NORMAL_EXEC
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M01-02: 维护锁定状态拒绝执行
        print("\n[TC-M01-02] 维护锁定状态拒绝执行操控指令")
        try:
            mcc = CerebellumDispatchCore()
            mcc.set_driving_intent_query(lambda: DrivingIntentCommand(
                msg_id="T01", intent_type=IntentType.CRUISE, target_speed_kmh=80.0
            ))
            mcc.run_dispatch_cycle()
            assert mcc._total_rejected_in_lock == 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M01-03: 紧急指令抢占
        print("\n[TC-M01-03] 紧急指令清空队列并立即执行")
        try:
            mcc = CerebellumDispatchCore()
            mcc.set_self_check_result_query(lambda: {"overall_passed": True})
            mcc.run_dispatch_cycle()
            mcc.set_emergency_command_query(lambda: EmergencyCommand(
                msg_id="EMG", target_deceleration_ms2=8.0
            ))
            mcc.run_dispatch_cycle()
            assert mcc.state == DispatchState.EMERGENCY_EXEC
            assert mcc._total_emergency_commands == 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M01-04: 驾驶模式切换
        print("\n[TC-M01-04] 驾驶模式切换至降级")
        try:
            mcc = CerebellumDispatchCore()
            mcc.set_self_check_result_query(lambda: {"overall_passed": True})
            mcc.run_dispatch_cycle()
            mcc.set_mode_signal_query(lambda: ExecutionMode.DEGRADED_LEVEL2)
            mcc.run_dispatch_cycle()
            assert mcc._current_mode == ExecutionMode.DEGRADED_LEVEL2
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M01-05: 紧急熔断
        print("\n[TC-M01-05] 紧急熔断")
        try:
            mcc = CerebellumDispatchCore()
            mcc.emergency_shutdown()
            assert mcc.state == DispatchState.SYSTEM_PAUSED
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