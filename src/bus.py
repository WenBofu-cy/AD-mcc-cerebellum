#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AD-mcc-cerebellum 内部总线 CerebellumBus

为运动小脑 38 个模块提供松耦合的标准化通信层。
在完整系统中，CerebellumBus 由中间件实现（如 ZeroMQ / DDS / 共享内存）。
此处为最小可运行实现，用于模块联调、单元测试和最小闭环验证。

特性:
- 点对点消息投递（模块间精确通信）
- 广播消息（一对多）
- 消息优先级队列（紧急/高/普通/低 四级）
- 消息日志记录与统计
- 模块注册与回调机制
- 请求-响应关联追踪

与 AD-mlnf-mem 的 MemoryBus 和 AD-ecc-brain 的 MemoryBus 风格完全统一。
"""

from typing import Dict, List, Any, Callable, Optional, Tuple
from dataclasses import dataclass, field
from collections import defaultdict, deque
from enum import Enum
import time
import uuid


class MessagePriority(Enum):
    CRITICAL = 3
    HIGH = 2
    NORMAL = 1
    LOW = 0


class MessageType(Enum):
    # 顶层调度
    MODE_SWITCH = "mode_switch"
    GLOBAL_DISPATCH = "global_dispatch"
    DRIVING_INTENT = "driving_intent"
    EMERGENCY_SHUTDOWN = "emergency_shutdown"
    
    # 转向控制集群
    STEERING_TARGET = "steering_target"
    STEERING_SMOOTHED = "steering_smoothed"
    STEERING_CONSTRAINED = "steering_constrained"
    STEERING_DEVIATION = "steering_deviation"
    STEERING_UNPAVED_ADAPT = "steering_unpaved_adapt"
    
    # 动力控制集群
    THROTTLE_TARGET = "throttle_target"
    THROTTLE_CONSTRAINED = "throttle_constrained"
    THROTTLE_SMOOTHED = "throttle_smoothed"
    THROTTLE_DEVIATION = "throttle_deviation"
    
    # 制动控制集群
    BRAKE_TARGET = "brake_target"
    BRAKE_RESPONSE = "brake_response"
    BRAKE_SMOOTHED = "brake_smoothed"
    BRAKE_DEVIATION = "brake_deviation"
    REGEN_RATIO = "regen_ratio"
    
    # 车身姿态稳定
    ATTITUDE_VECTOR = "attitude_vector"
    ROLL_RISK = "roll_risk"
    YAW_CONTROL = "yaw_control"
    ROLL_PROTECTION = "roll_protection"
    BUMP_COMPENSATION = "bump_compensation"
    
    # 灯光与外设
    TURN_SIGNAL = "turn_signal"
    HAZARD_BRAKE_LIGHT = "hazard_brake_light"
    HIGH_BEAM = "high_beam"
    WIPER_CONTROL = "wiper_control"
    
    # 档位与驻车
    GEAR_SHIFT = "gear_shift"
    EPB_CONTROL = "epb_control"
    
    # 硬件异常应急防护
    STEERING_FAULT = "steering_fault"
    BRAKE_FAULT = "brake_fault"
    POWERTRAIN_FAULT = "powertrain_fault"
    BUS_FAULT = "bus_fault"
    DEGRADATION_REQUEST = "degradation_request"
    
    # 多车型参数
    PARAM_QUERY = "param_query"
    PARAM_QUERY_RESPONSE = "param_query_response"
    PARAM_UPDATE = "param_update"
    PARAM_FAULT = "param_fault"
    
    # 执行反馈与日志
    CLOSED_LOOP_ACK = "closed_loop_ack"
    DEVIATION_PACKAGE = "deviation_package"
    QUALITY_REPORT = "quality_report"
    LOG_EVENT = "log_event"
    LOG_QUERY = "log_query"
    
    # 通用
    STATUS_REPORT = "status_report"
    ACK = "ack"
    ERROR = "error"


@dataclass
class BusMessage:
    msg_id: str
    source_module: str
    target_module: str
    msg_type: MessageType
    payload: Dict[str, Any] = field(default_factory=dict)
    priority: MessagePriority = MessagePriority.NORMAL
    timestamp: float = field(default_factory=time.time)
    correlation_id: Optional[str] = None


@dataclass
class BusStats:
    total_sent: int = 0
    total_delivered: int = 0
    total_errors: int = 0
    by_priority: Dict[MessagePriority, int] = field(default_factory=lambda: defaultdict(int))
    by_type: Dict[MessageType, int] = field(default_factory=lambda: defaultdict(int))
    pending_messages: int = 0
    registered_modules: int = 0
    uptime_seconds: float = 0.0


class CerebellumBus:
    MAX_LOG_SIZE = 2000
    MAX_PENDING_PER_MODULE = 500
    
    def __init__(self):
        self._inboxes: Dict[str, Dict[MessagePriority, deque]] = defaultdict(
            lambda: {p: deque() for p in MessagePriority}
        )
        self._subscribers: Dict[str, Callable] = {}
        self._message_log: List[BusMessage] = []
        self._stats = BusStats()
        self._start_time = time.time()
        self._pending_requests: Dict[str, BusMessage] = {}
        
        print("[CerebellumBus] AD-mcc-cerebellum 内部总线初始化完成")
    
    def register_module(self, module_id: str, callback: Optional[Callable] = None) -> None:
        self._subscribers[module_id] = callback
        self._stats.registered_modules = len(self._subscribers)
        print(f"[CerebellumBus] 注册模块: {module_id} (总计 {self._stats.registered_modules} 个)")
    
    def unregister_module(self, module_id: str) -> None:
        if module_id in self._subscribers:
            del self._subscribers[module_id]
            if module_id in self._inboxes:
                del self._inboxes[module_id]
            self._stats.registered_modules = len(self._subscribers)
            print(f"[CerebellumBus] 注销模块: {module_id}")
    
    def send(self, source: str, target: str, msg_type: MessageType,
             payload: Dict[str, Any] = None,
             priority: MessagePriority = MessagePriority.NORMAL,
             correlation_id: Optional[str] = None) -> str:
        msg = BusMessage(
            msg_id=self._generate_msg_id(),
            source_module=source,
            target_module=target,
            msg_type=msg_type,
            payload=payload or {},
            priority=priority,
            correlation_id=correlation_id
        )
        self._inboxes[target][priority].append(msg)
        self._stats.total_sent += 1
        self._stats.by_priority[priority] += 1
        self._stats.by_type[msg_type] += 1
        if len(self._message_log) >= self.MAX_LOG_SIZE:
            self._message_log = self._message_log[-self.MAX_LOG_SIZE // 2:]
        self._message_log.append(msg)
        return msg.msg_id
    
    def broadcast(self, source: str, msg_type: MessageType,
                  payload: Dict[str, Any] = None,
                  priority: MessagePriority = MessagePriority.NORMAL,
                  exclude_self: bool = True) -> List[str]:
        msg_ids = []
        for target in list(self._subscribers.keys()):
            if exclude_self and target == source:
                continue
            msg_ids.append(self.send(source, target, msg_type, payload, priority))
        return msg_ids
    
    def request(self, source: str, target: str, msg_type: MessageType,
                payload: Dict[str, Any] = None,
                priority: MessagePriority = MessagePriority.NORMAL) -> Tuple[str, str]:
        correlation_id = self._generate_msg_id()
        msg_id = self.send(source, target, msg_type, payload, priority, correlation_id)
        self._pending_requests[correlation_id] = None
        return msg_id, correlation_id
    
    def respond(self, original_msg: BusMessage, response_type: MessageType,
                payload: Dict[str, Any] = None,
                priority: MessagePriority = MessagePriority.NORMAL) -> str:
        return self.send(
            source=original_msg.target_module,
            target=original_msg.source_module,
            msg_type=response_type,
            payload=payload,
            priority=priority,
            correlation_id=original_msg.correlation_id
        )
    
    def poll(self, module_id: str, max_messages: int = 10,
             priority_filter: Optional[MessagePriority] = None) -> List[BusMessage]:
        if module_id not in self._inboxes:
            return []
        messages = []
        inbox = self._inboxes[module_id]
        priorities = [MessagePriority.CRITICAL, MessagePriority.HIGH,
                      MessagePriority.NORMAL, MessagePriority.LOW]
        if priority_filter:
            priorities = [priority_filter]
        for priority in priorities:
            queue = inbox[priority]
            while queue and len(messages) < max_messages:
                messages.append(queue.popleft())
                self._stats.total_delivered += 1
        return messages
    
    def poll_all(self, module_id: str) -> List[BusMessage]:
        return self.poll(module_id, max_messages=self.MAX_PENDING_PER_MODULE)
    
    def get_stats(self) -> BusStats:
        self._stats.pending_messages = sum(
            sum(len(q) for q in inbox.values())
            for inbox in self._inboxes.values()
        )
        self._stats.uptime_seconds = time.time() - self._start_time
        return self._stats
    
    def get_message_log(self, limit: int = 100,
                        msg_type: Optional[MessageType] = None,
                        source: Optional[str] = None,
                        target: Optional[str] = None) -> List[BusMessage]:
        result = self._message_log
        if msg_type: result = [m for m in result if m.msg_type == msg_type]
        if source: result = [m for m in result if m.source_module == source]
        if target: result = [m for m in result if m.target_module == target]
        return result[-limit:]
    
    def reset_stats(self) -> None:
        self._stats = BusStats(registered_modules=len(self._subscribers))
        self._message_log = []
    
    def _generate_msg_id(self) -> str:
        return f"msg-{uuid.uuid4().hex[:12]}"


if __name__ == "__main__":
    print("=" * 60)
    print("CerebellumBus 内部总线 单元测试")
    print("=" * 60)
    
    passed, failed = 0, 0
    
    print("\n[TC-BUS-01] 发送点对点消息")
    try:
        bus = CerebellumBus()
        bus.register_module("ad-mcc-01")
        bus.register_module("ad-mcc-04")
        msg_id = bus.send("ad-mcc-01", "ad-mcc-04", MessageType.STEERING_TARGET,
                          payload={"angle": 15.0})
        messages = bus.poll("ad-mcc-04")
        assert len(messages) == 1
        assert messages[0].payload["angle"] == 15.0
        print("   ✅ PASS")
        passed += 1
    except Exception as e:
        print(f"   ❌ FAIL: {e}")
        failed += 1
    
    print("\n[TC-BUS-02] 广播消息")
    try:
        bus = CerebellumBus()
        bus.register_module("ad-mcc-01")
        bus.register_module("ad-mcc-04")
        bus.register_module("ad-mcc-09")
        msg_ids = bus.broadcast("ad-mcc-01", MessageType.MODE_SWITCH,
                                payload={"mode": "normal"})
        assert len(msg_ids) == 2
        print("   ✅ PASS")
        passed += 1
    except Exception as e:
        print(f"   ❌ FAIL: {e}")
        failed += 1
    
    print("\n[TC-BUS-03] 消息优先级排序")
    try:
        bus = CerebellumBus()
        bus.register_module("ad-mcc-01")
        bus.register_module("ad-mcc-02")
        bus.send("ad-mcc-01", "ad-mcc-02", MessageType.STATUS_REPORT, priority=MessagePriority.LOW)
        bus.send("ad-mcc-01", "ad-mcc-02", MessageType.EMERGENCY_SHUTDOWN, priority=MessagePriority.CRITICAL)
        bus.send("ad-mcc-01", "ad-mcc-02", MessageType.STEERING_TARGET, priority=MessagePriority.HIGH)
        messages = bus.poll_all("ad-mcc-02")
        assert messages[0].priority == MessagePriority.CRITICAL
        assert messages[1].priority == MessagePriority.HIGH
        assert messages[2].priority == MessagePriority.LOW
        print("   ✅ PASS")
        passed += 1
    except Exception as e:
        print(f"   ❌ FAIL: {e}")
        failed += 1
    
    print("\n[TC-BUS-04] 请求-响应模式")
    try:
        bus = CerebellumBus()
        bus.register_module("ad-mcc-01")
        bus.register_module("ad-mcc-32")
        msg_id, corr_id = bus.request("ad-mcc-01", "ad-mcc-32", MessageType.PARAM_QUERY,
                                       payload={"params": ["wheelbase_m"]})
        requests = bus.poll("ad-mcc-32")
        assert len(requests) == 1
        assert requests[0].correlation_id == corr_id
        resp_id = bus.respond(requests[0], MessageType.PARAM_QUERY_RESPONSE,
                              payload={"wheelbase_m": 2.9})
        responses = bus.poll("ad-mcc-01")
        assert len(responses) == 1
        assert responses[0].correlation_id == corr_id
        print("   ✅ PASS")
        passed += 1
    except Exception as e:
        print(f"   ❌ FAIL: {e}")
        failed += 1
    
    print("\n[TC-BUS-05] 总线统计")
    try:
        bus = CerebellumBus()
        bus.register_module("ad-mcc-01")
        bus.register_module("ad-mcc-13")
        bus.send("ad-mcc-01", "ad-mcc-13", MessageType.BRAKE_TARGET, priority=MessagePriority.CRITICAL)
        stats = bus.get_stats()
        assert stats.total_sent == 1
        assert stats.registered_modules == 2
        print("   ✅ PASS")
        passed += 1
    except Exception as e:
        print(f"   ❌ FAIL: {e}")
        failed += 1
    
    print("\n" + "=" * 60)
    print(f"测试结果: {passed} PASS, {failed} FAIL")
    print("=" * 60)