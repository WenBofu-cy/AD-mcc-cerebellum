#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-38
模块名称: 执行日志记录单元
所属分区: 十、执行反馈与日志
核心职责: 全链路记录 MCC 运动小脑所有操控指令、实际执行结果、偏差数据、异常事件及故障信息，
          生成不可篡改的审计日志。日志存储周期 ≥ 6 个月，支持按时间、模块、事件类型等多维度
          检索。为事故追溯、责任判定、系统调试及离线复盘提供完整的数据基础。不参与任何驾驶决策，
          仅负责日志的记录、存储与检索服务。

依赖模块:
    ad-mcc-01 至 ad-mcc-37(所有模块),
    ad-mcc-36(执行闭环反馈单元),
    ad-mcc-37(运动质量评估单元)
被依赖模块:
    ad-mcc-01(小脑总控调度核心),
    ECC-12(通过 CerebellumBus 查询),
    离线复盘系统

安全约束:
  S-01: 事故相关的关键日志（异常、故障、严重偏差）必须标记为不可覆写，存储周期 ≥ 3年
  S-02: 日志记录必须包含 UTC 时间戳、模块编号、事件类型等完整元数据
  S-03: 日志存储必须采用追加写模式，禁止修改或删除已落盘的日志条目
  S-04: 本模块仅负责日志的记录、存储与检索，不参与任何车辆控制决策
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
from collections import deque


class LoggerState(Enum):
    NORMAL_LOGGING = "normal_logging"
    LOW_STORAGE = "low_storage"
    STORAGE_FAULT = "storage_fault"
    SYSTEM_PAUSED = "system_paused"


class LogCategory(Enum):
    COMMAND = "操控指令"
    RESULT = "执行结果"
    DEVIATION = "偏差事件"
    FAULT = "异常/故障"
    QUALITY = "质量评估"
    STATE_CHANGE = "状态变更"


@dataclass
class LogEntry:
    log_id: str = ""
    timestamp: float = field(default_factory=time.time)
    category: LogCategory = LogCategory.STATE_CHANGE
    source_module: str = ""
    event_type: str = ""
    details: Dict[str, Any] = field(default_factory=dict)
    non_overwritable: bool = False


@dataclass
class LogQueryRequest:
    time_range: Tuple[float, float] = (0.0, 0.0)
    module_id: str = ""
    event_type: str = ""
    keyword: str = ""
    max_results: int = 100


@dataclass
class LogQueryResult:
    request: LogQueryRequest = field(default_factory=LogQueryRequest)
    entries: List[LogEntry] = field(default_factory=list)
    total_matches: int = 0
    query_duration_ms: float = 0.0
    complete: bool = True


@dataclass
class StorageStatus:
    total_capacity: int = 0
    used_capacity: int = 0
    remaining_capacity: int = 0
    earliest_log_time: float = 0.0
    latest_log_time: float = 0.0


@dataclass
class StorageAlert:
    alert_type: str = ""
    remaining_pct: float = 0.0
    cleaned_entries: int = 0
    earliest_log_time: float = 0.0


MAX_LOG_ENTRIES = 100000
LOW_STORAGE_THRESHOLD = 0.20
WRITE_FAILURE_THRESHOLD = 3
REPORT_INTERVAL_S = 10.0
BUFFER_MAX_SIZE = 5000


class ExecutionLogger:
    def __init__(self):
        self.module_id = "ad-mcc-38"
        self.module_name = "执行日志记录单元"
        self.version = "V1.0"

        self.state = LoggerState.NORMAL_LOGGING
        self._log_storage: List[LogEntry] = []
        self._buffer: deque = deque()
        self._write_failures = 0
        self._last_report_time = 0.0
        self._pending_queries: List[LogQueryRequest] = []

        self._query_log_event = None
        self._query_log_query = None

        self._publish_query_result = None
        self._publish_storage_status = None
        self._publish_storage_alert = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_log_event_query(self, callback):
        self._query_log_event = callback

    def set_log_query_query(self, callback):
        self._query_log_query = callback

    def set_query_result_publisher(self, callback):
        self._publish_query_result = callback

    def set_storage_status_publisher(self, callback):
        self._publish_storage_status = callback

    def set_storage_alert_publisher(self, callback):
        self._publish_storage_alert = callback

    def push_log(self, category: LogCategory, source: str, event_type: str, details: Dict[str, Any],
                 non_overwritable: bool = False):
        entry = LogEntry(
            log_id=f"log-{uuid.uuid4().hex[:8]}",
            category=category,
            source_module=source,
            event_type=event_type,
            details=details,
            non_overwritable=non_overwritable
        )
        if self.state == LoggerState.STORAGE_FAULT:
            self._buffer.append(entry)
            if len(self._buffer) > BUFFER_MAX_SIZE:
                self._buffer.popleft()
        else:
            self._persist_entry(entry)

    def run_logger_cycle(self):
        now = time.time()
        if self.state == LoggerState.SYSTEM_PAUSED:
            return

        # 处理外部查询请求
        query = self._query_log_query() if self._query_log_query else None
        if query:
            self._pending_queries.append(query)

        while self._pending_queries:
            req = self._pending_queries.pop(0)
            self._handle_query(req)

        # 尝试恢复存储
        if self.state == LoggerState.STORAGE_FAULT:
            # 尝试写入一条测试日志
            test_entry = LogEntry(category=LogCategory.STATE_CHANGE, source_module=self.module_id, event_type="test_write")
            if self._write_to_storage(test_entry):
                self.state = LoggerState.NORMAL_LOGGING
                self._write_failures = 0
                # 回放缓冲区
                while self._buffer:
                    self._persist_entry(self._buffer.popleft())

        # 存储空间检查
        if self.state != LoggerState.STORAGE_FAULT and len(self._log_storage) >= MAX_LOG_ENTRIES * (1 - LOW_STORAGE_THRESHOLD):
            self.state = LoggerState.LOW_STORAGE
            cleaned = self._perform_rollover()
            if self._publish_storage_alert:
                self._publish_storage_alert(StorageAlert(
                    alert_type="空间不足",
                    remaining_pct=round(1.0 - len(self._log_storage) / MAX_LOG_ENTRIES, 2),
                    cleaned_entries=cleaned,
                ))

        # 周期性上报
        if now - self._last_report_time >= REPORT_INTERVAL_S:
            self._last_report_time = now
            if self._publish_storage_status:
                self._publish_storage_status(self._get_storage_status())

    def _persist_entry(self, entry: LogEntry):
        if self.state == LoggerState.STORAGE_FAULT:
            self._buffer.append(entry)
            return
        if self._write_to_storage(entry):
            self._write_failures = 0
        else:
            self._write_failures += 1
            if self._write_failures >= WRITE_FAILURE_THRESHOLD:
                self.state = LoggerState.STORAGE_FAULT
                self._buffer.append(entry)
                if self._publish_storage_alert:
                    self._publish_storage_alert(StorageAlert(alert_type="存储故障", remaining_pct=0.0))

    def _write_to_storage(self, entry: LogEntry) -> bool:
        # 模拟存储写入，实际应写入文件或数据库
        try:
            self._log_storage.append(entry)
            return True
        except Exception:
            return False

    def _perform_rollover(self) -> int:
        cleaned = 0
        # 删除最旧的可覆写日志，直到占用低于80%
        target_count = int(MAX_LOG_ENTRIES * 0.8)
        remaining = [e for e in self._log_storage if e.non_overwritable]
        overwritable = [e for e in self._log_storage if not e.non_overwritable]
        # 保留最新的可覆写日志，删除旧的
        keep_count = max(0, target_count - len(remaining))
        if keep_count < len(overwritable):
            cleaned = len(overwritable) - keep_count
            overwritable = overwritable[-keep_count:] if keep_count > 0 else []
        self._log_storage = remaining + overwritable
        return cleaned

    def _handle_query(self, req: LogQueryRequest):
        start_time = time.time()
        results = []
        for entry in self._log_storage:
            if req.time_range[1] > 0 and entry.timestamp < req.time_range[0]:
                continue
            if req.time_range[1] > 0 and entry.timestamp > req.time_range[1]:
                continue
            if req.module_id and entry.source_module != req.module_id:
                continue
            if req.event_type and entry.event_type != req.event_type:
                continue
            if req.keyword and req.keyword not in str(entry.details):
                continue
            results.append(entry)
            if len(results) >= req.max_results:
                break
        elapsed = (time.time() - start_time) * 1000.0
        if self._publish_query_result:
            self._publish_query_result(LogQueryResult(
                request=req,
                entries=results,
                total_matches=len(results),
                query_duration_ms=elapsed,
                complete=len(results) < req.max_results
            ))

    def _get_storage_status(self) -> StorageStatus:
        times = [e.timestamp for e in self._log_storage]
        return StorageStatus(
            total_capacity=MAX_LOG_ENTRIES,
            used_capacity=len(self._log_storage),
            remaining_capacity=MAX_LOG_ENTRIES - len(self._log_storage),
            earliest_log_time=min(times) if times else 0.0,
            latest_log_time=max(times) if times else 0.0,
        )

    def get_state(self) -> LoggerState:
        return self.state

    def emergency_shutdown(self):
        self.state = LoggerState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 执行日志记录单元 (ad-mcc-38) 演示")
    print("=" * 70)

    logger = ExecutionLogger()

    print_separator("STEP 1: 写入操控指令日志")
    logger.push_log(LogCategory.COMMAND, "ad-mcc-01", "throttle_command", {"target": 30.0})
    print(f"  已写入 {len(logger._log_storage)} 条日志")

    print_separator("STEP 2: 写入严重故障日志（不可覆写）")
    logger.push_log(LogCategory.FAULT, "ad-mcc-28", "sensor_failure", {"sensor": "steering_angle"}, non_overwritable=True)
    print(f"  已写入 {len(logger._log_storage)} 条日志，不可覆写: {sum(1 for e in logger._log_storage if e.non_overwritable)}")

    print_separator("STEP 3: 查询日志")
    logger.set_log_query_query(lambda: LogQueryRequest(module_id="ad-mcc-01"))
    logger.run_logger_cycle()
    print("  查询完成 (查看回调输出)")

    print("\n✅ 执行日志记录单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-38 执行日志记录单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_logger():
            l = ExecutionLogger()
            return l

        print("\n[TC-M38-01] 正常写入日志")
        try:
            l = setup_logger()
            l.push_log(LogCategory.COMMAND, "ad-mcc-01", "steer_cmd", {"angle": 15.0})
            assert len(l._log_storage) == 1
            assert l._log_storage[0].source_module == "ad-mcc-01"
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M38-02] 不可覆写日志标记")
        try:
            l = setup_logger()
            l.push_log(LogCategory.FAULT, "ad-mcc-28", "critical", {}, non_overwritable=True)
            assert l._log_storage[0].non_overwritable
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M38-03] 查询日志")
        try:
            l = setup_logger()
            l.push_log(LogCategory.COMMAND, "ad-mcc-01", "test", {})
            result = None
            def trap_result(r):
                nonlocal result
                result = r
            l.set_query_result_publisher(trap_result)
            l.set_log_query_query(lambda: LogQueryRequest(module_id="ad-mcc-01", max_results=10))
            l.run_logger_cycle()
            assert result is not None and result.total_matches == 1
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M38-04] 滚动清理")
        try:
            l = setup_logger()
            # 填满日志
            for i in range(MAX_LOG_ENTRIES):
                l._log_storage.append(LogEntry(non_overwritable=(i < 100)))  # 前100条不可覆写
            l.state = LoggerState.LOW_STORAGE
            cleaned = l._perform_rollover()
            assert cleaned > 0
            # 不可覆写日志应保留
            non_overwritable_count = sum(1 for e in l._log_storage if e.non_overwritable)
            assert non_overwritable_count == 100
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M38-05] 存储故障切换缓冲区")
        try:
            l = setup_logger()
            # 模拟写入失败
            def fail_write(entry):
                return False
            l._write_to_storage = fail_write
            l.push_log(LogCategory.COMMAND, "ad-mcc-01", "test", {})
            l.push_log(LogCategory.COMMAND, "ad-mcc-01", "test", {})
            l.push_log(LogCategory.COMMAND, "ad-mcc-01", "test", {})
            l.push_log(LogCategory.COMMAND, "ad-mcc-01", "test", {})
            assert l.state == LoggerState.STORAGE_FAULT
            assert len(l._buffer) > 0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        print("\n[TC-M38-06] 紧急熔断")
        try:
            l = setup_logger()
            l.emergency_shutdown()
            assert l.state == LoggerState.SYSTEM_PAUSED
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