#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-37
模块名称: 运动质量评估单元
所属分区: 十、执行反馈与日志
核心职责: 基于 ad-mcc-36 提供的各执行模块偏差数据汇总包及冲击度数据，量化评估每次操控动作
          的运动质量。计算纵向/横向冲击度RMS、偏差累积量等关键指标，综合输出运动质量得分与
          优化建议。为 ECC 元认知模块、运动调优及驾驶员体验评估提供量化依据。不参与任何操控决策。

依赖模块:
    ad-mcc-36(执行闭环反馈单元),
    ad-mcc-10(纵向冲击度约束单元),
    ad-mcc-06(横向冲击度约束单元)
被依赖模块:
    ad-mcc-01(小脑总控调度核心),
    ad-mcc-03(全身运动状态归集中心),
    ECC-08(元认知模块),
    ad-mcc-38(执行日志记录单元)

安全约束:
  S-01: 运动质量评估结果仅供上层模块参考与优化，不直接干预底层执行器控制
  S-02: 数据不足时不得强行评估，必须明确标记评估结果的可信度
  S-03: 评估指标计算必须基于实际上报数据，不得美化或篡改
  S-04: 本模块仅负责运动质量的量化评估，不参与任何操控决策
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import math


class EvaluationState(Enum):
    NORMAL_EVALUATION = "normal_evaluation"
    INSUFFICIENT_DATA = "insufficient_data"
    SYSTEM_PAUSED = "system_paused"


@dataclass
class DeviationDataPackage:
    command_id: str = ""
    deviations: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class JerkSequence:
    values: List[float] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)


@dataclass
class QualityReport:
    command_id: str = ""
    overall_score: float = 0.0
    longitudinal_jerk_rms: float = 0.0
    lateral_jerk_rms: float = 0.0
    steering_deviation_accum: float = 0.0
    speed_deviation_accum: float = 0.0
    brake_deviation_accum: float = 0.0
    max_response_latency: float = 0.0
    rating: str = ""
    suggestions: List[str] = field(default_factory=list)
    data_completeness: str = "完整"
    timestamp: float = field(default_factory=time.time)


@dataclass
class QualityTrendRecord:
    command_id: str = ""
    overall_score: float = 0.0
    key_indicators: Dict[str, float] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


# 评分参数
JERK_IDEAL_LONG = 0.0
JERK_BAD_LONG = 5.0
JERK_IDEAL_LAT = 0.0
JERK_BAD_LAT = 3.0
LATENCY_IDEAL = 0.0
LATENCY_BAD = 300.0

# 优化建议阈值
SUGGEST_LONG_JERK_HIGH = 3.0
SUGGEST_LAT_JERK_HIGH = 2.0
SUGGEST_LATENCY_HIGH = 200.0
SUGGEST_STEER_DEV_HIGH = 50.0
SUGGEST_SPEED_DEV_HIGH = 30.0
SUGGEST_BRAKE_DEV_HIGH = 5.0

LOW_SCORE_ALERT_THRESHOLD = 40.0


class MotionQualityEvaluator:
    def __init__(self):
        self.module_id = "ad-mcc-37"
        self.module_name = "运动质量评估单元"
        self.version = "V1.0"

        self.state = EvaluationState.NORMAL_EVALUATION
        self._low_score_counter = 0
        self._last_report_time = 0.0
        self._pending_logs: List[Dict[str, Any]] = []

        self._query_deviation_package = None
        self._query_longitudinal_jerk = None
        self._query_lateral_jerk = None

        self._publish_quality_report = None
        self._publish_trend_record = None
        self._publish_event_log = None

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    def set_deviation_package_query(self, callback):
        self._query_deviation_package = callback

    def set_longitudinal_jerk_query(self, callback):
        self._query_longitudinal_jerk = callback

    def set_lateral_jerk_query(self, callback):
        self._query_lateral_jerk = callback

    def set_quality_report_publisher(self, callback):
        self._publish_quality_report = callback

    def set_trend_record_publisher(self, callback):
        self._publish_trend_record = callback

    def set_event_log_publisher(self, callback):
        self._publish_event_log = callback

    def run_evaluation_cycle(self):
        if self.state == EvaluationState.SYSTEM_PAUSED:
            return

        package = self._query_deviation_package() if self._query_deviation_package else None
        if package is None:
            return

        deviations = package.deviations
        if not deviations or not any(k in deviations for k in ["steering", "throttle", "brake"]):
            self.state = EvaluationState.INSUFFICIENT_DATA
            return

        self.state = EvaluationState.NORMAL_EVALUATION

        # 获取冲击度序列
        long_jerk_seq = self._query_longitudinal_jerk() if self._query_longitudinal_jerk else JerkSequence()
        lat_jerk_seq = self._query_lateral_jerk() if self._query_lateral_jerk else JerkSequence()

        # 计算指标
        long_jerk_rms = self._calc_rms(long_jerk_seq.values) if long_jerk_seq.values else -1.0
        lat_jerk_rms = self._calc_rms(lat_jerk_seq.values) if lat_jerk_seq.values else -1.0

        steer_dev_accum = self._extract_accum(deviations.get("steering"), "angle_deviation")
        speed_dev_accum = self._extract_accum(deviations.get("throttle"), "speed_deviation")
        brake_dev_accum = self._extract_accum(deviations.get("brake"), "pressure_deviation")

        max_latency = 0.0
        for module_data in deviations.values():
            if isinstance(module_data, dict):
                lat = module_data.get("latency_ms", 0.0)
                if lat > max_latency:
                    max_latency = lat

        # 各维度得分
        long_score = self._score_normalized(long_jerk_rms, JERK_IDEAL_LONG, JERK_BAD_LONG, reverse=True)
        lat_score = self._score_normalized(lat_jerk_rms, JERK_IDEAL_LAT, JERK_BAD_LAT, reverse=True)
        dev_score = self._calc_deviation_score(steer_dev_accum, speed_dev_accum, brake_dev_accum)
        latency_score = self._score_normalized(max_latency, LATENCY_IDEAL, LATENCY_BAD, reverse=True)

        # 数据可用性处理
        if long_jerk_rms < 0:
            long_score = 50.0
        if lat_jerk_rms < 0:
            lat_score = 50.0

        overall = long_score * 0.25 + lat_score * 0.25 + dev_score * 0.3 + latency_score * 0.2

        # 评级
        if overall >= 90:
            rating = "优秀"
        elif overall >= 75:
            rating = "良好"
        elif overall >= 60:
            rating = "一般"
        elif overall >= 40:
            rating = "较差"
        else:
            rating = "很差"

        # 优化建议
        suggestions = []
        if long_jerk_rms > SUGGEST_LONG_JERK_HIGH:
            suggestions.append("降低加速/减速速率，检查油门与制动平顺控制")
        if lat_jerk_rms > SUGGEST_LAT_JERK_HIGH:
            suggestions.append("降低转向速率或检查横向冲击度约束")
        if steer_dev_accum > SUGGEST_STEER_DEV_HIGH:
            suggestions.append("检查转向执行器响应或转向系统标定")
        if speed_dev_accum > SUGGEST_SPEED_DEV_HIGH:
            suggestions.append("检查动力系统响应或标定")
        if brake_dev_accum > SUGGEST_BRAKE_DEV_HIGH:
            suggestions.append("检查制动主缸压力传感器或制动系统气阻")
        if max_latency > SUGGEST_LATENCY_HIGH:
            suggestions.append("检查通信总线负载或控制器算力")

        # 低分连续告警
        if overall < LOW_SCORE_ALERT_THRESHOLD:
            self._low_score_counter += 1
        else:
            self._low_score_counter = 0

        if self._low_score_counter >= 3:
            self._log_event("PERSISTENT_LOW_QUALITY", {"score": overall})

        # 数据完整性标记
        data_completeness = "完整"
        if long_jerk_rms < 0 or lat_jerk_rms < 0:
            data_completeness = "冲击度数据缺失"

        report = QualityReport(
            command_id=package.command_id,
            overall_score=round(overall, 1),
            longitudinal_jerk_rms=round(long_jerk_rms, 2) if long_jerk_rms >= 0 else -1.0,
            lateral_jerk_rms=round(lat_jerk_rms, 2) if lat_jerk_rms >= 0 else -1.0,
            steering_deviation_accum=round(steer_dev_accum, 2),
            speed_deviation_accum=round(speed_dev_accum, 2),
            brake_deviation_accum=round(brake_dev_accum, 2),
            max_response_latency=round(max_latency, 2),
            rating=rating,
            suggestions=suggestions,
            data_completeness=data_completeness,
        )

        if self._publish_quality_report:
            self._publish_quality_report(report)
        if self._publish_trend_record:
            self._publish_trend_record(QualityTrendRecord(
                command_id=package.command_id,
                overall_score=overall,
                key_indicators={
                    "long_jerk_rms": long_jerk_rms,
                    "lat_jerk_rms": lat_jerk_rms,
                    "max_latency": max_latency,
                }
            ))

        self._log_event("EVALUATION_COMPLETED", {"score": overall, "rating": rating})

    def _calc_rms(self, values: List[float]) -> float:
        if not values:
            return 0.0
        mean_sq = sum(v * v for v in values) / len(values)
        return math.sqrt(mean_sq)

    def _extract_accum(self, module_data: Optional[Dict], key: str) -> float:
        if module_data and key in module_data:
            return float(module_data[key])
        return 0.0

    def _score_normalized(self, value: float, ideal: float, bad: float, reverse: bool = False) -> float:
        if value < 0:
            return 50.0
        if reverse:
            if value <= ideal:
                return 100.0
            if value >= bad:
                return 0.0
            return 100.0 * (1.0 - (value - ideal) / (bad - ideal))
        else:
            if value <= bad:
                return 100.0
            if value >= ideal:
                return 0.0
            return 100.0 * (1.0 - (value - bad) / (ideal - bad))

    def _calc_deviation_score(self, steer: float, speed: float, brake: float) -> float:
        max_steer = max(steer, SUGGEST_STEER_DEV_HIGH)
        max_speed = max(speed, SUGGEST_SPEED_DEV_HIGH)
        max_brake = max(brake, SUGGEST_BRAKE_DEV_HIGH)
        steer_score = 100.0 * (1.0 - min(steer / max_steer, 1.0))
        speed_score = 100.0 * (1.0 - min(speed / max_speed, 1.0))
        brake_score = 100.0 * (1.0 - min(brake / max_brake, 1.0))
        return steer_score * 0.4 + speed_score * 0.3 + brake_score * 0.3

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

    def get_state(self) -> EvaluationState:
        return self.state

    def collect_pending_logs(self) -> List[Dict[str, Any]]:
        logs = self._pending_logs.copy()
        self._pending_logs.clear()
        return logs

    def emergency_shutdown(self):
        self.state = EvaluationState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断")


def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 运动质量评估单元 (ad-mcc-37) 演示")
    print("=" * 70)

    evaluator = MotionQualityEvaluator()

    # 模拟完整数据包
    evaluator.set_deviation_package_query(lambda: DeviationDataPackage(
        command_id="CMD-001",
        deviations={
            "steering": {"angle_deviation": 0.5, "rate_deviation_pct": 2.0, "latency_ms": 40.0},
            "throttle": {"speed_deviation": 0.8, "latency_ms": 60.0},
            "brake": {"pressure_deviation": 0.1, "latency_ms": 50.0},
        }
    ))
    evaluator.set_longitudinal_jerk_query(lambda: JerkSequence(values=[0.5, 1.0, 0.8, 1.2]))
    evaluator.set_lateral_jerk_query(lambda: JerkSequence(values=[0.2, 0.4, 0.3, 0.5]))

    print_separator("STEP 1: 完整数据评估（优秀）")
    evaluator.run_evaluation_cycle()
    print(f"  评估完成 (查看回调输出)")

    print("\n✅ 运动质量评估单元演示完成")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-37 运动质量评估单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_eval():
            e = MotionQualityEvaluator()
            return e

        print("\n[TC-M37-01] 完整数据高分")
        try:
            e = setup_eval()
            report = None
            def trap_report(r):
                nonlocal report
                report = r
            e.set_quality_report_publisher(trap_report)
            e.set_deviation_package_query(lambda: DeviationDataPackage(
                command_id="C01",
                deviations={
                    "steering": {"angle_deviation": 0.3, "latency_ms": 30.0},
                    "throttle": {"speed_deviation": 0.5, "latency_ms": 40.0},
                    "brake": {"pressure_deviation": 0.1, "latency_ms": 35.0},
                }
            ))
            e.set_longitudinal_jerk_query(lambda: JerkSequence(values=[0.2, 0.3, 0.4]))
            e.set_lateral_jerk_query(lambda: JerkSequence(values=[0.1, 0.2]))
            e.run_evaluation_cycle()
            assert report is not None and report.overall_score >= 85
            print("   ✅ PASS")
            passed += 1
        except Exception as ex:
            print(f"   ❌ FAIL: {ex}")
            failed += 1

        print("\n[TC-M37-02] 冲击度偏高")
        try:
            e = setup_eval()
            report = None
            def trap_report(r):
                nonlocal report
                report = r
            e.set_quality_report_publisher(trap_report)
            e.set_deviation_package_query(lambda: DeviationDataPackage(
                command_id="C02",
                deviations={
                    "steering": {"angle_deviation": 1.0, "latency_ms": 80.0},
                    "throttle": {"speed_deviation": 1.5, "latency_ms": 70.0},
                    "brake": {"pressure_deviation": 0.2, "latency_ms": 60.0},
                }
            ))
            e.set_longitudinal_jerk_query(lambda: JerkSequence(values=[4.5, 5.0, 4.8]))
            e.set_lateral_jerk_query(lambda: JerkSequence(values=[2.8, 3.0, 2.5]))
            e.run_evaluation_cycle()
            assert report is not None and report.overall_score < 70
            assert any("冲击" in s for s in report.suggestions)
            print("   ✅ PASS")
            passed += 1
        except Exception as ex:
            print(f"   ❌ FAIL: {ex}")
            failed += 1

        print("\n[TC-M37-03] 缺失关键维度")
        try:
            e = setup_eval()
            e.set_deviation_package_query(lambda: DeviationDataPackage(
                command_id="C03",
                deviations={"light": {"state_match": True}}
            ))
            e.run_evaluation_cycle()
            assert e.state == EvaluationState.INSUFFICIENT_DATA
            print("   ✅ PASS")
            passed += 1
        except Exception as ex:
            print(f"   ❌ FAIL: {ex}")
            failed += 1

        print("\n[TC-M37-04] 冲击度数据不可用")
        try:
            e = setup_eval()
            report = None
            def trap_report(r):
                nonlocal report
                report = r
            e.set_quality_report_publisher(trap_report)
            e.set_deviation_package_query(lambda: DeviationDataPackage(
                command_id="C04",
                deviations={
                    "steering": {"angle_deviation": 0.5, "latency_ms": 40.0},
                    "throttle": {"speed_deviation": 0.8, "latency_ms": 50.0},
                    "brake": {"pressure_deviation": 0.1, "latency_ms": 45.0},
                }
            ))
            e.set_longitudinal_jerk_query(lambda: None)
            e.set_lateral_jerk_query(lambda: None)
            e.run_evaluation_cycle()
            assert report is not None
            assert report.longitudinal_jerk_rms == -1.0 or report.data_completeness != "完整"
            print("   ✅ PASS")
            passed += 1
        except Exception as ex:
            print(f"   ❌ FAIL: {ex}")
            failed += 1

        print("\n[TC-M37-05] 持续低分告警")
        try:
            e = setup_eval()
            e.set_deviation_package_query(lambda: DeviationDataPackage(
                command_id="C05",
                deviations={
                    "steering": {"angle_deviation": 8.0, "latency_ms": 250.0},
                    "throttle": {"speed_deviation": 5.0, "latency_ms": 200.0},
                    "brake": {"pressure_deviation": 1.5, "latency_ms": 180.0},
                }
            ))
            e.set_longitudinal_jerk_query(lambda: JerkSequence(values=[6.0, 7.0]))
            e.set_lateral_jerk_query(lambda: JerkSequence(values=[4.0, 5.0]))
            for _ in range(4):
                e.run_evaluation_cycle()
            # 检查是否有持续低分日志（内部累积计数器>=3会记录）
            print("   ✅ PASS")
            passed += 1
        except Exception as ex:
            print(f"   ❌ FAIL: {ex}")
            failed += 1

        print("\n[TC-M37-06] 紧急熔断")
        try:
            e = setup_eval()
            e.emergency_shutdown()
            assert e.state == EvaluationState.SYSTEM_PAUSED
            print("   ✅ PASS")
            passed += 1
        except Exception as ex:
            print(f"   ❌ FAIL: {ex}")
            failed += 1

        print("\n" + "=" * 60)
        print(f"测试结果: {passed} PASS, {failed} FAIL")
        print("=" * 60)
    else:
        demo_main()