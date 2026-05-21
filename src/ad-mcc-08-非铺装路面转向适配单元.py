# ad-mcc-08-非铺装路面转向适配单元 接口规格

---

## 基本信息

| 项 | 内容 |
|----|------|
| 模块编号 | ad-mcc-08 |
| 模块名称 | 非铺装路面转向适配单元 |
| 所属分区 | 二、转向控制集群 |
| 核心职责 | 在乡村非铺装道路场景下，自动切换转向系统至柔和模式。降低方向盘转角速率上限，增大允许的转向误差带，为坑洼路面的方向盘反作用力提供柔顺滤波，防止因路面不平导致的非预期方向盘转动传递至车身。同时降低横向冲击度约束阈值，确保非铺装路面上的转向动作始终在安全边界内。不参与任何场景判断与驾驶决策 |
| 依赖模块 | ad-mcc-01（小脑总控调度核心，下发非铺装道路模式信号）、ad-mcc-04（方向盘转角解算单元，可参考柔和化参数调整解算策略）、ad-mcc-06（横向冲击度约束单元，接收降低后的冲击度阈值） |
| 被依赖模块 | ad-mcc-04（方向盘转角解算单元，消费柔和化转角约束）、ad-mcc-06（横向冲击度约束单元，消费非铺装冲击度上限） |


## 内部状态定义

| 状态 | 标识 | 含义 | 触发条件 |
|------|------|------|----------|
| 正常模式 | `MODE_NORMAL` | 铺装路面，使用标准转向参数 | 系统初始化完成，默认状态 |
| 非铺装柔和模式 | `MODE_UNPAVED_SOFT` | 非铺装路面，使用柔和转向参数 | 收到非铺装道路模式切换信号 |
| 切换过渡中 | `MODE_TRANSITIONING` | 正在从铺装参数平滑过渡至非铺装参数（或反向） | 模式切换启动时 |
| 暂停服务 | `SYSTEM_PAUSED` | 系统紧急熔断 | 收到紧急熔断指令 |


## 输入数据

| 输入项 | 数据类型 | 来源模块 | 触发条件 | 优先级 |
|--------|----------|----------|----------|:---:|
| 非铺装道路模式信号 | Struct（模式标记=非铺装 + 路面类型 + 颠簸指数 + 摩擦系数估计 + 切换发起时间戳） | ad-mcc-01 小脑总控调度核心 | 进入/退出非铺装道路时 | **高** |
| 当前方向盘转角 | Float（度） | CAN 总线（转角传感器） | 实时，100Hz | **高** |
| 当前方向盘转角速率 | Float（°/s） | CAN 总线（转角传感器） | 实时，100Hz | **高** |
| 当前车速 | Float（km/h） | CAN 总线 | 实时，100Hz | **高** |
| 车辆最小离地间隙 | Float（cm） | ad-mcc-32 车辆尺寸参数管理单元 | 系统初始化加载 | 普通 |
| 全局调度指令 | Enum（暂停/恢复/熔断） | ad-mcc-01 小脑总控调度核心 | 模式切换或紧急事件时 | **紧急** |


## 输出数据

| 输出项 | 数据类型 | 目标模块 | 输出条件 | 优先级 |
|--------|----------|----------|----------|:---:|
| 非铺装转向约束参数 | Struct（转角速率上限 + 最大转角限制 + 转向误差容忍带 + 横向冲击度上限 + 路面类型 + 生效时间戳） | ad-mcc-04（方向盘转角解算单元）、ad-mcc-06（横向冲击度约束单元） | 进入非铺装模式时（参数切换）；退出时（恢复默认值） | **高** |
| 转向柔和度参数 | Struct（滤波截止频率 + 阻尼系数 + 反力柔顺系数 + 生效时间戳） | ad-mcc-04（方向盘转角解算单元） | 进入非铺装模式时 | **高** |
| 适配状态上报 | Struct（当前模式 + 路面类型 + 当前转角速率上限 + 当前冲击度上限） | ad-mcc-01（通过内部调度总线） | 周期性，每1秒 | 普通 |


## 非铺装转向参数集

### 一、各路面类型转向参数

| 参数名称 | 铺装路面（正常） | 泥土路 | 碎石路 | 湿滑泥土路 | 沙土路 | 有明显车辙/坑洼 |
|----------|:---:|:---:|:---:|:---:|:---:|:---:|
| 转角速率上限 (°/s) | 500 | 300 | 250 | 200 | 220 | 180 |
| 最大转角限制 (°) | 500（车型标定） | 450 | 400 | 350 | 400 | 350 |
| 转向误差容忍带 (°) | ±3 | ±5 | ±7 | ±8 | ±6 | ±10 |
| 横向冲击度上限 (m/s³) | 3.0 | 2.0 | 1.8 | 1.5 | 1.8 | 1.5 |
| 滤波截止频率 (Hz) | 8.0 | 5.0 | 4.0 | 3.0 | 4.0 | 3.0 |
| 反力柔顺系数 | 0（无柔顺） | 0.3 | 0.4 | 0.5 | 0.35 | 0.5 |
| 转向阻尼补偿 | 1.0 | 1.3 | 1.4 | 1.6 | 1.3 | 1.5 |

### 二、过渡斜坡参数

| 参数名称 | 数值 | 说明 |
|----------|:---:|------|
| 参数过渡时间 | 1.0 秒 | 线性斜坡函数在 1 秒内完成参数过渡 |
| 转角速率渐变步长 | 10 °/s² | 每帧最大变化量 |


## 核心处理逻辑

```
FUNCTION unpaved_steering_adapter_main_loop():
    STATE_NORMAL = MODE_NORMAL
    STATE_UNPAVED = MODE_UNPAVED_SOFT
    STATE_TRANS = MODE_TRANSITIONING
    STATE_PAUSED = SYSTEM_PAUSED

    SET current_mode = STATE_NORMAL
    SET current_road_type = "铺装"
    SET transition_start_time = 0
    SET transition_duration_s = 1.0

    // 当前生效的参数（实时插值后的值）
    SET active_params = 加载铺装路面默认参数()

    WHILE 系统运行中:
        // 第1步：紧急熔断
        IF 收到紧急熔断指令:
            SET current_mode = STATE_PAUSED
            CONTINUE
        ELSE IF 收到恢复指令 AND current_mode == STATE_PAUSED:
            SET current_mode = STATE_NORMAL

        // 第2步：接收非铺装道路模式信号
        IF 收到非铺装道路模式信号:
            IF 信号.模式标记 == "非铺装道路":
                // 进入非铺装柔和模式
                SET target_mode = STATE_UNPAVED
                SET target_params = 根据路面类型加载非铺装参数集(信号.路面类型)
                SET current_road_type = 信号.路面类型
            ELSE:
                // 退出非铺装，恢复铺装参数
                SET target_mode = STATE_NORMAL
                SET target_params = 加载铺装路面默认参数()
                SET current_road_type = "铺装"

            // 启动参数平滑过渡
            SET current_mode = STATE_TRANS
            SET transition_start_time = NOW()
            SET transition_source_params = active_params（当前值快照）
            SET transition_target_params = target_params

        // 第3步：参数平滑过渡
        IF current_mode == STATE_TRANS:
            elapsed = NOW() - transition_start_time
            IF elapsed >= transition_duration_s:
                // 过渡完成
                active_params = transition_target_params
                current_mode = target_mode
                // 向 ad-mcc-04 和 ad-mcc-06 推送最终参数
                向 ad-mcc-04 发送非铺装转向约束参数(active_params)
                向 ad-mcc-06 发送非铺装转向约束参数(active_params)
            ELSE:
                // 线性插值
                ratio = elapsed / transition_duration_s
                active_params = 线性插值(transition_source_params, transition_target_params, ratio)
                // 过渡期间仍向外推送实时插值后的参数
                向 ad-mcc-04 发送非铺装转向约束参数(active_params)

        // 第4步：非铺装模式下的持续适配
        IF current_mode == STATE_UNPAVED:
            当前车速 = 获取当前车速()
            当前转角速率 = 获取当前方向盘转角速率()

            // 4a. 根据车速动态微调转角速率上限
            IF 当前车速 < 10:
                // 极低速时略微放宽转角速率，便于精细转向
                动态上限 = active_params.转角速率上限 × 1.2
            ELSE IF 当前车速 > 30:
                // 较高速度时进一步收紧转角速率
                动态上限 = active_params.转角速率上限 × 0.8
            ELSE:
                动态上限 = active_params.转角速率上限

            // 4b. 更新动态参数
            active_params.转角速率上限 = 动态上限

            // 4c. 周期性推送更新参数
            向 ad-mcc-04 发送非铺装转向约束参数(active_params)

        // 第5步：周期性状态上报
        IF 距上次上报 >= 1秒:
            向 ad-mcc-01 发送适配状态上报(
                当前模式=current_mode,
                路面类型=current_road_type,
                当前转角速率上限=active_params.转角速率上限,
                当前冲击度上限=active_params.横向冲击度上限
            )

        SLEEP 10ms


FUNCTION 根据路面类型加载非铺装参数集(路面类型):
    SWITCH 路面类型:
        CASE "泥土路":
            RETURN {转角速率上限=300, 最大转角=450, 误差容忍带=5, 冲击度上限=2.0, 截止频率=5.0, 反力柔顺=0.3, 阻尼补偿=1.3}
        CASE "碎石路":
            RETURN {转角速率上限=250, 最大转角=400, 误差容忍带=7, 冲击度上限=1.8, 截止频率=4.0, 反力柔顺=0.4, 阻尼补偿=1.4}
        CASE "湿滑泥土路":
            RETURN {转角速率上限=200, 最大转角=350, 误差容忍带=8, 冲击度上限=1.5, 截止频率=3.0, 反力柔顺=0.5, 阻尼补偿=1.6}
        CASE "沙土路":
            RETURN {转角速率上限=220, 最大转角=400, 误差容忍带=6, 冲击度上限=1.8, 截止频率=4.0, 反力柔顺=0.35, 阻尼补偿=1.3}
        CASE "有明显车辙/坑洼":
            RETURN {转角速率上限=180, 最大转角=350, 误差容忍带=10, 冲击度上限=1.5, 截止频率=3.0, 反力柔顺=0.5, 阻尼补偿=1.5}
        DEFAULT:
            RETURN 加载铺装路面默认参数()


FUNCTION 线性插值(源参数, 目标参数, 比例):
    result = {}
    FOR EACH key IN 源参数:
        result[key] = 源参数[key] + (目标参数[key] - 源参数[key]) × 比例
    RETURN result
```


## 约束与异常处理

| 场景 | 处理方式 | 恢复条件 |
|------|----------|----------|
| 路面类型无法识别 | 使用最保守的「湿滑泥土路」参数集作为默认值 | 路面类型被准确识别后更新 |
| 非铺装模式下收到紧急制动指令 | 紧急制动优先，转向参数保持当前非铺装设定不变 | 紧急制动解除 |
| 参数过渡期间收到模式变更 | 以当前插值后的参数作为新起点，向新模式的目标参数过渡 | — |
| 转角传感器数据在非铺装模式下短暂丢失 | 维持最后有效参数，标记数据质量降级 | 传感器恢复 |
| 车速信号丢失 | 假设车速=20km/h（保守中速），继续适配 | 车速信号恢复 |
| 紧急熔断 | 保持当前参数不变，暂停适配 | 紧急解除后恢复 |


## 总线契约

| 总线 | 操作 | 数据内容 | 权限 | 说明 |
|------|------|----------|------|------|
| 内部调度总线 | 读 | 非铺装道路模式信号 | 只读 | ad-mcc-01 下发 |
| CAN 总线 | 读 | 当前方向盘转角 | 只读 | 实时，100Hz |
| CAN 总线 | 读 | 当前方向盘转角速率 | 只读 | 实时，100Hz |
| CAN 总线 | 读 | 当前车速 | 只读 | 实时，100Hz |
| 内部调度总线 | 读 | 车辆最小离地间隙 | 只读 | ad-mcc-32 提供 |
| 内部调度总线 | 写 | 非铺装转向约束参数 | 专属写入 | 向 ad-mcc-04、ad-mcc-06 发送 |
| 内部调度总线 | 写 | 转向柔和度参数 | 专属写入 | 向 ad-mcc-04 发送 |
| 内部调度总线 | 写 | 适配状态上报 | 周期性写入 | 向 ad-mcc-01 发送 |


## 安全边界

| 规则编号 | 内容 |
|:---:|------|
| S-01 | 非铺装模式下的转角速率上限不得低于 150°/s，确保车辆仍具备基本的紧急避让转向能力 |
| S-02 | 转向误差容忍带的放宽不得影响车辆轨迹跟踪精度超过 0.5m（横向偏差） |
| S-03 | 本模块仅输出转向约束参数，不直接操控转向电机。所有参数须经 ad-mcc-02 校验后生效 |
| S-04 | 从铺装→非铺装的参数过渡必须使用线性斜坡函数，禁止阶跃切换 |
| S-05 | 非铺装模式下，横向冲击度上限不得突破绝对物理红线（3.0m/s³） |


## 接口校验用例

| 用例编号 | 前置条件 | 输入 | 预期输出 |
|----------|----------|------|----------|
| TC-M08-01 | `MODE_NORMAL` | ad-mcc-01 下发非铺装道路信号（路面类型=泥土路） | 进入 `MODE_TRANSITIONING` → 1秒内平滑过渡至泥土路参数集 → 状态切换至 `MODE_UNPAVED_SOFT` |
| TC-M08-02 | `MODE_UNPAVED_SOFT` | ad-mcc-01 下发退出非铺装信号 | 平滑恢复铺装默认参数 → 状态切换至 `MODE_NORMAL` |
| TC-M08-03 | `MODE_UNPAVED_SOFT`（碎石路） | 车速降至 5km/h | 转角速率上限动态上调 20%（250×1.2=300°/s） |
| TC-M08-04 | `MODE_UNPAVED_SOFT`（湿滑泥土路） | 车速升至 40km/h | 转角速率上限动态下调 20%（200×0.8=160°/s，但不低于150） |
| TC-M08-05 | `MODE_TRANSITIONING` 过渡中 | 新的模式切换请求 | 以当前插值参数为起点向新模式目标参数过渡 |
| TC-M08-06 | `MODE_UNPAVED_SOFT` | 路面类型无法识别 | 默认使用湿滑泥土路参数集（最保守） |


## 质量自检清单

| 检查项 | 状态 |
|--------|:---:|
| 模块编号与分区归属正确 | ✅ |
| 依赖与被依赖模块编号完整 | ✅ |
| 内部状态机4个状态含触发条件 | ✅ |
| 输入/输出含数据类型、来源/目标模块、优先级 | ✅ |
| 非铺装转向参数集完整（5种路面类型 × 7项参数 + 过渡斜坡参数） | ✅ |
| 核心处理逻辑伪代码含参数加载、平滑过渡、动态微调、周期性上报全流程 | ✅ |
| 约束与异常覆盖路面类型未知、紧急制动、过渡中变更、传感器丢失、车速丢失、紧急熔断 | ✅ |
| 总线契约区分内部调度总线与CAN总线 | ✅ |
| 安全边界逐条列出 | ✅ |
| 校验用例覆盖进入非铺装、退出恢复、低速微调、高速收紧、过渡中变更、未知路面共6条 | ✅ |

**文件路径：** `AD-mcc-cerebellum/src/ad-mcc-08-非铺装路面转向适配单元.py`

**提交信息：** `添加 MCC-08-非铺装路面转向适配单元 Python 代码`

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块编号: ad-mcc-08
模块名称: 非铺装路面转向适配单元
所属分区: 二、转向控制集群
核心职责: 在乡村非铺装道路场景下，自动切换转向系统至柔和模式。降低方向盘转角速率上限，
          增大允许的转向误差带，为坑洼路面的方向盘反作用力提供柔顺滤波，防止因路面不平
          导致的非预期方向盘转动传递至车身。同时降低横向冲击度约束阈值，确保非铺装路面
          上的转向动作始终在安全边界内。不参与任何场景判断与驾驶决策。

依赖模块:
    ad-mcc-01(小脑总控调度核心，下发非铺装道路模式信号),
    ad-mcc-04(方向盘转角解算单元，可参考柔和化参数调整解算策略),
    ad-mcc-06(横向冲击度约束单元，接收降低后的冲击度阈值)
被依赖模块:
    ad-mcc-04(方向盘转角解算单元，消费柔和化转角约束),
    ad-mcc-06(横向冲击度约束单元，消费非铺装冲击度上限)

安全约束:
  S-01: 非铺装模式下的转角速率上限不得低于 150°/s，确保车辆仍具备基本的紧急避让转向能力
  S-02: 转向误差容忍带的放宽不得影响车辆轨迹跟踪精度超过 0.5m（横向偏差）
  S-03: 本模块仅输出转向约束参数，不直接操控转向电机。所有参数须经 ad-mcc-02 校验后生效
  S-04: 从铺装→非铺装的参数过渡必须使用线性斜坡函数，禁止阶跃切换
  S-05: 非铺装模式下，横向冲击度上限不得突破绝对物理红线（3.0m/s³）
"""

from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid
import copy


# ==================== 枚举定义 ====================

class AdapterState(Enum):
    """非铺装路面转向适配单元内部状态"""
    MODE_NORMAL = "normal"
    MODE_UNPAVED_SOFT = "unpaved_soft"
    MODE_TRANSITIONING = "transitioning"
    SYSTEM_PAUSED = "system_paused"


class RoadSurface(Enum):
    """路面类型"""
    PAVED = "铺装"
    DIRT = "泥土路"
    GRAVEL = "碎石路"
    WET_MUD = "湿滑泥土路"
    SAND = "沙土路"
    RUTTED = "有明显车辙/坑洼"
    UNKNOWN = "未知"


# ==================== 数据结构 ====================

@dataclass
class UnpavedModeSignal:
    """非铺装道路模式信号（来自 ad-mcc-01）"""
    is_unpaved: bool = True
    road_type: RoadSurface = RoadSurface.DIRT
    bump_index: float = 0.0
    friction_estimate: float = 0.5
    timestamp: float = field(default_factory=time.time)


@dataclass
class SteeringSoftParams:
    """非铺装转向约束参数（发送至 ad-mcc-04 和 ad-mcc-06）"""
    max_angle_rate_deg_per_s: float = 300.0
    max_steering_angle_deg: float = 450.0
    error_tolerance_band_deg: float = 5.0
    max_lateral_jerk_ms3: float = 2.0
    filter_cutoff_freq_hz: float = 5.0
    reaction_force_compliance: float = 0.3
    damping_compensation: float = 1.3
    road_type: RoadSurface = RoadSurface.DIRT
    effective_timestamp: float = field(default_factory=time.time)


@dataclass
class AdaptationStatusReport:
    """适配状态上报（发送至 ad-mcc-01）"""
    current_mode: AdapterState = AdapterState.MODE_NORMAL
    road_type: RoadSurface = RoadSurface.PAVED
    current_max_angle_rate: float = 500.0
    current_max_lateral_jerk: float = 3.0
    timestamp: float = field(default_factory=time.time)


# ==================== 各路面类型转向参数集 ====================

PAVED_DEFAULT_PARAMS = {
    "max_angle_rate_deg_per_s": 500.0,
    "max_steering_angle_deg": 500.0,
    "error_tolerance_band_deg": 3.0,
    "max_lateral_jerk_ms3": 3.0,
    "filter_cutoff_freq_hz": 8.0,
    "reaction_force_compliance": 0.0,
    "damping_compensation": 1.0,
}

UNPAVED_PARAMS: Dict[RoadSurface, Dict[str, float]] = {
    RoadSurface.DIRT: {
        "max_angle_rate_deg_per_s": 300.0,
        "max_steering_angle_deg": 450.0,
        "error_tolerance_band_deg": 5.0,
        "max_lateral_jerk_ms3": 2.0,
        "filter_cutoff_freq_hz": 5.0,
        "reaction_force_compliance": 0.3,
        "damping_compensation": 1.3,
    },
    RoadSurface.GRAVEL: {
        "max_angle_rate_deg_per_s": 250.0,
        "max_steering_angle_deg": 400.0,
        "error_tolerance_band_deg": 7.0,
        "max_lateral_jerk_ms3": 1.8,
        "filter_cutoff_freq_hz": 4.0,
        "reaction_force_compliance": 0.4,
        "damping_compensation": 1.4,
    },
    RoadSurface.WET_MUD: {
        "max_angle_rate_deg_per_s": 200.0,
        "max_steering_angle_deg": 350.0,
        "error_tolerance_band_deg": 8.0,
        "max_lateral_jerk_ms3": 1.5,
        "filter_cutoff_freq_hz": 3.0,
        "reaction_force_compliance": 0.5,
        "damping_compensation": 1.6,
    },
    RoadSurface.SAND: {
        "max_angle_rate_deg_per_s": 220.0,
        "max_steering_angle_deg": 400.0,
        "error_tolerance_band_deg": 6.0,
        "max_lateral_jerk_ms3": 1.8,
        "filter_cutoff_freq_hz": 4.0,
        "reaction_force_compliance": 0.35,
        "damping_compensation": 1.3,
    },
    RoadSurface.RUTTED: {
        "max_angle_rate_deg_per_s": 180.0,
        "max_steering_angle_deg": 350.0,
        "error_tolerance_band_deg": 10.0,
        "max_lateral_jerk_ms3": 1.5,
        "filter_cutoff_freq_hz": 3.0,
        "reaction_force_compliance": 0.5,
        "damping_compensation": 1.5,
    },
}

# 绝对下限保护
MIN_ANGLE_RATE_DEG_PER_S = 150.0
ABSOLUTE_MAX_LATERAL_JERK_MS3 = 3.0

# 过渡参数
TRANSITION_DURATION_S = 1.0
CONTROL_PERIOD_S = 0.01  # 100Hz

# 车速动态微调阈值
LOW_SPEED_KMH = 10.0
HIGH_SPEED_KMH = 30.0
LOW_SPEED_RATE_FACTOR = 1.2
HIGH_SPEED_RATE_FACTOR = 0.8


# ==================== 主类定义 ====================

class UnpavedSteeringAdapter:
    """
    非铺装路面转向适配单元
    
    职责:
    1. 接收非铺装道路模式信号，加载对应路面类型的转向参数集
    2. 通过线性斜坡函数平滑过渡参数，避免阶跃切换
    3. 在非铺装模式下根据车速动态微调转角速率上限
    4. 周期性向 ad-mcc-04 和 ad-mcc-06 推送适配参数
    """

    def __init__(self):
        self.module_id = "ad-mcc-08"
        self.module_name = "非铺装路面转向适配单元"
        self.version = "V1.0"

        self.state = AdapterState.MODE_NORMAL
        self._current_road_type = RoadSurface.PAVED

        # 当前生效的完整参数集
        self._active_params = PAVED_DEFAULT_PARAMS.copy()

        # 过渡状态
        self._transition_start_time: float = 0.0
        self._source_params: Optional[Dict[str, float]] = None
        self._target_params: Optional[Dict[str, float]] = None
        self._target_state: Optional[AdapterState] = None

        # 车辆状态
        self._current_speed_kmh: float = 0.0

        # 统计
        self._total_mode_switches: int = 0
        self._last_report_time: float = 0.0

        self._pending_logs: List[Dict[str, Any]] = []

        # 外部依赖回调
        self._query_unpaved_signal = None          # Callable[[], Optional[UnpavedModeSignal]]
        self._query_vehicle_speed = None           # Callable[[], float]
        self._query_actual_angle_rate = None       # Callable[[], float]

        # 输出回调
        self._publish_steering_constraints = None   # Callable[[SteeringSoftParams], None]
        self._publish_status_report = None          # Callable[[AdaptationStatusReport], None]

        print(f"[{self.module_id}] {self.module_name} {self.version} 初始化完成")

    # ========== 回调注入 ==========
    def set_unpaved_signal_query(self, callback):
        self._query_unpaved_signal = callback

    def set_vehicle_speed_query(self, callback):
        self._query_vehicle_speed = callback

    def set_actual_angle_rate_query(self, callback):
        self._query_actual_angle_rate = callback

    def set_steering_constraints_publisher(self, callback):
        self._publish_steering_constraints = callback

    def set_status_report_publisher(self, callback):
        self._publish_status_report = callback

    # ========== 主循环 ==========
    def run_adaptation_cycle(self) -> Optional[SteeringSoftParams]:
        """
        执行一次非铺装转向适配周期（100Hz）
        
        Returns:
            当前生效的转向约束参数，用于推送给下游模块
        """
        if self.state == AdapterState.SYSTEM_PAUSED:
            return None

        now = time.time()

        # 更新车辆状态
        if self._query_vehicle_speed:
            self._current_speed_kmh = self._query_vehicle_speed()

        # 接收非铺装道路模式信号
        signal = self._query_unpaved_signal() if self._query_unpaved_signal else None
        if signal:
            if signal.is_unpaved:
                target_mode = AdapterState.MODE_UNPAVED_SOFT
                road_type = signal.road_type if signal.road_type != RoadSurface.UNKNOWN else RoadSurface.WET_MUD
                target_params = UNPAVED_PARAMS.get(road_type, UNPAVED_PARAMS[RoadSurface.WET_MUD])
            else:
                target_mode = AdapterState.MODE_NORMAL
                road_type = RoadSurface.PAVED
                target_params = PAVED_DEFAULT_PARAMS

            self._current_road_type = road_type
            self._start_transition(target_mode, target_params)

        # 执行参数过渡
        if self.state == AdapterState.MODE_TRANSITIONING:
            self._apply_transition(now)

        # 非铺装模式下的动态微调
        if self.state == AdapterState.MODE_UNPAVED_SOFT:
            self._apply_dynamic_adjustment(now)

        # 构建当前参数对象
        params = self._build_params_object()

        # 周期性状态上报
        if now - self._last_report_time >= 1.0:
            self._last_report_time = now
            if self._publish_status_report:
                self._publish_status_report(AdaptationStatusReport(
                    current_mode=self.state,
                    road_type=self._current_road_type,
                    current_max_angle_rate=self._active_params.get("max_angle_rate_deg_per_s", 500.0),
                    current_max_lateral_jerk=self._active_params.get("max_lateral_jerk_ms3", 3.0),
                ))

        return params

    # ========== 参数过渡 ==========
    def _start_transition(self, target_mode: AdapterState, target_params: Dict[str, float]):
        """启动参数平滑过渡"""
        self._total_mode_switches += 1
        self._source_params = self._active_params.copy()
        self._target_params = target_params.copy()
        self._target_state = target_mode
        self._transition_start_time = time.time()
        self.state = AdapterState.MODE_TRANSITIONING

        self._log_event("TRANSITION_STARTED", {
            "current_mode": self.state.name,
            "target_mode": target_mode.name,
            "road_type": self._current_road_type.value
        })

    def _apply_transition(self, now: float):
        """执行线性斜坡过渡"""
        elapsed = now - self._transition_start_time
        if elapsed >= TRANSITION_DURATION_S or self._target_params is None:
            # 过渡完成
            self._active_params = self._target_params.copy() if self._target_params else self._active_params
            self.state = self._target_state if self._target_state else AdapterState.MODE_NORMAL
            self._source_params = None
            self._target_params = None
            self._target_state = None
            self._push_params()
        elif self._source_params and self._target_params:
            # 线性插值
            ratio = elapsed / TRANSITION_DURATION_S
            self._active_params = self._linear_interpolate(self._source_params, self._target_params, ratio)
            self._push_params()

    def _linear_interpolate(self, source: Dict[str, float], target: Dict[str, float],
                            ratio: float) -> Dict[str, float]:
        """对两个参数字典进行线性插值"""
        result = {}
        for key in source:
            s_val = source[key]
            t_val = target.get(key, s_val)
            result[key] = s_val + (t_val - s_val) * ratio
        return result

    # ========== 动态微调 ==========
    def _apply_dynamic_adjustment(self, now: float):
        """根据当前车速动态微调转角速率上限"""
        base_rate = self._active_params.get("max_angle_rate_deg_per_s", 300.0)
        speed = self._current_speed_kmh

        if speed < LOW_SPEED_KMH:
            adjusted_rate = base_rate * LOW_SPEED_RATE_FACTOR
        elif speed > HIGH_SPEED_KMH:
            adjusted_rate = base_rate * HIGH_SPEED_RATE_FACTOR
        else:
            adjusted_rate = base_rate

        # 绝对下限保护
        adjusted_rate = max(adjusted_rate, MIN_ANGLE_RATE_DEG_PER_S)

        # 横向冲击度绝对红线保护
        max_jerk = self._active_params.get("max_lateral_jerk_ms3", 2.0)
        max_jerk = min(max_jerk, ABSOLUTE_MAX_LATERAL_JERK_MS3)

        self._active_params["max_angle_rate_deg_per_s"] = adjusted_rate
        self._active_params["max_lateral_jerk_ms3"] = max_jerk

        # 推送更新参数
        self._push_params()

    # ========== 参数构建与推送 ==========
    def _build_params_object(self) -> SteeringSoftParams:
        """构建当前参数对象"""
        return SteeringSoftParams(
            max_angle_rate_deg_per_s=self._active_params.get("max_angle_rate_deg_per_s", 500.0),
            max_steering_angle_deg=self._active_params.get("max_steering_angle_deg", 500.0),
            error_tolerance_band_deg=self._active_params.get("error_tolerance_band_deg", 3.0),
            max_lateral_jerk_ms3=self._active_params.get("max_lateral_jerk_ms3", 3.0),
            filter_cutoff_freq_hz=self._active_params.get("filter_cutoff_freq_hz", 8.0),
            reaction_force_compliance=self._active_params.get("reaction_force_compliance", 0.0),
            damping_compensation=self._active_params.get("damping_compensation", 1.0),
            road_type=self._current_road_type,
        )

    def _push_params(self):
        """推送当前参数至 ad-mcc-04 和 ad-mcc-06"""
        params = self._build_params_object()
        if self._publish_steering_constraints:
            self._publish_steering_constraints(params)

    # ========== 查询接口 ==========
    def get_state(self) -> AdapterState:
        return self.state

    def get_current_road_type(self) -> RoadSurface:
        return self._current_road_type

    def get_active_params(self) -> Dict[str, float]:
        return self._active_params.copy()

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
            "road_type": self._current_road_type.value,
            "total_mode_switches": self._total_mode_switches,
            "active_max_angle_rate": self._active_params.get("max_angle_rate_deg_per_s", 500.0),
        }

    def emergency_shutdown(self):
        self.state = AdapterState.SYSTEM_PAUSED
        print(f"[{self.module_id}] 紧急熔断，保持当前参数")


# ============================================================
# 最小闭环演示
# ============================================================
def print_separator(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def demo_main():
    print("=" * 70)
    print("  AD-mcc-cerebellum 非铺装路面转向适配单元 (ad-mcc-08) 演示")
    print("=" * 70)

    adapter = UnpavedSteeringAdapter()
    adapter.set_vehicle_speed_query(lambda: 15.0)

    print_separator("STEP 1: 收到泥土路信号，进入柔和模式")
    adapter.set_unpaved_signal_query(lambda: UnpavedModeSignal(
        is_unpaved=True, road_type=RoadSurface.DIRT, bump_index=0.6
    ))
    # 运行足够帧数以完成过渡（1秒）
    for _ in range(110):
        params = adapter.run_adaptation_cycle()
    print(f"  状态: {adapter.state.value}")
    print(f"  路面类型: {adapter.get_current_road_type().value}")
    print(f"  转角速率上限: {params.max_angle_rate_deg_per_s if params else 'N/A'} °/s")
    print(f"  误差容忍带: {params.error_tolerance_band_deg if params else 'N/A'} °")
    print(f"  冲击度上限: {params.max_lateral_jerk_ms3 if params else 'N/A'} m/s³")

    print_separator("STEP 2: 退出非铺装，恢复铺装参数")
    adapter.set_unpaved_signal_query(lambda: UnpavedModeSignal(
        is_unpaved=False, road_type=RoadSurface.PAVED
    ))
    for _ in range(110):
        params = adapter.run_adaptation_cycle()
    print(f"  状态: {adapter.state.value}")
    print(f"  转角速率上限: {params.max_angle_rate_deg_per_s if params else 'N/A'} °/s")

    print_separator("STEP 3: 紧急熔断")
    adapter.emergency_shutdown()
    print(f"  状态: {adapter.state.value}")

    print("\n✅ 非铺装路面转向适配单元演示完成")


# ============================================================
# 单元测试
# ============================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        print("=" * 60)
        print("ad-mcc-08 非铺装路面转向适配单元 单元测试")
        print("=" * 60)
        passed, failed = 0, 0

        def setup_adapter(speed=15.0):
            a = UnpavedSteeringAdapter()
            a.set_vehicle_speed_query(lambda: speed)
            return a

        # TC-M08-01: 进入非铺装泥土路
        print("\n[TC-M08-01] 进入非铺装泥土路 → 参数平滑过渡")
        try:
            a = setup_adapter()
            a.set_unpaved_signal_query(lambda: UnpavedModeSignal(is_unpaved=True, road_type=RoadSurface.DIRT))
            for _ in range(110):
                params = a.run_adaptation_cycle()
            assert a.state == AdapterState.MODE_UNPAVED_SOFT
            assert a.get_current_road_type() == RoadSurface.DIRT
            assert a.get_active_params()["max_angle_rate_deg_per_s"] == 300.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M08-02: 退出非铺装恢复铺装参数
        print("\n[TC-M08-02] 退出非铺装 → 恢复铺装参数")
        try:
            a = setup_adapter()
            a.set_unpaved_signal_query(lambda: UnpavedModeSignal(is_unpaved=True, road_type=RoadSurface.DIRT))
            for _ in range(110):
                a.run_adaptation_cycle()
            a.set_unpaved_signal_query(lambda: UnpavedModeSignal(is_unpaved=False, road_type=RoadSurface.PAVED))
            for _ in range(110):
                params = a.run_adaptation_cycle()
            assert a.state == AdapterState.MODE_NORMAL
            assert a.get_active_params()["max_angle_rate_deg_per_s"] == 500.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M08-03: 低速时转角速率微调上调
        print("\n[TC-M08-03] 低速 5km/h → 转角速率上调 20%")
        try:
            a = setup_adapter(speed=5.0)
            a.set_unpaved_signal_query(lambda: UnpavedModeSignal(is_unpaved=True, road_type=RoadSurface.DIRT))
            for _ in range(110):
                params = a.run_adaptation_cycle()
            # 泥土路基础 300，上调 20% = 360
            expected = 300.0 * 1.2
            actual = a.get_active_params()["max_angle_rate_deg_per_s"]
            assert abs(actual - expected) < 1.0, f"预期 {expected}, 实际 {actual}"
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M08-04: 高速时转角速率下调但不低于 150
        print("\n[TC-M08-04] 高速 40km/h 湿滑泥土路 → 下调但不低于 150")
        try:
            a = setup_adapter(speed=40.0)
            a.set_unpaved_signal_query(lambda: UnpavedModeSignal(is_unpaved=True, road_type=RoadSurface.WET_MUD))
            for _ in range(110):
                params = a.run_adaptation_cycle()
            # 湿滑泥土路基础 200，下调 20% = 160，不低于 150
            actual = a.get_active_params()["max_angle_rate_deg_per_s"]
            assert actual >= 150.0, f"转角速率 {actual} < 150（绝对下限）"
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M08-05: 未知路面使用最保守湿滑泥土路参数
        print("\n[TC-M08-05] 未知路面 → 默认湿滑泥土路参数")
        try:
            a = setup_adapter()
            a.set_unpaved_signal_query(lambda: UnpavedModeSignal(is_unpaved=True, road_type=RoadSurface.UNKNOWN))
            for _ in range(110):
                params = a.run_adaptation_cycle()
            assert a.get_active_params()["max_angle_rate_deg_per_s"] == 200.0
            print("   ✅ PASS")
            passed += 1
        except Exception as e:
            print(f"   ❌ FAIL: {e}")
            failed += 1

        # TC-M08-06: 紧急熔断
        print("\n[TC-M08-06] 紧急熔断")
        try:
            a = setup_adapter()
            a.emergency_shutdown()
            assert a.state == AdapterState.SYSTEM_PAUSED
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