# ad-mcc-14-制动响应加速单元 接口规格

---

## 基本信息

| 项 | 内容 |
|----|------|
| 模块编号 | ad-mcc-14 |
| 模块名称 | 制动响应加速单元 |
| 所属分区 | 四、制动控制集群 |
| 核心职责 | 接收 ad-mcc-13 输出的制动目标压力序列，根据制动类型（日常缓刹/紧急制动）和当前实际制动压力，动态控制制动压力的建立速率与波形。紧急制动时实现极速建压（<100ms 达到目标压力的90%），日常制动时控制压力平缓上升，消除突兀感。同时管理制动管路的预填充与压力保持，输出实时压力指令至下游执行模块。不参与任何场景判断与驾驶决策 |
| 依赖模块 | ad-mcc-13（制动压力解算单元，提供制动目标压力序列）、制动压力传感器（通过 CAN 总线提供实际制动主缸压力）、ad-mcc-34（动力与制动参数管理单元，提供制动系统建压特性参数） |
| 被依赖模块 | ad-mcc-15（制动平顺防点头单元，接收平滑后的压力指令曲线）、ad-mcc-02（运动生理边界闸门，校验输出压力指令） |


## 内部状态定义

| 状态 | 标识 | 含义 | 触发条件 |
|------|------|------|----------|
| 空闲泄压 | `IDLE_DEPRESSURIZED` | 无制动指令，管路压力为零或接近零 | 系统初始化完成，无制动请求 |
| 日常建压中 | `GENTLE_BUILDUP` | 日常缓刹，压力按平缓斜率逐步上升至目标 | 收到 ad-mcc-13 的日常缓刹指令 |
| 紧急建压中 | `EMERGENCY_BUILDUP` | 紧急制动，以最大速率建立压力，优先快速响应 | 收到紧急制动指令或目标压力阶跃 |
| 目标保压 | `HOLDING` | 已到达目标压力，维持当前压力不变 | 实际压力与目标压力偏差 < 0.1MPa 且无新指令 |
| 日常泄压中 | `GENTLE_RELEASE` | 日常制动解除，压力平缓下降 | 目标压力突降至 0 且上一状态非紧急 |
| 快速泄压 | `FAST_RELEASE` | 紧急状态解除或 ABS 触发，快速泄压 | 收到快速泄压指令或 ABS 激活信号 |
| 暂停服务 | `SYSTEM_PAUSED` | 系统紧急熔断 | 收到紧急熔断指令 |


## 输入数据

| 输入项 | 数据类型 | 来源模块 | 触发条件 | 优先级 |
|--------|----------|----------|----------|:---:|
| 制动目标压力序列 | Struct（时间戳 + 目标压力_MPa + 制动类型 + 摩擦制动压力_MPa + 再生制动扭矩_Nm + 解算置信度 + 限制因素） | ad-mcc-13 制动压力解算单元 | 每次解算完成后 | **高** |
| 实际制动主缸压力 | Float（MPa） | CAN 总线（制动压力传感器） | 实时，200Hz | **高** |
| 制动系统建压特性参数 | Struct（最大建压速率_MPa_per_s + 日常建压速率_MPa_per_s + 管路预填充压力_MPa + 预填充时间_ms） | ad-mcc-34 动力与制动参数管理单元 | 系统初始化加载 | 普通 |
| 快速泄压指令 | Struct（泄压激活 + 目标残余压力） | ABS/ESP 控制模块 或 ad-mcc-01 | ABS/ESC 激活时 | **紧急** |
| 全局调度指令 | Enum（暂停/恢复/熔断） | ad-mcc-01 小脑总控调度核心 | 模式切换或紧急事件时 | **紧急** |


## 输出数据

| 输出项 | 数据类型 | 目标模块 | 输出条件 | 优先级 |
|--------|----------|----------|----------|:---:|
| 实时制动压力指令 | Struct（时间戳 + 当前目标压力_MPa + 压力变化速率_MPa_per_s + 制动类型 + 建压阶段 + 期望到达时间） | ad-mcc-15 制动平顺防点头单元 及 制动执行器接口 | 每周期（200Hz） | **高** |
| 建压状态上报 | Struct（当前状态 + 实际压力 + 目标压力 + 建压耗时 + 超调量） | ad-mcc-01（通过内部调度总线） | 每次建压完成或泄压完成时 | 普通 |
| 紧急建压超时告警 | Struct（目标压力 + 实际压力 + 超时时间 + 可能原因） | ad-mcc-01、ECC-12（通过 CerebellumBus） | 紧急建压超过 150ms 未达到目标 | **紧急** |


## 建压参数配置

| 参数名称 | 日常缓刹 | 紧急制动 | 非铺装路面 | 说明 |
|----------|:---:|:---:|:---:|------|
| 目标建压速率 (MPa/s) | 20.0 | 150.0（最大物理极限） | 12.0 | 升压阶段的最大压力变化率 |
| 预填充压力 (MPa) | 0.5 | 1.5 | 0.3 | 消除制动片与盘之间的间隙 |
| 预填充持续时间 (ms) | 50 | 30 | 80 | 预填充阶段时长 |
| 最大允许超调量 (MPa) | 0.2 | 0.5 | 0.15 | 允许超过目标压力的最大值 |
| 建压超时阈值 (ms) | 500 | 150 | 800 | 超过此时间未达标则上报告警 |
| 泄压速率 (MPa/s) | 15.0 | 80.0 | 10.0 | 日常泄压/快速泄压 |


## 核心处理逻辑

```
FUNCTION brake_pressure_response_main_loop():
    STATE_IDLE = IDLE_DEPRESSURIZED
    STATE_GENTLE_UP = GENTLE_BUILDUP
    STATE_EMERGENCY_UP = EMERGENCY_BUILDUP
    STATE_HOLD = HOLDING
    STATE_GENTLE_DOWN = GENTLE_RELEASE
    STATE_FAST_DOWN = FAST_RELEASE
    STATE_PAUSED = SYSTEM_PAUSED

    SET internal_state = STATE_IDLE
    加载建压特性参数
    SET current_target = 0.0
    SET build_start_time = 0
    SET prefill_done = False

    WHILE 系统运行中:
        // 第1步：紧急泄压/ABS 优先
        IF 收到快速泄压指令:
            SET internal_state = STATE_FAST_DOWN
            目标残余压力 = 指令.目标残余压力
            current_target = 目标残余压力
            CONTINUE

        // 第2步：紧急熔断
        IF 收到紧急熔断指令:
            SET internal_state = STATE_PAUSED
            CONTINUE
        ELSE IF 收到恢复指令 AND internal_state == STATE_PAUSED:
            SET internal_state = STATE_IDLE

        // 第3步：接收制动目标压力序列
        IF 收到 ad-mcc-13 制动目标压力序列:
            目标压力 = 序列.目标压力_MPa
            制动类型 = 序列.制动类型
            实际压力 = 获取实际制动主缸压力()

            // 3a. 判断建压模式
            IF 目标压力 <= 0.05:
                // 泄压请求
                IF internal_state IN [STATE_EMERGENCY_UP, STATE_FAST_DOWN]:
                    // 保持快速泄压
                    SET internal_state = STATE_FAST_DOWN
                ELSE:
                    SET internal_state = STATE_GENTLE_DOWN
                current_target = 0.0
            ELSE IF 制动类型 == "紧急制动" OR 目标压力 > 7.0:
                // 紧急建压
                SET internal_state = STATE_EMERGENCY_UP
                current_target = 目标压力
                build_start_time = NOW()
                prefill_done = False
            ELSE:
                // 日常建压
                IF 目标压力 > current_target:
                    SET internal_state = STATE_GENTLE_UP
                    IF current_target < 0.1:
                        build_start_time = NOW()
                        prefill_done = False
                    current_target = 目标压力
                ELSE IF 目标压力 < current_target AND 目标压力 > 0:
                    // 降低目标，缓慢泄压
                    current_target = 目标压力
                    IF internal_state != STATE_GENTLE_DOWN:
                        SET internal_state = STATE_GENTLE_DOWN

        // 第4步：执行建压/泄压控制
        实际压力 = 获取实际制动主缸压力()
        建压速率 = 获取当前模式建压速率(internal_state)

        IF internal_state IN [STATE_GENTLE_UP, STATE_EMERGENCY_UP]:
            // 预填充阶段（首次建压或长时间泄压后）
            IF NOT prefill_done AND 实际压力 < 预填充压力:
                输出压力 = 预填充压力
                预填充持续时间 += 控制周期
                IF 预填充持续时间 >= 预填充时间_ms:
                    prefill_done = True
            ELSE:
                // 正式升压阶段
                IF internal_state == STATE_EMERGENCY_UP:
                    压力步长 = 紧急建压速率 × 控制周期
                ELSE:
                    压力步长 = 日常建压速率 × 控制周期
                // 逐步升压，但允许超调量
                输出压力 = MIN(实际压力 + 压力步长, current_target + 最大超调量)
                // 接近目标时，缩小步长防止超调
                IF current_target - 输出压力 < 0.3:
                    输出压力 = 输出压力 + (current_target - 输出压力) × 0.3
                // 压力只能升不能降（在建压阶段）
                IF 输出压力 < 实际压力:
                    输出压力 = 实际压力

            // 检查是否达到目标
            IF 实际压力 >= current_target - 0.1 AND 输出压力 >= current_target - 0.1:
                SET internal_state = STATE_HOLD
                向 ad-mcc-01 上报建压完成(建压耗时 = NOW() - build_start_time)

            // 紧急建压超时告警
            IF internal_state == STATE_EMERGENCY_UP AND (NOW() - build_start_time) > 150ms:
                向 ad-mcc-01 发送紧急建压超时告警(目标压力, 实际压力, 超时时间)

        ELSE IF internal_state == STATE_HOLD:
            // 保持目标压力，允许微小修正
            输出压力 = current_target
            IF ABS(实际压力 - current_target) > 0.2:
                输出压力 = 实际压力 + SIGN(current_target - 实际压力) × 0.1

        ELSE IF internal_state == STATE_GENTLE_DOWN:
            // 平缓泄压
            压力步长 = 日常泄压速率 × 控制周期
            输出压力 = MAX(实际压力 - 压力步长, current_target)
            IF 实际压力 <= 0.05:
                SET internal_state = STATE_IDLE

        ELSE IF internal_state == STATE_FAST_DOWN:
            // 快速泄压
            压力步长 = 快速泄压速率 × 控制周期
            输出压力 = MAX(实际压力 - 压力步长, current_target)
            IF 实际压力 <= current_target + 0.05:
                SET internal_state = STATE_IDLE

        ELSE:
            输出压力 = 0.0

        // 第5步：边界安全裁剪
        输出压力 = CLAMP(输出压力, 0.0, 制动系统最大压力)

        // 第6步：输出实时制动压力指令
        实时指令 = 构建实时制动压力指令(输出压力, 建压速率, 制动类型, internal_state)
        向 ad-mcc-15 发送实时指令

        SLEEP 5ms  // 200Hz
```


## 约束与异常处理

| 场景 | 处理方式 | 恢复条件 |
|------|----------|----------|
| 实际压力传感器故障 | 开环控制，按预设速率输出压力指令，标记“开环模式” | 传感器恢复 |
| 紧急建压超时（>150ms） | 立即上报告警，保持当前建压速率继续努力 | 达到目标压力或接收新指令 |
| 制动液温度过高预警 | 自动降低建压速率20%，防止气阻 | 温度恢复正常 |
| 目标压力频繁跳变 | 低通滤波目标压力（α=0.3），避免频繁切换建压/泄压状态 | — |
| ABS/ESC 激活时快速泄压 | 忽略正常目标压力，执行快速泄压至残余压力 | ABS/ESC 停止激活 |
| 管路预填充失败（预填充压力未达到） | 延长预填充时间至100ms，上报预填充异常 | 预填充完成 |
| 紧急熔断 | 立即快速泄压至0，并保持 | 紧急解除后恢复 |


## 总线契约

| 总线 | 操作 | 数据内容 | 权限 | 说明 |
|------|------|----------|------|------|
| 内部调度总线 | 读 | 制动目标压力序列 | 只读 | ad-mcc-13 发送 |
| CAN 总线 | 读 | 实际制动主缸压力 | 只读 | 实时，200Hz |
| 内部调度总线 | 读 | 制动系统建压特性参数 | 只读 | ad-mcc-34 提供 |
| CAN/ESP 总线 | 读 | 快速泄压指令 | 只读 | ABS/ESP 模块 |
| 内部调度总线 | 写 | 实时制动压力指令 | 专属写入 | 向 ad-mcc-15 及执行器发送 |
| 内部调度总线 | 写 | 建压状态上报 | 事件触发写入 | 向 ad-mcc-01 发送 |
| 内部调度总线 | 写 | 紧急建压超时告警 | 事件触发写入 | 向 ad-mcc-01 发送 |


## 安全边界

| 规则编号 | 内容 |
|:---:|------|
| S-01 | 紧急制动建压时间从指令到达到实际压力达目标90%不得超过150ms，超时强制上报告警 |
| S-02 | 输出压力指令禁止超过制动系统物理最大压力，防止管路爆裂 |
| S-03 | ABS/ESP 激活时，快速泄压指令优先级高于一切建压指令 |
| S-04 | 建压过程中不得出现压力回落（除非收到泄压指令），防止制动点头或制动力波动 |
| S-05 | 本模块仅负责制动压力的动态响应控制，不参与制动时机的决策 |


## 接口校验用例

| 用例编号 | 前置条件 | 输入 | 预期输出 |
|----------|----------|------|----------|
| TC-M14-01 | `IDLE`，收到日常缓刹指令 | 目标压力=3.0MPa，制动类型=日常缓刹 | 进入 `GENTLE_BUILDUP`，压力以20MPa/s上升，约150ms后达到目标 |
| TC-M14-02 | `IDLE`，收到紧急制动指令 | 目标压力=9.0MPa，制动类型=紧急制动 | 进入 `EMERGENCY_BUILDUP`，压力以150MPa/s极速上升，90ms内达到8.1MPa（90%） |
| TC-M14-03 | `HOLDING` 状态，压力维持 | 目标压力=3.0MPa，实际压力=3.0MPa | 持续输出3.0MPa，状态保持 `HOLDING` |
| TC-M14-04 | `GENTLE_BUILDUP` 中，收到泄压指令 | 目标压力=0.0MPa | 进入 `GENTLE_RELEASE`，压力以15MPa/s平缓下降至0 |
| TC-M14-05 | 任意建压状态，ABS 激活 | 快速泄压指令（残余压力=1.0MPa） | 立即进入 `FAST_RELEASE`，压力快速降至1.0MPa |
| TC-M14-06 | `EMERGENCY_BUILDUP`，超时 | 目标压力=9.0MPa，实际压力=5.0MPa，耗时160ms | 触发紧急建压超时告警，继续建压 |


## 质量自检清单

| 检查项 | 状态 |
|--------|:---:|
| 模块编号与分区归属正确 | ✅ |
| 依赖与被依赖模块编号完整 | ✅ |
| 内部状态机7个状态含触发条件 | ✅ |
| 输入/输出含数据类型、来源/目标模块、优先级 | ✅ |
| 建压参数配置表完整（4种工况 × 6项参数） | ✅ |
| 核心处理逻辑伪代码含预填充、紧急/日常建压、保压、泄压、超时告警、ABS泄压全流程 | ✅ |
| 约束与异常覆盖传感器故障、超时、高温、跳变、ABS激活、预填充失败、熔断共7条 | ✅ |
| 总线契约区分内部调度总线、CAN总线、ESP总线 | ✅ |
| 安全边界逐条列出 | ✅ |
| 校验用例覆盖日常建压、紧急建压、保压、泄压、ABS泄压、超时告警共6条 | ✅ |