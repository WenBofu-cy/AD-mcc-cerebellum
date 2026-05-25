# ad-mcc-26-档位切换管控单元 接口规格

---

## 基本信息

| 项 | 内容 |
|----|------|
| 模块编号 | ad-mcc-26 |
| 模块名称 | 档位切换管控单元 |
| 所属分区 | 七、档位与驻车管理 |
| 核心职责 | 接收 ECC 大脑通过 ad-mcc-01 下发的档位切换意图（P/R/N/D），结合当前车速、制动踏板状态及车辆运动状态，执行安全、平顺的档位逻辑切换。确保 P 档仅能在车辆完全静止且满足驻车条件时切入，R 档与 D 档之间的切换需在车速低于安全阈值（如 5 km/h）且制动踏板踩下时执行，防止高速误操作损坏变速器。输出标准化档位指令至变速箱控制器（TCU），并回传切换完成回执至 ad-mcc-01。不参与任何驾驶决策，仅执行档位切换的安全管控与执行 |
| 依赖模块 | ad-mcc-01（小脑总控调度核心，下发档位意图）、当前车速（CAN 总线）、制动踏板状态（CAN 总线/制动开关）、ad-mcc-27（电子驻车制动控制单元，协调 P 档与 EPB 的联动） |
| 被依赖模块 | 变速箱控制器（TCU，通过 CAN 总线执行档位指令）、ad-mcc-01（接收切换完成回执）、ad-mcc-38（执行日志记录单元，记录异常切换事件） |


## 内部状态定义

| 状态 | 标识 | 含义 | 触发条件 |
|------|------|------|----------|
| 档位稳定 | `GEAR_STABLE` | 当前档位稳定，无切换请求 | 系统初始化完成，无待处理档位请求 |
| 切换进行中 | `SHIFT_IN_PROGRESS` | 档位正在切换，等待 TCU 反馈 | 发送档位切换指令后，尚未收到 TCU 确认 |
| 切换失败 | `SHIFT_FAILED` | TCU 报告切换失败或超时 | 收到 TCU 错误码或切换超时 |
| 禁止切换 | `SHIFT_INHIBITED` | 当前条件不满足安全切换要求，拒绝请求 | 收到档位意图但车速/制动不满足条件 |
| 暂停服务 | `SYSTEM_PAUSED` | 系统紧急熔断 | 收到紧急熔断指令 |


## 输入数据

| 输入项 | 数据类型 | 来源模块 | 触发条件 | 优先级 |
|--------|----------|----------|----------|:---:|
| 档位切换意图 | Struct（目标档位：P/R/N/D + 请求来源 + 优先级） | ad-mcc-01 小脑总控调度核心（来自 ECC 或驾驶员操作） | ECC 下发或驾驶员操作档位杆时 | **高** |
| 当前车速 | Float（km/h） | CAN 总线（轮速传感器） | 实时，100Hz | **高** |
| 制动踏板状态 | Struct（制动开关 + 制动压力_MPa） | CAN 总线（制动踏板开关/制动压力传感器） | 实时，100Hz | **高** |
| 当前实际档位 | Enum（P/R/N/D） | CAN 总线（TCU 反馈） | 实时或事件触发 | **高** |
| TCU 状态与故障码 | Struct（就绪 + 故障码 + 温度保护） | CAN 总线（TCU） | 周期性，10Hz 或事件触发 | 普通 |
| EPB 状态 | Struct（夹紧/释放 + 故障） | ad-mcc-27 电子驻车制动控制单元 | 周期性或状态变更 | 普通 |
| 全局调度指令 | Enum（暂停/恢复/熔断） | ad-mcc-01 小脑总控调度核心 | 模式切换或紧急事件时 | **紧急** |


## 输出数据

| 输出项 | 数据类型 | 目标模块 | 输出条件 | 优先级 |
|--------|----------|----------|----------|:---:|
| 档位切换指令 | Struct（目标档位 + 切换模式 + 超时时间） | TCU（通过 CAN 总线） | 安全条件满足后发送 | **高** |
| 切换完成回执 | Struct（目标档位 + 实际档位 + 切换结果 + 耗时） | ad-mcc-01（通过内部调度总线） | 收到 TCU 确认或超时/失败后 | **高** |
| EPB 联动请求（P 档时） | Struct（请求 EPB 夹紧） | ad-mcc-27 电子驻车制动控制单元 | 成功切入 P 档后 | **高** |
| 档位状态上报 | Struct（当前档位 + 状态 + 故障码） | ad-mcc-01 | 状态变更或周期性 1Hz | 普通 |
| 档位切换事件记录 | Struct（请求档位 + 结果 + 时间戳） | ad-mcc-38 执行日志记录单元 | 每次切换完成/失败时 | 普通 |


## 档位切换安全策略

| 目标档位 | 允许切换条件 | 附加要求 |
|----------|-------------|----------|
| P | 车速 = 0 km/h，持续 > 500ms | 制动踏板踩下或 EPB 已夹紧 |
| R | 车速 < 5 km/h，且制动踏板踩下 | 从 D→R 或 N→R 均需制动 |
| N | 任意车速 | 无附加条件（但高速切入 N 需谨慎） |
| D | 车速 < 5 km/h，且制动踏板踩下（从 P/R 切出时） | 从 N→D 在低速下可直接切换，高速下需制动 |

**禁止行为**：
- 车辆未完全静止时（车速 > 0.5 km/h）严禁切入 P 档。
- 车速 > 8 km/h 时禁止在 R 与 D 之间直接切换。
- 档位切换过程中（SHIFT_IN_PROGRESS）禁止再次下发新指令，新请求需排队等待。


## 核心处理逻辑

```
FUNCTION gear_shift_control_main_loop():
    STATE_STABLE = GEAR_STABLE
    STATE_PROGRESS = SHIFT_IN_PROGRESS
    STATE_FAILED = SHIFT_FAILED
    STATE_INHIBIT = SHIFT_INHIBITED
    STATE_PAUSED = SYSTEM_PAUSED

    SET internal_state = STATE_STABLE
    SET current_gear = "P"
    SET pending_gear_request = None
    SET shift_timer = 0.0
    SET shift_timeout = 2.0  // 秒

    WHILE 系统运行中:
        // 第1步：紧急熔断
        IF 收到紧急熔断指令:
            SET internal_state = STATE_PAUSED
            CONTINUE
        ELSE IF 收到恢复指令 AND internal_state == STATE_PAUSED:
            SET internal_state = STATE_STABLE

        // 第2步：获取当前车辆状态
        车速 = 获取当前车速()
        制动 = 获取制动踏板状态()
        TCU反馈 = 获取TCU当前档位与状态()
        EPB状态 = 获取EPB状态()

        // 更新当前实际档位
        IF TCU反馈.有效:
            current_gear = TCU反馈.当前档位

        // 第3步：处理切换进行中
        IF internal_state == STATE_PROGRESS:
            IF TCU反馈.故障码 != 0:
                SET internal_state = STATE_FAILED
                向 ad-mcc-01 发送切换完成回执(失败, TCU故障)
                记录故障日志
                CONTINUE

            IF TCU反馈.当前档位 == pending_gear_request:
                // 切换成功
                SET internal_state = STATE_STABLE
                向 ad-mcc-01 发送切换完成回执(成功)
                记录成功日志
                IF pending_gear_request == "P":
                    向 ad-mcc-27 发送 EPB 联动请求(夹紧)
                pending_gear_request = None
                CONTINUE

            IF NOW() - shift_timer > shift_timeout:
                SET internal_state = STATE_FAILED
                向 ad-mcc-01 发送切换完成回执(超时)
                记录超时日志
                pending_gear_request = None
                CONTINUE

            // 否则继续等待
            CONTINUE

        // 第4步：接收新档位意图
        IF internal_state == STATE_STABLE OR internal_state == STATE_FAILED OR internal_state == STATE_INHIBIT:
            意图 = 获取 ad-mcc-01 档位切换意图
            IF 意图 == None:
                CONTINUE

            目标档位 = 意图.目标档位

            // 第5步：安全条件校验
            条件满足 = False
            拒绝原因 = ""

            CASE 目标档位 OF:
                "P":
                    IF 车速 == 0 AND 车速持续 > 500ms:
                        IF 制动.制动开关 OR EPB状态.夹紧:
                            条件满足 = True
                        ELSE:
                            拒绝原因 = "未踩制动且EPB未夹紧"
                    ELSE:
                        拒绝原因 = "车辆未完全静止"
                "R":
                    IF 车速 < 5.0 AND 制动.制动开关:
                        条件满足 = True
                    ELSE:
                        拒绝原因 = "车速过高或未踩制动"
                "N":
                    条件满足 = True  // 无附加条件
                "D":
                    IF 车速 < 5.0:
                        IF current_gear IN ["P", "R"] AND NOT 制动.制动开关:
                            拒绝原因 = "从P/R切换需踩制动"
                        ELSE:
                            条件满足 = True
                    ELSE:
                        拒绝原因 = "车速过高"

            IF NOT 条件满足:
                SET internal_state = STATE_INHIBIT
                向 ad-mcc-01 发送切换完成回执(拒绝, 拒绝原因)
                CONTINUE

            // 第6步：发送切换指令
            向 TCU 发送档位切换指令(目标档位)
            SET internal_state = STATE_PROGRESS
            pending_gear_request = 目标档位
            shift_timer = NOW()

        SLEEP 20ms
```


## 约束与异常处理

| 场景 | 处理方式 | 恢复条件 |
|------|----------|----------|
| 切换超时（> 2 秒） | 标记切换失败，上报告警，回退请求 | 收到新指令或 TCU 恢复 |
| TCU 报告故障 | 标记切换失败，禁止后续切换，上报告警 | TCU 故障清除 |
| 车速信号丢失 | 假设车速=0（保守），仅允许切换至 P/N，禁止 R/D | 信号恢复 |
| 高速行驶中误触 P 档 | 拒绝切换，提示“车辆未静止” | — |
| 切换过程中收到新请求 | 排队等待当前切换完成，若新请求与当前目标相同则忽略 | 当前切换完成 |
| 紧急熔断 | 保持当前档位不变，暂停所有切换 | 紧急解除后恢复 |


## 总线契约

| 总线 | 操作 | 数据内容 | 权限 | 说明 |
|------|------|----------|------|------|
| 内部调度总线 | 读 | 档位切换意图 | 只读 | ad-mcc-01 下发 |
| CAN 总线 | 读 | 当前车速 | 只读 | 实时 |
| CAN 总线 | 读 | 制动踏板状态 | 只读 | 实时 |
| CAN 总线 | 读 | TCU 当前档位与状态 | 只读 | 实时/周期性 |
| 内部调度总线 | 读 | EPB 状态 | 只读 | ad-mcc-27 提供 |
| CAN 总线 | 写 | 档位切换指令 | 专属写入 | 向 TCU 发送 |
| 内部调度总线 | 写 | 切换完成回执 | 事件触发写入 | 向 ad-mcc-01 发送 |
| 内部调度总线 | 写 | EPB 联动请求 | 事件触发写入 | 向 ad-mcc-27 发送 |
| 内部调度总线 | 写 | 档位状态上报 | 周期性写入 | 向 ad-mcc-01 发送 |
| 内部调度总线 | 写 | 档位切换事件记录 | 事件触发写入 | 向 ad-mcc-38 发送 |


## 安全边界

| 规则编号 | 内容 |
|:---:|------|
| S-01 | 车辆未完全静止（车速>0）时严禁切入 P 档，防止变速器锁止机构损坏 |
| S-02 | 切换超时或 TCU 故障时，必须标记失败并禁止连续重试 |
| S-03 | 从 P/R 切换至 D 时必须检测制动踏板，防止车辆意外起步 |
| S-04 | 本模块仅执行档位切换的安全管控，不参与驾驶策略决策 |


## 接口校验用例

| 用例编号 | 前置条件 | 输入 | 预期输出 |
|----------|----------|------|----------|
| TC-M26-01 | `GEAR_STABLE`，车速=0，制动踩下 | 请求切换至 D | 发送 D 档指令，进入 `SHIFT_IN_PROGRESS` |
| TC-M26-02 | `GEAR_STABLE`，车速=0，制动未踩 | 请求切换至 D | 拒绝，回执“需踩制动” |
| TC-M26-03 | `GEAR_STABLE`，车速=10 km/h | 请求切换至 P | 拒绝，回执“车辆未静止” |
| TC-M26-04 | `SHIFT_IN_PROGRESS`，TCU 确认 | TCU 当前档位变为目标档位 | 切换成功，回执成功 |
| TC-M26-05 | `SHIFT_IN_PROGRESS`，超时 2.5 秒 | TCU 未响应 | 切换失败，回执超时 |
| TC-M26-06 | `GEAR_STABLE`，P 档切换成功 | 切换至 P 成功 | 发送 EPB 联动夹紧请求 |


## 质量自检清单

| 检查项 | 状态 |
|--------|:---:|
| 模块编号与分区归属正确 | ✅ |
| 依赖与被依赖模块编号完整 | ✅ |
| 内部状态机5个状态含触发条件 | ✅ |
| 输入/输出含数据类型、来源/目标模块、优先级 | ✅ |
| 安全策略表完整（P/R/N/D 四种目标档位的准入条件） | ✅ |
| 核心处理逻辑伪代码覆盖条件校验、切换执行、超时故障处理全流程 | ✅ |
| 约束与异常覆盖超时、TCU故障、信号丢失、高速误触、排队、熔断 | ✅ |
| 总线契约区分内部总线与CAN总线 | ✅ |
| 安全边界逐条列出 | ✅ |
| 校验用例覆盖正常切换、条件拒绝、安全拒绝、成功、超时、EPB联动共6条 | ✅ |