# AD-mcc-cerebellum：自动驾驶 MCC 运动小脑

**EM-Core 运动小脑 · 自动驾驶专项实现 · 38模块正式定稿**

> 版本：V1.0
> 原创提出者：文波福
> 开源协议：CC BY 4.0（知识共享署名 4.0 国际许可证）
> 所属体系：EM-Core AD 自动驾驶认知系统
> 配套仓库：[EM-Core-AD-Spec](https://gitee.com/expanding-research/em-core-ad-spec)（总规范）｜ [AD-mlnf-mem](https://gitee.com/expanding-research/ad-mlnf-mem)（记忆中枢）｜ [AD-ecc-brain](https://gitee.com/expanding-research/ad-ecc-brain)（认知大脑）


## 一、仓库定位

本仓库为 EM-Core 通用智能系统中 **MCC 运动小脑** 的自动驾驶专项实现仓库，是自动驾驶车辆唯一底层执行单元。

AD-mcc-cerebellum 只负责将 ECC 大脑下发的目标级行驶意图转化为车辆物理操控指令，独立完成方向盘转向、油门控制、制动执行、灯光切换、档位管理、雨刮外设及车身姿态稳定。只负责精准执行，不参与任何场景判断与驾驶决策。

本系统不纳入：逆运动学、底层轨迹插补、PID 控制、底层力控、电机驱动、硬件总线驱动、机械硬限位保护等底层范畴。底层全部运动硬件闭环、底层控制算法、硬件参数调参、设备驱动，全由硬件厂商独立承接。


## 二、模块分区总览（38个模块）

### 分区一：顶层总控中枢（01–03）

| 编号 | 模块名称 |
|:---:|------|
| 01 | [小脑总控调度核心](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-01-小脑总控调度核心.md) |
| 02 | [运动生理边界闸门](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-02-运动生理边界闸门.md) |
| 03 | [全身运动状态归集中心](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-03-全身运动状态归集中心.md) |

### 分区二：转向控制集群（04–08）

| 编号 | 模块名称 |
|:---:|------|
| 04 | [方向盘转角解算单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-04-方向盘转角解算单元.md) |
| 05 | [转向平顺滤波单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-05-转向平顺滤波单元.md) |
| 06 | [横向冲击度约束单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-06-横向冲击度约束单元.md) |
| 07 | [转向执行偏差监控单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-07-转向执行偏差监控单元.md) |
| 08 | [非铺装路面转向适配单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-08-非铺装路面转向适配单元.md) |

### 分区三：动力控制集群（09–12）

| 编号 | 模块名称 |
|:---:|------|
| 09 | [油门开度解算单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-09-油门开度解算单元.md) |
| 10 | [纵向冲击度约束单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-10-纵向冲击度约束单元.md) |
| 11 | [加速平顺滤波单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-11-加速平顺滤波单元.md) |
| 12 | [动力执行偏差监控单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-12-动力执行偏差监控单元.md) |

### 分区四：制动控制集群（13–17）

| 编号 | 模块名称 |
|:---:|------|
| 13 | [制动压力解算单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-13-制动压力解算单元.md) |
| 14 | [制动响应加速单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-14-制动响应加速单元.md) |
| 15 | [制动平顺防点头单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-15-制动平顺防点头单元.md) |
| 16 | [制动执行偏差监控单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-16-制动执行偏差监控单元.md) |
| 17 | [再生制动优先协调单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-17-再生制动优先协调单元.md) |

### 分区五：车身姿态稳定（18–21）

| 编号 | 模块名称 |
|:---:|------|
| 18 | [车身姿态实时监测单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-18-车身姿态实时监测单元.md) |
| 19 | [横摆稳定控制单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-19-横摆稳定控制单元.md) |
| 20 | [侧翻临界保护单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-20-侧翻临界保护单元.md) |
| 21 | [颠簸路面姿态补偿单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-21-颠簸路面姿态补偿单元.md) |

### 分区六：灯光与外设管理（22–25）

| 编号 | 模块名称 |
|:---:|------|
| 22 | [转向灯自动控制单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-22-转向灯自动控制单元.md) |
| 23 | [双闪与刹车灯控制单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-23-双闪与刹车灯控制单元.md) |
| 24 | [远近光灯自动切换单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-24-远近光灯自动切换单元.md) |
| 25 | [雨刮与外设自适应单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-25-雨刮与外设自适应单元.md) |

### 分区七：档位与驻车管理（26–27）

| 编号 | 模块名称 |
|:---:|------|
| 26 | [档位切换管控单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-26-档位切换管控单元.md) |
| 27 | [电子驻车制动控制单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-27-电子驻车制动控制单元.md) |

### 分区八：硬件异常应急防护（28–31）

| 编号 | 模块名称 |
|:---:|------|
| 28 | [转向系统异常监测单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-28-转向系统异常监测单元.md) |
| 29 | [制动系统异常监测单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-29-制动系统异常监测单元.md) |
| 30 | [动力系统异常监测单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-30-动力系统异常监测单元.md) |
| 31 | [通信总线异常监测单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-31-通信总线异常监测单元.md) |

### 分区九：多车型自适应适配（32–35）

| 编号 | 模块名称 |
|:---:|------|
| 32 | [车辆尺寸参数管理单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-32-车辆尺寸参数管理单元.md) |
| 33 | [转向特性参数管理单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-33-转向特性参数管理单元.md) |
| 34 | [动力与制动参数管理单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-34-动力与制动参数管理单元.md) |
| 35 | [能源参数管理单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-35-能源参数管理单元.md) |

### 分区十：执行反馈与日志（36–38）

| 编号 | 模块名称 |
|:---:|------|
| 36 | [执行闭环反馈单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-36-执行闭环反馈单元.md) |
| 37 | [运动质量评估单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-37-运动质量评估单元.md) |
| 38 | [执行日志记录单元](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/spec/ad-mcc-38-执行日志记录单元.md) |


## 三、目录结构

```
AD-mcc-cerebellum/
├── README.md
├── LICENSE
├── spec/                  ← 38个模块接口规格文档
│   ├── README.md
│   ├── ad-mcc-01-小脑总控调度核心.md
│   └── ...
├── src/                   ← 38个模块 Python 源代码
│   ├── [bus.py](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/src/bus.py)
│   ├── [main.py](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/src/main.py)
│   ├── [module_registry.py](https://gitee.com/expanding-research/ad-mcc-cerebellum/blob/master/src/module_registry.py)
│   └── ...
└── .gitignore
```


## 四、与 AD-ecc-brain 的协同

| 数据流方向 | 内容 |
|-----------|------|
| [AD-ecc-brain](https://gitee.com/expanding-research/ad-ecc-brain) → 本仓库 | 标准化行驶意图指令（巡航/避险/变道/路口/补能） |
| 本仓库 → [AD-ecc-brain](https://gitee.com/expanding-research/ad-ecc-brain) | 执行状态反馈、偏差告警、运动质量报告 |

跨系统通信统一走 **CerebellumBus** 运动总线。


## 五、开源协议

本仓库内容采用 **CC BY 4.0**（知识共享署名 4.0 国际许可证）进行全球开源授权。

- 必须显著保留原作者署名：**文波福**
- 架构首创权永久归属原作者，不可剥夺、不可转移


## 六、学术引用

文波福. AD-mcc-cerebellum V1.0——自动驾驶 MCC 运动小脑模块定稿[EB/OL]. 2026.


## 七、联系方式

- **原创提出者**：文波福
- **邮箱**：710705008@qq.com
- **首发平台**：知乎、CSDN、稀土掘金、GitHub
- **Gitee 组织**：拓研（expanding-research）


## 八、镜像仓库

本仓库同步镜像至 [GitHub](https://github.com/expanding-research/ad-mcc-cerebellum)