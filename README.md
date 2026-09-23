# robot-motion-control — 机器人运动控制与安全内核

> 面向**人形 / 四足机器人二次开发**的底层运动控制库：
> 关节位置-速度-力矩控制（Kp/Kd）、平滑插值、动作序列编排、
> 状态监控与异常检测、以及一整套**实机安全机制**。
>
> **C++17 · 零第三方依赖 · 135 项单元测试**（不需要真实机器人即可全部跑通）

![cpp](https://img.shields.io/badge/C%2B%2B-17-blue)
![deps](https://img.shields.io/badge/dependencies-none-green)
![tests](https://img.shields.io/badge/tests-135%20passed-brightgreen)

---

## 1. 这个项目对应岗位的哪几条职责

| 岗位职责 | 本项目的对应实现 |
|---|---|
| ③ 底层关节控制：**位置/速度/力矩 + Kp/Kd** | `JointCommand{q, dq, tau, kp, kd}` + `JointLimit`；增益上限直接进安全检查 |
| ③ **平滑插值、周期控制、动作序列** | 线性 / 梯形 / **S 曲线（五次多项式）** 三种插值；500 Hz 固定周期；`Sequence` 动作序列 |
| ④ **状态数据处理**：关节/IMU/足端 | `RobotState{joints, imu_rpy, imu_gyro, foot_force, battery}` |
| ④ **数据记录、异常检测、超时处理、运行监控** | `Monitor`：逐通道统计（min/max/mean/样本数/缺失数/最大间隔）+ 超时判定 |
| ⑥ **实机安全机制** | `SafetyChecker`：初始状态检查、关节限位、速度/力矩上限、过温、姿态异常、通信丢失、CRC、低电、急停锁存 |
| ⑤ **通信抽象**（DDS / 发布订阅 / Client API） | `RobotPort` 接口：真机接 DDS，仿真用 `SimPort`（可注入丢包 / CRC 错 / 状态超时） |
| ⑧ **仿真验证 → 实机联调** | 同一套控制器代码，换 `RobotPort` 实现即可；仿真里能在毫秒内跑完几万拍 |

---

## 2. 快速开始

```bash
make test                                   # 135 项单元测试
make demo                                   # 全部场景演示
./build/motiondemo curves | stand | seq | safety
```

手动编译：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude \
    src/motion.cpp tests/test_motion.cpp -o build/mtest && ./build/mtest
```

最小用法：

```cpp
#include "motion.h"
using namespace motion;

SimPort port(12, 0.002);          // 12 个关节，500 Hz；真机换成 DDS 后端
RobotController rc(port, {});

if (!rc.start()) return 1;         // 连接 + 初始状态检查 + 切站立
std::vector<double> target(12, 0.2);
rc.move_joints(target, 0.6, Curve::SCURVE);   // 走 S 曲线，不用阶跃

for (int i = 0; i < 1000; ++i) {
    auto r = rc.tick();            // 读状态 -> 安全检查 -> 算指令 -> 下发
    if (!r.ok) {                   // 已自动切到 damp（阻尼保护态）
        printf("故障：%s - %s\n", safety_event_name(r.fault), r.detail.c_str());
        break;
    }
}
```

---

## 3. 目录结构

```
include/motion.h          全部接口（单头文件，便于直接搬进工程）
src/motion.cpp            插值 / 序列 / 安全 / 监控 / 仿真端口 / 控制器
src/main.cpp              四个演示场景
tests/test_motion.cpp     135 项单元测试
unitree/                  与 Unitree SDK2 的接口映射说明（见第 6 节）
```

---

## 4. 实测结果

```
==== 结果：135 passed, 0 failed ====
```

覆盖：

- **插值**：三种曲线的端点/单调性/越界夹取；S 曲线**起点与终点加速度严格为 0**、
  中点加速度为 0（对称性）、峰值位置正确；梯形在 `v_max` 不可行/过大两种边界都能到位；
  反向位移、零位移、零时长**都不出 NaN**
- **轨迹拼接**：多段总时长、**段边界连续**（不跳变）、超时停在终点、空轨迹不崩
- **动作序列**：步进与进度、每步 `on_enter` 各调一次、**步间位置连续**（最大跳变 < 0.05 rad）、
  空序列、reset 后重放
- **安全**：初始检查（关节无效/NaN/限位/姿态/低电/无关节）；
  **去抖**（单次超限不切安全态，连续 3 次才确认）；限位/速度/力矩/过温/姿态/角速度/低电/
  通信超时/CRC 各自触发；**指令本身非法（NaN、目标超限、增益过大）在下发前被拦截**；
  **急停锁存**（跑 500 拍仍拦截，必须显式解除）
- **监控**：min/max/mean/样本数、gap 计数、超时通道、整机状态批量记录、自适应注册
- **仿真端口**：跟随、尺寸不匹配指令被拒、丢包、状态超时、CRC 累积、服务切换记录
- **控制器**：启动 / 站立 / 行走 / 轨迹跟随 / 阻尼；**关节运动每拍位移 < 0.02 rad（非阶跃）**；
  初始检查不过则拒绝启动；运行中通信丢失 / CRC / 姿态异常各自切安全态

### 演示输出（安全场景）

```
① 初始姿态异常（roll=1.40 rad）：启动被拒绝
② 初始电量 5%：启动被拒绝
③ 运行中通信丢失：已检出（COMM_LOST）-> 服务切到 damp
④ CRC 错误累积：已检出（CRC）
⑤ 姿态异常（pitch=1.20 rad）：已检出（ATTITUDE）
⑥ 急停：立即拦截=是，跑 500 拍后仍拦截=是（锁存），显式解除后恢复=是
⑦ 姿态异常第 1 拍：放行（去抖中，不立刻切安全态）
```

---

## 5. 开发中踩到并修掉的问题

| # | 问题 | 性质 | 根因与修复 |
|---|---|---|---|
| 1 | ⭐ **动作序列首拍位置跳变 1.0 rad** | **实现真 bug** | 首步的起点取的是**目标值**而不是当前位置（`prev_` 为空时退化成"从目标出发"），于是第一拍直接跳到目标。实机上这种阶跃会把减速器打坏。修：加 `Sequence::set_initial()` 由调用方注入当前实际指令，`RobotController::play()` 里用当前 `cmd_` 注入 |
| 2 | ⭐ **梯形速度曲线在不可行区间退化** | **算法不完整** | 只判了 `v_max ≥ 三角峰值` 那一档，中间档的加速时间公式 `ta = 1 - 1/v_norm` 在 `v_norm ≤ 1`（连平均速度都达不到）时会变成**负数**，恒速段为空、形状退化。改为**三段显式判断**：`≥2` 用三角、`1~2` 用真梯形、`≤1` 退化为三角并尽快走完 |
| 3 | `SimPort::write_command` 返回类型与接口不符 | 编译错误 | 接口是 `bool`（发送成功与否），实现写成了 `void`。顺便让它对**尺寸不匹配**返回 false —— 不能让上层以为发成功了 |
| 4 | S 曲线加速度测试预期写反 | 我的测试预期错 | S 曲线加速度在中点**恰好为 0**（峰值在 u≈0.211 / 0.789），我拿中点当"中段峰值"去比两端。真正的性质是"**从 0 严格起、连续上升**"，改为查端点值 |
| 5 | 梯形测试参数落在不可行区间 | 我的测试预期错 | `v_max=0.8` 而平均速度需要 1.0，本来就走不完 → 换成 1.5（有效梯形），并**补了两条边界用例**（不可行 / 过大） |
| 6 | 粗心写坏的一段代码 | 自查 | `Sequence::tick` 里出现过 `traj_[idx_].push_back ? void() : void();` 这类无意义片段，重写时清掉 |

> 前两条是**实现里的真问题**：一个会**打坏硬件**（位置阶跃），
> 一个会在**特定参数下静默退化**（曲线形状不对但不报错）。
> 两者都由"用解析可验证的数据跑单测"抓出来。

---

## 6. 与 Unitree SDK2 的关系（**重要，务必看清**）

岗位要求基于 **Unitree SDK2** 做二次开发。**本项目没有接过 Unitree 机器人**
（手上没有机器狗/人形，也没有 Unitree 的 SDK 与仿真环境）。

本项目做的是**与 SDK 无关的那一半**，也就是真正难的一半：

| Unitree SDK2 里的东西 | 本项目里的对应物 | 关系 |
|---|---|---|
| `LowCmd`（关节指令） | `JointCommand{q, dq, tau, kp, kd}` | **字段形态一致**，可直接互转 |
| `LowState`（关节/IMU 状态） | `RobotState{joints, imu_rpy, foot_force, ...}` | **字段形态一致** |
| `SportClient`（站立/行走/阻尼等高层服务） | `RobotController::stand() / walk() / damp()` + `RobotPort::switch_service()` | 语义对应 |
| DDS Topic 发布订阅 | `RobotPort` 接口（真机实现接 DDS） | **抽象层次对应** |
| 急停 / 保护 | `SafetyChecker`（限位/姿态/通信/CRC/急停） | SDK 不提供，**必须自己写** |

**换句话说**：换成 Unitree 的机器人时，需要做的是
**写一个 `RobotPort` 的 DDS 实现**（把 `JointCommand` 填进 `LowCmd`、
把 `LowState` 读进 `RobotState`），其余逻辑一行不用改。

> 详细映射与接入步骤见 `unitree/接入说明.md`。
> **请勿把本项目说成"做过 Unitree 二次开发"** —— 没做过就是没做过。

---

## 7. 边界说明（重要）

- **没有连接过任何真实机器人**（Unitree 或其他品牌），也没有跑过 Gazebo/Isaac 仿真
- **没有使用 Unitree SDK2**：第 6 节列的是**接口形态的对应关系**，不是集成经验
- **`SimPort` 的被控对象是简化模型**（一阶跟随 + 弹簧-阻尼力矩估算 + 温度累积），
  不是刚体动力学仿真，**不能用来验证步态、稳定性、能耗**
- **未实现**：步态规划与平衡控制（ZMP/MPC/WBC）、状态估计（EKF/IMU 融合）、
  力控与阻抗控制、足端接触检测、全身动力学、真实 DDS 后端
- **`walk()` / `follow_trajectory()` 是示意实现**：真实系统里这些应下发给本体的
  步态控制器，由它做平衡；本项目只演示"高层指令 -> 关节指令"的这条通路
- **默认阈值为示例值**（姿态 0.6 rad、温度 80℃、心跳 200 ms 等），
  实机必须按机型与实测数据重新标定

**下一步可做**：写真实 DDS 后端（对接 Unitree LowCmd/LowState）；接 Gazebo 做整机验证；
补阻抗控制与足端接触检测；把阈值做成按机型加载的配置文件。

---

## 8. 许可

MIT
