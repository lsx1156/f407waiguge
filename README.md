# F407 外骨骼控制器

> **最新工程版本：`v2.0.1`** （tag `v2.0.1`，commit `eec6c49`）
> 工程制作过程见 [Tags](https://github.com/lsx1156/f407waiguge/tags)：`v0.01` → `v1.0` → `v2.0` → `v2.0.1`

基于 STM32F407ZGT6 的双独立 CAN 总线外骨骼实时控制系统。
6 关节（2 下肢 + 4 上肢）协同助力，支持步态/工业/鳌臂助力多模式切换，
内置 ABO 自适应力矩观测器与多层安全保护。

---

## 硬件平台

- **MCU**：STM32F407ZGT6（Cortex-M4F，168MHz，192KB SRAM + 1MB Flash）
- **通信**：以太网 UDP（与 RK3506 上位机通信，端口 5001）
- **CAN1**：CyberGear 电机 ×2（下肢髋关节），1Mbps
- **CAN2**：RS01 电机 ×4（上肢肩/肘关节），MIT 力矩模式，1Mbps
- **调试**：USART1（PA9/PA10，115200bps）
- **存储**：AT24C02 EEPROM（I2C1，参数持久化）
- **外部 SRAM**：IS62WV51216（1MB，LWIP 缓冲）
- **调试器**：ST-Link（SWD）
- **传感器限制**：无 IMU，仅 ABS 编码器 + 相电流 + CAN 反馈

---

## 架构

```
裸机前后台架构（无 RTOS，1ms 实时控制周期）
├── 前台（中断上下文）
│   ├── TIM6 中断 (1ms, 优先级0) → 控制主循环
│   │   ├── 1ms: local_cmd_generate()  各模式命令生成
│   │   │   ├── 模式分发 (ZERO/GAIT/ARM_ASSIST/INDUSTRIAL)
│   │   │   ├── GAIT 步态表插值 + 上肢阻抗摆臂
│   │   │   ├── 重力/摩擦补偿 (ARM_ASSIST)
│   │   │   └── ABO 观测器 abo_update_one()
│   │   ├── CAN 状态路由 (g_leg/g_arm_status)
│   │   ├── 安全监控 (safety_check)
│   │   ├── 软限位 / 碰撞检测
│   │   └── CAN 指令发送 (can_motor_send_command)
│   ├── CAN1_RX0 中断 → 下肢电机状态接收
│   └── CAN2_RX0 中断 → 上肢电机状态接收
│
└── 后台（主循环）
    ├── MX_LWIP_Process（以太网协议栈）
    ├── UDP 数据发送（6 关节状态报告，从 FIFO 读取）
    ├── UDP 命令解析（控制/参数/模式帧）
    ├── industrial_load_estimator()（100Hz 负载估计 LPF）
    └── EEPROM 读写任务
```

---

## 控制模式

`local_mode` 定义 4 种工作模式（mode_manager 管理）：

| 模式 | 值 | 下肢 | 上肢 | ABO | 场景 |
|------|----|------|------|-----|------|
| ZERO_TORQUE | 0 | TORQUE 0 透明 | TORQUE 0 + 软限位 | 全关 | 默认安全态/穿戴 |
| GAIT | 1 | POSITION 步态表 | TORQUE 阻抗摆臂 | 腿启用 | 自适应步行 |
| ARM_ASSIST | 2 | TORQUE 0 + ABO | TORQUE 0 + ABO 重力悬停 | 全启用 | 鳌臂助力 |
| INDUSTRIAL | 3 | TORQUE 0 + ABO工业 | TORQUE 0 + ABO工业 | 工业模式 | 工业搬运助力 |

- **GAIT 步态**：5 点插值步态表，髋摆幅 30°，周期 4s，POSITION 模式 Kp=3 Kd=1.0；
  启动 2 秒渐变；上肢阻抗反相摆臂（低刚度+死区，方向经 ARM_DIR 翻转）
- **ARM_ASSIST**：上肢 ABO 重力悬停 + 库仑/粘性摩擦补偿，gain=0.25×
- **INDUSTRIAL**：ABO 工业模式，速度推断人力，按负载分档增益调度

---

## ABO 自适应力矩观测器

`abo_update_one()` 运行于 1ms ISR，命令生成后、CAN 发送前就地修改 `syn_target`。

### 医疗模式（idx 0~5）

```
tau_meas → 偏置估计(leak吸收重力) → HPF(α)提取tau_human → assist=gain·tau_human
```

| 关节/相位 | gain | α (HPF) | leak (偏置) |
|-----------|------|---------|-------------|
| 默认 | 1.0× | ~1.6Hz | ~1s |
| 腿-站立相 | 1.0× | 增大 | ~1s |
| 腿-摆动相 | 1.0× | 减小 | 加快 |
| 臂(GAIT) | 0 (阻抗接管) | - | - |
| 臂(ARM_ASSIST) | 0.25× | ~3Hz | ~4s |

限幅 ±3.0 N·m

### 工业模式（速度推断，v2.0.1 新增）

纯力矩模式下 `tau_meas≈命令` 不反映静态人力，改用速度推断：

```
人推 → 关节速度 → 同向助力(负阻尼)
tau_human = K_VEL · vel  (vel 有死区+上限)
```

| 参数 | 值 | 说明 |
|------|----|------|
| K_VEL | 0.6 Nm·s/rad | 负阻尼系数（须 < 系统摩擦） |
| VEL_DEADZONE | 0.15 rad/s | 死区，滤噪声漂移 |
| VEL_MAX | 3.0 rad/s | 速度上限，防过速 |
| 输出 LPF | ~12Hz | 防振荡 |
| 偏置 leak | ~60s | 极慢积分，突变冻结 500ms |
| 增益调度 | 1.0/0.5/0.25× | 按 load_est 分档（<5/<15/≥15 N·m） |
| 限幅 | ±1.5 N·m | 防失控 |

---

## 安全机制

### 多层保护

| 层级 | 机制 | 触发 | 动作 |
|------|------|------|------|
| 软限位 | 上肢 ±90° + 15° 软带 | 位置超限 | 线性回复至 1.5 N·m |
| 碰撞检测 | 力矩突变 > 8 N·m/ms | Δτ 超阈 | 瞬间零力矩 + 失能 + FAULT |
| GAIT 回归 | 模式切出 GAIT | prev=GAIT | 3 秒 POSITION 平滑回归 15°/s |
| ZERO 切断 | 切入 ZERO | mode=0 | ABO 全关 + 力矩清零（跳过渐变） |
| 急停 | FAULT_ESTOP | 急停信号 | 全机零力矩 |
| CAN 超时 | 100ms 无反馈 | 总线异常 | 该总线关节零力矩 |
| 状态机 | safety.c | WARN/FAULT | WARN 不改力矩，FAULT 零力矩 |

### 力矩限幅汇总

| 路径 | 限幅 |
|------|------|
| GAIT 上肢阻抗 | ±0.3 N·m |
| ABO 医疗模式 | ±3.0 N·m |
| ABO 工业模式 | ±1.5 N·m |
| 软限位回复 | ±1.5 N·m |
| PID tlimit | 参数表配置 |

---

## 方向约定

**臂电机（RS01 MIT 模式）正方向 = 上肢解剖反方向**（4 臂全部反相）。

- GAIT 上肢阻抗：`ref` 项乘 `ARM_DIR = -1.0`（pos/vel 项因 S²=1 不变）
- ABO 测量/命令同在电机系，S 自动抵消
- 工业模式速度推断：直接用电机系 vel，方向自洽

腿电机方向按原约定（正方向 = 屈髋/前摆）。

---

## 通信协议

与 RK3506 通过以太网 UDP 通信（端口 5001）：

| 帧类型 | 方向 | 说明 |
|--------|------|------|
| 0x01 报告帧 | F407 → RK3506 | 6 关节状态 + 称重/电压 ADC |
| 0x02 控制帧 | RK3506 → F407 | 6 关节目标力矩（含重力补偿） |
| 0x03 参数帧 | RK3506 → F407 | 阻尼/摩擦/PID 参数更新 |
| 0x0F 心跳帧 | 双向 | 连接保活 |

模式切换经控制帧字段下发（local_mode）。

---

## 关节映射

| 编号 | 关节 | CAN 总线 | 电机类型 | 模式 |
|------|------|----------|----------|------|
| 0 | 左髋 | CAN1 | CyberGear | POSITION(GAIT)/TORQUE |
| 1 | 右髋 | CAN1 | CyberGear | POSITION(GAIT)/TORQUE |
| 2 | 左肩 | CAN2 | RS01 | TORQUE |
| 3 | 左肘 | CAN2 | RS01 | TORQUE |
| 4 | 右肩 | CAN2 | RS01 | TORQUE |
| 5 | 右肘 | CAN2 | RS01 | TORQUE |

---

## 双 CAN 隔离设计

| 资源 | CAN1（下肢） | CAN2（上肢） |
|------|-------------|-------------|
| 过滤器 Bank | 0 | 14 |
| 中断优先级 | 0（抢占） | 1（抢占） |
| GPIO | PA11/PA12（AF9） | PB12/PB13（AF9） |
| 状态变量 | `g_leg_status[2]` | `g_arm_status[4]` |
| 超时计数器 | `g_can1_timeout_cnt` | `g_can2_timeout_cnt` |

双总线故障域隔离：一条总线异常不影响另一条。

---

## 目录结构

```
trae-ecc/
├── Core/               # STM32CubeIDE 生成
│   ├── Inc/            # main.h, hal_conf, it.h
│   ├── Src/            # main.c, hal_msp, it, syscalls, sysmem
│   └── Startup/        # startup_stm32f407zgtx.s
├── Drivers/
│   ├── CMSIS/          # ARM CMSIS
│   ├── STM32F4xx_HAL_Driver/  # STM32 HAL 库
│   └── SYSTEM/         # delay, malloc, sys, usart
├── Middlewares/
│   └── lwip/           # LwIP 协议栈（NO_SYS 模式）
│       ├── arch/       # 移植层（ethernetif, lwip_comm, udp_net, sys_arch）
│       └── src/        # LwIP 内核
├── App/                # 应用层
│   ├── bsp/            # 板级支持（ethernet, sram, contract, bsp_config, LCD）
│   ├── can_motor/      # CAN 电机驱动（MIT 协议, RS01/CyberGear）
│   ├── control/        # 控制核心
│   │   ├── control_isr.c/h   # 1ms ISR 主控（模式分发/ABO/阻抗/步态）
│   │   ├── tasks.c/h         # 后台任务（负载估计/状态上报）
│   │   ├── mode_manager.c/h  # 模式状态机
│   │   ├── joint_unit.c/h    # v2.0 ESO3/导纳/三角查表（默认关闭）
│   │   ├── global_phase.c/h  # 步态相位管理
│   │   ├── global_pose.c/h   # 全局姿态
│   │   └── global_coordinator.c/h  # 全局协调
│   ├── eeprom/         # EEPROM 参数存储（eeprom_params）
│   ├── interp/         # 三级缓冲插值
│   ├── safety/         # 安全监控状态机
│   └── udp_proto/      # UDP 协议解析
├── trae.ioc            # CubeMX 工程配置
├── STM32F407ZGTX_FLASH.ld / RAM.ld  # 链接脚本
├── generate_hex.bat / download_stlink.bat / download_daplink.bat
└── openocd_*.cfg / trae Debug.cfg    # 调试配置
```

---

## 编译与烧录

**环境**：STM32CubeIDE 1.15.1 + ARM GNU Toolchain 12.3

```bash
# 编译
cd Debug && make -j16 all

# 生成 HEX
generate_hex.bat

# 烧录（ST-Link）
download_stlink.bat
# 或 DAP-Link
download_daplink.bat
```

**BOOT0 必须接 LOW（GND）**，否则从系统 Bootloader 启动。

---

## 中断优先级

| 中断 | 优先级 | 说明 |
|------|--------|------|
| TIM6_DAC | 0（最高） | 1ms 控制主循环 |
| CAN1_RX0 | 0 | 下肢电机状态接收 |
| CAN2_RX0 | 1 | 上肢电机状态接收 |
| ETH | 2 | 以太网 DMA |

---

## 版本演进

| 版本 | 主要内容 |
|------|----------|
| v0.01 | 初始版本：双 CAN 基础控制 + UDP 通信 |
| v1.6.x | ABO 观测器 + GAIT 步态 + 安全监控 |
| v2.0 | 安全状态机 + ESO3/导纳（默认关闭）+ 三角查表 + 单测回放 |
| v2.0.1 | 安全修复：上肢方向(ARM_DIR) + 工业模式速度推断 + ±90°软限位 + GAIT 降幅度降刚度 |

---

## 已知限制

1. **零位未标定**：GAIT 上肢重力前馈关闭，靠阻抗死区缓解 pos 偏移
2. **工业模式负阻尼**：K_VEL 须 < 系统摩擦系数，需现场调参
3. **GAIT 开环轨迹**：不跟随用户意图（已降幅度30°+降刚度Kp=3 缓解）
4. **ESO3/导纳默认关闭**：`eso_enable=0`，待调参收敛后启用

---

## 许可

内部项目，未开源。
