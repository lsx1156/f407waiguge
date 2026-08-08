# F407 外骨骼控制器 v0.01

基于 STM32F407ZGT6 的双独立 CAN 总线外骨骼实时控制系统。

## 硬件平台

- **MCU**：STM32F407ZGT6（Cortex-M4，168MHz）
- **通信**：以太网 UDP（与 RK3506 上位机通信，端口 5001）
- **CAN1**：CyberGear 电机 ×2（下肢关节），1Mbps
- **CAN2**：RS01 电机 ×4（鳌臂关节），1Mbps
- **调试**：USART1（PA9/PA10，115200bps）
- **存储**：AT24C02 EEPROM（I2C1）
- **调试器**：ST-Link（SWD）

## 架构

```
裸机前后台架构（无 RTOS）
├── 前台（中断上下文，1ms/2ms）
│   ├── TIM6 中断 → 控制主循环
│   │   ├── ADC 采集（称重/电压）
│   │   ├── CAN 状态接收与路由
│   │   ├── 三级缓冲插值
│   │   ├── 阻尼/摩擦补偿
│   │   ├── 安全监控
│   │   ├── CAN 指令发送
│   │   └── FIFO 写入（供主循环发送）
│   ├── CAN1_RX0 中断 → 下肢电机状态接收
│   └── CAN2_RX0 中断 → 鳌臂电机状态接收
│
└── 后台（主循环）
    ├── MX_LWIP_Process（以太网协议栈）
    ├── UDP 数据发送（从 FIFO 读取）
    ├── UDP 命令解析（控制/参数帧）
    └── EEPROM 读写任务
```

## 通信协议

与 RK3506 通过以太网 UDP 通信，帧类型：

| 帧类型 | 方向 | 说明 |
|--------|------|------|
| 0x01 报告帧 | F407 → RK3506 | 6 关节状态 + 称重/电压 ADC |
| 0x02 控制帧 | RK3506 → F407 | 6 关节目标力矩（含重力补偿） |
| 0x03 参数帧 | RK3506 → F407 | 阻尼表/摩擦表/PID 参数更新 |
| 0x0F 心跳帧 | 双向 | 连接保活 |

## 关节映射

| 编号 | 关节 | CAN 总线 | 电机类型 |
|------|------|----------|----------|
| 0x01 | 左髋 | CAN1 | CyberGear |
| 0x02 | 左膝 | CAN1 | CyberGear |
| 0x10 | 鳌臂1 | CAN2 | RS01 |
| 0x11 | 鳌臂2 | CAN2 | RS01 |
| 0x12 | 鳌臂3 | CAN2 | RS01 |
| 0x13 | 鳌臂4 | CAN2 | RS01 |

## 安全机制

- 急停检测（FAULT_ESTOP）
- CAN 超时检测（CAN1/CAN2 各独立 100ms 超时）
- 位置跳变检测
- 速度/力矩超限
- 电压过低检测
- 称重传感器开路检测
- 双 CAN 总线故障域隔离（一条总线异常不影响另一条）

## 双 CAN 隔离设计

| 资源 | CAN1（下肢） | CAN2（鳌臂） |
|------|-------------|-------------|
| 过滤器 Bank | 0 | 14 |
| 中断优先级 | 0（抢占） | 1（抢占） |
| GPIO 引脚 | PA11/PA12（AF9） | PB12/PB13（AF9） |
| 状态变量 | `g_leg_status[2]` | `g_arm_status[4]` |
| 超时计数器 | `g_can1_timeout_cnt` | `g_can2_timeout_cnt` |

## 目录结构

```
IDE/
├── Core/               # STM32CubeIDE 生成
│   ├── Inc/            # 头文件（main.h, stm32f4xx_hal_conf.h, stm32f4xx_it.h）
│   ├── Src/            # 源文件（main.c, stm32f4xx_hal_msp.c, stm32f4xx_it.c）
│   └── Startup/        # 启动文件（startup_stm32f407zgtx.s）
├── Drivers/            # 驱动
│   ├── CMSIS/          # ARM CMSIS
│   ├── STM32F4xx_HAL_Driver/  # STM32 HAL 库
│   └── SYSTEM/         # 系统驱动（delay, malloc, sys, usart）
├── Middlewares/        # 中间件
│   └── lwip/           # LwIP 协议栈（NO_SYS 模式）
│       ├── arch/       # 移植层（ethernetif, lwip_comm, udp_net, sys_arch）
│       └── src/        # LwIP 内核
├── App/                # 应用层
│   ├── bsp/            # 板级支持（ethernet, sram, contract, bsp_config）
│   ├── can_motor/      # CAN 电机驱动（MIT 协议）
│   ├── control/        # 控制逻辑（control_isr, tasks）
│   ├── eeprom/         # EEPROM 参数存储
│   ├── interp/         # 三级缓冲插值
│   ├── safety/         # 安全监控
│   └── udp_proto/      # UDP 协议解析
└── trae.ioc            # CubeMX 工程配置
```

## 编译与烧录

**环境**：STM32CubeIDE 1.15.1 + ARM GNU Toolchain 12.3

```bash
# 编译
cd Debug && make -j16 all

# 生成 HEX
generate_hex.bat

# 烧录（ST-Link）
download_stlink.bat
```

**BOOT0 必须接 LOW（GND）**，否则程序从系统 Bootloader 启动。

## 中断优先级

| 中断 | 优先级 | 说明 |
|------|--------|------|
| TIM6_DAC | 0（最高） | 1ms 控制主循环 |
| CAN1_RX0 | 0 | 下肢电机状态接收 |
| CAN2_RX0 | 1 | 鳌臂电机状态接收 |
| ETH | 2 | 以太网 DMA |

## 许可

内部项目，未开源。