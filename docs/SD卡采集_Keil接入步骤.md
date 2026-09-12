# SD 卡采集 · Keil 接入步骤（v2.2.0）

> 代码已全部写入仓库，**但 Keil 工程文件需你在 GUI 里加文件**（Keil 会覆写 `uvprojx`，手工改容易被覆盖）。
> 设计依据：`docs/SD卡采集子系统设计_v1.md`（v1.1：无按键 / 上电即采 / 断电即止 / ZERO 硬禁）

---

## 一、新增到工程的文件（5 个 .c）

| 建议分组名 | 文件 | 说明 |
|---|---|---|
| `FATFS` | `Middlewares/FATFS/source/ff.c` | FatFS R0.15 主体 |
| `FATFS` | `Middlewares/FATFS/source/ffunicode.c` | **必需**（提供 `MKCVTBL` 与码表，R0.15 起 ff.c 依赖它） |
| `FATFS` | `Middlewares/FATFS/source/diskio.c` | 我们改写的版本（仅 SD、有限重试、无 NORFLASH） |
| `SDIO` | `Drivers/BSP/SDIO/sdio_sdcard.c` | **DMA 版** SDIO 驱动（去掉了厂商的 `__disable_irq()` 轮询） |
| `App/sdlog` | `App/sdlog/sd_log.c` | 日志子系统（环形块缓冲 / 头 / 周期 f_sync） |
| `HAL` | `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c` | SD HAL 驱动（仓库已有，此前未加入编译） |

> `ffsystem.c` **不用加**（`FF_FS_REENTRANT=0` 时是空文件）；
> `ffunicode.c` 虽然 1.97 MB，但只有所选码表会被编译，可放心加入。

## 二、新增包含路径（Options for Target → C/C++ → Include Paths）

```
..\..\Middlewares\FATFS\source
..\..\Drivers\BSP\SDIO
..\..\App\sdlog
```
（`..\..\Drivers` 已存在，厂商代码里的 `./SYSTEM/...`、`./BSP/...` 靠它解析，无需再加。）

## 三、不需要手动改的

| 项 | 状态 |
|---|---|
| `HAL_SD_MODULE_ENABLED` | **已开**（`User/stm32f4xx_hal_conf.h:65`）✓ |
| SDIO 引脚（PC8~PC12 / PD2） | 已核对**全空闲**，无冲突 ✓ |
| DMA2_Stream3/6 Channel4 | 已核对**未被占用** ✓ |
| 编译器 | **必须 ARM Compiler 6**（工程 `<uAC6>`；AC5 会 144 个 `contract.h` 报错，与本次改动无关） |

## 四、编译后应看到（自检输出，串口 115200）

```
[SDLOG] SD 卡: 类型=.. 版本=.. 类别=.. 容量=.... MB
[SDLOG] 吞吐自检: 1024 KB / NNN ms = X.XX MB/s
[SDLOG] 开始采集 → LOG_0001.BIN (记录 112 B, 块 1024 记录, 4 块缓冲)
```
**吞吐自检 < 0.5 MB/s 就不正常**（1 kHz × 112 B 只需 0.11 MB/s）。

## 五、上板运行要点

| 项 | 要求 |
|---|---|
| 卡 | **micro SD，≤32 GB，FAT32**（本 FatFS `FF_LBA64=0`，**不认 exFAT**）；Class 10/U1 以上 |
| 插入时机 | **上电前插好**（无热插拔处理） |
| 采集 | **上电即开始**，断电即结束 → 每次上电生成 `LOG_nnnn.BIN`（号自动递增） |
| 模式 | `SDLOG_ZERO_ONLY=1`（`bsp_config.h`）→ `mode_request_switch()` 硬拒其余模式；六电机恒零力矩 |
| 断电保护 | 每 **2 s** 回写文件头 + `f_sync()`；断电最多丢最后 2 s |
| 数据 | 记录 = 112 B `ReportFrame_t`，**与 UDP 上报帧同格式**，可逐帧交叉验证 |

## 六、取数据

1. 断电 → 拔卡 → 插到 PC；
2. 拷出 `LOG_nnnn.BIN`；
3. ```bash
   python tools\sd_log_extract.py LOG_0001.BIN --csv LOG_0001.csv
   ```
   工具会校验头、统计**覆盖率/时间空洞/序号缺口/CRC**，并给出"通过 / 不达标"判定
   （`--selftest` 可先在无数据时试跑）。

## 七、若要回退

`SDLOG_ENABLE=0`（`App/bsp/bsp_config.h`）→ 日志子系统与 ISR 入队全部编译掉，恢复原行为（UDP 路不受影响，它一直保留）。
