/**
 * @file    sd_log.h
 * @brief   SD 卡连续日志子系统 (v2.2.0)
 *
 * 设计见 docs/SD卡采集子系统设计_v1.md：
 *   - 上电即采, 断电即止 (无按键)
 *   - 记录 = 112B ReportFrame_t (与 UDP 上报帧完全同格式, 可逐帧交叉验证)
 *   - ISR 只做内存拷贝 (sdlog_push), SD 写入全在主循环 (sdlog_task)
 *   - 环形块缓冲放【外部 SRAM 空闲区】, 吸收 SD 卡写延迟尖峰
 *   - 周期 f_sync(<=2s) —— 断电是唯一停止方式, 不同步则文件会丢
 */
#ifndef __SD_LOG_H
#define __SD_LOG_H

#include <stdint.h>
#include "contract.h"       /* ReportFrame_t */

/* ---- 缓冲布局 (外部 SRAM, 见 bsp_config.h 内存图免费区 0x6805C000~0x680FFFFF) ---- */
#define SDLOG_SRAM_BASE     0x68060000UL
#define SDLOG_BLK_BYTES     114688UL               /* 单块字节: 1024记录 × 112B = 224扇区 ✓ */
#define SDLOG_BLK_RECS      (SDLOG_BLK_BYTES / 112) /* = 1024 (1.024 s @1kHz) */
#define SDLOG_BLK_COUNT     4                      /* 4 块 = 458752 B ≈ 4.1 s 容错 */
#define SDLOG_SYNC_MS       2000u                  /* f_sync 周期 (设计 §0 强制) */

typedef struct {
    uint32_t rec_pushed;      /* 已入队 (含尚在缓冲未落卡) */
    uint32_t rec_flushed;     /* 已落卡记录数 (权威) */
    uint32_t rec_dropped;     /* 因环形满而丢弃的记录数 */
    uint32_t blk_flushed;
    uint32_t write_err;
    uint8_t  mounted;
    uint8_t  logging;
    char     fname[16];
} sdlog_stat_t;

/* 初始化: 挂载 + 吞吐自检 + 建新文件 + 写文件头; 返回 0 成功 */
int  sdlog_init(void);

/* ISR 调用: 入队一帧 (纯内存拷贝, µs 级) */
void sdlog_push(const ReportFrame_t *f);

/* 主循环调用: 写出满块 + 周期 f_sync */
void sdlog_task(void);

/* 查询统计 */
void sdlog_get_stat(sdlog_stat_t *out);

#endif /* __SD_LOG_H */
