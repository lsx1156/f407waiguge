/**
 * @file    sd_log.c
 * @brief   SD 卡连续日志子系统实现 (v2.2.0)
 * 设计: docs/SD卡采集子系统设计_v1.md (v1.1: 无按键, 上电即采, 断电即止)
 */
#include "sd_log.h"
#include "ff.h"
#include "diskio.h"
#include "sdio_sdcard.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#define SDLOG_REC_BYTES     112u
#define SDLOG_HDR_BYTES     512u
#define SDLOG_MAGIC         "F407LOG1"
#define SDLOG_VER           0x0202u
#define SDLOG_FW            "v2.2.0-sdlog"
#define SDLOG_SPEED_MB      1u          /* 吞吐自检写入 1 MB */

#define BLK(i)   ((uint8_t *)(SDLOG_SRAM_BASE + (uint32_t)(i) * SDLOG_BLK_BYTES))

/* ---------------- 文件头 (512B) ---------------- */
typedef struct __attribute__((packed)) {
    char     magic[8];          /* "F407LOG1" */
    uint16_t ver;
    uint16_t rec_size;          /* 112 */
    uint32_t sample_rate;       /* 1000 */
    uint32_t blk_recs;          /* 每块记录数 */
    uint32_t rec_total;         /* ★ 周期回写: 文件中有效记录数 (权威字段) */
    uint32_t rec_dropped;       /* ★ 周期回写: 因缓冲满丢弃的记录数 */
    uint32_t ts_first;
    uint32_t ts_last;           /* ★ 周期回写 */
    char     fw[16];
    uint32_t blk_flushed;
    uint32_t write_err;
    uint8_t  reserved[SDLOG_HDR_BYTES - 60];   /* 前 60 B 为上述字段 */
} sdlog_hdr_t;

typedef char _sdlog_hdr_check[(sizeof(sdlog_hdr_t) == SDLOG_HDR_BYTES) ? 1 : -1];

/* ---------------- 状态 ---------------- */
static FATFS          s_fs;
static FIL            s_fil;
static sdlog_hdr_t    s_hdr;

static volatile uint32_t s_head_blk;     /* 生产(ISR): 正在写的块 */
static volatile uint32_t s_wr_cnt;       /* 生产(ISR): 当前块已写记录数 */
static volatile uint32_t s_tail_blk;     /* 消费(主循环): 下一个待写出块 */
static volatile uint32_t s_ts_first, s_ts_last;

static sdlog_stat_t   s_st;
static uint8_t        s_inited;
static uint32_t       s_last_sync_ms;

/* ---------------- 文件头写入 + f_sync ---------------- */
static void sdlog_write_header(void)
{
    UINT bw = 0;

    memcpy(s_hdr.magic, SDLOG_MAGIC, 8);
    s_hdr.ver         = SDLOG_VER;
    s_hdr.rec_size    = SDLOG_REC_BYTES;
    s_hdr.sample_rate = 1000;
    s_hdr.blk_recs    = SDLOG_BLK_RECS;
    s_hdr.rec_total   = s_st.rec_flushed;
    s_hdr.rec_dropped = s_st.rec_dropped;
    s_hdr.ts_first    = s_ts_first;
    s_hdr.ts_last     = s_ts_last;
    memcpy(s_hdr.fw, SDLOG_FW, sizeof(s_hdr.fw));
    s_hdr.blk_flushed = s_st.blk_flushed;
    s_hdr.write_err   = s_st.write_err;

    if (f_lseek(&s_fil, 0) != FR_OK) { s_st.write_err++; return; }
    if (f_write(&s_fil, &s_hdr, SDLOG_HDR_BYTES, &bw) != FR_OK || bw != SDLOG_HDR_BYTES) {
        s_st.write_err++;
    }
    (void)f_sync(&s_fil);                     /* ★ 断电保护: 目录项+FAT 落盘 */
}

/* ---------------- 吞吐自检 ---------------- */
static void sdlog_speed_test(void)
{
    FIL      tf;
    UINT     bw;
    uint32_t total = SDLOG_SPEED_MB * 1024u * 1024u;
    uint32_t chunk = 32u * 1024u;
    uint8_t *buf   = BLK(0);                  /* 借第 0 块做测试缓冲(此时尚未开始采集) */

    if (f_open(&tf, "SPEED.BIN", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        printf("[SDLOG] 吞吐自检: 建文件失败\r\n");
        return;
    }
    for (uint32_t i = 0; i < chunk; i++) {
        buf[i] = (uint8_t)(i * 7u);
    }

    uint32_t t0 = HAL_GetTick();
    for (uint32_t off = 0; off < total; off += chunk) {
        if (f_write(&tf, buf, chunk, &bw) != FR_OK || bw != chunk) {
            printf("[SDLOG] 吞吐自检: 写失败 @%lu\r\n", (unsigned long)off);
            (void)f_close(&tf);
            (void)f_unlink("SPEED.BIN");
            return;
        }
    }
    (void)f_sync(&tf);
    uint32_t dt = HAL_GetTick() - t0;
    (void)f_close(&tf);
    (void)f_unlink("SPEED.BIN");

    if (dt == 0) {
        dt = 1;
    }
    printf("[SDLOG] 吞吐自检: %lu KB / %lu ms = %lu.%02lu MB/s\r\n",
           (unsigned long)(total / 1024u), (unsigned long)dt,
           (unsigned long)((total / 1024u) / dt), (unsigned long)(((total % (dt * 1024u)) * 100u) / (dt * 1024u)));
}

/* ---------------- 建新文件 (扫不到目录列表, 用 CREATE_NEW 递增试) ---------------- */
static int sdlog_new_file(void)
{
    char name[16];

    for (int n = 1; n <= 9999; n++) {
        snprintf(name, sizeof(name), "LOG_%04d.BIN", n);
        FRESULT fr = f_open(&s_fil, name, FA_CREATE_NEW | FA_WRITE | FA_READ);
        if (fr == FR_OK) {
            strncpy(s_st.fname, name, sizeof(s_st.fname) - 1);
            s_st.fname[sizeof(s_st.fname) - 1] = '\0';
            return 0;
        }
        if (fr != FR_EXIST) {
            printf("[SDLOG] f_open(%s) 失败 fr=%d\r\n", name, (int)fr);
            return -1;
        }
    }
    return -1;
}

/* ---------------- 初始化 ---------------- */
int sdlog_init(void)
{
    DSTATUS ds;
    FRESULT fr;
    HAL_SD_CardInfoTypeDef ci;
    uint8_t r;

    memset(&s_st, 0, sizeof(s_st));
    s_inited = 0;
    s_st.logging = 0;

    r = sd_init();
    if (r != 0) {
        printf("[SDLOG] SD 初始化失败 code=%u (卡是否上电前插好?)\r\n", (unsigned)r);
        return -1;
    }
    (void)get_sd_card_info(&ci);
    printf("[SDLOG] SD 卡: 类型=%lu 版本=%lu 类别=%lu 容量=%lu MB\r\n",
           (unsigned long)ci.CardType, (unsigned long)ci.CardVersion,
           (unsigned long)ci.Class, (unsigned long)(((uint64_t)ci.LogBlockNbr * 512ull) >> 20));

    ds = disk_initialize(0);
    if (ds != 0) {
        printf("[SDLOG] disk_initialize 失败 ds=%u\r\n", (unsigned)ds);
        return -2;
    }

    fr = f_mount(&s_fs, "", 1);
    if (fr != FR_OK) {
        printf("[SDLOG] f_mount 失败 fr=%d (卡需为 FAT32; exFAT 不支持)\r\n", (int)fr);
        return -3;
    }

    sdlog_speed_test();

    if (sdlog_new_file() != 0) {
        printf("[SDLOG] 无法创建日志文件\r\n");
        return -4;
    }

    memset(&s_hdr, 0, sizeof(s_hdr));
    s_head_blk = 0;
    s_wr_cnt   = 0;
    s_tail_blk = 0;
    s_last_sync_ms = HAL_GetTick();
    s_inited   = 1;
    s_st.mounted = 1;
    s_st.logging = 1;

    sdlog_write_header();          /* 先写一次头, 保证断电后文件可识别 */
    printf("[SDLOG] 开始采集 → %s (记录 %u B, 块 %u 记录, %u 块缓冲)\r\n",
           s_st.fname, (unsigned)SDLOG_REC_BYTES, (unsigned)SDLOG_BLK_RECS,
           (unsigned)SDLOG_BLK_COUNT);
    return 0;
}

/* ---------------- ISR: 入队 ---------------- */
void sdlog_push(const ReportFrame_t *f)
{
    if (f == NULL || !s_st.logging) {
        return;
    }

    if (s_wr_cnt >= SDLOG_BLK_RECS) {
        uint32_t next = (s_head_blk + 1u) % SDLOG_BLK_COUNT;
        if (next == s_tail_blk) {          /* 环形满(SD 停摆) → 丢弃并计数 */
            s_st.rec_dropped++;
            return;
        }
        s_head_blk = next;
        s_wr_cnt   = 0;
    }

    memcpy(BLK(s_head_blk) + (uint32_t)s_wr_cnt * SDLOG_REC_BYTES, f, SDLOG_REC_BYTES);
    s_wr_cnt++;
    s_st.rec_pushed++;

    if (s_st.rec_pushed == 1u) {
        s_ts_first = f->header.timestamp;
    }
    s_ts_last = f->header.timestamp;
}

/* ---------------- 主循环: 写出 + 周期同步 ---------------- */
void sdlog_task(void)
{
    if (!s_st.logging) {
        return;
    }

    /* 1) 写出所有已满块 (tail 追 head; head 块未满不写) */
    uint32_t guard = 0;
    while (s_tail_blk != s_head_blk && guard++ < SDLOG_BLK_COUNT) {
        UINT bw = 0;
        uint32_t off = SDLOG_HDR_BYTES + (uint32_t)(s_st.rec_flushed * SDLOG_REC_BYTES);

        if (f_lseek(&s_fil, off) != FR_OK) {
            s_st.write_err++;
            break;
        }
        if (f_write(&s_fil, BLK(s_tail_blk), SDLOG_BLK_BYTES, &bw) != FR_OK
            || bw != SDLOG_BLK_BYTES) {
            s_st.write_err++;
            break;                          /* 不推进 tail, 下一轮重试 */
        }
        s_st.blk_flushed++;
        s_st.rec_flushed += SDLOG_BLK_RECS;
        s_tail_blk = (s_tail_blk + 1u) % SDLOG_BLK_COUNT;
    }

    /* 2) 周期回写文件头 + f_sync (断电保护的唯一手段) */
    uint32_t now = HAL_GetTick();
    if ((now - s_last_sync_ms) >= SDLOG_SYNC_MS) {
        s_last_sync_ms = now;
        sdlog_write_header();
    }
}

void sdlog_get_stat(sdlog_stat_t *out)
{
    if (out) {
        *out = s_st;
    }
}
