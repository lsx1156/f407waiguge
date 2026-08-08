#include "interpolation.h"
#include <stdio.h>

#if LZ4_TABLES_USE_COMPRESSION
#include "lz4.h"
#include "lz4_tables_data.h"
#endif

DampingTable_t g_damping_table = {0};
FrictionTable_t g_friction_table = {0};
PIDParams_t g_pid_params[PID_PARAM_SET_COUNT] = {0};

static int32_t g_vel_max_mdeg_s = 1000000;

/* LZ4 解压入口: 当表格较大时, 从上电 Flash 中的压缩数据解压到外部 SRAM
 * 当前表小 (2KB), LZ4_TABLES_USE_COMPRESSION=0, 此函数为空操作 */
static int table_decompress_init(void)
{
#if LZ4_TABLES_USE_COMPRESSION
    int32_t *fric_buf = (int32_t *)MEM_TABLE_ADDR;
    int32_t *damp_buf = (int32_t *)(MEM_TABLE_ADDR + (FRICTION_POINTS * 4));

    /* 1. 解压到外部 SRAM 的临时工作区 */
    int decompressed = LZ4_decompress_safe(
        (const char *)lz4_tables_data,
        (char *)MEM_TABLE_ADDR,
        LZ4_TABLES_COMPRESSED_SIZE,
        LZ4_TABLES_ORIGINAL_SIZE
    );

    if (decompressed != LZ4_TABLES_ORIGINAL_SIZE) {
        return -1;  /* 解压失败: 数据损坏或 CRC 不匹配 */
    }

    /* 2. 校验 CRC32 (防止 Flash 位翻转) */
    /* CRC32 校验可在此处添加, 参考 zlib crc32() */

    /* 3. Delta 解码 (还原原始值) */
    for (int i = 1; i < FRICTION_POINTS; i++) {
        fric_buf[i] += fric_buf[i - 1];
    }
    for (int i = 1; i < DAMPING_POINTS; i++) {
        damp_buf[i] += damp_buf[i - 1];
    }

    /* 4. 将外部 SRAM 的表数据同步到 g_friction_table / g_damping_table */
    /* 未来: 直接将指针指向外部 SRAM, 省掉拷贝. 当前表小, 直接拷贝. */
    for (int i = 0; i < FRICTION_TABLE_SIZE && i < FRICTION_POINTS; i++) {
        g_friction_table.friction[i] = fric_buf[i];
    }
    for (int i = 0; i < DAMPING_TABLE_SIZE && i < DAMPING_POINTS; i++) {
        g_damping_table.damping[i] = damp_buf[i];
    }

    return 0;
#else
    return 0;  /* LZ4 未启用, 跳过 */
#endif
}

void interpolation_init(void)
{
    for (int i = 0; i < DAMPING_TABLE_SIZE; i++) {
        g_damping_table.vel[i] = (i * g_vel_max_mdeg_s) / (DAMPING_TABLE_SIZE - 1);
        g_damping_table.damping[i] = 0;
    }
    for (int i = 0; i < FRICTION_TABLE_SIZE; i++) {
        g_friction_table.vel[i] = (i * g_vel_max_mdeg_s) / (FRICTION_TABLE_SIZE - 1);
        g_friction_table.friction[i] = 0;
    }
    for (int i = 0; i < PID_PARAM_SET_COUNT; i++) {
        g_pid_params[i].kp = 10;
        g_pid_params[i].kd = 2;
        g_pid_params[i].torque_limit = 10000;
        g_pid_params[i].velocity_limit = 30000;  /* v1.6.9fix2: 1000(1°/s)太低, 步态/回归极慢 */
        g_pid_params[i].dead_zone = 0;
        g_pid_params[i].reserved = 0;
    }

    /* LZ4 解压入口: 表大时覆盖上面的零初始化 */
    int ret = table_decompress_init();

    /* v1.5: 校验表格地址和数据, 确认 SRAM 布局正确 */
    {
        int32_t *p = (int32_t *)MEM_TABLE_ADDR;
        printf("[INTERP] TABLE_ADDR=0x%08lX size=%uKB LZ4=%s ret=%d\r\n",
               (unsigned long)MEM_TABLE_ADDR,
               (unsigned)(MEM_TABLE_SIZE / 1024),
               LZ4_TABLES_USE_COMPRESSION ? "ON" : "OFF",
               ret);
        printf("[INTERP] First4: %d %d %d %d\r\n", p[0], p[1], p[2], p[3]);
    }
}

static int32_t linear_interpolate(int32_t x, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    if (x1 == x0) return y0;
    int64_t dy = (int64_t)(y1 - y0) * (x - x0);
    return y0 + (int32_t)(dy / (x1 - x0));
}

int32_t lookup_damping(int32_t velocity)
{
    int32_t abs_vel = (velocity < 0) ? -velocity : velocity;
    int32_t damping_val;

    if (abs_vel >= g_damping_table.vel[DAMPING_TABLE_SIZE - 1]) {
        damping_val = g_damping_table.damping[DAMPING_TABLE_SIZE - 1];
    } else {
        damping_val = 0;
        for (int i = 1; i < DAMPING_TABLE_SIZE; i++) {
            if (abs_vel <= g_damping_table.vel[i]) {
                damping_val = linear_interpolate(abs_vel,
                                      g_damping_table.vel[i - 1], g_damping_table.damping[i - 1],
                                      g_damping_table.vel[i], g_damping_table.damping[i]);
                break;
            }
        }
    }

    /* ★ v1.6.9fix: 恢复速度方向符号 (与 lookup_friction 一致)
     * 阻尼补偿是帮助电机克服自身粘性阻力, 方向与速度同向:
     *   velocity > 0 → +damping (帮助正向运动)
     *   velocity < 0 → -damping (帮助负向运动)
     * 之前始终返回正值 → 负速度时正反馈加速 */
    if (velocity > 0) return damping_val;
    if (velocity < 0) return -damping_val;
    return 0;
}

int32_t lookup_friction(int32_t velocity)
{
    int32_t abs_vel = (velocity < 0) ? -velocity : velocity;
    int32_t friction_val;

    if (abs_vel >= g_friction_table.vel[FRICTION_TABLE_SIZE - 1]) {
        friction_val = g_friction_table.friction[FRICTION_TABLE_SIZE - 1];
    } else {
        friction_val = 0;
        for (int i = 1; i < FRICTION_TABLE_SIZE; i++) {
            if (abs_vel <= g_friction_table.vel[i]) {
                friction_val = linear_interpolate(abs_vel,
                                                   g_friction_table.vel[i - 1], g_friction_table.friction[i - 1],
                                                   g_friction_table.vel[i], g_friction_table.friction[i]);
                break;
            }
        }
    }

    if (velocity > 0) return friction_val;
    if (velocity < 0) return -friction_val;
    return 0;
}

int32_t trapezoidal_interpolate(int32_t current, int32_t target, int32_t max_rate, int32_t period_ms)
{
    int32_t diff = target - current;
    int32_t max_step = (max_rate * period_ms + 500) / 1000;

    if (diff > max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
}

int32_t torque_rate_limit(int32_t current, int32_t target, int32_t rate_limit, int32_t period_ms)
{
    int32_t diff = target - current;
    int32_t max_step = (rate_limit * period_ms + 500) / 1000;

    if (diff > max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
}
