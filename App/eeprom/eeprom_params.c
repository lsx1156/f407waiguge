#include "eeprom_params.h"
#include "eeprom.h"
#include "control_isr.h"
#include "interpolation.h"
#include <string.h>

/* ========== Internal helpers ========== */

static uint16_t calc_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* Write header (magic + version + CRC) with blocking API (boot-time only) */
static void write_header_blocking(void)
{
    uint8_t buf[4];
    buf[0] = EEP_MAGIC_VALUE;
    buf[1] = EEP_VERSION_CURRENT;

    /* Calc CRC of everything after header (0x04 ~ 0xFF) */
    uint8_t scratch[252];
    eeprom_read_buffer(0x04, scratch, sizeof(scratch));
    uint16_t crc = calc_crc16(scratch, sizeof(scratch));
    buf[2] = (uint8_t)(crc & 0xFF);
    buf[3] = (uint8_t)((crc >> 8) & 0xFF);

    eeprom_write_byte(EEP_MAGIC_ADDR, buf[0]);
    eeprom_write_byte(EEP_VERSION_ADDR, buf[1]);
    eeprom_write_byte(EEP_CRC_ADDR, buf[2]);
    eeprom_write_byte(EEP_CRC_ADDR + 1, buf[3]);
}

/* Read and verify header. Returns 1 if valid, 0 if blank/corrupt. */
static uint8_t read_and_verify_header(void)
{
    uint8_t magic = eeprom_read_byte(EEP_MAGIC_ADDR);
    if (magic != EEP_MAGIC_VALUE) return 0;

    uint8_t version = eeprom_read_byte(EEP_VERSION_ADDR);
    if (version != EEP_VERSION_CURRENT) return 0;

    uint16_t stored_crc = (uint16_t)eeprom_read_byte(EEP_CRC_ADDR)
                          | ((uint16_t)eeprom_read_byte(EEP_CRC_ADDR + 1) << 8);

    uint8_t scratch[252];
    eeprom_read_buffer(0x04, scratch, sizeof(scratch));
    uint16_t calc_crc = calc_crc16(scratch, sizeof(scratch));

    return (stored_crc == calc_crc) ? 1 : 0;
}

/* ========== Public API ========== */

uint8_t eeprom_params_load_all(void)
{
    /* --- Check if EEPROM has been initialized --- */
    if (!read_and_verify_header()) {
        /* First boot or corrupt: set defaults, write to EEPROM */

        /* ABO: all joints use defaults from contract.h, bias=0 */
        for (int i = 0; i < 6; i++) {
            abo_set_param(i,
                ABO_DEFAULT_GAIN_Q10,
                ABO_DEFAULT_ALPHA_Q16,
                ABO_DEFAULT_LEAK_Q16,
                0);  /* 默认关闭 ABO, RK 下发后开启 */
            abo_set_bias(i, 0);
        }

        /* PID defaults already set in interpolation_init(), just sync first set to EEPROM */
        /* Write header last (it triggers CRC calc of current EEPROM content) */
        write_header_blocking();
        return 0;
    }

    /* --- Load ABO shared defaults --- */
    EEP_ABOShared_t shared;
    eeprom_read_buffer(EEP_ABO_SHARED_ADDR, (uint8_t *)&shared, sizeof(shared));

    for (int i = 0; i < 6; i++) {
        uint8_t en = (shared.enable_mask >> i) & 0x01;
        abo_set_param(i, shared.assist_gain_q10,
                         shared.hpf_alpha_q16,
                         shared.bias_leak_q16,
                         en);
    }

    /* --- Load per-joint ABO bias_est --- */
    for (int i = 0; i < 6; i++) {
        int32_t bias;
        eeprom_read_buffer(EEP_ABO_BIAS_ADDR + i * EEP_ABO_BIAS_PER_JOINT,
                           (uint8_t *)&bias, sizeof(bias));
        abo_set_bias(i, bias);
    }

    /* --- Load PID default params (set 0) --- */
    PIDParams_t pid_def;
    eeprom_read_buffer(EEP_PID_DEFAULT_ADDR, (uint8_t *)&pid_def, sizeof(pid_def));
    g_pid_params[0] = pid_def;

    return 0;
}

uint8_t eeprom_params_save_all(void)
{
    /* --- Pack ABO shared defaults --- */
    EEP_ABOShared_t shared = {0};
    shared.assist_gain_q10 = ABO_DEFAULT_GAIN_Q10;
    shared.hpf_alpha_q16   = ABO_DEFAULT_ALPHA_Q16;
    shared.bias_leak_q16   = ABO_DEFAULT_LEAK_Q16;
    shared.enable_mask     = 0;

    for (int i = 0; i < 6; i++) {
        ABOState_t st;
        if (abo_get_param(i, &st)) {
            shared.assist_gain_q10 = st.assist_gain_q10;
            shared.hpf_alpha_q16   = st.hpf_alpha_q16;
            shared.bias_leak_q16   = st.bias_leak_q16;
            if (st.enable) shared.enable_mask |= (1U << i);
        }
    }

    eeprom_write_buffer(EEP_ABO_SHARED_ADDR, (uint8_t *)&shared, sizeof(shared));

    /* --- Save per-joint ABO bias_est --- */
    for (int i = 0; i < 6; i++) {
        ABOState_t st;
        if (abo_get_param(i, &st)) {
            eeprom_write_buffer(EEP_ABO_BIAS_ADDR + i * EEP_ABO_BIAS_PER_JOINT,
                                 (uint8_t *)&st.bias_est, sizeof(st.bias_est));
        }
    }

    /* --- Save PID default (set 0) --- */
    eeprom_write_buffer(EEP_PID_DEFAULT_ADDR, (uint8_t *)&g_pid_params[0],
                         EEP_PID_DEFAULT_SIZE);

    /* Header + CRC written last (after all writes flushed by state machine)
     * Caller should invoke eeprom_process_write_buffer() periodically */
    return 0;
}

uint8_t eeprom_params_save_abo_bias(void)
{
    for (int i = 0; i < 6; i++) {
        ABOState_t st;
        if (abo_get_param(i, &st)) {
            eeprom_write_buffer(EEP_ABO_BIAS_ADDR + i * EEP_ABO_BIAS_PER_JOINT,
                                 (uint8_t *)&st.bias_est, sizeof(st.bias_est));
        }
    }
    return 0;
}

/* ========== P2-4: 温度零漂系数表读写 (每关节 21 个 float) ==========
 * EEPROM 容量约束: AT24C02 仅 256 字节, Reserved 区 (0x3C~0xFF, 196B) 只够存
 *                 idx 0,1 两关节 (每关节 84B), 故 EEP_TEMP_COEFF_MAX_JOINTS=2.
 * 异步落盘约束:   eeprom_write_buffer 仅入队单缓冲, 调用者需周期调
 *                 eeprom_process_write_buffer() 落盘, 且两次 write 之间必须等
 *                 上一次落盘完成 (与 eeprom_params_save_all 一致). */

uint8_t eeprom_write_temp_coeff(uint8_t idx, const float coeff[21])
{
    /* idx 超出 EEPROM 容量 → 拒绝 */
    if (idx >= EEP_TEMP_COEFF_MAX_JOINTS) return 1u;

    uint16_t addr = (uint16_t)(EEP_TEMP_COEFF_ADDR_BASE + idx * EEP_TEMP_COEFF_PER_JOINT);
    /* 异步入队: 调用者须后续周期调 eeprom_process_write_buffer() 落盘,
     * 且本调用与上一次 write 之间需确保已落盘 (单缓冲限制). */
    eeprom_write_buffer(addr, (uint8_t *)coeff, EEP_TEMP_COEFF_PER_JOINT);
    return 0u;
}

uint8_t eeprom_read_temp_coeff(uint8_t idx, float coeff[21])
{
    /* idx 超出 EEPROM 容量 → 拒绝 */
    if (idx >= EEP_TEMP_COEFF_MAX_JOINTS) return 1u;

    uint16_t addr = (uint16_t)(EEP_TEMP_COEFF_ADDR_BASE + idx * EEP_TEMP_COEFF_PER_JOINT);
    /* 同步读取: eeprom_read_buffer 内部按字节阻塞读, 上电加载时调用 */
    eeprom_read_buffer(addr, (uint8_t *)coeff, EEP_TEMP_COEFF_PER_JOINT);
    return 0u;
}
