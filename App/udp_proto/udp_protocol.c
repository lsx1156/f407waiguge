#include "udp_protocol.h"
#include "lwip/sys.h"
#include "safety.h"  /* g_safety_state.fault_code, g_comm_mode */

typedef struct {
    volatile uint32_t head;
    volatile uint32_t tail;
    uint32_t size;
    uint32_t item_size;
    uint8_t *buffer;
} Fifo_t;

static Fifo_t g_report_fifo;
static Fifo_t g_command_fifo;
static Fifo_t g_fault_fifo;

static uint8_t g_report_buffer[FIFO_REPORT_SIZE * sizeof(ReportFrame_t)];
static uint8_t g_command_buffer[FIFO_COMMAND_SIZE * sizeof(JointCommand_t) * 6];
static uint8_t g_fault_buffer[FIFO_FAULT_SIZE * sizeof(uint16_t)];

static void fifo_init(Fifo_t *fifo, uint8_t *buffer, uint32_t size, uint32_t item_size)
{
    sys_prot_t lev = sys_arch_protect();
    fifo->head = 0;
    fifo->tail = 0;
    fifo->size = size;
    fifo->item_size = item_size;
    fifo->buffer = buffer;
    sys_arch_unprotect(lev);
}

static uint8_t fifo_is_full(Fifo_t *fifo)
{
    return ((fifo->head + 1) % fifo->size) == fifo->tail;
}

static uint8_t fifo_is_empty(Fifo_t *fifo)
{
    return fifo->head == fifo->tail;
}

static uint8_t fifo_write(Fifo_t *fifo, const void *data)
{
    sys_prot_t lev;
    uint32_t index;
    uint8_t ret = 0;

    lev = sys_arch_protect();
    if (!fifo_is_full(fifo)) {
        index = fifo->head * fifo->item_size;
        for (uint32_t i = 0; i < fifo->item_size; i++) {
            fifo->buffer[index + i] = ((const uint8_t*)data)[i];
        }
        fifo->head = (fifo->head + 1) % fifo->size;
        ret = 1;
    }
    sys_arch_unprotect(lev);

    return ret;
}

static uint8_t fifo_read(Fifo_t *fifo, void *data)
{
    sys_prot_t lev;
    uint32_t index;
    uint8_t ret = 0;

    lev = sys_arch_protect();
    if (!fifo_is_empty(fifo)) {
        index = fifo->tail * fifo->item_size;
        for (uint32_t i = 0; i < fifo->item_size; i++) {
            ((uint8_t*)data)[i] = fifo->buffer[index + i];
        }
        fifo->tail = (fifo->tail + 1) % fifo->size;
        ret = 1;
    }
    sys_arch_unprotect(lev);

    return ret;
}

uint16_t crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;
    uint32_t i, j;

    for (i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8; j++) {
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

void report_frame_build(ReportFrame_t *frame, JointStatus_t *leg, JointStatus_t *arm)
{
    sys_prot_t lev;

    frame->header.frame_type = FRAME_TYPE_REPORT;
    frame->header.reserved = 0x00;

    lev = sys_arch_protect();
    frame->header.seq_num = g_report_seq_num++;
    sys_arch_unprotect(lev);

    frame->header.timestamp = HAL_GetTick();

    /* v1.6.3: 统一故障码策略 (按总线分别应用 CANx_TIMEOUT)
     *   - 每关节 fault_code 存系统级 g_safety_state.fault_code
     *   - 腿部(CAN1 CyberGear): 只受 CAN1_TIMEOUT 影响, 屏蔽 CAN2_TIMEOUT
     *   - 臂部(CAN2 RS01):     只受 CAN2_TIMEOUT 影响, 屏蔽 CAN1_TIMEOUT
     *   - STANDALONE 模式下额外屏蔽 COMM_LOST, 与 safety / lcd_status 逻辑保持一致
     * 原实现: leg[i].fault_code 存 CyberGear 电机内部故障寄存器/0 (RS01), 与上位机故障码字典不兼容,
     *         导致 ESTOP/VOLTAGE_LOW/CANx_TIMEOUT 全误报, 且 CAN2 不接电机时腿部也显示 CAN2_TIMEOUT */
    SAFETY_LOCK();
    uint16_t sys_faults_all = (uint16_t)g_safety_state.fault_code;
    /* 腿部 (CAN1): 仅应用 CAN1_TIMEOUT, 屏蔽 CAN2_TIMEOUT */
    uint16_t leg_faults = sys_faults_all & ~(uint16_t)FAULT_CAN2_TIMEOUT;
    /* 臂部 (CAN2): 仅应用 CAN2_TIMEOUT, 屏蔽 CAN1_TIMEOUT */
    uint16_t arm_faults = sys_faults_all & ~(uint16_t)FAULT_CAN1_TIMEOUT;
    if (g_comm_mode == COMM_MODE_STANDALONE) {
        leg_faults &= ~(uint16_t)(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
        arm_faults &= ~(uint16_t)(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
    }
    SAFETY_UNLOCK();

    for (int i = 0; i < 2; i++) {
        frame->leg_status[i] = leg[i];
        frame->leg_status[i].fault_code = leg_faults;
    }

    for (int i = 0; i < 4; i++) {
        frame->arm_status[i] = arm[i];
        frame->arm_status[i].fault_code = arm_faults;
    }

    frame->crc = crc16((uint8_t*)frame, sizeof(ReportFrame_t) - 2);
}

void command_frame_parse(CommandFrame_t *frame, JointCommand_t *commands)
{
    for (int i = 0; i < 6; i++) {
        commands[i] = frame->commands[i];
    }
}

/* 调试计数器 (在主循环中打印) */
volatile uint32_t g_dbg_isr_calls = 0;
volatile uint32_t g_dbg_fifo_write_ok = 0;
volatile uint32_t g_dbg_fifo_write_fail = 0;

uint8_t report_fifo_write(ReportFrame_t *frame)
{
    uint8_t ret = fifo_write(&g_report_fifo, frame);
    if (ret) g_dbg_fifo_write_ok++; else g_dbg_fifo_write_fail++;
    return ret;
}

uint8_t command_fifo_write(JointCommand_t *commands)
{
    return fifo_write(&g_command_fifo, commands);
}

uint8_t fault_fifo_write(uint16_t fault_code)
{
    return fifo_write(&g_fault_fifo, &fault_code);
}

uint8_t report_fifo_read(ReportFrame_t *frame)
{
    return fifo_read(&g_report_fifo, frame);
}

uint8_t command_fifo_read(JointCommand_t *commands)
{
    return fifo_read(&g_command_fifo, commands);
}

uint8_t fault_fifo_read(uint16_t *fault_code)
{
    return fifo_read(&g_fault_fifo, fault_code);
}

/* v1.6.3+fix: 默认全力矩模式 (CTRL_MODE_TORQUE=1) 零力矩, 防止上电默认POSITION(0)kp=10闭环拉回0→手动转动过流Error
 * 关节ID: 0x01左髋,0x02右髋,0x10~0x13臂1~臂4 */
JointCommand_t g_active_command[6] = {
    {0x01, 1, 0, 60000, 20000, 50000, 0, 0, 0, 1, 1024, 6553, 131},  /* 腿0: 左髋, TORQUE, 零力矩 */
    {0x02, 1, 0, 60000, 20000, 50000, 0, 0, 0, 1, 1024, 6553, 131},  /* 腿1: 右髋, TORQUE, 零力矩 */
    {0x10, 1, 0, 60000, 20000, 50000, 0, 0, 0, 1, 1024, 6553, 131},  /* 臂0: Arm-1 */
    {0x11, 1, 0, 60000, 20000, 50000, 0, 0, 0, 1, 1024, 6553, 131},  /* 臂1: Arm-2 */
    {0x12, 1, 0, 60000, 20000, 50000, 0, 0, 0, 1, 1024, 6553, 131},  /* 臂2: Arm-3 */
    {0x13, 1, 0, 60000, 20000, 50000, 0, 0, 0, 1, 1024, 6553, 131},  /* 臂3: Arm-4 */
};
uint32_t g_report_seq_num = 0;

static SafeCmd_t g_cmd_buf[2] = {0};
static volatile uint8_t g_cmd_idx = 0;
static uint8_t g_cmd_version = 0;

void safe_cmd_write(JointCommand_t *new_cmd)
{
    uint8_t next = g_cmd_idx ^ 1;
    SafeCmd_t *dst = &g_cmd_buf[next];
    for (int i = 0; i < 6; i++) dst->cmd[i] = new_cmd[i];
    dst->crc = crc16((uint8_t*)dst->cmd, sizeof(dst->cmd));
    dst->version = ++g_cmd_version;
    __DMB();
    g_cmd_idx = next;
}

uint8_t safe_cmd_read(JointCommand_t *out_cmd)
{
    uint8_t idx = g_cmd_idx;
    SafeCmd_t *src = &g_cmd_buf[idx];
    uint32_t calc_crc = crc16((uint8_t*)src->cmd, sizeof(src->cmd));
    if (calc_crc != src->crc) {
        return 0;
    }
    for (int i = 0; i < 6; i++) out_cmd[i] = src->cmd[i];
    return 1;
}

uint32_t safe_cmd_get_version(void)
{
    return g_cmd_buf[g_cmd_idx].version;
}

volatile uint32_t g_last_comm_ts = 0;
volatile CommMode_e g_comm_mode = COMM_MODE_HOST;  /* 默认上位机模式 */

void comm_heartbeat_kick(void)
{
    g_last_comm_ts = HAL_GetTick();
}

uint8_t comm_is_heartbeat_ok(void)
{
    /* STANDALONE 模式下: 心跳超时不判故障，直接返回"真" */
    if (g_comm_mode == COMM_MODE_STANDALONE) return 1;

    if (g_last_comm_ts == 0) return 0;
    return (HAL_GetTick() - g_last_comm_ts) < COMM_HEARTBEAT_TIMEOUT_MS;
}

void udp_protocol_init(void)
{
    fifo_init(&g_report_fifo, g_report_buffer, FIFO_REPORT_SIZE, sizeof(ReportFrame_t));
    fifo_init(&g_command_fifo, g_command_buffer, FIFO_COMMAND_SIZE, sizeof(JointCommand_t) * 6);
    fifo_init(&g_fault_fifo, g_fault_buffer, FIFO_FAULT_SIZE, sizeof(uint16_t));
}
