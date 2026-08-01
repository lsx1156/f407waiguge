#include "udp_protocol.h"
#include "lwip/sys.h"

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

    for (int i = 0; i < 2; i++) {
        frame->leg_status[i] = leg[i];
    }

    for (int i = 0; i < 4; i++) {
        frame->arm_status[i] = arm[i];
    }

    frame->crc = crc16((uint8_t*)frame, sizeof(ReportFrame_t) - 2);
}

void command_frame_parse(CommandFrame_t *frame, JointCommand_t *commands)
{
    for (int i = 0; i < 6; i++) {
        commands[i] = frame->commands[i];
    }
}

uint8_t report_fifo_write(ReportFrame_t *frame)
{
    return fifo_write(&g_report_fifo, frame);
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

JointCommand_t g_active_command[6] = {0};
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

void comm_heartbeat_kick(void)
{
    g_last_comm_ts = HAL_GetTick();
}

uint8_t comm_is_heartbeat_ok(void)
{
    if (g_last_comm_ts == 0) return 0;
    return (HAL_GetTick() - g_last_comm_ts) < COMM_HEARTBEAT_TIMEOUT_MS;
}

void udp_protocol_init(void)
{
    fifo_init(&g_report_fifo, g_report_buffer, FIFO_REPORT_SIZE, sizeof(ReportFrame_t));
    fifo_init(&g_command_fifo, g_command_buffer, FIFO_COMMAND_SIZE, sizeof(JointCommand_t) * 6);
    fifo_init(&g_fault_fifo, g_fault_buffer, FIFO_FAULT_SIZE, sizeof(uint16_t));
}
