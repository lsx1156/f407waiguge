#include "udp_protocol.h"

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
    fifo->head = 0;
    fifo->tail = 0;
    fifo->size = size;
    fifo->item_size = item_size;
    fifo->buffer = buffer;
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
    if (fifo_is_full(fifo)) {
        return 0;
    }
    
    uint32_t index = fifo->head * fifo->item_size;
    for (uint32_t i = 0; i < fifo->item_size; i++) {
        fifo->buffer[index + i] = ((const uint8_t*)data)[i];
    }
    
    fifo->head = (fifo->head + 1) % fifo->size;
    return 1;
}

static uint8_t fifo_read(Fifo_t *fifo, void *data)
{
    if (fifo_is_empty(fifo)) {
        return 0;
    }
    
    uint32_t index = fifo->tail * fifo->item_size;
    for (uint32_t i = 0; i < fifo->item_size; i++) {
        ((uint8_t*)data)[i] = fifo->buffer[index + i];
    }
    
    fifo->tail = (fifo->tail + 1) % fifo->size;
    return 1;
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
    frame->header.frame_type = FRAME_TYPE_REPORT;
    frame->header.reserved = 0x00;
    frame->header.seq_num = g_report_seq_num++;
    frame->header.timestamp = HAL_GetTick();
    
    for (int i = 0; i < 2; i++) {
        frame->leg_status[i] = leg[i];
    }
    
    for (int i = 0; i < 4; i++) {
        frame->arm_status[i] = arm[i];
    }
    
    frame->weight_adc = g_weight_adc_value;
    frame->voltage_adc = g_voltage_adc_value;
    
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
uint16_t g_weight_adc_value = 0;
uint16_t g_voltage_adc_value = 0;
uint32_t g_report_seq_num = 0;

void udp_protocol_init(void)
{
    fifo_init(&g_report_fifo, g_report_buffer, FIFO_REPORT_SIZE, sizeof(ReportFrame_t));
    fifo_init(&g_command_fifo, g_command_buffer, FIFO_COMMAND_SIZE, sizeof(JointCommand_t) * 6);
    fifo_init(&g_fault_fifo, g_fault_buffer, FIFO_FAULT_SIZE, sizeof(uint16_t));
}
