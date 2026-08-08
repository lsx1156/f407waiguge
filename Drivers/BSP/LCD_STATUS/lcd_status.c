#include <stdio.h>
#include <math.h>
#include "./BSP/LCD_STATUS/lcd_status.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/SYS_INFO/sys_info.h"
#include "./SYSTEM/delay/delay.h"
#include "can_motor.h"
#include "contract.h"
#include "control_isr.h"
#include "udp_net.h"
#include "mode_manager.h"
#include "safety.h"

/* Getters from main.c (basic mode) */
extern uint8_t  ctrl_get_mode(void);
extern uint8_t  ctrl_get_selected(void);
extern int32_t  ctrl_get_target_pos(void);
extern int32_t  ctrl_get_target_torque(void);
extern uint8_t  ctrl_get_pos_manual(void);

static uint32_t g_last_refresh = 0;
static uint8_t g_initialized = 0;

/* === 高对比度深色主题配色 === */
#define GUI_BG              0x0843
#define GUI_CARD_BG         0x10A2
#define GUI_CARD_BORDER     0x2965
#define GUI_TITLE_BG        0x2B5F
#define GUI_TITLE_FG        0xFFFF
#define GUI_LABEL_FG        0x9CD3
#define GUI_VALUE_FG        0xFFFF
#define GUI_ACCENT          0x5D7C
#define GUI_GREEN           0x2FC4
#define GUI_YELLOW          0xFF40
#define GUI_RED             0xF926
#define GUI_CYAN            0x7FFF
#define GUI_GRAY            0x4A89
#define GUI_DARK_GRAY       0x2104

/* === 布局参数 === */
#define SCREEN_W            lcddev.width
#define SCREEN_H            lcddev.height
#define PADDING             6
#define CARD_GAP            5
#define LABEL_W             70
#define BAR_X               (LABEL_W + PADDING * 2)
#define BAR_W               (SCREEN_W - LABEL_W - PADDING * 5 - 52)
#define BAR_H               12
#define VALUE_X             (SCREEN_W - PADDING - 50)

/* === 电机状态框参数 === */
#define MOTOR_BOX_W         22
#define MOTOR_BOX_H         14
#define MOTOR_BOX_GAP       4

static void draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    lcd_draw_rectangle(x, y, x + w - 1, y + h - 1, color);
}

static void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    lcd_fill(x, y, x + w - 1, y + h - 1, color);
}

static void draw_card(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    fill_rect(x, y, w, h, GUI_CARD_BG);
    fill_rect(x, y, w, 18, GUI_TITLE_BG);
    draw_rect(x, y, w, h, GUI_CARD_BORDER);
}

static void draw_progress_bar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               uint8_t percent, uint16_t bar_color)
{
    uint16_t fill_w;

    if (percent > 100) percent = 100;
    fill_w = (uint16_t)((uint32_t)w * percent / 100);

    draw_rect(x, y, w, h, GUI_GRAY);
    fill_rect(x + 1, y + 1, w - 2, h - 2, GUI_BG);

    if (fill_w > 2)
    {
        fill_rect(x + 2, y + 2, fill_w - 3, h - 4, bar_color);
    }
}

static uint16_t get_bar_color(uint8_t percent)
{
    if (percent >= 90) return GUI_RED;
    if (percent >= 70) return GUI_YELLOW;
    return GUI_GREEN;
}

static void format_size(char *buf, uint32_t bytes)
{
    if (bytes >= 1024 * 1024)
    {
        sprintf(buf, "%dMB", (int)(bytes / (1024 * 1024)));
    }
    else if (bytes >= 1024)
    {
        sprintf(buf, "%dKB", (int)(bytes / 1024));
    }
    else
    {
        sprintf(buf, "%dB", (int)bytes);
    }
}

static void format_uptime(char *buf, uint32_t ms)
{
    uint32_t sec = ms / 1000;
    uint32_t min = sec / 60;
    uint32_t hour = min / 60;
    sec %= 60; min %= 60; hour %= 24;
    sprintf(buf, "%02d:%02d:%02d", (int)hour, (int)min, (int)sec);
}

static void draw_text(uint16_t x, uint16_t y, uint16_t w, const char *str,
                      uint16_t fg, uint16_t bg)
{
    uint16_t old_back = g_back_color;
    g_back_color = bg;
    lcd_show_string(x, y, w, 16, 16, (char *)str, fg);
    g_back_color = old_back;
}

/**
 * @brief  Draw a motor status indicator box
 * @param  x, y: top-left position
 * @param  motor_id: motor ID number (displayed inside box)
 * @param  status: 0=offline(gray), 1=online(green), 2=error(red)
 */
static void draw_motor_box(uint16_t x, uint16_t y, uint8_t motor_id, uint8_t status)
{
    uint16_t bg_color, fg_color, border_color;

    switch (status)
    {
        case 1:  bg_color = GUI_GREEN;   fg_color = 0x0000; border_color = GUI_GREEN;   break;
        case 2:  bg_color = GUI_RED;     fg_color = 0xFFFF; border_color = GUI_RED;     break;
        default: bg_color = GUI_DARK_GRAY; fg_color = GUI_GRAY; border_color = GUI_GRAY; break;
    }

    fill_rect(x, y, MOTOR_BOX_W, MOTOR_BOX_H, bg_color);
    draw_rect(x, y, MOTOR_BOX_W, MOTOR_BOX_H, border_color);

    char id_str[4];
    sprintf(id_str, "%d", motor_id);
    uint16_t old_back = g_back_color;
    g_back_color = bg_color;
    lcd_show_string(x + 2, y - 1, MOTOR_BOX_W, 12, 12, (char *)id_str, fg_color);
    g_back_color = old_back;
}

void lcd_status_init(void)
{
    uint16_t card_y;

    g_back_color = GUI_BG;
    lcd_clear(GUI_BG);

    /* 顶部标题栏 */
    fill_rect(0, 0, SCREEN_W, 22, GUI_TITLE_BG);
    draw_rect(0, 0, SCREEN_W, 22, GUI_CARD_BORDER);
    draw_text(SCREEN_W / 2 - 48, 3, 96, "设备状态", GUI_TITLE_FG, GUI_TITLE_BG);

    /* 卡片1: CPU & Clock (精简: 只保留 CPU + CAN) */
    card_y = 28;
    draw_card(PADDING, card_y, SCREEN_W - PADDING * 2, 62);
    draw_text(PADDING + 6, card_y + 1, 120, "CPU / Clock", GUI_TITLE_FG, GUI_TITLE_BG);

    draw_text(PADDING + 6, card_y + 24, LABEL_W, "CPU:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6, card_y + 42, LABEL_W, "CAN:", GUI_LABEL_FG, GUI_CARD_BG);

    /* 卡片2: Memory (保持不变) */
    card_y = 28 + 62 + CARD_GAP;
    draw_card(PADDING, card_y, SCREEN_W - PADDING * 2, 80);
    draw_text(PADDING + 6, card_y + 1, 100, "Memory", GUI_TITLE_FG, GUI_TITLE_BG);

    draw_text(PADDING + 6, card_y + 22, LABEL_W, "Flash:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6, card_y + 40, LABEL_W, "IntRAM:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6, card_y + 58, LABEL_W, "ExtSRAM:", GUI_LABEL_FG, GUI_CARD_BG);

    /* 卡片3: Status (新布局: Uptime + CAN1电机 + CAN2电机 + 温度 + ATP) */
    card_y = 28 + 62 + CARD_GAP + 80 + CARD_GAP;
    uint16_t card3_h = SCREEN_H - card_y - PADDING;
    if (card3_h < 110) card3_h = 110;
    draw_card(PADDING, card_y, SCREEN_W - PADDING * 2, card3_h);
    draw_text(PADDING + 6, card_y + 1, 80, "Status", GUI_TITLE_FG, GUI_TITLE_BG);

    /* Status 内部布局 */
    uint16_t sy = card_y + 20;
    draw_text(PADDING + 6, sy, LABEL_W, "Uptime:", GUI_LABEL_FG, GUI_CARD_BG);
    sy += 16;
    draw_text(PADDING + 6, sy, LABEL_W, "Mode:", GUI_LABEL_FG, GUI_CARD_BG);
    sy += 16;
    draw_text(PADDING + 6, sy, LABEL_W, "CAN1:", GUI_LABEL_FG, GUI_CARD_BG);
    sy += 16;
    draw_text(PADDING + 6, sy, LABEL_W, "CAN2:", GUI_LABEL_FG, GUI_CARD_BG);
    sy += 16;
    draw_text(PADDING + 6, sy, LABEL_W, "Temp:", GUI_LABEL_FG, GUI_CARD_BG);
    sy += 16;
    draw_text(PADDING + 6, sy, LABEL_W, "ATP:", GUI_LABEL_FG, GUI_CARD_BG);

    g_initialized = 1;
    lcd_status_update();
}

void lcd_status_update(void)
{
    sys_info_t *info = sys_info_get();
    char buf[64];
    uint8_t percent;
    uint16_t bar_color;
    uint16_t card_y;

    if (!g_initialized) return;

    /* === 卡片1: CPU & CAN (卡片高度: 62) === */
    card_y = 28;

    /* CPU 进度条 + 百分比 */
    percent = info->cpu_usage;
    bar_color = get_bar_color(percent);
    draw_progress_bar(BAR_X, card_y + 24, BAR_W, BAR_H, percent, bar_color);
    sprintf(buf, "%3d%%", percent);
    draw_text(VALUE_X, card_y + 22, 50, buf, GUI_VALUE_FG, GUI_CARD_BG);

    /* CAN 总线使用率 (占位: --) */
    /* TODO: 后续接入 CAN 负载统计 */
    sprintf(buf, "CAN:%2d%%", 0);
    draw_text(BAR_X, card_y + 42, 80, buf, GUI_YELLOW, GUI_CARD_BG);

    /* === 卡片2: Memory (卡片高度: 80, 起始: 28+62+5=95) === */
    card_y = 28 + 62 + CARD_GAP;

    /* Flash */
    percent = (uint8_t)((uint32_t)info->flash.used * 100 / info->flash.total);
    bar_color = get_bar_color(percent);
    draw_progress_bar(BAR_X, card_y + 23, BAR_W, BAR_H, percent, bar_color);
    format_size(buf, info->flash.used);
    draw_text(VALUE_X - 6, card_y + 22, 56, buf, GUI_VALUE_FG, GUI_CARD_BG);

    /* IntRAM */
    percent = (uint8_t)((uint32_t)info->ram.used * 100 / info->ram.total);
    bar_color = get_bar_color(percent);
    draw_progress_bar(BAR_X, card_y + 41, BAR_W, BAR_H, percent, bar_color);
    format_size(buf, info->ram.used);
    draw_text(VALUE_X - 6, card_y + 40, 56, buf, GUI_VALUE_FG, GUI_CARD_BG);

    /* ExtSRAM */
    percent = (uint8_t)((uint32_t)info->sram.used * 100 / info->sram.total);
    bar_color = get_bar_color(percent);
    draw_progress_bar(BAR_X, card_y + 59, BAR_W, BAR_H, percent, bar_color);
    format_size(buf, info->sram.used);
    draw_text(VALUE_X - 6, card_y + 58, 56, buf, GUI_VALUE_FG, GUI_CARD_BG);

    /* === 卡片3: Status (起始: 28+62+5+80+5=180) === */
    card_y = 28 + 62 + CARD_GAP + 80 + CARD_GAP;
    uint16_t sy = card_y + 20;

    /* Uptime */
    format_uptime(buf, info->uptime_ms);
    draw_text(BAR_X, sy, 100, buf, GUI_ACCENT, GUI_CARD_BG);
    sy += 16;

    /* v1.6.2: 当前模式 + 子模式 */
    {
        const char *main_name = mode_get_main_name();
        char mode_buf[24];
        if (mode_get_main() == MODE_GAIT) {
            snprintf(mode_buf, sizeof(mode_buf), "%s:%s", main_name, mode_get_gait_sub_name());
        } else if (mode_get_main() == MODE_ARM_ASSIST) {
            snprintf(mode_buf, sizeof(mode_buf), "%s:%s", main_name, mode_get_arm_level_name());
        } else {
            snprintf(mode_buf, sizeof(mode_buf), "%s ABO:%s", main_name,
                     mode_get_abo_enabled() ? "ON" : "OFF");
        }
        draw_text(BAR_X, sy, 120, mode_buf, GUI_CYAN, GUI_CARD_BG);
    }
    sy += 16;

    /* CAN1 电机: 2 个状态框 (腿部 ID=1,2) */
    {
        uint16_t bx = BAR_X;
        draw_motor_box(bx, sy + 1, 1, can_motor_is_online(0) ? 1 : 0); bx += MOTOR_BOX_W + MOTOR_BOX_GAP;
        draw_motor_box(bx, sy + 1, 2, can_motor_is_online(1) ? 1 : 0);
    }
    sy += 16;

    /* CAN2 电机: 4 个状态框 (手臂 ID=16,17,18,19) */
    {
        uint16_t bx = BAR_X;
        draw_motor_box(bx, sy + 1, 16, can_motor_is_online(2) ? 1 : 0); bx += MOTOR_BOX_W + MOTOR_BOX_GAP;
        draw_motor_box(bx, sy + 1, 17, can_motor_is_online(3) ? 1 : 0); bx += MOTOR_BOX_W + MOTOR_BOX_GAP;
        draw_motor_box(bx, sy + 1, 18, can_motor_is_online(4) ? 1 : 0); bx += MOTOR_BOX_W + MOTOR_BOX_GAP;
        draw_motor_box(bx, sy + 1, 19, can_motor_is_online(5) ? 1 : 0);
    }
    sy += 16;

    /* 环境温度 (占位: --) */
    draw_text(BAR_X, sy, 80, "-- C", GUI_VALUE_FG, GUI_CARD_BG);
    sy += 16;

    /* ATP: 系统状态 (error/stop/run)
     * v1.6.3: STANDALONE 模式下屏蔽通信故障, 与 safety 逻辑保持一致
     */
    {
        const char *sys_status;
        uint16_t status_color;
        uint32_t faults = safety_get_faults();
        /* STANDALONE 模式下忽略通信相关故障 */
        if (g_comm_mode == COMM_MODE_STANDALONE) {
            faults &= ~(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
        }
        if (faults != 0) {
            sys_status   = "ERROR";
            status_color = GUI_RED;
        } else if (mode_get_main() == MODE_ZERO_TORQUE) {
            sys_status   = "STOP";
            status_color = GUI_YELLOW;
        } else {
            sys_status   = "RUN";
            status_color = GUI_GREEN;
        }
        draw_text(BAR_X, sy, 100, sys_status, status_color, GUI_CARD_BG);
    }
}

void lcd_status_task(void)
{
    uint32_t now = HAL_GetTick();

    if (!g_initialized)
    {
        lcd_status_init();
    }

    if (now - g_last_refresh >= LCD_STATUS_REFRESH_MS)
    {
        g_last_refresh = now;
        sys_info_update();
        lcd_status_update();
    }
}
