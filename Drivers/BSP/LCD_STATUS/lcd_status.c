#include <stdio.h>
#include "./BSP/LCD_STATUS/lcd_status.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/SYS_INFO/sys_info.h"
#include "./SYSTEM/delay/delay.h"

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

/* === 布局参数 === */
#define SCREEN_W            lcddev.width
#define SCREEN_H            lcddev.height
#define PADDING             6
#define CARD_GAP            5
#define LABEL_W             82
#define BAR_X               (LABEL_W + PADDING * 2)
#define BAR_W               (SCREEN_W - LABEL_W - PADDING * 5 - 52)
#define BAR_H               14
#define VALUE_X             (SCREEN_W - PADDING - 50)

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
    lcd_show_string(x, y, w, 16, 16, (uint8_t *)str, fg);
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
    draw_text(SCREEN_W / 2 - 60, 3, 120, "System Monitor", GUI_TITLE_FG, GUI_TITLE_BG);

    /* 卡片1: CPU & Clock */
    card_y = 28;
    draw_card(PADDING, card_y, SCREEN_W - PADDING * 2, 94);
    draw_text(PADDING + 6, card_y + 1, 120, "CPU / Clock", GUI_TITLE_FG, GUI_TITLE_BG);

    draw_text(PADDING + 6, card_y + 24, LABEL_W, "CPU:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6, card_y + 42, LABEL_W, "SYSCLK:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6, card_y + 60, LABEL_W, "HCLK:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6, card_y + 78, LABEL_W, "PCLK1:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6 + 160, card_y + 42, LABEL_W, "PCLK2:", GUI_LABEL_FG, GUI_CARD_BG);

    /* 卡片2: Memory */
    card_y = 28 + 94 + CARD_GAP;
    draw_card(PADDING, card_y, SCREEN_W - PADDING * 2, 92);
    draw_text(PADDING + 6, card_y + 1, 100, "Memory", GUI_TITLE_FG, GUI_TITLE_BG);

    draw_text(PADDING + 6, card_y + 24, LABEL_W, "Flash:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6, card_y + 44, LABEL_W, "IntRAM:", GUI_LABEL_FG, GUI_CARD_BG);
    draw_text(PADDING + 6, card_y + 64, LABEL_W, "ExtSRAM:", GUI_LABEL_FG, GUI_CARD_BG);

    /* 卡片3: Status */
    card_y = 28 + 94 + CARD_GAP + 92 + CARD_GAP;
    draw_card(PADDING, card_y, SCREEN_W - PADDING * 2, 34);
    draw_text(PADDING + 6, card_y + 1, 80, "Status", GUI_TITLE_FG, GUI_TITLE_BG);

    draw_text(PADDING + 6, card_y + 20, LABEL_W, "Uptime:", GUI_LABEL_FG, GUI_CARD_BG);

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

    /* === 卡片1: CPU & Clock === */
    card_y = 28;

    /* CPU 进度条 + 百分比 */
    percent = info->cpu_usage;
    bar_color = get_bar_color(percent);
    draw_progress_bar(BAR_X, card_y + 25, BAR_W, BAR_H, percent, bar_color);
    sprintf(buf, "%3d%%", percent);
    draw_text(VALUE_X, card_y + 24, 50, buf, GUI_VALUE_FG, GUI_CARD_BG);

    /* 时钟频率 */
    sprintf(buf, "%3d MHz", (int)info->clk.sysclk_mhz);
    draw_text(BAR_X, card_y + 42, 70, buf, GUI_CYAN, GUI_CARD_BG);
    sprintf(buf, "%3d MHz", (int)info->clk.hclk_mhz);
    draw_text(BAR_X, card_y + 60, 70, buf, GUI_CYAN, GUI_CARD_BG);
    sprintf(buf, "%3d MHz", (int)info->clk.pclk1_mhz);
    draw_text(BAR_X, card_y + 78, 70, buf, GUI_CYAN, GUI_CARD_BG);
    sprintf(buf, "%3d MHz", (int)info->clk.pclk2_mhz);
    draw_text(BAR_X + 160, card_y + 42, 70, buf, GUI_CYAN, GUI_CARD_BG);

    /* === 卡片2: Memory === */
    card_y = 28 + 94 + CARD_GAP;

    /* Flash */
    percent = (uint8_t)((uint32_t)info->flash.used * 100 / info->flash.total);
    bar_color = get_bar_color(percent);
    draw_progress_bar(BAR_X, card_y + 25, BAR_W, BAR_H, percent, bar_color);
    format_size(buf, info->flash.used);
    draw_text(VALUE_X - 6, card_y + 24, 56, buf, GUI_VALUE_FG, GUI_CARD_BG);

    /* IntRAM */
    percent = (uint8_t)((uint32_t)info->ram.used * 100 / info->ram.total);
    bar_color = get_bar_color(percent);
    draw_progress_bar(BAR_X, card_y + 45, BAR_W, BAR_H, percent, bar_color);
    format_size(buf, info->ram.used);
    draw_text(VALUE_X - 6, card_y + 44, 56, buf, GUI_VALUE_FG, GUI_CARD_BG);

    /* ExtSRAM */
    percent = (uint8_t)((uint32_t)info->sram.used * 100 / info->sram.total);
    bar_color = get_bar_color(percent);
    draw_progress_bar(BAR_X, card_y + 65, BAR_W, BAR_H, percent, bar_color);
    format_size(buf, info->sram.used);
    draw_text(VALUE_X - 6, card_y + 64, 56, buf, GUI_VALUE_FG, GUI_CARD_BG);

    /* === 卡片3: Status === */
    card_y = 28 + 94 + CARD_GAP + 92 + CARD_GAP;
    format_uptime(buf, info->uptime_ms);
    draw_text(BAR_X, card_y + 20, 100, buf, GUI_ACCENT, GUI_CARD_BG);
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
