/**
 * SI22G Dosimeter — тестова прошивка ESP32-C3 Super Mini
 *
 * Розводка (мітки на платі = номер GPIO):
 *   OLED SSD1306 128x32 I2C : SDA=4,  SCL=5
 *   LED                     : 6  → 330 Ω → анод, катод → G
 *   П'єзо push-pull         : 7 (центр), 10 (кільце)
 *     фаза A: 7=HIGH, 10=LOW  |  фаза B: 7=LOW, 10=HIGH
 *     T=400 мкс (2500 Гц), півперіод=200 мкс
 *   АКБ (дільник 10M+4.7M)  : 3  — точка МІЖ 10M і 4.7M
 *   СИ-22Г (імпульси)       : 2  — BC547 колектор → 15k → 3.3V, спад на GPIO (ISR)
 *     HV модуль 405V (вимір HV+↔GND), баласт анод 10MΩ
 *     детектор: база 4.7M→GND, 100nF→470nF DET→база (рекомендовано на платі)
 *   Кнопки → GND            : 8 → CPM, 9 → µSv/h (екран), 20 → лічильник
 *   Живлення OLED           : 3.3 (НЕ 5V!)
 *
 * Не використовувати: 0, 1 (boot/UART). GPIO8/9 — strapping, не тиснути при увімкненні.
 * GPIO20/21 — консоль UART, не підключати сюди детектор.
 * USB + зовнішнє 3.3 V одночасно — ЗАБОРОНЕНО.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ssd1306.h"

#include "splash_bitmap.h"

static const char *TAG = "SI22G_TEST";

/* ── Піни ESP32-C3 Super Mini (мітка на платі = GPIO) ───────────────────── */
#define PIN_OLED_SDA      GPIO_NUM_4
#define PIN_OLED_SCL      GPIO_NUM_5
#define PIN_LED           GPIO_NUM_6
#define PIN_PIEZO_CENTER  GPIO_NUM_7
#define PIN_PIEZO_RING    GPIO_NUM_10
#define PIN_BAT_ADC       GPIO_NUM_3
#define PIN_BTN_1         GPIO_NUM_8   /* → CPM */
#define PIN_BTN_2         GPIO_NUM_9   /* → µSv/h */
#define PIN_BTN_3         GPIO_NUM_20  /* → лічильник імпульсів */
#define PIN_GEIGER        GPIO_NUM_2

/* ── СИ-22Г: зчитування імпульсів ─────────────────────────────────────────── */
#define GEIGER_DEADTIME_US      2500      /* антидребезг BC547, мкс */
#define GEIGER_CPM_WINDOW_SEC   60
#define GEIGER_CPM_INST_SEC     10        /* швидкий CPM (режим CPM на OLED) */
#define GEIGER_CPM_PER_USVH     800.0f    /* ~100 CPM60 @ 0.126 µSv/h, HV 405V, Safecast 126 nSv/h */
#define GEIGER_STATS_PERIOD_MS  1000

/* ── АКБ: дільник 10 MΩ / 4.7 MΩ ─────────────────────────────────────────── */
#define BAT_ADC_ATTEN         ADC_ATTEN_DB_6    /* Vadc 0.3–1.34 V, не DB_11 */
#define BAT_DIVIDER_RATIO     3.12766f          /* (10 + 4.7) / 4.7 */
#define BAT_ADC_VREF_MV       1300              /* ~макс. на GPIO при DB_6 (C3) */
#define BAT_ADC_RAW_SAT       4000
#define BAT_VOLTAGE_FULL      4.2f
#define BAT_VOLTAGE_EMPTY     3.3f
#define BAT_ADC_SAMPLES       8
#define BAT_ADC_SAMPLE_MS     20
#define BAT_ADC_INVALID_V     (-1.0f)
#define BAT_CHECK_MS            120000    /* опит АКБ раз на 2 хв */
#define BAT_ADC_SAMPLES_QUICK   4
#define BAT_ADC_SAMPLE_MS_QUICK 5
#define SPLASH_MS               2000
#define SPLASH_INVERT_COLORS    1   /* тест: 1 = інверсія, 0 = як у PNG */

/* ── OLED UI (128×32) ─────────────────────────────────────────────────────── */
#define OLED_I2C_ADDR           0x3C
#define OLED_I2C_HZ             (400 * 1000)
#define OLED_WIDTH              128
#define OLED_HEIGHT             32
#define OLED_BUF_SIZE           (OLED_WIDTH * OLED_HEIGHT / 8)
#define UI_STATUS_H             8
#define UI_DOSE_Y               8
#define UI_DOSE_SCALE           3
#define UI_DOSE_X               0         /* значення дози — максимум ліворуч */
#define UI_UNIT_Y               24
#define UI_UNIT_MARGIN          2         /* відступ одиниці від правого краю */
#define BAT_ICON_BODY_W         11
#define BAT_ICON_BODY_H         6
#define BAT_ICON_CAP_W          2
#define BAT_ICON_CAP_H          3

/* ── Кнопки (активний LOW → GND) ─────────────────────────────────────────── */
#define BTN_COUNT               3
#define BTN_DEBOUNCE_MS         50

/* ── LED: короткий спалах на кожен імпульс ───────────────────────────────── */
#define LED_PULSE_US            25000

/* ── П'єзо: клік на реальний імпульс трубки ──────────────────────────────── */
#define PIEZO_HALF_PERIOD_US    200
#define PIEZO_BURST_MS          1         /* короткий тик на кожен імпульс */
#define PIEZO_TICK_QUEUE_MAX    8

/* ── Шрифт 6×8 (ASCII 32–127) ─────────────────────────────────────────────── */
#define FONT_CHAR_W  6
#define FONT_CHAR_H  8

static const uint8_t font6x8[96][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* space */
    {0x00,0x00,0x5F,0x00,0x00,0x00,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00,0x00,0x00,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00,0x00,0x00}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00,0x00}, /* $ */
    {0x23,0x13,0x08,0x64,0x62,0x00,0x00,0x00}, /* % */
    {0x36,0x49,0x55,0x22,0x50,0x00,0x00,0x00}, /* & */
    {0x00,0x05,0x03,0x00,0x00,0x00,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00,0x00,0x00}, /* ) */
    {0x14,0x08,0x3E,0x08,0x14,0x00,0x00,0x00}, /* * */
    {0x08,0x08,0x3E,0x08,0x08,0x00,0x00,0x00}, /* + */
    {0x00,0x50,0x30,0x00,0x00,0x00,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08,0x00,0x00,0x00}, /* - */
    {0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02,0x00,0x00,0x00}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00,0x00,0x00}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00,0x00,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00,0x00,0x00}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00,0x00,0x00}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00,0x00,0x00}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00,0x00,0x00}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00,0x00,0x00}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00,0x00,0x00}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00,0x00,0x00}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00,0x00,0x00}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00,0x00,0x00,0x00}, /* ; */
    {0x08,0x14,0x22,0x41,0x00,0x00,0x00,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14,0x00,0x00,0x00}, /* = */
    {0x00,0x41,0x22,0x14,0x08,0x00,0x00,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06,0x00,0x00,0x00}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00,0x00,0x00}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00,0x00,0x00}, /* A */
    {0x7F,0x49,0x49,0x49,0x36,0x00,0x00,0x00}, /* B */
    {0x3E,0x41,0x41,0x41,0x22,0x00,0x00,0x00}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00,0x00,0x00}, /* D */
    {0x7F,0x49,0x49,0x49,0x41,0x00,0x00,0x00}, /* E */
    {0x7F,0x09,0x09,0x09,0x01,0x00,0x00,0x00}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A,0x00,0x00,0x00}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00,0x00,0x00}, /* H */
    {0x00,0x41,0x7F,0x41,0x00,0x00,0x00,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01,0x00,0x00,0x00}, /* J */
    {0x7F,0x08,0x14,0x22,0x41,0x00,0x00,0x00}, /* K */
    {0x7F,0x40,0x40,0x40,0x40,0x00,0x00,0x00}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00,0x00}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00,0x00,0x00}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00,0x00,0x00}, /* O */
    {0x7F,0x09,0x09,0x09,0x06,0x00,0x00,0x00}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00,0x00,0x00}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00,0x00,0x00}, /* R */
    {0x46,0x49,0x49,0x49,0x31,0x00,0x00,0x00}, /* S */
    {0x01,0x01,0x7F,0x01,0x01,0x00,0x00,0x00}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00,0x00,0x00}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00,0x00,0x00}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F,0x00,0x00,0x00}, /* W */
    {0x63,0x14,0x08,0x14,0x63,0x00,0x00,0x00}, /* X */
    {0x07,0x08,0x70,0x08,0x07,0x00,0x00,0x00}, /* Y */
    {0x61,0x51,0x49,0x45,0x43,0x00,0x00,0x00}, /* Z */
    {0x00,0x7F,0x41,0x41,0x00,0x00,0x00,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20,0x00,0x00,0x00}, /* \ */
    {0x00,0x41,0x41,0x7F,0x00,0x00,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04,0x00,0x00,0x00}, /* ^ */
    {0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00}, /* _ */
    {0x00,0x03,0x05,0x00,0x00,0x00,0x00,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78,0x00,0x00,0x00}, /* a */
    {0x7F,0x48,0x44,0x44,0x38,0x00,0x00,0x00}, /* b */
    {0x38,0x44,0x44,0x44,0x20,0x00,0x00,0x00}, /* c */
    {0x38,0x44,0x44,0x48,0x7F,0x00,0x00,0x00}, /* d */
    {0x38,0x54,0x54,0x54,0x18,0x00,0x00,0x00}, /* e */
    {0x08,0x7E,0x09,0x01,0x02,0x00,0x00,0x00}, /* f */
    {0x0C,0x52,0x52,0x52,0x3E,0x00,0x00,0x00}, /* g */
    {0x7F,0x08,0x04,0x04,0x78,0x00,0x00,0x00}, /* h */
    {0x00,0x44,0x7D,0x40,0x00,0x00,0x00,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00,0x00,0x00,0x00}, /* j */
    {0x7F,0x10,0x28,0x44,0x00,0x00,0x00,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00,0x00,0x00,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78,0x00,0x00,0x00}, /* m */
    {0x7C,0x08,0x04,0x04,0x78,0x00,0x00,0x00}, /* n */
    {0x38,0x44,0x44,0x44,0x38,0x00,0x00,0x00}, /* o */
    {0x7C,0x14,0x14,0x14,0x08,0x00,0x00,0x00}, /* p */
    {0x08,0x14,0x14,0x18,0x7C,0x00,0x00,0x00}, /* q */
    {0x7C,0x08,0x04,0x04,0x08,0x00,0x00,0x00}, /* r */
    {0x48,0x54,0x54,0x54,0x20,0x00,0x00,0x00}, /* s */
    {0x04,0x3F,0x44,0x40,0x20,0x00,0x00,0x00}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00,0x00,0x00}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00,0x00,0x00}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00,0x00,0x00}, /* w */
    {0x44,0x28,0x10,0x28,0x44,0x00,0x00,0x00}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00,0x00,0x00}, /* y */
    {0x44,0x64,0x54,0x4C,0x44,0x00,0x00,0x00}, /* z */
    {0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00,0x00,0x00,0x00}, /* } */
    {0x10,0x08,0x08,0x10,0x08,0x00,0x00,0x00}, /* ~ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* DEL placeholder */
};

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;
static SemaphoreHandle_t s_oled_mtx = NULL;
static uint8_t s_oled_buf[OLED_BUF_SIZE];
static bool s_splash_active = false;
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;
static adc_channel_t s_bat_channel = ADC_CHANNEL_2;
static bool s_adc_cali_ok = false;
static int s_bat_last_raw = 0;
static int s_bat_display_pct = -1;      /* -1 = ще немає усередненого значення */
static uint32_t s_bat_last_check_ms = 0;
static int s_display_mode = 1;          /* 0=CPM, 1=µSv/h (за замовч.), 2=total */
static int s_display_value = 0;
static int s_cpm = 0;
static int s_cpm_inst = 0;
static volatile uint32_t s_pulse_isr_total = 0;
static volatile int64_t s_last_pulse_us = 0;
static uint32_t s_sec_buckets[GEIGER_CPM_WINDOW_SEC];
static int s_bucket_idx = 0;
static uint32_t s_sec_pulse_count = 0;
static uint32_t s_last_isr_total = 0;
static volatile uint32_t s_gpio_low_samples = 0;
static int64_t s_last_commit_sec = -1;
static volatile int64_t s_led_off_us = 0;
static volatile uint32_t s_pending_piezo_ticks = 0;
static volatile bool s_geiger_armed = true;

static void IRAM_ATTR geiger_on_pulse(int64_t now_us)
{
    s_geiger_armed = false;
    s_last_pulse_us = now_us;
    s_pulse_isr_total = s_pulse_isr_total + 1;
    gpio_set_level(PIN_LED, 1);
    s_led_off_us = now_us + LED_PULSE_US;
    if (s_pending_piezo_ticks < PIEZO_TICK_QUEUE_MAX) {
        s_pending_piezo_ticks++;
    }
}

static void IRAM_ATTR geiger_gpio_isr(void *arg)
{
    (void)arg;
    const int level = gpio_get_level(PIN_GEIGER);
    if (level == 1) {
        s_geiger_armed = true;
        return;
    }
    if (!s_geiger_armed) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    if ((now - s_last_pulse_us) < GEIGER_DEADTIME_US) {
        return;
    }
    geiger_on_pulse(now);
}

static void geiger_recalc_cpm(void)
{
    uint32_t sum = 0;
    uint32_t sum_inst = 0;
    for (int i = 0; i < GEIGER_CPM_WINDOW_SEC; i++) {
        sum += s_sec_buckets[i];
    }
    for (int i = 0; i < GEIGER_CPM_INST_SEC; i++) {
        const int idx = (s_bucket_idx - 1 - i + GEIGER_CPM_WINDOW_SEC) % GEIGER_CPM_WINDOW_SEC;
        sum_inst += s_sec_buckets[idx];
    }
    s_cpm = (int)sum;
    s_cpm_inst = (int)(sum_inst * 60 / GEIGER_CPM_INST_SEC);

    s_display_value = (int)((float)s_cpm / GEIGER_CPM_PER_USVH * 100.0f + 0.5f);
}

static int geiger_usvh_x100(void)
{
    return (int)((float)s_cpm * 100.0f / GEIGER_CPM_PER_USVH + 0.5f);
}

static void geiger_push_bucket(uint32_t delta)
{
    s_sec_buckets[s_bucket_idx] = delta;
    s_bucket_idx = (s_bucket_idx + 1) % GEIGER_CPM_WINDOW_SEC;
    geiger_recalc_cpm();
}

/* Оновлення раз на календарну секунду (не залежить від OLED/логу) */
static void geiger_commit_seconds(void)
{
    const int64_t sec = esp_timer_get_time() / 1000000LL;
    if (s_last_commit_sec < 0) {
        s_last_isr_total = s_pulse_isr_total;
        s_last_commit_sec = sec;
        return;
    }
    if (sec <= s_last_commit_sec) {
        return;
    }

    const uint32_t total = s_pulse_isr_total;
    const uint32_t delta = total - s_last_isr_total;
    s_last_isr_total = total;

    const int64_t skipped = sec - s_last_commit_sec;
    if (skipped == 1) {
        s_sec_pulse_count = delta;
        geiger_push_bucket(delta);
    } else {
        for (int64_t i = 1; i < skipped; i++) {
            geiger_push_bucket(0);
        }
        s_sec_pulse_count = delta;
        geiger_push_bucket(delta);
    }
    s_last_commit_sec = sec;
}

/* ── OLED: буфер у форматі SSD1306 (vertical page mapping) ─────────────────── */
static void oled_clear(void)
{
    memset(s_oled_buf, 0x00, sizeof(s_oled_buf));
}

static void oled_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }
    uint8_t *byte = &s_oled_buf[(y >> 3) * OLED_WIDTH + x];
    if (on) {
        *byte |= (1 << (y & 7));
    } else {
        *byte &= ~(1 << (y & 7));
    }
}

static void oled_draw_char(int x, int y, char c)
{
    if (c < 32 || c > 127) {
        c = '?';
    }
    const uint8_t *glyph = font6x8[c - 32];
    for (int col = 0; col < FONT_CHAR_W - 1; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < FONT_CHAR_H; row++) {
            if (line & (1 << row)) {
                oled_set_pixel(x + col, y + row, true);
            }
        }
    }
}

static void oled_draw_string(int x, int y, const char *text)
{
    int cursor = x;
    while (*text) {
        oled_draw_char(cursor, y, *text++);
        cursor += FONT_CHAR_W;
    }
}

static int oled_text_width(const char *text)
{
    int n = 0;
    while (*text++) {
        n++;
    }
    return n * FONT_CHAR_W;
}

static void oled_draw_char_scaled(int x, int y, char c, int scale)
{
    if (c < 32 || c > 127) {
        c = '?';
    }
    const uint8_t *glyph = font6x8[c - 32];
    for (int col = 0; col < FONT_CHAR_W - 1; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < FONT_CHAR_H; row++) {
            if (line & (1 << row)) {
                for (int sy = 0; sy < scale; sy++) {
                    for (int sx = 0; sx < scale; sx++) {
                        oled_set_pixel(x + col * scale + sx, y + row * scale + sy, true);
                    }
                }
            }
        }
    }
}

static void oled_draw_string_scaled(int x, int y, const char *text, int scale)
{
    int cursor = x;
    while (*text) {
        oled_draw_char_scaled(cursor, y, *text++, scale);
        cursor += FONT_CHAR_W * scale;
    }
}

static int oled_string_scaled_width(const char *text, int scale)
{
    int len = 0;
    while (text[len]) {
        len++;
    }
    return len * FONT_CHAR_W * scale;
}

static void oled_clear_rect(int x0, int y0, int x1, int y1)
{
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            oled_set_pixel(x, y, false);
        }
    }
}

static esp_err_t oled_flush(void)
{
    return esp_lcd_panel_draw_bitmap(s_panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, s_oled_buf);
}

static bool oled_mutex_take(void)
{
    if (s_oled_mtx == NULL) {
        return false;
    }
    return xSemaphoreTake(s_oled_mtx, pdMS_TO_TICKS(500)) == pdTRUE;
}

static void oled_mutex_give(void)
{
    xSemaphoreGive(s_oled_mtx);
}

static void oled_flush_locked(void)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        if (oled_flush() == ESP_OK) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* Частковий flush — швидше за повний екран (менше I2C) */
static void oled_flush_region_locked(int x0, int y0, int x1, int y1)
{
    if (y0 < 0) {
        y0 = 0;
    }
    if (y1 > OLED_HEIGHT) {
        y1 = OLED_HEIGHT;
    }
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 > OLED_WIDTH) {
        x1 = OLED_WIDTH;
    }
    if (y0 >= y1 || x0 >= x1) {
        return;
    }

    const int page0 = y0 >> 3;
    const int page1 = (y1 - 1) >> 3;
    const int y_start = page0 * 8;
    const int y_end = (page1 + 1) * 8;
    const uint8_t *src = &s_oled_buf[page0 * OLED_WIDTH];

    for (int attempt = 0; attempt < 3; attempt++) {
        if (esp_lcd_panel_draw_bitmap(s_panel, x0, y_start, x1, y_end, src) == ESP_OK) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* Іконка АКБ у верхньому правому куті */
static void oled_draw_battery_icon(int x, int y, int pct)
{
    const int bw = BAT_ICON_BODY_W;
    const int bh = BAT_ICON_BODY_H;

    for (int i = 0; i < bw; i++) {
        oled_set_pixel(x + i, y, true);
        oled_set_pixel(x + i, y + bh - 1, true);
    }
    for (int j = 1; j < bh - 1; j++) {
        oled_set_pixel(x, y + j, true);
        oled_set_pixel(x + bw - 1, y + j, true);
    }

    const int cap_y = y + (bh - BAT_ICON_CAP_H) / 2;
    for (int j = 0; j < BAT_ICON_CAP_H; j++) {
        for (int i = 0; i < BAT_ICON_CAP_W; i++) {
            oled_set_pixel(x + bw + i, cap_y + j, true);
        }
    }

    if (pct < 0) {
        return;
    }

    const int inner_w = bw - 2;
    const int fill_w = (inner_w * pct + 50) / 100;
    for (int i = 0; i < fill_w; i++) {
        for (int j = 1; j < bh - 1; j++) {
            oled_set_pixel(x + 1 + i, y + j, true);
        }
    }
}

static void oled_draw_battery_top_right(void)
{
    char pct_str[16];
    const int icon_w = BAT_ICON_BODY_W + BAT_ICON_CAP_W;

    if (s_bat_display_pct < 0) {
        snprintf(pct_str, sizeof(pct_str), "--%%");
    } else {
        snprintf(pct_str, sizeof(pct_str), "%d%%", s_bat_display_pct);
    }

    const int pct_w = oled_text_width(pct_str);
    const int total_w = icon_w + 2 + pct_w;
    const int x0 = OLED_WIDTH - total_w;
    const int y = 1;

    oled_clear_rect(x0 - 2, 0, OLED_WIDTH, UI_STATUS_H);
    oled_draw_battery_icon(x0, y, s_bat_display_pct);
    oled_draw_string(x0 + icon_w + 2, y, pct_str);
}

static void oled_draw_dose_main(void)
{
    char value[24];
    const char *unit = "CPM";

    switch (s_display_mode) {
    case 0:
        unit = "CPM";
        snprintf(value, sizeof(value), "%d", s_cpm_inst);
        break;
    case 2:
        unit = "cnt";
        snprintf(value, sizeof(value), "%lu", (unsigned long)s_pulse_isr_total);
        break;
    case 1: {
        unit = "uSv/h";
        const int usvh_x100 = geiger_usvh_x100();
        snprintf(value, sizeof(value), "%d.%02d", usvh_x100 / 100, usvh_x100 % 100);
        break;
    }
    default:
        unit = "CPM";
        snprintf(value, sizeof(value), "%d", s_cpm_inst);
        break;
    }

    oled_clear_rect(0, UI_STATUS_H, OLED_WIDTH, OLED_HEIGHT);
    oled_draw_string_scaled(UI_DOSE_X, UI_DOSE_Y, value, UI_DOSE_SCALE);
    const int unit_w = oled_text_width(unit);
    oled_draw_string(OLED_WIDTH - unit_w - UI_UNIT_MARGIN, UI_UNIT_Y, unit);
}

static void oled_refresh_dose(void)
{
    if (s_splash_active) {
        return;
    }
    if (!oled_mutex_take()) {
        return;
    }
    oled_draw_dose_main();
    oled_flush_region_locked(0, UI_STATUS_H, OLED_WIDTH, OLED_HEIGHT);
    oled_mutex_give();
}

static void oled_refresh_battery(void)
{
    if (s_splash_active) {
        return;
    }
    if (!oled_mutex_take()) {
        return;
    }
    oled_draw_battery_top_right();
    oled_flush_region_locked(0, 0, OLED_WIDTH, UI_STATUS_H);
    oled_mutex_give();
}

static void oled_draw_work_screen(void)
{
    if (!oled_mutex_take()) {
        return;
    }
    oled_clear();
    oled_draw_battery_top_right();
    oled_draw_dose_main();
    oled_flush_locked();
    oled_mutex_give();
}

static void oled_update_battery_corner(void)
{
    oled_refresh_battery();
}

static void oled_draw_splash(void)
{
    if (!oled_mutex_take()) {
        return;
    }
    memcpy(s_oled_buf, splash_bitmap, sizeof(splash_bitmap));
#if SPLASH_INVERT_COLORS
    for (size_t i = 0; i < sizeof(splash_bitmap); i++) {
        s_oled_buf[i] ^= 0xFF;
    }
#endif
    oled_flush_locked();
    oled_mutex_give();
}

static esp_err_t oled_init(void)
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = PIN_OLED_SDA;
    bus_cfg.scl_io_num = PIN_OLED_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = 1;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "I2C bus init failed");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_i2c_config_t io_cfg = {};
    io_cfg.dev_addr = OLED_I2C_ADDR;
    io_cfg.control_phase_bytes = 1;
    io_cfg.dc_bit_offset = 6;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;
    io_cfg.scl_speed_hz = OLED_I2C_HZ;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(s_i2c_bus, &io_cfg, &io), TAG, "panel IO failed");

    esp_lcd_panel_ssd1306_config_t ssd1306_cfg = {
        .height = OLED_HEIGHT,
    };
    esp_lcd_panel_dev_config_t panel_cfg = {};
    panel_cfg.reset_gpio_num = GPIO_NUM_NC;
    panel_cfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_cfg.data_endian = LCD_RGB_DATA_ENDIAN_BIG;
    panel_cfg.bits_per_pixel = 1;
    panel_cfg.vendor_config = &ssd1306_cfg;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ssd1306(io, &panel_cfg, &s_panel), TAG, "SSD1306 init failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "panel on failed");

    s_oled_mtx = xSemaphoreCreateMutex();
    if (s_oled_mtx == NULL) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ── АКБ: ADC на GPIO3 ─────────────────────────────────────────────────────── */
static bool battery_adc_calibration_init(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = s_bat_channel,
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_handle) == ESP_OK) {
        return true;
    }
#endif
    return false;
}

static void battery_adc_pin_scan(void)
{
    static const int scan_gpios[] = {0, 1, 3, 4}; /* не GPIO2 — там СИ-22Г */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_LOGI(TAG, "=== ADC scan at boot ===");
    for (int gpio : scan_gpios) {
        adc_unit_t unit = ADC_UNIT_1;
        adc_channel_t channel = ADC_CHANNEL_0;
        if (adc_oneshot_io_to_channel(gpio, &unit, &channel) != ESP_OK || unit != ADC_UNIT_1) {
            continue;
        }
        if (adc_oneshot_config_channel(s_adc_handle, channel, &chan_cfg) != ESP_OK) {
            continue;
        }

        int raw_sum = 0;
        for (int i = 0; i < 8; i++) {
            int raw = 0;
            if (adc_oneshot_read(s_adc_handle, channel, &raw) == ESP_OK) {
                raw_sum += raw;
            }
        }
        const int raw = raw_sum / 8;
        const int pin_mv = (raw * BAT_ADC_VREF_MV) / 4095;
        ESP_LOGI(TAG, "  GPIO%d ch%d raw=%d pin~%dmV", gpio, channel, raw, pin_mv);
    }

    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, s_bat_channel, &chan_cfg));
}

static void battery_adc_init(void)
{
    adc_unit_t unit = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(PIN_BAT_ADC, &unit, &s_bat_channel));

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, s_bat_channel, &chan_cfg));

    s_adc_cali_ok = battery_adc_calibration_init();
    ESP_LOGI(TAG, "Battery ADC GPIO%d -> ch%d cali=%s",
             PIN_BAT_ADC, s_bat_channel, s_adc_cali_ok ? "OK" : "linear");

    battery_adc_pin_scan();
}

static float battery_read_voltage(void)
{
    /* НЕ викликати gpio_reset_pin тут — це знімає пін з ADC і дає raw=4095 */
    for (int i = 0; i < 4; i++) {
        int dummy = 0;
        adc_oneshot_read(s_adc_handle, s_bat_channel, &dummy);
        vTaskDelay(pdMS_TO_TICKS(BAT_ADC_SAMPLE_MS));
    }

    int raw_sum = 0;
    for (int i = 0; i < BAT_ADC_SAMPLES; i++) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc_handle, s_bat_channel, &raw));
        raw_sum += raw;
        vTaskDelay(pdMS_TO_TICKS(BAT_ADC_SAMPLE_MS));
    }

    const int raw_avg = raw_sum / BAT_ADC_SAMPLES;
    s_bat_last_raw = raw_avg;

    if (raw_avg >= BAT_ADC_RAW_SAT) {
        return BAT_ADC_INVALID_V;
    }

    int adc_mv = (raw_avg * BAT_ADC_VREF_MV) / 4095;
    if (s_adc_cali_ok && adc_cali_raw_to_voltage(s_adc_cali_handle, raw_avg, &adc_mv) != ESP_OK) {
        adc_mv = (raw_avg * BAT_ADC_VREF_MV) / 4095;
    }

    return (adc_mv / 1000.0f) * BAT_DIVIDER_RATIO;
}

static int battery_percent(float battery_v)
{
    if (battery_v < 0.0f) {
        return 0;
    }
    float pct = (battery_v - BAT_VOLTAGE_EMPTY) / (BAT_VOLTAGE_FULL - BAT_VOLTAGE_EMPTY) * 100.0f;
    if (pct < 0.0f) {
        pct = 0.0f;
    } else if (pct > 100.0f) {
        pct = 100.0f;
    }
    return (int)(pct + 0.5f);
}

/* Швидке читання АКБ для рідкого фонового опиту */
static float battery_read_voltage_quick(void)
{
    int raw_sum = 0;
    for (int i = 0; i < BAT_ADC_SAMPLES_QUICK; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc_handle, s_bat_channel, &raw) == ESP_OK) {
            raw_sum += raw;
        }
        vTaskDelay(pdMS_TO_TICKS(BAT_ADC_SAMPLE_MS_QUICK));
    }

    const int raw_avg = raw_sum / BAT_ADC_SAMPLES_QUICK;
    s_bat_last_raw = raw_avg;

    if (raw_avg >= BAT_ADC_RAW_SAT) {
        return BAT_ADC_INVALID_V;
    }

    int adc_mv = (raw_avg * BAT_ADC_VREF_MV) / 4095;
    if (s_adc_cali_ok && adc_cali_raw_to_voltage(s_adc_cali_handle, raw_avg, &adc_mv) != ESP_OK) {
        adc_mv = (raw_avg * BAT_ADC_VREF_MV) / 4095;
    }

    return (adc_mv / 1000.0f) * BAT_DIVIDER_RATIO;
}

/* ── АКБ: раз на 2 хв, без блокування OLED ───────────────────────────────── */
static bool battery_sample_tick(uint32_t now_ms)
{
    if (now_ms - s_bat_last_check_ms < BAT_CHECK_MS) {
        return false;
    }
    s_bat_last_check_ms = now_ms;

    const float vbat = battery_read_voltage_quick();
    if (vbat >= 0.0f) {
        s_bat_display_pct = battery_percent(vbat);
        ESP_LOGI(TAG, "Bat: %d%% (raw=%d)", s_bat_display_pct, s_bat_last_raw);
        return true;
    }

    ESP_LOGW(TAG, "Bat: invalid sample (raw=%d)", s_bat_last_raw);
    return false;
}

/* Перший % АКБ під час заставки */
static int battery_measure_boot(void)
{
    int sum = 0;
    int valid = 0;
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)SPLASH_MS * 1000;

    while (esp_timer_get_time() < deadline_us) {
        const float vbat = battery_read_voltage();
        if (vbat >= 0.0f) {
            sum += battery_percent(vbat);
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (valid == 0) {
        ESP_LOGW(TAG, "Boot bat: no valid samples");
        return -1;
    }

    const int pct = (sum + valid / 2) / valid;
    ESP_LOGI(TAG, "Boot bat: %d%% (%d samples)", pct, valid);
    return pct;
}

/* ── LED ───────────────────────────────────────────────────────────────────── */
static void led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_LED, 0);
}

/* ── Кнопки GPIO8 / GPIO9 / GPIO20 (LOW = натиснуто) ─────────────────────── */
static void buttons_init(void)
{
    const uint64_t mask = (1ULL << PIN_BTN_1) | (1ULL << PIN_BTN_2) | (1ULL << PIN_BTN_3);
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    ESP_LOGI(TAG, "Buttons: GPIO8=CPM, GPIO9=uSv/h, GPIO20=total (active LOW)");
}

static void buttons_poll(uint32_t now_ms)
{
    static const struct {
        gpio_num_t pin;
        int value;
    } btns[BTN_COUNT] = {
        { PIN_BTN_1, 0 },
        { PIN_BTN_2, 1 },
        { PIN_BTN_3, 2 },
    };

    static bool debounced_high[BTN_COUNT] = { true, true, true };
    static bool latch[BTN_COUNT] = { false, false, false };
    static uint32_t change_ms[BTN_COUNT] = { 0, 0, 0 };

    for (int i = 0; i < BTN_COUNT; i++) {
        const bool high = gpio_get_level(btns[i].pin) != 0;

        if (high != debounced_high[i]) {
            change_ms[i] = now_ms;
            debounced_high[i] = high;
        }

        if ((now_ms - change_ms[i]) < BTN_DEBOUNCE_MS) {
            continue;
        }

        if (!high) {
            if (!latch[i]) {
                latch[i] = true;
                s_display_mode = btns[i].value;
                oled_refresh_dose();
                ESP_LOGI(TAG, "Button GPIO%d -> mode %d", btns[i].pin, btns[i].value);
            }
        } else {
            latch[i] = false;
        }
    }
}

/* ── П'єзо ─────────────────────────────────────────────────────────────────── */
static void piezo_pins_off(void)
{
    gpio_set_level(PIN_PIEZO_CENTER, 0);
    gpio_set_level(PIN_PIEZO_RING, 0);
}

static void piezo_pushpull_phase(bool center_high)
{
    gpio_set_level(PIN_PIEZO_CENTER, center_high ? 1 : 0);
    gpio_set_level(PIN_PIEZO_RING, center_high ? 0 : 1);
}

static void buzzer_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_PIEZO_CENTER) | (1ULL << PIN_PIEZO_RING),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_drive_capability(PIN_PIEZO_CENTER, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(PIN_PIEZO_RING, GPIO_DRIVE_CAP_3);
    piezo_pins_off();
}

static void geiger_burst_pushpull(uint32_t burst_ms)
{
    const int64_t end_us = esp_timer_get_time() + (int64_t)burst_ms * 1000;

    while (esp_timer_get_time() < end_us) {
        piezo_pushpull_phase(true);
        esp_rom_delay_us(PIEZO_HALF_PERIOD_US);
        piezo_pushpull_phase(false);
        esp_rom_delay_us(PIEZO_HALF_PERIOD_US);
    }
    piezo_pins_off();
}

static void geiger_configure_input(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

static void geiger_pulse_init(void)
{
    geiger_configure_input(PIN_GEIGER);

    gpio_set_pull_mode(PIN_GEIGER, GPIO_PULLDOWN_ONLY);
    vTaskDelay(pdMS_TO_TICKS(5));
    const int lvl_pd = gpio_get_level(PIN_GEIGER);

    gpio_set_pull_mode(PIN_GEIGER, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(5));
    const int lvl_pu = gpio_get_level(PIN_GEIGER);

    gpio_set_pull_mode(PIN_GEIGER, GPIO_FLOATING);
    s_geiger_armed = true;
    const int lvl_f = gpio_get_level(PIN_GEIGER);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_GEIGER, geiger_gpio_isr, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_GEIGER, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_intr_enable(PIN_GEIGER));

    ESP_LOGI(TAG, "Geiger GPIO%d: float=%d pulldown=%d pullup=%d (OFF:0,1,1; ON 15k:1,1,1)",
             PIN_GEIGER, lvl_f, lvl_pd, lvl_pu);
    ESP_LOGI(TAG, "  -> SHORT this pin to GND 3s: total must rise, gpio must be 0");
}

static void geiger_pin_scan_log(void)
{
    static const gpio_num_t scan[] = {
        GPIO_NUM_0, GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3,
    };
    char buf[64];
    int pos = 0;
    for (gpio_num_t pin : scan) {
        const int n = snprintf(buf + pos, sizeof(buf) - (size_t)pos, " g%d=%d", pin, gpio_get_level(pin));
        if (n > 0) {
            pos += n;
        }
    }
    ESP_LOGI(TAG, "Pin scan:%s  (short YOUR wire to GND — one must go 0)", buf);
}

static void geiger_poll_task(void *arg)
{
    (void)arg;
    uint32_t low_samples = 0;
    uint32_t poll_loops = 0;

    while (true) {
        if (gpio_get_level(PIN_GEIGER) == 0) {
            low_samples++;
        }

        const int64_t led_now = esp_timer_get_time();
        if (s_led_off_us != 0 && led_now >= s_led_off_us) {
            gpio_set_level(PIN_LED, 0);
            s_led_off_us = 0;
        }

        poll_loops++;
        if ((poll_loops % 5000) == 0) {
            s_gpio_low_samples = low_samples;
            low_samples = 0;
            vTaskDelay(1);
        } else {
            esp_rom_delay_us(20);
        }
    }
}

static void piezo_task(void *arg)
{
    (void)arg;
    while (true) {
        const uint32_t ticks = s_pending_piezo_ticks;
        if (ticks > 0) {
            s_pending_piezo_ticks = ticks - 1;
            geiger_burst_pushpull(PIEZO_BURST_MS);
        } else {
            vTaskDelay(1);
        }
    }
}

static void geiger_stats_task(void *arg)
{
    (void)arg;
    memset(s_sec_buckets, 0, sizeof(s_sec_buckets));
    s_last_commit_sec = -1;

    while (true) {
        geiger_commit_seconds();
        vTaskDelay(pdMS_TO_TICKS(GEIGER_STATS_PERIOD_MS));

        ESP_LOGI(TAG, "Geiger: +%lu/s CPM10=%d CPM60=%d uSv/h=%.2f total=%lu gpio=%d low5k=%lu",
                 (unsigned long)s_sec_pulse_count, s_cpm_inst, s_cpm,
                 (double)geiger_usvh_x100() / 100.0,
                 (unsigned long)s_pulse_isr_total,
                 gpio_get_level(PIN_GEIGER),
                 (unsigned long)s_gpio_low_samples);
        if (s_pulse_isr_total == 0) {
            geiger_pin_scan_log();
        }
        oled_refresh_dose();
    }
}

/* ── Головний цикл ─────────────────────────────────────────────────────────── */
static void app_task(void *arg)
{
    s_splash_active = true;
    oled_draw_splash();
    const int boot_pct = battery_measure_boot();
    s_splash_active = false;
    if (boot_pct >= 0) {
        s_bat_display_pct = boot_pct;
    }
    s_bat_last_check_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    oled_draw_work_screen();

    while (true) {
        const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (battery_sample_tick(now)) {
            oled_update_battery_corner();
        }

        buttons_poll(now);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "System started");

    battery_adc_init();
    ESP_ERROR_CHECK(oled_init());
    led_init();
    buttons_init();
    buzzer_init();
    geiger_pulse_init();

    xTaskCreate(geiger_poll_task, "geiger_poll", 3072, NULL, 10, NULL);
    xTaskCreate(piezo_task, "piezo", 2048, NULL, 4, NULL);
    xTaskCreate(geiger_stats_task, "geiger", 3072, NULL, 6, NULL);
    xTaskCreate(app_task, "si22g_app", 4096, NULL, 5, NULL);
}
