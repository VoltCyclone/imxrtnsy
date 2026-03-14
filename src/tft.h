// tft.h — TFT display driver for i.MX RT1062
// LPSPI4 + eDMA, double-buffered 8-bit indexed framebuffer
// Supports ST7735 (128x160) and ILI9341 (240x320) via TFT_DRIVER

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ---- Driver selection (set via Makefile: TFT_DRIVER=1 or 3) ----
#define TFT_DRIVER_ST7735   1
#define TFT_DRIVER_ILI9341  3

#ifndef TFT_DRIVER
#define TFT_DRIVER  TFT_DRIVER_ILI9341
#endif
#if TFT_DRIVER == TFT_DRIVER_ILI9341
#define TFT_WIDTH    240
#define TFT_HEIGHT   320
#else
#define TFT_WIDTH    128
#define TFT_HEIGHT   160
#endif
#define TFT_FB_SIZE  (TFT_WIDTH * TFT_HEIGHT)
#define TFT_CS_PIN_BIT   0   // Pin 10 = GPIO_B0_00 = GPIO7 bit 0
#define TFT_DC_PIN_BIT   11  // Pin 9  = GPIO_B0_11 = GPIO7 bit 11
#define TFT_RST_PIN_BIT  10  // Pin 6  = GPIO_B0_10 = GPIO7 bit 10
#define TFT_BL_PIN_BIT   17
#define TFT_FONT_W   6
#define TFT_FONT_H   8
#define TFT_COLS     (TFT_WIDTH / TFT_FONT_W)
#define TFT_ROWS     (TFT_HEIGHT / TFT_FONT_H)
extern uint16_t tft_palette[256];
extern uint8_t  tft_font[256 * TFT_FONT_H];
extern uint8_t *tft_input;
void tft_init(void);
void tft_fill(uint8_t color);
void tft_swap_buffers(void);
void tft_sync(void);
void tft_swap_sync(void);
void tft_draw_pixel(int x, int y, uint8_t color);
void tft_draw_rect(int x0, int y0, int x1, int y1, uint8_t color);
void tft_draw_glyph(int x, int y, uint8_t color, char c);
void tft_draw_string(int x, int y, uint8_t color, const char *str);
void tft_draw_string_right(int x, int y, uint8_t color, const char *str);
void tft_draw_hline(int x0, int x1, int y, uint8_t color);
void tft_command(uint8_t cmd, const uint8_t *data, size_t len);
void tft_preflight(void);
void tft_begin_sync(void);
#define COL_BG       0x00  // black
#define COL_WHITE    0x0F  // white
#define COL_GREEN    0xF4  // bright green
#define COL_YELLOW   0xF1  // yellow-orange
#define COL_RED      0xFF  // red
#define COL_CYAN     0xF7  // cyan
#define COL_GRAY     0x0A  // medium gray
#define COL_DARK     0x06  // dark gray
#define COL_DIM      0x04  // dim gray
