// tft.h — ST7735 TFT display driver for i.MX RT1062
// LPSPI4 + eDMA, double-buffered 8-bit indexed framebuffer

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ---- Display dimensions (ST7735 128x160) ----
#define TFT_WIDTH    128
#define TFT_HEIGHT   160
#define TFT_FB_SIZE  (TFT_WIDTH * TFT_HEIGHT)

// ---- Pin assignments (Teensy 4.1) ----
// LPSPI4 SPI pins (directly muxed, not GPIO)
// Pin 13 = GPIO_B0_03 = LPSPI4_SCK  (ALT3)
// Pin 11 = GPIO_B0_02 = LPSPI4_SDO  (ALT3)
// Pin 12 = GPIO_B0_01 = LPSPI4_SDI  (ALT3, unused for write-only display)

// GPIO-controlled pins  (active low for CS, active low for RST)
#define TFT_CS_PIN_BIT   0   // Pin 10 = GPIO_B0_00 = GPIO7 bit 0
#define TFT_DC_PIN_BIT   11  // Pin 9  = GPIO_B0_11 = GPIO7 bit 11
#define TFT_RST_PIN_BIT  10  // Pin 6  = GPIO_B0_10 = GPIO7 bit 10
// Backlight: Pin 7 = GPIO_B1_01 = GPIO7 bit 17 (optional, active high)
#define TFT_BL_PIN_BIT   17

// ---- Font dimensions (6x8 bitmap font) ----
#define TFT_FONT_W   6
#define TFT_FONT_H   8

// ---- Palette & font ----
extern uint16_t tft_palette[256];
extern uint8_t  tft_font[256 * TFT_FONT_H];

// ---- Framebuffer ----
// Active buffer — draw here
extern uint8_t *tft_input;

// ---- API ----

// Initialize LPSPI4, eDMA ch1, GPIO pins, reset display, run ST7735 init.
// Call once after clocks are up.
void tft_init(void);

// Fill entire framebuffer with palette index
void tft_fill(uint8_t color);

// Swap double buffers
void tft_swap_buffers(void);

// Transfer committed buffer to display via DMA SPI (blocks until complete)
void tft_sync(void);

// swap + sync convenience
void tft_swap_sync(void);

// ---- Drawing primitives (operate on tft_input) ----

void tft_draw_pixel(int x, int y, uint8_t color);
void tft_draw_rect(int x0, int y0, int x1, int y1, uint8_t color);
void tft_draw_glyph(int x, int y, uint8_t color, char c);
void tft_draw_string(int x, int y, uint8_t color, const char *str);
void tft_draw_string_right(int x, int y, uint8_t color, const char *str);
void tft_draw_hline(int x0, int x1, int y, uint8_t color);

// ---- Low-level ----

// Send command + optional data bytes to ST7735
void tft_command(uint8_t cmd, const uint8_t *data, size_t len);

// ST7735 controller init (implemented in st7735.c)
void tft_preflight(void);
void tft_begin_sync(void);

// ---- Palette color indices ----
#define COL_BG       0x00  // black
#define COL_WHITE    0x0F  // white
#define COL_GREEN    0xF4  // bright green
#define COL_YELLOW   0xF1  // yellow-orange
#define COL_RED      0xFF  // red
#define COL_CYAN     0xF7  // cyan
#define COL_GRAY     0x0A  // medium gray
#define COL_DARK     0x06  // dark gray
#define COL_DIM      0x04  // dim gray
