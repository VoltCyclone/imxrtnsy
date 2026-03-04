// st7735.c — ST7735 TFT controller init sequence for i.MX RT1062
// 128x160 RGB565 display over SPI
// Ported from RaspberryKMBox/bridge/st7735.c

#include "tft.h"

extern void delay(uint32_t msec);

void tft_preflight(void)
{
	// SWRESET
	tft_command(0x01, 0, 0);
	delay(120);

	// SLPOUT
	tft_command(0x11, 0, 0);
	delay(120);

	// MADCTL: no rotation
	uint8_t madctl = 0x00;
	tft_command(0x36, &madctl, 1);

	// COLMOD: 16-bit RGB565
	uint8_t colmod = 0x05;
	tft_command(0x3a, &colmod, 1);

	// DISPON
	tft_command(0x29, 0, 0);
	delay(120);

	// CASET: 0 to 127
	uint8_t caset[] = { 0, 0, 0, TFT_WIDTH - 1 };
	tft_command(0x2a, caset, 4);

	// RASET: 0 to 159
	uint8_t raset[] = { 0, 0, 0, TFT_HEIGHT - 1 };
	tft_command(0x2b, raset, 4);
}

void tft_begin_sync(void)
{
	// RAMWR: start pixel data
	tft_command(0x2c, 0, 0);
}
