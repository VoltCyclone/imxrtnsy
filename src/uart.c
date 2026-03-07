// Minimal polled UART driver for Teensy 4.1
// Uses LPUART6 = Serial1 (pin 1 = TX, pin 0 = RX)
// Pin 1 = GPIO_AD_B0_02, ALT2 = LPUART6_TX
// UART clock = 24 MHz (set by startup: CCM_CSCDR1_UART_CLK_SEL)

#include "imxrt.h"
#include "uart.h"

#ifndef UART_BAUD
#define UART_BAUD 2000000
#endif
#define UART_CLOCK 24000000
#define RX_GPIO_BIT (1u << 3)
static void uart_set_baud(uint32_t baud)
{
	uint32_t osr;
	if (baud <= 460800) {
		osr = 15;
	} else {
		osr = UART_CLOCK / baud - 1;
		if (osr < 4) osr = 4;
	}
	uint32_t sbr = UART_CLOCK / (baud * (osr + 1));
	if (sbr == 0) sbr = 1;

	LPUART6_BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);
}

#if UART_AUTOBAUD
static uint32_t uart_autobaud_detect(void)
{
	ARM_DEMCR |= ARM_DEMCR_TRCENA;
	ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 = IOMUXC_PAD_PKE | IOMUXC_PAD_PUE |
		IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
	GPIO1_GDIR &= ~RX_GPIO_BIT; // input
	uint32_t t0 = ARM_DWT_CYCCNT;
	while (!(GPIO1_DR & RX_GPIO_BIT)) {
		if ((ARM_DWT_CYCCNT - t0) > (F_CPU / 20)) return 0;
	}
	t0 = ARM_DWT_CYCCNT;
	while (GPIO1_DR & RX_GPIO_BIT) {
		if ((ARM_DWT_CYCCNT - t0) > (F_CPU / 2)) return 0;
	}
	uint32_t min_width = UINT32_MAX;
	uint32_t last_edge = ARM_DWT_CYCCNT;
	uint32_t last_level = 0;
	int edges = 0;

	while (edges < 80) {
		uint32_t level = GPIO1_DR & RX_GPIO_BIT;
		if (level != last_level) {
			uint32_t now = ARM_DWT_CYCCNT;
			uint32_t width = now - last_edge;
			if (width < min_width)
				min_width = width;
			last_edge = now;
			last_level = level;
			edges++;
		}
		// 10ms gap = end of transmission
		if ((ARM_DWT_CYCCNT - last_edge) > (F_CPU / 100))
			break;
	}

	if (edges < 8 || min_width == UINT32_MAX)
		return 0;
	uint32_t raw_baud = F_CPU / min_width;
	static const uint32_t std[] = {
		9600, 19200, 38400, 57600, 115200,
		230400, 460800, 921600, 1000000, 2000000
	};
	uint32_t best = std[0];
	int32_t best_diff = (int32_t)raw_baud - (int32_t)best;
	if (best_diff < 0) best_diff = -best_diff;

	for (uint32_t i = 1; i < sizeof(std) / sizeof(std[0]); i++) {
		int32_t diff = (int32_t)raw_baud - (int32_t)std[i];
		if (diff < 0) diff = -diff;
		if (diff < best_diff) {
			best = std[i];
			best_diff = diff;
		}
	}
	if (best_diff > (int32_t)(best / 6))
		return 0;

	return best;
}
#endif // UART_AUTOBAUD

void uart_init(void)
{
#if !UART_ENABLED
	return;
#else
	CCM_CCGR3 |= CCM_CCGR3_LPUART6(CCM_CCGR_ON);
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	IOMUXC_LPUART6_TX_SELECT_INPUT = 1; // Select GPIO_AD_B0_02 for LPUART6_TX
	uint32_t baud = UART_BAUD;

#if UART_AUTOBAUD
	uint32_t detected = uart_autobaud_detect();
	if (detected)
		baud = detected;
#endif
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
	IOMUXC_LPUART6_RX_SELECT_INPUT = 1;

	uart_set_baud(baud);
	LPUART6_CTRL = LPUART_CTRL_TE | LPUART_CTRL_RE;
#endif
}

void uart_putc(char c)
{
#if !UART_ENABLED
	(void)c;
	return;
#else
	if (LPUART6_BAUD & LPUART_BAUD_TDMAE) return;
	uint32_t timeout = 10000;
	while (!(LPUART6_STAT & LPUART_STAT_TDRE)) {
		if (--timeout == 0) return;
	}
	LPUART6_DATA = c;
#endif
}

void uart_puts(const char *s)
{
	while (*s) {
		uart_putc(*s++);
	}
}

static const char hex_chars[] = "0123456789ABCDEF";

void uart_puthex8(uint8_t val)
{
	uart_putc(hex_chars[(val >> 4) & 0xF]);
	uart_putc(hex_chars[val & 0xF]);
}

void uart_puthex16(uint16_t val)
{
	uart_puthex8(val >> 8);
	uart_puthex8(val & 0xFF);
}

void uart_puthex32(uint32_t val)
{
	uart_puthex16(val >> 16);
	uart_puthex16(val & 0xFFFF);
}

void uart_putdec(uint32_t val)
{
	char buf[12];
	int i = 0;
	if (val == 0) {
		uart_putc('0');
		return;
	}
	while (val > 0) {
		buf[i++] = '0' + (val % 10);
		val /= 10;
	}
	while (i > 0) {
		uart_putc(buf[--i]);
	}
}

void uart_hexdump(const uint8_t *data, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		if (i > 0 && (i % 16) == 0) uart_puts("\r\n");
		else if (i > 0) uart_putc(' ');
		uart_puthex8(data[i]);
	}
	uart_puts("\r\n");
}
