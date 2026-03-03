// Minimal polled UART driver for Teensy 4.1
// Uses LPUART6 = Serial1 (pin 1 = TX, pin 0 = RX)
// Pin 1 = GPIO_AD_B0_02, ALT2 = LPUART6_TX
// UART clock = 24 MHz (set by startup: CCM_CSCDR1_UART_CLK_SEL)

#include "imxrt.h"
#include "uart.h"

#define UART_BAUD 115200
#define UART_CLOCK 24000000

void uart_init(void)
{
#if !UART_ENABLED
	return;
#else
	// Enable LPUART6 clock gate
	CCM_CCGR3 |= CCM_CCGR3_LPUART6(CCM_CCGR_ON);

	// Configure pin 1 (GPIO_AD_B0_02) as LPUART6_TX (ALT2)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	IOMUXC_LPUART6_TX_SELECT_INPUT = 1; // Select GPIO_AD_B0_02 for LPUART6_TX

	// Configure pin 0 (GPIO_AD_B0_03) as LPUART6_RX (ALT2)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
	IOMUXC_LPUART6_RX_SELECT_INPUT = 1;

	// Calculate baud rate
	// SBR = UART_CLOCK / (BAUD * (OSR + 1))
	// With OSR=15: SBR = 24000000 / (115200 * 16) = 13.02 -> 13
	uint32_t osr = 15; // oversample ratio - 1
	uint32_t sbr = UART_CLOCK / (UART_BAUD * (osr + 1));

	LPUART6_BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);
	LPUART6_CTRL = LPUART_CTRL_TE | LPUART_CTRL_RE; // Enable TX and RX
#endif
}

void uart_putc(char c)
{
#if !UART_ENABLED
	(void)c;
	return;
#else
	// Wait for transmit data register empty.  LPUART TDRE reflects
	// internal shift-register state and cycles normally even with no
	// serial cable connected, so this never blocks without hardware.
	// Timeout guards against UART clock misconfiguration only.
	// At 600 MHz / 115200 baud, one byte ≈ 52k cycles ≈ 70k loop iters.
	uint32_t timeout = 80000;
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
