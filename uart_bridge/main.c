// UART Bridge for Seeed XIAO RP2350
// USB CDC <-> UART0 bidirectional passthrough
// NeoPixel shows status: red=no USB, cyan=idle, green=data
//
// Wiring: XIAO UART0 TX (GPIO 0 / D6) -> Teensy pin 0 (LPUART6 RX)
//         XIAO UART0 RX (GPIO 1 / D7) <- Teensy pin 1 (LPUART6 TX)

#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

// ---- Pin config (Seeed XIAO RP2350) ----
#define UART_ID          uart0
#define UART_TX_PIN      0
#define UART_RX_PIN      1
#define NEOPIXEL_PIN     22
#define NEOPIXEL_PWR_PIN 23
#define BAUD_RATE        2000000

// ---- NeoPixel ----
static PIO  neo_pio;
static uint neo_sm;

static inline uint32_t neo_grb(uint8_t r, uint8_t g, uint8_t b)
{
	return ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
}

static void neo_set(uint8_t r, uint8_t g, uint8_t b)
{
	pio_sm_put_blocking(neo_pio, neo_sm, neo_grb(r, g, b));
}

static void neo_init(void)
{
	// XIAO RP2350 requires power pin HIGH to enable NeoPixel
	gpio_init(NEOPIXEL_PWR_PIN);
	gpio_set_dir(NEOPIXEL_PWR_PIN, GPIO_OUT);
	gpio_put(NEOPIXEL_PWR_PIN, 1);

	neo_pio = pio0;
	uint offset = pio_add_program(neo_pio, &ws2812_program);
	neo_sm = pio_claim_unused_sm(neo_pio, true);
	ws2812_program_init(neo_pio, neo_sm, offset, NEOPIXEL_PIN, 800000, false);
	neo_set(0, 0, 40); // dim blue = booting
}

// ---- Breathing effect state ----
static uint8_t  breath_val = 40;
static int8_t   breath_dir = 2;

static void breath_tick(uint8_t r, uint8_t g, uint8_t b)
{
	int16_t next = (int16_t)breath_val + breath_dir;
	if (next >= 255) { next = 255; breath_dir = -2; }
	if (next <= 10)  { next = 10;  breath_dir = 2;  }
	breath_val = (uint8_t)next;

	// Scale color by brightness
	neo_set((r * breath_val) >> 8, (g * breath_val) >> 8, (b * breath_val) >> 8);
}

// ---- UART init ----
static void bridge_uart_init(void)
{
	uart_init(UART_ID, BAUD_RATE);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_set_fifo_enabled(UART_ID, true);
}

// ---- Status tracking ----
typedef enum {
	ST_NO_USB,
	ST_IDLE,
	ST_DATA,
} status_t;

int main(void)
{
	stdio_init_all();
	neo_init();
	bridge_uart_init();

	status_t status = ST_NO_USB;
	uint32_t last_neo_ms = 0;
	uint32_t last_data_ms = 0;

	while (1) {
		bool usb_connected = stdio_usb_connected();
		bool had_data = false;

		// Interleave CDC->UART and UART->CDC to prevent RX FIFO overflow.
		// Process a small batch from CDC, then drain all pending UART RX,
		// and repeat until both are empty.
		bool did_work;
		do {
			did_work = false;

			// CDC -> UART (limited batch to let UART RX drain)
			if (usb_connected) {
				for (int i = 0; i < 32; i++) {
					int ch = getchar_timeout_us(0);
					if (ch == PICO_ERROR_TIMEOUT) break;
					uart_putc(UART_ID, (char)ch);
					had_data = true;
					did_work = true;
				}
			}

			// UART -> CDC
			while (uart_is_readable(UART_ID)) {
				char c = uart_getc(UART_ID);
				if (usb_connected)
					putchar_raw(c);
				had_data = true;
				did_work = true;
			}
		} while (did_work);

		if (had_data)
			last_data_ms = to_ms_since_boot(get_absolute_time());

		// Update status
		uint32_t now = to_ms_since_boot(get_absolute_time());
		if (!usb_connected) {
			status = ST_NO_USB;
		} else if (had_data || (now - last_data_ms) < 200) {
			status = ST_DATA;
		} else {
			status = ST_IDLE;
		}

		// NeoPixel update at ~30 Hz
		if ((now - last_neo_ms) >= 33) {
			last_neo_ms = now;
			switch (status) {
			case ST_NO_USB:
				breath_tick(255, 0, 0);    // red breathing
				break;
			case ST_IDLE:
				breath_tick(0, 255, 255);   // cyan breathing = running, idle
				break;
			case ST_DATA:
				neo_set(0, 255, 0);        // bright green = data flowing
				break;
			}
		}
	}

	return 0;
}
