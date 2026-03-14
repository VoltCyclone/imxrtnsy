// UART Bridge for Seeed XIAO RP2350
// USB CDC <-> UART0 bidirectional passthrough
// NeoPixel shows status: red=no USB, cyan=idle, green=data
//
// Wiring: XIAO UART0 TX (GPIO 0 / D6) -> Teensy pin 0 (LPUART6 RX)
//         XIAO UART0 RX (GPIO 1 / D7) <- Teensy pin 1 (LPUART6 TX)
//
// Optimizations:
//   - RP2350 overclocked to 200 MHz for headroom
//   - Bulk CDC reads/writes via TinyUSB (no per-byte overhead)
//   - DMA circular RX on UART (zero-CPU byte capture)
//   - UART TX via DMA for USB->UART direction

#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/status_led.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "tusb.h"
#include "ws2812.pio.h"

// ---- Config (pin defs from SDK board header seeed_xiao_rp2350.h) ----
#define BRIDGE_UART      uart_get_instance(PICO_DEFAULT_UART)
#define BAUD_RATE        4000000
#define OVERCLOCK_KHZ    200000

#ifndef PICO_COLORED_STATUS_LED_WS2812_FREQ
#define PICO_COLORED_STATUS_LED_WS2812_FREQ 800000
#endif

// ---- DMA RX ring buffer ----
#define DMA_RX_SIZE      512
static uint8_t dma_rx_buf[DMA_RX_SIZE] __attribute__((aligned(DMA_RX_SIZE)));
static int dma_rx_chan = -1;
static volatile uint16_t rx_tail;

// ---- Staging buffer for bulk transfers ----
#define STAGE_BUF_SIZE   512
static uint8_t stage_buf[STAGE_BUF_SIZE];

// ---- NeoPixel ----
static PIO  neo_pio;
static uint neo_sm;

static void neo_set(uint8_t r, uint8_t g, uint8_t b)
{
	// SDK ws2812 PIO format: GRB, MSB-first, shifted left 8 bits
	uint32_t grb = ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
	pio_sm_put_blocking(neo_pio, neo_sm, grb);
}

static void neo_init(void)
{
	// Power pin for NeoPixel (SDK board define)
#ifdef PICO_DEFAULT_WS2812_POWER_PIN
	gpio_init(PICO_DEFAULT_WS2812_POWER_PIN);
	gpio_set_dir(PICO_DEFAULT_WS2812_POWER_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_WS2812_POWER_PIN, true);
#endif

	// Claim PIO resources using SDK helper
	uint offset;
	pio_claim_free_sm_and_add_program_for_gpio_range(
		&ws2812_program, &neo_pio, &neo_sm, &offset,
		PICO_DEFAULT_WS2812_PIN, 1, true);
	ws2812_program_init(neo_pio, neo_sm, offset, PICO_DEFAULT_WS2812_PIN,
	                    PICO_COLORED_STATUS_LED_WS2812_FREQ,
	                    PICO_COLORED_STATUS_LED_USES_WRGB);
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

// ---- DMA-based UART RX init ----
static void dma_uart_rx_init(void)
{
	dma_rx_chan = dma_claim_unused_channel(true);
	dma_channel_config c = dma_channel_get_default_config(dma_rx_chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_ring(&c, true, 9); // wrap write at 2^9 = 512 bytes
	channel_config_set_dreq(&c, uart_get_dreq(BRIDGE_UART, false)); // RX DREQ

	dma_channel_configure(
		dma_rx_chan,
		&c,
		dma_rx_buf,                          // write to ring buffer
		&uart_get_hw(BRIDGE_UART)->dr,           // read from UART data register
		0xFFFFFFFF,                          // transfer "forever"
		true                                 // start now
	);
	rx_tail = 0;
}

// Get DMA write position (head of ring buffer)
static inline uint16_t dma_rx_head(void)
{
	return ((uint32_t)dma_channel_hw_addr(dma_rx_chan)->write_addr
	        - (uint32_t)dma_rx_buf) & (DMA_RX_SIZE - 1);
}

// ---- UART init ----
static void bridge_uart_init(void)
{
	uart_init(BRIDGE_UART, BAUD_RATE);
	gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);

	// Enable FIFO, set RX threshold low for DMA responsiveness
	uart_set_fifo_enabled(BRIDGE_UART, true);

	// Enable UART RX DMA
	hw_set_bits(&uart_get_hw(BRIDGE_UART)->dmacr, UART_UARTDMACR_RXDMAE_BITS);
}

// ---- Status tracking ----
typedef enum {
	ST_NO_USB,
	ST_IDLE,
	ST_DATA,
} status_t;

int main(void)
{
	// Overclock RP2350 to 200 MHz
	vreg_set_voltage(VREG_VOLTAGE_1_20);
	sleep_ms(2);
	set_sys_clock_khz(OVERCLOCK_KHZ, true);

	// Ensure clk_peri tracks clk_sys after overclock — some SDK versions
	// leave clk_peri at the old frequency, causing UART baud mismatch.
	clock_configure(clk_peri,
	                0,
	                CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
	                OVERCLOCK_KHZ * 1000,
	                OVERCLOCK_KHZ * 1000);

	stdio_init_all();
	neo_init();
	bridge_uart_init();
	dma_uart_rx_init();

	status_t status = ST_NO_USB;
	uint32_t last_neo_ms = 0;
	uint32_t last_data_ms = 0;

	while (1) {
		bool usb_connected = stdio_usb_connected();
		bool had_data = false;

		// ---- USB CDC -> UART TX (bulk) ----
		if (usb_connected && tud_cdc_available()) {
			uint32_t n = tud_cdc_read(stage_buf, sizeof(stage_buf));
			if (n > 0) {
				uart_write_blocking(BRIDGE_UART, stage_buf, n);
				had_data = true;
			}
		}

		// ---- UART RX (DMA ring) -> USB CDC (bulk) ----
		uint16_t head = dma_rx_head();
		if (head != rx_tail) {
			uint16_t n;
			if (head > rx_tail) {
				// Contiguous chunk
				n = head - rx_tail;
				if (usb_connected) {
					tud_cdc_write(&dma_rx_buf[rx_tail], n);
					tud_cdc_write_flush();
				}
			} else {
				// Wrapped — send in two parts
				uint16_t n1 = DMA_RX_SIZE - rx_tail;
				uint16_t n2 = head;
				if (usb_connected) {
					tud_cdc_write(&dma_rx_buf[rx_tail], n1);
					if (n2 > 0)
						tud_cdc_write(&dma_rx_buf[0], n2);
					tud_cdc_write_flush();
				}
				n = n1 + n2;
			}
			rx_tail = head;
			had_data = true;
		}

		if (had_data)
			last_data_ms = to_ms_since_boot(get_absolute_time());
		uint32_t now = to_ms_since_boot(get_absolute_time());
		if (!usb_connected) {
			status = ST_NO_USB;
		} else if (had_data || (now - last_data_ms) < 200) {
			status = ST_DATA;
		} else {
			status = ST_IDLE;
		}
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

		tud_task(); // let TinyUSB process events
	}

	return 0;
}
