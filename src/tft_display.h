// tft_display.h — Stats rendering for TFT display
// Renders USB proxy metrics onto the ST7735 framebuffer

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	// Connection status
	bool     host_connected;
	bool     device_configured;
	bool     kmbox_active;
	uint8_t  protocol_mode;     // 0=none, 1=KMBox, 2=Makcu, 3=Ferrum

	// Device info
	uint8_t  num_endpoints;
	uint8_t  device_speed;      // 0=full, 1=low, 2=high

	// Throughput
	uint32_t report_count;
	uint32_t drop_count;
	uint32_t reports_per_sec;

	// Smooth injection
	bool     smooth_active;
	uint8_t  smooth_queue_depth;
	uint8_t  smooth_queue_max;
	uint32_t inject_count;

	// KMBox frame stats
	uint32_t kmbox_frames_ok;
	uint32_t kmbox_frames_err;

	// UART / KMBox RX
	uint32_t uart_rx_bytes;    // total bytes received on LPUART6 (DMA ring)

	// System
	uint32_t uptime_sec;
} tft_proxy_stats_t;

// Initialize display hardware + draw splash screen
void tft_display_init(void);

// Render stats into framebuffer and flush to display.
// Call from main loop at ~10 Hz (100ms intervals).
void tft_display_update(const tft_proxy_stats_t *stats);

// Show an error message (replaces normal display)
void tft_display_error(const char *msg);
