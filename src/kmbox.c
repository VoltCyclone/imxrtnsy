// KMBox B-style binary command injection over LPUART6
// Receives commands from an external controller and merges injected
// keyboard/mouse inputs additively with real device HID reports.

#include "kmbox.h"
#include "imxrt.h"
#include "usb_device.h"
#include <string.h>

// ---- UART constants (same as uart.c) ----
#define UART_BAUD  115200
#define UART_CLOCK 24000000

// ---- Parser state machine ----
typedef enum {
	KMBOX_STATE_SYNC1,
	KMBOX_STATE_SYNC2,
	KMBOX_STATE_CMD,
	KMBOX_STATE_LEN,
	KMBOX_STATE_PAYLOAD,
	KMBOX_STATE_CHECKSUM
} kmbox_parse_state_t;

// ---- Injection state ----
typedef struct {
	int16_t  mouse_dx;
	int16_t  mouse_dy;
	uint8_t  mouse_buttons;
	int8_t   mouse_wheel;
	bool     mouse_dirty;

	uint8_t  kb_modifier;
	uint8_t  kb_keys[6];
	bool     kb_dirty;
} kmbox_inject_t;

// ---- Static state ----
static kmbox_parse_state_t parse_state;
static uint8_t  frame_buf[KMBOX_MAX_PAYLOAD];
static uint8_t  frame_cmd;
static uint8_t  frame_len;
static uint8_t  frame_pos;
static uint8_t  frame_checksum;

static kmbox_inject_t inject;
static uint32_t frames_ok;
static uint32_t frames_err;

// Track whether a merge happened this cycle (reset each poll)
static bool merged_this_cycle;

// ---- Forward declarations ----
static void dispatch_frame(void);
static void uart_tx_byte(uint8_t b);

// ---- Public API ----

void kmbox_init(void)
{
	// Enable LPUART6 clock gate
	CCM_CCGR3 |= CCM_CCGR3_LPUART6(CCM_CCGR_ON);

	// Pin 1 (GPIO_AD_B0_02) = LPUART6_TX (ALT2)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	IOMUXC_LPUART6_TX_SELECT_INPUT = 1;

	// Pin 0 (GPIO_AD_B0_03) = LPUART6_RX (ALT2)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2) |
		IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
	IOMUXC_LPUART6_RX_SELECT_INPUT = 1;

	// Baud: OSR=15, SBR = 24MHz / (115200 * 16) = 13
	uint32_t osr = 15;
	uint32_t sbr = UART_CLOCK / (UART_BAUD * (osr + 1));
	LPUART6_BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);
	LPUART6_CTRL = LPUART_CTRL_TE | LPUART_CTRL_RE;

	parse_state = KMBOX_STATE_SYNC1;
	memset(&inject, 0, sizeof(inject));
	frames_ok = 0;
	frames_err = 0;
}

void kmbox_poll(void)
{
	merged_this_cycle = false;

	// Clear line errors and reset parser if any occurred
	uint32_t stat = LPUART6_STAT;
	if (stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF)) {
		LPUART6_STAT = stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF);
		parse_state = KMBOX_STATE_SYNC1;
	}

	// Drain all available RX bytes (non-blocking)
	while (LPUART6_STAT & LPUART_STAT_RDRF) {
		uint8_t b = (uint8_t)(LPUART6_DATA & 0xFF);

		switch (parse_state) {
		case KMBOX_STATE_SYNC1:
			if (b == KMBOX_SYNC1) {
				frame_checksum = b;
				parse_state = KMBOX_STATE_SYNC2;
			}
			break;

		case KMBOX_STATE_SYNC2:
			if (b == KMBOX_SYNC2) {
				frame_checksum += b;
				parse_state = KMBOX_STATE_CMD;
			} else if (b == KMBOX_SYNC1) {
				// Could be start of a new frame
				frame_checksum = b;
				// Stay in SYNC2
			} else {
				parse_state = KMBOX_STATE_SYNC1;
			}
			break;

		case KMBOX_STATE_CMD:
			frame_cmd = b;
			frame_checksum += b;
			parse_state = KMBOX_STATE_LEN;
			break;

		case KMBOX_STATE_LEN:
			frame_len = b;
			frame_checksum += b;
			if (frame_len > KMBOX_MAX_PAYLOAD) {
				frames_err++;
				parse_state = KMBOX_STATE_SYNC1;
			} else if (frame_len == 0) {
				parse_state = KMBOX_STATE_CHECKSUM;
			} else {
				frame_pos = 0;
				parse_state = KMBOX_STATE_PAYLOAD;
			}
			break;

		case KMBOX_STATE_PAYLOAD:
			frame_buf[frame_pos++] = b;
			frame_checksum += b;
			if (frame_pos >= frame_len) {
				parse_state = KMBOX_STATE_CHECKSUM;
			}
			break;

		case KMBOX_STATE_CHECKSUM:
			if ((frame_checksum & 0xFF) == b) {
				dispatch_frame();
				frames_ok++;
			} else {
				frames_err++;
			}
			parse_state = KMBOX_STATE_SYNC1;
			break;
		}
	}
}

void kmbox_merge_report(uint8_t iface_protocol, uint8_t *report, uint8_t len)
{
	if (iface_protocol == 2 && inject.mouse_dirty) {
		// Mouse merge
		// Buttons: OR the masks (byte 0)
		report[0] |= inject.mouse_buttons;

		if (len == 3) {
			// Boot protocol: X and Y are int8
			int16_t mx = (int8_t)report[1] + inject.mouse_dx;
			int16_t my = (int8_t)report[2] + inject.mouse_dy;
			if (mx > 127) mx = 127;
			if (mx < -127) mx = -127;
			if (my > 127) my = 127;
			if (my < -127) my = -127;
			report[1] = (uint8_t)(int8_t)mx;
			report[2] = (uint8_t)(int8_t)my;
		} else if (len >= 5) {
			// Extended: X and Y are int16 LE at bytes 1-4
			int32_t rx = (int16_t)(report[1] | (report[2] << 8));
			int32_t ry = (int16_t)(report[3] | (report[4] << 8));
			int32_t mx = rx + inject.mouse_dx;
			int32_t my = ry + inject.mouse_dy;
			if (mx > 32767) mx = 32767;
			if (mx < -32767) mx = -32767;
			if (my > 32767) my = 32767;
			if (my < -32767) my = -32767;
			report[1] = mx & 0xFF;
			report[2] = (mx >> 8) & 0xFF;
			report[3] = my & 0xFF;
			report[4] = (my >> 8) & 0xFF;

			if (len >= 6 && inject.mouse_wheel != 0) {
				int16_t w = (int8_t)report[5] + inject.mouse_wheel;
				if (w > 127) w = 127;
				if (w < -127) w = -127;
				report[5] = (uint8_t)(int8_t)w;
			}
		}

		// Clear one-shot deltas after merge
		inject.mouse_dx = 0;
		inject.mouse_dy = 0;
		inject.mouse_wheel = 0;
		inject.mouse_dirty = false;
		merged_this_cycle = true;

	} else if (iface_protocol == 1 && inject.kb_dirty) {
		// Keyboard merge (8-byte boot report)
		if (len >= 8) {
			// OR modifier bytes
			report[0] |= inject.kb_modifier;

			// Merge keycodes into empty slots
			for (int i = 0; i < 6; i++) {
				if (inject.kb_keys[i] == 0) continue;

				// Check if already present
				bool found = false;
				for (int j = 2; j < 8; j++) {
					if (report[j] == inject.kb_keys[i]) {
						found = true;
						break;
					}
				}
				if (!found) {
					for (int j = 2; j < 8; j++) {
						if (report[j] == 0) {
							report[j] = inject.kb_keys[i];
							break;
						}
					}
				}
			}
		}
		merged_this_cycle = true;
	}
}

void kmbox_send_pending(const captured_descriptors_t *desc)
{
	// If we already merged into a real report this cycle, nothing to do
	if (merged_this_cycle) return;

	// Send synthetic mouse report if dirty
	if (inject.mouse_dirty) {
		for (uint8_t i = 0; i < desc->num_ifaces; i++) {
			if (desc->ifaces[i].iface_protocol != 2) continue;
			if (desc->ifaces[i].interrupt_ep == 0) continue;

			uint8_t ep = desc->ifaces[i].interrupt_ep & 0x0F;
			uint16_t maxpkt = desc->ifaces[i].interrupt_maxpkt;

			// Build synthetic report (zeroed base + inject data)
			uint8_t synth[8];
			memset(synth, 0, sizeof(synth));
			synth[0] = inject.mouse_buttons;

			uint8_t rlen;
			if (maxpkt <= 3) {
				// Boot protocol format
				int16_t dx = inject.mouse_dx;
				int16_t dy = inject.mouse_dy;
				if (dx > 127) dx = 127;
				if (dx < -127) dx = -127;
				if (dy > 127) dy = 127;
				if (dy < -127) dy = -127;
				synth[1] = (uint8_t)(int8_t)dx;
				synth[2] = (uint8_t)(int8_t)dy;
				rlen = 3;
			} else {
				// Extended format: int16 LE
				synth[1] = inject.mouse_dx & 0xFF;
				synth[2] = (inject.mouse_dx >> 8) & 0xFF;
				synth[3] = inject.mouse_dy & 0xFF;
				synth[4] = (inject.mouse_dy >> 8) & 0xFF;
				synth[5] = (uint8_t)inject.mouse_wheel;
				rlen = (maxpkt < 8) ? (uint8_t)maxpkt : 8;
			}

			usb_device_send_report(ep, synth, rlen);

			inject.mouse_dx = 0;
			inject.mouse_dy = 0;
			inject.mouse_wheel = 0;
			inject.mouse_dirty = false;
			break;
		}
	}

	// Send synthetic keyboard report if dirty and no real keyboard report
	if (inject.kb_dirty) {
		for (uint8_t i = 0; i < desc->num_ifaces; i++) {
			if (desc->ifaces[i].iface_protocol != 1) continue;
			if (desc->ifaces[i].interrupt_ep == 0) continue;

			uint8_t ep = desc->ifaces[i].interrupt_ep & 0x0F;
			uint8_t synth[8];
			synth[0] = inject.kb_modifier;
			synth[1] = 0;
			memcpy(&synth[2], inject.kb_keys, 6);
			usb_device_send_report(ep, synth, 8);
			break;
		}
	}
}

uint32_t kmbox_frame_count(void) { return frames_ok; }
uint32_t kmbox_error_count(void) { return frames_err; }

// ---- Internal ----

static void uart_tx_byte(uint8_t b)
{
	uint32_t timeout = 80000;
	while (!(LPUART6_STAT & LPUART_STAT_TDRE)) {
		if (--timeout == 0) return;
	}
	LPUART6_DATA = b;
}

static void dispatch_frame(void)
{
	switch (frame_cmd) {
	case KMBOX_CMD_MOUSE_MOVE:
		if (frame_len >= 4) {
			inject.mouse_dx += (int16_t)(frame_buf[0] | (frame_buf[1] << 8));
			inject.mouse_dy += (int16_t)(frame_buf[2] | (frame_buf[3] << 8));
			inject.mouse_dirty = true;
		}
		break;

	case KMBOX_CMD_MOUSE_BUTTON:
		if (frame_len >= 1) {
			inject.mouse_buttons = frame_buf[0];
			inject.mouse_dirty = true;
		}
		break;

	case KMBOX_CMD_MOUSE_WHEEL:
		if (frame_len >= 1) {
			inject.mouse_wheel += (int8_t)frame_buf[0];
			inject.mouse_dirty = true;
		}
		break;

	case KMBOX_CMD_MOUSE_ALL:
		if (frame_len >= 6) {
			inject.mouse_buttons = frame_buf[0];
			inject.mouse_dx += (int16_t)(frame_buf[1] | (frame_buf[2] << 8));
			inject.mouse_dy += (int16_t)(frame_buf[3] | (frame_buf[4] << 8));
			inject.mouse_wheel += (int8_t)frame_buf[5];
			inject.mouse_dirty = true;
		}
		break;

	case KMBOX_CMD_KEYBOARD:
		if (frame_len >= 8) {
			inject.kb_modifier = frame_buf[0];
			// frame_buf[1] is reserved
			memcpy(inject.kb_keys, &frame_buf[2], 6);
			inject.kb_dirty = true;
		}
		break;

	case KMBOX_CMD_KEYBOARD_REL:
		inject.kb_modifier = 0;
		memset(inject.kb_keys, 0, 6);
		inject.kb_dirty = false;
		break;

	case KMBOX_CMD_PING:
		uart_tx_byte(0xFE);
		break;

	default:
		// Unknown command — silently ignore
		break;
	}
}
