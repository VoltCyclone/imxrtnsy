// Makcu binary protocol command handler
// Ported from RaspberryKMBox bridge/makcu_translator.c
// Handles mouse commands: MOVE, MO, button controls, WHEEL, CLICK
// Keyboard/misc commands are silently ignored.

#include "makcu.h"
#include <string.h>

// HID button masks
#define BTN_LEFT    0x01
#define BTN_RIGHT   0x02
#define BTN_MIDDLE  0x04
#define BTN_BACK    0x08
#define BTN_FORWARD 0x10

// Persistent button state
static uint8_t g_buttons;

static uint8_t button_to_mask(uint8_t button)
{
	switch (button) {
	case 1: return BTN_LEFT;
	case 2: return BTN_RIGHT;
	case 3: return BTN_MIDDLE;
	case 4: return BTN_BACK;
	case 5: return BTN_FORWARD;
	default: return 0;
	}
}

void makcu_init(void)
{
	g_buttons = 0;
}

bool makcu_parse_command(uint8_t cmd, const uint8_t *payload, uint16_t len,
                         makcu_result_t *out)
{
	memset(out, 0, sizeof(*out));

	switch (cmd) {
	case MAKCU_CMD_MOVE:
		// [dx:i16, dy:i16, segments:u8, cx1:i8, cy1:i8]
		if (len < 4) return false;
		out->mouse_dx = (int16_t)(payload[0] | (payload[1] << 8));
		out->mouse_dy = (int16_t)(payload[2] | (payload[3] << 8));
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		return true;

	case MAKCU_CMD_MO:
		// [buttons:u8, x:i16, y:i16, wheel:i8, pan:i8, tilt:i8]
		if (len < 6) return false;
		g_buttons = payload[0];
		out->mouse_buttons = g_buttons;
		out->mouse_dx = (int16_t)(payload[1] | (payload[2] << 8));
		out->mouse_dy = (int16_t)(payload[3] | (payload[4] << 8));
		if (len >= 6) out->mouse_wheel = (int8_t)payload[5];
		out->has_mouse = true;
		return true;

	case MAKCU_CMD_LEFT_BUTTON:
	case MAKCU_CMD_RIGHT_BUTTON:
	case MAKCU_CMD_MIDDLE_BUTTON:
	case MAKCU_CMD_SIDE1_BUTTON:
	case MAKCU_CMD_SIDE2_BUTTON: {
		if (len < 1) return false;
		uint8_t state = payload[0];
		uint8_t bnum = 0;
		switch (cmd) {
		case MAKCU_CMD_LEFT_BUTTON:   bnum = 1; break;
		case MAKCU_CMD_RIGHT_BUTTON:  bnum = 2; break;
		case MAKCU_CMD_MIDDLE_BUTTON: bnum = 3; break;
		case MAKCU_CMD_SIDE1_BUTTON:  bnum = 4; break;
		case MAKCU_CMD_SIDE2_BUTTON:  bnum = 5; break;
		}
		uint8_t mask = button_to_mask(bnum);
		if (!mask) return false;
		if (state == 1)
			g_buttons |= mask;
		else
			g_buttons &= ~mask;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		return true;
	}

	case MAKCU_CMD_WHEEL:
		if (len < 1) return false;
		out->mouse_wheel = (int8_t)payload[0];
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		return true;

	case MAKCU_CMD_CLICK:
		// [button:u8, count:u8, delay_ms:u8]
		// We treat click as a button-down (the release timing is
		// not handled here — the caller can schedule it).
		if (len < 1) return false;
		{
			uint8_t mask = button_to_mask(payload[0]);
			if (!mask) return false;
			g_buttons |= mask;
			out->mouse_buttons = g_buttons;
			out->has_mouse = true;
		}
		return true;

	default:
		return false;
	}
}
