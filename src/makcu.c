// Makcu binary protocol command handler
// Frame: [0x50] [cmd] [len_lo] [len_hi] [payload...]
// Response: [0x50] [cmd] [len_lo] [len_hi] [status/payload...]
// Handles mouse, keyboard, and system commands.

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

// Persistent keyboard state
static uint8_t g_kb_modifier;
static uint8_t g_kb_keys[6];

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

// Build a setter-OK response: [0x50][cmd][0x01][0x00][0x00]
static void set_ok_response(makcu_result_t *out, uint8_t cmd)
{
	out->needs_response = true;
	out->resp_cmd = cmd;
	out->resp_payload[0] = 0x00; // status OK
	out->resp_payload_len = 1;
}

void makcu_init(void)
{
	g_buttons = 0;
	g_kb_modifier = 0;
	memset(g_kb_keys, 0, sizeof(g_kb_keys));
}

bool makcu_parse_command(uint8_t cmd, const uint8_t *payload, uint16_t len,
                         makcu_result_t *out)
{
	memset(out, 0, sizeof(*out));

	switch (cmd) {

	// ---- Mouse commands ----

	case MAKCU_CMD_MOVE:
		// [dx:i16, dy:i16, segments:u8?, cx1:i8?, cy1:i8?, cx2:i8?, cy2:i8?]
		if (len < 4) return false;
		out->mouse_dx = (int16_t)(payload[0] | (payload[1] << 8));
		out->mouse_dy = (int16_t)(payload[2] | (payload[3] << 8));
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		set_ok_response(out, cmd);
		return true;

	case MAKCU_CMD_MO:
		// [buttons:u8, x:i16, y:i16, wheel:i8, pan:i8, tilt:i8]
		if (len < 6) return false;
		g_buttons = payload[0];
		out->mouse_buttons = g_buttons;
		out->mouse_dx = (int16_t)(payload[1] | (payload[2] << 8));
		out->mouse_dy = (int16_t)(payload[3] | (payload[4] << 8));
		out->mouse_wheel = (int8_t)payload[5];
		out->has_mouse = true;
		set_ok_response(out, cmd);
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
		set_ok_response(out, cmd);
		return true;
	}

	case MAKCU_CMD_WHEEL:
		if (len < 1) return false;
		out->mouse_wheel = (int8_t)payload[0];
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		set_ok_response(out, cmd);
		return true;

	case MAKCU_CMD_CLICK:
		// [button:u8, count:u8?, delay_ms:u8?]
		// Press button, then signal caller to schedule release
		if (len < 1) return false;
		{
			uint8_t mask = button_to_mask(payload[0]);
			if (!mask) return false;
			g_buttons |= mask;
			out->mouse_buttons = g_buttons;
			out->has_mouse = true;
			out->click_release = true; // caller must release after sending press
		}
		set_ok_response(out, cmd);
		return true;

	// ---- Keyboard commands ----

	case MAKCU_CMD_KB_DOWN:
		// [key:u8]
		if (len < 1) return false;
		{
			uint8_t key = payload[0];
			// Add key to first empty slot
			for (int i = 0; i < 6; i++) {
				if (g_kb_keys[i] == key) break; // already pressed
				if (g_kb_keys[i] == 0) {
					g_kb_keys[i] = key;
					break;
				}
			}
			out->kb_modifier = g_kb_modifier;
			memcpy(out->kb_keys, g_kb_keys, 6);
			out->has_keyboard = true;
		}
		set_ok_response(out, cmd);
		return true;

	case MAKCU_CMD_KB_UP:
		// [key:u8]
		if (len < 1) return false;
		{
			uint8_t key = payload[0];
			for (int i = 0; i < 6; i++) {
				if (g_kb_keys[i] == key) {
					g_kb_keys[i] = 0;
					break;
				}
			}
			out->kb_modifier = g_kb_modifier;
			memcpy(out->kb_keys, g_kb_keys, 6);
			out->has_keyboard = true;
		}
		set_ok_response(out, cmd);
		return true;

	case MAKCU_CMD_KB_PRESS:
		// [key:u8, hold_ms:u16?, rand_ms:u16?]
		// Press key — caller handles timed release
		if (len < 1) return false;
		{
			uint8_t key = payload[0];
			for (int i = 0; i < 6; i++) {
				if (g_kb_keys[i] == key) break;
				if (g_kb_keys[i] == 0) {
					g_kb_keys[i] = key;
					break;
				}
			}
			out->kb_modifier = g_kb_modifier;
			memcpy(out->kb_keys, g_kb_keys, 6);
			out->has_keyboard = true;
		}
		set_ok_response(out, cmd);
		return true;

	case MAKCU_CMD_KB_INIT:
		// Clear all keyboard state
		g_kb_modifier = 0;
		memset(g_kb_keys, 0, sizeof(g_kb_keys));
		out->kb_modifier = 0;
		memset(out->kb_keys, 0, 6);
		out->has_keyboard = true;
		set_ok_response(out, cmd);
		return true;

	case MAKCU_CMD_KB_STREAM:
		// [modifiers:u8, keys[14]]  — full keyboard state injection
		if (len < 1) return false;
		g_kb_modifier = payload[0];
		memset(g_kb_keys, 0, sizeof(g_kb_keys));
		// Copy up to 6 keys (boot protocol limit) from up to 14 available
		for (int i = 0; i < 6 && (i + 1) < (int)len; i++)
			g_kb_keys[i] = payload[i + 1];
		out->kb_modifier = g_kb_modifier;
		memcpy(out->kb_keys, g_kb_keys, 6);
		out->has_keyboard = true;
		set_ok_response(out, cmd);
		return true;

	// ---- System commands ----

	case MAKCU_CMD_VERSION:
		// Getter: return version string
		out->needs_response = true;
		out->resp_cmd = cmd;
		{
			// "3.9" — matches Makcu firmware version
			static const uint8_t ver[] = { '3', '.', '9' };
			memcpy(out->resp_payload, ver, sizeof(ver));
			out->resp_payload_len = sizeof(ver);
		}
		return true;

	case MAKCU_CMD_INFO:
		// Getter: return minimal device info
		out->needs_response = true;
		out->resp_cmd = cmd;
		{
			static const uint8_t info[] = { 'i', 'm', 'x', 'r', 't' };
			memcpy(out->resp_payload, info, sizeof(info));
			out->resp_payload_len = sizeof(info);
		}
		return true;

	case MAKCU_CMD_BAUD:
		// Getter (len==0) or setter (len==4)
		// We acknowledge but don't change baud at runtime
		if (len == 0) {
			// Return current baud as u32 LE (115200 = 0x0001C200)
			out->needs_response = true;
			out->resp_cmd = cmd;
			out->resp_payload[0] = 0x00;
			out->resp_payload[1] = 0xC2;
			out->resp_payload[2] = 0x01;
			out->resp_payload[3] = 0x00;
			out->resp_payload_len = 4;
		} else {
			set_ok_response(out, cmd);
		}
		return true;

	case MAKCU_CMD_REBOOT:
		// Acknowledge then let caller handle actual reboot if desired
		set_ok_response(out, cmd);
		return true;

	case MAKCU_CMD_DEVICE:
		// Return active device type — 0x01 = mouse+keyboard composite
		out->needs_response = true;
		out->resp_cmd = cmd;
		out->resp_payload[0] = 0x01;
		out->resp_payload_len = 1;
		return true;

	case MAKCU_CMD_ECHO:
	case MAKCU_CMD_LOG:
	case MAKCU_CMD_LED:
	case MAKCU_CMD_BYPASS:
	case MAKCU_CMD_HS:
	case MAKCU_CMD_RELEASE:
	case MAKCU_CMD_SCREEN:
	case MAKCU_CMD_SERIAL:
	case MAKCU_CMD_FAULT:
		// Acknowledge but no action needed
		set_ok_response(out, cmd);
		return true;

	case MAKCU_CMD_KB_DISABLE:
	case MAKCU_CMD_KB_ISDOWN:
	case MAKCU_CMD_KB_MASK:
	case MAKCU_CMD_KB_REMAP:
	case MAKCU_CMD_KB_STRING:
		// Acknowledged but not fully implemented
		set_ok_response(out, cmd);
		return true;

	default:
		return false;
	}
}
