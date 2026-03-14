#pragma once
#include <stdint.h>
#include <stdbool.h>

// Makcu binary protocol parser
// Frame: [0x50] [cmd] [len_lo] [len_hi] [payload...]
// Supported mouse commands: MOVE, MO, CLICK, WHEEL, button controls
// Keyboard/misc commands are unsupported.

#define MAKCU_SYNC         0x50
#define MAKCU_MAX_PAYLOAD  64   // we only handle mouse commands, 64 is plenty
#define MAKCU_CMD_MOVE          0x0D
#define MAKCU_CMD_MO            0x0B
#define MAKCU_CMD_CLICK         0x04
#define MAKCU_CMD_WHEEL         0x18
#define MAKCU_CMD_LEFT_BUTTON   0x08
#define MAKCU_CMD_RIGHT_BUTTON  0x11
#define MAKCU_CMD_MIDDLE_BUTTON 0x0A
#define MAKCU_CMD_SIDE1_BUTTON  0x12
#define MAKCU_CMD_SIDE2_BUTTON  0x13
#define MAKCU_CMD_KB_DISABLE    0xA1
#define MAKCU_CMD_KB_DOWN       0xA2
#define MAKCU_CMD_KB_INIT       0xA3
#define MAKCU_CMD_KB_ISDOWN     0xA4
#define MAKCU_CMD_KB_STREAM     0xA5
#define MAKCU_CMD_KB_MASK       0xA6
#define MAKCU_CMD_KB_PRESS      0xA7
#define MAKCU_CMD_KB_REMAP      0xA8
#define MAKCU_CMD_KB_STRING     0xA9
#define MAKCU_CMD_KB_UP         0xAA
#define MAKCU_CMD_BAUD          0xB1
#define MAKCU_CMD_BYPASS        0xB2
#define MAKCU_CMD_DEVICE        0xB3
#define MAKCU_CMD_ECHO          0xB4
#define MAKCU_CMD_FAULT         0xB5
#define MAKCU_CMD_HS            0xB7
#define MAKCU_CMD_INFO          0xB8
#define MAKCU_CMD_LED           0xB9
#define MAKCU_CMD_LOG           0xBA
#define MAKCU_CMD_REBOOT        0xBB
#define MAKCU_CMD_RELEASE       0xBC
#define MAKCU_CMD_SCREEN        0xBD
#define MAKCU_CMD_SERIAL        0xBE
#define MAKCU_CMD_VERSION       0xBF
typedef struct {
	int16_t  mouse_dx;
	int16_t  mouse_dy;
	uint8_t  mouse_buttons;
	int8_t   mouse_wheel;
	bool     has_mouse;
	bool     click_release;  // true = schedule button release after sending press

	uint8_t  kb_modifier;
	uint8_t  kb_keys[6];
	bool     has_keyboard;
	bool     kb_click_release; // true = schedule key release after sending press
	uint8_t  kb_release_key;   // key to release

	bool     needs_response;  // true = send [0x50][cmd][len][status] back
	uint8_t  resp_cmd;
	uint8_t  resp_payload[32];
	uint8_t  resp_payload_len;
} makcu_result_t;
void makcu_init(void);
bool makcu_parse_command(uint8_t cmd, const uint8_t *payload, uint16_t len,
                         makcu_result_t *out);
