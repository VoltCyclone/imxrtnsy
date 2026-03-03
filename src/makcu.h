#pragma once
#include <stdint.h>
#include <stdbool.h>

// Makcu binary protocol parser
// Frame: [0x50] [cmd] [len_lo] [len_hi] [payload...]
// Supported mouse commands: MOVE, MO, CLICK, WHEEL, button controls
// Keyboard/misc commands are unsupported.

#define MAKCU_SYNC         0x50
#define MAKCU_MAX_PAYLOAD  64   // we only handle mouse commands, 64 is plenty

// Makcu mouse command codes
#define MAKCU_CMD_MOVE          0x0D
#define MAKCU_CMD_MO            0x0B
#define MAKCU_CMD_CLICK         0x04
#define MAKCU_CMD_WHEEL         0x18
#define MAKCU_CMD_LEFT_BUTTON   0x08
#define MAKCU_CMD_RIGHT_BUTTON  0x11
#define MAKCU_CMD_MIDDLE_BUTTON 0x0A
#define MAKCU_CMD_SIDE1_BUTTON  0x12
#define MAKCU_CMD_SIDE2_BUTTON  0x13

// Result of parsing a Makcu command
typedef struct {
	int16_t  mouse_dx;
	int16_t  mouse_dy;
	uint8_t  mouse_buttons;
	int8_t   mouse_wheel;
	bool     has_mouse;
} makcu_result_t;

// Initialize Makcu parser (resets button state)
void makcu_init(void);

// Parse a complete Makcu command (cmd + payload, after sync+header already stripped).
// Returns true if the command was recognized and produced mouse output.
bool makcu_parse_command(uint8_t cmd, const uint8_t *payload, uint16_t len,
                         makcu_result_t *out);
