#pragma once
#include <stdint.h>
#include <stdbool.h>

// Ferrum KM API text protocol parser
// Format: km.command(arg1, arg2)\n  Response: >>>\r\n
// Supported: move, left, right, middle, side1, side2, wheel
// Keyboard commands are unsupported (returns false).

#define FERRUM_MAX_LINE 128

// Result of parsing a Ferrum text line
typedef struct {
	int16_t  mouse_dx;
	int16_t  mouse_dy;
	uint8_t  mouse_buttons;  // full button mask after applying command
	int8_t   mouse_wheel;
	bool     has_mouse;      // true if mouse state changed
	bool     has_keyboard;   // true if keyboard command (unsupported)
	bool     needs_response; // true if >>> response should be sent
	bool     click_release;  // true = schedule button release after click
	// Optional text response (for km.version, km.isdown, etc.)
	const char *text_response; // NULL = no extra text before >>>
} ferrum_result_t;

// Initialize Ferrum parser (resets button state)
void ferrum_init(void);

// Parse a complete text line (without trailing \n or \r\n).
// Returns true if the line was a recognized command.
bool ferrum_parse_line(const char *line, uint8_t len, ferrum_result_t *out);
