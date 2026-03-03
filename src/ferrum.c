// Ferrum KM API text protocol parser
// Ported from RaspberryKMBox bridge/ferrum_translator.c
// Parses km.move(x,y), km.left(state), etc.
// Produces injection commands for the kmbox inject state.

#include "ferrum.h"
#include <string.h>

// HID button masks (same as kmbox)
#define BTN_LEFT    0x01
#define BTN_RIGHT   0x02
#define BTN_MIDDLE  0x04
#define BTN_BACK    0x08
#define BTN_FORWARD 0x10

// Persistent button state (sticky press/release)
static uint8_t g_buttons;

// ---- Helpers ----

static bool parse_int(const char **p, int32_t *out)
{
	while (**p == ' ' || **p == '\t') (*p)++;
	if (**p == '\0' || **p == ')') return false;

	bool neg = false;
	if (**p == '-') { neg = true; (*p)++; }
	else if (**p == '+') { (*p)++; }

	if (**p < '0' || **p > '9') return false;

	int32_t val = 0;
	while (**p >= '0' && **p <= '9') {
		val = val * 10 + (**p - '0');
		(*p)++;
	}
	*out = neg ? -val : val;
	return true;
}

static void skip_sep(const char **p)
{
	while (**p == ' ' || **p == '\t' || **p == ',') (*p)++;
}

static bool starts_with(const char *s, const char *prefix)
{
	while (*prefix) {
		if (*s++ != *prefix++) return false;
	}
	return true;
}

// ---- Public API ----

void ferrum_init(void)
{
	g_buttons = 0;
}

bool ferrum_parse_line(const char *line, uint8_t len, ferrum_result_t *out)
{
	memset(out, 0, sizeof(*out));

	if (len < 7 || !starts_with(line, "km.")) return false;
	const char *p = line + 3;

	// km.move(x, y)
	if (starts_with(p, "move(")) {
		p += 5;
		int32_t x, y;
		if (!parse_int(&p, &x)) return false;
		skip_sep(&p);
		if (!parse_int(&p, &y)) return false;
		if (*p != ')') return false;

		out->mouse_dx = (int16_t)x;
		out->mouse_dy = (int16_t)y;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// km.wheel(amount)
	if (starts_with(p, "wheel(")) {
		p += 6;
		int32_t w;
		if (!parse_int(&p, &w)) return false;
		if (*p != ')') return false;

		out->mouse_wheel = (int8_t)w;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// Button commands: km.left(state), km.right(state), etc.
	struct { const char *name; uint8_t nlen; uint8_t mask; } btns[] = {
		{ "left(",   5, BTN_LEFT },
		{ "right(",  6, BTN_RIGHT },
		{ "middle(", 7, BTN_MIDDLE },
		{ "side1(",  6, BTN_BACK },
		{ "side2(",  6, BTN_FORWARD },
	};

	for (uint8_t i = 0; i < 5; i++) {
		if (starts_with(p, btns[i].name)) {
			p += btns[i].nlen;
			int32_t st;
			if (!parse_int(&p, &st)) return false;
			if (*p != ')') return false;

			if (st != 0)
				g_buttons |= btns[i].mask;
			else
				g_buttons &= ~btns[i].mask;

			out->mouse_buttons = g_buttons;
			out->has_mouse = true;
			out->needs_response = true;
			return true;
		}
	}

	// Keyboard commands — recognized but unsupported
	if (starts_with(p, "down(") || starts_with(p, "up(") ||
	    starts_with(p, "press(") || starts_with(p, "multidown(") ||
	    starts_with(p, "multiup(") || starts_with(p, "multipress(")) {
		out->has_keyboard = true;
		return false;
	}

	return false;
}
