// Ferrum KM API text protocol parser
// Parses km.move(x,y), km.left(state), km.version(), etc.
// Produces injection commands for the kmbox inject state.
// Supports both parenthesized km.move(x,y) and space-separated km.move x,y formats.

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

// Skip optional opening paren or space (supports both km.move(x,y) and km.move x,y)
static void skip_open(const char **p)
{
	if (**p == '(') (*p)++;
	while (**p == ' ' || **p == '\t') (*p)++;
}

// ---- Public API ----

void ferrum_init(void)
{
	g_buttons = 0;
}

bool ferrum_parse_line(const char *line, uint8_t len, ferrum_result_t *out)
{
	memset(out, 0, sizeof(*out));

	if (len < 4 || !starts_with(line, "km.")) return false;
	const char *p = line + 3;

	// km.move(x, y) or km.move x,y
	if (starts_with(p, "move(") || starts_with(p, "move ")) {
		p += 4;
		skip_open(&p);
		int32_t x, y;
		if (!parse_int(&p, &x)) return false;
		skip_sep(&p);
		if (!parse_int(&p, &y)) return false;

		out->mouse_dx = (int16_t)x;
		out->mouse_dy = (int16_t)y;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// km.moveto(x, y) — treated as relative since we have no absolute position tracking
	if (starts_with(p, "moveto(") || starts_with(p, "moveto ")) {
		p += 6;
		skip_open(&p);
		int32_t x, y;
		if (!parse_int(&p, &x)) return false;
		skip_sep(&p);
		if (!parse_int(&p, &y)) return false;

		out->mouse_dx = (int16_t)x;
		out->mouse_dy = (int16_t)y;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// km.wheel(amount)
	if (starts_with(p, "wheel(") || starts_with(p, "wheel ")) {
		p += 5;
		skip_open(&p);
		int32_t w;
		if (!parse_int(&p, &w)) return false;

		out->mouse_wheel = (int8_t)w;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// km.click(button) — auto press+release
	if (starts_with(p, "click(") || starts_with(p, "click ")) {
		p += 5;
		skip_open(&p);
		int32_t btn;
		if (!parse_int(&p, &btn)) return false;

		uint8_t mask = 0;
		switch (btn) {
		case 1: mask = BTN_LEFT; break;
		case 2: mask = BTN_RIGHT; break;
		case 3: mask = BTN_MIDDLE; break;
		case 4: mask = BTN_BACK; break;
		case 5: mask = BTN_FORWARD; break;
		default: return false;
		}
		g_buttons |= mask;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->click_release = true;
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

	// km.version() or km.version — device identification handshake
	if (starts_with(p, "version")) {
		out->text_response = "kmbox: 2.0.0 Aug 31 2020 21:49:51\r\n";
		out->needs_response = true;
		return true;
	}

	// km.isdown_left() etc. — query button state
	if (starts_with(p, "isdown_left(")) {
		out->text_response = (g_buttons & BTN_LEFT) ? "1\r\n" : "0\r\n";
		out->needs_response = true;
		return true;
	}
	if (starts_with(p, "isdown_right(")) {
		out->text_response = (g_buttons & BTN_RIGHT) ? "1\r\n" : "0\r\n";
		out->needs_response = true;
		return true;
	}
	if (starts_with(p, "isdown_middle(")) {
		out->text_response = (g_buttons & BTN_MIDDLE) ? "1\r\n" : "0\r\n";
		out->needs_response = true;
		return true;
	}
	if (starts_with(p, "isdown_side1(")) {
		out->text_response = (g_buttons & BTN_BACK) ? "1\r\n" : "0\r\n";
		out->needs_response = true;
		return true;
	}
	if (starts_with(p, "isdown_side2(")) {
		out->text_response = (g_buttons & BTN_FORWARD) ? "1\r\n" : "0\r\n";
		out->needs_response = true;
		return true;
	}

	// km.monitor(enable) — acknowledged but no action (we always pass through)
	if (starts_with(p, "monitor(")) {
		out->needs_response = true;
		return true;
	}

	// km.reboot() — acknowledged
	if (starts_with(p, "reboot(")) {
		out->needs_response = true;
		return true;
	}

	// km.baud(rate) — acknowledged but baud doesn't change at runtime
	if (starts_with(p, "baud(")) {
		out->needs_response = true;
		return true;
	}

	// km.help() — return minimal help text
	if (starts_with(p, "help(")) {
		out->text_response = "move,wheel,left,right,middle,side1,side2,click,version\r\n";
		out->needs_response = true;
		return true;
	}

	// Keyboard commands — recognized, acknowledged with >>> but not injected
	if (starts_with(p, "down(") || starts_with(p, "up(") ||
	    starts_with(p, "press(") || starts_with(p, "multidown(") ||
	    starts_with(p, "multiup(") || starts_with(p, "multipress(") ||
	    starts_with(p, "init(")) {
		out->has_keyboard = true;
		out->needs_response = true;
		return true;
	}

	return false;
}
