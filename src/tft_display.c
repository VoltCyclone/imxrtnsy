// tft_display.c — Stats rendering for USB proxy TFT display
// Renders proxy metrics onto ST7735 128x160 framebuffer
// Layout: 21 chars x 20 lines (6x8 font)

#include "tft_display.h"
#include "tft.h"
#include <string.h>

// ---- Formatting helpers (no snprintf, no libc) ----

static char *u32_to_str(char *buf, uint32_t v)
{
	char tmp[10];
	int i = 0;
	if (v == 0) { *buf++ = '0'; return buf; }
	while (v) { tmp[i++] = '0' + (v % 10); v /= 10; }
	while (i--) *buf++ = tmp[i];
	return buf;
}

static char *u32_to_str02(char *buf, uint32_t v)
{
	*buf++ = '0' + (v / 10) % 10;
	*buf++ = '0' + v % 10;
	return buf;
}

static int fmt_done(char *start, char *end)
{
	*end = '\0';
	return (int)(end - start);
}

static char *u16_to_hex4(char *buf, uint16_t v)
{
	static const char hex[] = "0123456789ABCDEF";
	*buf++ = hex[(v >> 12) & 0xF];
	*buf++ = hex[(v >> 8) & 0xF];
	*buf++ = hex[(v >> 4) & 0xF];
	*buf++ = hex[v & 0xF];
	return buf;
}

static char *i8_to_str(char *buf, int8_t v)
{
	if (v < 0) { *buf++ = '-'; v = -v; }
	if (v >= 100) { *buf++ = '0' + v / 100; v %= 100; }
	if (v >= 10 || buf[-1] != '-') *buf++ = '0' + v / 10;
	*buf++ = '0' + v % 10;
	return buf;
}

// ---- Layout constants ----
#define FW   TFT_FONT_W
#define FH   TFT_FONT_H
#define LINE(n)  ((n) * FH)
#define COL(n)   ((n) * FW)

static void draw_separator(int y)
{
	tft_draw_hline(2, TFT_WIDTH - 3, y + FH / 2, COL_DIM);
}

// ---- Stats rendering ----
static void draw_stats(const tft_proxy_stats_t *s)
{
	char buf[24];
	char *p;

	tft_fill(COL_BG);

	// Line 0: Title
	tft_draw_string(COL(7), LINE(0), COL_CYAN, "IMXRTNSY");

	// Line 1: separator
	draw_separator(LINE(1));

	// Line 2: Host + Device status
	tft_draw_string(COL(0), LINE(2), COL_GRAY, "Host:");
	tft_draw_string(COL(5), LINE(2),
		s->host_connected ? COL_GREEN : COL_RED,
		s->host_connected ? "OK" : "--");
	tft_draw_string(COL(9), LINE(2), COL_GRAY, "Dev:");
	tft_draw_string(COL(13), LINE(2),
		s->device_configured ? COL_GREEN : COL_RED,
		s->device_configured ? "OK" : "--");

	// Line 3: Protocol + endpoints
	{
		const char *proto[] = { "None", "KMBox", "Makcu", "Ferrum" };
		uint8_t pm = s->protocol_mode;
		if (pm > 3) pm = 0;
		tft_draw_string(COL(0), LINE(3), COL_GRAY, "Proto:");
		tft_draw_string(COL(6), LINE(3),
			s->kmbox_active ? COL_GREEN : COL_DARK,
			proto[pm]);
	}

	// Line 4: Speed + EPs
	{
		const char *spd[] = { "Full", "Low", "High" };
		uint8_t sp = s->device_speed;
		if (sp > 2) sp = 0;
		tft_draw_string(COL(0), LINE(4), COL_GRAY, "Spd:");
		tft_draw_string(COL(4), LINE(4), COL_WHITE, spd[sp]);
		tft_draw_string(COL(12), LINE(4), COL_GRAY, "EPs:");
		p = buf;
		p = u32_to_str(p, s->num_endpoints);
		fmt_done(buf, p);
		tft_draw_string(COL(16), LINE(4), COL_WHITE, buf);
	}

	// Line 5: separator
	draw_separator(LINE(5));

	// Line 6: Report count
	tft_draw_string(COL(0), LINE(6), COL_GRAY, "Reports:");
	p = buf;
	p = u32_to_str(p, s->report_count);
	fmt_done(buf, p);
	tft_draw_string(COL(9), LINE(6), COL_WHITE, buf);

	// Line 7: Rate
	tft_draw_string(COL(0), LINE(7), COL_GRAY, "Rate:");
	p = buf;
	p = u32_to_str(p, s->reports_per_sec);
	*p++ = '/'; *p++ = 's';
	fmt_done(buf, p);
	tft_draw_string(COL(5), LINE(7), COL_GREEN, buf);

	// Line 8: Drops
	tft_draw_string(COL(0), LINE(8), COL_GRAY, "Drops:");
	p = buf;
	p = u32_to_str(p, s->drop_count);
	fmt_done(buf, p);
	tft_draw_string(COL(6), LINE(8),
		s->drop_count > 0 ? COL_RED : COL_DARK, buf);

	// Line 9: KMBox frames
	tft_draw_string(COL(0), LINE(9), COL_GRAY, "KMBox:");
	p = buf;
	p = u32_to_str(p, s->kmbox_frames_ok);
	*p++ = ' '; *p++ = 'o'; *p++ = 'k';
	fmt_done(buf, p);
	tft_draw_string(COL(6), LINE(9), COL_WHITE, buf);

	if (s->kmbox_frames_err > 0) {
		tft_draw_string(COL(14), LINE(9), COL_GRAY, "E:");
		p = buf;
		p = u32_to_str(p, s->kmbox_frames_err);
		fmt_done(buf, p);
		tft_draw_string(COL(16), LINE(9), COL_RED, buf);
	}

	// Line 10: separator
	draw_separator(LINE(10));

	// Line 11: Smooth injection
	tft_draw_string(COL(0), LINE(11), COL_GRAY, "Smooth:");
	if (s->smooth_active) {
		uint8_t filled = 0;
		if (s->smooth_queue_max > 0)
			filled = (s->smooth_queue_depth * 8) / s->smooth_queue_max;
		buf[0] = '[';
		for (int i = 0; i < 8; i++)
			buf[1 + i] = (i < filled) ? '=' : ' ';
		buf[9] = ']';
		p = buf + 10;
		p = u32_to_str(p, s->smooth_queue_depth);
		fmt_done(buf, p);
		tft_draw_string(COL(7), LINE(11), COL_YELLOW, buf);
	} else {
		tft_draw_string(COL(7), LINE(11), COL_DARK, "idle");
	}

	// Line 12: separator
	draw_separator(LINE(12));

	// Line 13: UART RX activity
	tft_draw_string(COL(0), LINE(13), COL_GRAY, "UART RX:");
	if (s->uart_rx_bytes > 0) {
		p = buf;
		p = u32_to_str(p, s->uart_rx_bytes);
		*p++ = 'B';
		fmt_done(buf, p);
		tft_draw_string(COL(9), LINE(13), COL_GREEN, buf);
	} else {
		tft_draw_string(COL(9), LINE(13), COL_RED, "no data");
	}

	// Line 14: separator
	draw_separator(LINE(14));

	// Line 15: Uptime + CPU temp
	{
		uint32_t sec = s->uptime_sec;
		uint32_t hr  = sec / 3600;
		uint32_t min = (sec / 60) % 60;
		uint32_t s2  = sec % 60;
		tft_draw_string(COL(0), LINE(15), COL_GRAY, "Up:");
		p = buf;
		p = u32_to_str02(p, hr);
		*p++ = ':';
		p = u32_to_str02(p, min);
		*p++ = ':';
		p = u32_to_str02(p, s2);
		fmt_done(buf, p);
		tft_draw_string(COL(3), LINE(15), COL_WHITE, buf);

		tft_draw_string(COL(14), LINE(15), COL_GRAY, "CPU:");
		p = buf;
		p = i8_to_str(p, s->cpu_temp_c);
		*p++ = 'C';
		fmt_done(buf, p);
		uint8_t tcol = (s->cpu_temp_c > 80) ? COL_RED :
		               (s->cpu_temp_c > 60) ? COL_YELLOW : COL_GREEN;
		tft_draw_string(COL(18), LINE(15), tcol, buf);
	}

	// Line 16: separator
	draw_separator(LINE(16));

	// Line 17: VID:PID
	{
		tft_draw_string(COL(0), LINE(17), COL_GRAY, "USB:");
		p = buf;
		p = u16_to_hex4(p, s->usb_vid);
		*p++ = ':';
		p = u16_to_hex4(p, s->usb_pid);
		fmt_done(buf, p);
		tft_draw_string(COL(4), LINE(17), COL_WHITE, buf);
	}

	// Line 18-19: Product name
	if (s->usb_product[0]) {
		tft_draw_string(COL(0), LINE(18), COL_DARK, s->usb_product);
	}
}

// ---- Public API ----

void tft_display_init(void)
{
	tft_init();

	// Splash screen
	tft_fill(COL_BG);
	tft_draw_string(COL(7), LINE(8), COL_CYAN, "IMXRTNSY");
	tft_draw_string(COL(4), LINE(10), COL_GRAY, "USB HID Proxy");
	tft_draw_string(COL(6), LINE(12), COL_DARK, "ST7735 TFT");
	tft_swap_sync();
}

void tft_display_update(const tft_proxy_stats_t *stats)
{
	draw_stats(stats);
	tft_swap_sync();
}

void tft_display_error(const char *msg)
{
	tft_fill(COL_BG);
	tft_draw_string(COL(8), LINE(8), COL_RED, "ERROR");

	int y = LINE(10);
	int max_chars = TFT_WIDTH / FW;
	while (*msg && y < TFT_HEIGHT - FH) {
		int len = 0;
		const char *p = msg;
		while (*p && len < max_chars) { p++; len++; }
		for (int i = 0; i < len; i++)
			tft_draw_glyph(i * FW, y, COL_WHITE, msg[i]);
		msg += len;
		y += FH;
	}
	tft_swap_sync();
}
