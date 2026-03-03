// Smooth injection queue for Cortex-M7
// Ported from RaspberryKMBox smooth_injection.c — stripped to essentials:
// no Pico SDK, no flash, no multicore, no linked-list.
// Simple circular queue + inline easing + FPU humanization.

#include "smooth.h"
#include "humanize.h"
#include <string.h>
#include <math.h>

// ---- Fixed-point helpers ----

static inline int32_t int_to_fp(int16_t v)
{
	return (int32_t)v << SMOOTH_FP_SHIFT;
}

static inline int16_t fp_to_int(int32_t fp)
{
	return (int16_t)((fp + SMOOTH_FP_HALF) >> SMOOTH_FP_SHIFT);
}

static inline int32_t fp_mul(int32_t a, int32_t b)
{
	return (int32_t)(((int64_t)a * b) >> SMOOTH_FP_SHIFT);
}

static inline int32_t fp_div(int32_t a, int32_t b)
{
	if (b == 0) return 0;
	return (int32_t)((float)a / (float)b * (float)SMOOTH_FP_ONE);
}

// ---- Ease-in-out cubic (inline, no LUT) ----
// t in [0, FP_ONE] -> eased value in [0, FP_ONE]
static inline int32_t ease_in_out_cubic(int32_t t)
{
	if (t < SMOOTH_FP_HALF) {
		int32_t t2 = fp_mul(t, t);
		return fp_mul(t2, t) * 4;
	}
	int32_t p = SMOOTH_FP_ONE - t;
	int32_t p2 = fp_mul(p, p);
	return SMOOTH_FP_ONE - fp_mul(p2, p) * 4;
}

// ---- Queue entry ----
typedef struct {
	int32_t x_remaining_fp;
	int32_t y_remaining_fp;
	uint8_t frames_left;
	uint8_t total_frames;
	bool    active;
} queue_entry_t;

// ---- State ----
static struct {
	int32_t x_accum_fp;
	int32_t y_accum_fp;
	queue_entry_t queue[SMOOTH_QUEUE_SIZE];
	uint8_t head;
	uint8_t count;
	int16_t max_per_frame;
} state;

// ---- Frame count heuristic ----
// Small moves = more frames (precise), large = fewer (ballistic)
static uint8_t compute_spread_frames(int32_t abs_x_fp, int32_t abs_y_fp)
{
	int32_t max_comp = abs_x_fp > abs_y_fp ? abs_x_fp : abs_y_fp;
	float px = (float)max_comp / (float)SMOOTH_FP_ONE;

	if (px < 20.0f) return 8;
	if (px < 60.0f) return 5;
	if (px < 120.0f) return 3;
	return 2;
}

// ---- Public API ----

void smooth_init(void)
{
	memset(&state, 0, sizeof(state));
	state.max_per_frame = 127;
	humanize_init();
}

void smooth_inject(int16_t x, int16_t y)
{
	if (x == 0 && y == 0) return;

	// Find a free slot
	if (state.count >= SMOOTH_QUEUE_SIZE) {
		// Queue full — fall back to immediate injection
		state.x_accum_fp += int_to_fp(x);
		state.y_accum_fp += int_to_fp(y);
		return;
	}

	// Scan for an inactive slot
	uint8_t slot = 0;
	for (uint8_t i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
		if (!state.queue[i].active) {
			slot = i;
			break;
		}
	}

	int32_t xfp = int_to_fp(x);
	int32_t yfp = int_to_fp(y);
	int32_t ax = xfp >= 0 ? xfp : -xfp;
	int32_t ay = yfp >= 0 ? yfp : -yfp;

	uint8_t frames = compute_spread_frames(ax, ay);

	state.queue[slot].x_remaining_fp = xfp;
	state.queue[slot].y_remaining_fp = yfp;
	state.queue[slot].frames_left    = frames;
	state.queue[slot].total_frames   = frames;
	state.queue[slot].active         = true;
	state.count++;
}

void smooth_process_frame(int16_t *out_x, int16_t *out_y)
{
	// Process all active queue entries
	for (uint8_t i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
		queue_entry_t *e = &state.queue[i];
		if (!e->active) continue;
		if (e->frames_left == 0) {
			e->active = false;
			if (state.count > 0) state.count--;
			continue;
		}

		// Compute eased progress step
		// progress = 1 - frames_left / total_frames
		int32_t progress = SMOOTH_FP_ONE -
			fp_div(int_to_fp((int16_t)e->frames_left),
			       int_to_fp((int16_t)e->total_frames));
		int32_t next_progress = SMOOTH_FP_ONE -
			fp_div(int_to_fp((int16_t)(e->frames_left - 1)),
			       int_to_fp((int16_t)e->total_frames));

		int32_t eased_now  = ease_in_out_cubic(progress);
		int32_t eased_next = ease_in_out_cubic(next_progress);
		int32_t step_frac  = eased_next - eased_now;

		// Original total was remaining / (remaining progress)
		int32_t remaining_progress = SMOOTH_FP_ONE - eased_now;
		int32_t frame_dx, frame_dy;
		if (remaining_progress > 0) {
			frame_dx = fp_mul(e->x_remaining_fp,
				fp_div(step_frac, remaining_progress));
			frame_dy = fp_mul(e->y_remaining_fp,
				fp_div(step_frac, remaining_progress));
		} else {
			frame_dx = e->x_remaining_fp;
			frame_dy = e->y_remaining_fp;
		}

		e->x_remaining_fp -= frame_dx;
		e->y_remaining_fp -= frame_dy;
		e->frames_left--;

		// On last frame, flush any rounding remainder
		if (e->frames_left == 0) {
			frame_dx += e->x_remaining_fp;
			frame_dy += e->y_remaining_fp;
			e->x_remaining_fp = 0;
			e->y_remaining_fp = 0;
			e->active = false;
			if (state.count > 0) state.count--;
		}

		// Add humanization tremor scaled by movement magnitude
		float mag = sqrtf((float)fp_mul(frame_dx, frame_dx) +
		                  (float)fp_mul(frame_dy, frame_dy))
		            / (float)SMOOTH_FP_ONE;
		float scale = humanize_jitter_scale(mag);
		float tdx, tdy;
		humanize_get_tremor(scale, &tdx, &tdy);

		// Add tremor as fixed-point
		frame_dx += (int32_t)(tdx * (float)SMOOTH_FP_ONE);
		frame_dy += (int32_t)(tdy * (float)SMOOTH_FP_ONE);

		// Accumulate sub-pixel
		state.x_accum_fp += frame_dx;
		state.y_accum_fp += frame_dy;
	}

	// Extract integer part, keep fractional remainder
	int16_t ix = fp_to_int(state.x_accum_fp);
	int16_t iy = fp_to_int(state.y_accum_fp);

	state.x_accum_fp -= int_to_fp(ix);
	state.y_accum_fp -= int_to_fp(iy);

	// Clamp to max_per_frame
	if (ix > state.max_per_frame) ix = state.max_per_frame;
	if (ix < -state.max_per_frame) ix = -state.max_per_frame;
	if (iy > state.max_per_frame) iy = state.max_per_frame;
	if (iy < -state.max_per_frame) iy = -state.max_per_frame;

	*out_x = ix;
	*out_y = iy;
}

bool smooth_has_pending(void)
{
	return state.count > 0 ||
	       state.x_accum_fp > SMOOTH_FP_HALF ||
	       state.x_accum_fp < -SMOOTH_FP_HALF ||
	       state.y_accum_fp > SMOOTH_FP_HALF ||
	       state.y_accum_fp < -SMOOTH_FP_HALF;
}

void smooth_clear(void)
{
	for (uint8_t i = 0; i < SMOOTH_QUEUE_SIZE; i++)
		state.queue[i].active = false;
	state.count = 0;
	state.x_accum_fp = 0;
	state.y_accum_fp = 0;
}

void smooth_set_max_per_frame(int16_t max)
{
	if (max < 1) max = 1;
	state.max_per_frame = max;
}
