#pragma once
#include <stdint.h>
#include <stdbool.h>

// Smooth injection queue for Cortex-M7
// Sub-pixel 16.16 fixed-point accumulation, temporal spreading with
// ease-in-out cubic easing, humanization tremor integration.
// Runs comfortably at 8kHz (< 300 cycles/frame).

#define SMOOTH_FP_SHIFT  16
#define SMOOTH_FP_ONE    (1 << SMOOTH_FP_SHIFT)
#define SMOOTH_FP_HALF   (1 << (SMOOTH_FP_SHIFT - 1))

#define SMOOTH_QUEUE_SIZE 32

// Initialize smooth injection (calls humanize_init)
void smooth_init(void);

// Enqueue a movement for temporal spreading
void smooth_inject(int16_t x, int16_t y);

// Process one frame: drain queue, apply easing + humanization,
// output integer deltas with sub-pixel remainder preserved.
void smooth_process_frame(int16_t *out_x, int16_t *out_y);

// Check if queue has active entries
bool smooth_has_pending(void);

// Flush all pending queue entries
void smooth_clear(void);

// Configure per-frame rate limit (default 127)
void smooth_set_max_per_frame(int16_t max);
