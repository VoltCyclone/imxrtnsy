#pragma once
#include <stdint.h>
#include <stdbool.h>

// Smooth injection queue for Cortex-M7
// Sub-pixel 16.16 fixed-point accumulation, temporal spreading with
// asymmetric easing, session-level humanization model.
// Tick-rate independent: pass actual interval_us to smooth_init.
// Tunable parameters in smooth_config.h.

#define SMOOTH_FP_SHIFT  16
#define SMOOTH_FP_ONE    (1 << SMOOTH_FP_SHIFT)
#define SMOOTH_FP_HALF   (1 << (SMOOTH_FP_SHIFT - 1))

#define SMOOTH_QUEUE_SIZE 32

// Initialize smooth injection.
// interval_us: PIT tick interval in microseconds (e.g. 1000 for 1kHz, 125 for 8kHz)
void smooth_init(uint32_t interval_us);

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

// Enable or disable humanization (noise, arc bias, velocity smoothing).
// When disabled, easing curve still applies but no noise/personality is added.
// Enabled by default after smooth_init().
void smooth_set_humanize(bool enabled);

// ---- Timing humanization (call from PIT ISR) ----
// Computes the next PIT reload value with jitter and occasional poll skips.
// Returns the LDVAL to write to PIT_LDVAL0.
// When skip=true is written to *out_skip, the caller should suppress
// smooth_process_frame() for this tick (simulates a missed USB poll).
// base_ldval: the nominal PIT LDVAL computed at init.
// Must be called exactly once per PIT interrupt — maintains internal state.
uint32_t smooth_timing_next(uint32_t base_ldval, bool *out_skip);
