#pragma once
#include <stdint.h>
#include <stdbool.h>

// FPU-based humanization tremor generator for Cortex-M7
// Layered incommensurate sine oscillators in the 8-25Hz physiological
// tremor band plus LFSR noise. No LUTs — computed inline on VFPv5.

// Initialize LFSR from DWT->CYCCNT
void humanize_init(void);

// Generate one tremor sample. scale controls magnitude.
// dx/dy receive perpendicular tremor offsets (clamped to +/-3.0 px).
void humanize_get_tremor(float scale, float *dx, float *dy);

// Movement-dependent jitter scale (piecewise linear).
// Small moves -> 1.8x tremor, large moves -> 0.4x floor.
static inline float humanize_jitter_scale(float magnitude)
{
	if (magnitude < 20.0f) return 1.8f;
	if (magnitude < 60.0f) return 1.8f - (magnitude - 20.0f) * 0.015f;
	if (magnitude < 120.0f) return 1.2f - (magnitude - 60.0f) * 0.00833f;
	float s = 0.7f - (magnitude - 120.0f) * 0.0015f;
	return s < 0.4f ? 0.4f : s;
}
