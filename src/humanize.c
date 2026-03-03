// FPU-based humanization tremor for Cortex-M7 (VFPv5-D16)
// Ported from RaspberryKMBox humanization_fpu.c — no Pico SDK deps.
//
// 3 incommensurate X oscillators (8.7, 12.3, 19.1 Hz)
// 3 incommensurate Y oscillators (9.4, 13.7, 17.8 Hz)
// + Galois LFSR xorshift32 noise at 0.3 weight
// Phase wraps at 100K to prevent float32 precision loss.
// Clamp output to +/-3.0 px.

#include "humanize.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---- LFSR state ----
static uint32_t g_jitter_lfsr = 0xDEADBEEF;
static uint32_t g_tremor_phase = 0;

// ---- LFSR noise [-1.0, 1.0] ----
static inline float jitter_next(void)
{
	g_jitter_lfsr ^= g_jitter_lfsr << 13;
	g_jitter_lfsr ^= g_jitter_lfsr >> 17;
	g_jitter_lfsr ^= g_jitter_lfsr << 5;
	int32_t balanced = (int32_t)(g_jitter_lfsr >> 8) - 0x800000;
	return (float)balanced * (1.0f / 8388608.0f);
}

// ---- Fast sine (degree-5 minimax, max err ~2.5e-5) ----
static const float TWO_PI  = 6.28318530718f;
static const float INV_2PI = 0.15915494309f;

static inline float fast_sinf(float x)
{
	float n = x * INV_2PI;
	n = (float)(int32_t)(n + (n >= 0.0f ? 0.5f : -0.5f));
	x -= n * TWO_PI;

	float x2 = x * x;
	float r = fmaf(x2, -1.984126984e-4f, 8.333333333e-3f);
	r = fmaf(x2, r, -1.666666667e-1f);
	r = fmaf(x2, r, 1.0f);
	return x * r;
}

// ---- Angular frequency constants (2*PI*freq) ----
static const float W_X1 = 8.7f  * (2.0f * (float)M_PI);
static const float W_X2 = 12.3f * (2.0f * (float)M_PI);
static const float W_X3 = 19.1f * (2.0f * (float)M_PI);
static const float W_Y1 = 9.4f  * (2.0f * (float)M_PI);
static const float W_Y2 = 13.7f * (2.0f * (float)M_PI);
static const float W_Y3 = 17.8f * (2.0f * (float)M_PI);

// ---- Public API ----

void humanize_init(void)
{
	// Seed from DWT cycle counter (always running on Cortex-M7)
	volatile uint32_t *dwt_cyccnt = (volatile uint32_t *)0xE0001004;
	volatile uint32_t *dwt_ctrl   = (volatile uint32_t *)0xE0001000;

	// Enable DWT cycle counter if not already enabled
	*dwt_ctrl |= 1;
	uint32_t seed = *dwt_cyccnt;
	if (seed == 0) seed = 0xDEADBEEF;

	g_jitter_lfsr = seed;
	g_tremor_phase = seed >> 16;
}

void humanize_get_tremor(float scale, float *dx, float *dy)
{
	g_tremor_phase++;
	float t = (float)(g_tremor_phase % 100000u) * 0.001f;

	// X-axis: 3 incommensurate oscillators + noise
	float tx = t + 0.7f;
	float tremor_x = fast_sinf(tx * W_X1) * 0.40f
	               + fast_sinf(tx * W_X2) * 0.25f
	               + fast_sinf(tx * W_X3) * 0.15f;
	tremor_x = fmaf(jitter_next(), 0.3f, tremor_x);

	// Y-axis: different offset for decorrelation
	float ty = t + 1.3f;
	float tremor_y = fast_sinf(ty * W_Y1) * 0.40f
	               + fast_sinf(ty * W_Y2) * 0.25f
	               + fast_sinf(ty * W_Y3) * 0.15f;
	tremor_y = fmaf(jitter_next(), 0.3f, tremor_y);

	*dx = tremor_x * scale;
	*dy = tremor_y * scale;

	if (*dx >  3.0f) *dx =  3.0f;
	if (*dx < -3.0f) *dx = -3.0f;
	if (*dy >  3.0f) *dy =  3.0f;
	if (*dy < -3.0f) *dy = -3.0f;
}
