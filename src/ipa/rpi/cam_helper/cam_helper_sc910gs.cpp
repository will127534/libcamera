/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_sc910gs.cpp - camera information for SC910GS sensor
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "cam_helper.h"

namespace RPiController {

class CamHelperSc910gs : public CamHelper
{
public:
	CamHelperSc910gs();
	uint32_t gainCode(double gain) const override;   // libcamera gain -> driver code [0..348]
	double gain(uint32_t gainCode) const override;   // driver code [0..348] -> libcamera gain
	unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	static constexpr int frameIntegrationDiff = 4;
};

namespace {

/*
 * Gain code mapping (your table):
 *   code 0..63    : ANA=0x03, FINE=0x40..0x7F, coarse=1.00
 *   code 64..92   : ANA=0x07, FINE=0x40..0x5C, coarse=2.00
 *   code 93..156  : ANA=0x23, FINE=0x40..0x7F, coarse=2.89
 *   code 157..220 : ANA=0x27, FINE=0x40..0x7F, coarse=5.78
 *   code 221..284 : ANA=0x2F, FINE=0x40..0x7F, coarse=11.56
 *   code 285..348 : ANA=0x3F, FINE=0x40..0x7F, coarse=23.12
 *
 * For all segments: gain = coarse * (fine / 64.0).
 */
static constexpr uint32_t kMinGainCode = 0;
static constexpr uint32_t kMaxGainCode = 348;

static constexpr uint8_t kFineBase = 0x40;              // 64
static constexpr uint8_t kFineMaxFull = 0x7F;           // 127
static constexpr uint8_t kFineMaxSeg07 = 0x5C;          // 92

static constexpr std::array<double, 6> kCoarse = {{
	1.00,   // ANA 0x03
	2.00,   // ANA 0x07
	2.89,   // ANA 0x23
	5.78,   // ANA 0x27
	11.56,  // ANA 0x2F
	23.12,  // ANA 0x3F
}};

static constexpr std::array<uint32_t, 6> kBaseCode = {{
	0,        // 0x03
	64,       // 0x07
	93,       // 0x23
	157,      // 0x27
	221,      // 0x2F
	285,      // 0x3F
}};

static inline uint8_t clamp_u8_int(int v, uint8_t lo, uint8_t hi)
{
	if (v < (int)lo) return lo;
	if (v > (int)hi) return hi;
	return (uint8_t)v;
}

static inline double gain_from_coarse_fine(double coarse, uint8_t fine)
{
	return coarse * (static_cast<double>(fine) / 64.0);
}

static inline void code_to_coarse_fine(uint32_t code, double *coarse, uint8_t *fine)
{
	code = std::min(code, kMaxGainCode);

	if (code < 64) {
		*coarse = kCoarse[0];
		*fine = static_cast<uint8_t>(kFineBase + code);          // 0x40..0x7F
		return;
	}

	if (code < 93) { /* 64..92 */
		*coarse = kCoarse[1];
		*fine = static_cast<uint8_t>(kFineBase + (code - 64));   // 0x40..0x5C
		return;
	}

	/* 93..348 broken into 4 blocks of 64 */
	uint32_t idx = code - 93;        // 0..255
	uint32_t seg = idx / 64;         // 0..3 -> coarse[2..5]
	uint32_t off = idx % 64;         // 0..63 -> fine 0x40..0x7F

	*coarse = kCoarse[2 + seg];
	*fine = static_cast<uint8_t>(kFineBase + off);
}

static inline uint32_t best_code_for_gain(double g)
{
	/* Evaluate nearest candidate in each segment, choose smallest error. */
	uint32_t best_code = kMinGainCode;
	double best_err = std::numeric_limits<double>::infinity();

	/* Segment 0: coarse=1.0, fine 0x40..0x7F, base 0 */
	{
		const double coarse = kCoarse[0];
		int fine_i = (int)std::llround(g * 64.0 / coarse);
		uint8_t fine = clamp_u8_int(fine_i, kFineBase, kFineMaxFull);
		uint32_t code = kBaseCode[0] + (fine - kFineBase);
		double gg = gain_from_coarse_fine(coarse, fine);
		double err = std::fabs(gg - g);
		if (err < best_err || (err == best_err && code < best_code)) {
			best_err = err;
			best_code = code;
		}
	}

	/* Segment 1: coarse=2.0, fine 0x40..0x5C, base 64 */
	{
		const double coarse = kCoarse[1];
		int fine_i = (int)std::llround(g * 64.0 / coarse);
		uint8_t fine = clamp_u8_int(fine_i, kFineBase, kFineMaxSeg07);
		uint32_t code = kBaseCode[1] + (fine - kFineBase);
		double gg = gain_from_coarse_fine(coarse, fine);
		double err = std::fabs(gg - g);
		if (err < best_err || (err == best_err && code < best_code)) {
			best_err = err;
			best_code = code;
		}
	}

	/* Segments 2..5: full fine range, bases 93/157/221/285 */
	for (int s = 2; s <= 5; ++s) {
		const double coarse = kCoarse[s];
		int fine_i = (int)std::llround(g * 64.0 / coarse);
		uint8_t fine = clamp_u8_int(fine_i, kFineBase, kFineMaxFull);
		uint32_t code = kBaseCode[s] + (fine - kFineBase);
		double gg = gain_from_coarse_fine(coarse, fine);
		double err = std::fabs(gg - g);
		if (err < best_err || (err == best_err && code < best_code)) {
			best_err = err;
			best_code = code;
		}
	}

	return std::min(best_code, kMaxGainCode);
}

static constexpr double kMinGain = 1.0; /* 0x03,0x40 */
static constexpr double kMaxGain = 23.12 * (127.0 / 64.0); /* 0x3F,0x7F */

} // namespace

CamHelperSc910gs::CamHelperSc910gs()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperSc910gs::gainCode(double gain) const
{
	/* Robust handling of NaN/inf/<=0 */
	if (!std::isfinite(gain) || !(gain > 0.0))
		gain = kMinGain;

	gain = std::clamp(gain, kMinGain, kMaxGain);

	return best_code_for_gain(gain);
}

double CamHelperSc910gs::gain(uint32_t gainCode) const
{
	double coarse = kCoarse[0];
	uint8_t fine = kFineBase;

	code_to_coarse_fine(gainCode, &coarse, &fine);
	return gain_from_coarse_fine(coarse, fine);
}

unsigned int CamHelperSc910gs::hideFramesStartup() const
{
	return 1;
}

unsigned int CamHelperSc910gs::hideFramesModeSwitch() const
{
	return 1;
}

static CamHelper *create()
{
	return new CamHelperSc910gs();
}

static RegisterCamHelper reg("sc910gs", &create);

} // namespace RPiController
