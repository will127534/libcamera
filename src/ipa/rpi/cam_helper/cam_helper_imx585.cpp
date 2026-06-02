/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx585.cpp - camera information for Imx585 sensor
 */

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

class CamHelperImx585 : public CamHelper
{
public:
	CamHelperImx585();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
        unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;
	std::pair<uint32_t, uint32_t>
	getBlanking(Duration &exposure, Duration minFrameDuration,
		    Duration maxFrameDuration) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
};

/*
 * Imx585 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx585::CamHelperImx585()
	: CamHelper({}, frameIntegrationDiff)
{
}


uint32_t CamHelperImx585::gainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx585::gain(uint32_t gainCode) const
{
	return pow(10, 0.015 * gainCode);
}

unsigned int CamHelperImx585::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperImx585::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}


/*
 * Exact-frame-rate blanking.
 *
 * The IMX585 frame period is VMAX * HMAX / INCK, with VMAX (frame length) and
 * HMAX (line length) both integer registers and the driver rounding VMAX even.
 * The default CamHelper::getBlanking() holds HMAX at the mode minimum and only
 * rounds VMAX, so a requested rate quantises to ~one line (~0.01 fps).
 *
 * When a single fixed frame rate is pinned we instead search the integer
 * (VMAX, HMAX) lattice for the pair whose product lands on the requested period
 * (VMAX even, HMAX within a bounded stretch of the mode minimum), then realise
 * that HMAX through HBLANK. This hits standard rates exactly and arbitrary rates
 * to well under a part per million, at the cost of a slightly longer line.
 *
 * One wrinkle: rpicam-apps passes the rate as the FrameDurationLimits control,
 * which is integer microseconds, so "30 fps" arrives as 33333µs (= 30.0007 fps)
 * and the true 30.000 period is already lost. We snap it back: if the requested
 * (truncated) period is exactly what a nominal integer rate rounds to, and that
 * rate has a whole, even tick count (so an exact VMAX*HMAX exists), we retarget
 * the true period. 24/30/60 then come out dead-on instead of tens of ppm high.
 */
std::pair<uint32_t, uint32_t>
CamHelperImx585::getBlanking(Duration &exposure, Duration minFrameDuration,
			     Duration maxFrameDuration) const
{
	/*
	 * Default blanking first: this clamps the exposure to the frame and
	 * handles the variable-rate (AGC free-running) path unchanged.
	 */
	auto [vblank, hblank] = CamHelper::getBlanking(exposure, minFrameDuration,
						       maxFrameDuration);

	/* Only refine a single pinned rate on a mode whose line can be stretched. */
	if (minFrameDuration != maxFrameDuration ||
	    mode_.maxLineLength <= mode_.minLineLength)
		return { vblank, hblank };

	/* IMX585 INCK-derived HMAX/VMAX clock (matches driver IMX585_PIXEL_RATE). */
	static constexpr double kClock = 74250000.0;
	auto toTicks = [](const Duration &d) {
		return d.get<std::nano>() * (kClock * 1e-9);
	};

	const long width = mode_.width;
	const long height = mode_.height;
	const long minHmax = std::lround(toTicks(mode_.minLineLength));
	const long capHmax = std::min<long>(std::lround(toTicks(mode_.maxLineLength)),
					    minHmax + minHmax / 2);
	long long target = std::llround(toTicks(minFrameDuration));
	if (minHmax <= 0 || target <= 0)
		return { vblank, hblank };

	/*
	 * Snap to a nominal integer frame rate. The request reached us as integer
	 * microseconds (rpicam truncates 1e6/fps, so 24 fps arrives as 41666µs, not
	 * 41667), so recover the rate the user almost certainly meant: take the
	 * implied fps, and if the request is within a microsecond of that nominal
	 * rate's exact period, the µs quantisation is the only difference — retarget
	 * the true period. Guard on the period being a whole, even number of ticks
	 * so an exact (even VMAX)*HMAX solution exists; this also naturally excludes
	 * the NTSC fractions (23.976/29.97/59.94), whose period is not an integer
	 * tick count and is tens of µs off an integer rate, so it cannot snap.
	 */
	const double reqUs = minFrameDuration.get<std::micro>();
	const long nominalFps = std::lround(kClock / static_cast<double>(target));
	if (nominalFps > 0 && std::fabs(1.0e6 / nominalFps - reqUs) < 1.0) {
		const double exact = kClock / static_cast<double>(nominalFps);
		const long long exactTicks = std::llround(exact);
		if (exact == std::floor(exact) && (exactTicks & 1LL) == 0)
			target = exactTicks;
	}

	long bestH = minHmax;
	long long bestV = (static_cast<long long>(vblank) + height) & ~1LL;
	long long bestErr = std::llabs(bestV * bestH - target);

	/*
	 * Longer lines mean more rolling-shutter skew, so among acceptable
	 * solutions we want the *smallest* HMAX. The requested period is itself
	 * only known to ±½µs (the FrameDurationLimits control is integer µs), so
	 * any lattice point within that band is as accurate as the request. Track
	 * the smallest such HMAX and prefer it, unless the rate can be hit exactly
	 * (e.g. 25/50 fps) — exact always wins.
	 */
	const long long halfUs = std::llround(kClock / 2.0e6);
	long lowH = -1;
	long long lowV = 0;

	for (long h = minHmax; h <= capHmax && bestErr; ++h) {
		long long vIdeal = (target + h / 2) / h;	/* nearest VMAX */
		long long vEven = vIdeal & ~1LL;
		for (long long v : { vEven, vEven + 2 }) {
			if (v < mode_.minFrameLength || v > mode_.maxFrameLength)
				continue;
			long long err = std::llabs(v * h - target);
			if (err < bestErr) {
				bestErr = err;
				bestV = v;
				bestH = h;
			}
			if (lowH < 0 && err <= halfUs) {	/* smallest in-µs HMAX */
				lowH = h;
				lowV = v;
			}
		}
	}

	/* Non-exact rate: drop to the lowest HMAX that is still within the µs. */
	if (bestErr != 0 && lowH >= 0 && lowH < bestH) {
		bestH = lowH;
		bestV = lowV;
	}

	/*
	 * Realise the integer HMAX through HBLANK: the driver programs
	 * HMAX = floor(minHmax * (width + hblank) / width), so take the smallest
	 * line length (width + hblank) that floors to bestH.
	 */
	long lineLengthPck = (bestH * width + minHmax - 1) / minHmax;	/* ceil */
	uint32_t newHblank = static_cast<uint32_t>(std::max<long>(0, lineLengthPck - width));
	uint32_t newVblank = static_cast<uint32_t>(std::max<long long>(0, bestV - height));

	/* Re-fit the exposure to the chosen frame and line length. */
	Duration lineLength = hblankToLineLength(newHblank);
	uint32_t expLines = std::min<uint32_t>(exposureLines(exposure, lineLength),
					       static_cast<uint32_t>(bestV) - frameIntegrationDiff);
	exposure = CamHelper::exposure(expLines, lineLength);

	return { newVblank, newHblank };
}


static CamHelper *create()
{
	return new CamHelperImx585();
}

static RegisterCamHelper reg("imx585", &create);
