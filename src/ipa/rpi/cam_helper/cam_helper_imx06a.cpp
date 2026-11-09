/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * camera helper for the Sony IMX06A sensor
 *
 * CCS/SMIA++-style register model:
 *   analogue gain = 16384 / (16384 - ANA_GAIN_GLOBAL)   (code multiple of 4;
 *     0..15360 = 0..24 dB on the full-resolution modes, 0..16128 = 0..36 dB
 *     on the 4x2 binned mode - the driver's control range enforces both)
 *   exposure = COARSE_INTEG_TIME lines, max FRM_LENGTH_LINES - 64 (CIT_MARGIN)
 *
 * Long exposure: the driver accepts VBLANK/EXPOSURE beyond the 16-bit
 * registers and programs them as value >> n with CIT_LSHIFT = n (n <= 9), so
 * beyond 65532 lines exposure is quantised to 2^n lines; getBlanking() mirrors
 * that (same scheme as imx477).
 *
 * HDR (driver "HDR Mode" control, set before the camera is opened): both
 * families are single-frame, sensor-fused outputs. DCG-HDR's analogue gain is
 * exposed by the driver in low-side-gain units, so its sensitivity equals the
 * normal modes. QBC-HDR keeps the long exposure in the bottom quarter of its
 * 12-bit range, so AGC simply exposes ~4x longer; its reported Lux reads 4x low
 * (the mode cannot be told apart from DCG-HDR 4:1 by its CameraMode).
 *
 * Line timing: the driver reports PIXEL_RATE = INCK x 64 with an extensible
 * HBLANK (one step = one INCK cycle), so the generic line-length handling -
 * including the PiSP pixel-rate limit - applies unchanged.
 *
 * No embedded-data parsing yet: the driver exposes only the image pad, so the
 * pipeline uses DelayedControls for exposure/gain metadata.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <libcamera/base/log.h>

#include "cam_helper.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

class CamHelperImx06a : public CamHelper
{
public:
	CamHelperImx06a();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	std::pair<uint32_t, uint32_t> getBlanking(Duration &exposure, Duration minFrameDuration,
						  Duration maxFrameDuration) const override;

private:
	/* CIT_MARGIN (0x68C2), matches IMX06A_EXPOSURE_MARGIN in the driver. */
	static constexpr int frameIntegrationDiff = 64;
	/* Highest analogue gain code the sensor accepts (36 dB). */
	static constexpr uint32_t maxGainCode = 16128;
	/* FRM_LENGTH_LINES register maximum and the largest CIT_LSHIFT. */
	static constexpr uint32_t frameLengthMax = 65532;
	static constexpr unsigned int longExposureShiftMax = 9;
	/* Driver timing base: PIXEL_RATE = INCK x 64, HBLANK step = 1 INCK. */
	static constexpr long pixPerTick = 64;
	/*
	 * VBLANK step: 4 (binned / DCG-HDR), 8 in the 4x4-binned modes. binY is
	 * capped at 2 by the IPA, so look at the real vertical scale.
	 */
	long long vStep() const { return mode_.scaleY > 3.5 ? 8 : 4; }
};

CamHelperImx06a::CamHelperImx06a()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx06a::gainCode(double gain) const
{
	gain = std::max(gain, 1.0);
	uint32_t code = static_cast<uint32_t>(16384.0 - 16384.0 / gain);
	/* The sensor requires a multiple of 4; round down so we never overshoot. */
	code &= ~3u;
	return std::min(code, maxGainCode);
}

double CamHelperImx06a::gain(uint32_t gainCode) const
{
	return 16384.0 / (16384 - std::min(gainCode, maxGainCode));
}

/*
 * Exact pinned frame rates. The driver's line period is LINE_LENGTH_INCK (one
 * HBLANK step = one INCK cycle) and its frame length moves in steps of up to 4
 * lines, so the frame period is (line ticks h) x (lines v) INCK cycles. The
 * default getBlanking() keeps the line at its minimum and only rounds the frame
 * length, so a requested rate lands within ~one line. When a single rate is
 * pinned we search the (v, h) lattice for h * v == the requested period instead
 * (24 MHz / 24, 25, 30, 48, 50, 60, 100, 120 ... are all whole tick counts), and
 * prefer the SHORTEST such line (least rolling-shutter skew). If no exact pair
 * exists (e.g. full resolution at 12 fps: 2,000,000 ticks has no divisor in the
 * legal line range) we keep the pair with the smallest error.
 * Same approach as the imx294/imx492 helpers.
 */
std::pair<uint32_t, uint32_t> CamHelperImx06a::getBlanking(Duration &exposure,
							   Duration minFrameDuration,
							   Duration maxFrameDuration) const
{
	auto [vblank, hblank] = CamHelper::getBlanking(exposure, minFrameDuration,
						       maxFrameDuration);
	uint32_t frameLength = mode_.height + vblank;
	unsigned int shift = 0;

	if (minFrameDuration == maxFrameDuration &&
	    mode_.maxLineLength > mode_.minLineLength && frameLength <= frameLengthMax) {
		/* INCK: the driver reports PIXEL_RATE = INCK x 64. */
		const double clock = mode_.pixelRate / static_cast<double>(pixPerTick);
		auto toTicks = [clock](const Duration &d) {
			return d.get<std::nano>() * clock * 1e-9;
		};
		const long minH = std::lround(std::ceil(toTicks(mode_.minLineLength) - 1e-6));
		const long capH = std::min<long>(std::lround(toTicks(mode_.maxLineLength)),
						 minH + minH / 2);
		long long target = std::llround(toTicks(minFrameDuration));

		/*
		 * The request arrives in integer microseconds (rpicam truncates
		 * 1e6/fps: 24 fps -> 41666 us). If it is within a microsecond of an
		 * integer rate whose period is a whole tick count, aim for that.
		 */
		const long fps = std::lround(1.0e6 / minFrameDuration.get<std::micro>());
		if (fps > 0 && std::fabs(1.0e6 / fps - minFrameDuration.get<std::micro>()) < 1.0 &&
		    std::fmod(clock, static_cast<double>(fps)) == 0.0)
			target = std::llround(clock / fps);

		long exactH = -1, nearH = -1;
		long long exactV = 0, nearV = 0;
		long long nearErr = std::numeric_limits<long long>::max();
		/*
		 * Exact pairs may use a longer line (up to 4x the minimum) when none
		 * exists within capH; the nearest-pair fallback stays within capH.
		 */
		const long exactCapH = std::min<long>(std::lround(toTicks(mode_.maxLineLength)),
						      4 * minH);

		/* h ascends: the first hit is the shortest line of its kind. */
		for (long h = std::max<long>(minH, 1); h <= exactCapH && exactH < 0; h++) {
			long long v0 = (target / h) & ~(vStep() - 1);
			for (long long v : { v0, v0 + vStep() }) {
				if (v < mode_.minFrameLength || v > mode_.maxFrameLength ||
				    v > frameLengthMax)
					continue;
				long long err = std::llabs(v * h - target);
				if (err == 0) {
					exactH = h;
					exactV = v;
					break;
				}
				if (h <= capH && err < nearErr) {
					nearErr = err;
					nearH = h;
					nearV = v;
				}
			}
		}
		long bestH = exactH > 0 ? exactH : nearH;
		long long bestV = exactH > 0 ? exactV : nearV;

		if (bestH > 0) {
			hblank = static_cast<uint32_t>(std::max<long>(0, bestH * pixPerTick -
								     static_cast<long>(mode_.width)));
			vblank = static_cast<uint32_t>(bestV - mode_.height);
			frameLength = static_cast<uint32_t>(bestV);

			Duration lineLength = hblankToLineLength(hblank);
			uint32_t expLines = std::min<uint32_t>(exposureLines(exposure, lineLength),
							       frameLength - frameIntegrationDiff);
			exposure = CamHelper::exposure(expLines, lineLength);
		}
	}

	while ((frameLength >> shift) > frameLengthMax && shift < longExposureShiftMax)
		shift++;

	if (shift) {
		Duration lineLength = hblankToLineLength(hblank);
		uint32_t exposureLines = CamHelper::exposureLines(exposure, lineLength);

		/* The sensor sees both values divided by 2^shift. */
		exposureLines = (exposureLines >> shift) << shift;
		exposureLines = std::min(exposureLines,
					 frameLength - (frameIntegrationDiff << shift));
		exposure = CamHelper::exposure(exposureLines, lineLength);
	}

	return { vblank, hblank };
}

static CamHelper *create()
{
	return new CamHelperImx06a();
}

static RegisterCamHelper reg("imx06a", &create);
