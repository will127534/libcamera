/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx585.cpp - camera information for Imx585 sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperImx585 : public CamHelper
{
public:
	CamHelperImx585();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	double getModeSensitivity(const CameraMode &mode) const override;
        unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;

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

/*
 * IMX585 sensitivity per mode.
 *
 * Tried sensitivity=4 for bitdepth==16 (Clear HDR linear) on the basis
 * that EXP_GAIN=+12dB ought to make Clear HDR ~4× as sensitive per
 * photon as SDR, but empirically that overcorrects — AGC pumped ET
 * down to ~SDR_ET/16 and the JPEG came out dark. The IPA's existing
 * Clear-HDR-16 stats path must already absorb most of the HG/LG signal
 * boost (either in the SW stats producer or in the tuning JSON's
 * reference_Y/lux). Without disentangling that, leave the per-mode
 * sensitivity at 1.0 for both Clear HDR-16 and the indistinguishable
 * SDR-12/HDR-12-CCMP 4K case. The Lux delta we see (Clear HDR-16 ~1.1×
 * SDR, Clear HDR-12 ~3× SDR) is from the IPA tuning baseline, not a
 * physical sensitivity mismatch we should compensate here.
 */
double CamHelperImx585::getModeSensitivity([[maybe_unused]] const CameraMode &mode) const
{
	return 1.0;
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


static CamHelper *create()
{
	return new CamHelperImx585();
}

static RegisterCamHelper reg("imx585", &create);
