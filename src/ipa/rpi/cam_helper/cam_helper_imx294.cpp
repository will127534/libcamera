/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx294.cpp - camera information for Imx294 sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperImx294 : public CamHelper
{
public:
	CamHelperImx294();
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
 * Imx294 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx294::CamHelperImx294()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx294::gainCode(double gain) const
{
	return static_cast<uint32_t>(2048.0 - 2048.0/gain);
}

double CamHelperImx294::gain(uint32_t gainCode) const
{
	return static_cast<double>(2048.0/(2048 - gainCode));
}

/*
 * IMX294 NQ (Normal Quad-Bayer, 2×2 binned) and QBC (Quad-Bayer Coding,
 * full per-sub-pixel readout) have very different per-output-pixel
 * sensitivities — empirically NQ post-BLC ≈ 8 × QBC post-BLC for the
 * same scene (see project memory imx294_nq_qbc_8x for the measurement
 * setup). The IPA's Lux/AGC calibration is referenced against the NQ
 * stats; tell the controller that QBC is 1/8 as sensitive so:
 *   - Lux reports the same value in both modes for the same scene
 *     (was QBC ~1/8 of NQ Lux before this fix).
 *   - AGC, on mode switch from NQ → QBC, multiplies ExposureTime by
 *     ~8 (lastSensitivity / currentSensitivity) to keep the rendered
 *     image at the same brightness — matching the physical
 *     compensation the user would otherwise have to do manually.
 */
double CamHelperImx294::getModeSensitivity(const CameraMode &mode) const
{
	/*
	 * The IMX294 driver reports analogCrop matching the output size
	 * rather than the pre-binning sensor extent, so mode.scaleX/binX
	 * always report 1.0/1 and cannot distinguish NQ from QBC. Compare
	 * the output width against the full sensor width directly:
	 *   NQ binned (4144×2176 etc.) : sensorWidth/width ≈ 2
	 *   QBC full (8432×5648 etc.)  : sensorWidth/width ≈ 1
	 */
	if (mode.width > 0 && mode.sensorWidth >= mode.width * 3 / 2)
		return 1.0;          /* NQ binned — reference sensitivity */
	return 1.0 / 8.0;            /* QBC full readout — 8× less sensitive */
}

unsigned int CamHelperImx294::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperImx294::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}


static CamHelper *create()
{
	return new CamHelperImx294();
}

static RegisterCamHelper reg("imx294", &create);
