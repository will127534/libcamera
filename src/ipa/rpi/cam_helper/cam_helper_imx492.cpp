/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx492.cpp - camera information for Imx492 sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperImx492 : public CamHelper
{
public:
	CamHelperImx492();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
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
 * Imx492 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx492::CamHelperImx492()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx492::gainCode(double gain) const
{
	return static_cast<uint32_t>(2048.0 - 2048.0 / gain);
}

double CamHelperImx492::gain(uint32_t gainCode) const
{
	return static_cast<double>(2048.0 / (2048 - gainCode));
}

unsigned int CamHelperImx492::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperImx492::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}

static CamHelper *create()
{
	return new CamHelperImx492();
}

static RegisterCamHelper reg("imx492", &create);
