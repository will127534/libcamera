/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx662.cpp - camera information for Imx662 sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperImx662 : public CamHelper
{
public:
	CamHelperImx662();
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
 * Imx662 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx662::CamHelperImx662()
	: CamHelper({}, frameIntegrationDiff)
{
}


uint32_t CamHelperImx662::gainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx662::gain(uint32_t gainCode) const
{
	return pow(10, 0.015 * gainCode);
}

unsigned int CamHelperImx662::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperImx662::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}


static CamHelper *create()
{
	return new CamHelperImx662();
}

static RegisterCamHelper reg("imx662", &create);
