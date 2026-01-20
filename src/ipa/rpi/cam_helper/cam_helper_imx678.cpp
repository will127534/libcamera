/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx678.cpp - camera information for Imx678 sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;

class CamHelperImx678 : public CamHelper
{
public:
	CamHelperImx678();
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
 * Imx678 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx678::CamHelperImx678()
	: CamHelper({}, frameIntegrationDiff)
{
}


uint32_t CamHelperImx678::gainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx678::gain(uint32_t gainCode) const
{
	return pow(10, 0.015 * gainCode);
}

unsigned int CamHelperImx678::hideFramesStartup() const
{
	/* On startup, we seem to get 1 bad frame. */
	return 1;
}

unsigned int CamHelperImx678::hideFramesModeSwitch() const
{
	/* After a mode switch, we seem to get 1 bad frame. */
	return 1;
}


static CamHelper *create()
{
	return new CamHelperImx678();
}

static RegisterCamHelper reg("imx678", &create);
