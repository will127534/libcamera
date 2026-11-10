/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * PDAF Metadata
 */
#pragma once

#include <stdint.h>
#include <vector>

#include <libcamera/geometry.h>

#include "region_stats.h"

namespace RPiController {

struct PdafData {
	/* Confidence, in arbitrary units */
	uint16_t conf;
	/* Phase error, in s16 Q4 format (S.11.4) */
	int16_t phase;
};

using PdafRegions = RegionStats<PdafData>;

/*
 * Raw left/right phase-detect images for display (rpi::PdafImage), produced by
 * camera helpers that compute the phase themselves.
 */
struct PdafImage {
	unsigned int width;
	unsigned int height;
	/* The rpi::PdafImageWindow region shown; empty for the whole field. */
	libcamera::Rectangle window;
	/* Left plane then right plane, 8 bits per sample. */
	std::vector<uint8_t> data;
};

} /* namespace RPiController */
