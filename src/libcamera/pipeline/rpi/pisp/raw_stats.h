/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * SW stats producer for the PiSP pipeline handler — sized for non-QBC raw
 * paths where the CFE hardware-stats path is unreliable. The CFE has a known
 * bug for 14-bit and 16-bit unpacked raw (libcamera-rpi already does a SW
 * unpack/endian-swap on the data path; this class does the same for the
 * stats path). Gated on the existing Needs14bitUnpack / Needs16bitEndianSwap
 * stream flags — no sensor-specific control needed.
 *
 * The data is already standard 2×2 RGGB Bayer; no remosaic is needed. Reuses
 * the same statistics layout the IPA expects (pisp_statistics: AWB zones,
 * AGC histogram, AGC Y_sum/counted). Symmetric with QbcRemosaic — same
 * AWB/metering interface, same output stats layout — without the QBC
 * permutation step.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include <libpisp/frontend/pisp_statistics.h>

#include <libcamera/controls.h>

namespace libcamera {

class RawStatsProducer
{
public:
	RawStatsProducer();

	void loadMeteringWeights(const std::string &sensorModel);

	/*
	 * Read the IPA tuning's rpi.black_level.black_level entry (16-bit-shifted
	 * pixel-units) so the SW kernel uses the same pedestal the BE does.
	 * Returns the loaded value, or 0 if the entry is absent / unreadable.
	 * Stored as the initial BLC; setBlackLevel() can override it per-frame.
	 */
	uint16_t loadBlackLevel(const std::string &sensorModel);

	/*
	 * Live-adjustable BLC (16-bit-shifted pixel-units). The pipeline handler
	 * re-reads the sensor's V4L2 BLC control each frame and pushes the
	 * (scaled) value here so changes propagate within ~1 frame.
	 */
	void setBlackLevel(uint16_t blc);

	/* Capture ColourGains from the IPA's metadataReady signal. */
	void updateWbGains(const ControlList &metadata);

	/*
	 * Compute pisp_statistics from a standard RGGB Bayer buffer at any
	 * supported bit-depth (10/12/14/16 unpacked, samples left-justified
	 * in the high bits of u16). Does NOT modify the raw buffer.
	 *   rawPx     : pointer to the raw buffer
	 *   st        : output stats (entire struct is overwritten)
	 *   width/h   : image dimensions in pixels
	 *   strideBytes: bytes per row
	 */
	void process(const uint16_t *rawPx, pisp_statistics *st,
		     unsigned int width, unsigned int height,
		     unsigned int strideBytes);

private:
	std::atomic<float> wbGainR_{ 1.0f };
	std::atomic<float> wbGainG_{ 1.0f };
	std::atomic<float> wbGainB_{ 1.0f };
	std::atomic<uint16_t> blackLevel_{ 3200 }; /* 16-bit-shifted; live-adjustable */

	std::vector<uint16_t> pairCellX_;          /* RGGB-cell-pair → AWB-cell-X */
	std::vector<uint16_t> meteringWeights_;    /* NxN row-major */
	unsigned int meteringGridW_ = 0;
	unsigned int meteringGridH_ = 0;
	std::vector<uint8_t> cellWeight_;          /* per-RGGB-cell weight */
};

} /* namespace libcamera */
