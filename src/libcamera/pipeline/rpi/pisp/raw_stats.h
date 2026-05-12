/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * SW stats producer for the PiSP pipeline handler — sized for non-QBC raw
 * paths where the CFE hardware-stats path is unreliable (ClearHDR / 14-bit
 * / 16-bit unpacked). The data is already standard 2×2 RGGB Bayer; no
 * remosaic is needed. Reuses the same statistics layout the IPA expects
 * (pisp_statistics: AWB zones, AGC histogram, AGC Y_sum/counted), gated on
 * a per-sensor V4L2 control (e.g. V4L2_CID_USER_BASE + 0x10b9 on IMX585).
 *
 * Symmetric with QbcRemosaic — same control/AWB/metering interface, same
 * output stats layout — but without the QBC permutation step.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include <linux/v4l2-controls.h>

#include <libpisp/frontend/pisp_statistics.h>

#include <libcamera/controls.h>

namespace libcamera {

class RawStatsProducer
{
public:
	/* ClearHDR / "HW stats unreliable" flag — currently published by the
	 * IMX585 driver when V4L2_CID_WIDE_DYNAMIC_RANGE is on. */
	static constexpr uint32_t kClearHdrCid = V4L2_CID_USER_BASE + 0x10b9;

	RawStatsProducer();

	void loadMeteringWeights(const std::string &sensorModel);

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

	std::vector<uint16_t> pairCellX_;          /* RGGB-cell-pair → AWB-cell-X */
	std::vector<uint16_t> meteringWeights_;    /* NxN row-major */
	unsigned int meteringGridW_ = 0;
	unsigned int meteringGridH_ = 0;
	std::vector<uint8_t> cellWeight_;          /* per-RGGB-cell weight */
};

} /* namespace libcamera */
