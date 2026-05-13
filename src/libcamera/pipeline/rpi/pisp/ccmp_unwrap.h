/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * IMX585 ClearHDR 12-bit gradation-compression (CCMP) inverse.
 *
 * In 12-bit ClearHDR mode, the IMX585 packs a ~17-bit-range linear signal
 * into 12 bits via a two-knee piecewise-linear gradation curve (see the
 * IMX585 AppNote §4.3):
 *
 *     output 12-bit                                         high-slope
 *           ▲                                          (ACMP2_EXP, e.g. 1/64)
 *      4095 ┤                                       ╱──────────
 *           │                                  ╱────
 *           │                              ╱──     mid-slope (ACMP1_EXP)
 *           │                          ╱──         e.g. 1/4
 *           │                      ╱──
 *           │                  ╱──
 *           │             ╱────                     low slope (= 1, "as is")
 *           │      ╱──────
 *         0 └──────────────────────────────────────────────────►  input 17-bit
 *           0   CCMP1_EXP            CCMP2_EXP                 0x1FFFF
 *
 * To get usable linear stats / colour out of a 12-bit ClearHDR frame, the
 * compression must be inverted *before* the BE / SW stats kernels see the
 * data. CcmpUnwrap builds a 4096-entry LUT mapping 12-bit-output to
 * 16-bit-equivalent linear (scaled so the unwrapped 17-bit input range
 * 0..0x1FFFF maps onto the u16 range 0..0xFFFF — losing one LSB of
 * precision but keeping the buffer u16-shaped for the rest of the pipeline)
 * and applies it in-place to the raw buffer.
 *
 * Engaged from the pisp pipeline handler when wide_dynamic_range=1 AND
 * the sensor format is 12-bit. RawStatsProducer then operates on the
 * post-unwrap buffer the same way it operates on a native 16-bit
 * ClearHDR buffer.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>

namespace libcamera {

class CcmpUnwrap
{
public:
	struct Params {
		uint32_t thresh1;   /* CCMP1_EXP in 17-bit input units (≤ 0x1FFFF) */
		uint32_t thresh2;   /* CCMP2_EXP in 17-bit input units, ≥ thresh1 */
		uint8_t  slope_l_idx;  /* ACMP1_EXP menu index, 0..11; slope = 2^-idx */
		uint8_t  slope_h_idx;  /* ACMP2_EXP menu index, 0..11 */
		uint16_t blc_u12;   /* Sensor pedestal in 12-bit output units
				     * (e.g. 200 for IMX585 BLKLEVEL=50). Used
				     * to anchor the LUT so post-unwrap BLC in
				     * u16 matches the IPA's static tuning
				     * value (3200 for IMX585), i.e. the BE's
				     * static BLC subtraction stays valid. */
	};

	CcmpUnwrap();

	/*
	 * Rebuild the inverse LUT from new CCMP params. Cheap (4096 entries,
	 * a handful of muls/adds each). Returns true if params actually
	 * changed.
	 */
	bool setParams(const Params &p);

	/*
	 * Apply the inverse LUT to each u16 pixel of `buf` (which holds the
	 * raw CFE output, 12-bit left-justified into u16). After this call,
	 * the same buffer holds 16-bit-equivalent linear values clipped to
	 * 0..0xFFFF. NEON-vectorised across 16 pixels per inner iteration.
	 */
	void process(uint16_t *buf, size_t pixels);

	const Params &params() const { return params_; }

private:
	std::mutex lutMutex_;
	Params params_{};
	bool paramsValid_ = false;
	uint16_t lut_[4096] = {};  /* output_12 → linearised_u16 */
};

} /* namespace libcamera */
