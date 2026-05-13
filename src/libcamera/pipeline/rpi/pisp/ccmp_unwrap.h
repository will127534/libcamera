/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * IMX585 ClearHDR 12-bit gradation-compression (CCMP) inverse — STATS-SIDE.
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
 * To let the IPA's AGC see the *true* scene luminance distribution (not the
 * compressed one), the stats kernel applies the inverse curve at
 * accumulation time. The raw buffer is NOT rewritten — the BE consumes the
 * unmodified CCMP-compressed buffer, which is what already happens when the
 * HW NQ stats path is used today. Only the stats are linearised.
 *
 * Output LUT design — `LUT[u12] = clip(0..0xFFFF, BLC_u16 + (inverse(u12) - BLC_u12) << 4)`:
 *
 *   * Indexed by the 12-bit CCMP-compressed pixel value (pixel_u16 >> 4).
 *   * Anchored on the BLC: at u12 = BLC_u12 (e.g. 200), LUT = BLC_u16 (= 3200),
 *     so the IPA's static-tuning BLC subtraction stays valid.
 *   * For u12 < T1 (the curve's linear region) the LUT collapses to the
 *     standard `u12 << 4` non-HDR scaling — bit-exact match with native
 *     12-bit captures.
 *   * For u12 > T1 (compressed mid + highlights) the LUT expands compressed
 *     values back to their true linear scale, clipping at 0xFFFF when the
 *     unsloped value exceeds u16 range. AGC then sees highlight pixels at
 *     the top of the histogram (correct "scene is bright" signal) rather
 *     than the compressed mid-band (which would tell AGC to keep exposing).
 *
 * Engaged from the pisp pipeline handler when wide_dynamic_range = 1 on an
 * IMX585 12-bit sensor format. The CcmpUnwrap object builds the LUT and
 * the pipeline handler hands its `lut()` pointer to RawStatsProducer's
 * `setLinearizationLut()`. From then on RawStatsProducer applies LUT[u12]
 * per pixel during accumulation.
 */

#pragma once

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
				     * to anchor the LUT so post-linearisation
				     * BLC in u16 matches the IPA's static
				     * tuning value (3200 for IMX585). */
	};

	CcmpUnwrap();

	/*
	 * Rebuild the inverse LUT from new CCMP params. Cheap (4096 entries,
	 * a handful of muls/adds each). Returns true if params actually
	 * changed.
	 */
	bool setParams(const Params &p);

	/*
	 * LUT pointer for the stats kernel. 4096 u16 entries indexed by the
	 * compressed pixel value (pixel_u16 >> 4); each entry is the
	 * linearised, BLC-anchored u16 representation. Hand this directly to
	 * RawStatsProducer::setLinearizationLut().
	 *
	 * The pointer stays valid for the lifetime of the CcmpUnwrap instance,
	 * but the contents may be rebuilt asynchronously when setParams() is
	 * called. Readers (i.e. the stats kernel inner loop) should treat the
	 * entries as immutable for the duration of one stats pass — a
	 * mid-pass rebuild would only cause a transient inconsistency in a
	 * single frame's stats, not a crash.
	 */
	const uint16_t *lut() const { return lut_; }

	const Params &params() const { return params_; }

private:
	std::mutex lutMutex_;
	Params params_{};
	bool paramsValid_ = false;
	uint16_t lut_[4096] = {};  /* output_12 → linearised_u16 */
};

} /* namespace libcamera */
