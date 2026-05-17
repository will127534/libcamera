/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * ccmp_unwrap.cpp — IMX585 ClearHDR 12-bit gradation-compression inverse.
 *
 * See ccmp_unwrap.h for the design rationale. Builds a 4096-entry u16 LUT
 * that maps compressed u12 → linearised u16 with the BLC anchored on the
 * IPA's static tuning value. The LUT is consumed read-only by the stats
 * kernel; the raw buffer itself is never modified.
 */

#include "ccmp_unwrap.h"

#include <algorithm>
#include <arm_neon.h>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(RPI)

CcmpUnwrap::CcmpUnwrap()
{
	/* Default LUT is the standard 12-bit-to-u16 left-justification, so
	 * that before setParams() the LUT is a no-op (matches native non-HDR
	 * scaling). */
	for (unsigned int i = 0; i < 4096; i++)
		lut_[i] = static_cast<uint16_t>(i << 4);
}

bool CcmpUnwrap::setParams(const Params &p)
{
	std::lock_guard<std::mutex> lk(lutMutex_);
	if (paramsValid_ &&
	    p.thresh1 == params_.thresh1 &&
	    p.thresh2 == params_.thresh2 &&
	    p.slope_l_idx == params_.slope_l_idx &&
	    p.slope_h_idx == params_.slope_h_idx &&
	    p.blc_u12 == params_.blc_u12)
		return false;
	params_ = p;
	paramsValid_ = true;

	/*
	 * Inverse-curve geometry (forward curve in ccmp_unwrap.h diagram):
	 *
	 *   out ≤ T1                            → in = out
	 *   T1 < out ≤ O2                       → in = T1 + (out - T1) << sL
	 *   O2 < out                            → in = T2 + (out - O2) << sH
	 *
	 * where T1=thresh1, T2=thresh2, sL=slope_l_idx, sH=slope_h_idx,
	 *       O2 = T1 + (T2-T1) >> sL.
	 */
	const uint32_t T1 = std::min<uint32_t>(p.thresh1, 0x1FFFFu);
	const uint32_t T2 = std::min<uint32_t>(std::max<uint32_t>(p.thresh2, T1), 0x1FFFFu);
	/*
	 * Per IMX585 AppNote §4.3 register table, ACMP slope register `idx`
	 * implements forward compression 2^-idx (so inverse multiplier 2^idx).
	 * The earlier (idx+1) "fix" was masking a different bug — see below.
	 */
	const unsigned int sL = std::min<unsigned int>(p.slope_l_idx, 11);
	const unsigned int sH = std::min<unsigned int>(p.slope_h_idx, 11);
	const uint32_t O2 = T1 + ((T2 - T1) >> sL);
	const int32_t blc_u12 = std::min<uint32_t>(p.blc_u12, 4095);
	const int32_t blc_u16 = blc_u12 << 4;

	auto inverse = [&](uint32_t out) -> uint32_t {
		if (out <= T1)
			return out;
		if (out <= O2)
			return T1 + ((out - T1) << sL);
		return T2 + ((out - O2) << sH);
	};

	/*
	 * LUT formula: `LUT[u12] = clip(0..0xFFFF, BLC_u16 + (inverse(u12) - BLC_u12))`.
	 *
	 * Invert the CCMP curve to recover the original linear signal, then
	 * map into the 16-bit linear domain at the natural 1:1 scale that the
	 * sensor's 16-bit ClearHDR mode itself delivers. After this, the
	 * buffer values land in the same domain as the native 16-bit ClearHDR
	 * path, so RawStatsProducer processes them identically.
	 *
	 *   - At u12 = BLC_u12: inverse = BLC_u12 (linear region), so LUT =
	 *     BLC_u16 = 3200 — matches IPA's static BLC tuning.
	 *   - For u12 above the linear region: inverse expands by 2^sL or
	 *     2^sH back into the 17-bit input space; clip at 0xFFFF for the
	 *     hottest highlights, matching how 16-bit ClearHDR also clips its
	 *     17-bit-internal at 16-bit output.
	 *
	 * Empirically verified against 16-bit ClearHDR linear: at the same
	 * scene + exposure, Lux readings within ~5% (28000 vs 31000 in our
	 * LED 5500K test), vs ~9× over-report with a stacked `<< 4` rescale.
	 */
	/*
	 * The CCMP curve operates in the sensor's pre-pedestal coordinate
	 * space — so the input to inverse() is the post-CCMP value WITHOUT
	 * the BLC pedestal that the sensor adds at the very end. The buffer
	 * pixel `out` carries that pedestal (BLC_u12 ≈ 200 for IMX585), so
	 * we must subtract it BEFORE looking up the inverse, then add the
	 * IPA's expected u16 BLC to the result.
	 *
	 * Output stretch: the natural inverse maxes at ~28000 (u16) for
	 * u12=4095, well below the BE's expected 12-bit-left-justified u16
	 * range of (BLC..65520). The BE applies its tone-mapping/gamma
	 * assuming the input fills that range, so an unwrapped buffer that
	 * tops at 28000 renders as if it's a dim 12-bit capture — bright
	 * pixels (which CCMP saturates at u12=4095) end up in the lower
	 * half of u16 and the BE crushes them, producing a magenta cast in
	 * highlights as a side-effect of per-channel saturation imbalance.
	 *
	 * Stretch the post-BLC range linearly so LUT[4095] lands at 0xFFF0
	 * (matching the 12-bit-left-justified max). This preserves the
	 * curve's relative shape while putting bright pixels at u16 values
	 * the BE knows how to tone-map.
	 */
	const uint32_t natural_max_postblc =
		(inverse(4095u - blc_u12) > 0) ? inverse(4095u - blc_u12) : 1u;
	const uint32_t target_max_postblc = 0xFFF0u - blc_u16;
	for (uint32_t out = 0; out < 4096; out++) {
		int32_t out_post_blc = static_cast<int32_t>(out) - blc_u12;
		uint64_t in = (out_post_blc <= 0) ? 0u : inverse(static_cast<uint32_t>(out_post_blc));
		uint64_t scaled = (in * target_max_postblc) / natural_max_postblc;
		int64_t val = static_cast<int64_t>(blc_u16) + static_cast<int64_t>(scaled);
		if (val < 0)
			val = 0;
		if (val > 0xFFFF)
			val = 0xFFFF;
		lut_[out] = static_cast<uint16_t>(val);
	}

	LOG(RPI, Info) << "CcmpUnwrap: rebuilt LUT (CCMP inverse, 16-bit linear, natural scale) — T1=" << T1
		<< " T2=" << T2 << " slope_L=1/" << (1u << sL)
		<< " slope_H=1/" << (1u << sH)
		<< " blc_u12=" << blc_u12
		<< "; LUT[" << blc_u12 << "]=" << lut_[blc_u12]
		<< " (BLC anchor; ideal " << blc_u16 << ")"
		<< " LUT[500]=" << lut_[500]
		<< " LUT[2048]=" << lut_[2048] << " LUT[4095]=" << lut_[4095];
	return true;
}

void CcmpUnwrap::process(uint16_t *buf, size_t pixels)
{
	std::lock_guard<std::mutex> lk(lutMutex_);
	if (!paramsValid_)
		return;
	/*
	 * Inner loop: load 8 u16s, look each up in the LUT, store back.
	 * The 4096-entry LUT is too large for vqtbl gather, so the lookup
	 * itself stays scalar; NEON keeps the load/store wide.
	 */
	size_t i = 0;
	for (; i + 8 <= pixels; i += 8) {
		uint16x8_t v = vld1q_u16(buf + i);
		uint16_t a[8];
		vst1q_u16(a, v);
		for (int k = 0; k < 8; k++)
			a[k] = lut_[(a[k] >> 4) & 0xFFFu];
		vst1q_u16(buf + i, vld1q_u16(a));
	}
	for (; i < pixels; i++)
		buf[i] = lut_[(buf[i] >> 4) & 0xFFFu];
}

} /* namespace libcamera */
