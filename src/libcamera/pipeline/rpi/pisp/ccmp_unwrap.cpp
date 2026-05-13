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
	 * LUT formula: `LUT[u12] = clip(0..0xFFFF, BLC_u16 + (inverse(u12) - BLC_u12) << 4)`.
	 *
	 *   - At u12 = BLC_u12: inverse = BLC_u12 (linear region), so LUT =
	 *     BLC_u16 exactly. The IPA's static BLC subtraction lands on a
	 *     0-floor signal here.
	 *   - For u12 ≤ T1 (the curve's linear region): inverse = u12, so the
	 *     LUT collapses to BLC_u16 + (u12 - BLC_u12) << 4  =  u12 << 4.
	 *     Bit-exact with native 12-bit non-HDR scaling.
	 *   - For u12 > T1 (compressed region): inverse expands by 2^sL or
	 *     2^sH, and the << 4 scales the linearised value into the IPA's
	 *     16-bit-shifted reference frame. The clip at 0xFFFF saturates the
	 *     hottest highlights — those pixels read as "full bright" in the
	 *     histogram, which is the correct AGC signal (don't expose more).
	 *
	 * Below-BLC u12 values produce a sub-BLC u16 (or zero after the
	 * clip), which the BE's static BLC subtraction maps to zero.
	 */
	for (uint32_t out = 0; out < 4096; out++) {
		int32_t in = static_cast<int32_t>(inverse(out));
		int64_t val = static_cast<int64_t>(blc_u16) +
			      (static_cast<int64_t>(in - blc_u12) << 4);
		if (val < 0)
			val = 0;
		if (val > 0xFFFF)
			val = 0xFFFF;
		lut_[out] = static_cast<uint16_t>(val);
	}

	LOG(RPI, Info) << "CcmpUnwrap: rebuilt LUT — T1=" << T1
		<< " T2=" << T2 << " slope_L=1/" << (1u << sL)
		<< " slope_H=1/" << (1u << sH)
		<< " blc_u12=" << blc_u12
		<< "; LUT[" << blc_u12 << "]=" << lut_[blc_u12]
		<< " (BLC anchor; ideal " << blc_u16 << ")"
		<< " LUT[500]=" << lut_[500]
		<< " (linear region check; ideal " << (500 << 4) << ")"
		<< " LUT[2048]=" << lut_[2048]
		<< " LUT[4095]=" << lut_[4095];
	return true;
}

} /* namespace libcamera */
