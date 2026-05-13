/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * ccmp_unwrap.cpp — IMX585 ClearHDR 12-bit gradation-compression inverse.
 *
 * See ccmp_unwrap.h for the design rationale.
 */

#include "ccmp_unwrap.h"

#include <algorithm>
#include <arm_neon.h>
#include <cstring>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(RPI)

CcmpUnwrap::CcmpUnwrap()
{
	/* Default LUT is identity (×1) so that, before setParams() is called,
	 * process() is a no-op rather than a corruption. */
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
	 * Build the inverse curve. The sensor's forward curve maps INPUT
	 * 17-bit space `in` to OUTPUT 12-bit `out` via:
	 *
	 *   in ≤ T1                            → out = in
	 *   T1 < in ≤ T2                       → out = T1 + (in - T1) * s_L
	 *   T2 < in                            → out = O2 + (in - T2) * s_H
	 *
	 * where T1=thresh1, T2=thresh2, s_L=2^-slope_l_idx,
	 *       s_H=2^-slope_h_idx, O2=T1 + (T2-T1)*s_L.
	 *
	 * The inverse maps `out` back to `in` in 17-bit space. We then
	 * re-anchor on the BLC: the IPA's static BLC subtraction in the BE
	 * is calibrated against the 12-bit-non-HDR pedestal value
	 * (BLC_u16 = blc_u12 << 4). Express the unwrapped signal as
	 *
	 *   output_u16 = BLC_u16 + max(0, in(out) - in(blc_u12)) >> 1
	 *
	 * so:
	 *   - At out=blc_u12, output equals BLC_u16 exactly (the BE
	 *     subtracts the static BLC and leaves a clean 0-floor signal).
	 *   - Below-BLC pixels (sensor edge cases) clamp at BLC_u16,
	 *     producing a 0-floor after the BE's static BLC subtraction.
	 *   - Above-BLC signal is expanded by (1 / s_L or s_H) into a
	 *     16-bit-equivalent linear scale, clipped at 0xFFFF for the
	 *     hottest highlights.
	 */
	const uint32_t T1 = std::min<uint32_t>(p.thresh1, 0x1FFFFu);
	const uint32_t T2 = std::min<uint32_t>(std::max<uint32_t>(p.thresh2, T1), 0x1FFFFu);
	const unsigned int sL = std::min<unsigned int>(p.slope_l_idx, 11);
	const unsigned int sH = std::min<unsigned int>(p.slope_h_idx, 11);
	const uint32_t O2 = T1 + ((T2 - T1) >> sL);
	const uint32_t blc_u12 = std::min<uint32_t>(p.blc_u12, 4095);
	const uint32_t blc_u16 = blc_u12 << 4;

	auto inverse = [&](uint32_t out) -> uint32_t {
		if (out <= T1)
			return out;
		if (out <= O2)
			return T1 + ((out - T1) << sL);
		return T2 + ((out - O2) << sH);
	};
	const uint32_t in_blc = inverse(blc_u12);

	/*
	 * Map the inverse-curve output (`in_linear`) to u16 by anchoring two
	 * fixed points:
	 *
	 *   in_linear = in_blc            → u16 = blc_u16   (= 3200)
	 *   in_linear = inverse(4095)     → u16 = u16_top   (= 65520)
	 *
	 * and linearly interpolating between them based on `in_linear`. This
	 * keeps the BE's static BLC subtraction valid AND uses the full u16
	 * range for the sensor's full output range — avoiding the AGC
	 * over-correction we'd hit if the inverse curve's natural span
	 * exceeded u16 (it routinely does — for default ClearHDR-12 the curve
	 * tops out near 17-bit, hugely above 0xFFFF). The trade-off is that
	 * the unwrapped values are compressed back into a 12-bit-equivalent
	 * u16 representation (the BE's max sensible input range) and we lose
	 * some of the captured DR — but that DR was never going to survive
	 * the rest of a 12-bit-tuned BE pipeline anyway. Real wide-DR work
	 * still wants the explicit 16-bit ClearHDR path.
	 *
	 * Sub-BLC clamp prevents below-pedestal values from polluting the
	 * BE's static BLC subtraction.
	 */
	const uint32_t u16_top = 65520;
	const uint32_t in_max = inverse(4095);
	const uint32_t span_in = (in_max > in_blc) ? (in_max - in_blc) : 1;
	const uint32_t span_u16 = u16_top - blc_u16;

	for (uint32_t out = 0; out < 4096; out++) {
		uint32_t in = inverse(out);
		uint32_t u16;
		if (in <= in_blc) {
			u16 = blc_u16;
		} else {
			uint64_t scaled = static_cast<uint64_t>(in - in_blc) * span_u16 / span_in;
			u16 = blc_u16 + static_cast<uint32_t>(scaled);
			if (u16 > 0xFFFFu)
				u16 = 0xFFFFu;
		}
		lut_[out] = static_cast<uint16_t>(u16);
	}

	LOG(RPI, Info) << "CcmpUnwrap: rebuilt LUT — T1=" << T1
		<< " T2=" << T2 << " slope_L=1/" << (1u << sL)
		<< " slope_H=1/" << (1u << sH)
		<< " blc_u12=" << blc_u12
		<< "; LUT[" << blc_u12 << "]=" << lut_[blc_u12]
		<< " (BLC anchor; ideal " << blc_u16 << ")"
		<< " LUT[100]=" << lut_[100]
		<< " LUT[2048]=" << lut_[2048]
		<< " LUT[4095]=" << lut_[4095];
	return true;
}

void CcmpUnwrap::process(uint16_t *buf, size_t pixels)
{
	std::lock_guard<std::mutex> lk(lutMutex_);
	if (!paramsValid_)
		return;

	/*
	 * Inner loop: lookup u16 → (>> 4) → LUT[u12] → u16. Scalar form is
	 * straightforward; vector form below pulls 8 pixels per iteration.
	 * vqtbl4q_u8 is the lookup engine but only addresses 64 entries —
	 * not enough for a 4096-entry LUT — so we keep the lookup scalar
	 * and use NEON only for load/store / scatter. In practice memory
	 * bandwidth dominates, so the scalar inner is competitive.
	 */
	size_t i = 0;
	for (; i + 8 <= pixels; i += 8) {
		uint16x8_t v = vld1q_u16(buf + i);
		uint16_t a[8];
		vst1q_u16(a, v);
		for (int k = 0; k < 8; k++) {
			unsigned int idx = static_cast<unsigned int>(a[k]) >> 4;
			a[k] = lut_[idx & 0xFFFu];
		}
		vst1q_u16(buf + i, vld1q_u16(a));
	}
	for (; i < pixels; i++) {
		unsigned int idx = static_cast<unsigned int>(buf[i]) >> 4;
		buf[i] = lut_[idx & 0xFFFu];
	}
}

} /* namespace libcamera */
