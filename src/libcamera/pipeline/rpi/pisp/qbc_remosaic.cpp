/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * qbc_remosaic.cpp - SW QBC remosaic + stats producer (see qbc_remosaic.h).
 *
 * Algorithm (Hamilton-Adams direction-aware demosaic adapted for QBC):
 *
 * For each 4×4 macro, the QBC layout is
 *     [R R G G]
 *     [R R G G]
 *     [G G B B]
 *     [G G B B]
 * and the output is the standard 2×2 RGGB tile pattern
 *     [R G R G]
 *     [G B G B]
 *     [R G R G]
 *     [G B G B]
 * at the same pixel resolution.
 *
 * 1. Compute sub-block sums (R, G_top, G_bot, B) and pixel averages.
 * 2. Compute "horizontal" and "vertical" gradients on the sub-block averages:
 *      gH = |(R + G_bot) − (G_top + B)|   (vertical edges in scene)
 *      gV = |(R + G_top) − (G_bot + B)|   (horizontal edges in scene)
 * 3. Smooth macros (gH and gV both small) → emit the sub-block-average tile,
 *    eliminating aliasing/false-colour in flat regions.
 * 4. Edge macros → for each output position blend two candidate sources from
 *    a fixed LUT (H_src — closest same-colour pixel in the row, V_src —
 *    closest same-colour pixel in the column) by α = gH/(gH+gV+ε). Vertical
 *    edges (large gH) lean toward V_src so the interpolation stays on one
 *    side of the edge; horizontal edges (large gV) lean toward H_src.
 *
 * Two-thread NEON implementation. Each worker thread runs the full per-row
 * pipeline (LUT-pass NEON + bilinear-row NEON + per-thread stats accumulators)
 * over a disjoint horizontal strip of macro rows, and the per-thread
 * ThreadState (histogram bins, AWB-zone sums, totalY, macroCount) is merged
 * after both threads join. The bilinear R/B post-pass (the dominant cost) is
 * NEON-vectorised using Q15 fixed-point so all arithmetic stays in 16/32-bit
 * lanes. Strip boundaries stash a 1-row copy of each neighbour's edge source
 * row so bilinear at the strip seam is bit-identical to the single-thread
 * version (the bottom-edge / top-edge clamps only kick in at the true frame
 * boundary, not at the strip seam).
 */

#include "qbc_remosaic.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#include <arm_neon.h>

#include <libcamera/base/log.h>

#include <libpisp/frontend/pisp_statistics.h>

#include <libcamera/control_ids.h>

#include "tuning_helpers.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RPI)

QbcRemosaic::QbcRemosaic()
{
}

void QbcRemosaic::loadMeteringWeights(const std::string &sensorModel)
{
	std::lock_guard<std::mutex> lk(meteringMutex_);

	sensorModel_ = sensorModel;
	meteringWeights_.clear();
	macroWeight_.clear();
	meteringGridW_ = meteringGridH_ = 0;

	std::string first;
	(void)loadMeteringWeightsForMode(sensorModel, "", &first);
	if (first.empty()) {
		LOG(RPI, Warning) << "QBC: no metering modes in tuning for "
				  << sensorModel << " — uniform metering";
		return;
	}
	auto weights = loadMeteringWeightsForMode(sensorModel, first);
	if (weights.empty()) {
		LOG(RPI, Warning) << "QBC: empty weights for '" << first
				  << "' in " << sensorModel << " tuning — uniform";
		return;
	}
	unsigned int n = weights.size();
	unsigned int side = static_cast<unsigned int>(std::sqrt(n));
	if (side * side != n) {
		LOG(RPI, Warning) << "QBC: non-square metering grid (" << n
				  << ") — uniform";
		return;
	}
	meteringWeights_ = std::move(weights);
	meteringGridW_ = meteringGridH_ = side;
	meteringModeName_ = first;
	LOG(RPI, Info) << "QBC: loaded " << side << "×" << side
		       << " '" << first << "' metering grid for " << sensorModel;
}

void QbcRemosaic::setMeteringMode(const std::string &modeName)
{
	std::lock_guard<std::mutex> lk(meteringMutex_);
	if (modeName == meteringModeName_ || sensorModel_.empty())
		return;
	auto weights = loadMeteringWeightsForMode(sensorModel_, modeName);
	if (weights.empty()) {
		LOG(RPI, Warning) << "QBC: metering mode '" << modeName
				  << "' not in " << sensorModel_
				  << " tuning — keeping '" << meteringModeName_ << "'";
		return;
	}
	unsigned int n = weights.size();
	unsigned int side = static_cast<unsigned int>(std::sqrt(n));
	if (side * side != n) {
		LOG(RPI, Warning) << "QBC: non-square metering grid for '"
				  << modeName << "' (" << n << ") — keeping current";
		return;
	}
	LOG(RPI, Info) << "QBC: metering mode '" << meteringModeName_
		       << "' → '" << modeName << "'";
	meteringWeights_ = std::move(weights);
	meteringGridW_ = meteringGridH_ = side;
	meteringModeName_ = modeName;
	macroWeight_.clear();  /* invalidate cache; rebuild on next process() */
}

uint16_t QbcRemosaic::loadBlackLevel(const std::string &sensorModel)
{
	uint16_t v = loadBlackLevelFromTuning(sensorModel);
	if (v != 0)
		blackLevel_.store(v, std::memory_order_relaxed);
	return v;
}

void QbcRemosaic::setBlackLevel(uint16_t blc)
{
	blackLevel_.store(blc, std::memory_order_relaxed);
}

void QbcRemosaic::updateWbGains(const ControlList &metadata)
{
	const auto &gains = metadata.get(controls::ColourGains);
	if (!gains)
		return;
	wbGainR_.store((*gains)[0], std::memory_order_relaxed);
	wbGainB_.store((*gains)[1], std::memory_order_relaxed);
	wbGainG_.store(1.0f, std::memory_order_relaxed);
}

namespace {

/*
 * Per-output LUT (16 entries, indexed by macro-relative row*4+col).
 *   h_src: source pixel (row*4+col) supplying the "horizontal" candidate
 *          (closest same-colour pixel in the same row, falling back to the
 *          nearest same-colour pixel when the row has no match).
 *   v_src: same, for the "vertical" candidate.
 * Where h_src == v_src the directional blend collapses to a single value.
 */
struct OutputLUT {
	uint8_t h_src;
	uint8_t v_src;
};

/*
 * Macro indices (row*4 + col, 0..15):
 *     0  1  2  3
 *     4  5  6  7
 *     8  9 10 11
 *    12 13 14 15
 * Source colour:
 *     R  R  G  G
 *     R  R  G  G
 *     G  G  B  B
 *     G  G  B  B
 * Output colour (2×2 RGGB tile):
 *     R  G  R  G
 *     G  B  G  B
 *     R  G  R  G
 *     G  B  G  B
 */
constexpr OutputLUT kRemosaicLUT[16] = {
	/* (0,0) R */ { 0,  0},
	/* (0,1) G */ { 2,  9},  /* H: src(0,2) G_top; V: src(2,1) G_bot */
	/* (0,2) R */ { 1,  5},  /* H: src(0,1) R;     V: src(1,1) R     */
	/* (0,3) G */ { 3,  3},
	/* (1,0) G */ { 6,  8},  /* H: src(1,2) G_top; V: src(2,0) G_bot */
	/* (1,1) B */ {10, 10},
	/* (1,2) G */ { 6,  6},
	/* (1,3) B */ {11, 11},
	/* (2,0) R */ { 4,  4},
	/* (2,1) G */ { 9,  9},
	/* (2,2) R */ { 5,  5},
	/* (2,3) G */ { 9,  7},  /* H: src(2,1) G_bot; V: src(1,3) G_top */
	/* (3,0) G */ {12, 12},
	/* (3,1) B */ {14, 14},
	/* (3,2) G */ {13,  6},  /* H: src(3,1) G_bot; V: src(1,2) G_top */
	/* (3,3) B */ {15, 15},
};

} /* anonymous namespace */

/* ============================================================================
 * NEON LUT pass: process 8 macros per iteration (32 source columns).
 *
 * Per-macro layout: a macro spans 4 source rows × 4 source cols. We load 4
 * NEON registers per row using vld4q_u16 (4-way deinterleave), which packs
 * 8 consecutive macros' 4 columns into 4 uint16x8_t registers.
 *
 * After deinterleave at row r, regs[r].val[c] contains: pixel at column c of
 * macro 0, column c of macro 1, ..., column c of macro 7. So:
 *   s_lane[r*4+c][m] = regs[r].val[c][m]  for macro m ∈ [0, 8).
 *
 * Sub-block sums per macro fit naturally in u16/u32 lanes; gradients and
 * alpha use 32-bit lanes for headroom; the 16 outputs are emitted as 16
 * uint16x8_t registers (one per output index, 8 macros' values across lanes)
 * and stored back with vst4q_u16 in 4 row-strided writes per macro row.
 *
 * AGC histogram + AWB stats are computed in scalar: histogram-update needs a
 * scatter (no NEON instruction), and AWB-zone-update needs gather/scatter on
 * the zone index per macro. The arithmetic to derive each pixel's Y is
 * NEON-vectorised; only the final scatter is scalar.
 * ========================================================================= */
namespace {

/*
 * Vectorised per-pixel Y for one of the 4 colour groups (R, Gt, Gb, B) of one
 * macro-row block (8 macros). Each group has 4 pixel positions in the macro.
 *
 * Inputs:
 *   s[16]                — uint16x8_t lanes, s[i] holds pixel i of macros [0..7]
 *   pos[4]               — pixel indices within the macro for this colour
 *   coef                 — Q10 colour coefficient (gR_q / gG_q / gB_q)
 *   bg                   — Q10 background sum: u32x4 lanes (low half) holding
 *                          bg per macro for macros [0..7] (so we use two halves)
 *
 * For each of the 4 positions and 8 macros = 32 pixels, we compute
 *   y_q = signal(s) * coef + bg
 *   y   = clamp((y_q >> 10), 0, 65535)
 * and *return* a uint16x8_t for each of the 4 positions, ready for the
 * histogram scatter.
 *
 * Returns a struct of 4 uint16x8_t — the caller then scatters each lane into
 * the AGC histogram with the per-macro metering weight.
 */
struct YLanes4 {
	uint16x8_t y[4];
};

[[gnu::always_inline]] static inline YLanes4
computeYGroup(const uint16x8_t s[16],
	      const uint8_t pos[4],
	      uint32_t coef,
	      uint32x4_t bg_lo, uint32x4_t bg_hi,
	      uint16_t BLC,
	      uint32x4_t &ysum_lo, uint32x4_t &ysum_hi)
{
	YLanes4 out;
	uint16x8_t blcV = vdupq_n_u16(BLC);
	/* All coefs fit in u16 (gG_q max ≈ 4810 < 65536). Use vmlal_n_u16
	 * for widening MAC u16 × u16 → u32 in one instruction. */
	uint16_t coef_u16 = static_cast<uint16_t>(coef);

	uint16x8_t coefV = vdupq_n_u16(coef_u16);
	for (int i = 0; i < 4; i++) {
		uint16x8_t pv = s[pos[i]];
		uint16x8_t sig = vqsubq_u16(pv, blcV);   /* saturating sub keeps clamp at 0 */
		/* y_q = bg + sig * coef (Q10). vmlal_high_u16 does the high
		 * half without an explicit vget_high. */
		uint32x4_t y_lo = vmlal_u16(bg_lo, vget_low_u16(sig), vget_low_u16(coefV));
		uint32x4_t y_hi = vmlal_high_u16(bg_hi, sig, coefV);
		/* vqshrn_n_u32(x, 10): shift right 10 and saturating-narrow
		 * to u16. Combines the (>> 10), the clamp-to-65535, and
		 * the u32→u16 narrow into one instruction. */
		uint16x4_t y_lo_u16 = vqshrn_n_u32(y_lo, 10);
		uint16x4_t y_hi_u16 = vqshrn_n_u32(y_hi, 10);
		uint16x8_t y_combined = vcombine_u16(y_lo_u16, y_hi_u16);
		out.y[i] = y_combined;
		ysum_lo = vaddq_u32(ysum_lo, vmovl_u16(y_lo_u16));
		ysum_hi = vaddq_u32(ysum_hi, vmovl_u16(y_hi_u16));
	}
	return out;
}

/*
 * Direction-aware LUT blend for one output index, across 8 macros.
 *   hv   — uint16x8_t of the 8 H-source pixels for those macros
 *   vv   — uint16x8_t of the 8 V-source pixels
 *   alpha — uint32x4_t lo/hi: per-macro α in Q15 [0, 32768]
 * Returns uint16x8_t of the 8 blended outputs.
 *
 * Numerics: blended = hv + ((alpha * (vv - hv)) >> 15), saturated to u16.
 * Match the scalar reference's signed arithmetic on the diff term.
 */
[[gnu::always_inline]] static inline uint16x8_t
blendOne(uint16x8_t hv, uint16x8_t vv, int32x4_t alpha_lo, int32x4_t alpha_hi)
{
	int32x4_t hv_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(hv)));
	int32x4_t hv_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(hv)));
	int32x4_t vv_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(vv)));
	int32x4_t vv_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(vv)));
	int32x4_t diff_lo = vsubq_s32(vv_lo, hv_lo);
	int32x4_t diff_hi = vsubq_s32(vv_hi, hv_hi);
	/* alpha is in [0, 32768] but always fits in 16-bit unsigned; diff in
	 * [-65535, 65535]. Use 64-bit widening multiply for full precision,
	 * then narrow. */
	int64x2_t prod_lo_a = vmull_s32(vget_low_s32(diff_lo),  vget_low_s32(alpha_lo));
	int64x2_t prod_lo_b = vmull_s32(vget_high_s32(diff_lo), vget_high_s32(alpha_lo));
	int64x2_t prod_hi_a = vmull_s32(vget_low_s32(diff_hi),  vget_low_s32(alpha_hi));
	int64x2_t prod_hi_b = vmull_s32(vget_high_s32(diff_hi), vget_high_s32(alpha_hi));
	/* Arithmetic-shift right by 15. */
	prod_lo_a = vshrq_n_s64(prod_lo_a, 15);
	prod_lo_b = vshrq_n_s64(prod_lo_b, 15);
	prod_hi_a = vshrq_n_s64(prod_hi_a, 15);
	prod_hi_b = vshrq_n_s64(prod_hi_b, 15);
	int32x4_t scaled_lo = vcombine_s32(vmovn_s64(prod_lo_a), vmovn_s64(prod_lo_b));
	int32x4_t scaled_hi = vcombine_s32(vmovn_s64(prod_hi_a), vmovn_s64(prod_hi_b));
	int32x4_t blended_lo = vaddq_s32(hv_lo, scaled_lo);
	int32x4_t blended_hi = vaddq_s32(hv_hi, scaled_hi);
	/* Saturate to u16. */
	uint16x4_t b_lo = vqmovun_s32(blended_lo);
	uint16x4_t b_hi = vqmovun_s32(blended_hi);
	return vcombine_u16(b_lo, b_hi);
}

constexpr unsigned int MACROS_PER_BLOCK = 8;

} /* anonymous namespace */

/*
 * Bilinear R/B post-pass.
 *
 * QBC samples R only at (r, c) where r%4 ∈ {0,1} and c%4 ∈ {0,1}, and B only
 * at r%4 ∈ {2,3}, c%4 ∈ {2,3}. The baseline LUT pass writes R outputs at
 * Bayer positions (2i, 2j) by passing through sources at (variable offsets
 * from the cluster), and B outputs at (2i+1, 2j+1) similarly. Because each
 * macro's same-colour sources sit in a single 2×2 quadrant, the resulting
 * R / B Bayer planes alternate "exact source" and "near-source" columns at
 * a 4-pixel period — surviving the BE demosaic as visible macro-block
 * blockiness.
 *
 * Fix: for each R / B output position, find the (up to 4) nearest same-
 * colour source pixels in the un-modified raw and bilinear-interpolate.
 * When the output position coincides with a source position (corner of a
 * cluster) the interp passes through unchanged; otherwise the (2/3, 1/3)
 * weights smoothly bridge same-colour clusters of neighbouring macros.
 *
 * NEON: Q15 fixed-point with weights ∈ {0, 10923, 21845, 32768} (= round(
 * 0, 1/3, 2/3, 1 × 32768)). All arithmetic in u16/u32 lanes. We process 8
 * output columns per inner-loop iteration (16 source columns worth) using
 * vld1q_u16 + vmlal_u16-style fused multiply-accumulate. Outer loop on row
 * branches once per row to pick wr_top and source rows.
 *
 * Edge handling: out-of-bounds source rows/cols are clamped to the nearest
 * valid same-colour source position — adds bias at the frame edge but
 * avoids reading past the buffer. The clamp is performed on the row index
 * in the outer loop, and at the column extremes by reading a scalar-fallback
 * 4-pixel head/tail block.
 */

namespace {

/*
 * NEON Q15 bilinear blend kernel for one row of 8 output positions:
 *   out[i] = round((w_tl * src_tl[i] + w_tr * src_tr[i] +
 *                   w_bl * src_bl[i] + w_br * src_br[i]) / 32768)
 * Inputs are uint16 lanes (8 per register). Outputs are uint16 saturated to
 * the 12-bit-shifted dynamic range (65535).
 *
 * The four weights are passed as Q15 u16 scalars.
 */
[[gnu::always_inline]] static inline uint16x8_t
bilerpQ15_8(uint16x8_t tl, uint16x8_t tr, uint16x8_t bl, uint16x8_t br,
	    uint16_t w_tl_q, uint16_t w_tr_q, uint16_t w_bl_q, uint16_t w_br_q)
{
	/* Accumulate into 32-bit lanes (low + high halves separately). */
	uint32x4_t acc_lo = vmull_n_u16(vget_low_u16(tl), w_tl_q);
	uint32x4_t acc_hi = vmull_n_u16(vget_high_u16(tl), w_tl_q);
	acc_lo = vmlal_n_u16(acc_lo, vget_low_u16(tr), w_tr_q);
	acc_hi = vmlal_n_u16(acc_hi, vget_high_u16(tr), w_tr_q);
	acc_lo = vmlal_n_u16(acc_lo, vget_low_u16(bl), w_bl_q);
	acc_hi = vmlal_n_u16(acc_hi, vget_high_u16(bl), w_bl_q);
	acc_lo = vmlal_n_u16(acc_lo, vget_low_u16(br), w_br_q);
	acc_hi = vmlal_n_u16(acc_hi, vget_high_u16(br), w_br_q);
	/* round-to-nearest, then narrow with saturation to u16. */
	uint16x4_t out_lo = vqrshrn_n_u32(acc_lo, 15);
	uint16x4_t out_hi = vqrshrn_n_u32(acc_hi, 15);
	return vcombine_u16(out_lo, out_hi);
}

} /* anonymous namespace */

/*
 * Per-row bilinear post-pass for one Bayer position group (R or B).
 *
 * For R (pos == 0):
 *   Outputs at ox = 0, 2, 4, ..., width-2.
 *   ox % 4 == 0: cs_l = cs_r = ox, wc_l = 1 (exact phase).
 *   ox % 4 == 2: cs_l = ox-1, cs_r = ox+2, wc_l = 2/3, wc_r = 1/3 (blend).
 *
 * For B (pos == 1):
 *   Outputs at ox = 1, 3, 5, ..., width-1.
 *   ox % 4 == 3: cs_l = cs_r = ox, wc_l = 1 (exact phase).
 *   ox % 4 == 1: cs_l = ox-2, cs_r = ox+1, wc_l = 1/3, wc_r = 2/3 (blend).
 *
 * Inside a row, neighbouring output positions alternate exact / blend
 * (period 4 in ox). For a 16-source-column block starting at c0 (c0 mod 4
 * == 0), R produces 8 outputs and B produces 8 outputs with the column
 * source indices below.
 *
 * R block (8 outputs at ox = c0 .. c0+14 step 2):
 *   cs_l indices (relative to c0): { 0, 1, 4, 5, 8, 9, 12, 13 }
 *   cs_r indices (relative to c0): { 0, 4, 4, 8, 8, 12, 12, 16 }
 *   wc_l per output: { 1, 2/3, 1, 2/3, 1, 2/3, 1, 2/3 }
 *
 * B block (8 outputs at ox = c0+1 .. c0+15 step 2):
 *   cs_l indices (relative to c0): { -1, 3, 3, 7, 7, 11, 11, 15 }
 *   cs_r indices (relative to c0): {  2, 2, 6, 6, 10, 10, 14, 14 }
 *   (note: B's first output, ox=c0+1, has cs_l = c0-1 → out of this block)
 *
 * For the NEON middle of B we shift the block by 2 source cols: load 16
 * sources from c0-1..c0+14 and access cs_l at relative {0, 4, 4, 8, ...},
 * cs_r at {3, 3, 7, 7, ...}. Cleaner: just do B with c0_eff = c0-2 so the
 * relative indices match R-block layout shifted by 2.
 *
 * Implementation strategy: per output row, build a u16x8 cs_l and cs_r
 * vector (8 source values gathered from the row) using the known lane
 * indices, then Q15 multiply-add with vmlal_n_u16 / vqrshrn_n_u32. Phase
 * (exact vs blend) is encoded by per-lane (wc_l, wc_r) — we use TWO Q15
 * weight pairs (one for the exact lanes, one for the blend lanes), or
 * simpler: encode them as u16x8 vectors and use vmlal_u16 (per-lane
 * widening MLA) instead of broadcast-MLA.
 *
 * Row weight (wr_top, wr_bot) is constant across the row; we accumulate
 * with vmlal_n_u16 using wr_top * wc + wr_bot * wc patterns. Since wr × wc
 * is just Q15 × Q15 = Q30, we'd need to shift right 30 — too lossy for
 * 16-bit values. Instead: compute the column-blended values first (Q15
 * weight × u16 source → u32, shift right 15 to u16), THEN row-blend
 * (Q15 weight × u16 → u32, shift right 15 to u16). Two-stage Q15.
 *
 * Numerics: the two-stage Q15 approach introduces ≤ 1 LSB rounding twice
 * (max combined error ~2 LSB), which is within the ≤ 1 LSB target on
 * non-edge pixels for moderate gradients. To make it tighter, we use
 * round-to-nearest narrowing (vrshrn / vqrshrn) so each stage rounds.
 */
namespace {

constexpr uint16_t Q15_ONE        = 32768;
constexpr uint16_t Q15_TWO_THIRDS = 21845;
constexpr uint16_t Q15_ONE_THIRD  = 10923;
constexpr uint16_t Q15_ZERO       = 0;

/* Q15 column-blend of two u16x8 vectors with per-lane weights → u16x8.
 *   out_i = round((wc_l_i * vl_i + wc_r_i * vr_i) / 32768)
 * Uses vmlal_u16 for widening multiply-accumulate. */
[[gnu::always_inline]] static inline uint16x8_t
colBlendQ15(uint16x8_t vl, uint16x8_t vr, uint16x8_t wc_l, uint16x8_t wc_r)
{
	uint32x4_t acc_lo = vmull_u16(vget_low_u16(vl), vget_low_u16(wc_l));
	uint32x4_t acc_hi = vmull_high_u16(vl, wc_l);
	acc_lo = vmlal_u16(acc_lo, vget_low_u16(vr), vget_low_u16(wc_r));
	acc_hi = vmlal_high_u16(acc_hi, vr, wc_r);
	uint16x4_t out_lo = vqrshrn_n_u32(acc_lo, 15);
	uint16x4_t out_hi = vqrshrn_n_u32(acc_hi, 15);
	return vcombine_u16(out_lo, out_hi);
}

/* Q15 row-blend of two u16x8 column-blended values with scalar row weights
 * (rounded narrow). Inputs must already be the column-blended u16 values;
 * weights are Q15 u16 scalars. */
[[gnu::always_inline]] static inline uint16x8_t
rowBlendQ15(uint16x8_t vt, uint16x8_t vb, uint16_t wr_top_q, uint16_t wr_bot_q)
{
	uint16x8_t wtV = vdupq_n_u16(wr_top_q);
	uint16x8_t wbV = vdupq_n_u16(wr_bot_q);
	uint32x4_t acc_lo = vmull_u16(vget_low_u16(vt), vget_low_u16(wtV));
	uint32x4_t acc_hi = vmull_high_u16(vt, wtV);
	acc_lo = vmlal_u16(acc_lo, vget_low_u16(vb), vget_low_u16(wbV));
	acc_hi = vmlal_high_u16(acc_hi, vb, wbV);
	uint16x4_t out_lo = vqrshrn_n_u32(acc_lo, 15);
	uint16x4_t out_hi = vqrshrn_n_u32(acc_hi, 15);
	return vcombine_u16(out_lo, out_hi);
}

} /* anonymous namespace */

/*
 * Build the 8-lane (vl, vr, wc_l, wc_r) tuple for an R block of 8 outputs
 * starting at ox=c0 (c0 % 4 == 0). Source row pointer `row` must support
 * reads at row[c0+16]. Output lane order matches output column order:
 *   lanes 0..7 = outputs ox = c0+0, c0+2, c0+4, ..., c0+14.
 *
 * cs_l rel indices: {0, 1, 4, 5, 8, 9, 12, 13}
 * cs_r rel indices: {0, 4, 4, 8, 8, 12, 12, 16}
 * wc_l: Q15 {ONE, TWO_THIRDS, ONE, TWO_THIRDS, ONE, TWO_THIRDS, ONE, TWO_THIRDS}
 * wc_r: Q15 {ZERO, ONE_THIRD, ZERO, ONE_THIRD, ZERO, ONE_THIRD, ZERO, ONE_THIRD}
 *
 * For B block at output ox = c0+1, c0+3, ..., c0+15 (c0 % 4 == 0):
 *   ox=c0+1: cs_l=c0-1 (out of block; clamp on left-edge handler), cs_r=c0+2
 *   ox=c0+3: cs_l=c0+3, cs_r=c0+3 (exact)
 *   ox=c0+5: cs_l=c0+3, cs_r=c0+6
 *   ox=c0+7: cs_l=c0+7, cs_r=c0+7
 *   ox=c0+9: cs_l=c0+7, cs_r=c0+10
 *   ox=c0+11: cs_l=c0+11, cs_r=c0+11
 *   ox=c0+13: cs_l=c0+11, cs_r=c0+14
 *   ox=c0+15: cs_l=c0+15, cs_r=c0+15
 * The first output's cs_l is at c0-1 which is outside the local block; we
 * handle B specially by stepping the NEON loop with c0 = c0_block + (-2)
 * so that ox=c0_block+1 maps to cs_l = c0_block-1 = (c0_block-2)+1 inside
 * the loop's 16-col span.
 */

namespace {

/* TBL-based gather: load 24 source u16 once into 3 regs, then permute. */
[[gnu::always_inline]] static inline void
loadRowSlice24(const uint16_t *row, int c0, uint16x8_t &t0, uint16x8_t &t1, uint16x8_t &t2)
{
	t0 = vld1q_u16(row + c0);          /* u16 lanes 0..7   (bytes 0..15)  */
	t1 = vld1q_u16(row + c0 +  8);     /* u16 lanes 8..15  (bytes 16..31) */
	t2 = vld1q_u16(row + c0 + 16);     /* u16 lanes 16..23 (bytes 32..47) */
}

/* Reinterpret 3 u16x8 as 3 u8x16 then build a uint8x16x3_t for vqtbl3q_u8. */
[[gnu::always_inline]] static inline uint8x16x3_t
asByteTable3(uint16x8_t t0, uint16x8_t t1, uint16x8_t t2)
{
	uint8x16x3_t tbl;
	tbl.val[0] = vreinterpretq_u8_u16(t0);
	tbl.val[1] = vreinterpretq_u8_u16(t1);
	tbl.val[2] = vreinterpretq_u8_u16(t2);
	return tbl;
}

/* Byte permutation indices for R gather (single-row source slice of 24 u16):
 *   cs_l (indices {0,1,4,5,8,9,12,13}): bytes {0,1, 2,3, 8,9, 10,11, 16,17, 18,19, 24,25, 26,27}
 *   cs_r (indices {0,4,4,8,8,12,12,16}): bytes {0,1, 8,9, 8,9, 16,17, 16,17, 24,25, 24,25, 32,33}
 */
[[gnu::always_inline]] static inline uint16x8_t
gatherCsl_R(const uint16_t *row, int c0)
{
	uint16x8_t t0, t1, t2;
	loadRowSlice24(row, c0, t0, t1, t2);
	static const uint8_t idx_data[16] = { 0,1, 2,3, 8,9, 10,11, 16,17, 18,19, 24,25, 26,27 };
	uint8x16_t idx = vld1q_u8(idx_data);
	uint8x16_t r = vqtbl3q_u8(asByteTable3(t0, t1, t2), idx);
	return vreinterpretq_u16_u8(r);
}

[[gnu::always_inline]] static inline uint16x8_t
gatherCsr_R(const uint16_t *row, int c0)
{
	uint16x8_t t0, t1, t2;
	loadRowSlice24(row, c0, t0, t1, t2);
	static const uint8_t idx_data[16] = { 0,1, 8,9, 8,9, 16,17, 16,17, 24,25, 24,25, 32,33 };
	uint8x16_t idx = vld1q_u8(idx_data);
	uint8x16_t r = vqtbl3q_u8(asByteTable3(t0, t1, t2), idx);
	return vreinterpretq_u16_u8(r);
}

/*
 * For B block: ox = c0+1, c0+3, c0+5, c0+7, c0+9, c0+11, c0+13, c0+15.
 *   ox=c0+1 : cs_l=c0-1, cs_r=c0+2     wc_l=1/3, wc_r=2/3
 *   ox=c0+3 : cs_l=c0+3, cs_r=c0+3     exact
 *   ox=c0+5 : cs_l=c0+3, cs_r=c0+6     wc_l=1/3, wc_r=2/3
 *   ox=c0+7 : cs_l=c0+7, cs_r=c0+7     exact
 *   ox=c0+9 : cs_l=c0+7, cs_r=c0+10    wc_l=1/3, wc_r=2/3
 *   ox=c0+11: cs_l=c0+11,cs_r=c0+11    exact
 *   ox=c0+13: cs_l=c0+11,cs_r=c0+14    wc_l=1/3, wc_r=2/3
 *   ox=c0+15: cs_l=c0+15,cs_r=c0+15    exact
 */
/* B gather: load 17 source u16 spanning row[c0-1..c0+15]. We use a
 * 24-u16 (48-byte) slice starting at c0-1 → load row+c0-1 onward.
 *
 * cs_l_B indices relative to (c0-1): {0, 4, 4, 8, 8, 12, 12, 16}
 *   bytes: {0,1, 8,9, 8,9, 16,17, 16,17, 24,25, 24,25, 32,33}
 * cs_r_B indices relative to (c0-1): {3, 4, 7, 8, 11, 12, 15, 16}
 *   bytes: {6,7, 8,9, 14,15, 16,17, 22,23, 24,25, 30,31, 32,33}
 */
[[gnu::always_inline]] static inline uint16x8_t
gatherCsl_B(const uint16_t *row, int c0)
{
	uint16x8_t t0, t1, t2;
	loadRowSlice24(row, c0 - 1, t0, t1, t2);
	static const uint8_t idx_data[16] = { 0,1, 8,9, 8,9, 16,17, 16,17, 24,25, 24,25, 32,33 };
	uint8x16_t idx = vld1q_u8(idx_data);
	uint8x16_t r = vqtbl3q_u8(asByteTable3(t0, t1, t2), idx);
	return vreinterpretq_u16_u8(r);
}

[[gnu::always_inline]] static inline uint16x8_t
gatherCsr_B(const uint16_t *row, int c0)
{
	uint16x8_t t0, t1, t2;
	loadRowSlice24(row, c0 - 1, t0, t1, t2);
	static const uint8_t idx_data[16] = { 6,7, 8,9, 14,15, 16,17, 22,23, 24,25, 30,31, 32,33 };
	uint8x16_t idx = vld1q_u8(idx_data);
	uint8x16_t r = vqtbl3q_u8(asByteTable3(t0, t1, t2), idx);
	return vreinterpretq_u16_u8(r);
}

/*
 * Per-lane Q15 col weights. For R: {1, 2/3, 1, 2/3, 1, 2/3, 1, 2/3} on cs_l;
 * complementary on cs_r. Compiled to a single immediate vector load. */
[[gnu::always_inline]] static inline uint16x8_t
wcL_R(void)
{
	const uint16_t v[8] = { Q15_ONE, Q15_TWO_THIRDS, Q15_ONE, Q15_TWO_THIRDS,
				Q15_ONE, Q15_TWO_THIRDS, Q15_ONE, Q15_TWO_THIRDS };
	return vld1q_u16(v);
}

[[gnu::always_inline]] static inline uint16x8_t
wcR_R(void)
{
	const uint16_t v[8] = { Q15_ZERO, Q15_ONE_THIRD, Q15_ZERO, Q15_ONE_THIRD,
				Q15_ZERO, Q15_ONE_THIRD, Q15_ZERO, Q15_ONE_THIRD };
	return vld1q_u16(v);
}

/* For B: {1/3, 1, 1/3, 1, 1/3, 1, 1/3, 1} ... but the first lane has
 * blend (ox=c0+1) where cs_l is in the previous block. The second lane
 * (ox=c0+3) is exact. Pattern: {blend, exact, blend, exact, blend, exact,
 * blend, exact} → cs_l weights {1/3, 1, 1/3, 1, 1/3, 1, 1/3, 1}. */
[[gnu::always_inline]] static inline uint16x8_t
wcL_B(void)
{
	const uint16_t v[8] = { Q15_ONE_THIRD, Q15_ONE, Q15_ONE_THIRD, Q15_ONE,
				Q15_ONE_THIRD, Q15_ONE, Q15_ONE_THIRD, Q15_ONE };
	return vld1q_u16(v);
}

[[gnu::always_inline]] static inline uint16x8_t
wcR_B(void)
{
	const uint16_t v[8] = { Q15_TWO_THIRDS, Q15_ZERO, Q15_TWO_THIRDS, Q15_ZERO,
				Q15_TWO_THIRDS, Q15_ZERO, Q15_TWO_THIRDS, Q15_ZERO };
	return vld1q_u16(v);
}

} /* anonymous namespace */

/*
 * Per-row NEON bilinear pass for R or B.
 *
 * `out` is the destination row. We write to out[c0], out[c0+2], ..., for R;
 * to out[c0+1], out[c0+3], ..., for B. Strided u16 writes (every other
 * column); the other columns are owned by the G pass (already correct from
 * the LUT pass).
 *
 * `src_top` and `src_bot` are the source row pointers (already row-clamped).
 * wr_top_q + wr_bot_q must equal Q15_ONE (or wr_bot_q = 0 with src_bot ==
 * src_top for the exact-row case).
 */
static inline void
bilinearRowNEON(uint16_t * __restrict out,
		const uint16_t * __restrict src_top,
		const uint16_t * __restrict src_bot,
		uint16_t wr_top_q, uint16_t wr_bot_q,
		int width, int pos /* 0=R, 1=B */)
{
	/* Scalar helper to bilerp 1 output, with column clamping. */
	auto src_clamp = [&](const uint16_t *r, int c) -> uint16_t {
		if (c < 0) c = 0;
		else if (c >= width) c = width - 1;
		return r[c];
	};
	auto bilerp1 = [&](int cs_l, int cs_r, uint16_t wc_l_q, uint16_t wc_r_q) -> uint16_t {
		uint32_t v_tl = src_clamp(src_top, cs_l);
		uint32_t v_tr = src_clamp(src_top, cs_r);
		uint32_t v_bl = src_clamp(src_bot, cs_l);
		uint32_t v_br = src_clamp(src_bot, cs_r);
		/* Two-stage Q15 to match the NEON path. */
		uint32_t top_q = (uint32_t)wc_l_q * v_tl + (uint32_t)wc_r_q * v_tr;
		uint32_t bot_q = (uint32_t)wc_l_q * v_bl + (uint32_t)wc_r_q * v_br;
		uint32_t top = (top_q + (1u << 14)) >> 15;
		uint32_t bot = (bot_q + (1u << 14)) >> 15;
		if (top > 65535) top = 65535;
		if (bot > 65535) bot = 65535;
		uint64_t row_q = (uint64_t)wr_top_q * top + (uint64_t)wr_bot_q * bot;
		uint64_t res = (row_q + (1u << 14)) >> 15;
		if (res > 65535) res = 65535;
		return (uint16_t)res;
	};

	int neon_c0_lo, neon_c0_hi;
	if (pos == 0) {
		/* R: blocks at c0 = 0, 16, 32, ..., each accesses [c0..c0+16].
		 * Last block may read row[c0+16] — only safe if c0+16 < width. */
		neon_c0_lo = 0;
		neon_c0_hi = ((width - 16) / 16) * 16;
		if (neon_c0_hi < 0) neon_c0_hi = 0;
	} else {
		/* B: blocks at c0 = 16, 32, ..., each accesses [c0-1..c0+15].
		 * Need c0 >= 1 and c0+15 < width. */
		neon_c0_lo = 16;
		neon_c0_hi = ((width / 16) * 16);
		if (neon_c0_hi >= width) neon_c0_hi = (width / 16) * 16;
		if (neon_c0_hi < neon_c0_lo) neon_c0_hi = neon_c0_lo;
	}

	/* ---- Head scalar (covers col 0..neon_c0_lo) ---- */
	if (pos == 0) {
		/* For R, neon_c0_lo == 0, so no head scalar needed unless the
		 * frame is too small for any NEON block. */
		if (neon_c0_hi == 0) {
			for (int ox = 0; ox < width; ox += 2) {
				int cs_l, cs_r;
				uint16_t wc_l_q, wc_r_q;
				if ((ox & 3) == 0) { cs_l = cs_r = ox; wc_l_q = Q15_ONE; wc_r_q = Q15_ZERO; }
				else { cs_l = ox - 1; cs_r = ox + 2; wc_l_q = Q15_TWO_THIRDS; wc_r_q = Q15_ONE_THIRD; }
				out[ox] = bilerp1(cs_l, cs_r, wc_l_q, wc_r_q);
			}
			return;
		}
	} else {
		/* For B: scalar cover ox = 1, 3, 5, ..., neon_c0_lo-1 (= 15). */
		for (int ox = 1; ox < neon_c0_lo + 1; ox += 2) {
			if (ox >= width) break;
			int cs_l, cs_r;
			uint16_t wc_l_q, wc_r_q;
			if ((ox & 3) == 3) { cs_l = cs_r = ox; wc_l_q = Q15_ONE; wc_r_q = Q15_ZERO; }
			else { cs_l = ox - 2; cs_r = ox + 1; wc_l_q = Q15_ONE_THIRD; wc_r_q = Q15_TWO_THIRDS; }
			out[ox] = bilerp1(cs_l, cs_r, wc_l_q, wc_r_q);
		}
	}

	const uint16x8_t wL = (pos == 0) ? wcL_R() : wcL_B();
	const uint16x8_t wR_ = (pos == 0) ? wcR_R() : wcR_B();

	/* Pre-load byte index vectors for vqtbl3q_u8. */
	static const uint8_t idxL_R_data[16] = { 0,1, 2,3, 8,9, 10,11, 16,17, 18,19, 24,25, 26,27 };
	static const uint8_t idxR_R_data[16] = { 0,1, 8,9, 8,9, 16,17, 16,17, 24,25, 24,25, 32,33 };
	static const uint8_t idxL_B_data[16] = { 0,1, 8,9, 8,9, 16,17, 16,17, 24,25, 24,25, 32,33 };
	static const uint8_t idxR_B_data[16] = { 6,7, 8,9, 14,15, 16,17, 22,23, 24,25, 30,31, 32,33 };
	const uint8x16_t idxL = vld1q_u8(pos == 0 ? idxL_R_data : idxL_B_data);
	const uint8x16_t idxR = vld1q_u8(pos == 0 ? idxR_R_data : idxR_B_data);

	/* ---- NEON loop ----
	 * Per-iteration: load 24 source u16 from each of src_top and src_bot
	 * (3 regs each), do TBL gather × 4 (L/R × top/bot), col-blend × 2,
	 * row-blend, store 8 strided u16. The exact-row case (wr_bot_q == 0)
	 * skips the col_bot computation and the row blend. */
	for (int c0 = neon_c0_lo; c0 < neon_c0_hi; c0 += 16) {
		int src_c0 = pos == 0 ? c0 : c0 - 1;
		uint16x8_t t0 = vld1q_u16(src_top + src_c0);
		uint16x8_t t1 = vld1q_u16(src_top + src_c0 + 8);
		uint16x8_t t2 = vld1q_u16(src_top + src_c0 + 16);
		uint8x16x3_t tt = { { vreinterpretq_u8_u16(t0),
				      vreinterpretq_u8_u16(t1),
				      vreinterpretq_u8_u16(t2) } };
		uint16x8_t vL_top = vreinterpretq_u16_u8(vqtbl3q_u8(tt, idxL));
		uint16x8_t vR_top = vreinterpretq_u16_u8(vqtbl3q_u8(tt, idxR));
		uint16x8_t col_top = colBlendQ15(vL_top, vR_top, wL, wR_);

		uint16x8_t result;
		if (wr_bot_q == 0) {
			result = col_top;
		} else {
			uint16x8_t b0 = vld1q_u16(src_bot + src_c0);
			uint16x8_t b1 = vld1q_u16(src_bot + src_c0 + 8);
			uint16x8_t b2 = vld1q_u16(src_bot + src_c0 + 16);
			uint8x16x3_t bb = { { vreinterpretq_u8_u16(b0),
					      vreinterpretq_u8_u16(b1),
					      vreinterpretq_u8_u16(b2) } };
			uint16x8_t vL_bot = vreinterpretq_u16_u8(vqtbl3q_u8(bb, idxL));
			uint16x8_t vR_bot = vreinterpretq_u16_u8(vqtbl3q_u8(bb, idxR));
			uint16x8_t col_bot = colBlendQ15(vL_bot, vR_bot, wL, wR_);
			result = rowBlendQ15(col_top, col_bot, wr_top_q, wr_bot_q);
		}
		/* Strided store: 8 R or B outputs at every other position
		 * within out[c0..c0+15]. Use vld2q_u16 + vst2q_u16 to read
		 * the 8 paired G values (LUT-written), merge in `result`,
		 * and write the 16-u16 block back interleaved.
		 * For R: result goes to val[0] (positions 0, 2, 4, ...).
		 * For B: result goes to val[1] (positions 1, 3, 5, ...). */
		uint16x8x2_t io = vld2q_u16(out + c0);
		if (pos == 0)
			io.val[0] = result;
		else
			io.val[1] = result;
		vst2q_u16(out + c0, io);
	}

	/* ---- Tail scalar ---- */
	if (pos == 0) {
		for (int ox = neon_c0_hi; ox < width; ox += 2) {
			int cs_l, cs_r;
			uint16_t wc_l_q, wc_r_q;
			if ((ox & 3) == 0) { cs_l = cs_r = ox; wc_l_q = Q15_ONE; wc_r_q = Q15_ZERO; }
			else { cs_l = ox - 1; cs_r = ox + 2; wc_l_q = Q15_TWO_THIRDS; wc_r_q = Q15_ONE_THIRD; }
			out[ox] = bilerp1(cs_l, cs_r, wc_l_q, wc_r_q);
		}
	} else {
		int ox_start = neon_c0_hi + 1;
		for (int ox = ox_start; ox < width; ox += 2) {
			int cs_l, cs_r;
			uint16_t wc_l_q, wc_r_q;
			if ((ox & 3) == 3) { cs_l = cs_r = ox; wc_l_q = Q15_ONE; wc_r_q = Q15_ZERO; }
			else { cs_l = ox - 2; cs_r = ox + 1; wc_l_q = Q15_ONE_THIRD; wc_r_q = Q15_TWO_THIRDS; }
			out[ox] = bilerp1(cs_l, cs_r, wc_l_q, wc_r_q);
		}
	}
}

[[maybe_unused]] static void bilinearPostPassRB(uint16_t *out, const uint16_t *src,
			       unsigned int width, unsigned int height,
			       unsigned int strideU16)
{
	auto src_row = [&](int r) -> const uint16_t * {
		if (r < 0) r = 0;
		else if (r >= static_cast<int>(height)) r = height - 1;
		return src + static_cast<unsigned>(r) * strideU16;
	};

	/* R rows. */
	for (unsigned int oy = 0; oy < height; oy += 2) {
		const uint16_t *st, *sb;
		uint16_t wr_top_q, wr_bot_q;
		if ((oy & 3) == 0) {
			st = src_row(oy);
			sb = st;
			wr_top_q = Q15_ONE;
			wr_bot_q = Q15_ZERO;
		} else {
			st = src_row(static_cast<int>(oy) - 1);
			sb = src_row(static_cast<int>(oy) + 2);
			wr_top_q = Q15_TWO_THIRDS;
			wr_bot_q = Q15_ONE_THIRD;
		}
		bilinearRowNEON(out + oy * strideU16, st, sb,
				wr_top_q, wr_bot_q, static_cast<int>(width), 0);
	}

	/* B rows. */
	for (unsigned int oy = 1; oy < height; oy += 2) {
		const uint16_t *st, *sb;
		uint16_t wr_top_q, wr_bot_q;
		if ((oy & 3) == 3) {
			st = src_row(oy);
			sb = st;
			wr_top_q = Q15_ONE;
			wr_bot_q = Q15_ZERO;
		} else {
			st = src_row(static_cast<int>(oy) - 2);
			sb = src_row(static_cast<int>(oy) + 1);
			wr_top_q = Q15_ONE_THIRD;
			wr_bot_q = Q15_TWO_THIRDS;
		}
		bilinearRowNEON(out + oy * strideU16, st, sb,
				wr_top_q, wr_bot_q, static_cast<int>(width), 1);
	}
}

/*
 * Bilinear pass for one macro-row (4 output rows). Takes pointers to the
 * original source rows of THIS macro row, the PREVIOUS macro row, and the
 * NEXT macro row (each pointer is to the first of the 4 rows in that
 * macro row). Outputs are written to `out` (4 rows starting at `out`).
 *
 * For macro row mr, output rows are oy = 4mr, 4mr+1, 4mr+2, 4mr+3.
 *   oy = 4mr     (R, oy%4=0): rs = oy (this macro's row 0)
 *   oy = 4mr+1   (B, oy%4=1): rs_top = 4mr-1 (prev macro's row 3), rs_bot = 4mr+2 (this row 2)
 *   oy = 4mr+2   (R, oy%4=2): rs_top = 4mr+1 (this row 1), rs_bot = 4mr+4 (next macro row 0)
 *   oy = 4mr+3   (B, oy%4=3): rs = oy (this row 3)
 */
static inline void bilinearMacroRow(uint16_t *out_base,
				    const uint16_t *prev_row3,
				    const uint16_t *this_row0,
				    const uint16_t *this_row1,
				    const uint16_t *this_row2,
				    const uint16_t *this_row3,
				    const uint16_t *next_row0,
				    int width,
				    unsigned int strideU16)
{
	/* Output row 0 (R, exact). */
	bilinearRowNEON(out_base + 0 * strideU16,
			this_row0, this_row0,
			Q15_ONE, Q15_ZERO, width, 0);
	/* Output row 1 (B, blend: rs_top=prev_row3, rs_bot=this_row2). */
	bilinearRowNEON(out_base + 1 * strideU16,
			prev_row3, this_row2,
			Q15_ONE_THIRD, Q15_TWO_THIRDS, width, 1);
	/* Output row 2 (R, blend: rs_top=this_row1, rs_bot=next_row0). */
	bilinearRowNEON(out_base + 2 * strideU16,
			this_row1, next_row0,
			Q15_TWO_THIRDS, Q15_ONE_THIRD, width, 0);
	/* Output row 3 (B, exact). */
	bilinearRowNEON(out_base + 3 * strideU16,
			this_row3, this_row3,
			Q15_ONE, Q15_ZERO, width, 1);
}


void QbcRemosaic::process(uint16_t *rawPx, pisp_statistics *st,
			  unsigned int width, unsigned int height,
			  unsigned int strideBytes)
{
	const unsigned int strideU16 = strideBytes / 2;
	const unsigned int macroRows = height / 4;
	const unsigned int macroCols = width / 4;

	auto t_memcpyStart = std::chrono::steady_clock::now();

	/*
	 * Rolling source-row ring buffer for the fused LUT + bilinear pass.
	 * We need source rows of macros mr-2, mr-1, mr available when running
	 * the bilinear pass on output macro row mr-1, so we ring 3 macro-rows
	 * of source data (12 source rows = ~200 KB at full HD QBC) per worker
	 * thread. L2-resident on Cortex-A76. With two worker threads we allocate
	 * 2 × 3 = 6 macro-row slots back-to-back; each thread owns its own
	 * disjoint half of the buffer so the two workers never touch the same
	 * cache lines while reading/writing their rings.
	 *
	 * Ring layout (per thread t ∈ {0, 1}):
	 *   ring[t * 3 + slot][r]   where slot = mrIdx % 3, r ∈ [0, 4)
	 */
	constexpr unsigned int kNumThreads = 2;
	constexpr unsigned int kRingMacroRows = 3;
	const size_t ringElemsPerThread = static_cast<size_t>(strideU16) * 4 * kRingMacroRows;
	const size_t ringElems = ringElemsPerThread * kNumThreads;
	/*
	 * Plus 2 boundary rows (one row each direction at the strip seam) so
	 * thread 0's bilinear post-pass for its last macro row reads the
	 * stashed copy of thread 1's first source row, and vice versa. This
	 * preserves bit-exact output across the seam — the in-frame bilinear
	 * never invokes the top-edge/bottom-edge clamp at the seam, only at
	 * the true frame boundaries.
	 */
	const size_t boundaryElems = static_cast<size_t>(strideU16) * 2;
	const size_t totalElems = ringElems + boundaryElems;
	if (sourceBuffer_.size() != totalElems)
		sourceBuffer_.resize(totalElems);
	uint16_t * const ringBase = sourceBuffer_.data();
	auto ringRow = [&](unsigned int threadIdx, unsigned int mrIdx, int r) -> uint16_t * {
		unsigned int slot = mrIdx % kRingMacroRows;
		return ringBase + (threadIdx * kRingMacroRows + slot) * 4 * strideU16
			+ r * strideU16;
	};
	uint16_t * const boundaryT0NextRow0 = ringBase + ringElems;
	uint16_t * const boundaryT1PrevRow3 = ringBase + ringElems + strideU16;

	/* AWB cell sizing — libpisp finalise_awb() with offset=0. */
	const unsigned int awbCellW = std::max<unsigned int>(
		2u, 2u * ((width + PISP_AWB_STATS_SIZE) / (2u * PISP_AWB_STATS_SIZE)));
	const unsigned int awbCellH = std::max<unsigned int>(
		2u, 2u * ((height + PISP_AWB_STATS_SIZE) / (2u * PISP_AWB_STATS_SIZE)));

	/*
	 * Per-macro metering weight LUT — each 4×4 macro maps to one cell in
	 * the IPA's metering grid (15×15 for pisp). Built once per resolution,
	 * also rebuilt when setMeteringMode() invalidates macroWeight_.
	 */
	{
	std::lock_guard<std::mutex> lk(meteringMutex_);
	if (macroWeight_.size() != macroRows * macroCols &&
	    meteringGridW_ > 0 && meteringGridH_ > 0) {
		macroWeight_.resize(macroRows * macroCols);
		for (unsigned int mr = 0; mr < macroRows; mr++) {
			unsigned int gy = std::min(meteringGridH_ - 1,
						   (mr * meteringGridH_) / macroRows);
			for (unsigned int mc = 0; mc < macroCols; mc++) {
				unsigned int gx = std::min(meteringGridW_ - 1,
							   (mc * meteringGridW_) / macroCols);
				macroWeight_[mr * macroCols + mc] =
					static_cast<uint8_t>(meteringWeights_[gy * meteringGridW_ + gx]);
			}
		}
	}
	} /* meteringMutex_ scope */

	/*
	 * Post-WB BT.601 Y coefficients baked into Q10.
	 */
	const uint32_t gR_q = static_cast<uint32_t>(
		std::clamp(wbGainR_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.299f * 1024.0f);
	const uint32_t gG_q = static_cast<uint32_t>(
		std::clamp(wbGainG_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.587f * 1024.0f);
	const uint32_t gB_q = static_cast<uint32_t>(
		std::clamp(wbGainB_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.114f * 1024.0f);

	auto t0 = std::chrono::steady_clock::now();

	/* Live BLC plumbed in from the V4L2 sensor subdev (pisp.cpp). */
	const uint16_t BLC = static_cast<uint16_t>(blackLevel_.load(std::memory_order_relaxed));
	auto signalBlk = [BLC](uint32_t bsum, unsigned int n) -> uint32_t {
		uint32_t nbl = n * BLC;
		return (bsum > nbl) ? (bsum - nbl) : 0;
	};

	/* Pixel-position groups by colour within the macro. */
	static const uint8_t kRpos[4]  = { 0,  1,  4,  5};
	static const uint8_t kGtpos[4] = { 2,  3,  6,  7};
	static const uint8_t kGbpos[4] = { 8,  9, 12, 13};
	static const uint8_t kBpos[4]  = {10, 11, 14, 15};

	/*
	 * Per-thread accumulator state. Each thread owns one ThreadState and
	 * writes to its own histogram / AWB zones / totalY / macroCount; the
	 * main thread merges them after both workers join.
	 *
	 * The 4 interleaved histograms (hist4) inside each ThreadState break
	 * the load-add-store chain in the inner scatter: each pixel position
	 * 0..15 writes to hist4[idx & 3]. With 4 independent hist arrays the
	 * writes can be issued OoO in parallel.
	 */
	struct ThreadState {
		alignas(64) uint32_t hist4[4][PISP_AGC_STATS_NUM_BINS] = {};
		pisp_awb_statistics_zone zones[PISP_AWB_STATS_NUM_ZONES] = {};
		uint64_t totalY = 0;
		uint64_t macroCount = 0;
	};
	ThreadState states[kNumThreads];

	/* Worker: processes [mrStart, mrEnd) of macro rows. */
	auto processStrip = [&](unsigned int threadIdx,
				unsigned int mrStart, unsigned int mrEnd) {
		auto &state = states[threadIdx];
		auto (&hist4)[4][PISP_AGC_STATS_NUM_BINS] = state.hist4;
		auto (&zones)[PISP_AWB_STATS_NUM_ZONES] = state.zones;
		uint64_t totalY = 0;
		uint64_t macroCount = 0;

	for (unsigned int mr = mrStart; mr < mrEnd; mr++) {
		const unsigned int my = mr * 4;
		const unsigned int cellY = std::min<unsigned int>(
			my / awbCellH, PISP_AWB_STATS_SIZE - 1);
		uint16_t * const rowBase = rawPx + my * strideU16;
		const uint8_t * const wRow = macroWeight_.empty() ? nullptr :
			(macroWeight_.data() + mr * macroCols);

		/* Save the 4 source rows of this macro row into the per-thread
		 * ring buffer BEFORE the LUT pass overwrites them. The
		 * bilinear post-pass for a previous macro row (run later in
		 * this iteration) reads from the ring. */
		for (int r = 0; r < 4; r++)
			std::memcpy(ringRow(threadIdx, mr, r), rowBase + r * strideU16,
				    strideU16 * sizeof(uint16_t));

		unsigned int mc = 0;
		/* NEON block: process MACROS_PER_BLOCK = 8 macros at a time. */
		for (; mc + MACROS_PER_BLOCK <= macroCols; mc += MACROS_PER_BLOCK) {
			const unsigned int mx = mc * 4;
			uint16_t * const blkBase = rowBase + mx;
			/* Prefetch the NEXT block's source rows so they're hot
			 * in L1 by the time we load them. The L2 hardware
			 * prefetcher won't always anticipate strided access. */
			if (mc + 2 * MACROS_PER_BLOCK <= macroCols) {
				uint16_t * const nb = blkBase + 4 * MACROS_PER_BLOCK;
				__builtin_prefetch(nb + 0 * strideU16);
				__builtin_prefetch(nb + 1 * strideU16);
				__builtin_prefetch(nb + 2 * strideU16);
				__builtin_prefetch(nb + 3 * strideU16);
			}

			/*
			 * Load 32 source columns × 4 source rows = 128 u16 into
			 * 16 NEON regs. Use vld4q_u16 to deinterleave each row
			 * into 4 vectors corresponding to the 4 columns of each
			 * macro. After deinterleave at row r:
			 *   col_regs[r].val[c][m] = pixel (r, c) of macro m.
			 */
			uint16x8x4_t row[4];
			row[0] = vld4q_u16(blkBase + 0 * strideU16);
			row[1] = vld4q_u16(blkBase + 1 * strideU16);
			row[2] = vld4q_u16(blkBase + 2 * strideU16);
			row[3] = vld4q_u16(blkBase + 3 * strideU16);

			/* Index into the flat-16-pixel s[] for each block: row*4+col. */
			uint16x8_t s[16];
			s[ 0] = row[0].val[0]; s[ 1] = row[0].val[1]; s[ 2] = row[0].val[2]; s[ 3] = row[0].val[3];
			s[ 4] = row[1].val[0]; s[ 5] = row[1].val[1]; s[ 6] = row[1].val[2]; s[ 7] = row[1].val[3];
			s[ 8] = row[2].val[0]; s[ 9] = row[2].val[1]; s[10] = row[2].val[2]; s[11] = row[2].val[3];
			s[12] = row[3].val[0]; s[13] = row[3].val[1]; s[14] = row[3].val[2]; s[15] = row[3].val[3];

			/* Sub-block sums per macro. Each sum can reach 4 × 65520
			 * = 262080, which overflows u16, so widen to u32 first.
			 * Use vaddl_u16 to do u16 + u16 → u32 in one instruction. */
			uint32x4_t Rsum_lo, Rsum_hi, Gtsum_lo, Gtsum_hi;
			uint32x4_t Gbsum_lo, Gbsum_hi, Bsum_lo, Bsum_hi;
			{
				/* R: s[0] + s[1] + s[4] + s[5]. */
				uint32x4_t r01_lo = vaddl_u16(vget_low_u16(s[0]), vget_low_u16(s[1]));
				uint32x4_t r01_hi = vaddl_u16(vget_high_u16(s[0]), vget_high_u16(s[1]));
				uint32x4_t r45_lo = vaddl_u16(vget_low_u16(s[4]), vget_low_u16(s[5]));
				uint32x4_t r45_hi = vaddl_u16(vget_high_u16(s[4]), vget_high_u16(s[5]));
				Rsum_lo = vaddq_u32(r01_lo, r45_lo);
				Rsum_hi = vaddq_u32(r01_hi, r45_hi);

				uint32x4_t gt23_lo = vaddl_u16(vget_low_u16(s[2]), vget_low_u16(s[3]));
				uint32x4_t gt23_hi = vaddl_u16(vget_high_u16(s[2]), vget_high_u16(s[3]));
				uint32x4_t gt67_lo = vaddl_u16(vget_low_u16(s[6]), vget_low_u16(s[7]));
				uint32x4_t gt67_hi = vaddl_u16(vget_high_u16(s[6]), vget_high_u16(s[7]));
				Gtsum_lo = vaddq_u32(gt23_lo, gt67_lo);
				Gtsum_hi = vaddq_u32(gt23_hi, gt67_hi);

				uint32x4_t gb89_lo  = vaddl_u16(vget_low_u16(s[8]), vget_low_u16(s[9]));
				uint32x4_t gb89_hi  = vaddl_u16(vget_high_u16(s[8]), vget_high_u16(s[9]));
				uint32x4_t gb1213_lo = vaddl_u16(vget_low_u16(s[12]), vget_low_u16(s[13]));
				uint32x4_t gb1213_hi = vaddl_u16(vget_high_u16(s[12]), vget_high_u16(s[13]));
				Gbsum_lo = vaddq_u32(gb89_lo, gb1213_lo);
				Gbsum_hi = vaddq_u32(gb89_hi, gb1213_hi);

				uint32x4_t b1011_lo = vaddl_u16(vget_low_u16(s[10]), vget_low_u16(s[11]));
				uint32x4_t b1011_hi = vaddl_u16(vget_high_u16(s[10]), vget_high_u16(s[11]));
				uint32x4_t b1415_lo = vaddl_u16(vget_low_u16(s[14]), vget_low_u16(s[15]));
				uint32x4_t b1415_hi = vaddl_u16(vget_high_u16(s[14]), vget_high_u16(s[15]));
				Bsum_lo = vaddq_u32(b1011_lo, b1415_lo);
				Bsum_hi = vaddq_u32(b1011_hi, b1415_hi);
			}

			/* Gradients (32-bit per macro). */
			int32x4_t gH_lo = vabdq_s32(
				vreinterpretq_s32_u32(vaddq_u32(Rsum_lo, Gbsum_lo)),
				vreinterpretq_s32_u32(vaddq_u32(Gtsum_lo, Bsum_lo)));
			int32x4_t gH_hi = vabdq_s32(
				vreinterpretq_s32_u32(vaddq_u32(Rsum_hi, Gbsum_hi)),
				vreinterpretq_s32_u32(vaddq_u32(Gtsum_hi, Bsum_hi)));
			int32x4_t gV_lo = vabdq_s32(
				vreinterpretq_s32_u32(vaddq_u32(Rsum_lo, Gtsum_lo)),
				vreinterpretq_s32_u32(vaddq_u32(Gbsum_lo, Bsum_lo)));
			int32x4_t gV_hi = vabdq_s32(
				vreinterpretq_s32_u32(vaddq_u32(Rsum_hi, Gtsum_hi)),
				vreinterpretq_s32_u32(vaddq_u32(Gbsum_hi, Bsum_hi)));

			/* alpha = (gH << 15) / (gH + gV + 1024). Use float
			 * reciprocal-estimate + 1 Newton-Raphson step for ~24-bit
			 * mantissa precision (way more than the 15-bit Q15 we
			 * need), much lower latency than vdivq_f32 on A76. */
			float32x4_t fH_lo = vcvtq_f32_s32(gH_lo);
			float32x4_t fH_hi = vcvtq_f32_s32(gH_hi);
			float32x4_t fV_lo = vcvtq_f32_s32(gV_lo);
			float32x4_t fV_hi = vcvtq_f32_s32(gV_hi);
			float32x4_t eps   = vdupq_n_f32(1024.0f);
			float32x4_t denom_lo = vaddq_f32(vaddq_f32(fH_lo, fV_lo), eps);
			float32x4_t denom_hi = vaddq_f32(vaddq_f32(fH_hi, fV_hi), eps);
			float32x4_t scale = vdupq_n_f32(32768.0f);
			/* recip estimate + 1 NR step: recip *= (2 - recip * d). */
			float32x4_t r_lo = vrecpeq_f32(denom_lo);
			float32x4_t r_hi = vrecpeq_f32(denom_hi);
			r_lo = vmulq_f32(r_lo, vrecpsq_f32(r_lo, denom_lo));
			r_hi = vmulq_f32(r_hi, vrecpsq_f32(r_hi, denom_hi));
			float32x4_t a_lo = vmulq_f32(vmulq_f32(fH_lo, scale), r_lo);
			float32x4_t a_hi = vmulq_f32(vmulq_f32(fH_hi, scale), r_hi);
			int32x4_t alpha_lo = vcvtq_s32_f32(a_lo);
			int32x4_t alpha_hi = vcvtq_s32_f32(a_hi);

			/* LUT blend: only compute the 8 G outputs across 8 macros.
			 * Output positions 0, 2, 5, 7, 8, 10, 13, 15 are R/B
			 * which get overwritten by the bilinear post-pass — skip
			 * them entirely. Positions {1, 3, 4, 6, 9, 11, 12, 14}
			 * are G; we compute and store these.
			 *
			 * The G-output indices and their LUT entries are:
			 *   1  → ( 2, 9)   blend
			 *   3  → ( 3, 3)   pass-through
			 *   4  → ( 6, 8)   blend
			 *   6  → ( 6, 6)   pass-through
			 *   9  → ( 9, 9)   pass-through
			 *   11 → ( 9, 7)   blend
			 *   12 → (12,12)   pass-through
			 *   14 → (13, 6)   blend
			 *
			 * 4 blends + 4 pass-throughs (vs the previous 6 blends +
			 * 10 pass-throughs — though all 16 were being stored).
			 * The previous code did 6 blends total (idx 1, 2, 4, 11,
			 * 14, and... actually looking at LUT: idx 0 = (0,0)
			 * pass; idx 1 = (2,9) blend; idx 2 = (1,5) blend; idx 3 =
			 * (3,3) pass; idx 4 = (6,8) blend; idx 5 = (10,10) pass;
			 * idx 6 = (6,6) pass; idx 7 = (11,11) pass; idx 8 =
			 * (4,4) pass; idx 9 = (9,9) pass; idx 10 = (5,5) pass;
			 * idx 11 = (9,7) blend; idx 12 = (12,12) pass; idx 13 =
			 * (14,14) pass; idx 14 = (13,6) blend; idx 15 = (15,15)
			 * pass.) — so 6 blends, 10 passes. After removing
			 * R/B outputs (positions 0, 2, 5, 7, 8, 10, 13, 15)
			 * we drop blend at idx 2 → 4 blends + 4 pass-throughs.
			 */
			uint16x8_t g1  = blendOne(s[2],  s[9], alpha_lo, alpha_hi);
			uint16x8_t g3  = s[3];
			uint16x8_t g4_ = blendOne(s[6],  s[8], alpha_lo, alpha_hi);
			uint16x8_t g6  = s[6];
			uint16x8_t g9  = s[9];
			uint16x8_t g11 = blendOne(s[9],  s[7], alpha_lo, alpha_hi);
			uint16x8_t g12 = s[12];
			uint16x8_t g14 = blendOne(s[13], s[6], alpha_lo, alpha_hi);

			/* Store using vld2q_u16 + vst2q_u16: read the row of
			 * 16 u16 (R/G interleave), overwrite the G lanes only,
			 * write back. The R/B positions retain whatever was
			 * there (the original source, since LUT no longer writes
			 * over it) — bilinear post-pass will overwrite them.
			 *
			 * Row 0 of the macro block at blkBase: col 0=R, col 1=G,
			 * col 2=R, col 3=G. vld2q_u16(blkBase) gives val[0] =
			 * {col0,col2,col4,col6,...} (R positions of macros 0..7
			 * INTERLEAVED across the 8 lanes). Hmm actually the
			 * 8 macros are SPATIALLY adjacent in cols 0..31, so
			 * vld2q_u16(blkBase) loads 16 u16 = first 16 cols of
			 * the 32-col block, NOT all 8 macros' col 0/1.
			 *
			 * Wait — we want to write 8 G outputs at columns 1, 3,
			 * 5, 7, ..., 15 in the FIRST half of the 32-col block,
			 * and 1, 3, ..., 15 of the SECOND half. So two vld2q +
			 * vst2q per row.
			 *
			 * vld2q_u16(blkBase) deinterleaves: val[0] = cols
			 * {0,2,4,6,8,10,12,14}, val[1] = cols {1,3,5,7,9,11,13,15}.
			 * We want to update val[1] (G columns) but leave val[0]
			 * untouched. val[1] across 8 macros = pixel 1 of macro
			 * 0 (= G), pixel 3 of macro 0 (= G_other side), pixel
			 * 1 of macro 1, pixel 3 of macro 1, ...
			 *
			 * Hmm but my g1, g3, etc. are uint16x8_t with lane 0 =
			 * macro 0's G output, lane 1 = macro 1's G output, etc.
			 * I need lane 0 of the *write* to be macro 0's pixel 1,
			 * lane 1 to be macro 0's pixel 3, lane 2 to be macro 1's
			 * pixel 1, etc.
			 *
			 * This requires interleaving g1 and g3 across macros:
			 *   val[1] = vzip1q_u16(g1, g3) → first half: macros 0..3
			 *   2nd vst2q val[1] = vzip2q_u16(g1, g3) → macros 4..7
			 *
			 * Same for row 1: val[1] = G outputs at idx 5 (B-pos,
			 * not G) and idx 7 (B-pos, not G). Wait — row 1's
			 * positions are idx 4..7: 4=G, 5=B, 6=G, 7=B.
			 *   val[1] in row 1 = cols 1, 3, 5, 7, ... = pixel
			 *   5 of macro 0 (B), pixel 7 of macro 0 (B), pixel
			 *   5 of macro 1, ...
			 * So row 1's val[1] is all B outputs — we skip them.
			 * Row 1's val[0] = cols 0, 2, 4, ... = pixel 4 of macro
			 * 0 (G), pixel 6 of macro 0 (G), pixel 4 of macro 1, ...
			 * So row 1's val[0] is the G outputs (g4_, g6).
			 *
			 * Same for row 2 (mid-low half): pixels 8 (G), 9 (G),
			 * 10 (B), 11 (B). cols 0=G, 1=G, 2=B, 3=B.
			 *   vld2q val[0] = cols {0,2,4,6} = G@8, B@10, G@8(macro1), B@10(macro1), ...
			 *   That mixes G and B in val[0] — not clean.
			 *
			 * Hmm. Let me re-think the layout.
			 *
			 * The 32-col block has cols 0..31 corresponding to:
			 *   macro 0: cols 0..3
			 *   macro 1: cols 4..7
			 *   ...
			 *   macro 7: cols 28..31
			 *
			 * Row 0 of macro 0 has 4 pixels at cols 0,1,2,3 →
			 * idx 0, 1, 2, 3 (R, G, R, G).
			 *
			 * To write only G positions in row 0:
			 *   col 1, 3, 5, 7, 9, 11, ..., 29, 31
			 *
			 * That's NOT a simple deinterleave — it's a strided
			 * write at every other col. So use vld2q_u16 +
			 * vst2q_u16 with stride 2.
			 *
			 * vld2q_u16(blkBase + 16, ...) loads cols 16..31. We
			 * need cols 0..15 first and 16..31 second.
			 */
			{
				/*
				 * We already have `row[r]` from the initial
				 * vld4q_u16 loads (each holding 8 macros' 4 cols
				 * for source row r). The R/B positions of `row[r]`
				 * still hold the *source* values which the bilinear
				 * post-pass will overwrite — write them back
				 * unchanged (no-op for the final output). Only the
				 * G positions need to be replaced with the LUT
				 * blend outputs.
				 *
				 * Row 0: G at col 1 (val[1]) and col 3 (val[3]).
				 * Row 1: G at col 0 (val[0]) and col 2 (val[2]).
				 * Row 2: G at col 1 (val[1]) and col 3 (val[3]).
				 * Row 3: G at col 0 (val[0]) and col 2 (val[2]).
				 *
				 * vst4q_u16 stores 32 u16 interleaved across the 4
				 * vecs. R/B positions keep their original source
				 * values — they get overwritten by bilinear later.
				 */
				/* Row 0: G outputs at cols 1, 3 of each macro */
				uint16x8x2_t row0a = vld2q_u16(blkBase + 0 * strideU16);
				uint16x8x2_t row0b = vld2q_u16(blkBase + 0 * strideU16 + 16);
				row0a.val[1] = vzip1q_u16(g1, g3);
				row0b.val[1] = vzip2q_u16(g1, g3);
				vst2q_u16(blkBase + 0 * strideU16, row0a);
				vst2q_u16(blkBase + 0 * strideU16 + 16, row0b);

				/* Row 1: G outputs at cols 0, 2 of each macro */
				uint16x8x2_t row1a = vld2q_u16(blkBase + 1 * strideU16);
				uint16x8x2_t row1b = vld2q_u16(blkBase + 1 * strideU16 + 16);
				row1a.val[0] = vzip1q_u16(g4_, g6);
				row1b.val[0] = vzip2q_u16(g4_, g6);
				vst2q_u16(blkBase + 1 * strideU16, row1a);
				vst2q_u16(blkBase + 1 * strideU16 + 16, row1b);

				/* Row 2: macro positions 8 (R), 9 (G), 10 (R),
				 * 11 (G). G cols are 1 and 3. */
				uint16x8x2_t row2a = vld2q_u16(blkBase + 2 * strideU16);
				uint16x8x2_t row2b = vld2q_u16(blkBase + 2 * strideU16 + 16);
				row2a.val[1] = vzip1q_u16(g9, g11);
				row2b.val[1] = vzip2q_u16(g9, g11);
				vst2q_u16(blkBase + 2 * strideU16, row2a);
				vst2q_u16(blkBase + 2 * strideU16 + 16, row2b);

				/* Row 3: macro positions 12 (G), 13 (B), 14 (G),
				 * 15 (B). G cols are 0 and 2. */
				uint16x8x2_t row3a = vld2q_u16(blkBase + 3 * strideU16);
				uint16x8x2_t row3b = vld2q_u16(blkBase + 3 * strideU16 + 16);
				row3a.val[0] = vzip1q_u16(g12, g14);
				row3b.val[0] = vzip2q_u16(g12, g14);
				vst2q_u16(blkBase + 3 * strideU16, row3a);
				vst2q_u16(blkBase + 3 * strideU16 + 16, row3b);
			}

			/* --- Stats: AWB-zone sums + AGC histogram --- */
			/* Per-macro post-BLC group sums (NEON). */
			uint32x4_t blc4_lo = vdupq_n_u32(4u * BLC);
			uint32x4_t blc4_hi = blc4_lo;
			uint32x4_t r4_lo = vqsubq_u32(Rsum_lo, blc4_lo);
			uint32x4_t r4_hi = vqsubq_u32(Rsum_hi, blc4_hi);
			uint32x4_t b4_lo = vqsubq_u32(Bsum_lo, blc4_lo);
			uint32x4_t b4_hi = vqsubq_u32(Bsum_hi, blc4_hi);
			uint32x4_t gtsum_blc_lo = vqsubq_u32(Gtsum_lo, blc4_lo);
			uint32x4_t gtsum_blc_hi = vqsubq_u32(Gtsum_hi, blc4_hi);
			uint32x4_t gbsum_blc_lo = vqsubq_u32(Gbsum_lo, blc4_lo);
			uint32x4_t gbsum_blc_hi = vqsubq_u32(Gbsum_hi, blc4_hi);
			/* g4 = signalBlk(Gtsum+Gbsum, 8) >> 1; matches scalar:
			 *   total = Gtsum + Gbsum
			 *   sig8 = total > 8*BLC ? total - 8*BLC : 0
			 *   g4   = sig8 >> 1
			 * Using qsubq on (Gt+Gb) - 8*BLC keeps semantics. */
			uint32x4_t blc8_lo = vshlq_n_u32(blc4_lo, 1);
			uint32x4_t gtgb_lo = vaddq_u32(Gtsum_lo, Gbsum_lo);
			uint32x4_t gtgb_hi = vaddq_u32(Gtsum_hi, Gbsum_hi);
			uint32x4_t g4_lo = vshrq_n_u32(vqsubq_u32(gtgb_lo, blc8_lo), 1);
			uint32x4_t g4_hi = vshrq_n_u32(vqsubq_u32(gtgb_hi, blc8_lo), 1);

			/* AGC histogram bg terms (per-macro averages). */
			uint32x4_t Ravg_lo  = vshrq_n_u32(r4_lo, 2);
			uint32x4_t Ravg_hi  = vshrq_n_u32(r4_hi, 2);
			uint32x4_t GTavg_lo = vshrq_n_u32(gtsum_blc_lo, 2);
			uint32x4_t GTavg_hi = vshrq_n_u32(gtsum_blc_hi, 2);
			uint32x4_t GBavg_lo = vshrq_n_u32(gbsum_blc_lo, 2);
			uint32x4_t GBavg_hi = vshrq_n_u32(gbsum_blc_hi, 2);
			uint32x4_t Bavg_lo  = vshrq_n_u32(b4_lo, 2);
			uint32x4_t Bavg_hi  = vshrq_n_u32(b4_hi, 2);
			uint32x4_t Gavg_lo = vshrq_n_u32(vaddq_u32(GTavg_lo, GBavg_lo), 1);
			uint32x4_t Gavg_hi = vshrq_n_u32(vaddq_u32(GTavg_hi, GBavg_hi), 1);

			/* bg_R, bg_G, bg_B Q10. Each is a u32x4 of 8 macros. */
			uint32x4_t gG_v = vdupq_n_u32(gG_q);
			uint32x4_t gR_v = vdupq_n_u32(gR_q);
			uint32x4_t gB_v = vdupq_n_u32(gB_q);
			uint32x4_t bg_R_lo = vmlaq_u32(vmulq_u32(Gavg_lo, gG_v), Bavg_lo, gB_v);
			uint32x4_t bg_R_hi = vmlaq_u32(vmulq_u32(Gavg_hi, gG_v), Bavg_hi, gB_v);
			uint32x4_t bg_G_lo = vmlaq_u32(vmulq_u32(Ravg_lo, gR_v), Bavg_lo, gB_v);
			uint32x4_t bg_G_hi = vmlaq_u32(vmulq_u32(Ravg_hi, gR_v), Bavg_hi, gB_v);
			uint32x4_t bg_B_lo = vmlaq_u32(vmulq_u32(Ravg_lo, gR_v), Gavg_lo, gG_v);
			uint32x4_t bg_B_hi = vmlaq_u32(vmulq_u32(Ravg_hi, gR_v), Gavg_hi, gG_v);

			/* Per-pixel Y per colour group, then scatter.
			 * ysum_{lo,hi} accumulates Y across pixels & groups within
			 * this 8-macro block; summed into totalY after scatter. */
			uint32x4_t ysum_lo = vdupq_n_u32(0);
			uint32x4_t ysum_hi = vdupq_n_u32(0);
			YLanes4 yR  = computeYGroup(s, kRpos,  gR_q, bg_R_lo, bg_R_hi, BLC, ysum_lo, ysum_hi);
			YLanes4 yGt = computeYGroup(s, kGtpos, gG_q, bg_G_lo, bg_G_hi, BLC, ysum_lo, ysum_hi);
			YLanes4 yGb = computeYGroup(s, kGbpos, gG_q, bg_G_lo, bg_G_hi, BLC, ysum_lo, ysum_hi);
			YLanes4 yB  = computeYGroup(s, kBpos,  gB_q, bg_B_lo, bg_B_hi, BLC, ysum_lo, ysum_hi);

			/* Spill r4/g4/b4 per-macro to scalar arrays for AWB-zone scatter. */
			uint32_t r4_a[8], g4_a[8], b4_a[8];
			vst1q_u32(r4_a,     r4_lo);     vst1q_u32(r4_a + 4, r4_hi);
			vst1q_u32(b4_a,     b4_lo);     vst1q_u32(b4_a + 4, b4_hi);
			vst1q_u32(g4_a,     g4_lo);     vst1q_u32(g4_a + 4, g4_hi);

			/*
			 * Transpose Y from per-pixel-position (16 vectors of 8 macros)
			 * to per-macro (8 vectors of 16 pixels): y_arr[m][i].
			 * Use vst1q_u16 per pixel-position then reshuffle... cheaper
			 * to just SoA store as [pixel][macro] and let the inner loop
			 * walk it. Stride-1 row access wins L1 prefetcher.
			 */
			alignas(16) uint16_t y_arr[16][8];
			vst1q_u16(y_arr[ 0], yR.y[0]);  vst1q_u16(y_arr[ 1], yR.y[1]);
			vst1q_u16(y_arr[ 4], yR.y[2]);  vst1q_u16(y_arr[ 5], yR.y[3]);
			vst1q_u16(y_arr[ 2], yGt.y[0]); vst1q_u16(y_arr[ 3], yGt.y[1]);
			vst1q_u16(y_arr[ 6], yGt.y[2]); vst1q_u16(y_arr[ 7], yGt.y[3]);
			vst1q_u16(y_arr[ 8], yGb.y[0]); vst1q_u16(y_arr[ 9], yGb.y[1]);
			vst1q_u16(y_arr[12], yGb.y[2]); vst1q_u16(y_arr[13], yGb.y[3]);
			vst1q_u16(y_arr[10], yB.y[0]);  vst1q_u16(y_arr[11], yB.y[1]);
			vst1q_u16(y_arr[14], yB.y[2]);  vst1q_u16(y_arr[15], yB.y[3]);

			/* Sum totalY across all 16 lanes × 8 macros. ysum_{lo,hi}
			 * was accumulated inside computeYGroup so we just reduce
			 * the 8 u32 lanes here. */
			{
				uint32x4_t ysum = vaddq_u32(ysum_lo, ysum_hi);
				totalY += vgetq_lane_u32(ysum, 0) + vgetq_lane_u32(ysum, 1) +
					  vgetq_lane_u32(ysum, 2) + vgetq_lane_u32(ysum, 3);
			}

			/* AWB-zone sums. The common case: all 8 macros share the
			 * same cellX. Sum r4/g4/b4 across 8 lanes once, do one
			 * scalar add to z. The rare boundary case (when 8 macros
			 * span two cellX values) falls back to per-macro updates. */
			const unsigned int mxL_first = mx;
			const unsigned int mxL_last  = mx + 7 * 4;
			const unsigned int cellX_first = std::min<unsigned int>(
				mxL_first / awbCellW, PISP_AWB_STATS_SIZE - 1);
			const unsigned int cellX_last = std::min<unsigned int>(
				mxL_last / awbCellW, PISP_AWB_STATS_SIZE - 1);
			const bool sameCell = (cellX_first == cellX_last);
			if (sameCell) {
				uint32x4_t r_sum8 = vaddq_u32(r4_lo, r4_hi);
				uint32x4_t g_sum8 = vaddq_u32(g4_lo, g4_hi);
				uint32x4_t b_sum8 = vaddq_u32(b4_lo, b4_hi);
				uint32_t r_tot = vaddvq_u32(r_sum8);
				uint32_t g_tot = vaddvq_u32(g_sum8);
				uint32_t b_tot = vaddvq_u32(b_sum8);
				pisp_awb_statistics_zone &z =
					zones[cellY * PISP_AWB_STATS_SIZE + cellX_first];
				z.R_sum   += r_tot;
				z.G_sum   += g_tot;
				z.B_sum   += b_tot;
				z.counted += 32;
			}
			for (int m = 0; m < 8; m++) {
				if (!sameCell) {
					unsigned int mxL = mx + m * 4;
					unsigned int cellX = std::min<unsigned int>(
						mxL / awbCellW, PISP_AWB_STATS_SIZE - 1);
					pisp_awb_statistics_zone &z =
						zones[cellY * PISP_AWB_STATS_SIZE + cellX];
					z.R_sum   += r4_a[m];
					z.G_sum   += g4_a[m];
					z.B_sum   += b4_a[m];
					z.counted += 4;
				}

				const unsigned int w = wRow ? wRow[mc + m] : 1u;
				/* 16-pixel histogram scatter into 4 interleaved
				 * hist arrays. */
#define HIST_BUMP(idx, hh) hh[y_arr[idx][m] >> PISP_AGC_HIST_BIN_SHIFT] += w
				HIST_BUMP(0,  hist4[0]); HIST_BUMP(1,  hist4[1]);
				HIST_BUMP(2,  hist4[2]); HIST_BUMP(3,  hist4[3]);
				HIST_BUMP(4,  hist4[0]); HIST_BUMP(5,  hist4[1]);
				HIST_BUMP(6,  hist4[2]); HIST_BUMP(7,  hist4[3]);
				HIST_BUMP(8,  hist4[0]); HIST_BUMP(9,  hist4[1]);
				HIST_BUMP(10, hist4[2]); HIST_BUMP(11, hist4[3]);
				HIST_BUMP(12, hist4[0]); HIST_BUMP(13, hist4[1]);
				HIST_BUMP(14, hist4[2]); HIST_BUMP(15, hist4[3]);
#undef HIST_BUMP
				macroCount += 16;
			}
		}

		/* Tail-scalar for macros not divisible by 8. */
		for (; mc < macroCols; mc++) {
			const unsigned int mx = mc * 4;
			uint16_t * const m = rowBase + mx;

			uint16_t sa[16];
			for (int r = 0; r < 4; r++) {
				const uint16_t *rp = m + r * strideU16;
				sa[r * 4 + 0] = rp[0];
				sa[r * 4 + 1] = rp[1];
				sa[r * 4 + 2] = rp[2];
				sa[r * 4 + 3] = rp[3];
			}
			uint32_t Rsum  = (uint32_t)sa[0]  + sa[1]  + sa[4]  + sa[5];
			uint32_t Gtsum = (uint32_t)sa[2]  + sa[3]  + sa[6]  + sa[7];
			uint32_t Gbsum = (uint32_t)sa[8]  + sa[9]  + sa[12] + sa[13];
			uint32_t Bsum  = (uint32_t)sa[10] + sa[11] + sa[14] + sa[15];
			int32_t gH = std::abs((int32_t)(Rsum + Gbsum) -
					      (int32_t)(Gtsum + Bsum));
			int32_t gV = std::abs((int32_t)(Rsum + Gtsum) -
					      (int32_t)(Gbsum + Bsum));
			uint64_t total = (uint64_t)gH + (uint64_t)gV + 1024;
			uint32_t alpha = (uint32_t)(((uint64_t)gH << 15) / total);
			uint16_t out[16];
			for (int idx = 0; idx < 16; idx++) {
				uint16_t hv = sa[kRemosaicLUT[idx].h_src];
				uint16_t vv = sa[kRemosaicLUT[idx].v_src];
				if (hv == vv) out[idx] = hv;
				else {
					int32_t diff = (int32_t)vv - (int32_t)hv;
					int32_t blended = (int32_t)hv +
						(int32_t)(((int64_t)alpha * diff) >> 15);
					out[idx] = (uint16_t)std::clamp(blended, 0, 65535);
				}
			}
			for (int r = 0; r < 4; r++) {
				uint16_t *rp = m + r * strideU16;
				rp[0] = out[r * 4 + 0];
				rp[1] = out[r * 4 + 1];
				rp[2] = out[r * 4 + 2];
				rp[3] = out[r * 4 + 3];
			}
			uint32_t r4 = signalBlk(Rsum, 4);
			uint32_t g4 = signalBlk(Gtsum + Gbsum, 8) >> 1;
			uint32_t b4 = signalBlk(Bsum, 4);
			const unsigned int cellX = std::min<unsigned int>(
				mx / awbCellW, PISP_AWB_STATS_SIZE - 1);
			pisp_awb_statistics_zone &z =
				zones[cellY * PISP_AWB_STATS_SIZE + cellX];
			z.R_sum   += r4;
			z.G_sum   += g4;
			z.B_sum   += b4;
			z.counted += 4;

			uint32_t Ravg  = r4;
			uint32_t GTavg = signalBlk(Gtsum, 4);
			uint32_t GBavg = signalBlk(Gbsum, 4);
			uint32_t Bavg  = b4;
			Ravg >>= 2; GTavg >>= 2; GBavg >>= 2; Bavg >>= 2;
			uint32_t Gavg  = (GTavg + GBavg) >> 1;
			uint64_t bg_R = (uint64_t)Gavg  * gG_q + (uint64_t)Bavg * gB_q;
			uint64_t bg_G = (uint64_t)Ravg  * gR_q + (uint64_t)Bavg * gB_q;
			uint64_t bg_B = (uint64_t)Ravg  * gR_q + (uint64_t)Gavg * gG_q;
			const unsigned int w = wRow ? wRow[mc] : 1u;
			auto addPx = [&](uint32_t v_signal, uint32_t coef, uint64_t bg, int idx_in_macro) {
				uint64_t y_q = (uint64_t)v_signal * coef + bg;
				uint32_t y = (uint32_t)(y_q >> 10);
				if (y > 65535) y = 65535;
				hist4[idx_in_macro & 3][y >> PISP_AGC_HIST_BIN_SHIFT] += w;
				totalY += y;
			};
			for (int i = 0; i < 4; i++) {
				uint16_t pv = sa[kRpos[i]];
				uint32_t sig = pv > BLC ? pv - BLC : 0;
				addPx(sig, gR_q, bg_R, kRpos[i]);
			}
			for (int i = 0; i < 4; i++) {
				uint16_t pv = sa[kGtpos[i]];
				uint32_t sig = pv > BLC ? pv - BLC : 0;
				addPx(sig, gG_q, bg_G, kGtpos[i]);
			}
			for (int i = 0; i < 4; i++) {
				uint16_t pv = sa[kGbpos[i]];
				uint32_t sig = pv > BLC ? pv - BLC : 0;
				addPx(sig, gG_q, bg_G, kGbpos[i]);
			}
			for (int i = 0; i < 4; i++) {
				uint16_t pv = sa[kBpos[i]];
				uint32_t sig = pv > BLC ? pv - BLC : 0;
				addPx(sig, gB_q, bg_B, kBpos[i]);
			}
			macroCount += 16;
		}

		/*
		 * Fused bilinear post-pass: now that LUT has finished macro
		 * row mr, run the bilinear pass on macro row (mr - 1). This
		 * needs source rows from macros (mr-2, mr-1, mr) which are
		 * all in the per-thread ring (clamping mr-2 → mr-1 only at
		 * the TRUE frame top edge, mr_b == 0). At a thread's strip
		 * top edge (mr_b == mrStart) we read the stashed source row
		 * from the previous thread for prev_row3 instead of clamping.
		 *
		 * For the LAST macro row in this strip (mr_b == mrEnd - 1) we
		 * run bilinear AFTER the loop, reading either next strip's
		 * stashed row 0 (thread 0) or the frame-edge clamp (thread 1
		 * at the bottom edge).
		 */
		if (mr >= mrStart + 1) {
			unsigned int mr_b = mr - 1;        /* macro row to bilerp */
			const uint16_t *prev_row3;
			if (mr_b == 0) {
				/* True top edge — clamp prev to self. */
				prev_row3 = ringRow(threadIdx, mr_b, 3);
			} else if (mr_b == mrStart) {
				/* Strip top edge but not frame top — read stashed
				 * boundary row from the other thread's strip. */
				prev_row3 = boundaryT1PrevRow3;
			} else {
				prev_row3 = ringRow(threadIdx, mr_b - 1, 3);
			}
			uint16_t *out_base = rawPx + mr_b * 4 * strideU16;
			bilinearMacroRow(out_base,
					 prev_row3,
					 ringRow(threadIdx, mr_b, 0),
					 ringRow(threadIdx, mr_b, 1),
					 ringRow(threadIdx, mr_b, 2),
					 ringRow(threadIdx, mr_b, 3),
					 ringRow(threadIdx, mr,   0),
					 static_cast<int>(width),
					 strideU16);
		}
	}

	/*
	 * Bilinear for the LAST macro row of this strip. Reads next_row0 from
	 * the stashed boundary row (for thread 0, whose strip ends mid-frame)
	 * or clamps to self (for thread 1, whose strip ends at the true frame
	 * bottom edge).
	 */
	if (mrEnd > mrStart) {
		unsigned int mr_b = mrEnd - 1;
		const uint16_t *prev_row3;
		if (mr_b == 0) {
			prev_row3 = ringRow(threadIdx, mr_b, 3);
		} else if (mr_b == mrStart) {
			/* Single-row strip starting above frame top. */
			prev_row3 = boundaryT1PrevRow3;
		} else {
			prev_row3 = ringRow(threadIdx, mr_b - 1, 3);
		}
		const uint16_t *next_row0;
		if (mrEnd == macroRows) {
			/* True frame bottom — clamp next to self. */
			next_row0 = ringRow(threadIdx, mr_b, 0);
		} else {
			/* Strip bottom (mid-frame) — read stashed row from
			 * the next thread's first macro row. */
			next_row0 = boundaryT0NextRow0;
		}
		uint16_t *out_base = rawPx + mr_b * 4 * strideU16;
		bilinearMacroRow(out_base,
				 prev_row3,
				 ringRow(threadIdx, mr_b, 0),
				 ringRow(threadIdx, mr_b, 1),
				 ringRow(threadIdx, mr_b, 2),
				 ringRow(threadIdx, mr_b, 3),
				 next_row0,
				 static_cast<int>(width),
				 strideU16);
	}

	state.totalY = totalY;
	state.macroCount = macroCount;
	}; /* end processStrip lambda */

	/* Compute strip boundaries: thread 0 = [0, mrMid), thread 1 = [mrMid, macroRows).
	 * Stash the boundary source rows BEFORE launching threads so the LUT
	 * pass can't overwrite them concurrently. */
	const unsigned int mrMid = macroRows / 2;
	if (macroRows >= 2 && mrMid > 0 && mrMid < macroRows) {
		/* Thread 0's last macro row needs row 0 of macro row mrMid
		 * (thread 1's first macro row) as next_row0. */
		std::memcpy(boundaryT0NextRow0,
			    rawPx + mrMid * 4 * strideU16,
			    strideU16 * sizeof(uint16_t));
		/* Thread 1's first macro row needs row 3 of macro row mrMid-1
		 * (thread 0's last macro row) as prev_row3. */
		std::memcpy(boundaryT1PrevRow3,
			    rawPx + ((mrMid - 1) * 4 + 3) * strideU16,
			    strideU16 * sizeof(uint16_t));
	}

	/* Launch worker threads. If macroRows is tiny (< 2), fall back to
	 * a single-thread run on thread 0's state. */
	if (macroRows >= 2 && mrMid > 0 && mrMid < macroRows) {
		std::thread worker1(processStrip, 1u, mrMid, macroRows);
		processStrip(0u, 0u, mrMid);
		worker1.join();
	} else {
		processStrip(0u, 0u, macroRows);
	}

	auto t_lut = std::chrono::steady_clock::now();

	/* Merge per-thread stats into the output. */
	memset(st, 0, sizeof(*st));
	uint64_t totalY = 0;
	uint64_t macroCount = 0;
	for (unsigned int t = 0; t < kNumThreads; t++) {
		const auto &state = states[t];
		for (unsigned int i = 0; i < PISP_AGC_STATS_NUM_BINS; i++)
			st->agc.histogram[i] += state.hist4[0][i] + state.hist4[1][i]
					      + state.hist4[2][i] + state.hist4[3][i];
		for (unsigned int i = 0; i < PISP_AWB_STATS_NUM_ZONES; i++) {
			st->awb.zones[i].R_sum   += state.zones[i].R_sum;
			st->awb.zones[i].G_sum   += state.zones[i].G_sum;
			st->awb.zones[i].B_sum   += state.zones[i].B_sum;
			st->awb.zones[i].counted += state.zones[i].counted;
		}
		totalY += state.totalY;
		macroCount += state.macroCount;
	}
	st->agc.floating[0].Y_sum = totalY;
	st->agc.floating[0].counted = macroCount;

	auto t_end = std::chrono::steady_clock::now();
	double ms_prep   = std::chrono::duration_cast<std::chrono::microseconds>(t0 - t_memcpyStart).count() / 1000.0;
	double ms_fused  = std::chrono::duration_cast<std::chrono::microseconds>(t_lut - t0).count() / 1000.0;
	double ms_stats  = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_lut).count() / 1000.0;
	double ms_total  = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_memcpyStart).count() / 1000.0;
	LOG(RPI, Debug) << "QBC " << width << "x" << height
		<< " total=" << ms_total << "ms"
		<< " prep=" << ms_prep << "ms"
		<< " fused(lut+rb)=" << ms_fused << "ms"
		<< " stats=" << ms_stats << "ms";
}

} /* namespace libcamera */
