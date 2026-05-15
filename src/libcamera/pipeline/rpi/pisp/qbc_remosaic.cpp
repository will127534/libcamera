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
 * Up to 3 worker threads process disjoint horizontal strips of macro rows.
 * Each thread owns its own AGC histogram + AWB zone state; the parent merges
 * once all threads have joined.
 */

#include "qbc_remosaic.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

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

constexpr unsigned int N_THREADS = 3;

struct ThreadState {
	uint32_t hist[PISP_AGC_STATS_NUM_BINS] = {};
	pisp_awb_statistics_zone zones[PISP_AWB_STATS_NUM_ZONES] = {};
	uint64_t totalY = 0;
	uint64_t macroCount = 0;
};

} /* anonymous namespace */

void QbcRemosaic::process(uint16_t *rawPx, pisp_statistics *st,
			  unsigned int width, unsigned int height,
			  unsigned int strideBytes)
{
	const unsigned int strideU16 = strideBytes / 2;
	const unsigned int macroRows = height / 4;
	const unsigned int macroCols = width / 4;

	/* AWB cell sizing — libpisp finalise_awb() with offset=0. */
	const unsigned int awbCellW = std::max<unsigned int>(
		2u, 2u * ((width + PISP_AWB_STATS_SIZE) / (2u * PISP_AWB_STATS_SIZE)));
	const unsigned int awbCellH = std::max<unsigned int>(
		2u, 2u * ((height + PISP_AWB_STATS_SIZE) / (2u * PISP_AWB_STATS_SIZE)));

	/*
	 * Per-macro metering weight LUT — each 4×4 macro maps to one cell in
	 * the IPA's metering grid (15×15 for pisp). Built once per resolution,
	 * also rebuilt when setMeteringMode() invalidates macroWeight_.
	 * meteringMutex_ serialises the (rare) rebuild against
	 * setMeteringMode(); the inner per-macro loop later reads macroWeight_
	 * lock-free because nothing modifies it while we still hold the lock.
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
	 * Post-WB BT.601 Y coefficients baked into Q10, matching HW NQ's
	 * rgby block (pisp.cpp:1022): Y = R·gR·0.299 + G·gG·0.587 + B·gB·0.114.
	 */
	const uint32_t gR_q = static_cast<uint32_t>(
		std::clamp(wbGainR_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.299f * 1024.0f);
	const uint32_t gG_q = static_cast<uint32_t>(
		std::clamp(wbGainG_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.587f * 1024.0f);
	const uint32_t gB_q = static_cast<uint32_t>(
		std::clamp(wbGainB_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.114f * 1024.0f);

	auto t0 = std::chrono::steady_clock::now();

	/* Live BLC plumbed in from the V4L2 sensor subdev (pisp.cpp). */
	const uint32_t BLC = blackLevel_.load(std::memory_order_relaxed);
	auto signalBlk = [BLC](uint32_t bsum, unsigned int n) -> uint32_t {
		uint32_t nbl = n * BLC;
		return (bsum > nbl) ? (bsum - nbl) : 0;
	};

	auto processStrip = [&](ThreadState &state,
				unsigned int mrStart, unsigned int mrEnd) {
		for (unsigned int mr = mrStart; mr < mrEnd; mr++) {
			const unsigned int my = mr * 4;
			const unsigned int cellY = std::min<unsigned int>(
				my / awbCellH, PISP_AWB_STATS_SIZE - 1);
			uint16_t * const rowBase = rawPx + my * strideU16;

			for (unsigned int mc = 0; mc < macroCols; mc++) {
				const unsigned int mx = mc * 4;
				uint16_t * const m = rowBase + mx;

				/* Load 16 source pixels (4 rows × 4 cols). */
				uint16_t s[16];
				for (int r = 0; r < 4; r++) {
					const uint16_t *rp = m + r * strideU16;
					s[r * 4 + 0] = rp[0];
					s[r * 4 + 1] = rp[1];
					s[r * 4 + 2] = rp[2];
					s[r * 4 + 3] = rp[3];
				}

				/* Sub-block sums (one per 2×2 sub-block of same colour). */
				uint32_t Rsum  = (uint32_t)s[0]  + s[1]  + s[4]  + s[5];
				uint32_t Gtsum = (uint32_t)s[2]  + s[3]  + s[6]  + s[7];
				uint32_t Gbsum = (uint32_t)s[8]  + s[9]  + s[12] + s[13];
				uint32_t Bsum  = (uint32_t)s[10] + s[11] + s[14] + s[15];

				/* Gradients on sub-block sums (no /4 → just scaled gradient,
				 * comparison is to a scaled threshold). */
				int32_t gH = std::abs((int32_t)(Rsum + Gbsum) -
						      (int32_t)(Gtsum + Bsum));
				int32_t gV = std::abs((int32_t)(Rsum + Gtsum) -
						      (int32_t)(Gbsum + Bsum));

				/* Direction-aware blend at every macro — no smooth-mode
				 * branch. The sub-block-average tile we used in the old
				 * smooth path produced visible 4×4 quantization blocks in
				 * flat colour regions (adjacent macros' averages differ
				 * slightly from sensor noise, so each macro emits a flat
				 * tile at its own average → eye reads it as posterization).
				 * The directional blend below naturally handles smooth
				 * regions: when gH ≈ gV ≈ 0, α ≈ 0.5 and the per-pixel
				 * noise from both H_src and V_src is preserved, no tile
				 * boundaries appear.
				 *
				 * α ∈ [0, 32768], large = vertical-edge dominant (gH
				 * dominates) → bias toward V_src so interpolation stays
				 * on one side of the edge. ε = 1024 stabilises very-low-
				 * gradient cases (avoids a noise-driven 0/0). */
				uint16_t out[16];
				uint64_t total = (uint64_t)gH + (uint64_t)gV + 1024;
				uint32_t alpha = (uint32_t)
					(((uint64_t)gH << 15) / total);
				for (int idx = 0; idx < 16; idx++) {
					uint16_t hv = s[kRemosaicLUT[idx].h_src];
					uint16_t vv = s[kRemosaicLUT[idx].v_src];
					if (hv == vv) {
						out[idx] = hv;
					} else {
						int32_t diff = (int32_t)vv - (int32_t)hv;
						int32_t blended = (int32_t)hv +
							(int32_t)(((int64_t)alpha * diff) >> 15);
						out[idx] = (uint16_t)std::clamp(
							blended, 0, 65535);
					}
				}

				/* Write back 16 output pixels. */
				for (int r = 0; r < 4; r++) {
					uint16_t *rp = m + r * strideU16;
					rp[0] = out[r * 4 + 0];
					rp[1] = out[r * 4 + 1];
					rp[2] = out[r * 4 + 2];
					rp[3] = out[r * 4 + 3];
				}

				/* Stats: per-macro signal sums (post-BLC) for AWB zones.
				 * HW NQ averages Gr+Gb into one G per 2×2 cell — match
				 * by halving the 8-pixel G sum. */
				uint32_t r4 = signalBlk(Rsum, 4);
				uint32_t g4 = signalBlk(Gtsum + Gbsum, 8) >> 1;
				uint32_t b4 = signalBlk(Bsum, 4);

				const unsigned int cellX = std::min<unsigned int>(
					mx / awbCellW, PISP_AWB_STATS_SIZE - 1);
				pisp_awb_statistics_zone &z =
					state.zones[cellY * PISP_AWB_STATS_SIZE + cellX];
				z.R_sum   += r4;
				z.G_sum   += g4;
				z.B_sum   += b4;
				z.counted += 4;

				/*
				 * Post-WB BT.601 Y *per pixel* for the AGC histogram and
				 * Y_sum, matching HW NQ semantics: each of the 16 source
				 * pixels contributes one bin entry. Earlier per-macro
				 * version (1 Y from sub-block averages, weight 16) hid
				 * highlight pixels in the macro's mean → AGC saturation-
				 * aware gain projection under-estimated the saturation
				 * risk and over-exposed (visible as magenta cast on near-
				 * white highlights at higher CCTs).
				 *
				 * For each pixel, Y is computed using its actual raw value
				 * for its sampled colour plus the local sub-block averages
				 * for the other two colours (closest available estimate at
				 * the pixel's spatial position). Per-pixel averages:
				 *   R_avg, GT_avg, GB_avg, B_avg  (post-BLC, /4)
				 *   G_avg = (GT_avg + GB_avg) / 2 used at R/B positions
				 * Pre-compute the 3 "background" terms (the constant
				 * contribution from the two non-sampled colours per macro)
				 * so the inner loop is one multiply + one add per pixel.
				 */
				uint32_t Ravg  = r4;       /* already 4-pixel sum post-BLC */
				uint32_t GTavg = signalBlk(Gtsum, 4);
				uint32_t GBavg = signalBlk(Gbsum, 4);
				uint32_t Bavg  = b4;
				Ravg >>= 2; GTavg >>= 2; GBavg >>= 2; Bavg >>= 2;
				uint32_t Gavg  = (GTavg + GBavg) >> 1;

				/* bg_R = G_avg*gG_q + B_avg*gB_q (for R-position pixels) */
				uint64_t bg_R = (uint64_t)Gavg  * gG_q +
						(uint64_t)Bavg  * gB_q;
				/* bg_G = R_avg*gR_q + B_avg*gB_q (for G-position pixels) */
				uint64_t bg_G = (uint64_t)Ravg  * gR_q +
						(uint64_t)Bavg  * gB_q;
				/* bg_B = R_avg*gR_q + G_avg*gG_q (for B-position pixels) */
				uint64_t bg_B = (uint64_t)Ravg  * gR_q +
						(uint64_t)Gavg  * gG_q;

				const unsigned int w = macroWeight_.empty()
					? 1u
					: macroWeight_[mr * macroCols + mc];

				/*
				 * QBC pixel-colour map (idx = row*4+col):
				 *   row 0-1 col 0-1 → R   (idx 0,1,4,5)
				 *   row 0-1 col 2-3 → Gt  (idx 2,3,6,7)
				 *   row 2-3 col 0-1 → Gb  (idx 8,9,12,13)
				 *   row 2-3 col 2-3 → B   (idx 10,11,14,15)
				 * Iterate by colour group so each block uses one bg term.
				 */
				static const uint8_t kRpos[4]  = { 0,  1,  4,  5};
				static const uint8_t kGtpos[4] = { 2,  3,  6,  7};
				static const uint8_t kGbpos[4] = { 8,  9, 12, 13};
				static const uint8_t kBpos[4]  = {10, 11, 14, 15};

				auto addPx = [&](uint32_t v_signal, uint32_t coef,
						 uint64_t bg) {
					uint64_t y_q = (uint64_t)v_signal * coef + bg;
					uint32_t y = (uint32_t)(y_q >> 10);
					if (y > 65535)
						y = 65535;
					state.hist[y >> PISP_AGC_HIST_BIN_SHIFT] += w;
					state.totalY += y;
				};
				for (int i = 0; i < 4; i++) {
					uint16_t pv = s[kRpos[i]];
					uint32_t sig = pv > BLC ? pv - BLC : 0;
					addPx(sig, gR_q, bg_R);
				}
				for (int i = 0; i < 4; i++) {
					uint16_t pv = s[kGtpos[i]];
					uint32_t sig = pv > BLC ? pv - BLC : 0;
					addPx(sig, gG_q, bg_G);
				}
				for (int i = 0; i < 4; i++) {
					uint16_t pv = s[kGbpos[i]];
					uint32_t sig = pv > BLC ? pv - BLC : 0;
					addPx(sig, gG_q, bg_G);
				}
				for (int i = 0; i < 4; i++) {
					uint16_t pv = s[kBpos[i]];
					uint32_t sig = pv > BLC ? pv - BLC : 0;
					addPx(sig, gB_q, bg_B);
				}
				state.macroCount += 16;
			}
		}
	};

	ThreadState states[N_THREADS];
	std::thread workers[N_THREADS];
	const unsigned int rowsPerThread =
		(macroRows + N_THREADS - 1) / N_THREADS;

	for (unsigned int t = 0; t < N_THREADS; t++) {
		unsigned int mrStart = t * rowsPerThread;
		unsigned int mrEnd = std::min(mrStart + rowsPerThread, macroRows);
		if (mrStart >= mrEnd)
			continue;
		workers[t] = std::thread(processStrip,
					 std::ref(states[t]), mrStart, mrEnd);
	}
	for (auto &th : workers) {
		if (th.joinable())
			th.join();
	}

	/* Merge per-thread stats into the output. */
	memset(st, 0, sizeof(*st));
	uint64_t totalY = 0;
	uint64_t macroCount = 0;
	for (unsigned int t = 0; t < N_THREADS; t++) {
		for (unsigned int i = 0; i < PISP_AGC_STATS_NUM_BINS; i++)
			st->agc.histogram[i] += states[t].hist[i];
		for (unsigned int i = 0; i < PISP_AWB_STATS_NUM_ZONES; i++) {
			st->awb.zones[i].R_sum   += states[t].zones[i].R_sum;
			st->awb.zones[i].G_sum   += states[t].zones[i].G_sum;
			st->awb.zones[i].B_sum   += states[t].zones[i].B_sum;
			st->awb.zones[i].counted += states[t].zones[i].counted;
		}
		totalY += states[t].totalY;
		macroCount += states[t].macroCount;
	}
	st->agc.floating[0].Y_sum = totalY;
	st->agc.floating[0].counted = macroCount;

	LOG(RPI, Debug) << "QBC " << width << "x" << height << " "
		<< std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now() - t0).count() / 1000.0 << " ms";
}

} /* namespace libcamera */
