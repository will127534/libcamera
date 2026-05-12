/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * qbc_remosaic.cpp - SW QBC remosaic + stats producer (see qbc_remosaic.h).
 */

#include "qbc_remosaic.h"

#include <algorithm>
#include <arm_neon.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libpisp/frontend/pisp_statistics.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RPI)

QbcRemosaic::QbcRemosaic()
{
}

void QbcRemosaic::loadMeteringWeights(const std::string &sensorModel)
{
	meteringWeights_.clear();
	macroWeight_.clear();
	meteringGridW_ = meteringGridH_ = 0;

	const std::string tuningPath =
		"/usr/local/share/libcamera/ipa/rpi/pisp/" + sensorModel + ".json";
	File f(tuningPath);
	if (!f.open(File::OpenModeFlag::ReadOnly)) {
		LOG(RPI, Warning) << "QBC: cannot open tuning " << tuningPath
				  << " — falling back to uniform metering";
		return;
	}
	std::unique_ptr<YamlObject> root = YamlParser::parse(f);
	if (!root) {
		LOG(RPI, Warning) << "QBC: failed to parse " << tuningPath;
		return;
	}

	/* Walk: algorithms[].rpi.agc.metering_modes.<active>.weights */
	const YamlObject &algos = (*root)["algorithms"];
	for (const YamlObject &entry : algos.asList()) {
		const YamlObject &agc = entry["rpi.agc"];
		if (!agc.isDictionary())
			continue;
		const YamlObject &modes = agc["metering_modes"];
		if (!modes.isDictionary())
			continue;
		const YamlObject *pick = nullptr;
		for (const auto &[name, node] : modes.asDict()) {
			if (name == "centre-weighted") { pick = &node; break; }
			if (!pick) pick = &node;
		}
		if (!pick)
			continue;
		const YamlObject &weights = (*pick)["weights"];
		if (!weights.size())
			continue;
		meteringWeights_.reserve(weights.size());
		for (const YamlObject &w : weights.asList())
			meteringWeights_.push_back(
				w.get<uint16_t>().value_or(1));
		break;
	}
	if (meteringWeights_.empty()) {
		LOG(RPI, Warning) << "QBC: no metering weights in tuning — uniform";
		return;
	}

	unsigned int n = meteringWeights_.size();
	unsigned int side = static_cast<unsigned int>(std::sqrt(n));
	if (side * side != n) {
		LOG(RPI, Warning) << "QBC: non-square metering grid (" << n
				  << ") — ignoring weights";
		meteringWeights_.clear();
		return;
	}
	meteringGridW_ = meteringGridH_ = side;
	LOG(RPI, Info) << "QBC: loaded " << side << "×" << side
		       << " AGC metering grid from " << tuningPath;
}

void QbcRemosaic::updateWbGains(const ControlList &metadata)
{
	const auto &gains = metadata.get(controls::ColourGains);
	if (!gains)
		return;
	wbGainR_.store((*gains)[0], std::memory_order_relaxed);
	wbGainB_.store((*gains)[1], std::memory_order_relaxed);
	/* G gain is the reference (always 1.0); store it for symmetry. */
	wbGainG_.store(1.0f, std::memory_order_relaxed);
}

void QbcRemosaic::process(uint16_t *rawPx, pisp_statistics *st,
			  unsigned int width, unsigned int height,
			  unsigned int strideBytes)
{
	/* AWB cell sizing — libpisp finalise_awb() with offset=0. */
	const unsigned int awbCellW = std::max<unsigned int>(
		2u, 2u * ((width + PISP_AWB_STATS_SIZE) / (2u * PISP_AWB_STATS_SIZE)));
	const unsigned int awbCellH = std::max<unsigned int>(
		2u, 2u * ((height + PISP_AWB_STATS_SIZE) / (2u * PISP_AWB_STATS_SIZE)));

	/* Per-macro scatter into the stats dma-buf would be slow if uncached;
	 * accumulate locally, memcpy out once at the end. */
	uint32_t localHist[PISP_AGC_STATS_NUM_BINS] = {};
	pisp_awb_statistics_zone localZones[PISP_AWB_STATS_NUM_ZONES] = {};

	const unsigned int strideU16 = strideBytes / 2;
	const unsigned int macroRows = height / 4;
	const unsigned int macroCols = width / 4;
	const unsigned int macroPairs = macroCols / 2;

	if (pairCellX_.size() != macroPairs * 2) {
		pairCellX_.resize(macroPairs * 2);
		for (unsigned int m = 0; m < macroPairs * 2; m++)
			pairCellX_[m] = static_cast<uint16_t>(std::min<unsigned int>(
				(m * 4) / awbCellW, PISP_AWB_STATS_SIZE - 1));
	}

	/*
	 * Build the per-macro metering weight LUT once per resolution. Each
	 * macro (4×4 pixels) maps to a cell in the IPA's metering grid (15×15
	 * for pisp). Weight 0 means that macro contributes nothing to the
	 * histogram — matches HW NQ behaviour where w_i = 0 zones are skipped.
	 */
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

	uint64_t totalY = 0;
	auto t0 = std::chrono::steady_clock::now();

	/*
	 * AGC histogram is a Y (luma) histogram with WB gains applied
	 * (RPi tuning guide §5.9.4 — Pi 5 has access to a proper Y histogram;
	 * pisp.cpp declares AgcStatsPos::PostWb so the controller does NOT
	 * undo WB gains). Use the previous-frame ColourGains as a one-frame-late
	 * approximation; converges with AWB.
	 *
	 * Quantize gains to Q10 fixed-point. Clamp to a sane range to bound
	 * the integer math: max r0s≈4·62320, max gain·Q10≈8·1024, sum fits in u64.
	 */
	const uint32_t gR_q = static_cast<uint32_t>(
		std::clamp(wbGainR_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 1024.0f);
	const uint32_t gG_q = static_cast<uint32_t>(
		std::clamp(wbGainG_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 1024.0f);
	const uint32_t gB_q = static_cast<uint32_t>(
		std::clamp(wbGainB_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 1024.0f);

	constexpr uint16_t BLC = 3200;
	static const uint8_t kPermBytes[16] = {
		0,1,  4,5,  2,3,  6,7,
		8,9,  12,13, 10,11, 14,15,
	};
	const uint8x16_t perm = vld1q_u8(kPermBytes);

	auto signalBlk = [](uint32_t bsum, unsigned int n) -> uint32_t {
		uint32_t nbl = n * BLC;
		return (bsum > nbl) ? (bsum - nbl) : 0;
	};

	for (unsigned int mr = 0; mr < macroRows; mr++) {
		const unsigned int my = mr * 4;
		uint16_t *row0 = rawPx + my * strideU16;
		uint16_t *row1 = row0 + strideU16;
		uint16_t *row2 = row0 + 2 * strideU16;
		uint16_t *row3 = row0 + 3 * strideU16;

		const unsigned int cellY = std::min<unsigned int>(my / awbCellH,
								  PISP_AWB_STATS_SIZE - 1);
		pisp_awb_statistics_zone *zoneRow = &localZones[cellY * PISP_AWB_STATS_SIZE];

		uint32_t accR = 0, accG = 0, accB = 0, accCount = 0;
		unsigned int curCellX = 0;

		auto flushAndAdvance = [&](unsigned int newCellX) {
			if (newCellX != curCellX) {
				zoneRow[curCellX].R_sum   += accR;
				zoneRow[curCellX].G_sum   += accG;
				zoneRow[curCellX].B_sum   += accB;
				zoneRow[curCellX].counted += accCount;
				accR = accG = accB = accCount = 0;
				curCellX = newCellX;
			}
		};

		/*
		 * NEON inner loop: 2 macros (8 cols) per iteration.
		 *
		 * Source layout per macro (RGGB-quadbayer):
		 *   [R R G G]   ← row0
		 *   [R R G G]   ← row1
		 *   [G G B B]   ← row2
		 *   [G G B B]   ← row3
		 *
		 * Per row, the source 8-lane vector is
		 *   [m0c0 m0c1 m0c2 m0c3 m1c0 m1c1 m1c2 m1c3]
		 * Intra-group swap permutation [0,2,1,3,4,6,5,7] turns each row into
		 * a standard 2x2-RGGB row pattern (RGRG / GBGB tiled).
		 *
		 * Output row order is also swapped: src row 1 ↔ src row 2 (so the
		 * full 4x4 output is RGGB-tiled, not GBGB-RGRG-tiled).
		 *
		 * Each source pixel maps to a unique output position (no averaging),
		 * preserving sub-block resolution that block-average would destroy.
		 */
		for (unsigned int p = 0; p < macroPairs; p++) {
			const unsigned int mx = p * 8;

			uint16x8_t v0 = vld1q_u16(row0 + mx);
			uint16x8_t v1 = vld1q_u16(row1 + mx);
			uint16x8_t v2 = vld1q_u16(row2 + mx);
			uint16x8_t v3 = vld1q_u16(row3 + mx);

			/*
			 * Block sums (sum of 4 pixels per 2x2 block) computed in u32
			 * to survive 16-bit-shifted values that would overflow a u16
			 * row-sum (max 4 * 65520 = 262080).
			 */
			uint32x4_t v0pair = vpaddlq_u16(v0);
			uint32x4_t v1pair = vpaddlq_u16(v1);
			uint32x4_t v2pair = vpaddlq_u16(v2);
			uint32x4_t v3pair = vpaddlq_u16(v3);
			uint32x4_t topPairs = vaddq_u32(v0pair, v1pair);
			uint32x4_t botPairs = vaddq_u32(v2pair, v3pair);
			/* topPairs = [R_m0, Gtop_m0, R_m1, Gtop_m1]
			 * botPairs = [Gbot_m0, B_m0, Gbot_m1, B_m1] */

			/* Permute (intra-group swap RRGG → RGRG) and write back. */
			uint16x8_t v0p = vreinterpretq_u16_u8(vqtbl1q_u8(vreinterpretq_u8_u16(v0), perm));
			uint16x8_t v1p = vreinterpretq_u16_u8(vqtbl1q_u8(vreinterpretq_u8_u16(v1), perm));
			uint16x8_t v2p = vreinterpretq_u16_u8(vqtbl1q_u8(vreinterpretq_u8_u16(v2), perm));
			uint16x8_t v3p = vreinterpretq_u16_u8(vqtbl1q_u8(vreinterpretq_u8_u16(v3), perm));

			vst1q_u16(row0 + mx, v0p);
			vst1q_u16(row1 + mx, v2p);  /* src row 2 → out row 1 */
			vst1q_u16(row2 + mx, v1p);  /* src row 1 → out row 2 */
			vst1q_u16(row3 + mx, v3p);

			/* AWB block sums (raw, post-BLC). */
			uint32_t lanes[8];
			vst1q_u32(lanes,     topPairs);
			vst1q_u32(lanes + 4, botPairs);

			/* Per macro: R signal sum = signal_per_pixel × 4 pixels.
			 * G signal sum (halved) = signal_per_pixel × 4 pixels (since
			 * G has 8 pixels per macro, halving gives 4 G_avg samples
			 * matching HW NQ's per-cell-G counting). B signal × 4 pixels.
			 * accCount += 4 (one count per binned-cell-equivalent). */
			uint32_t r0s = signalBlk(lanes[0], 4);
			uint32_t g0s = signalBlk(lanes[1] + lanes[4], 8) >> 1;
			uint32_t b0s = signalBlk(lanes[5], 4);
			uint32_t r1s = signalBlk(lanes[2], 4);
			uint32_t g1s = signalBlk(lanes[3] + lanes[6], 8) >> 1;
			uint32_t b1s = signalBlk(lanes[7], 4);

			/*
			 * Per-cell post-WB BT.601 luma for the AGC histogram.
			 * r0s/g0s/b0s are post-BLC signal sums of 4 same-channel
			 * pixels per macro (so each is 4× the per-pixel value).
			 * With gains in Q10:
			 *   Y_pixel = (R·gR + 2·G·gG + B·gB) / 4
			 *           = (r·gR + 2·g·gG + b·gB) / 16
			 * Including the Q10 un-quantize, divide by 16 · 1024 = 2^14.
			 */
			auto yBin = [&](uint32_t r, uint32_t g, uint32_t b) -> unsigned int {
				uint64_t y_q = uint64_t(r) * gR_q
					     + 2ull * uint64_t(g) * gG_q
					     + uint64_t(b) * gB_q;
				uint32_t y = static_cast<uint32_t>(y_q >> 14);
				if (y > 65535) y = 65535;
				return y >> 6;
			};
			/* Per-pixel count weighted by metering w_i (RPi tuning guide
			 * §5.9.4: Pi 5 counts each pixel w_i times). Falls back to
			 * uniform w=1 when the weights table didn't load. */
			unsigned int macroIdx0 = mr * macroCols + 2 * p;
			unsigned int macroIdx1 = macroIdx0 + 1;
			unsigned int w0 = macroWeight_.empty() ? 1 : macroWeight_[macroIdx0];
			unsigned int w1 = macroWeight_.empty() ? 1 : macroWeight_[macroIdx1];
			localHist[yBin(r0s, g0s, b0s)] += 16 * w0;
			localHist[yBin(r1s, g1s, b1s)] += 16 * w1;

			flushAndAdvance(pairCellX_[2 * p]);
			accR += r0s; accG += g0s; accB += b0s; accCount += 4;
			totalY += (r0s + 2 * g0s + b0s) >> 4;

			flushAndAdvance(pairCellX_[2 * p + 1]);
			accR += r1s; accG += g1s; accB += b1s; accCount += 4;
			totalY += (r1s + 2 * g1s + b1s) >> 4;
		}

		zoneRow[curCellX].R_sum   += accR;
		zoneRow[curCellX].G_sum   += accG;
		zoneRow[curCellX].B_sum   += accB;
		zoneRow[curCellX].counted += accCount;
	}

	/* IPA reads all of agc.floating[1..3] + cdaf.foms[] too — zero everything
	 * before writing the fields we use. lux.cpp divides counted; must be nonzero. */
	memset(st, 0, sizeof(*st));
	memcpy(st->awb.zones, localZones, sizeof(localZones));
	memcpy(st->agc.histogram, localHist, sizeof(localHist));
	st->agc.floating[0].Y_sum = totalY;
	st->agc.floating[0].counted = macroRows * macroCols;

	LOG(RPI, Debug) << "QBC " << width << "x" << height << " "
		<< std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now() - t0).count() / 1000.0 << " ms";
}

} /* namespace libcamera */
