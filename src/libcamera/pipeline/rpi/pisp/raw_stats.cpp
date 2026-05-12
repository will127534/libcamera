/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * raw_stats.cpp — SW stats producer for non-QBC raw paths (see raw_stats.h).
 *
 * The kernel walks the input as standard 2×2 RGGB Bayer pairs (one R, two G,
 * one B per cell). For each cell it post-BLC subtracts and accumulates into
 * the AWB zone, the BT.601 Y for the AGC histogram (weighted by metering
 * w_i), and the AGC Y_sum/counted floating zone. NEON-vectorised across 8
 * source columns per inner iteration.
 */

#include "raw_stats.h"

#include <algorithm>
#include <arm_neon.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RPI)

RawStatsProducer::RawStatsProducer()
{
}

void RawStatsProducer::loadMeteringWeights(const std::string &sensorModel)
{
	meteringWeights_.clear();
	cellWeight_.clear();
	meteringGridW_ = meteringGridH_ = 0;

	const std::string tuningPath =
		"/usr/local/share/libcamera/ipa/rpi/pisp/" + sensorModel + ".json";
	File f(tuningPath);
	if (!f.open(File::OpenModeFlag::ReadOnly)) {
		LOG(RPI, Warning) << "RawStats: cannot open " << tuningPath
				  << " — uniform metering";
		return;
	}
	std::unique_ptr<YamlObject> root = YamlParser::parse(f);
	if (!root)
		return;

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
		meteringWeights_.reserve(weights.size());
		for (const YamlObject &w : weights.asList())
			meteringWeights_.push_back(
				w.get<uint16_t>().value_or(1));
		break;
	}
	if (meteringWeights_.empty())
		return;

	unsigned int n = meteringWeights_.size();
	unsigned int side = static_cast<unsigned int>(std::sqrt(n));
	if (side * side != n) {
		meteringWeights_.clear();
		return;
	}
	meteringGridW_ = meteringGridH_ = side;
}

void RawStatsProducer::updateWbGains(const ControlList &metadata)
{
	const auto &gains = metadata.get(controls::ColourGains);
	if (!gains)
		return;
	wbGainR_.store((*gains)[0], std::memory_order_relaxed);
	wbGainB_.store((*gains)[1], std::memory_order_relaxed);
	wbGainG_.store(1.0f, std::memory_order_relaxed);
}

void RawStatsProducer::process(const uint16_t *rawPx, pisp_statistics *st,
			       unsigned int width, unsigned int height,
			       unsigned int strideBytes)
{
	const unsigned int awbCellW = std::max<unsigned int>(
		2u, 2u * ((width + PISP_AWB_STATS_SIZE) / (2u * PISP_AWB_STATS_SIZE)));
	const unsigned int awbCellH = std::max<unsigned int>(
		2u, 2u * ((height + PISP_AWB_STATS_SIZE) / (2u * PISP_AWB_STATS_SIZE)));

	uint32_t localHist[PISP_AGC_STATS_NUM_BINS] = {};
	pisp_awb_statistics_zone localZones[PISP_AWB_STATS_NUM_ZONES] = {};

	const unsigned int strideU16 = strideBytes / 2;
	const unsigned int cellRows = height / 2;   /* one RGGB pair per row */
	const unsigned int cellCols = width / 2;
	const unsigned int cellPairs = cellCols / 4; /* NEON 8 source cols = 4 RGGB pairs */

	if (cellWeight_.size() != cellRows * cellCols &&
	    meteringGridW_ > 0 && meteringGridH_ > 0) {
		cellWeight_.resize(cellRows * cellCols);
		for (unsigned int cr = 0; cr < cellRows; cr++) {
			unsigned int gy = std::min(meteringGridH_ - 1,
						   (cr * meteringGridH_) / cellRows);
			for (unsigned int cc = 0; cc < cellCols; cc++) {
				unsigned int gx = std::min(meteringGridW_ - 1,
							   (cc * meteringGridW_) / cellCols);
				cellWeight_[cr * cellCols + cc] =
					static_cast<uint8_t>(meteringWeights_[gy * meteringGridW_ + gx]);
			}
		}
	}

	uint64_t totalY = 0;
	auto t0 = std::chrono::steady_clock::now();

	const uint32_t gR_q = static_cast<uint32_t>(
		std::clamp(wbGainR_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 1024.0f);
	const uint32_t gG_q = static_cast<uint32_t>(
		std::clamp(wbGainG_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 1024.0f);
	const uint32_t gB_q = static_cast<uint32_t>(
		std::clamp(wbGainB_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 1024.0f);

	/*
	 * BLC: at higher bit depths the sensor reports a wider raw value but
	 * the pedestal stays at the same physical level. The IPA tuning's
	 * black_level entry is in 16-bit-shifted units; with libcamera's CFE
	 * unpack we always see left-justified samples in u16 lanes, so BLC=3200
	 * (= 200 in 12-bit raw, ×16) lines up regardless of native depth.
	 */
	constexpr uint16_t BLC = 3200;

	auto signalBlk = [](uint32_t bsum, unsigned int n) -> uint32_t {
		uint32_t nbl = n * BLC;
		return (bsum > nbl) ? (bsum - nbl) : 0;
	};

	for (unsigned int cr = 0; cr < cellRows; cr++) {
		const unsigned int y0 = cr * 2;
		const uint16_t *row0 = rawPx + y0 * strideU16;
		const uint16_t *row1 = row0 + strideU16;

		const unsigned int cellY = std::min<unsigned int>(y0 / awbCellH,
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

		for (unsigned int p = 0; p < cellPairs; p++) {
			const unsigned int x0 = p * 8;

			uint16x8_t v0 = vld1q_u16(row0 + x0);  /* R G R G R G R G */
			uint16x8_t v1 = vld1q_u16(row1 + x0);  /* G B G B G B G B */

			/* Deinterleave evens/odds in each row, widen to u32 so
			 * the per-zone accumulations don't wrap. */
			uint32x4_t r_evens = vmovl_u16(vget_low_u16(vuzp1q_u16(v0, v0)));
			uint32x4_t g_odds  = vmovl_u16(vget_low_u16(vuzp2q_u16(v0, v0)));
			uint32x4_t g_evens = vmovl_u16(vget_low_u16(vuzp1q_u16(v1, v1)));
			uint32x4_t b_odds  = vmovl_u16(vget_low_u16(vuzp2q_u16(v1, v1)));

			/* 4 RGGB cells in this pair of lanes; collapse to scalar. */
			uint32_t r_sum_4 = vaddvq_u32(r_evens);
			uint32_t g_sum_8 = vaddvq_u32(g_odds) + vaddvq_u32(g_evens);
			uint32_t b_sum_4 = vaddvq_u32(b_odds);

			uint32_t rSig = signalBlk(r_sum_4, 4);
			uint32_t gSig = signalBlk(g_sum_8, 8) >> 1;  /* halve to match
									 * per-pixel scale */
			uint32_t bSig = signalBlk(b_sum_4, 4);

			/* Each cell-pair (4 RGGB cells) belongs to one AWB zone
			 * (cells in a row are striped by awbCellW pixels). Walk
			 * by AWB-cell-X. */
			unsigned int xPair = x0 * 2;  /* pair in pixel coords */
			unsigned int cellX = std::min<unsigned int>(
				xPair / awbCellW, PISP_AWB_STATS_SIZE - 1);
			flushAndAdvance(cellX);

			accR += rSig;
			accG += gSig;
			accB += bSig;
			accCount += 4;

			/* AGC histogram — one Y per cell-pair, weighted by w_i. */
			uint64_t y_q = uint64_t(rSig) * gR_q
				     + 2ull * uint64_t(gSig) * gG_q
				     + uint64_t(bSig) * gB_q;
			uint32_t y = static_cast<uint32_t>(y_q >> 14);
			if (y > 65535)
				y = 65535;
			unsigned int bin = y >> 6;

			/* The pair covers 4 RGGB cells (= 16 pixels). Sum the
			 * cell-weights for proper metering, default to 4 (=
			 * uniform w_i = 1 for all 4 cells). */
			unsigned int w = 4;
			if (!cellWeight_.empty()) {
				unsigned int base = cr * cellCols + 4 * p;
				w = (unsigned int)cellWeight_[base]
				  + (unsigned int)cellWeight_[base + 1]
				  + (unsigned int)cellWeight_[base + 2]
				  + (unsigned int)cellWeight_[base + 3];
			}
			localHist[bin] += 4 * w;
			totalY += (rSig + 2 * gSig + bSig) >> 4;
		}

		zoneRow[curCellX].R_sum   += accR;
		zoneRow[curCellX].G_sum   += accG;
		zoneRow[curCellX].B_sum   += accB;
		zoneRow[curCellX].counted += accCount;
	}

	memset(st, 0, sizeof(*st));
	memcpy(st->awb.zones, localZones, sizeof(localZones));
	memcpy(st->agc.histogram, localHist, sizeof(localHist));
	st->agc.floating[0].Y_sum = totalY;
	st->agc.floating[0].counted = cellRows * (cellCols / 4);

	LOG(RPI, Debug) << "RawStats " << width << "x" << height << " "
		<< std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now() - t0).count() / 1000.0 << " ms";
}

} /* namespace libcamera */
