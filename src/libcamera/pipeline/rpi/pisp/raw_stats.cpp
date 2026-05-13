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

namespace {

/*
 * Pull the weights[] grid for `modeName` out of the rpi.agc metering_modes
 * dict at /usr/local/share/libcamera/ipa/rpi/pisp/<sensorModel>.json.
 *
 * Two tuning structures are supported (matches what the AGC IPA accepts):
 *   1. Flat:        rpi.agc.metering_modes.<name>.weights  (imx294-style)
 *   2. Multi-chan:  rpi.agc.channels[].metering_modes.<name>.weights
 *                                                          (imx585 HDR-style)
 * For (2) we read from channel 0 — that's the primary AGC channel the IPA
 * runs on for the SW-stats path (HDR channel selection happens above and
 * doesn't affect the SW kernel's weight lookup).
 *
 * Empty result if anything fails. `firstName`, if non-null, is set to the
 * first-listed mode found (matches the IPA's defaultMeteringMode).
 */
std::vector<uint16_t> loadMeteringWeightsForMode(const std::string &sensorModel,
						 const std::string &modeName,
						 std::string *firstName = nullptr)
{
	std::vector<uint16_t> out;
	const std::string tuningPath =
		"/usr/local/share/libcamera/ipa/rpi/pisp/" + sensorModel + ".json";
	File f(tuningPath);
	if (!f.open(File::OpenModeFlag::ReadOnly))
		return out;
	std::unique_ptr<YamlObject> root = YamlParser::parse(f);
	if (!root)
		return out;

	auto pickFromModes = [&](const YamlObject &modes) -> const YamlObject * {
		const YamlObject *pick = nullptr;
		for (const auto &[name, node] : modes.asDict()) {
			if (firstName && firstName->empty())
				*firstName = name;
			if (name == modeName) {
				pick = &node;
				break;
			}
		}
		return pick;
	};

	for (const YamlObject &entry : (*root)["algorithms"].asList()) {
		const YamlObject &agc = entry["rpi.agc"];
		if (!agc.isDictionary())
			continue;

		/* Flat (imx294) or multi-channel (imx585) layout. */
		const YamlObject *modes = nullptr;
		const YamlObject &flat = agc["metering_modes"];
		if (flat.isDictionary())
			modes = &flat;
		else if (agc["channels"].isList() && agc["channels"].size() > 0) {
			const YamlObject &ch0 = agc["channels"][std::size_t(0)];
			const YamlObject &chmm = ch0["metering_modes"];
			if (chmm.isDictionary())
				modes = &chmm;
		}
		if (!modes)
			continue;

		const YamlObject *pick = pickFromModes(*modes);
		if (!pick)
			break;
		const YamlObject &weights = (*pick)["weights"];
		out.reserve(weights.size());
		for (const YamlObject &w : weights.asList())
			out.push_back(w.get<uint16_t>().value_or(1));
		break;
	}
	return out;
}

} /* anonymous namespace */

void RawStatsProducer::loadMeteringWeights(const std::string &sensorModel)
{
	std::lock_guard<std::mutex> lk(meteringMutex_);

	sensorModel_ = sensorModel;
	cellWeight_.clear();
	meteringGridW_ = meteringGridH_ = 0;
	meteringWeights_.clear();

	/* Look up the first-listed mode (= IPA's defaultMeteringMode). */
	std::string first;
	auto weights = loadMeteringWeightsForMode(sensorModel, "", &first);
	if (first.empty()) {
		LOG(RPI, Warning) << "RawStats: no metering modes in tuning for "
				  << sensorModel << " — uniform metering";
		return;
	}
	/* Now actually fetch its weights. */
	weights = loadMeteringWeightsForMode(sensorModel, first);
	if (weights.empty()) {
		LOG(RPI, Warning) << "RawStats: empty weights for " << first
				  << " in " << sensorModel << " tuning — uniform";
		return;
	}
	unsigned int n = weights.size();
	unsigned int side = static_cast<unsigned int>(std::sqrt(n));
	if (side * side != n) {
		LOG(RPI, Warning) << "RawStats: non-square metering grid (" << n
				  << ") — uniform";
		return;
	}
	meteringWeights_ = std::move(weights);
	meteringGridW_ = meteringGridH_ = side;
	meteringModeName_ = first;
	LOG(RPI, Info) << "RawStats: loaded " << side << "×" << side
		       << " '" << first << "' metering grid for " << sensorModel;
}

void RawStatsProducer::setMeteringMode(const std::string &modeName)
{
	std::lock_guard<std::mutex> lk(meteringMutex_);
	if (modeName == meteringModeName_ || sensorModel_.empty())
		return;
	auto weights = loadMeteringWeightsForMode(sensorModel_, modeName);
	if (weights.empty()) {
		LOG(RPI, Warning) << "RawStats: metering mode '" << modeName
				  << "' not in " << sensorModel_
				  << " tuning — keeping '" << meteringModeName_ << "'";
		return;
	}
	unsigned int n = weights.size();
	unsigned int side = static_cast<unsigned int>(std::sqrt(n));
	if (side * side != n) {
		LOG(RPI, Warning) << "RawStats: non-square metering grid for '"
				  << modeName << "' (" << n << ") — keeping current";
		return;
	}
	LOG(RPI, Info) << "RawStats: metering mode '" << meteringModeName_
		       << "' → '" << modeName << "'";
	meteringWeights_ = std::move(weights);
	meteringGridW_ = meteringGridH_ = side;
	meteringModeName_ = modeName;
	cellWeight_.clear();  /* force re-cache next process() */
}

uint16_t RawStatsProducer::loadBlackLevel(const std::string &sensorModel)
{
	const std::string tuningPath =
		"/usr/local/share/libcamera/ipa/rpi/pisp/" + sensorModel + ".json";
	File f(tuningPath);
	if (!f.open(File::OpenModeFlag::ReadOnly))
		return 0;
	std::unique_ptr<YamlObject> root = YamlParser::parse(f);
	if (!root)
		return 0;
	for (const YamlObject &entry : (*root)["algorithms"].asList()) {
		const YamlObject &bl = entry["rpi.black_level"];
		if (!bl.isDictionary())
			continue;
		uint16_t v = bl["black_level"].get<uint16_t>(0);
		if (v != 0)
			blackLevel_.store(v, std::memory_order_relaxed);
		return v;
	}
	return 0;
}

void RawStatsProducer::setBlackLevel(uint16_t blc)
{
	blackLevel_.store(blc, std::memory_order_relaxed);
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

void RawStatsProducer::setLinearizationLut(const uint16_t *lut)
{
	linearizationLut_.store(lut, std::memory_order_release);
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

	/*
	 * Briefly snapshot the metering state under meteringMutex_ — if a
	 * setMeteringMode() call has just come in from the IPA, this is where
	 * we pick up the new weights / cellWeight invalidation. We hold the
	 * lock only for the cellWeight_ rebuild; the inner loop reads
	 * cellWeight_ without locking because the build is complete by then
	 * and setMeteringMode() won't modify it while we still own the lock.
	 */
	{
	std::lock_guard<std::mutex> lk(meteringMutex_);
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
	} /* meteringMutex_ scope */

	uint64_t totalY = 0;
	auto t0 = std::chrono::steady_clock::now();

	/*
	 * Post-WB BT.601 Y coefficients baked into Q10 WB gains, matching the
	 * HW NQ rgby block (pisp.cpp:1022). Y = R·gR·0.299 + G·gG·0.587 +
	 * B·gB·0.114; coefficients sum to 1.0 so a white pixel at unity gain
	 * gives Y = v with no extra /4 normalization.
	 */
	const uint32_t gR_q = static_cast<uint32_t>(
		std::clamp(wbGainR_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.299f * 1024.0f);
	const uint32_t gG_q = static_cast<uint32_t>(
		std::clamp(wbGainG_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.587f * 1024.0f);
	const uint32_t gB_q = static_cast<uint32_t>(
		std::clamp(wbGainB_.load(std::memory_order_relaxed), 0.25f, 8.0f) * 0.114f * 1024.0f);

	/*
	 * BLC: live value plumbed from the V4L2 sensor subdev (see pisp.cpp
	 * cfeBufferDequeue → setBlackLevel). 16-bit-shifted pixel-units so it
	 * lines up with libcamera's left-justified u16 CFE samples regardless
	 * of native bit depth.
	 */
	const uint32_t BLC = blackLevel_.load(std::memory_order_relaxed);

	auto signalBlk = [BLC](uint32_t bsum, unsigned int n) -> uint32_t {
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

		/*
		 * Optional CCMP-inverse LUT. When set, each compressed u16
		 * pixel is replaced with its linearised u16 before any further
		 * accumulation — the BLC subtraction, BT.601 Y dot-product,
		 * and zone summation all run on the linear-domain values.
		 * Lookup is scalar (no NEON gather for 4096-entry tables); LUT
		 * fits in L1.
		 */
		const uint16_t *lut = linearizationLut_.load(std::memory_order_acquire);

		for (unsigned int p = 0; p < cellPairs; p++) {
			const unsigned int x0 = p * 8;

			uint16x8_t v0 = vld1q_u16(row0 + x0);  /* R G R G R G R G */
			uint16x8_t v1 = vld1q_u16(row1 + x0);  /* G B G B G B G B */

			if (lut) {
				uint16_t a0[8], a1[8];
				vst1q_u16(a0, v0);
				vst1q_u16(a1, v1);
				for (int k = 0; k < 8; k++) {
					a0[k] = lut[a0[k] >> 4];
					a1[k] = lut[a1[k] >> 4];
				}
				v0 = vld1q_u16(a0);
				v1 = vld1q_u16(a1);
			}

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
			uint32_t gSig = signalBlk(g_sum_8, 8) >> 1;  /* halve to
								      * match HW NQ
								      * G-per-cell */
			uint32_t bSig = signalBlk(b_sum_4, 4);

			/* Each cell-pair (4 RGGB cells = 8 source columns)
			 * belongs to one AWB zone. x0 = p*8 is already the
			 * pixel-column index of the cell-pair start. */
			unsigned int cellX = std::min<unsigned int>(
				x0 / awbCellW, PISP_AWB_STATS_SIZE - 1);
			flushAndAdvance(cellX);

			accR += rSig;
			accG += gSig;
			accB += bSig;
			accCount += 4;

			/* AGC histogram — one BT.601 Y per cell-pair (= 4 RGGB
			 * cells). rSig/gSig/bSig are per-pixel × 4 (block sum
			 * of 4 cells); shift >> 12 (= /(4·1024)) gives per-pixel
			 * Y at 16-bit scale. */
			uint64_t y_q = uint64_t(rSig) * gR_q
				     + uint64_t(gSig) * gG_q
				     + uint64_t(bSig) * gB_q;
			uint32_t y = static_cast<uint32_t>(y_q >> 12);
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
			totalY += y;
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
