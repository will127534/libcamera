/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Tuning-JSON helpers shared between the SW stats producers (QbcRemosaic,
 * RawStatsProducer). Both consume the same /usr/local/share/libcamera/ipa/
 * rpi/pisp/<sensor>.json file the IPA reads, so the parsing logic for the
 * AGC metering grids and BLC pedestal lives here in one place.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <libpisp/frontend/pisp_statistics.h>

namespace libcamera {

/*
 * AGC histogram bin width in u16-Y units. The IPA's 1024-bin histogram
 * spans the full 16-bit Y range, so each bin covers (1 << 16) /
 * PISP_AGC_STATS_NUM_BINS = 64 codes — i.e. `bin = y >> 6`.
 */
constexpr unsigned int PISP_AGC_HIST_BIN_SHIFT = 6;
static_assert((1u << (16 - PISP_AGC_HIST_BIN_SHIFT)) == PISP_AGC_STATS_NUM_BINS,
	      "AGC histogram bin shift must match PISP_AGC_STATS_NUM_BINS for full 16-bit Y range");

/*
 * Pull weights[] for `modeName` from the tuning JSON. Supports both the
 * imx294-style flat rpi.agc.metering_modes layout and the imx585-style
 * channels[0].metering_modes (multi-channel AGC for HDR). When firstName
 * is non-null and currently empty, it is set to the first-listed mode name
 * (matches the IPA's defaultMeteringMode behaviour). Empty result on any
 * failure (file missing, no rpi.agc, mode not found, etc.).
 */
std::vector<uint16_t> loadMeteringWeightsForMode(const std::string &sensorModel,
						 const std::string &modeName,
						 std::string *firstName = nullptr);

/*
 * Read rpi.black_level.black_level (16-bit-shifted) from the tuning JSON.
 * Returns 0 if absent / unreadable.
 */
uint16_t loadBlackLevelFromTuning(const std::string &sensorModel);

} /* namespace libcamera */
