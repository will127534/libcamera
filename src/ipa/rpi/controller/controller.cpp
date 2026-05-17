/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ISP controller
 */

#include <assert.h>
#include <byteswap.h>
#include <cstdint>
#include <cstdio>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "algorithm.h"
#include "controller.h"

using namespace RPiController;
using namespace libcamera;
using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(RPiController)

namespace {

/*
 * Compute the effective RP1_CLK_SYS rate so the PiSP CSI2→FE pixel-rate
 * cap scales with whatever rp1 overclock is in effect on this board.
 * The CSI2-to-ISP-FE bottleneck runs at 2 pixels per RP1_CLK_SYS cycle,
 * minus a ~20 Mpix/s per-scanline overhead — i.e. at stock 200 MHz the
 * cap is 200 × 2 − 20 = 380 Mpix/s (matching the historical hardcoded
 * value), at 333.3 MHz it becomes 333.3 × 2 − 20 ≈ 647 Mpix/s. If we
 * leave the bound at the stock-clock value an overclocked board would
 * over-pad the sensor line length and lose framerate it could otherwise
 * hit.
 *
 * Approach: read the *requested* RP1_PLL_SYS rate from the live device
 * tree at /sys/firmware/devicetree/base/axi/pcie@…/rp1/clocks@18000/
 * assigned-clock-rates (entry 2, per the rp1-clocks binding), then snap
 * it to the nearest rate the RP1 PLL can actually produce:
 *
 *     pll_sys = pll_sys_core / N   for integer N ≥ 1
 *     pll_sys_core = 1 GHz (fixed RP1 silicon parameter, also entry 0
 *                           of the same array)
 *
 * The Linux clock framework applies this same snap in-kernel when it
 * programs the PLL; we just reproduce the math in userspace so we don't
 * have to read debugfs (which is root-only on default Pi OS). On the
 * Pi 5 RP1 (kernel 6.12 bindings) clk_sys is derived from pll_sys with
 * a divider of 1, so snapped pll_sys = actual clk_sys.
 *
 * Returns Mpix/s. Cached via function-static so the lookup happens once
 * per process. Falls back to the stock 380 Mpix/s on any failure
 * (non-Pi5 host, file missing/short, implausible rate).
 */
constexpr double kStockMpixPerS = 380.0;
constexpr double kStockRp1ClkSysHz = 200e6;
constexpr double kRp1PllSysCoreHz = 1e9;         /* RP1 PLL_SYS_CORE is fixed at 1 GHz */
constexpr double kPixelsPerClock = 2.0;          /* CSI2→FE throughput per RP1_CLK_SYS tick */
constexpr double kPerScanlineMpixPenalty = 20.0; /* Per-scanline overhead, in Mpix/s */
constexpr unsigned int kIdxRp1PllSysCore = 0;    /* RP1_PLL_SYS_CORE in assigned-clock-rates */
constexpr unsigned int kIdxRp1PllSys = 2;        /* RP1_PLL_SYS     in assigned-clock-rates */
constexpr const char *kRp1ClocksAssignedPath =
	"/sys/firmware/devicetree/base/axi/pcie@1000120000/rp1/"
	"clocks@18000/assigned-clock-rates";

/* Read all entries of the rp1_clocks assigned-clock-rates array.
 * Returns the number of u32 entries successfully read. */
unsigned int readRp1AssignedClockRates(uint32_t out[16])
{
	FILE *f = std::fopen(kRp1ClocksAssignedPath, "rb");
	if (!f)
		return 0;
	size_t n = std::fread(out, sizeof(uint32_t), 16, f);
	std::fclose(f);
	return static_cast<unsigned int>(n);
}

/* Snap a requested rate to the nearest pll_sys_core / N rate the RP1
 * PLL_SYS can actually produce. Mirrors the in-kernel snap the clock
 * framework applies — letting us predict the achieved rate from
 * userspace without reading debugfs. */
double snapPllSysRate(double requestedHz, double pllSysCoreHz)
{
	if (requestedHz <= 0 || pllSysCoreHz <= 0)
		return 0;
	double n_real = pllSysCoreHz / requestedHz;
	/* Cap N at UINT_MAX before the float→int cast: very tiny requestedHz
	 * could push n_real past UINT_MAX, which is UB on the cast. The
	 * post-snap plausibility clamp [0.5x, 4x stock] will reject the
	 * result anyway, but keep the snap itself well-defined. */
	if (n_real > 4294967295.0)
		n_real = 4294967295.0;
	unsigned int n = static_cast<unsigned int>(n_real + 0.5); /* round */
	if (n == 0)
		n = 1;
	return pllSysCoreHz / static_cast<double>(n);
}

double detectPispPixelRateMpixPerS()
{
	uint32_t rates[16] = {};
	unsigned int n_read = readRp1AssignedClockRates(rates);
	if (n_read <= kIdxRp1PllSys) {
		LOG(RPiController, Debug)
			<< "RP1 assigned-clock-rates not found, using stock "
			<< kStockMpixPerS << " Mpix/s cap";
		return kStockMpixPerS;
	}

	/* Device-tree integers are big-endian. */
	uint64_t pllSysReq = bswap_32(rates[kIdxRp1PllSys]);
	uint64_t pllSysCoreReq = (n_read > kIdxRp1PllSysCore)
		? bswap_32(rates[kIdxRp1PllSysCore]) : 0;

	/* Use the live PLL_SYS_CORE request when present; this lets the math
	 * track an unusual configuration that overclocks the core too. Fall
	 * back to the hardcoded 1 GHz when absent (matches the stock silicon). */
	double coreHz = pllSysCoreReq ? static_cast<double>(pllSysCoreReq)
				      : kRp1PllSysCoreHz;
	double snappedHz = snapPllSysRate(static_cast<double>(pllSysReq), coreHz);

	/* Sanity-clamp to keep a wild config from disabling the cap. */
	double scale = snappedHz / kStockRp1ClkSysHz;
	if (snappedHz <= 0 || scale < 0.5 || scale > 4.0) {
		LOG(RPiController, Warning)
			<< "RP1_PLL_SYS request=" << pllSysReq << " Hz snaps to "
			<< snappedHz << " Hz (core=" << coreHz << " Hz), outside "
			<< "plausible range; using stock " << kStockMpixPerS
			<< " Mpix/s cap";
		return kStockMpixPerS;
	}

	/* pixel_rate = clk × 2 pixels/clock − 20 Mpix/s scanline overhead. */
	double mpix = snappedHz * kPixelsPerClock / 1e6 - kPerScanlineMpixPenalty;
	LOG(RPiController, Info)
		<< "RP1_PLL_SYS request=" << pllSysReq
		<< " Hz, snapped to pll_sys_core/" << static_cast<unsigned int>(coreHz / snappedHz + 0.5)
		<< " = " << static_cast<uint64_t>(snappedHz) << " Hz; "
		<< "PiSP pixel-rate cap = " << mpix << " Mpix/s ("
		<< kPixelsPerClock << "px/clk − " << kPerScanlineMpixPenalty
		<< " overhead)";
	return mpix;
}

double pispPixelRateMpixPerS()
{
	static const double v = detectPispPixelRateMpixPerS();
	return v;
}

const std::map<std::string, Controller::HardwareConfig> &hardwareConfigMap()
{
	static const std::map<std::string, Controller::HardwareConfig> map = {
		{
			"bcm2835",
			{
				/*
				* There are only ever 15 AGC regions computed by the firmware
				* due to zoning, but the HW defines AGC_REGIONS == 16!
				*/
				.agcRegions = { 15 , 1 },
				.agcZoneWeights = { 15 , 1 },
				.awbRegions = { 16, 12 },
				.cacRegions = { 0, 0 },
				.focusRegions = { 4, 3 },
				.numHistogramBins = 128,
				.numGammaPoints = 33,
				.pipelineWidth = 13,
				.statsInline = false,
				.minPixelProcessingTime = 0s,
				.dataBufferStrided = true,
			}
		},
		{
			"pisp",
			{
				.agcRegions = { 0, 0 },
				.agcZoneWeights = { 15, 15 },
				.awbRegions = { 32, 32 },
				.cacRegions = { 8, 8 },
				.focusRegions = { 8, 8 },
				.numHistogramBins = 1024,
				.numGammaPoints = 64,
				.pipelineWidth = 16,
				.statsInline = true,

				/*
				* The constraint below is on the rate of pixels going
				* from CSI2 peripheral to ISP-FE (400Mpix/s, plus tiny
				* overheads per scanline, for which 380Mpix/s is a
				* conservative bound at the stock RP1 clock of 200 MHz).
				*
				* The bottleneck rises linearly with RP1_CLK_SYS, so we
				* derive the effective cap from the live device-tree
				* clock at first call. A stock board reads 200 MHz and
				* keeps the 380 Mpix/s bound; the rp1-300mhz overlay
				* (or any other rp1 overclock) is picked up automatically
				* — no rebuild needed when the user re-tunes the clock.
				* See detectPispPixelRateMpixPerS() above.
				*
				* There is a 64kbit data FIFO before the bottleneck,
				* which means that in all reasonable cases the
				* constraint applies at a timescale >= 1 scanline, so
				* adding horizontal blanking can prevent loss.
				*
				* If the backlog were to grow beyond 64kbit during a
				* single scanline, there could still be loss. This
				* could happen using 4 lanes at 1.5Gbps at 10bpp with
				* frames wider than ~16,000 pixels.
				*/
				.minPixelProcessingTime = 1.0us / pispPixelRateMpixPerS(),
				.dataBufferStrided = false,
			}
		},
	};

	return map;
}

} /* namespace */

Controller::Controller()
	: switchModeCalled_(false)
{
}

Controller::~Controller() {}

int Controller::read(char const *filename)
{
	File file(filename);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(RPiController, Warning)
			<< "Failed to open tuning file '" << filename << "'";
		return -EINVAL;
	}

	std::unique_ptr<ValueNode> root = YamlParser::parse(file);
	if (!root)
		return -EINVAL;

	double version = (*root)["version"].get<double>(1.0);
	target_ = (*root)["target"].get<std::string>("bcm2835");

	if (version < 2.0) {
		LOG(RPiController, Warning)
			<< "This format of the tuning file will be deprecated soon!"
			<< " Please use the convert_tuning.py utility to update to version 2.0.";

		for (const auto &[key, value] : root->asDict()) {
			int ret = createAlgorithm(key, value);
			if (ret)
				return ret;
		}
	} else if (version < 3.0) {
		if (!root->contains("algorithms")) {
			LOG(RPiController, Error)
				<< "Tuning file " << filename
				<< " does not have an \"algorithms\" list!";
			return -EINVAL;
		}

		for (const auto &rootAlgo : (*root)["algorithms"].asList())
			for (const auto &[key, value] : rootAlgo.asDict()) {
				int ret = createAlgorithm(key, value);
				if (ret)
					return ret;
			}
	} else {
		LOG(RPiController, Error)
			<< "Unrecognised version " << version
			<< " for the tuning file " << filename;
		return -EINVAL;
	}

	return 0;
}

int Controller::createAlgorithm(const std::string &name, const ValueNode &params)
{
	/* Any algorithm may be disabled by setting "enabled" to false. */
	bool enabled = params["enabled"].get<bool>(true);
	LOG(RPiController, Debug)
		<< "Algorithm " << name << ": "
		<< (enabled ? "enabled" : "disabled");
	if (!enabled)
		return 0;

	auto it = getAlgorithms().find(name);
	if (it == getAlgorithms().end()) {
		LOG(RPiController, Warning)
			<< "No algorithm found for \"" << name << "\"";
		return 0;
	}

	/* Do not allow duplicate versions of algorithms (e.g. AWB) to run. */
	size_t pos = name.find_last_of('.');
	std::string const &algoType =
		pos == std::string::npos ? name : name.substr(pos + 1);
	if (getAlgorithm(algoType)) {
		LOG(RPiController, Error)
			<< "Algorithm type '" << algoType << "' already exists";
		return -1;
	}

	Algorithm *algo = (*it->second)(this);
	int ret = algo->read(params);
	if (ret)
		return ret;

	algorithms_.push_back(AlgorithmPtr(algo));
	return 0;
}

void Controller::initialise()
{
	for (auto &algo : algorithms_)
		algo->initialise();
}

void Controller::switchMode(CameraMode const &cameraMode, Metadata *metadata)
{
	for (auto &algo : algorithms_)
		algo->switchMode(cameraMode, metadata);
	switchModeCalled_ = true;
}

void Controller::prepare(Metadata *imageMetadata)
{
	assert(switchModeCalled_);
	for (auto &algo : algorithms_)
		algo->prepare(imageMetadata);
}

void Controller::process(StatisticsPtr stats, Metadata *imageMetadata)
{
	assert(switchModeCalled_);
	for (auto &algo : algorithms_)
		algo->process(stats, imageMetadata);
}

Metadata &Controller::getGlobalMetadata()
{
	return globalMetadata_;
}

Algorithm *Controller::getAlgorithm(std::string const &name) const
{
	/*
	 * The passed name must be the entire algorithm name, or must match the
	 * last part of it with a period (.) just before.
	 */
	size_t nameLen = name.length();
	for (auto &algo : algorithms_) {
		char const *algoName = algo->name();
		size_t algoNameLen = strlen(algoName);
		if (algoNameLen >= nameLen &&
		    strcasecmp(name.c_str(),
			       algoName + algoNameLen - nameLen) == 0 &&
		    (nameLen == algoNameLen ||
		     algoName[algoNameLen - nameLen - 1] == '.'))
			return algo.get();
	}
	return nullptr;
}

const std::string &Controller::getTarget() const
{
	return target_;
}

const Controller::HardwareConfig &Controller::getHardwareConfig() const
{
	auto cfg = hardwareConfigMap().find(getTarget());

	/*
	 * This really should not happen, the IPA ought to validate the target
	 * on initialisation.
	 */
	ASSERT(cfg != hardwareConfigMap().end());
	return cfg->second;
}
