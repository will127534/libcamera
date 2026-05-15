/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Tuning-JSON helpers shared between the SW stats producers.
 */

#include "tuning_helpers.h"

#include <memory>

#include <libcamera/base/file.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

namespace {

std::string tuningPathFor(const std::string &sensorModel)
{
	return "/usr/local/share/libcamera/ipa/rpi/pisp/" + sensorModel + ".json";
}

} /* anonymous namespace */

std::vector<uint16_t> loadMeteringWeightsForMode(const std::string &sensorModel,
						 const std::string &modeName,
						 std::string *firstName)
{
	std::vector<uint16_t> out;
	File f(tuningPathFor(sensorModel));
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

uint16_t loadBlackLevelFromTuning(const std::string &sensorModel)
{
	File f(tuningPathFor(sensorModel));
	if (!f.open(File::OpenModeFlag::ReadOnly))
		return 0;
	std::unique_ptr<YamlObject> root = YamlParser::parse(f);
	if (!root)
		return 0;
	for (const YamlObject &entry : (*root)["algorithms"].asList()) {
		const YamlObject &bl = entry["rpi.black_level"];
		if (!bl.isDictionary())
			continue;
		return bl["black_level"].get<uint16_t>(0);
	}
	return 0;
}

} /* namespace libcamera */
