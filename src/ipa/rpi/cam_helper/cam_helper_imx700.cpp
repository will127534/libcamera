/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * camera helper for the Sony IMX700 (RYYB) sensor
 *
 * Classic Sony SMIA++ gain/exposure model (same as imx477):
 *   analogue gain = 1024 / (1024 - code)
 *   exposure and frame/line length in line units via 0x0202 / 0x0340 / 0x0342.
 *
 * NATIVE PHASE-DETECT AF
 * ----------------------
 * The patched 2-pad imx700 driver emits a native PD line on the sensor's
 * embedded/metadata pad (CSI DT 0x30). Each frame the embedded buffer is
 * 16384 bytes and carries a grid of PD zones; every zone reports a confidence
 * byte and a signed int8 PHASE. The PD engine's grid is 8 columns wide; the
 * number of rows depends on the sensor readout (8x6 = 48 zones full-FOV, fewer
 * rows for the vertically-cropped modes). d2 (signed) is NEGATIVE below best
 * focus, crosses 0 at best focus (VCM ~390) and is POSITIVE beyond it; slope
 * ~ +0.13..0.15 phase/VCM-code (measured on the bench).
 *
 * The line is packed two different ways depending on the readout:
 *   - full-FOV (e.g. 4096x3072): 5 bytes/zone  [d0 d1 d2 d3 0x55]
 *   - cropped/binned preview   : 6 bytes/zone  [d0 d1 0x55 d2 d3 0x55]
 * Both start after a 2-zone header; a zone is "filler" (outside the PD area)
 * when its first two payload bytes are both 0x55, and "invalid" (no phase) when
 * its phase byte == 0x80 (-128). We auto-detect the packing from the 0x55
 * markers and fall back to no-PD (rpi.af then uses CDAF) if neither matches.
 *
 * We fill the standard rpi.af "pdaf.regions" metadata (mirroring imx708) so the
 * existing rpi.af PDAF closed loop drives the lens. Unlike imx708 our embedded
 * stream is PURE PD (not SMIA registers), so we must NOT call parseEmbeddedData
 * and must NOT touch device.status: exposure/gain come from DelayedControls via
 * fillDeviceStatus() before prepare() and are left intact.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

#include <libcamera/base/log.h>

#include "controller/pdaf_data.h"

#include "cam_helper.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

class CamHelperImx700 : public CamHelper
{
public:
	CamHelperImx700();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	bool sensorEmbeddedDataPresent() const override;
	unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines. Matches IMX700_EXPOSURE_MARGIN in the kernel driver.
	 */
	static constexpr int frameIntegrationDiff = 22;

	/* Native PD line geometry (DT 0x30). */
	static constexpr unsigned int pdHeaderZones = 2;	/* leading non-grid zones */
	static constexpr unsigned int pdGridCols = 8;		/* PD grid is 8 columns wide */
	static constexpr unsigned int pdGridMaxRows = 6;	/* 8 x 6 = 48 zones full-FOV */
	static constexpr uint8_t pdInvalid = 0x80;		/* phase == -128 => no phase */
	static constexpr uint8_t pdMarker = 0x55;
	/*
	 * Scale applied to the signed int8 phase before storing it in the S.11.4
	 * PdafData.phase field. Chosen (with the tuning's pdaf_gain and dioptre->VCM
	 * map slope) so the rpi.af loop converges without overshoot.
	 */
	static constexpr int pdPhaseScale = 8;

	/* Byte layout of one PD zone, auto-detected per readout mode. */
	struct PdLayout {
		unsigned int stride;	/* bytes per zone */
		unsigned int confOff;	/* confidence byte */
		unsigned int phaseOff;	/* signed phase byte (== pdInvalid when no phase) */
		unsigned int fillerA;	/* filler when both these bytes are 0x55 */
		unsigned int fillerB;
		unsigned int markerA;	/* 0x55 marker bytes used for detection */
		unsigned int markerB;
	};

	static bool detectLayout(libcamera::Span<const uint8_t> buffer, PdLayout &layout);
	static bool parsePdafData(libcamera::Span<const uint8_t> buffer, PdafRegions &pdaf);
};

CamHelperImx700::CamHelperImx700()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx700::gainCode(double gain) const
{
	uint32_t code = static_cast<uint32_t>(1024 - 1024 / gain);
	return std::min(code, 978u);	/* driver caps analogue gain code at 978 */
}

double CamHelperImx700::gain(uint32_t gainCode) const
{
	return 1024.0 / (1024 - gainCode);
}

bool CamHelperImx700::sensorEmbeddedDataPresent() const
{
	/*
	 * Returning true makes the pipeline set up the embedded (metadata) node
	 * and hand its buffer to prepare(), where we turn the native PD line into
	 * pdaf.regions for rpi.af.
	 */
	return true;
}

void CamHelperImx700::prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	/*
	 * IMPORTANT: our embedded stream carries the PD line, NOT SMIA register
	 * data, so we deliberately do NOT call parseEmbeddedData() and do NOT
	 * write device.status. Exposure/gain were already filled from the applied
	 * controls (fillDeviceStatus) before we were called, and must stay intact.
	 */
	PdafRegions pdaf;
	if (parsePdafData(buffer, pdaf))
		metadata.set("pdaf.regions", pdaf);
}

/*
 * Work out how the PD zones are packed in this readout mode. The two known
 * packings differ in stride and in where the 0x55 markers fall; we score both
 * over the first (header + a full 8x6 grid) worth of zones and pick the winner.
 */
bool CamHelperImx700::detectLayout(libcamera::Span<const uint8_t> buffer, PdLayout &layout)
{
	static const PdLayout candidates[] = {
		/* full-FOV: [d0 d1 d2 d3 0x55] */
		{ 5, 0, 2, 1, 3, 4, 4 },
		/* cropped/binned: [d0 d1 0x55 d2 d3 0x55] */
		{ 6, 1, 3, 0, 1, 2, 5 },
	};
	const uint8_t *data = buffer.data();
	size_t len = buffer.size();
	unsigned int probe = pdHeaderZones + pdGridCols * pdGridMaxRows; /* 2 + 48 */

	for (const PdLayout &c : candidates) {
		if (len < probe * c.stride)
			continue;
		unsigned int ok = 0;
		for (unsigned int z = pdHeaderZones; z < probe; ++z) {
			const uint8_t *p = data + z * c.stride;
			if (p[c.markerA] == pdMarker && p[c.markerB] == pdMarker)
				ok++;
		}
		if (ok >= 44) {	/* out of 48 */
			layout = c;
			return true;
		}
	}
	return false;
}

bool CamHelperImx700::parsePdafData(libcamera::Span<const uint8_t> buffer, PdafRegions &pdaf)
{
	const uint8_t *data = buffer.data();
	size_t len = buffer.size();

	PdLayout lay;
	if (!detectLayout(buffer, lay)) {
		LOG(IPARPI, Warning) << "IMX700 PD: unrecognised layout, bufsize="
				     << len << " (rpi.af will fall back to CDAF)";
		return false;
	}

	/*
	 * The grid is 8 columns wide; the number of live (in-area) rows depends
	 * on the readout crop. A row is outside the PD area only when EVERY
	 * zone in it carries the 0x55/0x55 filler signature: testing single
	 * zones is not safe, because the "filler" bytes are live payload bytes
	 * (d1/d3 in the 5-byte packing, d0/d1 in the 6-byte packing) in an
	 * in-area zone and can coincidentally read 0x55/0x55 - which used to
	 * truncate the grid at that zone, or disable PDAF outright when it
	 * happened in the first row. Count contiguous live rows from the top
	 * and stop at the first all-filler row. A zone-level coincidence
	 * inside a live row costs at most that one zone (the per-zone filler
	 * test below marks it invalid), never the rest of the grid.
	 * detectLayout() already guaranteed the buffer covers the header plus
	 * a full 8x6 grid at this stride.
	 */
	unsigned int rows = 0;
	for (unsigned int row = 0; row < pdGridMaxRows; ++row) {
		unsigned int fillerCount = 0;
		for (unsigned int col = 0; col < pdGridCols; ++col) {
			unsigned int z = pdHeaderZones + row * pdGridCols + col;
			const uint8_t *p = data + z * lay.stride;
			if (p[lay.fillerA] == pdMarker && p[lay.fillerB] == pdMarker)
				fillerCount++;
		}
		if (fillerCount == pdGridCols)
			break;
		rows++;
	}
	if (rows == 0) {
		LOG(IPARPI, Warning) << "IMX700 PD: no in-area zones (bufsize=" << len << ")";
		return false;
	}

	pdaf.init({ pdGridCols, rows });

	long sumConf = 0, sumConfPhase = 0;
	unsigned int nValid = 0;

	for (unsigned int row = 0; row < rows; ++row) {
		for (unsigned int col = 0; col < pdGridCols; ++col) {
			unsigned int z = pdHeaderZones + row * pdGridCols + col;
			const uint8_t *p = data + z * lay.stride;
			uint8_t phaseByte = p[lay.phaseOff];
			bool filler = (p[lay.fillerA] == pdMarker && p[lay.fillerB] == pdMarker);
			bool invalid = filler || (phaseByte == pdInvalid);

			PdafData pd;
			if (invalid) {
				pd.conf = 0;
				pd.phase = 0;
			} else {
				int phase = static_cast<int8_t>(phaseByte);
				pd.conf = p[lay.confOff];
				pd.phase = static_cast<int16_t>(phase * pdPhaseScale);
				sumConf += pd.conf;
				sumConfPhase += static_cast<long>(pd.conf) * phase;
				nValid++;
			}
			pdaf.set(libcamera::Point(col, row), { pd, 1, 0 });
		}
	}

	double aggPhase = sumConf ? static_cast<double>(sumConfPhase) / sumConf : 0.0;
	LOG(IPARPI, Debug) << "IMX700 PD: bufsize=" << len << " stride=" << lay.stride
			   << " grid=8x" << rows << " valid=" << nValid << '/' << (pdGridCols * rows)
			   << " aggPhase(d2)=" << aggPhase;

	return nValid > 0;
}

unsigned int CamHelperImx700::hideFramesStartup() const
{
	/* The first frame after stream-on is unreliable. */
	return 1;
}

unsigned int CamHelperImx700::hideFramesModeSwitch() const
{
	return 1;
}

static CamHelper *create()
{
	return new CamHelperImx700();
}

static RegisterCamHelper reg("imx700", &create);
