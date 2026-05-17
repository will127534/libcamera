/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * QBC (Quad-Bayer Coding) SW remosaic + stats producer for the PiSP pipeline
 * handler. Sits between the CFE buffer dequeue and the IPA stats consumer:
 *
 *   CFE writes raw 4×4-macro QBC buffer
 *        │
 *        ▼
 *   QbcRemosaic::process(rawBuffer, statsBuffer)
 *     - permutes RRGG → RGGB Bayer in-place so the BE can demosaic
 *     - computes pisp_statistics (AWB zones, AGC histogram, AGC Y_sum)
 *       from the raw QBC values, applying the same per-region metering
 *       weights HW NQ uses (loaded from the sensor tuning JSON)
 *        │
 *        ▼
 *   BE consumes the remosaic'd buffer; IPA consumes the SW-produced stats.
 *
 * Everything is sensor-format-agnostic except for the 4×4 QBC layout and
 * pisp's 32×32 AWB-zone / 15×15 metering grid. The class is owned 1:1 by
 * a PiSPCameraData when its sensor advertises QBC mode (see kQbcCid).
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <linux/v4l2-controls.h>

#include <libpisp/frontend/pisp_statistics.h>

#include <libcamera/controls.h>

namespace libcamera {

class QbcRemosaic
{
public:
	/*
	 * Vendor V4L2 control on the IMX294/IMX492 subdev advertising whether
	 * the current sensor mode emits a 4×4 QBC macro CFA. Read-only volatile.
	 * UAPI allocation pending — sits in the IMX user-CID block.
	 */
	static constexpr uint32_t kQbcCid = V4L2_CID_USER_BASE + 0x10b8;

	QbcRemosaic();

	/*
	 * Initial weight load. Picks the tuning JSON's first-listed metering
	 * mode (matches the AGC IPA's defaultMeteringMode behaviour). Called
	 * at pipeline-configure time, after QBC mode is detected.
	 */
	void loadMeteringWeights(const std::string &sensorModel);

	/*
	 * Switch metering-mode weight grid. Pipeline handler should call this
	 * from onMetadataReady when controls::AeMeteringMode changes, so the
	 * SW kernel's AGC histogram weights track the IPA's chosen mode.
	 */
	void setMeteringMode(const std::string &modeName);

	/*
	 * Read the IPA tuning's rpi.black_level.black_level entry (16-bit-shifted
	 * pixel-units). Returns the loaded value, or 0 if absent. Stored as the
	 * initial BLC; setBlackLevel() can override it per-frame.
	 */
	uint16_t loadBlackLevel(const std::string &sensorModel);

	/*
	 * Live-adjustable BLC (16-bit-shifted pixel-units). The pipeline handler
	 * re-reads the sensor's V4L2 BLC control each frame and pushes the
	 * (scaled) value here so changes propagate within ~1 frame.
	 */
	void setBlackLevel(uint16_t blc);

	/* Slot for the IPA's metadataReady signal — captures ColourGains. */
	void updateWbGains(const ControlList &metadata);

	/*
	 * In-place 4×4 QBC → 2×2 RGGB Bayer remosaic with per-pixel histogram
	 * + AWB-zone stats production. width/height/stride are the CFE output
	 * format in u16 units (stride is bytes). rawPx must be u16 left-
	 * justified 12-bit data (max value 65520); st is the IPA's stats
	 * buffer that this routine fully overwrites.
	 */
	void process(uint16_t *rawPx, pisp_statistics *st,
		     unsigned int width, unsigned int height,
		     unsigned int strideBytes);

private:
	/* AWB gains plumbed from the IPA, one frame behind. */
	std::atomic<float> wbGainR_{ 1.0f };
	std::atomic<float> wbGainG_{ 1.0f };
	std::atomic<float> wbGainB_{ 1.0f };
	/* BLC plumbed from the V4L2 sensor subdev each frame (16-bit-shifted). */
	std::atomic<uint16_t> blackLevel_{ 3200 };

	/*
	 * AGC metering state. meteringMutex_ serialises the (rare) reload
	 * triggered by setMeteringMode() against the per-frame reader in
	 * process(): process() takes it briefly at the top to rebuild the
	 * per-macro cache; the inner loop then reads macroWeight_ lock-free.
	 */
	std::mutex meteringMutex_;
	std::string sensorModel_;                 /* remembered for re-loads */
	std::string meteringModeName_;            /* currently active mode */
	std::vector<uint16_t> meteringWeights_;   /* row-major NxN grid */
	unsigned int meteringGridW_ = 0;
	unsigned int meteringGridH_ = 0;
	std::vector<uint8_t> macroWeight_;        /* per-macro weight cache */

	/*
	 * Per-thread rolling source-row scratch for the bilinear R/B post-pass.
	 *
	 * The QBC sampling structure puts all 4 R sub-pixels of each macro in
	 * cols 0-1 of the macro footprint, and all 4 B sub-pixels in cols 2-3.
	 * The LUT pass writes those source values into the output Bayer's R/B
	 * positions with a 1- to 2-pixel spatial offset, creating a 4-pixel-
	 * period bias visible as macro-block 'stipple' at 800%+ zoom on
	 * smooth regions. The bilinear post-pass re-reads same-colour
	 * sources and writes proper interpolated outputs to fix this.
	 *
	 * The post-pass needs the un-modified source values, but the LUT
	 * pass writes in-place — so we keep a small ring of the last few
	 * macro rows of source data per thread (kRingMacroRows × 4 lines ×
	 * stride × kNumThreads), plus 2 boundary rows at the strip seam so
	 * the per-thread halves of the bilinear pass produce bit-exact
	 * output across the seam. The buffer is reused frame-to-frame and
	 * only resized when geometry changes — ~400 KB total at full res
	 * (3 macro-rows × 4 lines × stride per thread × 2 threads, plus the
	 * 2 boundary rows) vs ~95 MB if we copied the whole frame.
	 */
	std::vector<uint16_t> sourceBuffer_;
};

} /* namespace libcamera */
