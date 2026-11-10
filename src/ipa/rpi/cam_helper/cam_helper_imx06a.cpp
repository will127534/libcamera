/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * camera helper for the Sony IMX06A sensor
 *
 * CCS/SMIA++-style register model:
 *   analogue gain = 16384 / (16384 - ANA_GAIN_GLOBAL)   (code multiple of 4;
 *     0..15360 = 0..24 dB on the full-resolution modes, 0..16128 = 0..36 dB
 *     on the 4x2 binned mode - the driver's control range enforces both)
 *   exposure = COARSE_INTEG_TIME lines, max FRM_LENGTH_LINES - 64 (CIT_MARGIN)
 *
 * Long exposure: the driver accepts VBLANK/EXPOSURE beyond the 16-bit
 * registers and programs them as value >> n with CIT_LSHIFT = n (n <= 9), so
 * beyond 65532 lines exposure is quantised to 2^n lines; getBlanking() mirrors
 * that (same scheme as imx477).
 *
 * HDR (driver "HDR Mode" control, set before the camera is opened): both
 * families are single-frame, sensor-fused outputs. DCG-HDR's analogue gain is
 * exposed by the driver in low-side-gain units, so its sensitivity equals the
 * normal modes. QBC-HDR keeps the long exposure in the bottom quarter of its
 * 12-bit range, so AGC simply exposes ~4x longer; its reported Lux reads 4x low
 * (the mode cannot be told apart from DCG-HDR 4:1 by its CameraMode).
 *
 * Line timing: the driver reports PIXEL_RATE = INCK x 64 with an extensible
 * HBLANK (one step = one INCK cycle), so the generic line-length handling -
 * including the PiSP pixel-rate limit - applies unchanged.
 *
 * Phase detect (PDAF): every pixel is a dual photodiode, but the sensor does no
 * phase computation of its own. With the driver's "pdaf" option the raw
 * left/right photodiode values arrive on the sensor's metadata pad (VC0 / DT
 * 0x30, see the driver), which the pipeline captures as "embedded data". There
 * is no register dump in that stream, so exposure/gain metadata still come
 * from DelayedControls; prepare() instead correlates the left and right images
 * per region and publishes the result as "pdaf.regions" for rpi.af, the same
 * metadata the IMX708 produces on chip. See computePdaf().
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string.h>
#include <vector>

#include <libcamera/base/log.h>

#include "controller/pdaf_data.h"

#include "cam_helper.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

class CamHelperImx06a : public CamHelper
{
public:
	CamHelperImx06a();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	std::pair<uint32_t, uint32_t> getBlanking(Duration &exposure, Duration minFrameDuration,
						  Duration maxFrameDuration) const override;
	void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/* PD stream layout of the current mode (see the driver's imx06a_pd_geom()). */
	struct PdGeom {
		unsigned int px;	/* RAW10 values per packet (L R L R ...) */
		unsigned int rows;	/* packets per frame */
		unsigned int sites;	/* L/R pairs per packet */
		double colsPerSite;	/* sensor columns covered by one site */
	};

	bool pdGeometry(PdGeom &g) const;
	bool computePdaf(const std::vector<uint8_t> &buffer, const PdGeom &g, PdafRegions &regions);
	void makePdafImage(const PdGeom &g, const libcamera::Rectangle *window, PdafImage &image) const;

	/* PDAF grid: the same as the IMX708's on-chip grid. */
	static constexpr unsigned int pdafCols = 16;
	static constexpr unsigned int pdafRows = 12;
	/* PD black level (RAW10) at the driver's default black level. */
	static constexpr int pdBlack = 64;

	std::vector<uint8_t> pdRaw_;		/* cached copy of the PD buffer */
	std::vector<int32_t> pdL_, pdR_;	/* vertically binned L/R planes */
	std::vector<int16_t> hpL_, hpR_;	/* locally normalised, Q10 */
	PdafRegions pdLast_;			/* last computed grid */
	std::chrono::steady_clock::time_point pdLastTime_;
	bool pdHaveLast_ = false;
	unsigned int pdLastKey_ = 0;		/* sites x rows of the cached grid's mode */
	std::chrono::steady_clock::time_point pdImageTime_;
	unsigned int pdWarned_ = 0;
	unsigned int pdFrames_ = 0;

	/* CIT_MARGIN (0x68C2), matches IMX06A_EXPOSURE_MARGIN in the driver. */
	static constexpr int frameIntegrationDiff = 64;
	/* Highest analogue gain code the sensor accepts (36 dB). */
	static constexpr uint32_t maxGainCode = 16128;
	/* FRM_LENGTH_LINES register maximum and the largest CIT_LSHIFT. */
	static constexpr uint32_t frameLengthMax = 65532;
	static constexpr unsigned int longExposureShiftMax = 9;
	/* Driver timing base: PIXEL_RATE = INCK x 64, HBLANK step = 1 INCK. */
	static constexpr long pixPerTick = 64;
	/*
	 * VBLANK step: 4 (binned / DCG-HDR), 8 in the 4x4-binned modes. binY is
	 * capped at 2 by the IPA, so look at the real vertical scale.
	 */
	long long vStep() const { return mode_.scaleY > 3.5 ? 8 : 4; }
};

CamHelperImx06a::CamHelperImx06a()
	: CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperImx06a::gainCode(double gain) const
{
	gain = std::max(gain, 1.0);
	uint32_t code = static_cast<uint32_t>(16384.0 - 16384.0 / gain);
	/* The sensor requires a multiple of 4; round down so we never overshoot. */
	code &= ~3u;
	return std::min(code, maxGainCode);
}

double CamHelperImx06a::gain(uint32_t gainCode) const
{
	return 16384.0 / (16384 - std::min(gainCode, maxGainCode));
}

/*
 * Exact pinned frame rates. The driver's line period is LINE_LENGTH_INCK (one
 * HBLANK step = one INCK cycle) and its frame length moves in steps of up to 4
 * lines, so the frame period is (line ticks h) x (lines v) INCK cycles. The
 * default getBlanking() keeps the line at its minimum and only rounds the frame
 * length, so a requested rate lands within ~one line. When a single rate is
 * pinned we search the (v, h) lattice for h * v == the requested period instead
 * (24 MHz / 24, 25, 30, 48, 50, 60, 100, 120 ... are all whole tick counts), and
 * prefer the SHORTEST such line (least rolling-shutter skew). If no exact pair
 * exists (e.g. full resolution at 12 fps: 2,000,000 ticks has no divisor in the
 * legal line range) we keep the pair with the smallest error.
 * Same approach as the imx294/imx492 helpers.
 */
std::pair<uint32_t, uint32_t> CamHelperImx06a::getBlanking(Duration &exposure,
							   Duration minFrameDuration,
							   Duration maxFrameDuration) const
{
	auto [vblank, hblank] = CamHelper::getBlanking(exposure, minFrameDuration,
						       maxFrameDuration);
	uint32_t frameLength = mode_.height + vblank;
	unsigned int shift = 0;

	if (minFrameDuration == maxFrameDuration &&
	    mode_.maxLineLength > mode_.minLineLength && frameLength <= frameLengthMax) {
		/* INCK: the driver reports PIXEL_RATE = INCK x 64. */
		const double clock = mode_.pixelRate / static_cast<double>(pixPerTick);
		auto toTicks = [clock](const Duration &d) {
			return d.get<std::nano>() * clock * 1e-9;
		};
		const long minH = std::lround(std::ceil(toTicks(mode_.minLineLength) - 1e-6));
		const long capH = std::min<long>(std::lround(toTicks(mode_.maxLineLength)),
						 minH + minH / 2);
		long long target = std::llround(toTicks(minFrameDuration));

		/*
		 * The request arrives in integer microseconds (rpicam truncates
		 * 1e6/fps: 24 fps -> 41666 us). If it is within a microsecond of an
		 * integer rate whose period is a whole tick count, aim for that.
		 */
		const long fps = std::lround(1.0e6 / minFrameDuration.get<std::micro>());
		if (fps > 0 && std::fabs(1.0e6 / fps - minFrameDuration.get<std::micro>()) < 1.0 &&
		    std::fmod(clock, static_cast<double>(fps)) == 0.0)
			target = std::llround(clock / fps);

		long exactH = -1, nearH = -1;
		long long exactV = 0, nearV = 0;
		long long nearErr = std::numeric_limits<long long>::max();
		/*
		 * Exact pairs may use a longer line (up to 4x the minimum) when none
		 * exists within capH; the nearest-pair fallback stays within capH.
		 */
		const long exactCapH = std::min<long>(std::lround(toTicks(mode_.maxLineLength)),
						      4 * minH);

		/* h ascends: the first hit is the shortest line of its kind. */
		for (long h = std::max<long>(minH, 1); h <= exactCapH && exactH < 0; h++) {
			long long v0 = (target / h) & ~(vStep() - 1);
			for (long long v : { v0, v0 + vStep() }) {
				if (v < mode_.minFrameLength || v > mode_.maxFrameLength ||
				    v > frameLengthMax)
					continue;
				long long err = std::llabs(v * h - target);
				if (err == 0) {
					exactH = h;
					exactV = v;
					break;
				}
				if (h <= capH && err < nearErr) {
					nearErr = err;
					nearH = h;
					nearV = v;
				}
			}
		}
		long bestH = exactH > 0 ? exactH : nearH;
		long long bestV = exactH > 0 ? exactV : nearV;

		if (bestH > 0) {
			hblank = static_cast<uint32_t>(std::max<long>(0, bestH * pixPerTick -
								     static_cast<long>(mode_.width)));
			vblank = static_cast<uint32_t>(bestV - mode_.height);
			frameLength = static_cast<uint32_t>(bestV);

			Duration lineLength = hblankToLineLength(hblank);
			uint32_t expLines = std::min<uint32_t>(exposureLines(exposure, lineLength),
							       frameLength - frameIntegrationDiff);
			exposure = CamHelper::exposure(expLines, lineLength);
		}
	}

	while ((frameLength >> shift) > frameLengthMax && shift < longExposureShiftMax)
		shift++;

	if (shift) {
		Duration lineLength = hblankToLineLength(hblank);
		uint32_t exposureLines = CamHelper::exposureLines(exposure, lineLength);

		/* The sensor sees both values divided by 2^shift. */
		exposureLines = (exposureLines >> shift) << shift;
		exposureLines = std::min(exposureLines,
					 frameLength - (frameIntegrationDiff << shift));
		exposure = CamHelper::exposure(exposureLines, lineLength);
	}

	return { vblank, hblank };
}

bool CamHelperImx06a::sensorEmbeddedDataPresent() const
{
	/*
	 * The "embedded data" is the PD stream. The pipeline only enables it
	 * when the sensor exposes the PD pad (driver option "pdaf").
	 */
	return true;
}

/*
 * PD layout per mode, mirroring the driver: one packet per 4 output lines,
 * L/R values interleaved along the packet, one value per output column at
 * binned widths (full res: per 2 columns), at most 4096.
 *
 * Full-res RAW12 carries the sensor's sparse PD pattern instead (1016 samples
 * per packet, L/R in packet pairs). Its phase does not track the all-pixel PD
 * consistently (opposite sign, scene-dependent ratio), so it is not used: AF
 * falls back to contrast detection in that mode.
 */
bool CamHelperImx06a::pdGeometry(PdGeom &g) const
{
	const bool full = mode_.scaleX < 1.5;

	if (!mode_.width || !mode_.height || (full && mode_.bitdepth == 12))
		return false;

	g.px = std::min<unsigned int>(full ? mode_.width / 2 : mode_.width, 4096);
	g.rows = (mode_.height + 3) / 4;
	g.sites = g.px / 2;
	g.colsPerSite = mode_.width * mode_.scaleX / g.sites;
	return true;
}

void CamHelperImx06a::prepare(Span<const uint8_t> buffer, Metadata &metadata)
{
	PdGeom g;

	if (buffer.empty() || !pdGeometry(g))
		return;

	const size_t need = static_cast<size_t>(g.px) * 10 / 8 * g.rows;
	if (buffer.size() < need) {
		if (!pdWarned_++)
			LOG(IPARPI, Warning)
				<< "PD buffer " << buffer.size() << " B < " << need
				<< " B expected: PDAF disabled (driver pd_standin?)";
		return;
	}

	/*
	 * rpi.af runs at most ~30 times a second: at higher frame rates
	 * re-publish the last grid rather than recomputing it every frame.
	 */
	auto t0 = std::chrono::steady_clock::now();
	const unsigned int key = g.sites * 65536 + g.rows;
	if (pdHaveLast_ && key == pdLastKey_ &&
	    t0 - pdLastTime_ < std::chrono::milliseconds(25)) {
		metadata.set("pdaf.regions", pdLast_);
		return;
	}

	/*
	 * The receiver's buffer is not CPU-cached: copy it once with wide
	 * loads rather than reading it byte by byte.
	 */
	pdRaw_.resize(need);
	memcpy(pdRaw_.data(), buffer.data(), need);
	auto t1 = std::chrono::steady_clock::now();

	PdafRegions regions;
	if (!computePdaf(pdRaw_, g, regions))
		return;
	metadata.set("pdaf.regions", regions);
	pdLast_ = regions;
	pdLastTime_ = t0;
	pdLastKey_ = key;
	pdHaveLast_ = true;

	if (!(pdFrames_++ % 30)) {
		using std::chrono::microseconds;
		auto t2 = std::chrono::steady_clock::now();
		const PdafData &c = regions.get(libcamera::Point(pdafCols / 2, pdafRows / 2)).val;
		LOG(IPARPI, Debug)
			<< "PDAF " << g.sites << "x" << g.rows
			<< " sites: centre phase " << c.phase / 16.0 << " conf " << c.conf
			<< ", copy " << std::chrono::duration_cast<microseconds>(t1 - t0).count()
			<< " us, compute " << std::chrono::duration_cast<microseconds>(t2 - t1).count()
			<< " us";
	}

	/*
	 * Raw L/R images for applications that ask (rpi::PdafOutputEnable):
	 * a window (small) up to 25 Hz, the whole field up to 10 Hz.
	 */
	bool imageRequest = false;
	libcamera::Rectangle window;
	const bool haveWindow = !metadata.get("pdaf.image_window", window);
	if (!metadata.get("pdaf.image_request", imageRequest) && imageRequest &&
	    t0 - pdImageTime_ >= std::chrono::milliseconds(haveWindow ? 40 : 100)) {
		PdafImage image;
		makePdafImage(g, haveWindow ? &window : nullptr, image);
		if (haveWindow)
			image.window = window;
		metadata.set("pdaf.image", std::move(image));
		pdImageTime_ = t0;
	}
}

/*
 * Phase from the raw left/right photodiode images.
 *
 * The L and R planes are binned vertically (phase is horizontal) and each row
 * is divided by its local mean (box of 63 sites), which removes the L/R
 * shading - the two halves see opposite sides of the exit pupil, so their
 * gains differ and vary across the field - and keeps the texture that carries
 * the phase. Per region, the shift s maximising the normalised correlation of
 * L(x) and R(x + s) is found and refined with a parabola through the peak.
 *
 * Output per region: phase = shift in units of 4 sensor columns (one PD site
 * in the 4x2-binned modes; the unit is the same in every mode, so one
 * pdaf_gain fits all), Q4; conf = how far the correlation peak rises above an
 * uncorrelated floor, 0..1023 (rpi.af clips at conf_clip). A peak at the edge
 * of the search range, or a region too dark to trust, has conf 0.
 */
bool CamHelperImx06a::computePdaf(const std::vector<uint8_t> &buffer, const PdGeom &g,
				  PdafRegions &regions)
{
	const unsigned int W = g.sites;
	/* About 16 binned rows per region row. */
	const unsigned int vb = std::max(1u, g.rows / (pdafRows * 16));
	const unsigned int H = g.rows / vb;
	const size_t rowBytes = static_cast<size_t>(g.px) * 10 / 8;
	/* Raw values summed into one binned L (or R) value. */
	const int count = static_cast<int>(vb);

	if (W < pdafCols * 8 || H < pdafRows)
		return false;

	pdL_.assign(static_cast<size_t>(W) * H, 0);
	pdR_.assign(static_cast<size_t>(W) * H, 0);

	/* Unpack RAW10 (4 values in 5 bytes) and bin vertically. */
	const unsigned int groups = g.px / 4;
	auto addLR = [groups](const uint8_t *__restrict p, int32_t *__restrict a,
			      int32_t *__restrict b) {
		for (unsigned int i = 0; i < groups; i++) {
			const unsigned int lo = p[5 * i + 4];
			a[2 * i] += (p[5 * i] << 2) | (lo & 3);
			b[2 * i] += (p[5 * i + 1] << 2) | ((lo >> 2) & 3);
			a[2 * i + 1] += (p[5 * i + 2] << 2) | ((lo >> 4) & 3);
			b[2 * i + 1] += (p[5 * i + 3] << 2) | (lo >> 6);
		}
	};

	const uint8_t *base = buffer.data();
	for (unsigned int y = 0; y < H; y++) {
		int32_t *l = &pdL_[static_cast<size_t>(y) * W];
		int32_t *r = &pdR_[static_cast<size_t>(y) * W];

		for (unsigned int k = 0; k < vb; k++)
			addLR(base + (y * vb + k) * rowBytes, l, r);
	}

	/*
	 * Local normalisation: hp = a / boxmean(a) - 1 in Q10, clamped to
	 * +-2 so that int32 row sums cannot overflow. Rows too dark for the
	 * ratio to mean anything are zeroed.
	 */
	constexpr int box = 63;
	constexpr int one = 1024;
	const int black = pdBlack * count;
	const int minMean = 4 * count;	/* 4 LSB above black per raw value */
	hpL_.assign(static_cast<size_t>(W) * H, 0);
	hpR_.assign(static_cast<size_t>(W) * H, 0);
	std::vector<int32_t> pre(W + 1);

	std::vector<float> num(W), den(W);
	auto normalise = [&](const int32_t *__restrict a, int16_t *__restrict out) {
		const unsigned int h = box / 2;
		float *__restrict nu = num.data();
		float *__restrict de = den.data();

		pre[0] = 0;
		for (unsigned int x = 0; x < W; x++)
			pre[x + 1] = pre[x] + std::max(a[x] - black, 0);
		/*
		 * value x box width, and the box sum (the box is clipped at the
		 * row ends); a box too dark to trust gets a zero sum.
		 */
		for (unsigned int x = 0; x < W; x++) {
			const unsigned int x0 = x > h ? x - h : 0;
			const unsigned int x1 = std::min(x + h + 1, W);
			const int n = static_cast<int>(x1 - x0);
			const int sum = pre[x1] - pre[x0];
			nu[x] = static_cast<float>(std::max(a[x] - black, 0) * n);
			de[x] = sum >= minMean * n ? static_cast<float>(sum) : 0.0f;
		}
		/* fminf/fmaxf rather than std::clamp: branch-free, so it vectorises */
		for (unsigned int x = 0; x < W; x++) {
			const float d = de[x];
			float q = (nu[x] / std::fmax(d, 1.0f) - 1.0f) * one;
			q = std::fmin(std::fmax(q, -2.0f * one), 2.0f * one);
			out[x] = static_cast<int16_t>(static_cast<int>(q) * (d > 0.0f));
		}
	};
	for (unsigned int y = 0; y < H; y++) {
		normalise(&pdL_[static_cast<size_t>(y) * W], &hpL_[static_cast<size_t>(y) * W]);
		normalise(&pdR_[static_cast<size_t>(y) * W], &hpR_[static_cast<size_t>(y) * W]);
	}

	/* Search +-64 sensor columns, within a third of a region. */
	const unsigned int regW = W / pdafCols;
	const int S = std::clamp(static_cast<int>(std::lround(64.0 / g.colsPerSite)),
				 4, static_cast<int>(regW / 3));
	const double unit = g.colsPerSite / 4.0;
	constexpr float nccFloor = 0.4f;
	std::vector<int64_t> ab(2 * S + 1), bb(2 * S + 1);
	std::vector<float> ncc(2 * S + 1);

	/* Row prefix sums of R^2, for the R energy under every shifted window. */
	std::vector<int64_t> pbb(static_cast<size_t>(W + 1) * H);
	for (unsigned int y = 0; y < H; y++) {
		const int16_t *b = &hpR_[static_cast<size_t>(y) * W];
		int64_t *pp = &pbb[static_cast<size_t>(y) * (W + 1)];
		pp[0] = 0;
		for (unsigned int x = 0; x < W; x++)
			pp[x + 1] = pp[x] + b[x] * b[x];
	}

	regions.init({ pdafCols, pdafRows });
	for (unsigned int ry = 0; ry < pdafRows; ry++) {
		const unsigned int y0 = ry * H / pdafRows, y1 = (ry + 1) * H / pdafRows;

		for (unsigned int rx = 0; rx < pdafCols; rx++) {
			/* L window; R is read at +-S around it, inside the plane. */
			const int x0 = std::max<int>(rx * W / pdafCols, S);
			const int x1 = std::min<int>((rx + 1) * W / pdafCols, W - S);
			int64_t aa = 0;
			int64_t sig = 0;

			std::fill(ab.begin(), ab.end(), 0);
			std::fill(bb.begin(), bb.end(), 0);
			for (unsigned int y = y0; y < y1; y++) {
				const int16_t *__restrict a = &hpL_[static_cast<size_t>(y) * W];
				const int16_t *__restrict b = &hpR_[static_cast<size_t>(y) * W];
				const int32_t *raw = &pdL_[static_cast<size_t>(y) * W];
				const int64_t *pp = &pbb[static_cast<size_t>(y) * (W + 1)];
				int32_t sa = 0;

				for (int x = x0; x < x1; x++) {
					sa += a[x] * a[x];
					sig += raw[x];
				}
				aa += sa;
				for (int s = -S; s <= S; s++) {
					const int16_t *bs = b + s;
					int32_t sab = 0;
					for (int x = x0; x < x1; x++)
						sab += a[x] * bs[x];
					ab[s + S] += sab;
					bb[s + S] += pp[x1 + s] - pp[x0 + s];
				}
			}

			PdafData d{ 0, 0 };
			const int64_t n = static_cast<int64_t>(y1 - y0) * (x1 - x0);
			if (n > 0 && aa > 0 && sig / n - black >= minMean) {
				int best = 0;
				for (int i = 0; i <= 2 * S; i++) {
					ncc[i] = bb[i] > 0
						? static_cast<float>(ab[i] / std::sqrt(static_cast<double>(aa) * bb[i]))
						: 0.0f;
					if (ncc[i] > ncc[best])
						best = i;
				}
				if (best > 0 && best < 2 * S && ncc[best] > nccFloor) {
					const float y_0 = ncc[best - 1], y_1 = ncc[best], y_2 = ncc[best + 1];
					const float curv = y_0 - 2 * y_1 + y_2;
					const float sub = curv < 0 ? 0.5f * (y_0 - y_2) / curv : 0.0f;
					const double shift = (best - S + sub) * unit;
					const float c = (ncc[best] - nccFloor) / (1.0f - nccFloor);

					d.phase = static_cast<int16_t>(std::clamp<long>(std::lround(shift * 16),
											 -32768, 32767));
					d.conf = static_cast<uint16_t>(std::lround(1023 * c));
				}
			}
			regions.set(libcamera::Point(rx, ry), { d, 1, 0 });
		}
	}

	return true;
}

/*
 * The left and right photodiode images for display: the whole field at most
 * 400 samples wide with square samples (several PD sites x PD rows summed), or
 * a window (ScalerCrop coordinates) at the native PD sampling, one sample per
 * site and PD row (decimated only beyond 512 x 256). Black removed, both planes
 * scaled by one factor (99.5th percentile = 255) to 8 bits.
 */
void CamHelperImx06a::makePdafImage(const PdGeom &g, const libcamera::Rectangle *window,
				    PdafImage &image) const
{
	unsigned int x0 = 0, x1 = g.sites, y0 = 0, y1 = g.rows, dx, dy;

	if (window) {
		/* sensor -> mode output pixels -> PD sites (x) and PD rows (4 lines, y) */
		const double outPerSite = static_cast<double>(mode_.width) / g.sites;
		auto site = [&](double x) {
			return (x - mode_.cropX) / mode_.scaleX / outPerSite;
		};
		auto row = [&](double y) {
			return (y - mode_.cropY) / mode_.scaleY / 4.0;
		};
		auto clampTo = [](double v, unsigned int lo, unsigned int hi) {
			return static_cast<unsigned int>(std::clamp(v, static_cast<double>(lo),
								    static_cast<double>(hi)));
		};
		x0 = clampTo(std::floor(site(window->x)), 0, g.sites - 1);
		x1 = clampTo(std::ceil(site(window->x + static_cast<double>(window->width))), x0 + 1, g.sites);
		y0 = clampTo(std::floor(row(window->y)), 0, g.rows - 1);
		y1 = clampTo(std::ceil(row(window->y + static_cast<double>(window->height))), y0 + 1, g.rows);
		dx = (x1 - x0 + 511) / 512;
		dy = (y1 - y0 + 255) / 256;
	} else {
		dx = (g.sites + 399) / 400;
		/* square samples: the output aspect is the image's */
		const unsigned int w = g.sites / dx;
		const unsigned int hTarget = std::max(1u, static_cast<unsigned int>(
			std::lround(static_cast<double>(w) * mode_.height / mode_.width)));
		dy = std::max(1u, g.rows / hTarget);
	}
	const unsigned int w = (x1 - x0) / dx;
	const unsigned int h = (y1 - y0) / dy;
	const size_t rowBytes = static_cast<size_t>(g.px) * 10 / 8;
	std::vector<int32_t> acc(2 * static_cast<size_t>(w) * h, 0);
	const unsigned int xEnd = x0 + w * dx;

	for (unsigned int r = y0; r < y0 + h * dy; r++) {
		const uint8_t *row = pdRaw_.data() + r * rowBytes;
		int32_t *l = &acc[static_cast<size_t>((r - y0) / dy) * w];
		int32_t *rt = l + static_cast<size_t>(w) * h;
		/* one 5-byte group = L R L R = sites 2i and 2i + 1 */
		for (unsigned int i = x0 / 2; 2 * i < xEnd; i++) {
			const uint8_t *p = row + 5 * i;
			const unsigned int lo = p[4];
			const unsigned int sa = 2 * i, sb = 2 * i + 1;
			if (sa >= x0 && sa < xEnd) {
				l[(sa - x0) / dx] += (p[0] << 2) | (lo & 3);
				rt[(sa - x0) / dx] += (p[1] << 2) | ((lo >> 2) & 3);
			}
			if (sb >= x0 && sb < xEnd) {
				l[(sb - x0) / dx] += (p[2] << 2) | ((lo >> 4) & 3);
				rt[(sb - x0) / dx] += (p[3] << 2) | (lo >> 6);
			}
		}
	}

	const int black = pdBlack * static_cast<int>(dx * dy);
	int peak = 1;
	for (int32_t &v : acc) {
		v = std::max(v - black, 0);
		peak = std::max(peak, v);
	}
	/* 99.5th percentile from a histogram, so a few highlights do not set the scale */
	std::vector<unsigned int> hist(1024, 0);
	for (int32_t v : acc)
		hist[static_cast<int64_t>(v) * 1023 / peak]++;
	size_t count = 0, target = acc.size() * 995 / 1000;
	unsigned int bin = 0;
	while (bin < 1023 && (count += hist[bin]) < target)
		bin++;
	peak = std::max(1, static_cast<int>(static_cast<int64_t>(peak) * (bin + 1) / 1024));

	image.width = w;
	image.height = h;
	image.data.resize(acc.size());
	for (size_t i = 0; i < acc.size(); i++)
		image.data[i] = static_cast<uint8_t>(std::min<int64_t>(255, static_cast<int64_t>(acc[i]) * 255 / peak));
}

static CamHelper *create()
{
	return new CamHelperImx06a();
}

static RegisterCamHelper reg("imx06a", &create);
