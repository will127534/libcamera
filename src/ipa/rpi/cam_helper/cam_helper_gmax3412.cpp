/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_gmax3412.cpp - camera information for GMAX3412 sensor
 */

#include <algorithm>
#include <array>

#include "cam_helper.h"

namespace RPiController {

class CamHelperGmax3412 : public CamHelper
{
public:
        CamHelperGmax3412();
        uint32_t gainCode(double gain) const override;
        double gain(uint32_t gainCode) const override;
        unsigned int hideFramesStartup() const override;
        unsigned int hideFramesModeSwitch() const override;

private:
        static constexpr int frameIntegrationDiff = 4;
};

namespace {
static constexpr uint8_t kGainRegOffset = 0x12;   // driver expects (reg - 0x12)
static constexpr uint8_t kMinGainReg    = 0x12;
static constexpr uint8_t kMaxGainReg    = 0x3B;

static constexpr std::array<double, (kMaxGainReg - kMinGainReg + 1)> kGainTable = {{
        /* reg 0x12 */ 1.00,
        /* reg 0x13 */ 1.03,
        /* reg 0x14 */ 1.07,
        /* reg 0x15 */ 1.12,
        /* reg 0x16 */ 1.17,
        /* reg 0x17 */ 1.22,
        /* reg 0x18 */ 1.27,
        /* reg 0x19 */ 1.33,
        /* reg 0x1A */ 1.39,
        /* reg 0x1B */ 1.47,
        /* reg 0x1C */ 1.53,
        /* reg 0x1D */ 1.60,
        /* reg 0x1E */ 1.67,
        /* reg 0x1F */ 1.75,
        /* reg 0x20 */ 1.85,
        /* reg 0x21 */ 1.93,
        /* reg 0x22 */ 2.03,
        /* reg 0x23 */ 2.13,
        /* reg 0x24 */ 2.24,
        /* reg 0x25 */ 2.36,
        /* reg 0x26 */ 2.50,
        /* reg 0x27 */ 2.64,
        /* reg 0x28 */ 2.78,
        /* reg 0x29 */ 2.95,
        /* reg 0x2A */ 3.11,
        /* reg 0x2B */ 3.32,
        /* reg 0x2C */ 3.53,
        /* reg 0x2D */ 3.77,
        /* reg 0x2E */ 4.10,
        /* reg 0x2F */ 4.40,
        /* reg 0x30 */ 4.74,
        /* reg 0x31 */ 5.16,
        /* reg 0x32 */ 5.58,
        /* reg 0x33 */ 6.10,
        /* reg 0x34 */ 6.67,
        /* reg 0x35 */ 7.38,
        /* reg 0x36 */ 8.20,
        /* reg 0x37 */ 9.27,
        /* reg 0x38 */ 10.53,
        /* reg 0x39 */ 12.19,
        /* reg 0x3A */ 14.42,
        /* reg 0x3B */ 17.59,
}};
} // namespace

CamHelperGmax3412::CamHelperGmax3412()
        : CamHelper({}, frameIntegrationDiff)
{
}

uint32_t CamHelperGmax3412::gainCode(double gain) const
{
        // Handle NaN/negative/zero robustly.
        if (!(gain > 0.0))
                gain = kGainTable.front();

        gain = std::clamp(gain, kGainTable.front(), kGainTable.back());

        // Find first entry >= gain.
        auto it = std::lower_bound(kGainTable.begin(), kGainTable.end(), gain);

        size_t idx = 0;
        if (it == kGainTable.begin()) {
                idx = 0;
        } else if (it == kGainTable.end()) {
                idx = kGainTable.size() - 1;
        } else {
                const size_t hi = static_cast<size_t>(it - kGainTable.begin());
                const size_t lo = hi - 1;

                const double g_hi = kGainTable[hi];
                const double g_lo = kGainTable[lo];

                idx = ((gain - g_lo) <= (g_hi - gain)) ? lo : hi;
        }

        // IMPORTANT: return (reg - 0x12) because the driver expects offset removed.
        // Actual sensor register would be: reg = idx + 0x12.
        return static_cast<uint32_t>(idx);
}

double CamHelperGmax3412::gain(uint32_t gainCode) const
{
        // gainCode is (reg - 0x12).
        const size_t idx = std::min<size_t>(gainCode, kGainTable.size() - 1);
        return kGainTable[idx];
}

unsigned int CamHelperGmax3412::hideFramesStartup() const
{
        return 1;
}

unsigned int CamHelperGmax3412::hideFramesModeSwitch() const
{
        return 1;
}

static CamHelper *create()
{
        return new CamHelperGmax3412();
}

static RegisterCamHelper reg("gmax3412", &create);

} // namespace RPiController
