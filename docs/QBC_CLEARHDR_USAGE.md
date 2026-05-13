# QBC + ClearHDR usage notes

These notes cover the SW-stats pipeline this branch adds to libcamera-rpi
(the `qbc-pisp-handler` branch in `libcamera/`) for the IMX294 Quad-Bayer
mode and IMX585 ClearHDR mode. The hardware NQ stats engine on the PiSP CFE
has a known bug at ≥14-bit unpacked raw and never sees the QBC-permuted
buffer the way the downstream BE needs it; the SW path patches both.

## What the pipeline handler does

There are two SW workers, both living alongside `pipeline/rpi/pisp/`:

| Class               | When it engages                                          | What it does |
|---------------------|----------------------------------------------------------|--------------|
| `QbcRemosaic`       | Sensor exposes `V4L2_CID_USER_BASE + 0x10b8` (QBC mode)  | In-place 4×4 → 2×2 RGGB remosaic on the CFE buffer; full pisp_statistics from QBC raw (matches HW NQ AWB zones + AGC histogram + Y_sum) |
| `RawStatsProducer`  | CFE delivers 14- or 16-bit unpacked raw (any sensor)     | Stats-only, no buffer touch. Reads standard RGGB Bayer at high bit-depth, produces the same pisp_statistics layout the IPA expects |

Both auto-engage from existing stream flags / sensor controls — there is no
opt-in step. The bit-depth trigger uses the same `Needs14bitUnpack` /
`Needs16bitEndianSwap` flags the pipeline handler already sets in
`platformConfigure`, so adding a new sensor with a high-bit-depth mode picks
up the SW stats path automatically.

## How to use it

### IMX294 — full-resolution Quad-Bayer

```sh
# Full-res QBC at 12-bit, the SW remosaic + stats path
rpicam-still --camera 1 \
    --mode 8432:4348:12:U \
    --width 8432 --height 4348 \
    --output qbc.jpg --raw \
    --metadata qbc.json --metadata-format json \
    --awb auto --denoise off \
    --ev -0.5    # see "AGC highlight protection" below
```

The mode hint **must** end in `:U` (unpacked) — packed/compressed modes
(`:P`, `:C`) won't trigger the SW path because they don't deliver the raw
buffer in a form the NEON kernel can read.

Bit-depth fallback non-QBC mode (HW NQ stats, no SW path):

```sh
rpicam-still --camera 1 \
    --mode 4144:2176:12:U \
    --width 4144 --height 2176 \
    --output nq.jpg --raw --awb auto
```

#### Pairing QBC AE with the binned-NQ baseline

rpicam-still's preview phase runs in a smaller binned sensor mode for
speed and only switches to the full QBC mode at the actual capture, so
AGC's preview-mode convergence doesn't transfer cleanly: each NQ-binned
pixel reads ~8× the raw signal of a QBC pixel (see
`memory/imx294_nq_qbc_8x.md`) and AGC has too few frames in the new mode
to re-converge before the still grabs.

For paired comparisons (capture the same scene through both paths)
override QBC's shutter at capture time to 4× the NQ-binned shutter:

```sh
# 1. Auto NQ to get a converged AE/AWB
rpicam-still --camera 1 --mode 4144:2176:12:U --width 4144 --height 2176 \
    --output nq.jpg --raw --metadata nq.json --metadata-format json \
    --timeout 15000 --awb auto --denoise off

# 2. Read NQ metadata, scale shutter by 4× and reuse the AWB gains
read EXP AG RG BG < <(python3 -c "import json,sys; m=json.load(open('nq.json')); print(int(m['ExposureTime']), m['AnalogueGain'], *m['ColourGains'])")
rpicam-still --camera 1 --mode 8432:4348:12:U --width 8432 --height 4348 \
    --output qbc.jpg --raw --metadata qbc.json --metadata-format json \
    --shutter $((EXP * 4)) --gain $AG --awbgains "${RG},${BG}" \
    --timeout 4000 --denoise off
```

`scripts/capture_baseline.sh` does this automatically. The 4× factor is
the geometric mean between "shutter-paired" (literal copy of NQ's
shutter — QBC raw 8× dimmer, JPEG looks ~2 stops underexposed) and
"scene-paired" (8× the shutter — but then AWB gains × the brightest
pixels push past 65535, post-WB clip turns hot highlights magenta).
4× holds roughly one stop of headroom for the WB gains to swing inside
the linear range.

### IMX585 — ClearHDR (16-bit)

Enable ClearHDR via the sensor subdev V4L2 control **before** the libcamera
session starts. The driver flips CCMP off and MDBIT to 16-bit when it sees
the 16-bit format request:

```sh
# Find the subdev
SUBDEV=$(media-ctl -d /dev/media4 -e "imx585 10-001a")

# Engage ClearHDR
v4l2-ctl --device $SUBDEV --set-ctrl wide_dynamic_range=1

# Capture full-res 16-bit ClearHDR — SW stats producer engages automatically
rpicam-still --camera 0 \
    --mode 3856:2180:16:U \
    --width 3856 --height 2180 \
    --output hdr.jpg --raw \
    --metadata hdr.json --metadata-format json \
    --awb auto --denoise off

# Disable when done
v4l2-ctl --device $SUBDEV --set-ctrl wide_dynamic_range=0
```

Non-HDR 12-bit mode on IMX585 takes the regular HW-NQ path — no SW stats,
no extra setup, just the standard `--mode 3856:2180:12:U`.

### Confirming the SW path engaged

Look for these lines in the rpicam log (`LIBCAMERA_LOG_LEVELS='*:INFO'`):

```
RPI pisp.cpp High-bit-depth SW stats producer enabled for imx585 — CFE HW stats unreliable at this bit depth.
RPI pisp.cpp QBC remosaic enabled for imx294
RPI pisp.cpp BLC: tuning=3200 V4L2(cid=0x980900, def=50, now=50) → live pixel BLC=3200 (scale=64)
```

If the line is missing the sensor is running the HW path (which is fine
for 12-bit non-QBC modes but means the SW kernel is not doing the work
you may expect).

## Black-level (BLC) handling

The SW kernels (`QbcRemosaic`, `RawStatsProducer`) read the per-frame BLC
live from the sensor subdev's V4L2 BLC control (currently
`V4L2_CID_BRIGHTNESS` on IMX585 → register `BLKLEVEL`). The pipeline
handler:

1. At configure: loads `rpi.black_level.black_level` from the IPA tuning
   JSON and reads the V4L2 control's default value to derive a per-sensor
   pixel-units scale (e.g. IMX585 default brightness=50, tuning BLC=3200 ⇒
   scale=64). Logs a warning if the current V4L2 reading differs from the
   default ("V4L2 sensor BLC differs from tuning default").
2. Per frame, before `processQbcFrame` / `processRawStats`: re-reads the
   V4L2 control. If it changed, scales it to 16-bit-pixel units and pushes
   the new BLC to the kernel (live-adjustable). Logs the change.

This means you can change the pedestal at runtime:

```sh
# Raise the in-sensor pedestal from 50 (default) to 200; the SW kernel
# follows within ~1 frame, the BE matches via the IPA's black_level path.
v4l2-ctl --device $SUBDEV --set-ctrl brightness=200
# → libcamera log: "BLC: V4L2 sensor BLC changed 50 → 200, using pixel BLC 12800"
```

Sensors that don't expose a BLC control fall back to the IPA tuning value
silently — no warning, no live-adjust.

## Known issues

### AWB latches at wrong CT under warm lighting in `rpicam-still`

Under ~3000 K incandescent light, `rpicam-still` (but **not** `rpicam-vid`)
on the IMX585 16-bit ClearHDR path may converge AWB to ≈3960 K instead
of the correct ≈3290 K, producing a yellow/cream cast on whites. Root
cause is an IPA-side first-frame stats sensitivity that locks AWB into the
wrong local minimum of the Bayes search; the SW kernel's steady-state
zone ratios are identical to HW NQ.

Workaround until the IPA is patched: force a warm AWB mode.

```sh
rpicam-still ... --awb tungsten   # range 3000-3500K
# or --awb incandescent           # range 2500-3000K
```

5500 K and warmer-than-tungsten are fine on auto. See
`memory/awb_3000k_latch.md` for the full diagnosis.

## Reference: baseline captures

`scripts/capture_baseline.sh` produces a complete reference set under
`baseline/`: full-res JPG, full-res DNG, JSON metadata, and the AGC/AWB
debug trace for each mode. Re-run after tuning changes:

```sh
./scripts/capture_baseline.sh                # 5500K @ 70%
./scripts/capture_baseline.sh 3000 60        # 3000K @ 60%
```

The output is five tags, each with `{.jpg, .dng, .json, .log}`:

| Tag                  | Sensor mode                       | Stats path           |
|----------------------|-----------------------------------|----------------------|
| `imx585_12b_full`    | IMX585 3856×2180 12-bit           | HW NQ                |
| `imx585_16b_hdr`     | IMX585 3856×2180 16-bit ClearHDR  | SW RawStatsProducer  |
| `imx294_12b_nq`      | IMX294 4144×2176 12-bit binned    | HW NQ                |
| `imx294_12b_qbc`     | IMX294 8432×4348 12-bit QBC       | SW QbcRemosaic       |
| `imx294_14b`         | IMX294 3792×2824 14-bit           | SW RawStatsProducer  |

The `.log` file ends with a `=== controls metadata ===` block summarising
the final-frame AE/AWB state for that capture.

At 5500 K, IMX585 12b HW and 16b SW converge to within 1 K on CT and
under 0.1 % on gains — a strong validation that the SW stats are
producing the same scene description the HW NQ does.

## Files in this branch

| Path | Purpose |
|------|---------|
| `src/libcamera/pipeline/rpi/pisp/qbc_remosaic.{h,cpp}` | SW QBC remosaic + stats kernel (3-thread NEON) |
| `src/libcamera/pipeline/rpi/pisp/raw_stats.{h,cpp}`     | SW stats-only kernel for high-bit-depth raw |
| `src/libcamera/pipeline/rpi/pisp/pisp.cpp`              | Pipeline handler wiring: detection, BLC plumbing, dmabuf-sync, optional stats dump |
| `src/ipa/rpi/pisp/data/imx294.json`                     | AGC UPPER quantile bound for QBC highlight protection |

Companion driver changes live in `../imx585-v4l2-driver/imx585.c` (the
ClearHDR threshold-register fix). See that repo's commit log for details.
