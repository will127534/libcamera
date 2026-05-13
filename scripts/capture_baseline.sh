#!/bin/bash
# capture_baseline.sh — Reference captures for the SW-stats path verification.
#
# Paired captures, so the SW-stats path sees the same exposure as the HW-NQ
# baseline:
#
#   1. Auto-mode capture on the HW-NQ path (IMX585 12-bit, IMX294 NQ 12-bit) —
#      AGC/AWB converge here, we read the converged metadata.
#   2. Manual-mode capture on the SW-stats path of the same sensor, same
#      scene, using the just-read --shutter / --gain / --awbgains so the
#      two captures are pixel-for-pixel comparable.
#
# Outputs (under $OUT), per tag:
#   <tag>.jpg   - full-res tone-mapped JPEG from libcamera
#   <tag>.dng   - raw Bayer
#   <tag>.json  - controls::* metadata of the captured frame
#   <tag>.log   - AGC/AWB DEBUG trace + a "=== controls metadata ===" tail
#
# Modes covered:
#   imx585_12b_full     - 3856×2180 12-bit (auto)               HW NQ stats
#   imx585_16b_hdr      - 3856×2180 16-bit ClearHDR (manual)    SW RawStats
#   imx294_12b_nq       - 4144×2176 12-bit binned (auto)        HW NQ stats
#   imx294_12b_qbc      - 8432×4348 12-bit QBC (manual)         SW QbcRemosaic
#   imx294_14b          - 3792×2824 14-bit (auto)               SW RawStats
#
# Pre-conditions:
#   - LED on, set to a known CCT (default 5500K @ 70%). Override on the cmdline:
#       ./capture_baseline.sh 3000 60
#   - bluez stopped during the LED poke (script handles via bluez-control.sh).
#
# Usage:
#   ./capture_baseline.sh [cct_kelvin] [led_power_percent]
#   ./capture_baseline.sh                 # 5500K @ 70%
#   ./capture_baseline.sh 3000 60         # 3000K @ 60%
#
# Notes:
#   - Mode hint must end in :U (unpacked). Packed/compressed mode names won't
#     trigger the SW stats path.
#   - IMX585 ClearHDR is enabled via wide_dynamic_range=1 BEFORE the libcamera
#     session starts; the driver flips CCMP off and MDBIT to 16-bit when it
#     sees the 16-bit format request.
#   - Auto AWB on imx585.json's daylight prior is most reliable near 5500K;
#     under 3000K rpicam-still may latch at the wrong CT — use --awb tungsten
#     if you need correct warm WB (see USAGE.md).
set -u

CCT=${1:-5500}
POWER=${2:-70}
OUT=${OUT:-/home/pi/quadbayer_clearHDR_support/libcamera/baseline}
TIMEOUT_AUTO_MS=${TIMEOUT_AUTO_MS:-15000}    # auto-mode: AGC/AWB convergence
TIMEOUT_MANUAL_MS=${TIMEOUT_MANUAL_MS:-4000} # manual: no convergence needed
LIBCAMERA_LOG_LEVELS=${LIBCAMERA_LOG_LEVELS:-RPiAwb:1,RPiAgc:1,RPI:0}

LED_PY=/home/pi/bluetooth_light_control/weeylite.py
BLUEZ_SH=/home/pi/bluetooth_light_control/bluez-control.sh
WEEYLITE_OPTS=(env WEEYLITE_CHANNEL=2 WEEYLITE_GROUP=A python3 "$LED_PY")

mkdir -p "$OUT"

imx585_subdev() {
    for m in /dev/media*; do
        local s
        s=$(media-ctl -d "$m" -e "imx585 10-001a" 2>/dev/null) || true
        [ -n "$s" ] && { echo "$s"; return; }
    done
    return 1
}

cleanup() {
    sudo "$BLUEZ_SH" start 2>/dev/null || true
}
trap cleanup EXIT

# --- LED setup ---
sudo "$BLUEZ_SH" stop >/dev/null 2>&1 || true
sleep 1
"${WEEYLITE_OPTS[@]}" on >/dev/null 2>&1 || true
sleep 1
"${WEEYLITE_OPTS[@]}" cct "$CCT" "$POWER" >/dev/null 2>&1 || true
sleep 2
echo "LED set: CCT=${CCT}K POWER=${POWER}%"

SUBDEV585=$(imx585_subdev) || { echo "IMX585 subdev not found"; exit 1; }
echo "IMX585 subdev: $SUBDEV585"

# --- Capture wrapper ---
# do_capture tag cam mode w h timeout_ms extra_rpicam_args...
do_capture() {
    local tag=$1 cam=$2 mode=$3 w=$4 h=$5 to=$6 ; shift 6
    echo "=== capturing $tag (cam=$cam mode=$mode ${w}x${h} t=${to}ms) ==="
    LIBCAMERA_LOG_LEVELS="$LIBCAMERA_LOG_LEVELS" \
        rpicam-still --camera "$cam" \
            --mode "$mode" --width "$w" --height "$h" \
            --output "$OUT/${tag}.jpg" \
            --raw \
            --metadata "$OUT/${tag}.json" --metadata-format json \
            --timeout "$to" --denoise off \
            "$@" \
            >"$OUT/${tag}.log" 2>&1 || echo "WARN: $tag rpicam exited non-zero (see ${tag}.log)"
    if [ -f "$OUT/${tag}.json" ]; then
        python3 - "$OUT/${tag}.json" <<'PY' >>"$OUT/${tag}.log"
import json, sys
with open(sys.argv[1]) as f: m=json.load(f)
keys=['ExposureTime','AnalogueGain','DigitalGain','ColourGains',
      'ColourTemperature','SensorBlackLevels','AeState','Lux',
      'ColourCorrectionMatrix','ScalerCrop','FrameDuration']
print("=== controls metadata ===")
for k in keys:
    if k in m: print(f"  {k} = {m[k]}")
PY
    fi
    echo "  → ${tag}.{jpg,dng,json,log}  (full-res)"
}

# read_meta path -> echoes "ExposureTime AnalogueGain ColourGains[0] ColourGains[1]"
read_meta() {
    python3 - "$1" <<'PY'
import json, sys
with open(sys.argv[1]) as f: m=json.load(f)
exp=int(m.get('ExposureTime',0))
ag=float(m.get('AnalogueGain',1.0))
cg=m.get('ColourGains',[1.0,1.0])
print(f"{exp} {ag:.6f} {cg[0]:.6f} {cg[1]:.6f}")
PY
}

# --- IMX585 12-bit normal (HW NQ stats baseline, AUTO) ---
v4l2-ctl --device "$SUBDEV585" --set-ctrl wide_dynamic_range=0
sleep 1
do_capture imx585_12b_full 0 3856:2180:12:U 3856 2180 "$TIMEOUT_AUTO_MS" --awb auto

read EXP585 AG585 RG585 BG585 < <(read_meta "$OUT/imx585_12b_full.json")
echo ">> IMX585 baseline AE: shutter=${EXP585}µs gain=${AG585} awb=${RG585},${BG585}"

# --- IMX585 16-bit ClearHDR (SW RawStatsProducer, MANUAL = paired) ---
v4l2-ctl --device "$SUBDEV585" --set-ctrl wide_dynamic_range=1
sleep 1
do_capture imx585_16b_hdr 0 3856:2180:16:U 3856 2180 "$TIMEOUT_MANUAL_MS" \
    --shutter "$EXP585" --gain "$AG585" --awbgains "${RG585},${BG585}"
v4l2-ctl --device "$SUBDEV585" --set-ctrl wide_dynamic_range=0

# --- IMX294 12-bit binned non-QBC (HW NQ stats baseline, AUTO) ---
do_capture imx294_12b_nq 1 4144:2176:12:U 4144 2176 "$TIMEOUT_AUTO_MS" --awb auto

read EXP294 AG294 RG294 BG294 < <(read_meta "$OUT/imx294_12b_nq.json")
echo ">> IMX294 NQ baseline AE: shutter=${EXP294}µs gain=${AG294} awb=${RG294},${BG294}"

# --- IMX294 12-bit full QBC (SW QbcRemosaic, MANUAL with 4× shutter) ---
# NQ binned mode reads 4 photodiodes per output pixel — a single NQ-binned
# pixel is ~8× the raw signal of a QBC pixel at the same shutter (see
# memory/imx294_nq_qbc_8x.md). We boost shutter by 4× (not 8×) when promoting
# NQ's AE to QBC: 4× is the geometric mean between "match shutter" (NQ raw
# in JPEG, dark QBC) and "match scene brightness" (8×, but AWB gains then
# push the brightest pixels past saturation and the post-WB clip turns
# highlights magenta). 4× keeps ~1 stop of headroom for the WB gains to do
# their thing without clipping. AWB carries over directly since it's ratio.
#
# We override AE manually because rpicam-still's preview runs in a binned
# mode and AGC converges there; the still-capture mode switch to QBC happens
# too late for AGC to re-converge before the still is grabbed, so even with
# --awb auto the captured frame keeps the binned-mode shutter.
QBC_EXP=$(python3 -c "print(min(int($EXP294 * 4), 95000))")
do_capture imx294_12b_qbc 1 8432:4348:12:U 8432 4348 "$TIMEOUT_MANUAL_MS" \
    --shutter "$QBC_EXP" --gain "$AG294" --awbgains "${RG294},${BG294}"
echo ">> IMX294 QBC paired AE: shutter=${QBC_EXP}µs (= NQ shutter × 4)"

# --- IMX294 14-bit (SW RawStatsProducer, AUTO) ---
# 14-bit is its own sensor mode at a different resolution / frame timing, so
# auto-AE is appropriate; not directly comparable to the 12-bit NQ exposure.
do_capture imx294_14b 1 3792:2824:14:U 3792 2824 "$TIMEOUT_AUTO_MS" --awb auto

echo
echo "Baseline captures complete in $OUT"
ls -la "$OUT"
