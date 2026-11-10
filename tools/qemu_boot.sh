#!/usr/bin/env bash
# Boot a built flight image in Espressif's QEMU (inside the pinned ESP-IDF container) and check the UART log.
# Usage: tools/qemu_boot.sh <build-dir under firmware/flight> [seconds]
#   tools/qemu_boot.sh build-esp32s3-qemu 30
# QEMU has esp32, esp32c3 and esp32s3 machines only: no P4, no C5, no radio, no sensors. This proves the image
# boots, FreeRTOS starts and app_main runs without a panic; it says nothing about the board.
set -euo pipefail

IDF_IMAGE="${IDF_IMAGE:-espressif/idf:release-v5.5}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${1:?usage: tools/qemu_boot.sh <build-dir> [seconds]}"
SECONDS_TO_RUN="${2:-30}"
DIR="$ROOT/firmware/flight/$BUILD"
LOG="$DIR/qemu_uart.log"
EXPECT="${QEMU_EXPECT:-main_task: Calling app_main}"

docker run --rm -v "$ROOT":/project -w "/project/firmware/flight/$BUILD" -e HOME=/tmp "$IDF_IMAGE" bash -c '
  set -e
  esptool.py --chip esp32s3 merge_bin --output qemu_flash.bin --fill-flash-size 2MB @flash_args >/dev/null
  # eFuse image, raw blocks: chip revision v0.3 (block 1, as idf.py qemu writes it) and BLK_VERSION_MAJOR = 1
  # (block 2). Without the second the ADC start-up code self-calibrates against an ADC QEMU does not emulate.
  python3 -c "
e = bytearray(1024); e[0x26] = 0x0C; e[0x40] = 0x01; open(\"qemu_efuse.bin\", \"wb\").write(e)"
  timeout '"$SECONDS_TO_RUN"' /opt/esp/tools/qemu-xtensa/*/qemu/bin/qemu-system-xtensa -M esp32s3 -m 32M \
    -drive file=qemu_flash.bin,if=mtd,format=raw -drive file=qemu_efuse.bin,if=none,format=raw,id=efuse \
    -global driver=nvram.esp32s3.efuse,property=drive,value=efuse \
    -global driver=timer.esp32s3.timg,property=wdt_disable,value=true \
    -nographic -monitor none -serial file:qemu_uart.log >/dev/null 2>&1 || [ $? -eq 124 ]
'

echo "UART log: $LOG ($(wc -l < "$LOG") lines)"
if grep -E "Guru Meditation|abort\(\) was called|Backtrace:|stack overflow|assert failed" "$LOG"; then
  echo "FAIL: panic in the emulated boot" >&2
  exit 1
fi
if ! grep -q "$EXPECT" "$LOG"; then
  echo "FAIL: \"$EXPECT\" not reached; last lines:" >&2
  tail -5 "$LOG" >&2
  exit 1
fi
echo "PASS: reached \"$EXPECT\", no panic in ${SECONDS_TO_RUN} s"
