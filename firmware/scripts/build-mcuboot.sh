#!/usr/bin/env bash
# Build MCUboot for nRF54L10 using the nRF Connect SDK (Zephyr/west).
# Called by CI or locally via `just build-mcuboot`.
#
# Inputs (env vars):
#   NCS_VERSION    - nRF Connect SDK version tag (default: v2.9.0)
#   SIGNING_KEY    - path to ed25519 signing key PEM (private or public)
#   OUTPUT_HEX     - where to write the bootloader hex (default: firmware/mcuboot/mcuboot-nrf54l10.hex)
#   BOARD          - Zephyr board target (default: nrf54l10dk/nrf54l10/cpuapp)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MCUBOOT_DIR="$REPO_ROOT/firmware/mcuboot"

NCS_VERSION="${NCS_VERSION:-v2.9.0}"
SIGNING_KEY="${SIGNING_KEY:-$REPO_ROOT/firmware/keys/signing-key.pem}"
OUTPUT_HEX="${OUTPUT_HEX:-$MCUBOOT_DIR/mcuboot-nrf54l10.hex}"
BOARD="${BOARD:-nrf54l10dk/nrf54l10/cpuapp}"

WEST_WORKSPACE="${WEST_WORKSPACE:-/tmp/ncs-mcuboot-workspace}"

if [[ ! -f "$SIGNING_KEY" ]]; then
    echo "ERROR: Signing key not found at $SIGNING_KEY"
    echo "Run 'just gen-key' first, or set SIGNING_KEY env var."
    exit 1
fi

echo "=== Building MCUboot for nRF54L10 ==="
echo "  NCS version:  $NCS_VERSION"
echo "  Board:        $BOARD"
echo "  Signing key:  $SIGNING_KEY"
echo "  Output:       $OUTPUT_HEX"
echo ""

# --- Initialize west workspace (cached in CI) ---
if [[ ! -d "$WEST_WORKSPACE/.west" ]]; then
    echo "Initializing nRF Connect SDK workspace..."
    mkdir -p "$WEST_WORKSPACE"
    cd "$WEST_WORKSPACE"
    west init -m https://github.com/nrfconnect/sdk-nrf --mr "$NCS_VERSION"
    west update --narrow -o=--depth=1
else
    echo "Using cached west workspace at $WEST_WORKSPACE"
    cd "$WEST_WORKSPACE"
fi

# --- Build MCUboot ---
echo "Building MCUboot..."
west build \
    -b "$BOARD" \
    -d build-mcuboot \
    bootloader/mcuboot/boot/zephyr \
    --pristine=auto \
    -- \
    -DOVERLAY_CONFIG="$MCUBOOT_DIR/mcuboot.conf" \
    -DDTC_OVERLAY_FILE="$MCUBOOT_DIR/partitions.overlay" \
    -DCONFIG_BOOT_SIGNATURE_KEY_FILE="\"$SIGNING_KEY\""

# --- Extract output ---
BUILD_HEX="$WEST_WORKSPACE/build-mcuboot/zephyr/zephyr.hex"
if [[ ! -f "$BUILD_HEX" ]]; then
    echo "ERROR: Build succeeded but hex not found at $BUILD_HEX"
    exit 1
fi

cp "$BUILD_HEX" "$OUTPUT_HEX"
HEX_SIZE=$(stat -c%s "$OUTPUT_HEX")
echo ""
echo "=== MCUboot built successfully ==="
echo "  Output: $OUTPUT_HEX ($HEX_SIZE bytes)"
echo "  Flash:  just flash-factory"
