# Window Sensor firmware development commands
# Usage: just <recipe>   (install: cargo install just)

chip := "nRF54L10"
probe := "--chip " + chip + " --chip-description-path firmware/nRF54L10.yaml"
target := "thumbv8m.main-none-eabihf"
elf := "firmware/target/" + target + "/release/window-sensor"
manifest := "firmware/target/" + target + "/release/partition_manifest.json"

# Default recipe: build + lint
default: build clippy

# === BUILD ===

# Build release firmware (no hardware needed)
build:
    cd firmware && cargo build --release --features embedded-bin

# Show the generated stable partition manifest path
partition-manifest: build
    @printf '%s\n' '{{manifest}}'

# Build with all optional features
build-full:
    cd firmware && cargo build --release --features "embedded-bin,debug-uart,ids,soc-heater"

# Check compilation without producing binary (faster)
check:
    cd firmware && cargo check --release --features embedded-bin

# === QUALITY ===

# Run clippy lints
clippy:
    cd firmware && cargo clippy --release --features embedded-bin -- -D warnings

# Run clippy for the heater-assisted path as well
clippy-soc-heater:
    cd firmware && cargo clippy --release --features "embedded-bin,soc-heater" -- -D warnings

# Format code
fmt:
    cd firmware && cargo fmt

# Format check (CI)
fmt-check:
    cd firmware && cargo fmt -- --check

# === TESTS (host-side, no hardware) ===

# Run all host-side tests (unit + integration)
test:
    cd firmware && cargo test --lib --tests --target x86_64-unknown-linux-gnu

# Run tests with the heater-assisted path enabled
test-soc-heater:
    cd firmware && cargo test --lib --tests --target x86_64-unknown-linux-gnu --features soc-heater

# Run tests with coverage (outputs lcov.info at repo root)
test-cov:
    cd firmware && cargo llvm-cov --lib --tests --target x86_64-unknown-linux-gnu --lcov --output-path ../lcov.info

# Run tests with output (for debugging test failures)
test-verbose:
    cd firmware && cargo test --lib --tests --target x86_64-unknown-linux-gnu -- --nocapture

# === FLASH + RUN (requires debug probe connected) ===

# Flash and run with RTT output (blocks until Ctrl+C)
run:
    cd firmware && cargo run --release --features embedded-bin

# Flash and run the board bring-up test (via probe-rs + RTT), linked bare at 0x0 (no MCUboot)
# Steps: SoC/FICR, HFXO, LFXO load-cap sweep, VDD, POF, RRAM scratch, I2C scan, BLE; then VDD/POF monitor
bringup:
    cd firmware && HWTEST_BARE=1 cargo run --features embedded-bin --bin bringup

# Flash bringup via pyOCD when probe-rs fails to connect. RTT via `just rtt` in a second terminal.
bringup-pyocd:
    cd firmware && HWTEST_BARE=1 cargo build --features embedded-bin --bin bringup
    uvx --from pyocd pyocd flash --target nrf54l --format elf \
        firmware/target/thumbv8m.main-none-eabihf/debug/bringup

# Raw register-level I2C debug session from the first prototype (not clippy-clean)
hwtest-i2c:
    cd firmware && HWTEST_BARE=1 cargo run --features embedded-bin,hwtest-i2c --bin hwtest_i2c

# Per-board factory test in the panel (pogo clamp, DPS-150 at 3.0 V, Pico probe, magnet, UT139C).
# stage=top after the top-side reflow; stage=full after the bottom side (ends with production flash).
factory-test stage *args:
    uv run scripts/factory_test.py {{stage}} {{args}}

# Stream RTT output via pyOCD (run in a second terminal while bringup-pyocd runs)
rtt:
    uvx --from pyocd pyocd rtt --target nrf54l

# Flash only (no RTT capture)
flash:
    probe-rs download {{probe}} {{elf}}

# Reset device
reset:
    probe-rs reset {{probe}}

# === DEV LOOP (build → flash → RTT stream) ===

# One-command dev loop: build release, flash, stream RTT (Ctrl+C to stop)
dev: build
    @echo "--- Flashing {{chip}} ---"
    probe-rs run {{probe}} {{elf}}

# Dev loop with test gate: test → build → flash → RTT
dev-safe: test build
    @echo "--- Tests passed, flashing {{chip}} ---"
    probe-rs run {{probe}} {{elf}}

# === BLE VALIDATION (requires device running + BT adapter) ===

# Scan for BTHome advertisements (15s timeout)
ble-scan:
    cd scripts && uv run ble_scanner.py --timeout 15

# Validate BTHome payload structure
ble-validate:
    cd scripts && uv run ble_scanner.py --timeout 30 --validate

# === RTT MONITORING ===

# Attach to running device and stream RTT (no reflash)
rtt-attach:
    probe-rs attach {{probe}} {{elf}}

# === FULL CI LOOP (what the agent runs) ===

# Full CI loop: lint → test (with coverage) → build (no hardware)
ci: fmt-check clippy clippy-soc-heater test-cov test-soc-heater build
    @echo "[CI] All checks passed"

# Agent loop with hardware: build → flash → validate RTT → validate BLE
ci-hw: build
    @echo "[CI-HW] Flashing..."
    timeout 20 probe-rs run {{probe}} --log-format '{t} [{L}] {s}' {{elf}} 2>&1 \
        | tee /tmp/rtt_output.txt || true
    @echo "[CI-HW] Checking RTT output..."
    rg -q 'BOOT' /tmp/rtt_output.txt && echo "[PASS] Boot message found" || echo "[FAIL] No boot message"

# === SETUP ===

# Install all required tools
setup:
    @echo "Installing Rust toolchain..."
    rustup show || curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    rustup target add {{target}}
    @echo "Installing probe-rs..."
    cargo install probe-rs-tools --locked
    @echo "Installing just..."
    cargo install just
    @echo "Setup complete."

# Show binary size breakdown (cargo-binutils, no arm-none-eabi toolchain needed)
size: build
    cd firmware && cargo size --release -- -A

# Show section totals (text/data/bss)
size-total: build
    cd firmware && cargo size --release

# Show top 20 largest symbols
bloat: build
    cd firmware && cargo nm --release -- --size-sort --print-size -r | head -20

# === IMAGE SIGNING + FACTORY BUILD ===

slot_size   := "0x60000"
header_size := "0x800"
signing_key := "firmware/keys/signing-key.pem"
bin := "firmware/target/" + target + "/release/window-sensor.bin"
hex := "firmware/target/" + target + "/release/window-sensor.hex"
signed_bin := "firmware/target/" + target + "/release/window-sensor-ota.bin"
signed_hex := "firmware/target/" + target + "/release/window-sensor-signed.hex"
factory_hex := "firmware/target/" + target + "/release/window-sensor-factory.hex"
mcuboot_hex := "firmware/mcuboot/mcuboot-nrf54l10.hex"
version     := "0.1.0"

# Generate an MCUboot ed25519 signing key pair (run once)
gen-key:
    bash firmware/scripts/gen-signing-key.sh

# Build + convert ELF to raw binary and Intel HEX
objcopy: build
    rust-objcopy -O binary "{{justfile_directory()}}/{{elf}}" "{{justfile_directory()}}/{{bin}}"
    rust-objcopy -O ihex "{{justfile_directory()}}/{{elf}}" "{{justfile_directory()}}/{{hex}}"

# Sign the firmware for OTA delivery (produces .bin for SMP upload)
sign: objcopy
    imgtool sign \
        --key {{signing_key}} \
        --header-size {{header_size}} \
        --align 4 \
        --slot-size {{slot_size}} \
        --version {{version}} \
        --pad-header \
        {{bin}} {{signed_bin}}
    @echo "OTA image: {{signed_bin}} ($(stat -c%s {{signed_bin}}) bytes / 393216 max)"

# Sign the firmware as Intel HEX (for factory merge)
sign-hex: objcopy
    imgtool sign \
        --key {{signing_key}} \
        --header-size {{header_size}} \
        --align 4 \
        --slot-size {{slot_size}} \
        --version {{version}} \
        --pad-header \
        {{hex}} {{signed_hex}}

# Create factory image (MCUboot + signed app merged)
factory: sign-hex
    python3 -c "from intelhex import IntelHex; boot = IntelHex('{{mcuboot_hex}}'); app = IntelHex('{{signed_hex}}'); boot.merge(app, overlap='error'); boot.write_hex_file('{{factory_hex}}'); print(f'Factory image: {boot.minaddr():#010x}..{boot.maxaddr():#010x}')"
    @echo "Factory image: {{factory_hex}}"

# Flash factory image via probe-rs (first-time board bring-up); chip-erase drops bring-up remnants
flash-factory: factory
    probe-rs download {{probe}} --chip-erase --format hex {{factory_hex}}
    probe-rs reset {{probe}}
    @echo "Device flashed with factory image and reset."

# Flash only the signed app (assumes MCUboot already present)
flash-app: sign-hex
    probe-rs download {{probe}} --format hex {{signed_hex}}
    probe-rs reset {{probe}}

# Download the factory image (MCUboot + app signed with the CI key) of a release workflow run
fetch-factory run_id:
    gh run download {{run_id}} -R usimd/window-sensor -n firmware-factory -D firmware/production

# Flash a prebuilt factory hex (e.g. from fetch-factory) with chip erase
flash-factory-hex hex="firmware/production/window-sensor-factory.hex":
    probe-rs download {{probe}} --chip-erase --format hex {{hex}}
    probe-rs reset {{probe}}

# Install image-building tooling
setup-images:
    pip install imgtool intelhex
    @echo "Installed imgtool + intelhex."
    @echo "Also need: cargo install cargo-binutils"
    cargo install cargo-binutils --locked

# Build MCUboot from source (requires west/Zephyr toolchain)
build-mcuboot:
    bash firmware/scripts/build-mcuboot.sh
