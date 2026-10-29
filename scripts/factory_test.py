# /// script
# requires-python = ">=3.11"
# dependencies = ["bleak>=0.21"]
# ///
"""Two-stage factory test for window-sensor boards (in the panel, pogo clamp, DPS-150, Pico probe).

    just factory-test top    # after the top-side reflow
    just factory-test full   # after the bottom-side reflow; ends with the production flash

The production image is the CI-built factory hex (`just fetch-factory <run-id>`), so MCUboot and
the app are signed with the same key.

The bring-up firmware reports `MEAS key=value` lines over RTT; this script handles operator
prompts, BLE checks and manual measurements, applies scripts/factory_limits.toml and stores
one record per board in factory-results/<device_id>.json plus factory-results/summary.csv.
Exit code 0 = PASS.
"""

import argparse
import asyncio
import csv
import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import tomllib
from datetime import UTC, datetime
from pathlib import Path

from bleak import BleakScanner
from bleak.exc import BleakError

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware"
ELF = FIRMWARE / "target/thumbv8m.main-none-eabihf/debug/bringup"
LIMITS = Path(__file__).with_name("factory_limits.toml")
RESULTS = ROOT / "factory-results"
PRODUCTION_HEX = FIRMWARE / "production" / "window-sensor-factory.hex"
PROBE_RS_RUN = [
    "probe-rs", "run",
    "--chip", "nRF54L10",
    "--chip-description-path", str(FIRMWARE / "nRF54L10.yaml"),
    "--connect-under-reset",
    "--log-format", "{s}",
]
TEST_TIMEOUT_S = 120
BLE_SCAN_S = 15
BATTERY_SCAN_S = 30
# Values the debug run cannot produce; they come from the later full-stage steps.
LATE_KEYS = {"battery_ble_seen", "sleep_ua", "production_flash"}
STRING_KEYS = {"device_id", "ble_addr", "mode"}

MEAS = re.compile(r"\bMEAS (\w+)=(\S+)")
PROMPT = re.compile(r"\bPROMPT (\w+)")


def parse_value(key: str, raw: str):
    if key in STRING_KEYS or raw == "none":
        return raw
    try:
        return int(raw, 0)
    except ValueError:
        return float(raw)


def banner(text: str) -> None:
    print(f"\n{'=' * 72}\n>>> {text}\n{'=' * 72}", flush=True)


def ask_yes(question: str) -> bool:
    while (answer := input(f"\n>>> {question} [y/n] ").strip().lower()) not in ("y", "n"):
        pass
    return answer == "y"


def ask_float(question: str) -> float:
    while True:
        try:
            return float(input(f"\n>>> {question} ").strip().replace(",", "."))
        except ValueError:
            print("Enter a number.")


def build() -> None:
    print("Building bring-up firmware ...", flush=True)
    subprocess.run(
        ["cargo", "build", "--features", "embedded-bin", "--bin", "bringup"],
        cwd=FIRMWARE,
        env={**os.environ, "HWTEST_BARE": "1"},
        check=True,
    )


class RttSession:
    """`probe-rs run` with its output delivered line by line through a queue."""

    def __init__(self) -> None:
        self.proc = subprocess.Popen(
            [*PROBE_RS_RUN, str(ELF)],
            cwd=FIRMWARE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.lines: queue.Queue[str | None] = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self) -> None:
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            self.lines.put(line.rstrip())
        self.lines.put(None)

    def stop(self) -> None:
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


async def find_advertiser(addr_hex: str, timeout_s: float) -> bool:
    target = ":".join(addr_hex[i : i + 2] for i in range(0, 12, 2)).upper()
    seen = asyncio.Event()

    def on_advertisement(device, _adv) -> None:
        if device.address.upper() == target:
            seen.set()

    async with BleakScanner(detection_callback=on_advertisement):
        try:
            await asyncio.wait_for(seen.wait(), timeout_s)
        except TimeoutError:
            return False
    return True


def run_debug_test(stage: str) -> dict:
    meas: dict = {}
    session = RttSession()
    deadline = time.monotonic() + TEST_TIMEOUT_S
    try:
        while meas.get("done") != 1:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                print("!! firmware did not finish in time", flush=True)
                break
            try:
                line = session.lines.get(timeout=remaining)
            except queue.Empty:
                continue
            if line is None:
                print("!! probe-rs exited", flush=True)
                break
            print(f"  | {line}", flush=True)
            if m := MEAS.search(line):
                meas[m[1]] = parse_value(m[1], m[2])
            if p := PROMPT.search(line):
                started = time.monotonic()
                if p[1] == "magnet":
                    banner("Bring the magnet close to the TMAG5273 now (15 s window)")
                elif p[1] == "leds" and stage == "full":
                    meas["leds_ok"] = int(ask_yes("Do the blue (D1) and red (D2) LEDs blink alternately?"))
                deadline += time.monotonic() - started
        if meas.get("done") == 1 and isinstance(meas.get("ble_addr"), str):
            print(f"Scanning {BLE_SCAN_S} s for the board's advertisement ...", flush=True)
            meas["ble_seen"] = int(asyncio.run(find_advertiser(meas["ble_addr"], BLE_SCAN_S)))
    finally:
        session.stop()
    return meas


def battery_check(meas: dict) -> None:
    banner(
        "Battery holder: unplug the Pico USB, remove the pogo clamp, then press Enter\n"
        f">>> and insert a cell into BT1 within {BATTERY_SCAN_S} s"
    )
    input()
    meas["battery_ble_seen"] = int(asyncio.run(find_advertiser(meas["ble_addr"], BATTERY_SCAN_S)))
    print(f"Advertisement on battery power: {bool(meas['battery_ble_seen'])}", flush=True)


def sleep_current(meas: dict) -> None:
    banner(
        "Sleep current:\n"
        ">>>  1. Remove the cell, put the pogo clamp back on (Pico stays unplugged).\n"
        ">>>  2. UT139C in series with DPS+ in the mA range, then switch the DPS output on.\n"
        ">>>  3. After ~10 s (5 s advertising, then sleep) switch to µA and wait for a stable value."
    )
    meas["sleep_ua"] = ask_float("Sleep current in µA:")


def production_flash(meas: dict, production_hex: Path) -> None:
    banner("Remove the UT139C (DPS direct), reconnect the Pico, then press Enter to flash production")
    input()
    cmd = ["just", "flash-factory-hex", str(production_hex)]
    meas["production_flash"] = int(subprocess.run(cmd, cwd=ROOT, check=False).returncode == 0)


def within(value, rule: dict) -> bool:
    if value is None or isinstance(value, str):
        return False
    if "eq" in rule and value != rule["eq"]:
        return False
    if "min" in rule and value < rule["min"]:
        return False
    return not ("max" in rule and value > rule["max"])


def evaluate(stage: str, meas: dict, include_late: bool) -> list[dict]:
    limits = tomllib.loads(LIMITS.read_text())
    rules = dict(limits["top"])
    if stage == "full":
        rules.update(limits["full"])
    return [
        {"key": key, "value": meas.get(key), "rule": rule, "ok": within(meas.get(key), rule)}
        for key, rule in rules.items()
        if include_late or key not in LATE_KEYS
    ]


def report(checks: list[dict]) -> bool:
    print("\n----- checks -----")
    for c in checks:
        print(f"  {'ok  ' if c['ok'] else 'FAIL'} {c['key']:<24} {c['value']!s:<14} {c['rule']}")
    return all(c["ok"] for c in checks)


def store(stage: str, meas: dict, checks: list[dict], passed: bool) -> Path:
    RESULTS.mkdir(exist_ok=True)
    now = datetime.now(UTC).isoformat(timespec="seconds")
    device = meas.get("device_id", "unknown")
    path = RESULTS / f"{device}.json"
    history = json.loads(path.read_text()) if path.exists() else []
    history.append(
        {"stage": stage, "time": now, "result": "PASS" if passed else "FAIL", "measurements": meas, "checks": checks}
    )
    path.write_text(json.dumps(history, indent=2) + "\n")

    summary = RESULTS / "summary.csv"
    new = not summary.exists()
    with summary.open("a", newline="") as f:
        writer = csv.writer(f)
        if new:
            writer.writerow(["time", "device_id", "stage", "result", "failed"])
        failed = ";".join(c["key"] for c in checks if not c["ok"])
        writer.writerow([now, device, stage, "PASS" if passed else "FAIL", failed])
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stage", choices=["top", "full"])
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument(
        "--production-hex",
        type=Path,
        default=PRODUCTION_HEX,
        help="factory image flashed at the end of the full stage (default: %(default)s)",
    )
    args = parser.parse_args()

    if args.stage == "full" and not args.production_hex.is_file():
        print(f"Production image {args.production_hex} not found; get it with `just fetch-factory <run-id>`.")
        return 2

    try:
        asyncio.run(find_advertiser("000000000000", 0.5))
    except BleakError as err:
        print(f"Bluetooth not usable, fix before testing: {err}")
        return 2

    if not args.skip_build:
        build()
    meas = run_debug_test(args.stage)

    if args.stage == "full":
        if report(evaluate(args.stage, meas, include_late=False)) and isinstance(meas.get("ble_addr"), str):
            battery_check(meas)
            sleep_current(meas)
            production_flash(meas, args.production_hex)
        else:
            print("\nDebug-run checks failed: skipping battery, sleep-current and production steps.")

    checks = evaluate(args.stage, meas, include_late=True)
    passed = report(checks)
    path = store(args.stage, meas, checks, passed)
    print(f"\nRESULT {'PASS' if passed else 'FAIL'} device={meas.get('device_id', 'unknown')} stage={args.stage}")
    print(f"Record: {path}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
