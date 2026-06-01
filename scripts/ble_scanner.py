# /// script
# /// requires-python = ">=3.10"
# /// dependencies = ["bleak>=0.21"]
# ///
"""
BTHome v2 BLE Scanner — validates window-sensor advertisements.

Usage:
    uv run ble_scanner.py [--timeout SEC] [--validate] [--json]

Returns JSON-parseable output for agentic evaluation.
Exit code 0 = device found + valid payload.
Exit code 1 = no device found or invalid payload.
"""

import asyncio
import argparse
import json
import struct
import sys
from datetime import datetime

from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

# BTHome v2 service UUID
BTHOME_UUID = "0000fcd2-0000-1000-8000-00805f9b34fb"

# BTHome object decoders: obj_id -> (name, format, factor)
BTHOME_OBJECTS = {
    0x00: ("packet_id", "B", 1),
    0x01: ("battery", "B", 1),         # %
    0x02: ("temperature", "<h", 0.01), # °C
    0x03: ("humidity", "<H", 0.01),    # %
    0x0F: ("generic_boolean", "B", 1),
    0x2B: ("tamper", "B", 1),
    0x2D: ("window", "B", 1),
}

# Expected device name prefix
DEVICE_NAME_PREFIX = "window-sensor"


def decode_bthome_payload(data: bytes) -> dict:
    """Decode a BTHome v2 service data payload."""
    if len(data) < 1:
        return {"error": "empty payload"}

    result = {}
    device_info = data[0]
    version = (device_info >> 5) & 0x07
    trigger_based = bool(device_info & 0x04)
    encrypted = bool(device_info & 0x01)

    result["_version"] = version
    result["_trigger_based"] = trigger_based
    result["_encrypted"] = encrypted

    if version != 2:
        result["error"] = f"unsupported version {version}"
        return result

    pos = 1
    while pos < len(data):
        obj_id = data[pos]
        pos += 1

        if obj_id not in BTHOME_OBJECTS:
            result["error"] = f"unknown object 0x{obj_id:02x} at pos {pos-1}"
            break

        name, fmt, factor = BTHOME_OBJECTS[obj_id]
        size = struct.calcsize(fmt)
        if pos + size > len(data):
            result["error"] = f"truncated payload for {name}"
            break

        raw = struct.unpack(fmt, data[pos:pos + size])[0]
        value = raw * factor if factor != 1 else raw
        result[name] = value
        pos += size

    return result


def validate_payload(decoded: dict) -> list[str]:
    """Check decoded BTHome payload for correctness. Returns list of issues."""
    issues = []

    if decoded.get("_version") != 2:
        issues.append(f"version={decoded.get('_version')}, expected 2")

    if "window" not in decoded:
        issues.append("missing 'window' object")
    elif decoded["window"] not in (0, 1):
        issues.append(f"window={decoded['window']}, expected 0 or 1")

    if "generic_boolean" not in decoded:
        issues.append("missing 'generic_boolean' (tilt) object")

    if "packet_id" not in decoded:
        issues.append("missing 'packet_id' object")

    if "battery" in decoded:
        if not (0 <= decoded["battery"] <= 100):
            issues.append(f"battery={decoded['battery']}%, out of range")

    if "temperature" in decoded:
        t = decoded["temperature"]
        if not (-40 <= t <= 85):
            issues.append(f"temperature={t}°C, out of SHT4x range")

    if "humidity" in decoded:
        h = decoded["humidity"]
        if not (0 <= h <= 100):
            issues.append(f"humidity={h}%, out of range")

    return issues


class ScanResult:
    def __init__(self):
        self.devices_found = []
        self.payloads = []


async def scan(timeout: float, validate: bool, output_json: bool) -> int:
    """Scan for BTHome devices. Returns exit code."""
    result = ScanResult()

    def callback(device: BLEDevice, adv: AdvertisementData):
        # Check for BTHome service data
        if BTHOME_UUID in adv.service_data:
            payload = adv.service_data[BTHOME_UUID]
            decoded = decode_bthome_payload(payload)
            entry = {
                "timestamp": datetime.now().isoformat(),
                "address": device.address,
                "name": adv.local_name or "unknown",
                "rssi": adv.rssi,
                "raw_hex": payload.hex(),
                "decoded": decoded,
            }

            if validate:
                issues = validate_payload(decoded)
                entry["valid"] = len(issues) == 0
                entry["issues"] = issues

            result.devices_found.append(device.address)
            result.payloads.append(entry)

            if not output_json:
                status = "PASS" if entry.get("valid", True) else "FAIL"
                print(f"[{status}] {device.address} RSSI={adv.rssi} {decoded}")

    scanner = BleakScanner(detection_callback=callback)
    await scanner.start()
    await asyncio.sleep(timeout)
    await scanner.stop()

    # Output
    summary = {
        "scan_duration_s": timeout,
        "devices_found": len(set(result.devices_found)),
        "advertisements_captured": len(result.payloads),
        "payloads": result.payloads,
    }

    if validate:
        all_valid = all(p.get("valid", False) for p in result.payloads)
        summary["all_valid"] = all_valid

    if output_json:
        print(json.dumps(summary, indent=2))
    else:
        print(f"\n--- Summary ---")
        print(f"Duration: {timeout}s")
        print(f"Devices: {len(set(result.devices_found))}")
        print(f"Advertisements: {len(result.payloads)}")
        if validate and result.payloads:
            valid_count = sum(1 for p in result.payloads if p.get("valid", False))
            print(f"Valid: {valid_count}/{len(result.payloads)}")

    if not result.payloads:
        if not output_json:
            print("[FAIL] No BTHome devices found")
        return 1

    if validate and not summary.get("all_valid", True):
        return 1

    return 0


def main():
    parser = argparse.ArgumentParser(description="BTHome v2 BLE Scanner")
    parser.add_argument("--timeout", type=float, default=15, help="Scan duration in seconds")
    parser.add_argument("--validate", action="store_true", help="Validate payload structure")
    parser.add_argument("--json", action="store_true", help="JSON output for machine parsing")
    args = parser.parse_args()

    exit_code = asyncio.run(scan(args.timeout, args.validate, args.json))
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
