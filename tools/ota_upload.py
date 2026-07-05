#!/usr/bin/env python3
"""Upload a Pico firmware .bin to the OpenTrickler OTA staging area over REST."""

from __future__ import annotations

import argparse
import binascii
import json
import sys
import time
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import urlopen


DEFAULT_CHUNK_SIZE = 512


def request_json(host: str, path: str, params: dict[str, object] | None = None, timeout: float = 15.0) -> dict:
    query = ""
    if params:
        query = "?" + urlencode(params)
    url = f"http://{host}{path}{query}"
    with urlopen(url, timeout=timeout) as response:
        payload = response.read().decode("utf-8", errors="replace")
    try:
        return json.loads(payload)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"bad JSON from {url}: {payload[:200]!r}") from exc


def require_success(data: dict, step: str) -> None:
    if not data.get("success", False):
        message = data.get("message") or data.get("last_error") or "unknown error"
        raise RuntimeError(f"{step} failed: {message}")


def upload(host: str, firmware_bin: Path, chunk_size: int, apply: bool) -> None:
    image = firmware_bin.read_bytes()
    crc = binascii.crc32(image) & 0xFFFFFFFF

    print(f"Firmware: {firmware_bin}")
    print(f"Size:     {len(image)} bytes")
    print(f"CRC32:    {crc:08X}")
    print(f"Target:   http://{host}")

    status = request_json(host, "/rest/ota_status")
    if not status.get("supported", False):
        raise RuntimeError(f"device does not support OTA staging: {status.get('last_error') or status}")
    if apply and not status.get("apply_supported", False):
        raise RuntimeError("device can stage firmware but cannot apply it yet; USB-flash the transition firmware first")
    if len(image) > int(status.get("primary_limit_bytes", 0)):
        raise RuntimeError(
            f"firmware image is too large for primary slot "
            f"({len(image)} > {status.get('primary_limit_bytes')})"
        )
    if len(image) > int(status.get("capacity_bytes", 0)):
        raise RuntimeError(
            f"firmware image is too large for staging slot "
            f"({len(image)} > {status.get('capacity_bytes')})"
        )

    begin = request_json(host, "/rest/ota_begin", {"size": len(image), "crc32": f"0x{crc:08X}"})
    require_success(begin, "ota_begin")

    started = time.monotonic()
    last_print = 0.0
    for offset in range(0, len(image), chunk_size):
        chunk = image[offset : offset + chunk_size]
        data = request_json(host, "/rest/ota_chunk", {"offset": offset, "data": chunk.hex()})
        require_success(data, f"ota_chunk offset={offset}")

        now = time.monotonic()
        if now - last_print >= 1.0 or offset + len(chunk) >= len(image):
            pct = (offset + len(chunk)) * 100.0 / len(image)
            rate = (offset + len(chunk)) / max(0.001, now - started)
            print(f"\rUploaded {offset + len(chunk):7d}/{len(image)} bytes ({pct:5.1f}%) {rate:7.0f} B/s", end="")
            last_print = now

    print()
    final = request_json(host, "/rest/ota_finalize", timeout=30.0)
    require_success(final, "ota_finalize")

    print("Staged and verified on device.")
    print(f"Device CRC: {final.get('actual_crc32')}")

    if apply:
        applied = request_json(host, "/rest/ota_apply", timeout=30.0)
        if applied.get("success", False):
            print("Apply requested; device should reboot into the new firmware.")
        else:
            print("Apply not performed by this firmware:")
            print(applied.get("message") or applied.get("last_error") or applied)
            sys.exit(2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="OpenTrickler host or IP")
    parser.add_argument(
        "--bin",
        default=str(Path("build-pico2w-release") / "app.bin"),
        dest="firmware_bin",
        help="Firmware .bin path, not .uf2",
    )
    parser.add_argument("--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE, help="Upload chunk size in bytes")
    parser.add_argument("--apply", action="store_true", help="Ask the device to apply after staging")
    args = parser.parse_args()

    firmware_bin = Path(args.firmware_bin)
    if not firmware_bin.exists():
        parser.error(f"firmware .bin not found: {firmware_bin}")
    if args.chunk_size <= 0 or args.chunk_size > DEFAULT_CHUNK_SIZE or args.chunk_size % 256 != 0:
        parser.error("--chunk-size must be 256 or 512 bytes")

    try:
        upload(args.host, firmware_bin, args.chunk_size, args.apply)
    except KeyboardInterrupt:
        print("\nInterrupted; asking device to abort OTA upload...")
        try:
            request_json(args.host, "/rest/ota_abort", timeout=5.0)
        except Exception as exc:  # noqa: BLE001
            print(f"Could not abort cleanly: {exc}")
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
