#!/usr/bin/env python3
"""Add extra ed25519 public keys to MCUboot's key table (boot/zephyr/keys.c).

The Zephyr port compiles exactly one key from CONFIG_BOOT_SIGNATURE_KEY_FILE; bootutil itself
selects among any number of keys by the image's key-hash TLV. Run on a pristine keys.c.
"""

import base64
import re
import sys
from pathlib import Path

MARKER = "/* window-sensor: extra keys */"
TABLE_END = "};\nconst int bootutil_key_cnt = 1;\n#endif /* HAVE_KEYS */"


def der_from_pem(path: Path) -> bytes:
    body = re.search(
        r"-----BEGIN PUBLIC KEY-----(.*?)-----END PUBLIC KEY-----", path.read_text(), re.DOTALL
    )
    if body is None:
        sys.exit(f"{path}: not a PEM public key")
    der = base64.b64decode("".join(body.group(1).split()))
    # SubjectPublicKeyInfo for Ed25519: fixed 12-byte prefix + 32-byte key, as imgtool getpub emits.
    if len(der) != 44 or not der.startswith(bytes.fromhex("302a300506032b6570032100")):
        sys.exit(f"{path}: not an ed25519 public key")
    return der


def main() -> None:
    keys_c = Path(sys.argv[1])
    pems = [Path(p) for p in sys.argv[2:]]
    source = keys_c.read_text()
    if MARKER in source:
        sys.exit(f"{keys_c} is already patched; restore it first")
    if source.count(TABLE_END) != 1:
        sys.exit(f"{keys_c}: key table layout changed, update this script")

    arrays, entries = [MARKER], []
    for i, pem in enumerate(pems):
        der = ", ".join(f"0x{b:02x}" for b in der_from_pem(pem))
        arrays.append(f"static const unsigned char extra_pub_key_{i}[] = {{ {der} }};")
        arrays.append(f"static const unsigned int extra_pub_key_{i}_len = {len(der.split(','))};")
        entries.append(f"    {{ .key = extra_pub_key_{i}, .len = &extra_pub_key_{i}_len }},")

    patched_table = (
        "\n".join(entries)
        + f"\n}};\nconst int bootutil_key_cnt = {1 + len(pems)};\n#endif /* HAVE_KEYS */"
    )
    source = source.replace(TABLE_END, patched_table)
    source = source.replace(
        "#if defined(HAVE_KEYS)\n", "\n".join(arrays) + "\n#if defined(HAVE_KEYS)\n", 1
    )
    keys_c.write_text(source)
    print(f"Added {len(pems)} extra MCUboot key(s) to {keys_c}")


if __name__ == "__main__":
    main()
