#!/usr/bin/env bash
# Generate an MCUboot ed25519 signing key pair.
# Run ONCE, commit the public key, keep the private key SECRET.
#
# The private key (signing-key.pem) is gitignored and must be stored
# securely (e.g., GitHub Actions secret MCUBOOT_SIGNING_KEY).
#
# The public key gets baked into the MCUboot bootloader binary.

set -euo pipefail

KEY_DIR="$(cd "$(dirname "$0")/.." && pwd)/keys"
mkdir -p "$KEY_DIR"

PRIV_KEY="$KEY_DIR/signing-key.pem"
PUB_KEY="$KEY_DIR/signing-key-pub.pem"

if [[ -f "$PRIV_KEY" ]]; then
    echo "ERROR: $PRIV_KEY already exists. Delete it first if you want to regenerate."
    exit 1
fi

echo "Generating ed25519 signing key pair..."
# imgtool from MCUboot generates keys in the exact format MCUboot expects
if command -v imgtool &>/dev/null; then
    imgtool keygen -k "$PRIV_KEY" -t ed25519
    imgtool getpub -k "$PRIV_KEY" > "$PUB_KEY"
else
    echo "imgtool not found. Install: pip install imgtool"
    echo "Or: uvx imgtool keygen -k $PRIV_KEY -t ed25519"
    exit 1
fi

echo ""
echo "=== Keys generated ==="
echo "  Private (SECRET): $PRIV_KEY"
echo "  Public  (commit): $PUB_KEY"
echo ""
echo "Next steps:"
echo "  1. Add MCUBOOT_SIGNING_KEY as a GitHub Actions secret (base64-encode the .pem)"
echo "  2. Commit signing-key-pub.pem (needed to build MCUboot)"
echo "  3. NEVER commit signing-key.pem"
