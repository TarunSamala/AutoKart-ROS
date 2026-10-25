#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
OUTPUT_DIR=${1:-"$ROOT/dist"}
BASE_PACKAGE=AutoKart-Brake-Test-Ubuntu22.04-Pi4B
ARCH_PACKAGE=AutoKart-Brake-Test-Arch-Laptop
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/base" "$OUTPUT_DIR"

bash "$ROOT/brake_test/portable/build_package.sh" "$WORK/base"
tar -xzf "$WORK/base/$BASE_PACKAGE.tar.gz" -C "$WORK"
mv "$WORK/$BASE_PACKAGE" "$WORK/$ARCH_PACKAGE"
cp "$ROOT/brake_test/portable/AutoKart Brake Test.arch.desktop" \
    "$WORK/$ARCH_PACKAGE/AutoKart Brake Test.desktop"
chmod +x "$WORK/$ARCH_PACKAGE/AutoKart Brake Test.desktop" \
    "$WORK/$ARCH_PACKAGE/brake_test/portable/"*.sh

cp "$ROOT/brake_test/portable/ARCH_README.md" "$WORK/$ARCH_PACKAGE/README-ARCH.md"

ARCHIVE="$OUTPUT_DIR/$ARCH_PACKAGE.tar.gz"
tar -C "$WORK" -czf "$ARCHIVE" "$ARCH_PACKAGE"
(
    cd "$OUTPUT_DIR"
    sha256sum "$ARCH_PACKAGE.tar.gz" > "$ARCH_PACKAGE.tar.gz.sha256"
)
printf '%s\n' "Built: $ARCHIVE" "Checksum: $ARCHIVE.sha256"
