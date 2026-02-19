#!/usr/bin/env bash
# Usage: ./scripts/bump-version.sh <new-version>
# Example: ./scripts/bump-version.sh 0.6.0

set -euo pipefail

NEW_VERSION="${1:-}"
if [[ -z "$NEW_VERSION" ]]; then
    echo "Usage: $0 <new-version>"
    echo "Example: $0 0.6.0"
    exit 1
fi

if ! [[ "$NEW_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[a-zA-Z0-9._-]+)?(\+[a-zA-Z0-9._-]+)?$ ]]; then
    echo "Error: version must be semver format (e.g. 1.2.3 or 1.2.3-alpha.1)"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

CURRENT_VERSION="$(grep -m1 '^version = ' Cargo.toml | sed 's/version = "\(.*\)"/\1/')"
echo "Bumping: $CURRENT_VERSION -> $NEW_VERSION"

sed -i "s/^version = \"${CURRENT_VERSION}\"/version = \"${NEW_VERSION}\"/" Cargo.toml
sed -i -E "/path = \"[^\"]+\"/s/version = \"${CURRENT_VERSION}\"/version = \"${NEW_VERSION}\"/" Cargo.toml
sed -i "s|<version>${CURRENT_VERSION}</version>|<version>${NEW_VERSION}</version>|" package.xml
sed -i "s/^pkgver=${CURRENT_VERSION}/pkgver=${NEW_VERSION}/" packaging/arch/PKGBUILD
sed -i "s/^pkgver=${CURRENT_VERSION}/pkgver=${NEW_VERSION}/" packaging/arch-bin/PKGBUILD

echo ""
echo "── git diff ──────────────────────────────────────────────────────────"
git diff
