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

if sed --version 2>/dev/null | grep -q GNU; then
    SED_INPLACE=(-i)
else
    SED_INPLACE=(-i '')
fi

CURRENT_VERSION="$(grep -m1 '^version = ' Cargo.toml | sed 's/version = "\(.*\)"/\1/')"
echo "Bumping: $CURRENT_VERSION -> $NEW_VERSION"

OLD_MINOR="$(echo "$CURRENT_VERSION" | cut -d. -f1-2)"
NEW_MINOR="$(echo "$NEW_VERSION" | cut -d. -f1-2)"
IS_BREAKING=false
if [[ "$OLD_MINOR" != "$NEW_MINOR" ]]; then
    IS_BREAKING=true
    echo "Breaking change detected (${OLD_MINOR}.x -> ${NEW_MINOR}.x): updating vscode minot engine dependency"
fi

sed "${SED_INPLACE[@]}" "s/^version = \"${CURRENT_VERSION}\"/version = \"${NEW_VERSION}\"/" Cargo.toml
sed "${SED_INPLACE[@]}" -E "/path = \"[^\"]+\"/s/version = \"${CURRENT_VERSION}\"/version = \"${NEW_VERSION}\"/" Cargo.toml
sed "${SED_INPLACE[@]}" "s|<version>${CURRENT_VERSION}</version>|<version>${NEW_VERSION}</version>|" package.xml
sed "${SED_INPLACE[@]}" "s/^pkgver=${CURRENT_VERSION}/pkgver=${NEW_VERSION}/" packaging/arch/PKGBUILD
sed "${SED_INPLACE[@]}" "s/^pkgver=${CURRENT_VERSION}/pkgver=${NEW_VERSION}/" packaging/arch-bin/PKGBUILD

VSCODE_PKG="vscode/package.json"
sed "${SED_INPLACE[@]}" "s/\"version\": \"${CURRENT_VERSION}\"/\"version\": \"${NEW_VERSION}\"/" "$VSCODE_PKG"
if [[ "$IS_BREAKING" == "true" ]]; then
    sed "${SED_INPLACE[@]}" "s/\"minot\": \"${CURRENT_VERSION}\"/\"minot\": \"${NEW_VERSION}\"/" "$VSCODE_PKG"
fi

echo ""
echo "── git diff ──────────────────────────────────────────────────────────"
git diff
