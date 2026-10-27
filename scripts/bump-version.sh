#!/usr/bin/env bash
# Usage: ./scripts/bump-version.sh [<new-version>]
# Example: ./scripts/bump-version.sh 0.6.0
#
# Without a version, scripts/semver_check.py proposes the smallest one the
# changes allow. A given version must be at least that. Set
# SKIP_SEMVER_CHECK=1 to skip the check.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

NEW_VERSION="${1:-}"
if [[ -n "$NEW_VERSION" && ! "$NEW_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[a-zA-Z0-9._-]+)?(\+[a-zA-Z0-9._-]+)?$ ]]; then
    echo "Error: version must be semver format (e.g. 1.2.3 or 1.2.3-alpha.1)"
    exit 1
fi

if [[ "${SKIP_SEMVER_CHECK:-0}" == "1" ]]; then
    if [[ -z "$NEW_VERSION" ]]; then
        echo "Usage: $0 <new-version> (SKIP_SEMVER_CHECK needs a version)"
        exit 1
    fi
elif [[ -z "$NEW_VERSION" ]]; then
    NEW_VERSION="$(python3 scripts/semver_check.py)"
    read -r -p "Bump to $NEW_VERSION? [y/N] " answer
    [[ "$answer" =~ ^[Yy]$ ]] || exit 1
else
    python3 scripts/semver_check.py "$NEW_VERSION"
fi

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
# The AUR packages start a new release at pkgrel 1
for pkgbuild in packaging/arch/PKGBUILD packaging/arch-bin/PKGBUILD; do
    sed "${SED_INPLACE[@]}" -e "s/^pkgver=.*/pkgver=${NEW_VERSION}/" -e "s/^pkgrel=.*/pkgrel=1/" "$pkgbuild"
done

# The install snippet on the mt_pubsub crates.io page
sed "${SED_INPLACE[@]}" "s/mt_pubsub = { version = \"${CURRENT_VERSION}\"/mt_pubsub = { version = \"${NEW_VERSION}\"/" mt_pubsub/README.md

VSCODE_PKG="vscode/package.json"
sed "${SED_INPLACE[@]}" "s/\"version\": \"${CURRENT_VERSION}\"/\"version\": \"${NEW_VERSION}\"/" "$VSCODE_PKG"
# The extension accepts Minot from engines.minot up to the next breaking
# version, so a breaking release moves it along.
if [[ "$IS_BREAKING" == "true" ]]; then
    sed "${SED_INPLACE[@]}" -E "s/(\"minot\": \")[^\"]+/\1${NEW_VERSION}/" "$VSCODE_PKG"
fi

echo ""
echo "── Updating Cargo.lock ───────────────────────────────────────────────"
cargo update --workspace

echo ""
echo "── git diff ──────────────────────────────────────────────────────────"
git diff
