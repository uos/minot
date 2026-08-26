#!/usr/bin/env bash
# Automatically prepares manifests by stripping unpublished git dependencies (hiroz).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT_DIR}"

DRY_RUN=0
ALLOW_DIRTY="--allow-dirty"
EXTRA_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--dry-run" ]; then
        DRY_RUN=1
        EXTRA_ARGS+=("--dry-run")
    else
        EXTRA_ARGS+=("$arg")
    fi
done

echo "==> Preparing workspace manifests for crates.io..."

# Ensure we restore Cargo.toml files upon exit/interruption
cleanup() {
    echo "==> Restoring Cargo.toml files..."
    git checkout Cargo.toml mt_wind/Cargo.toml minot/Cargo.toml 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Strip unpublished git dependencies (hiroz) from Cargo manifests
python3 - <<'EOF'
import re

def strip_hiroz(content):
    # Strip multiline hiroz/hiroz-msgs dependencies
    content = re.sub(r"^\s*hiroz(?:-msgs)?\s*=\s*\{[^}]*\}\s*\n?", "", content, flags=re.MULTILINE)
    content = re.sub(r"^\s*hiroz(?:-msgs)?\s*=.*$\n?", "", content, flags=re.MULTILINE)
    # Strip hiroz features
    content = re.sub(r"^\s*default-hiroz\s*=.*$\n?", "", content, flags=re.MULTILINE)
    content = re.sub(r"^\s*embed-hiroz.*$\n?", "", content, flags=re.MULTILINE)
    content = re.sub(r"^\s*hiroz\s*=\s*\[[^\]]*\]\s*\n?", "", content, flags=re.MULTILINE)
    content = re.sub(r"^\s*\"embed-hiroz\",?\n?", "", content, flags=re.MULTILINE)
    # Strip [[bin]] wind-hiroz section
    content = re.sub(r"\[\[bin\]\]\s*\nname\s*=\s*\"wind-hiroz\"[^\n]*\npath\s*=\s*\"[^\"]*\"\nrequired-features\s*=\s*\[\"hiroz\"\]\n?", "", content, flags=re.MULTILINE)
    return content

for path in ["mt_wind/Cargo.toml", "minot/Cargo.toml", "Cargo.toml"]:
    with open(path, "r") as f:
        c = f.read()
    c = strip_hiroz(c)
    with open(path, "w") as f:
        f.write(c)
EOF

echo "==> Manifests stripped of unpublished git dependencies."

# Topological publishing order
PACKAGES=(
    "mt_log"
    "mt_net"
    "mt_mtc"
    "mt_sea"
    "mt_scope"
    "mt_bagread"
    "mt_coord"
    "mt_pubsub"
    "mt_service"
    "mt_flow"
    "mt_dataset"
    "mt_action"
    "mt_wind"
    "mt_rat"
    "minot"
)

echo "==> Publishing ${#PACKAGES[@]} crates in order:"
for pkg in "${PACKAGES[@]}"; do
    echo "    - $pkg"
done
echo ""

for pkg in "${PACKAGES[@]}"; do
    echo "=================================================================="
    echo "==> Publishing $pkg..."
    echo "=================================================================="

    CMD=(cargo publish -p "$pkg" $ALLOW_DIRTY "${EXTRA_ARGS[@]}")
    echo "Running: ${CMD[*]}"

    # Execute publish and capture output to handle already published crates
    set +e
    output=$("${CMD[@]}" 2>&1)
    status=$?
    set -e

    echo "$output"

    if [ $status -ne 0 ]; then
        if echo "$output" | grep -iqE "already exists|already uploaded|already published"; then
            echo "--> Package $pkg is already published at this version. Continuing."
        else
            echo "ERROR: Failed to publish $pkg"
            exit $status
        fi
    else
        echo "==> Successfully published $pkg"
        # When actually publishing, wait for the sparse/registry index to update before downstream crates compile against it
        if [ $DRY_RUN -eq 0 ]; then
            echo "--> Waiting 20s for crates.io index to update..."
            sleep 20
        fi
    fi
    echo ""
done

echo "=================================================================="
echo "==> All ${#PACKAGES[@]} crates published successfully!"
echo "=================================================================="
