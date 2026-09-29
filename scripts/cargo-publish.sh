#!/usr/bin/env bash
# Publishes the workspace crates to crates.io in dependency order.

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
