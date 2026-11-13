#!/usr/bin/env bash
# Run the peer-mode pub/sub tests of both transports repeatedly: their failures
# so far were races that show up in one run out of several.
#
#   scripts/test_pubsub_repeat.sh [repetitions, default 20]
set -euo pipefail
cd "$(dirname "$0")/.."
N=${1:-20}

cargo test -p mt_pubsub --lib
cargo test -p mt_pubsub --test quic --test peer --no-run

repeat() {
    local name=$1
    shift
    for i in $(seq 1 "$N"); do
        if ! "$@" >/tmp/pubsub_repeat.log 2>&1; then
            echo "$name: run $i of $N failed"
            cat /tmp/pubsub_repeat.log
            exit 1
        fi
    done
    echo "$name: $N of $N runs passed"
}

repeat "quic, plain" cargo test -p mt_pubsub --test quic
MINOT_QUIC_ENCRYPT=1 repeat "quic, encrypted" cargo test -p mt_pubsub --test quic
MINOT_SHM_DISABLED=1 repeat "quic, no shared memory" cargo test -p mt_pubsub --test quic -- --skip shared_memory
repeat "zenoh" cargo test -p mt_pubsub --test peer
