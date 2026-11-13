#!/usr/bin/env bash
# Measure how peer pub/sub recovers from link outages and address changes,
# on the Zenoh and on the QUIC peer transport.
#
# Runs a publisher and a subscriber (mt_pubsub example `blackout`) in two
# network namespaces joined by a veth pair, inside an unprivileged user
# namespace: no root needed. Usage, from the workspace root:
#
#   scripts/peer_blackout.sh [output directory]
#
# CUTS="2 5 15 40" sets the outage lengths in seconds, SETTLE the pause after
# each event, TRANSPORTS="zenoh quic" what to compare.
set -euo pipefail
OUT=${1:-target/blackout}
BIN=target/debug/examples/blackout
if [ -z "${BLACKOUT_IN_NS:-}" ]; then
    cargo build -p mt_pubsub --example blackout
    mkdir -p "$OUT"
    exec env BLACKOUT_IN_NS=1 unshare -Urnm "$0" "$@"
fi
CUTS=${CUTS:-2 5 15 40}
SETTLE=${SETTLE:-15}

mount -t tmpfs tmpfs /run
ip netns add a
ip netns add b
ip link add a0 netns a type veth peer name b0 netns b
address() { # namespace address
    ip -n "$1" addr flush dev "${1}0"
    ip -n "$1" addr add "$2/24" dev "${1}0"
    ip -n "$1" route replace 224.0.0.0/4 dev "${1}0"
}
link() { # netem arguments
    for ns in a b; do tc -n "$ns" qdisc replace dev "${ns}0" root netem "$@"; done
}
for ns in a b; do
    ip -n "$ns" link set lo up
    ip -n "$ns" link set "${ns}0" up
done
LINK=${LINK:-delay 2ms}
link $LINK

run() {
    local transport=$1 events="$OUT/$1.events"
    : > "$events"
    event() { echo "$(date +%s%3N) $*" >> "$events"; }
    address a 10.77.0.1
    address b 10.77.0.2
    # The publisher has the lower identifier, so on QUIC it dials.
    MINOT_PEER_TRANSPORT=$transport MINOT_QUIC_ID=2 MINOT_SHM_DISABLED=1 \
        nsenter --net=/run/netns/b "$BIN" sub > "$OUT/$transport.log" 2> "$OUT/$transport.sub.err" &
    local sub=$!
    MINOT_PEER_TRANSPORT=$transport MINOT_QUIC_ID=1 MINOT_SHM_DISABLED=1 \
        nsenter --net=/run/netns/a "$BIN" pub 2> "$OUT/$transport.pub.err" &
    local pub=$!
    sleep 5
    for seconds in $CUTS; do
        link loss 100%
        sleep "$seconds"
        link $LINK
        event "outage ${seconds}s"
        sleep "$SETTLE"
    done
    event "publisher address change"
    address a 10.77.0.11
    sleep "$SETTLE"
    event "subscriber address change"
    address b 10.77.0.12
    sleep "$SETTLE"
    event end
    kill "$pub" "$sub" 2> /dev/null || true
    wait "$pub" "$sub" 2> /dev/null || true
}

for transport in ${TRANSPORTS:-zenoh quic}; do
    echo "running $transport ..."
    run "$transport"
done

python3 - "$OUT" ${TRANSPORTS:-zenoh quic} <<'PY'
import sys
out, transports = sys.argv[1], sys.argv[2:]
topics = ["small", "large", "try"]
print("\nMilliseconds from the end of the event to the next sample ('-': none came).")
print("small: 64 B best effort, large: 500 kB best effort, try: 64 B TryReliable.\n")
print(f"{'event':<28}{'transport':<10}" + "".join(f"{t:>9}" for t in topics))
rows = {}
for transport in transports:
    arrivals = {t: [] for t in topics}
    for line in open(f"{out}/{transport}.log"):
        parts = line.split()
        if len(parts) == 4 and parts[1] in arrivals and parts[2] != "closed":
            arrivals[parts[1]].append(int(parts[0]))
    events = [line.split(None, 1) for line in open(f"{out}/{transport}.events")]
    events = [(int(t), name.strip()) for t, name in events]
    for (start, name), (end, _) in zip(events, events[1:]):
        cells = []
        for topic in topics:
            after = [t for t in arrivals[topic] if start <= t < end]
            if "address" in name:
                # No outage to wait out: report the longest silence instead.
                times = [start] + after + [end]
                cells.append(max(b - a for a, b in zip(times, times[1:])) if after else None)
            else:
                cells.append(after[0] - start if after else None)
        rows.setdefault(name, []).append((transport, cells))
for name, entries in rows.items():
    for transport, cells in entries:
        print(f"{name:<28}{transport:<10}" + "".join(f"{'-' if c is None else c:>9}" for c in cells))
print("\nAddress changes show the longest silence within the settle time.")
PY
