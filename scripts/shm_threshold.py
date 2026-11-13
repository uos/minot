#!/usr/bin/env python3
"""Where shared memory starts to pay off between QUIC peers on one host.

Runs the `transfer` example of mt_pubsub (release build) for a range of sample
sizes, once with every sample through shared memory and once with every sample
over QUIC on loopback, and prints delivery, latency and CPU per sample.

    cargo build --release -p mt_pubsub --example transfer
    scripts/shm_threshold.py [--seconds 6] [--repeat 3] [--qos besteffort reliable]
"""
import argparse, os, statistics, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
SIZES = [512, 1 << 10, 2 << 10, 4 << 10, 8 << 10, 16 << 10, 32 << 10, 64 << 10, 128 << 10, 256 << 10,
         512 << 10, 1 << 20, 2 << 20, 4 << 20]


def run(binary, size, hz, seconds, qos, shm):
    env = dict(os.environ, MINOT_PEER_TRANSPORT="quic", MINOT_LOCAL_ONLY="1")
    env.update({"MINOT_QUIC_SHM_THRESHOLD": "0"} if shm else {"MINOT_SHM_DISABLED": "1"})
    args = [str(size), str(hz), str(seconds), qos]
    sub = subprocess.Popen([binary, "sub", *args], env=env, stdout=subprocess.PIPE, text=True)
    pub = subprocess.Popen([binary, "pub", *args], env=env, stdout=subprocess.PIPE, text=True)
    pub.stdout.read()
    received, p50, p90 = (int(v) for v in sub.stdout.read().split()[:3])
    # Whole process lifetime, so it also works where /proc is missing (macOS).
    cpu = 0.0
    for child in (pub, sub):
        usage = os.wait4(child.pid, 0)[2]
        cpu += (usage.ru_utime + usage.ru_stime) * 1e6
    return received, p50, p90, cpu / max(received, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=os.path.join(HERE, "../target/release/examples/transfer"))
    ap.add_argument("--seconds", type=float, default=6)
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--qos", nargs="+", default=["besteffort", "reliable"])
    ap.add_argument("--mbyte-per-s", type=float, default=40, help="offered load cap")
    ap.add_argument("--max-hz", type=float, default=500)
    a = ap.parse_args()
    for qos in a.qos:
        print(f"\n{qos}: median of {a.repeat} runs, {a.seconds:.0f} s each; CPU is publisher + subscriber")
        print(f"{'bytes':>9} {'hz':>5} | {'QUIC recv':>9} {'p50 us':>7} {'p90 us':>7} {'cpu us':>7}"
              f" | {'shm recv':>9} {'p50 us':>7} {'p90 us':>7} {'cpu us':>7} | cpu shm/QUIC")
        for size in SIZES:
            hz = min(a.max_hz, a.mbyte_per_s * 1e6 / size)
            cells = []
            for shm in (False, True):
                runs = [run(a.bin, size, hz, a.seconds, qos, shm) for _ in range(a.repeat)]
                cells.append([statistics.median(column) for column in zip(*runs)])
            (qn, q50, q90, qc), (sn, s50, s90, sc) = cells
            print(f"{size:>9} {hz:>5.0f} | {qn:>9.0f} {q50:>7.0f} {q90:>7.0f} {qc:>7.0f}"
                  f" | {sn:>9.0f} {s50:>7.0f} {s90:>7.0f} {sc:>7.0f} | {sc / qc:>6.2f}")


if __name__ == "__main__":
    main()
