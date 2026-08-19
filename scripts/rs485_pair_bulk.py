#!/usr/bin/env python3
"""Push bulk traffic across an RS485 pair and report the first byte that differs.

Linux counterpart of `scripts/win/rs485_pair_bulk.ps1`. Use it after
`rs485_pair_probe.py` has told you which ports face each other: the probe's tag
is 16-odd bytes, which proves the wiring and nothing about sustained traffic at
1 or 2 Mbit/s — the rates the LK motors actually run at. This sends a
non-repeating pattern in both directions and reports the first mismatching
offset, so a marginal link shows up as a byte position rather than as a vague
"sometimes fails".

Reading happens on a thread while the write is still going out. Without that
the receiver's buffer overruns partway through and every run looks like a fault
at whatever offset the buffer happened to hold.

Ports have to be free, and the motors should be powered down — same reasons as
the probe.

Examples
--------
    scripts/rs485_pair_bulk.py --pairs /dev/ttyCH9344USB2,/dev/ttyCH9344USB3
    scripts/rs485_pair_bulk.py --pairs U2,U3 --baud 2000000 --bytes 65536

`U2` is shorthand for `/dev/ttyCH9344USB2`. Separate the two ports of a pair
with a comma and successive pairs with a semicolon.

Bench result 2026-08-19 (CH9344): USB2/USB3 carried 65536 bytes per direction
intact at both 1 and 2 Mbit/s, at 94-98% of line rate.
"""

import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


def expand(name):
    """`U2` -> `/dev/ttyCH9344USB2`; anything else is taken as written."""
    name = name.strip()
    if name.upper().startswith("U") and name[1:].isdigit():
        return f"/dev/ttyCH9344USB{name[1:]}"
    return name


def pattern(n):
    """Bytes that do not repeat over the test length.

    A constant or a short cycle would let a link drop a whole run of bytes and
    still compare equal, which is exactly the failure this test exists to
    catch.
    """
    out = bytearray(n)
    x = 0x1234
    for i in range(n):
        x = (x * 1103515245 + 12345) & 0xFFFF
        out[i] = x >> 8
    return bytes(out)


def one_way(tx_dev, rx_dev, baud, data, drain_s=2.0):
    tx = serial.Serial(tx_dev, baud, timeout=0.2, write_timeout=10)
    rx = serial.Serial(rx_dev, baud, timeout=0.2)
    rx.reset_input_buffer()

    got = bytearray()
    stop = threading.Event()

    def reader():
        while not stop.is_set() and len(got) < len(data):
            chunk = rx.read(8192)
            if chunk:
                got.extend(chunk)

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    t0 = time.time()
    tx.write(data)
    tx.flush()
    deadline = time.time() + drain_s
    while len(got) < len(data) and time.time() < deadline:
        time.sleep(0.01)
    elapsed = time.time() - t0
    stop.set()
    th.join(timeout=0.5)
    tx.close()
    rx.close()

    received = bytes(got)
    first = None
    for i in range(min(len(received), len(data))):
        if received[i] != data[i]:
            first = i
            break
    # Short but otherwise correct means bytes went missing off the end, which
    # is still a failure — report it at the offset where they stopped.
    if first is None and len(received) < len(data):
        first = len(received)
    mbit = len(received) * 8 / elapsed / 1e6 if elapsed > 0 else 0.0
    return received, first, elapsed, mbit


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pairs", required=True,
                    help='pairs as "A,B" separated by ";" — e.g. "U2,U3;U4,U5"')
    # Defaults to the LK motors' rate: the point is to test what production uses.
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--bytes", type=int, default=8192)
    args = ap.parse_args()

    pairs = []
    for chunk in args.pairs.split(";"):
        parts = [expand(p) for p in chunk.split(",") if p.strip()]
        if len(parts) != 2:
            sys.exit(f'bad pair {chunk!r} — expected "A,B"')
        pairs.append(tuple(parts))

    data = pattern(args.bytes)
    print(f"RS485 bulk test: {args.bytes} bytes per direction @ {args.baud} baud 8N1")
    print("(non-repeating pattern; a dropped run cannot compare equal)\n")

    failures = 0
    for a, b in pairs:
        for tx_dev, rx_dev in ((a, b), (b, a)):
            try:
                received, first, elapsed, mbit = one_way(tx_dev, rx_dev, args.baud, data)
            except Exception as e:
                print(f"  {tx_dev} -> {rx_dev}: ERROR {e}")
                failures += 1
                continue
            label = f"  {tx_dev} -> {rx_dev}:"
            if first is None and len(received) == len(data):
                print(f"{label} OK  all {len(received)} bytes intact"
                      f"   {elapsed:.2f}s  {mbit:.2f} Mbit/s")
            else:
                failures += 1
                print(f"{label} FAIL  first mismatch at offset {first}"
                      f"  (received {len(received)}/{len(data)})"
                      f"   {elapsed:.2f}s  {mbit:.2f} Mbit/s")
                if received and first is not None:
                    lo = max(0, first - 4)
                    print(f"      expected: {data[lo:first + 8].hex(' ')}")
                    print(f"      got     : {received[lo:first + 8].hex(' ')}")

    print()
    if failures:
        print(f"{failures} direction(s) failed")
        sys.exit(1)
    print("all directions intact")


if __name__ == "__main__":
    main()
