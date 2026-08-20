#!/usr/bin/env python3
"""Find LK Motor units when neither the baud rate nor the id is known.

`lkmotor-cli scan` needs `--baud` to be right already. That is fine once a
bench is known, and useless when a motor has just been plugged in and does not
answer: "no motors responded" reads exactly the same whether the id is wrong,
the rate is wrong, or the wiring is dead. This walks the whole grid instead —
every rate against every id — and prints what answered.

That case is not hypothetical. A unit set to 2 Mbit/s cost an afternoon,
because every sweep run against it stopped at 1 Mbit/s and reported silence.

Retries matter as much as coverage. On a marginal link a lost frame is
indistinguishable from "nothing here", so each id is asked several times and
partial hits are reported as such — `3/5` says the motor is there *and* that
the link is dropping frames, which one attempt could never separate.

Side-effect-free: sends only `0x9A` state reads.

Examples
--------
    scripts/rs485_scan_sweep.py /dev/ttyCH9344USB0
    scripts/rs485_scan_sweep.py /dev/ttyCH9344USB0 10 32   # stubborn, ids 1-32
"""

import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

# Every rate an LK V3 board is documented to take (setting `0x0C`, 0=9600 ...
# 10=4Mbps), minus the ones too slow to be worth the wall-clock.
BAUDS = [115200, 250000, 500000, 921600, 1000000,
         1500000, 2000000, 2500000, 3000000, 4000000]


def frame(cmd, motor_id):
    head = [0x3E, cmd, motor_id, 0x00]
    head.append(sum(head) & 0xFF)
    return bytes(head)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__.strip().splitlines()[-1].strip())
    port = sys.argv[1]
    retries = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    max_id = int(sys.argv[3]) if len(sys.argv) > 3 else 8

    print(f"sweeping {port}: ids 1..{max_id}, {retries} tries each, "
          f"{len(BAUDS)} bauds\n")
    found = []
    for baud in BAUDS:
        try:
            sp = serial.Serial(port, baud, timeout=0.12)
        except OSError as e:
            print(f"{baud:>8}: open failed: {e}")
            continue
        row = []
        for motor_id in range(1, max_id + 1):
            hits = 0
            for _ in range(retries):
                sp.reset_input_buffer()
                sp.write(frame(0x9A, motor_id))
                sp.flush()
                time.sleep(0.03)
                if sp.read(64):
                    hits += 1
            if hits:
                row.append(f"id{motor_id}:{hits}/{retries}")
                found.append((baud, motor_id, hits, retries))
        sp.close()
        print(f"{baud:>8}: {'  '.join(row) if row else '-'}")

    print()
    if not found:
        print("nothing responded on any baud/id")
        return
    print("== responded ==")
    for baud, motor_id, hits, tries in found:
        rate = 100 * hits // tries
        flag = "" if hits == tries else "   <-- INTERMITTENT"
        print(f"  baud {baud}  id {motor_id}   {hits}/{tries} ({rate}%){flag}")


if __name__ == "__main__":
    main()
