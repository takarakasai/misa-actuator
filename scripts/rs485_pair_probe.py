#!/usr/bin/env python3
"""Discover which serial ports are wired to each other over RS485.

Linux counterpart of `scripts/win/rs485_pair_probe.ps1`, same idea and the
same output shape: transmit a port-specific tag from one port at a time while
the rest listen, then print a receive matrix and the pairs confirmed in both
directions.

This answers a question `lkmotor-cli ports` cannot. That command lists the
device nodes the OS reports; on a multi-port adapter it cannot say which of
them face each other, and every port of a CH9344 reports the same VID:PID, so
there is nothing to group them by either.

Ports have to be free. Stop the GUI, any `lkmotor-cli` run and any terminal
program first — a port held elsewhere is reported and skipped, not waited on.

The payload is plain ASCII with no vendor header or checksum, so it cannot
decode into a valid command for any actuator sharing the bus (LK needs 0x3E
plus two checksums; DAMIAO/RobStride/MyActuator are CAN). Even so, power the
motors down first: nothing stops a motor midway through its own reply from
being confused by unsolicited traffic.

Exits non-zero if fewer than two ports open, so a harness can tell "nothing is
plugged in" from "wiring found".

Examples
--------
    scripts/rs485_pair_probe.py
    scripts/rs485_pair_probe.py --ports /dev/ttyCH9344USB2,/dev/ttyCH9344USB3
    scripts/rs485_pair_probe.py --baud 2000000 --settle-ms 400

Bench result 2026-08-19 (CH9344, 8 ports): USB2/USB3 confirmed in both
directions at 115200, 1 Mbit/s and 2 Mbit/s.
"""

import argparse
import glob
import sys

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


def default_ports():
    """Every CH9344 node, or failing that the usual USB-serial names."""
    found = sorted(glob.glob("/dev/ttyCH9344USB*"))
    return found or sorted(glob.glob("/dev/ttyUSB*"))


def short(name):
    """Trim a device path down to something that fits a matrix column."""
    return name.replace("/dev/tty", "").replace("CH9344USB", "U")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ports", default="",
                    help="comma-separated device list (default: every CH9344 node)")
    # 8N1 with no flow control is fixed — RS485 has no handshake lines.
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--settle-ms", type=int, default=250,
                    help="how long to let a tag cross the wire and land in the buffer")
    args = ap.parse_args()

    ports = [p.strip() for p in args.ports.split(",") if p.strip()] or default_ports()
    if not ports:
        sys.exit("no serial ports found")

    print(f"RS485 pair probe: {', '.join(ports)} @ {args.baud} baud 8N1\n")

    # A port that will not open is a finding to print and move past, not a
    # reason to abandon the ports that would have answered.
    open_ports = {}
    for name in ports:
        try:
            sp = serial.Serial(name, args.baud, timeout=args.settle_ms / 1000.0,
                               write_timeout=0.5)
            open_ports[name] = sp
            print(f"  {short(name):<8} open")
        except Exception as e:
            print(f"  {short(name):<8} CANNOT OPEN - {e}")

    if len(open_ports) < 2:
        for sp in open_ports.values():
            sp.close()
        sys.exit("\nNeed at least two open ports to probe a pair.")

    live = list(open_ports)
    print(f"\nProbing {len(live)} ports ({len(live)} rounds)...\n")

    heard = {}       # heard[tx][rx] = bytes that rx read during tx's round
    self_echo = {}

    for tx in live:
        tag = f"MISA-{short(tx)}-PROBE\n".encode()

        # Start each round with every buffer empty, so a receive can only be
        # attributed to this round's transmission.
        for p in live:
            open_ports[p].reset_input_buffer()
            open_ports[p].reset_output_buffer()

        try:
            open_ports[tx].write(tag)
            open_ports[tx].flush()
        except Exception as e:
            print(f"  {short(tx):<8} TX FAILED - {e}")
            heard[tx] = {}
            continue

        # One blocking read per receiver doubles as the settle delay: each
        # returns as soon as the tag lands, or after the timeout if it never
        # does.
        got = {}
        for rx in live:
            data = open_ports[rx].read(len(tag) * 2)
            if not data:
                continue
            if rx == tx:
                # Half-duplex transceivers with automatic direction control
                # often feed the transmitter its own bytes back. Says nothing
                # about who else is on the bus, so it is reported separately.
                self_echo[tx] = data
            else:
                got[rx] = data
        heard[tx] = got

        intact = [r for r, v in got.items() if tag.strip() in v]
        partial = [r for r in got if r not in intact]

        line = f"  TX {short(tx):<8} ->"
        if intact:
            print(f"{line} {', '.join(short(r) for r in intact)}")
        elif partial:
            print(f"{line} {', '.join(short(r) for r in partial)} (CORRUPT)")
        else:
            print(f"{line} nothing")
        # Corrupt bytes are worth showing raw: a baud mismatch, swapped A/B or
        # a missing terminator each leave a different signature.
        for r in partial:
            print(f"           {short(r)} got: {got[r][:32].hex(' ')}")

    print("\nReceive matrix (row = transmitter, column = receiver)")
    print("        " + "".join(f"{short(p):<7}" for p in live))
    for tx in live:
        row = f"{short(tx):<8}"
        for rx in live:
            if rx == tx:
                cell = "."
            elif rx in heard[tx]:
                cell = "OK" if f"MISA-{short(tx)}-PROBE".encode() in heard[tx][rx] else "~"
            else:
                cell = "-"
            row += f"{cell:<7}"
        print(row)
    print("  OK = tag received intact   ~ = bytes but corrupt   - = nothing")

    print("\nResult")
    paired = set()
    pairs = []
    for tx in live:
        if tx in paired:
            continue
        rx_ok = [r for r, v in heard[tx].items() if f"MISA-{short(tx)}-PROBE".encode() in v]
        # More than one receiver means a shared multi-drop bus, not a pair;
        # zero means nothing is listening. Neither is a pair, so leave it
        # unclaimed and let the matrix above tell the story.
        if len(rx_ok) != 1:
            continue
        partner = rx_ok[0]
        if partner in paired:
            continue
        # Only call it a pair if the partner reaches back. A one-way result
        # points at a transceiver stuck in receive, not at working wiring.
        back = heard.get(partner, {})
        if tx in back and f"MISA-{short(partner)}-PROBE".encode() in back[tx]:
            pairs.append((tx, partner))
            paired.update((tx, partner))

    if pairs:
        for a, b in pairs:
            print(f"  PAIR  {short(a)} <-> {short(b)}  (confirmed both directions)")
    else:
        print("  no pair confirmed in both directions")
    unpaired = [p for p in live if p not in paired]
    if unpaired:
        print(f"  unpaired: {', '.join(short(p) for p in unpaired)}")
    if self_echo:
        print(f"  self-echo seen on: {', '.join(short(p) for p in self_echo)}"
              f"   (normal for auto-direction transceivers)")

    for sp in open_ports.values():
        sp.close()


if __name__ == "__main__":
    main()
