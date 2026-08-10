#!/usr/bin/env bash
# HW communication-check harness for:
#   - RobStride 04        (extended-ID CAN, robstride-cli)
#   - MyActuator RMD X4   (standard-ID CAN V3, myactuator-cli)
#
# Linux/SocketCAN. The Windows equivalent is scripts/hw_check.ps1.
#
# The two families can share one 1 Mbps bus (RobStride uses extended 29-bit
# IDs, MyActuator standard 11-bit IDs, and each driver filters out the other
# kind) or live on separate physical buses/channels — use --rs-interface /
# --mya-interface to override per family (e.g. a 2-channel adapter wiring
# RobStride to can0 and MyActuator to can1).
#
# Default flow is READ-ONLY (scan + status reads — nothing moves). Pass
# --motion to add a small motion phase; free the shafts first.
#
# Usage:
#   scripts/hw_check.sh [options]
#     -i, --interface IF   default SocketCAN interface for both families (default: can0)
#     --rs-interface IF    RobStride-only override (default: -i/--interface)
#     --mya-interface IF   MyActuator-only override (default: -i/--interface)
#     --rs-id N            RobStride motor CAN ID     (default: 1)
#     --rs-model M         RobStride model            (default: rs04)
#     --mya-id N           MyActuator motor ID 1..32  (default: 1)
#     --kt KT              MyActuator Kt N·m/A (0 = current-units) (default: 0)
#     --timeout-ms MS      per-request timeout        (default: 200)
#     --setup              bring up the interface(s): sudo ip link ... bitrate 1M txqueuelen 1000
#     --motion             include the motion phase (small moves, then stop)
#     --yes                skip the motion confirmation prompt
#     --skip-rs / --skip-mya   test only one family
#
# Examples:
#   scripts/hw_check.sh --setup                                   # single bus, first run, read-only
#   scripts/hw_check.sh --rs-interface can0 --mya-interface can1  # 2-channel adapter, separate buses
#   scripts/hw_check.sh --motion                                  # add small test motions

set -u -o pipefail

IF="can0"
RS_IF=""
MYA_IF=""
RS_ID=1
RS_MODEL="rs04"
MYA_ID=1
KT=0
TIMEOUT_MS=200
BITRATE=1000000
TXQUEUELEN=1000
SETUP=0
MOTION=0
YES=0
SKIP_RS=0
SKIP_MYA=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--interface)  IF="$2"; shift 2 ;;
        --rs-interface)  RS_IF="$2"; shift 2 ;;
        --mya-interface) MYA_IF="$2"; shift 2 ;;
        --rs-id)         RS_ID="$2"; shift 2 ;;
        --rs-model)      RS_MODEL="$2"; shift 2 ;;
        --mya-id)        MYA_ID="$2"; shift 2 ;;
        --kt)            KT="$2"; shift 2 ;;
        --timeout-ms)    TIMEOUT_MS="$2"; shift 2 ;;
        --setup)         SETUP=1; shift ;;
        --motion)        MOTION=1; shift ;;
        --yes)           YES=1; shift ;;
        --skip-rs)       SKIP_RS=1; shift ;;
        --skip-mya)      SKIP_MYA=1; shift ;;
        -h|--help)       sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1 (see --help)"; exit 2 ;;
    esac
done

# Per-family interface defaults to the shared -i/--interface unless overridden.
RS_IF="${RS_IF:-$IF}"
MYA_IF="${MYA_IF:-$IF}"

# Distinct physical interfaces used across both families (dedup for setup/up-check).
declare -A _seen=()
IFACES=()
for f in "$RS_IF" "$MYA_IF"; do
    if [[ -z "${_seen[$f]:-}" ]]; then IFACES+=("$f"); _seen[$f]=1; fi
done

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RS_CLI="$ROOT/target/debug/robstride-cli"
MYA_CLI="$ROOT/target/debug/myactuator-cli"
LOG_DIR="$ROOT/hw_check_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/hw_check_$(date +%Y%m%d_%H%M%S).log"

BOLD=$'\e[1m'; RED=$'\e[31m'; GREEN=$'\e[32m'; YELLOW=$'\e[33m'; DIM=$'\e[2m'; RST=$'\e[0m'
declare -a SUMMARY
FAILURES=0

# run_step <PASS-mode> <label> <cmd...>
#   PASS-mode "exit"        → PASS on exit code 0
#   PASS-mode "grep:REGEX"  → PASS on exit 0 AND output matching REGEX
#   PASS-mode "warn"        → like "exit" but failure records WARN, not FAIL
run_step() {
    local mode="$1" label="$2"; shift 2
    echo "${BOLD}── ${label}${RST}"
    echo "── ${label}: $*" >>"$LOG"
    local out rc
    out="$("$@" 2>&1)"; rc=$?
    printf '%s\n' "$out" >>"$LOG"
    printf '%s\n' "$out" | sed "s/^/${DIM}   /; s/\$/${RST}/" | head -15

    local verdict="PASS"
    if [[ $rc -ne 0 ]]; then
        verdict="FAIL"
    elif [[ "$mode" == grep:* ]] && ! printf '%s\n' "$out" | grep -qE "${mode#grep:}"; then
        verdict="FAIL"
    fi
    if [[ "$verdict" == FAIL && "$mode" == warn ]]; then verdict="WARN"; fi

    case "$verdict" in
        PASS) echo "   ${GREEN}✔ PASS${RST}" ;;
        WARN) echo "   ${YELLOW}▲ WARN (optional feature — see log)${RST}" ;;
        FAIL) echo "   ${RED}✘ FAIL (rc=$rc)${RST}"; FAILURES=$((FAILURES + 1)) ;;
    esac
    SUMMARY+=("$verdict  $label")
    echo
}

skip_step() {
    SUMMARY+=("SKIP  $1")
    echo "${DIM}── $1: skipped${RST}"; echo
}

echo "${BOLD}=== misa-actuator HW check ===${RST}"
echo "robstride: if=$RS_IF id=$RS_ID model=$RS_MODEL  myactuator: if=$MYA_IF id=$MYA_ID kt=$KT"
echo "log: $LOG"
echo

# ---- 0) build the CLIs ----------------------------------------------------
run_step exit "build robstride-cli / myactuator-cli" \
    cargo build --manifest-path "$ROOT/Cargo.toml" -p robstride-cli -p myactuator-cli

# ---- 1) CAN interface(s) ---------------------------------------------------
if [[ $SETUP -eq 1 ]]; then
    for f in "${IFACES[@]}"; do
        run_step exit "bring up $f @ ${BITRATE} bps, txqueuelen ${TXQUEUELEN} (sudo)" \
            sudo ip link set "$f" type can bitrate "$BITRATE" up txqueuelen "$TXQUEUELEN"
    done
fi
for f in "${IFACES[@]}"; do
    run_step grep:"state UP" "interface $f is UP" ip -details link show "$f"
done

# ---- 2) RobStride (read-only) --------------------------------------------
if [[ $SKIP_RS -eq 0 ]]; then
    run_step grep:"found [1-9]" "robstride: probe id $RS_ID (device-id scan) on $RS_IF" \
        "$RS_CLI" -i "$RS_IF" -m "$RS_ID" --model "$RS_MODEL" scan --from "$RS_ID" --to "$RS_ID"
    run_step exit "robstride: parameter snapshot (pos/vel/Iq/Vbus)" \
        "$RS_CLI" -i "$RS_IF" -m "$RS_ID" --model "$RS_MODEL" info
else
    skip_step "robstride phase"
fi

# ---- 3) MyActuator (read-only) -------------------------------------------
if [[ $SKIP_MYA -eq 0 ]]; then
    run_step grep:"motor id [0-9]+.*responded" "myactuator: probe id $MYA_ID (0x9A scan) on $MYA_IF" \
        "$MYA_CLI" -i "$MYA_IF" -m "$MYA_ID" --timeout-ms "$TIMEOUT_MS" scan --from "$MYA_ID" --to "$MYA_ID"
    run_step exit "myactuator: status (0x9A voltage/errors + 0x92/0x9C state)" \
        "$MYA_CLI" -i "$MYA_IF" -m "$MYA_ID" --timeout-ms "$TIMEOUT_MS" status
    run_step exit "myactuator: multi-turn angle (0x92)" \
        "$MYA_CLI" -i "$MYA_IF" -m "$MYA_ID" --timeout-ms "$TIMEOUT_MS" angle
else
    skip_step "myactuator phase"
fi

# ---- 4) motion phase (opt-in) --------------------------------------------
if [[ $MOTION -eq 1 ]]; then
    echo "${YELLOW}${BOLD}Motion phase: the shafts WILL move (±0.2–0.3 rad, ±0.5 rad/s).${RST}"
    echo "${YELLOW}Make sure both motors can rotate freely.${RST}"
    if [[ $YES -eq 0 ]]; then
        read -r -p "proceed? [y/N] " ans
        [[ "$ans" == y || "$ans" == Y ]] || { echo "motion phase aborted."; MOTION=0; }
    fi
fi
if [[ $MOTION -eq 1 ]]; then
    if [[ $SKIP_RS -eq 0 ]]; then
        # Exercises position / velocity / torque / MIT in sequence and always
        # leaves the motor disabled. Small offsets for the 120 N·m RS-04.
        run_step exit "robstride: smoke-test (pos/vel/torque/MIT)" \
            "$RS_CLI" -i "$RS_IF" -m "$RS_ID" --model "$RS_MODEL" smoke-test \
                --pos-offset 0.2 --pos-speed 1.0 --vel 0.5 --torque 0.3 \
                --mit-kp 10 --mit-kd 0.5 --duration 0.8
    else
        skip_step "robstride motion"
    fi
    if [[ $SKIP_MYA -eq 0 ]]; then
        KT_ARGS=(); [[ "$KT" != 0 ]] && KT_ARGS=(--kt "$KT")
        run_step exit "myactuator: position move (0xA4, ±0.2 rad)" \
            "$MYA_CLI" -i "$MYA_IF" -m "$MYA_ID" --timeout-ms "$TIMEOUT_MS" "${KT_ARGS[@]}" \
                move-to 0.2 --speed 1.0 --duration 2
        run_step exit "myactuator: velocity (0xA2, 0.5 rad/s)" \
            "$MYA_CLI" -i "$MYA_IF" -m "$MYA_ID" --timeout-ms "$TIMEOUT_MS" "${KT_ARGS[@]}" \
                spin 0.5 --duration 2
        # Motion (MIT) mode needs RMD-X V3 firmware; a timeout here is a
        # firmware capability gap, not a wiring failure → WARN. The reply
        # decode bug (constant velocity with unchanging position) was root
        # caused against the official V4.3 manual and fixed in
        # myactuator_protocol::motion — re-run this step to confirm the
        # feedback now tracks real motion.
        run_step warn "myactuator: motion-mode MIT hold (0x400 channel)" \
            "$MYA_CLI" -i "$MYA_IF" -m "$MYA_ID" --timeout-ms "$TIMEOUT_MS" "${KT_ARGS[@]}" \
                mit --kp 5 --kd 0.5 --duration 2
    else
        skip_step "myactuator motion"
    fi
else
    skip_step "motion phase (pass --motion to enable)"
fi

# ---- summary --------------------------------------------------------------
echo "${BOLD}=== summary ===${RST}"
for line in "${SUMMARY[@]}"; do
    case "$line" in
        PASS*) echo "  ${GREEN}${line}${RST}" ;;
        WARN*) echo "  ${YELLOW}${line}${RST}" ;;
        FAIL*) echo "  ${RED}${line}${RST}" ;;
        *)     echo "  ${DIM}${line}${RST}" ;;
    esac
done
echo
echo "full log: $LOG"
if [[ $FAILURES -gt 0 ]]; then
    echo "${RED}${BOLD}$FAILURES step(s) failed.${RST} Common causes:"
    echo "  - interface down / wrong bitrate  → rerun with --setup (1 Mbps)"
    echo "  - wrong motor id                  → widen the probe: robstride-cli scan / myactuator-cli scan"
    echo "  - termination / wiring            → check 120 Ω at both ends, candump <interface>"
    exit 1
fi
echo "${GREEN}${BOLD}all good.${RST}"
