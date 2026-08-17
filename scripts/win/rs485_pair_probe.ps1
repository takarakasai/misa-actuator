<#
.SYNOPSIS
    Discover which serial ports are wired to each other over RS485.

.DESCRIPTION
    Opens every listed port, then transmits a port-specific tag from one at a
    time while all the others listen. Whoever receives the tag intact is on the
    same two-wire bus as the transmitter. Prints a receive matrix and the pairs
    it could confirm in both directions.

    This answers a question `lkmotor-cli ports` cannot: that command lists the
    COM numbers the OS reports, but on a multi-port adapter it cannot say which
    of them face each other. Worse, the CH348 on the current bench comes back
    with a bare "Unknown" detail — the serialport crate does not classify it as
    a USB port on Windows, so there is not even a VID:PID to group them by.

    Ports have to be free. Close the GUI, any lkmotor-cli run, and any terminal
    program first; a port held elsewhere is reported and skipped.

    The payload is plain ASCII with no vendor header or checksum, so it cannot
    decode into a valid command for any actuator that happens to share the bus
    (LK needs 0x3E plus two checksums; DAMIAO/RobStride/MyActuator are CAN).
    Still, power the motors down first — nothing stops a motor that is midway
    through its own reply from being confused by unsolicited traffic.

    Exits non-zero if fewer than two ports open, so a harness can tell the
    "nothing is plugged in" case from "wiring found".

.EXAMPLE
    .\scripts\win\rs485_pair_probe.ps1
    Sweeps the CH348's COM12-COM19 at 115200.

.EXAMPLE
    .\scripts\win\rs485_pair_probe.ps1 -Ports COM12,COM13,COM14,COM15,COM16,COM19 -Baud 1000000
    Re-checks the known pairs at the rate the LK motors actually run.

.EXAMPLE
    .\scripts\win\rs485_pair_probe.ps1 -Ports COM5,COM6 -SettleMs 600
    Two ports, with a longer window for a slow adapter or a long cable.

.EXAMPLE
    .\scripts\win\rs485_pair_probe.cmd -Ports "COM12,COM13"
    Through the wrapper, which sidesteps the execution policy. Quote the list
    as one string there — see the note above the $Ports normalization.

.NOTES
    Bench result 2026-08-17 (CH348, 8 ports as COM12-COM19): the pairs are
    COM12/COM13, COM14/COM15 and COM16/COM19 — the third one breaks the
    adjacent numbering — with COM17 and COM18 wired to nothing. Assuming
    COM16/COM17 and COM18/COM19 tests a wire that does not exist and reads as
    a dead link. Verified both directions at 115200 and 1 Mbit/s.
#>

[CmdletBinding()]
param(
    # Ports to sweep. At least two, and every one gets a turn transmitting.
    [string[]]$Ports = @("COM12", "COM13", "COM14", "COM15", "COM16", "COM17", "COM18", "COM19"),
    # Line rate for the sweep. 8N1 with no flow control is fixed — RS485 has no
    # handshake lines to configure.
    [int]$Baud = 115200,
    # How long to let a tag cross the wire and land in the receiver's buffer.
    [int]$SettleMs = 250
)

# Deliberately not "Stop". A port that will not open is a finding to print and
# move past, not a reason to abandon the ports that would have answered.
$ErrorActionPreference = "Continue"

# `powershell.exe -File` passes every argument through as a literal string
# without re-parsing it, so the .cmd wrapper cannot hand over a real array:
# `-Ports COM12,COM13` arrives as the single string "COM12,COM13" and tries to
# open a port by that name. Splitting here makes both spellings work, so the
# wrapper and a direct PowerShell call take the same arguments.
$Ports = @($Ports |
    ForEach-Object { $_ -split "[,;]" } |
    ForEach-Object { $_.Trim() } |
    Where-Object { $_ })

Write-Host "RS485 pair probe: $($Ports -join ', ') @ $Baud baud 8N1" -ForegroundColor Cyan
Write-Host ""

# ---- open every port ----------------------------------------------------
$open = [ordered]@{}
foreach ($name in $Ports) {
    try {
        $sp = New-Object System.IO.Ports.SerialPort $name, $Baud, "None", 8, "One"
        $sp.Handshake = "None"
        $sp.ReadTimeout = 100
        $sp.WriteTimeout = 500
        $sp.Open()
        $sp.DiscardInBuffer()
        $sp.DiscardOutBuffer()
        $open[$name] = $sp
        Write-Host ("  {0,-8} open" -f $name) -ForegroundColor Green
    } catch {
        Write-Host ("  {0,-8} CANNOT OPEN - {1}" -f $name, $_.Exception.Message) -ForegroundColor Red
    }
}

if ($open.Count -lt 2) {
    Write-Host "`nNeed at least two open ports to probe a pair." -ForegroundColor Red
    foreach ($sp in $open.Values) { $sp.Close(); $sp.Dispose() }
    exit 1
}

$live = @($open.Keys)
Write-Host "`nProbing $($live.Count) ports ($($live.Count) rounds)...`n" -ForegroundColor Cyan

# ---- probe: one transmitter per round -----------------------------------
# $heard[$tx] maps each port that received this round's tag to what it read.
$heard = @{}
$selfEcho = @{}

foreach ($tx in $live) {
    $tag = "MISA-$tx-PROBE`n"

    # Start each round with every buffer empty, so a receive can only be
    # attributed to this round's transmission.
    foreach ($p in $live) { $open[$p].DiscardInBuffer(); $open[$p].DiscardOutBuffer() }

    try {
        $open[$tx].Write($tag)
    } catch {
        Write-Host ("  {0,-8} TX FAILED - {1}" -f $tx, $_.Exception.Message) -ForegroundColor Red
        $heard[$tx] = [ordered]@{}
        continue
    }

    Start-Sleep -Milliseconds $SettleMs

    $got = [ordered]@{}
    foreach ($rx in $live) {
        if ($open[$rx].BytesToRead -le 0) { continue }
        $text = $open[$rx].ReadExisting()
        if ($rx -eq $tx) {
            # Half-duplex transceivers with automatic direction control often
            # feed the transmitter its own bytes back. Says nothing about who
            # else is on the bus, so it is reported separately.
            $selfEcho[$tx] = $text
        } else {
            $got[$rx] = $text
        }
    }
    $heard[$tx] = $got

    $intact  = @($got.Keys | Where-Object { $got[$_] -like "*MISA-$tx-PROBE*" })
    $partial = @($got.Keys | Where-Object { $got[$_] -notlike "*MISA-$tx-PROBE*" })

    $line = "  TX {0,-8} ->" -f $tx
    if ($intact.Count -gt 0) {
        Write-Host ("{0} {1}" -f $line, ($intact -join ", ")) -ForegroundColor Green
    } elseif ($partial.Count -gt 0) {
        Write-Host ("{0} {1} (CORRUPT)" -f $line, ($partial -join ", ")) -ForegroundColor Yellow
    } else {
        Write-Host ("{0} nothing" -f $line) -ForegroundColor DarkGray
    }
    # Corrupt bytes are worth showing raw: a baud mismatch, swapped A/B or a
    # missing terminator each leave a different signature.
    foreach ($p in $partial) {
        $bytes = [System.Text.Encoding]::ASCII.GetBytes($got[$p])
        $hex = ($bytes | ForEach-Object { $_.ToString("X2") }) -join " "
        Write-Host ("           {0} got: {1}" -f $p, $hex) -ForegroundColor Yellow
    }
}

# ---- matrix -------------------------------------------------------------
Write-Host "`nReceive matrix (row = transmitter, column = receiver)" -ForegroundColor Cyan
$header = "        " + (($live | ForEach-Object { "{0,-7}" -f ($_ -replace "^COM", "C") }) -join "")
Write-Host $header
foreach ($tx in $live) {
    $row = "{0,-8}" -f $tx
    foreach ($rx in $live) {
        if ($rx -eq $tx) {
            $cell = "."
        } elseif ($heard[$tx].Contains($rx)) {
            if ($heard[$tx][$rx] -like "*MISA-$tx-PROBE*") { $cell = "OK" } else { $cell = "~" }
        } else {
            $cell = "-"
        }
        $row += "{0,-7}" -f $cell
    }
    Write-Host $row
}
Write-Host "  OK = tag received intact   ~ = bytes but corrupt   - = nothing" -ForegroundColor DarkGray

# ---- infer pairs --------------------------------------------------------
Write-Host "`nResult" -ForegroundColor Cyan
$paired = @{}
$pairs = @()
foreach ($tx in $live) {
    if ($paired.Contains($tx)) { continue }
    $rxOk = @($heard[$tx].Keys | Where-Object { $heard[$tx][$_] -like "*MISA-$tx-PROBE*" })
    # More than one receiver means a shared multi-drop bus, not a pair; zero
    # means nothing is listening. Neither is a pair, so leave it unclaimed and
    # let the matrix above tell the story.
    if ($rxOk.Count -ne 1) { continue }
    $partner = $rxOk[0]
    if ($paired.Contains($partner)) { continue }
    # Only call it a pair if the partner reaches back. A one-way result points
    # at a transceiver stuck in receive, not at working wiring.
    $back = @($heard[$partner].Keys | Where-Object { $heard[$partner][$_] -like "*MISA-$partner-PROBE*" })
    if ($back.Count -eq 1 -and $back[0] -eq $tx) {
        $pairs += , @($tx, $partner)
        $paired[$tx] = $true
        $paired[$partner] = $true
    }
}

if ($pairs.Count -gt 0) {
    foreach ($p in $pairs) {
        Write-Host ("  PAIR  {0} <-> {1}  (verified both directions)" -f $p[0], $p[1]) -ForegroundColor Green
    }
} else {
    Write-Host "  No bidirectional pair confirmed." -ForegroundColor Red
}

$loners = @($live | Where-Object { -not $paired.Contains($_) })
if ($loners.Count -gt 0) {
    Write-Host ("  UNPAIRED  {0}" -f ($loners -join ", ")) -ForegroundColor DarkYellow
}

if ($selfEcho.Count -gt 0) {
    Write-Host ("`n  Self-echo seen on: {0}" -f (($selfEcho.Keys) -join ", ")) -ForegroundColor DarkGray
    Write-Host "  (normal for auto-direction half-duplex transceivers; ignored above)" -ForegroundColor DarkGray
}

foreach ($sp in $open.Values) { $sp.Close(); $sp.Dispose() }
Write-Host ""

# A confirmed pair is the useful outcome; say so in the exit code as well as
# on screen, so this can gate the bulk test in a harness.
if ($pairs.Count -eq 0) { exit 2 }
exit 0
