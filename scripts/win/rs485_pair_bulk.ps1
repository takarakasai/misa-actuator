<#
.SYNOPSIS
    Bulk integrity test for RS485 port pairs that are already known to be wired.

.DESCRIPTION
    rs485_pair_probe.ps1 proves the wiring with a 16-byte tag. Sixteen bytes
    say nothing about whether the link stays clean at 1 Mbit/s, which is the
    rate the LK motors run at. This pushes enough bytes to expose dropped or
    reordered ones and reports the first mismatching offset.

    Run the probe first to learn the pairing, then feed the pairs here. Each
    entry is one direction ("TX,RX"); pass the reversed list to test the other
    way, since a half-duplex transceiver can be healthy transmitting and deaf
    receiving.

    Elapsed time is worth as much as the error count. The theoretical wire time
    is printed alongside: a result that lands on it means the link ran at line
    rate with no stalls, while a much longer one points at retries or a driver
    struggling to keep up even though every byte eventually arrived.

    Exits non-zero if any direction fails.

.EXAMPLE
    .\scripts\win\rs485_pair_bulk.ps1
    All three bench pairs, 8 KB each at 1 Mbit/s.

.EXAMPLE
    .\scripts\win\rs485_pair_bulk.ps1 -Pairs "COM13,COM12","COM15,COM14","COM19,COM16"
    The same pairs the other way round. Quote each entry — an unquoted
    COM13,COM12 is parsed as two separate list items and neither has an RX.

.EXAMPLE
    .\scripts\win\rs485_pair_bulk.ps1 -Pairs "COM12,COM13" -Bytes 262144
    One pair, a longer soak to catch an intermittent fault.

.EXAMPLE
    .\scripts\win\rs485_pair_bulk.cmd -Pairs "COM13,COM12;COM15,COM14"
    Through the wrapper, which sidesteps the execution policy. Semicolons
    separate the pairs there — see the note above the $Pairs normalization.

.NOTES
    Bench result 2026-08-17 (CH348): all six directions of COM12/COM13,
    COM14/COM15 and COM16/COM19 passed 8 KB at 1 Mbit/s with zero byte errors,
    in 84-97 ms against a theoretical 81.9 ms.
#>

[CmdletBinding()]
param(
    # One direction per entry, as "TX,RX". Quote each entry.
    [string[]]$Pairs = @("COM12,COM13", "COM14,COM15", "COM16,COM19"),
    # Defaults to the LK motors' rate — the point is to test what production uses.
    [int]$Baud = 1000000,
    [int]$Bytes = 8192
)

# Deliberately not "Stop". Every pair has to get its turn; one bad link is a
# FAIL line, not the end of the sweep.
$ErrorActionPreference = "Continue"

# `powershell.exe -File` cannot pass an array (see the same note in
# rs485_pair_probe.ps1), so a whole list is accepted as one semicolon-separated
# string. The separator between pairs has to differ from the one inside a pair,
# hence ';' between and ',' or ':' within:
#   -Pairs "COM13,COM12","COM15,COM14"    from PowerShell
#   -Pairs "COM13,COM12;COM15,COM14"      through the .cmd wrapper
$Pairs = @($Pairs |
    ForEach-Object { $_ -split ";" } |
    ForEach-Object { $_.Trim() } |
    Where-Object { $_ })

# Deterministic and non-repeating: a dropped byte and a byte-order mix-up both
# surface as a mismatch, either of which a constant fill would hide completely.
$payload = New-Object byte[] $Bytes
for ($i = 0; $i -lt $Bytes; $i++) {
    $payload[$i] = [byte](((($i * 37) -bxor ($i -shr 5)) + 7) -band 0xFF)
}

Write-Host "RS485 bulk integrity: $Bytes bytes @ $Baud baud 8N1" -ForegroundColor Cyan
$expectedMs = [math]::Round($Bytes * 10.0 / $Baud * 1000, 1)
Write-Host "  wire time for $Bytes bytes at 10 bits/byte: ${expectedMs} ms`n" -ForegroundColor DarkGray

$failures = 0

foreach ($spec in $Pairs) {
    $parts = @($spec -split "[,:]")
    if ($parts.Count -lt 2) {
        Write-Host ("  {0,-16} BAD SPEC - expected `"TX,RX`", quote each entry" -f $spec) -ForegroundColor Red
        $failures++
        continue
    }
    $txName = $parts[0].Trim()
    $rxName = $parts[1].Trim()
    $label = "{0} -> {1}" -f $txName, $rxName

    $tx = $null
    $rx = $null
    try {
        $tx = New-Object System.IO.Ports.SerialPort $txName, $Baud, "None", 8, "One"
        $rx = New-Object System.IO.Ports.SerialPort $rxName, $Baud, "None", 8, "One"
        foreach ($sp in @($tx, $rx)) {
            $sp.Handshake = "None"
            $sp.ReadTimeout = 200
            $sp.WriteTimeout = 5000
            # The 4096-byte default is smaller than the transfer, so it would
            # overflow while we are still draining it and turn a clean link
            # into a spurious FAIL.
            $sp.ReadBufferSize = 65536
            $sp.WriteBufferSize = 65536
            $sp.Open()
            $sp.DiscardInBuffer()
            $sp.DiscardOutBuffer()
        }

        $got = New-Object byte[] $Bytes
        $have = 0

        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        $tx.Write($payload, 0, $Bytes)

        # Drain until the payload is complete or the wire goes quiet for a
        # second — a short read is the finding, so it must not hang forever.
        $lastProgress = $sw.ElapsedMilliseconds
        while ($have -lt $Bytes) {
            $avail = $rx.BytesToRead
            if ($avail -gt 0) {
                $want = [math]::Min($avail, $Bytes - $have)
                $n = $rx.Read($got, $have, $want)
                $have += $n
                $lastProgress = $sw.ElapsedMilliseconds
            } elseif (($sw.ElapsedMilliseconds - $lastProgress) -gt 1000) {
                break
            }
        }
        $sw.Stop()

        # Compare only what arrived; the shortfall is reported on its own line.
        $firstBad = -1
        for ($i = 0; $i -lt $have; $i++) {
            if ($got[$i] -ne $payload[$i]) { $firstBad = $i; break }
        }

        # Anything still buffered past the payload is noise on the bus, which
        # matters even though the payload itself checked out.
        $extra = $rx.BytesToRead

        if ($have -eq $Bytes -and $firstBad -lt 0 -and $extra -eq 0) {
            $kbps = [math]::Round($Bytes * 8.0 / [math]::Max($sw.ElapsedMilliseconds, 1), 1)
            Write-Host ("  {0,-16} OK  {1}/{2} bytes, 0 errors, {3} ms ({4} kbit/s)" -f `
                $label, $have, $Bytes, $sw.ElapsedMilliseconds, $kbps) -ForegroundColor Green
        } else {
            $failures++
            Write-Host ("  {0,-16} FAIL {1}/{2} bytes in {3} ms" -f `
                $label, $have, $Bytes, $sw.ElapsedMilliseconds) -ForegroundColor Red
            if ($have -lt $Bytes) {
                Write-Host ("      {0} bytes never arrived" -f ($Bytes - $have)) -ForegroundColor Red
            }
            if ($firstBad -ge 0) {
                Write-Host ("      first mismatch at offset {0}: expected 0x{1:X2}, got 0x{2:X2}" -f `
                    $firstBad, $payload[$firstBad], $got[$firstBad]) -ForegroundColor Red
            }
            if ($extra -gt 0) {
                Write-Host ("      {0} unexpected extra bytes still buffered" -f $extra) -ForegroundColor Red
            }
        }
    } catch {
        Write-Host ("  {0,-16} ERROR {1}" -f $label, $_.Exception.Message) -ForegroundColor Red
        $failures++
    } finally {
        foreach ($sp in @($tx, $rx)) {
            if ($null -ne $sp) {
                if ($sp.IsOpen) { $sp.Close() }
                $sp.Dispose()
            }
        }
    }
}

Write-Host ""
if ($failures -gt 0) {
    Write-Host "$failures direction(s) failed." -ForegroundColor Red
    exit 1
}
exit 0
