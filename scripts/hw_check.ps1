<#
.SYNOPSIS
    HW communication-check harness for Windows — the PowerShell port of
    scripts/hw_check.sh.

.DESCRIPTION
    Checks:
      - RobStride 04        (extended-ID CAN, robstride-cli)
      - MyActuator RMD X4   (standard-ID CAN V3, myactuator-cli)

    The two families can share one 1 Mbit/s bus (RobStride uses extended
    29-bit IDs, MyActuator standard 11-bit IDs, and each driver filters out
    the other kind) or live on separate channels — use -RsInterface /
    -MyaInterface to override per family (e.g. a two-channel PEAK adapter
    wiring RobStride to pcan:usb1 and MyActuator to pcan:usb2).

    Default flow is READ-ONLY (scan + status reads — nothing moves). Pass
    -Motion to add a small motion phase; free the shafts first.

    Unlike the Linux script there is no -Setup step: Windows has no `ip link`,
    the bitrate travels in the interface string instead (pcan:usb1@500K).

.EXAMPLE
    .\scripts\hw_check.ps1 -Interface pcan:usb1
.EXAMPLE
    .\scripts\hw_check.ps1 -RsInterface pcan:usb1 -MyaInterface pcan:usb2
.EXAMPLE
    .\scripts\hw_check.ps1 -Interface slcan:COM5 -Motion
#>

[CmdletBinding()]
param(
    # Default CAN interface for both families.
    [string]$Interface = "pcan:usb1",
    # RobStride-only override.
    [string]$RsInterface,
    # MyActuator-only override.
    [string]$MyaInterface,
    [int]$RsId = 1,
    [string]$RsModel = "rs04",
    # Torque-control leg of the motion phase (N*m). Has to beat the joint's
    # stiction or that leg silently does nothing: the 2026-07-30 bench put the
    # RS04's breakaway at 0.902 N*m once the unit-conversion and judder bugs
    # were fixed (notes section 7.4), so the older 0.3 default never moved it.
    [double]$RsTorque = 1.2,
    [int]$MyaId = 1,
    # MyActuator Kt in N*m/A; 0 keeps the CLI in current units.
    [double]$Kt = 0,
    [int]$TimeoutMs = 200,
    # Include the motion phase (small moves, then stop).
    [switch]$Motion,
    # Skip the motion confirmation prompt.
    [switch]$Yes,
    [switch]$SkipRs,
    [switch]$SkipMya,
    # Use the binaries already in target\debug instead of rebuilding.
    [switch]$SkipBuild
)

# Deliberately not "Stop". This harness exists to run every step and report a
# table; a single step going wrong has to be recorded as FAIL and moved past,
# not allowed to abort the sweep before the other motors are even probed.
$ErrorActionPreference = "Continue"

if (-not $RsInterface)  { $RsInterface  = $Interface }
if (-not $MyaInterface) { $MyaInterface = $Interface }

$Root = Split-Path -Parent $PSScriptRoot
$RsCli  = Join-Path $Root "target\debug\robstride-cli.exe"
$MyaCli = Join-Path $Root "target\debug\myactuator-cli.exe"

$LogDir = Join-Path $Root "hw_check_logs"
if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory -Path $LogDir | Out-Null }
$Log = Join-Path $LogDir ("hw_check_{0}.log" -f (Get-Date -Format "yyyyMMdd_HHmmss"))

$Summary = New-Object System.Collections.Generic.List[string]
$script:Failures = 0

# Run one step and record a verdict.
#   -Mode exit         PASS on exit code 0
#   -Mode "grep:REGEX" PASS on exit 0 AND output matching REGEX
#   -Mode warn         like exit, but a failure records WARN instead of FAIL
function Invoke-Step {
    param(
        [string]$Mode,
        [string]$Label,
        [string]$Exe,
        [string[]]$Arguments
    )

    Write-Host "-- $Label" -ForegroundColor White
    Add-Content -Path $Log -Value "-- ${Label}: $Exe $($Arguments -join ' ')" -Encoding utf8

    # Both streams are captured through files rather than with `2>&1`.
    #
    # In Windows PowerShell 5.1, redirecting a *native* command's stderr wraps
    # every line in an ErrorRecord (NativeCommandError) and trips
    # $ErrorActionPreference even when the program exited 0. Every CLI here
    # logs to stderr through env_logger, so `2>&1` turns a perfectly good scan
    # into a script abort. Start-Process keeps the streams apart, takes the
    # arguments as an array so there is nothing to quote, and hands back the
    # real exit code.
    $outFile = [System.IO.Path]::GetTempFileName()
    $errFile = [System.IO.Path]::GetTempFileName()
    $rc = 1
    try {
        $proc = Start-Process -FilePath $Exe -ArgumentList $Arguments -NoNewWindow -Wait -PassThru `
            -RedirectStandardOutput $outFile -RedirectStandardError $errFile
        $rc = $proc.ExitCode
    } catch {
        Set-Content -Path $errFile -Value "failed to start ${Exe}: $_" -Encoding utf8
    }
    $out = ((Get-Content $outFile -Raw -ErrorAction SilentlyContinue) +
            (Get-Content $errFile -Raw -ErrorAction SilentlyContinue))
    Remove-Item $outFile, $errFile -Force -ErrorAction SilentlyContinue
    if ($null -eq $out) { $out = "" }

    Add-Content -Path $Log -Value $out -Encoding utf8
    $out -split "`r?`n" | Select-Object -First 15 | ForEach-Object {
        if ($_ -ne "") { Write-Host "   $_" -ForegroundColor DarkGray }
    }

    $verdict = "PASS"
    if ($rc -ne 0) {
        $verdict = "FAIL"
    } elseif ($Mode -like "grep:*") {
        $pattern = $Mode.Substring(5)
        if ($out -notmatch $pattern) { $verdict = "FAIL" }
    }
    if ($verdict -eq "FAIL" -and $Mode -eq "warn") { $verdict = "WARN" }

    switch ($verdict) {
        "PASS" { Write-Host "   PASS" -ForegroundColor Green }
        "WARN" { Write-Host "   WARN (optional feature - see log)" -ForegroundColor Yellow }
        "FAIL" {
            Write-Host "   FAIL (rc=$rc)" -ForegroundColor Red
            $script:Failures++
        }
    }
    $Summary.Add("$verdict  $Label")
    Write-Host ""
}

function Add-SkippedStep {
    param([string]$Label)
    $Summary.Add("SKIP  $Label")
    Write-Host "-- ${Label}: skipped" -ForegroundColor DarkGray
    Write-Host ""
}

Write-Host "=== misa-actuator HW check (Windows) ===" -ForegroundColor White
Write-Host "robstride: if=$RsInterface id=$RsId model=$RsModel  myactuator: if=$MyaInterface id=$MyaId kt=$Kt"
Write-Host "log: $Log"
Write-Host ""

# ---- 0) build the CLIs ----------------------------------------------------
#
# `cargo` is looked up rather than assumed: rustup puts it on the *user* PATH,
# which a shell opened before the install never picked up. Failing here with
# "cargo is not recognized" would be a confusing way to start a hardware check.
function Resolve-Cargo {
    $onPath = Get-Command cargo -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }
    $fallback = Join-Path $env:USERPROFILE ".cargo\bin\cargo.exe"
    if (Test-Path $fallback) { return $fallback }
    return $null
}

if ($SkipBuild) {
    Add-SkippedStep "build (using the binaries already in target\debug)"
} else {
    $cargo = Resolve-Cargo
    if (-not $cargo) {
        Write-Host "cargo not found on PATH or in %USERPROFILE%\.cargo\bin." -ForegroundColor Yellow
        Write-Host "Open a new terminal so the installer's PATH change takes effect," -ForegroundColor Yellow
        Write-Host "or re-run with -SkipBuild to use the existing binaries." -ForegroundColor Yellow
        exit 2
    }
    Invoke-Step -Mode exit -Label "build robstride-cli / myactuator-cli" `
        -Exe $cargo -Arguments @(
            "build", "--manifest-path", (Join-Path $Root "Cargo.toml"),
            "-p", "robstride-cli", "-p", "myactuator-cli")
}

foreach ($exe in @($RsCli, $MyaCli)) {
    if (-not (Test-Path $exe)) {
        Write-Host "missing $exe - build first, or drop -SkipBuild." -ForegroundColor Red
        exit 2
    }
}

# There is no interface "up" check: on Windows the adapter is opened directly
# by the CLI, so a dead interface shows up as a failed open in the next step.

# ---- 1) RobStride (read-only) ---------------------------------------------
if (-not $SkipRs) {
    Invoke-Step -Mode "grep:found [1-9]" `
        -Label "robstride: probe id $RsId (device-id scan) on $RsInterface" `
        -Exe $RsCli -Arguments @(
            "-i", $RsInterface, "-m", "$RsId", "--model", $RsModel,
            "scan", "--from", "$RsId", "--to", "$RsId")
    Invoke-Step -Mode exit -Label "robstride: parameter snapshot (pos/vel/Iq/Vbus)" `
        -Exe $RsCli -Arguments @(
            "-i", $RsInterface, "-m", "$RsId", "--model", $RsModel, "info")
} else {
    Add-SkippedStep "robstride phase"
}

# ---- 2) MyActuator (read-only) --------------------------------------------
if (-not $SkipMya) {
    Invoke-Step -Mode "grep:motor id [0-9]+.*responded" `
        -Label "myactuator: probe id $MyaId (0x9A scan) on $MyaInterface" `
        -Exe $MyaCli -Arguments @(
            "-i", $MyaInterface, "-m", "$MyaId", "--timeout-ms", "$TimeoutMs",
            "scan", "--from", "$MyaId", "--to", "$MyaId")
    Invoke-Step -Mode exit -Label "myactuator: status (0x9A voltage/errors + 0x92/0x9C state)" `
        -Exe $MyaCli -Arguments @(
            "-i", $MyaInterface, "-m", "$MyaId", "--timeout-ms", "$TimeoutMs", "status")
    Invoke-Step -Mode exit -Label "myactuator: multi-turn angle (0x92)" `
        -Exe $MyaCli -Arguments @(
            "-i", $MyaInterface, "-m", "$MyaId", "--timeout-ms", "$TimeoutMs", "angle")
} else {
    Add-SkippedStep "myactuator phase"
}

# ---- 3) motion phase (opt-in) ---------------------------------------------
$runMotion = [bool]$Motion
if ($runMotion) {
    Write-Host "Motion phase: the shafts WILL move (+/-0.2-0.3 rad, +/-0.5 rad/s)." -ForegroundColor Yellow
    Write-Host "Make sure both motors can rotate freely." -ForegroundColor Yellow
    if (-not $Yes) {
        # Read-Host is deliberate here: this is the one interactive gate.
        $ans = Read-Host "proceed? [y/N]"
        if ($ans -notmatch '^[yY]$') {
            Write-Host "motion phase aborted."
            $runMotion = $false
        }
    }
}

if ($runMotion) {
    if (-not $SkipRs) {
        # Exercises position / velocity / torque / MIT in sequence and always
        # leaves the motor disabled. Small offsets for the 120 N*m RS-04.
        Invoke-Step -Mode exit -Label "robstride: smoke-test (pos/vel/torque/MIT)" `
            -Exe $RsCli -Arguments @(
                "-i", $RsInterface, "-m", "$RsId", "--model", $RsModel, "smoke-test",
                "--pos-offset", "0.2", "--pos-speed", "1.0", "--vel", "0.5",
                "--torque", "$RsTorque", "--mit-kp", "10", "--mit-kd", "0.5",
                "--duration", "0.8")
    } else {
        Add-SkippedStep "robstride motion"
    }

    if (-not $SkipMya) {
        $ktArgs = @()
        if ($Kt -ne 0) { $ktArgs = @("--kt", "$Kt") }
        $myaBase = @("-i", $MyaInterface, "-m", "$MyaId", "--timeout-ms", "$TimeoutMs") + $ktArgs

        Invoke-Step -Mode exit -Label "myactuator: position move (0xA4, +/-0.2 rad)" `
            -Exe $MyaCli -Arguments ($myaBase + @("move-to", "0.2", "--speed", "1.0", "--duration", "2"))
        Invoke-Step -Mode exit -Label "myactuator: velocity (0xA2, 0.5 rad/s)" `
            -Exe $MyaCli -Arguments ($myaBase + @("spin", "0.5", "--duration", "2"))
        # Motion (MIT) mode needs RMD-X V3 firmware; a timeout here is a
        # firmware capability gap, not a wiring failure -> WARN.
        Invoke-Step -Mode warn -Label "myactuator: motion-mode MIT hold (0x400 channel)" `
            -Exe $MyaCli -Arguments ($myaBase + @("mit", "--kp", "5", "--kd", "0.5", "--duration", "2"))
    } else {
        Add-SkippedStep "myactuator motion"
    }
} else {
    Add-SkippedStep "motion phase (pass -Motion to enable)"
}

# ---- summary --------------------------------------------------------------
Write-Host "=== summary ===" -ForegroundColor White
foreach ($line in $Summary) {
    $color = switch -Wildcard ($line) {
        "PASS*" { "Green" }
        "WARN*" { "Yellow" }
        "FAIL*" { "Red" }
        default { "DarkGray" }
    }
    Write-Host "  $line" -ForegroundColor $color
}
Write-Host ""
Write-Host "full log: $Log"

if ($script:Failures -gt 0) {
    Write-Host "$($script:Failures) step(s) failed." -ForegroundColor Red
    Write-Host "Common causes:"
    Write-Host "  - PEAK driver missing            -> install the PEAK-System device driver"
    Write-Host "  - channel already open           -> close PCAN-View"
    Write-Host "  - wrong bitrate                  -> try -Interface ""$($Interface)@500K"""
    Write-Host "  - wrong motor id                 -> widen the probe with the CLIs' scan"
    Write-Host "  - termination / wiring           -> check 120 ohm at both ends"
    exit 1
}
Write-Host "all good." -ForegroundColor Green
