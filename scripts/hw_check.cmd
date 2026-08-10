@echo off
REM Launcher for hw_check.ps1.
REM
REM PowerShell defaults to a Restricted execution policy on Windows client
REM installs, which refuses to load any .ps1 - including this repo's own.
REM Unlike npm there is no .cmd shim to fall back on, so this wrapper is it.
REM -ExecutionPolicy Bypass applies to this one invocation only; no
REM machine-wide setting is changed.
REM
REM Keep this file pure ASCII: cmd.exe parses batch files in the OEM
REM codepage, and a stray non-ASCII character breaks the line it is on.
REM
REM   scripts\hw_check.cmd -Interface pcan:usb1 -RsModel rs04 -RsId 1
REM   scripts\hw_check.cmd -Interface pcan:usb1 -Motion
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0hw_check.ps1" %*
exit /b %ERRORLEVEL%
