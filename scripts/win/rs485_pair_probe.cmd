@echo off
REM Launcher for rs485_pair_probe.ps1.
REM
REM PowerShell defaults to a Restricted execution policy on Windows client
REM installs, which refuses to load any .ps1 - including this repo's own.
REM -ExecutionPolicy Bypass applies to this one invocation only; no
REM machine-wide setting is changed. Same reason scripts\hw_check.cmd exists.
REM
REM Keep this file pure ASCII: cmd.exe parses batch files in the OEM
REM codepage, and a stray non-ASCII character breaks the line it is on.
REM
REM Pass a port list as ONE quoted string. -File does not re-parse the command
REM line, so separate tokens would bind to the following parameter instead of
REM building an array; the script splits the string itself.
REM
REM   scripts\win\rs485_pair_probe.cmd
REM   scripts\win\rs485_pair_probe.cmd -Ports "COM12,COM13" -Baud 1000000
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0rs485_pair_probe.ps1" %*
exit /b %ERRORLEVEL%
