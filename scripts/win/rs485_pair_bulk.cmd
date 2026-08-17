@echo off
REM Launcher for rs485_pair_bulk.ps1.
REM
REM PowerShell defaults to a Restricted execution policy on Windows client
REM installs, which refuses to load any .ps1 - including this repo's own.
REM -ExecutionPolicy Bypass applies to this one invocation only; no
REM machine-wide setting is changed. Same reason scripts\hw_check.cmd exists.
REM
REM Keep this file pure ASCII: cmd.exe parses batch files in the OEM
REM codepage, and a stray non-ASCII character breaks the line it is on.
REM
REM Pass the whole pair list as ONE quoted string, semicolons between pairs and
REM a comma inside each. -File does not re-parse the command line, so separate
REM tokens would bind to the following parameter instead of building an array.
REM
REM   scripts\win\rs485_pair_bulk.cmd
REM   scripts\win\rs485_pair_bulk.cmd -Pairs "COM13,COM12;COM15,COM14"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0rs485_pair_bulk.ps1" %*
exit /b %ERRORLEVEL%
