@echo off
REM ============================================================
REM  Robot Dog SDK launcher (Windows)
REM
REM  ASCII ONLY - do not put Chinese in this file.
REM  cmd.exe reads .bat byte-by-byte using the current code page;
REM  UTF-8 Chinese desyncs the parser and corrupts later lines.
REM  All messages and logic live in start.py instead.
REM
REM  Double-click this file, or:
REM      start.bat sim | observe | control | status | check
REM ============================================================
setlocal
cd /d "%~dp0"

set "PY="
REM Prefer known interpreters over "python", which on Windows may be the
REM Microsoft Store stub that just opens the Store and exits.
if exist "C:\ProgramData\anaconda3\python.exe" set "PY=C:\ProgramData\anaconda3\python.exe"
if not defined PY if exist "%USERPROFILE%\anaconda3\python.exe" set "PY=%USERPROFILE%\anaconda3\python.exe"
if not defined PY if exist "%USERPROFILE%\miniconda3\python.exe" set "PY=%USERPROFILE%\miniconda3\python.exe"
if not defined PY for /d %%D in ("%LOCALAPPDATA%\Programs\Python\Python*") do if exist "%%D\python.exe" set "PY=%%D\python.exe"
if not defined PY for /d %%D in ("C:\Python*") do if exist "%%D\python.exe" set "PY=%%D\python.exe"
if defined PY goto found

where python >nul 2>&1
if not errorlevel 1 set "PY=python"
if not defined PY goto nopython

:found
set "PYTHONIOENCODING=utf-8"
"%PY%" start.py %*
set "RC=%ERRORLEVEL%"
if "%RC%"=="9009" goto nopython
exit /b %RC%

:nopython
echo.
echo   [ERROR] Python not found.
echo.
echo   Install Python 3.10+ and tick "Add to PATH", or edit the PY
echo   line in this file to point at your python.exe.
echo.
pause
exit /b 1
