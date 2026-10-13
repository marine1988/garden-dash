@echo off
setlocal

for %%I in ("%~dp0..") do set "PROJECT_DIR=%%~fI\"
set "UPLOAD_PORT="
set "RESULT=1"

if not "%~1"=="" (
  set "UPLOAD_PORT=%~1"
)

rem Sets PIO, offering to install PlatformIO when it is missing.
call "%~dp0platformio.bat"
if errorlevel 1 goto finish

cd /d "%PROJECT_DIR%"

if not defined UPLOAD_PORT (
  for /f "usebackq delims=" %%P in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "$ports = & $env:PIO device list; $current = ''; foreach ($line in $ports) { if ($line -match '^(COM\d+)$') { $current = $matches[1]; continue }; if ($current -and $line -match 'Hardware ID:|Description:') { if ($line -match 'BTHENUM') { $current = ''; continue }; if ($line -match 'USB|CH340|CH910|CP210|Silicon Labs|FTDI|UART|1A86:7523|10C4:EA60') { $current; break } } }"`) do (
    set "UPLOAD_PORT=%%P"
  )
)

if defined UPLOAD_PORT (
  echo Building and uploading Garden-Dash to %UPLOAD_PORT%...
  "%PIO%" run -e garden -t upload --upload-port %UPLOAD_PORT%
) else (
  echo Building and uploading Garden-Dash using PlatformIO's auto-detected port...
  "%PIO%" run -e garden -t upload
)

if errorlevel 1 (
  echo.
  echo Upload failed.
  echo If you see "Wrong boot mode detected", GPIO0/BOOT was not held low during reset.
  echo Hold BOOT, tap RST/EN, keep holding BOOT until "Connecting..." changes, then release BOOT.
  echo If auto-detection picked the wrong port, specify one explicitly, for example:
  echo   scripts\upload_firmware_usb.bat COM4
  goto finish
)

echo.
echo Garden-Dash upload complete.
set "RESULT=0"

:finish
rem Keep the result on screen: garden.bat redraws its menu straight afterwards.
echo.
pause
exit /b %RESULT%
