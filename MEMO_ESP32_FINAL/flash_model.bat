@echo off
setlocal enabledelayedexpansion

echo =======================================================
echo    MEMO ESP32-S3 WAKENET9 MODEL FLASH TOOL
echo    Target Partition Offset: 0xC10000 (Size: ~3.3MB)
echo =======================================================

set PORT=%1
if "%PORT%"=="" (
    set /p PORT="Enter your ESP32-S3 COM Port (e.g. COM3, COM5): "
)

if "%PORT%"=="" (
    echo [ERROR] No COM port specified.
    pause
    exit /b 1
)

set ESPTOOL="%LOCALAPPDATA%\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe"
if not exist %ESPTOOL% (
    set ESPTOOL=esptool.py
)

echo.
echo [1/2] Checking srmodels.bin...
if not exist srmodels.bin (
    echo [ERROR] srmodels.bin not found in current folder!
    pause
    exit /b 1
)

echo [2/2] Flashing srmodels.bin to %PORT% at offset 0xC10000...
%ESPTOOL% --chip esp32s3 --port %PORT% --baud 921600 write_flash 0xC10000 srmodels.bin

if %ERRORLEVEL% EQU 0 (
    echo.
    echo =======================================================
    echo    SUCCESS! ESP-SR WakeNet9 Model Flashed to 0xC10000!
    echo    You can now upload MEMO_ESP32_FINAL.ino via Arduino IDE.
    echo =======================================================
) else (
    echo.
    echo [ERROR] Flashing failed with error code %ERRORLEVEL%.
)

pause
