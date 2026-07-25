@echo off
echo ========================================
echo   STM32F407 OpenOCD DAPLink Download
echo ========================================
echo.

cd /d "G:\DL26-7-20\f407\project\IDE"

if not exist "Debug\trae.elf" (
    echo [ERROR] Debug\trae.elf not found! Please build first.
    pause
    exit /b 1
)

echo [INFO] Programming...
"C:\Tools\openocd\bin\openocd.exe" -f "openocd_daplink.cfg" -c "program Debug/trae.elf verify reset exit"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo [SUCCESS] Program downloaded successfully!
) else (
    echo.
    echo [ERROR] Programming failed!
)

pause
