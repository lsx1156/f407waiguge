@echo off
set OBJCOPY=D:\cube IDE\STM32CubeIDE_1.15.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.100.202403111256\tools\bin\arm-none-eabi-objcopy.exe

cd /d "G:\DL26-7-20\f407\project\IDE\Debug"

echo Generating trae.hex...
"%OBJCOPY%" -O ihex trae.elf trae.hex
if errorlevel 1 goto error

echo Generating trae.bin...
"%OBJCOPY%" -O binary trae.elf trae.bin
if errorlevel 1 goto error

echo.
echo [SUCCESS] trae.hex and trae.bin generated!
echo Location: G:\DL26-7-20\f407\project\IDE\Debug\
goto end

:error
echo.
echo [ERROR] Failed to generate files!

:end
pause
