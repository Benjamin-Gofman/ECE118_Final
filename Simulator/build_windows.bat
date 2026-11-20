@echo off
setlocal
cd /d "%~dp0"
title ECE118 Simulator - Windows Build and Run
where gcc >nul 2>nul
if errorlevel 1 (
    echo ERROR: MinGW-w64 GCC not found in PATH.
    pause
    exit /b 1
)
echo Building sim_robot.dll for Windows...
rem PIC32 sources use the virtual IO_Ports.h via -ISimulatorSupport; no SIMULATOR macro is required.
gcc -std=c11 -Wall -Wextra -O2 -shared ^
    -ISimulatorSupport -I"..\ECE118_Final.X" ^
    -o sim_robot.dll ^
    SimHost.c ^
    "..\ECE118_Final.X\Navigation.c" ^
    "..\ECE118_Final.X\Bumper.c" ^
    "..\ECE118_Final.X\LineFollowing.c" ^
    "..\ECE118_Final.X\TapeSensor.c" ^
    "..\ECE118_Final.X\Opponent.c"
if errorlevel 1 (
    echo ERROR: DLL build failed.
    pause
    exit /b 1
)
echo Windows DLL build successful.
python bridge.py
if errorlevel 1 echo ERROR: Bridge exited with an error.
pause
endlocal
