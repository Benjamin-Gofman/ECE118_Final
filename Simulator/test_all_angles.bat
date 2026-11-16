@echo off
setlocal
cd /d "%~dp0"
where gcc >nul 2>nul
if errorlevel 1 (
    echo ERROR: MinGW-w64 gcc is not in PATH.
    pause
    exit /b 1
)
echo Building sim_robot.dll...
gcc -DSIMULATOR -std=c11 -Wall -Wextra -O2 -shared ^
    -ISimulatorSupport -I"..\ECE118_Final.X" ^
    -o sim_robot.dll ^
    SimHost.c ^
    "..\ECE118_Final.X\Navigation.c" ^
    "..\ECE118_Final.X\LineFollowing.c" ^
    "..\ECE118_Final.X\TapeSensor.c"
if errorlevel 1 (
    echo ERROR: DLL build failed.
    pause
    exit /b 1
)
echo OK: sim_robot.dll built.
python test_all_angles.py
if errorlevel 1 echo ERROR: Angle sweep failed.
pause
endlocal
