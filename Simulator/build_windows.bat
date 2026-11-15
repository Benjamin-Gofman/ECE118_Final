@echo off
setlocal
cd /d "%~dp0"
where gcc >nul 2>nul
if errorlevel 1 (
  echo ERROR: MinGW-w64 GCC not found in PATH.
  exit /b 1
)
gcc -DSIMULATOR -std=c11 -Wall -Wextra -O2 -shared -ISimulatorSupport -I"..\ECE118_Final.X" -o sim_robot.dll SimHost.c "..\ECE118_Final.X\Navigation.c" "..\ECE118_Final.X\Linefollowing.c" "..\ECE118_Final.X\TapeSensor.c"
if errorlevel 1 exit /b 1
echo Built sim_robot.dll with original Navigation.c and TapeSensor.c
endlocal

python -m pip install coppeliasim-zmqremoteapi-client
python bridge.py

PAUSE