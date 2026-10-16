@echo off
rem Renders the README screenshots with the local GPU (headless, no window).
rem Four views of the drive scene: the starting grid, a corner, the cockpit and
rem a broadside shot. The autopilot drives for --time seconds before the capture
rem (--time 0 leaves the car on the start/finish line).
setlocal
set ROOT=%~dp0..
set BIN=%ROOT%\build\windows-msvc-release\bin
set OUT=%ROOT%\docs\images
if not exist "%OUT%" mkdir "%OUT%"
pushd "%BIN%"

rem Grid: the car on the start/finish line, chase camera.
sim.exe --headless --frames 4 --width 1600 --height 900 --scene drive ^
    --camera chase --time 0 --screenshot "%OUT%\circuit.png" >> capture.log 2>&1

rem Cornering: the hairpin, chase camera.
sim.exe --headless --frames 4 --width 1600 --height 900 --scene drive ^
    --camera chase --time 16 --screenshot "%OUT%\cornering.png" >> capture.log 2>&1

rem Cockpit: from the driver's seat on the main straight.
sim.exe --headless --frames 4 --width 1600 --height 900 --scene drive ^
    --camera cockpit --time 10 --screenshot "%OUT%\cockpit.png" >> capture.log 2>&1

rem Side: broadside through the hairpin.
sim.exe --headless --frames 4 --width 1600 --height 900 --scene drive ^
    --camera side --time 16 --screenshot "%OUT%\side.png" >> capture.log 2>&1

popd
echo Screenshots written to %OUT%
endlocal
