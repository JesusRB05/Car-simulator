@echo off
rem Renders the README screenshots with the local GPU (headless, no window).
rem
rem The views are the ones the README shows, at 1600x900 and through the engine
rem itself: the drive scene (chase, cockpit, side, orbit and an oblique aerial)
rem and the showroom. Every capture is deterministic for a given build, so a
rem candidate can be compared with the published image.
rem
rem By default the files go to screenshots\golden_candidates\readme for review;
rem pass --publish to overwrite docs\images once the candidates are approved.
setlocal
set ROOT=%~dp0..
set BIN=%ROOT%\build\windows-msvc-release\bin
set OUT=%ROOT%\screenshots\golden_candidates\readme
if /i "%~1"=="--publish" set OUT=%ROOT%\docs\images
if not exist "%OUT%" mkdir "%OUT%"
set LOG=%OUT%\capture.log
if exist "%LOG%" del "%LOG%"

set SIM=%BIN%\sim.exe
set SIZE=--width 1600 --height 900 --frames 4
set SCENE=--headless --scene drive %SIZE%
pushd "%BIN%"

rem --- The car on the circuit ------------------------------------------------
rem Grid: the car on the start/finish line, chase camera.
"%SIM%" %SCENE% --camera chase --time 0 --screenshot "%OUT%\circuit.png" >> "%LOG%" 2>&1

rem Cornering: the first corner complex, chase camera.
"%SIM%" %SCENE% --camera chase --time 16 --screenshot "%OUT%\cornering.png" >> "%LOG%" 2>&1

rem Cockpit: from the driver's seat on the main straight.
"%SIM%" %SCENE% --camera cockpit --time 10 --screenshot "%OUT%\cockpit.png" >> "%LOG%" 2>&1

rem Side: broadside through the hairpin.
"%SIM%" %SCENE% --camera side --time 16 --screenshot "%OUT%\side.png" >> "%LOG%" 2>&1

rem --- Sierra Verde ----------------------------------------------------------
rem The grid with three cars: the player's on the line plus two behind it, from
rem in front and above so the whole grid and the gantry read.
"%SIM%" %SCENE% --camera orbit --yaw 3.3 --pitch 0.32 --orbit-distance 20 --time 0 ^
    --grid-cars 3 --screenshot "%OUT%\sierra_verde_grid.png" >> "%LOG%" 2>&1

rem Looking back down the grid from in front of the car.
"%SIM%" %SCENE% --camera orbit --yaw 3.1416 --pitch 0.22 --orbit-distance 9 --time 0 ^
    --grid-cars 3 --screenshot "%OUT%\sierra_verde_grid_front.png" >> "%LOG%" 2>&1

rem Turn 1 seen from outside the corner, so the kerbs and the sand trap frame
rem the car.
"%SIM%" %SCENE% --camera orbit --yaw 2.0 --pitch 0.42 --orbit-distance 14 --time 20 ^
    --screenshot "%OUT%\sierra_verde_turn1.png" >> "%LOG%" 2>&1

rem The hairpin, where the car is slow enough to read its attitude.
"%SIM%" %SCENE% --camera orbit --yaw 2.1 --pitch 0.36 --orbit-distance 14 --time 40 ^
    --screenshot "%OUT%\sierra_verde_hairpin.png" >> "%LOG%" 2>&1

rem An oblique aerial over the start line. Keep the pitch below ~0.6 rad: from
rem higher up the 24 mm depth resolution at 200 m beats the road's take-off and
rem the terrain wins the depth test.
"%SIM%" %SCENE% --camera orbit --yaw 0.8 --pitch 0.42 --orbit-distance 170 --time 0 ^
    --grid-cars 3 --screenshot "%OUT%\sierra_verde_aerial.png" >> "%LOG%" 2>&1

rem --- The showroom ----------------------------------------------------------
rem Both shipped liveries, in the car menu's own scene.
"%SIM%" --headless --scene garage %SIZE% --car base.core/halvard_gt3 --skin green_44 ^
    --yaw 2.4 --pitch 0.24 --screenshot "%OUT%\gt3_studio_front.png" >> "%LOG%" 2>&1
"%SIM%" --headless --scene garage %SIZE% --car base.core/halvard_gt3 --skin red_black_8 ^
    --yaw 0.7 --pitch 0.20 --screenshot "%OUT%\gt3_studio_rear.png" >> "%LOG%" 2>&1

popd
echo Screenshots written to %OUT%
echo Log: %LOG%
endlocal
