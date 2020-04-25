rem | ---------------------------------------------------------------------------------------
rem | Name: autoset.bat
rem | Author: UBC Capstone Team 48 - 2019/2020
rem | Desc: Running this batch file in the "head_impact_IMU\ble_app\test\ds1388_autoset_test"
rem | directory will load the RTC test code to the nrf_mdk development board
rem | with (near) real time values loaded onto the RTC. Because programming the board
rem | comes with inherent delay, the RTC will actually be behind by a few seconds.
rem | The full delay value is given at the end of the script and must be accounted for during
rem | eventual data offload.
rem | [Run this file via git bash using "./autoset.bat"]
rem | ---------------------------------------------------------------------------------------

@echo OFF

rem | These lines extract the system date into year/month/day
echo DATE: %date%
set year=%date:~2,2%
set month=%date:~5,2%
set day=%date:~8,2%

rem | These lines extract the system time into hour/minute/second/hundredth
echo PRE-FLASH TIME: %time%
set hour=%time:~0,2%
if "%time:~3,1%" == "0" (set min=%time:~4,1%) else (set min=%time:~3,2%)
if "%time:~6,1%" == "0" (set sec=%time:~7,1%) else (set sec=%time:~6,2%)
set /A sec=%sec%
if "%time:~9,1%" == "0" (set hsec=%time:~10,1%) else (set hsec=%time:~9,2%)

echo.
echo BEGIN: FILE COMPILE AND FLASH
echo -----------------------------
echo.

rem | this line replaces the default time/date values (all zeros) in the test file with the current time/date
sed -i 's/{0,0,0,0,0,0,0,0}/{%year%,%month%,%day%,0,%hour%,%min%,%sec%,%hsec%}/g' ds1388_auto.c\
rem | "make flash" runs make and then programs the executable files in this directory to the board
make flash
rem | this line re-writes the default time/date values (all zeros) in the test file to ensure that this script can be run again
sed -i 's/{%year%,%month%,%day%,0,%hour%,%min%,%sec%,%hsec%}/{0,0,0,0,0,0,0,0}/g' ds1388_auto.c

echo.
echo ------------------------------
echo FINISH: FILE COMPILE AND FLASH
echo.

rem | These lines compare the pre- and post-flash system times and determine RTC offset (delay)
echo POST-FLASH TIME: %time%
if "%time:~9,1%" == "0" (set hsec1=%time:~10,1%) else (set hsec1=%time:~9,2%)
if "%time:~6,1%" == "0" (set sec1=%time:~7,1%) else (set sec1=%time:~6,2%)
set /A hsecdiff = %hsec1% - %hsec%
if %hsecdiff% lss 0 set /a hsecdiff=%hsecdiff%+100
set /A secdiff = %sec1% - %sec%
echo RTC is behind real time by %secdiff%.%hsecdiff% seconds




