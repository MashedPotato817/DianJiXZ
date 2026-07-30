@echo off
rem Local SysConfig pre-build wrapper (absolute paths).
rem The project was copied out of the SDK examples tree, so the SDK's own
rem syscfg.bat (which auto-searches upward for .metadata\product.json) fails
rem across drives. This wrapper calls the SysConfig CLI directly.
rem If the SDK or SysConfig install location changes, edit the 3 paths below.

set "SYSCFG_CLI=C:\ti\sysconfig_1.20.0\sysconfig_cli.bat"
set "PRODUCT_JSON=C:\ti\mspm0_sdk_2_01_00_03\.metadata\product.json"
set "PROJ_ROOT=%~dp0.."

if not exist "%SYSCFG_CLI%" (
    echo [run_syscfg] Couldn't find SysConfig CLI: %SYSCFG_CLI%
    exit /b 1
)

"%SYSCFG_CLI%" -o "%PROJ_ROOT%" -s "%PRODUCT_JSON%" --compiler keil "%PROJ_ROOT%\timx_timer_mode_pwm_center_stop.syscfg"
