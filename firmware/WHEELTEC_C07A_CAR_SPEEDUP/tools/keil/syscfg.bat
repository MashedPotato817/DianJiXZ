@echo off

set SYSCFG_PATH="C:\ti\sysconfig_1.20.0\sysconfig_cli.bat"

if not exist "%SYSCFG_PATH%" (
    echo.
    echo Couldn't find Sysconfig Tool %SYSCFG_PATH%
    echo "Update the file located at <sdk path>/tools/keil/syscfg.bat"
    echo.
    exit
)

echo Using Sysconfig Tool from %SYSCFG_PATH%
echo "Update the file located at <sdk path>/tools/keil/syscfg.bat to use a different version"

set PROJ_DIR=%~1
set PROJ_DIR=%PROJ_DIR:'=%

set SYSCFG_FILE=%~2
set SYSCFG_FILE=%SYSCFG_FILE:'=%

:: 本机 SDK 安装在 C:\ti，工程目录不包含 SDK 元数据。
set SDK_ROOT=C:\ti\mspm0_sdk_2_01_00_03
if not exist "%SDK_ROOT%\.metadata\product.json" (
    echo Couldn't find MSPM0 SDK metadata at %SDK_ROOT%
    exit /b 1
)

:: Search for the directory containing the project's syscfg file
:: Going up a directory atleast 5 times but then give up
set SYSCFG_DIR=%PROJ_DIR%
set iter=0
:syscfg_search_loop
if exist %SYSCFG_DIR%\*.syscfg (
    :: Remove the trailing slash if it exist since Keil doesn't like it
    IF %SYSCFG_DIR:~-1%==\ SET SYSCFG_DIR=%SYSCFG_DIR:~0,-1%
    goto syscfg_search_exit
) else if %iter% geq 5 (
	@echo "Couldn't find syscfg file"
) else (
	set /a iter=%iter%+1
	set SYSCFG_DIR=%SYSCFG_DIR%..\
	goto syscfg_search_loop
)
:syscfg_search_exit

%SYSCFG_PATH% -o "%SYSCFG_DIR%" -s "%SDK_ROOT%\.metadata\product.json" --compiler keil "%SYSCFG_DIR%\%SYSCFG_FILE%"
