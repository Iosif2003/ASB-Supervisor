@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "CONFIG=Debug"
set "MAKE_ARGS=%*"
set "TARGET=%~1"

if /I "%~1"=="Debug" (
  set "CONFIG=Debug"
  set "MAKE_ARGS=%~2 %~3 %~4 %~5 %~6 %~7 %~8 %~9"
  set "TARGET=%~2"
) else if /I "%~1"=="Release" (
  set "CONFIG=Release"
  set "MAKE_ARGS=%~2 %~3 %~4 %~5 %~6 %~7 %~8 %~9"
  set "TARGET=%~2"
)

if "%TARGET%"=="" (
  set "TARGET=all"
)

if "%MAKE_ARGS%"=="" (
  set "MAKE_ARGS=all"
)

set "MAKE_EXE=C:\mingw64\bin\mingw32-make.exe"
set "TOOLCHAIN_BIN=C:\ST\STM32CubeIDE_2.1.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
set "BUILD_DIR=%SCRIPT_DIR%%CONFIG%"

if not exist "%MAKE_EXE%" (
  echo Missing make executable: "%MAKE_EXE%"
  exit /b 1
)

if not exist "%TOOLCHAIN_BIN%\arm-none-eabi-gcc.exe" (
  echo Missing STM32 toolchain: "%TOOLCHAIN_BIN%\arm-none-eabi-gcc.exe"
  exit /b 1
)

if not exist "%BUILD_DIR%\makefile" (
  echo Missing generated makefile: "%BUILD_DIR%\makefile"
  exit /b 1
)

if /I "%TARGET%"=="clean" (
  powershell -NoProfile -ExecutionPolicy Bypass -Command ^
    "$patterns = '.o','.d','.su','.cyclo','.elf','.list','.map';" ^
    "Get-ChildItem -Path '%BUILD_DIR%' -Recurse -File -ErrorAction SilentlyContinue | Where-Object { (($patterns -contains $_.Extension) -and $_.Name -ne 'objects.list') -or $_.Name -eq 'default.size.stdout' } | Remove-Item -Force -ErrorAction SilentlyContinue"
  exit /b %ERRORLEVEL%
)

set "PATH=%TOOLCHAIN_BIN%;C:\mingw64\bin;%PATH%"
"%MAKE_EXE%" -C "%BUILD_DIR%" %MAKE_ARGS%
exit /b %ERRORLEVEL%
