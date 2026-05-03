@echo off
setlocal EnableExtensions

set "CONFIG=%~1"
if "%CONFIG%"=="" set "CONFIG=Release"
set "ARCH=x64"
set "VSINSTALL="
set "CMAKE_EXE="

where cmake.exe >nul 2>nul
if not errorlevel 1 (
  for /f "delims=" %%I in ('where cmake.exe') do if not defined CMAKE_EXE set "CMAKE_EXE=%%I"
)

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" (
  for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%I"
)

if not defined VSINSTALL if exist "C:\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" set "VSINSTALL=C:\BuildTools"

if not defined CMAKE_EXE if defined VSINSTALL if exist "%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" set "CMAKE_EXE=%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

if not defined CMAKE_EXE (
  echo CMake was not found. Install Visual Studio Build Tools 2022 with C++ CMake tools.
  exit /b 1
)

if defined VSINSTALL (
  set "PATH=%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja;%PATH%"
)

echo Using CMake: %CMAKE_EXE%
"%CMAKE_EXE%" -S . -B build -G "Visual Studio 17 2022" -A %ARCH%
if errorlevel 1 exit /b %errorlevel%

"%CMAKE_EXE%" --build build --config %CONFIG% -- /m
if errorlevel 1 exit /b %errorlevel%

echo Built build\NestingApp\%CONFIG%\CigerNesting.exe
