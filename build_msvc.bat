@echo off
setlocal enabledelayedexpansion

if not exist build mkdir build
set SOURCES=
for %%D in (app ui render geometry import engine bridge) do (
  for %%F in (NestingApp\%%D\*.cpp) do (
    set SOURCES=!SOURCES! "%%F"
  )
)

cl /nologo /std:c++20 /EHsc /W4 /permissive- /DWIN32_LEAN_AND_MEAN /DNOMINMAX /DUNICODE /D_UNICODE /I NestingApp !SOURCES! /Fe:build\CigerNesting.exe /link user32.lib gdi32.lib comdlg32.lib shell32.lib
