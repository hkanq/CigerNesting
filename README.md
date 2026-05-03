# CigerNesting

Native Windows C++20 nesting application prototype with a modular geometry/import/engine/UI architecture.

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release
```

The target executable is `CigerNesting.exe`.

If CMake is not installed but an MSVC Developer Command Prompt is available:

```bat
build_msvc.bat
```
