# CigerNesting

Native Windows C++20 nesting application prototype with a modular geometry/import/engine/UI architecture.

## Requirements

- Windows 10 or Windows 11
- Visual Studio Build Tools 2022 or Visual Studio 2022
- Desktop development with C++ workload
- MSVC v143 toolset
- Windows 10/11 SDK
- CMake tools for Windows

No third-party geometry, UI, import, or packaging library is required.

## Build

Recommended Release build:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

If `cmake` is not on `PATH`, use the bundled build helper. It locates Visual Studio Build Tools via `vswhere` and uses the CMake bundled with Visual Studio:

```bat
build_msvc.bat Release
```

Debug build:

```bat
build_msvc.bat Debug
```

## Output

Release executable:

```text
build\NestingApp\Release\CigerNesting.exe
```

Debug executable:

```text
build\NestingApp\Debug\CigerNesting.exe
```

## Smoke Test

1. Run `build\NestingApp\Release\CigerNesting.exe`.
2. Use `Dosya Aç` / `Open File`.
3. Load `samples\simple_polygons.svg` or `samples\basic_shapes.svg`.
4. Press `Başlat` / `Start`.
5. Verify that the progress bar and phase text update and the canvas layout changes without freezing.

Additional sample files:

```text
samples\simple_plt.plt
samples\simple_dxf.dxf
```

## Known Gaps

- Current spacing is still based on simple collision/AABB checks, not true polygon offset.
- DXF/PLT import support is intentionally small and ASCII-oriented.
- Direct2D, GPU evaluation, AI import, and Corel macro installation are intentionally not implemented yet.
- The solver is still the first strategy inside the larger collision-driven engine architecture.
