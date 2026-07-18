# ASB-Supervisor

STM32F446RE firmware for the ASB (Autonomous System Brake) supervisor.

## Building

Works identically on Windows and macOS. The ARM toolchain is auto-detected
from STM32CubeIDE, STM32CubeCLT, the STM32Cube VS Code bundles, or the Arm GNU
Toolchain — newest install wins. No paths need to be configured.

### CMake (CLI or VS Code)

Requires CMake ≥ 3.22 and Ninja (both ship with
[STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)).

```sh
cmake --preset debug          # configure  (build_cmake/)
cmake --build --preset debug  # build → build_cmake/ASB_Supervisor.elf/.bin

cmake --preset release        # size-optimized (-Os) → build_release/
cmake --build --preset release
```

In VS Code the CMake Tools extension picks up `CMakePresets.json`
automatically — select the *Debug* or *Release* preset and build.

To force a specific toolchain: `cmake --preset debug -DTOOLCHAIN_DIR="<path to bin dir>"`.

### STM32CubeIDE

Import as an existing project and build normally (Debug/Release
configurations). Build output folders are git-ignored.

## Flashing

Any of: STM32CubeIDE debug session, `STM32_Programmer_CLI -c port=SWD -d
build_cmake/ASB_Supervisor.bin 0x08000000 -rst`, or drag-and-drop the `.bin`
onto the Nucleo mass-storage device.
