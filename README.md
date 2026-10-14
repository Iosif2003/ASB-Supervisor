    n# ASB-Supervisor

STM32F446RE firmware for the ASB (Autonomous System Brake) supervisor.
Aristurtle Formula Student.

## Build

### CMake (terminal or VS Code)

Needs CMake 3.22+ and Ninja, both come with STM32CubeCLT.
The ARM toolchain is picked up automatically from CubeIDE / CubeCLT /
Arm GNU Toolchain installs, whichever is newer.

```
cmake --preset debug
cmake --build --preset debug
```

Output goes to build_cmake/ (ASB_Supervisor.elf and .bin).
There is also a release preset (-Os) that builds into build_release/.

If the toolchain is not found automatically point to it manually:

```
cmake --preset debug -DTOOLCHAIN_DIR="<path to the toolchain bin folder>"
```

In VS Code just pick the Debug preset when CMake Tools asks and build.

### STM32CubeIDE

Import as existing project and build like normal. Build folders are
gitignored.

## Flash

From CubeIDE with a debug session, or with the command line:

```
STM32_Programmer_CLI -c port=SWD -d build_cmake/ASB_Supervisor.bin 0x08000000 -rst
```

Copying the .bin onto the Nucleo USB drive also works.
