cmake_minimum_required(VERSION 3.22)

# Toolchain must be set before project()
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Allow override via -DTOOLCHAIN_PREFIX=... on the cmake command line
if(NOT DEFINED TOOLCHAIN_PREFIX)
    set(TOOLCHAIN_PREFIX arm-none-eabi-)
endif()

# ---------------------------------------------------------------------------
# Toolchain auto-detection (Windows / macOS / Linux)
# ---------------------------------------------------------------------------
# Priority:
#   1. -DTOOLCHAIN_DIR=<bin dir>  explicit override on the cmake command line
#   2. Known install locations (STM32CubeIDE, STM32CubeCLT, Arm GNU Toolchain)
#      newest version first
#   3. Whatever arm-none-eabi-gcc is on PATH (last resort — the Homebrew
#      formula ships without newlib, see the sysroot check below)
if(DEFINED TOOLCHAIN_DIR)
    set(_TC_CANDIDATE_DIRS "${TOOLCHAIN_DIR}")
else()
    set(_TC_GLOBS
        # STM32CubeIDE bundled GNU tools
        "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin"
        "/Applications/STM32CubeIDE*.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin"
        "/opt/st/stm32cubeide*/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin"
        # STM32Cube bundles (newer CubeIDE / VS Code extension installs)
        "$ENV{LOCALAPPDATA}/stm32cube/bundles/gnu-tools-for-stm32/*/bin"
        "$ENV{HOME}/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/*/bin"
        "$ENV{HOME}/Library/Application Support/stm32cube/bundles/gnu-tools-for-stm32/*/bin"
        "$ENV{HOME}/.local/share/stm32cube/bundles/gnu-tools-for-stm32/*/bin"
        # STM32CubeCLT
        "C:/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin"
        "/opt/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin"
        # Arm's official GNU toolchain
        "/Applications/ArmGNUToolchain/*/arm-none-eabi/bin"
        "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin"
        "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin"
    )
    set(_TC_CANDIDATE_DIRS "")
    foreach(_glob IN LISTS _TC_GLOBS)
        file(GLOB _hits LIST_DIRECTORIES true "${_glob}")
        if(_hits)
            list(SORT _hits ORDER DESCENDING)   # newest version first
            list(APPEND _TC_CANDIDATE_DIRS ${_hits})
        endif()
    endforeach()
endif()

# HINTS are searched before the system PATH, so a full install wins over
# a bare compiler that happens to be on PATH.
find_program(ARM_NONE_EABI_GCC
    NAMES ${TOOLCHAIN_PREFIX}gcc
    HINTS ${_TC_CANDIDATE_DIRS}
    REQUIRED
)
get_filename_component(_TC_BIN "${ARM_NONE_EABI_GCC}" DIRECTORY)
message(STATUS "ARM toolchain: ${_TC_BIN}")

find_program(ARM_NONE_EABI_GXX
    NAMES ${TOOLCHAIN_PREFIX}g++
    HINTS "${_TC_BIN}"
    NO_DEFAULT_PATH
)
if(NOT ARM_NONE_EABI_GXX)
    set(ARM_NONE_EABI_GXX "${ARM_NONE_EABI_GCC}")   # C-only toolchain
endif()

set(CMAKE_C_COMPILER   "${ARM_NONE_EABI_GCC}")
set(CMAKE_CXX_COMPILER "${ARM_NONE_EABI_GXX}")
set(CMAKE_ASM_COMPILER "${ARM_NONE_EABI_GCC}")

# Prevent CMake from testing the compiler with a full executable link
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(ASB_Supervisor C CXX ASM)

# Locate binutils next to the resolved compiler so post-build steps work
# even when the toolchain bin dir is not on PATH (Windows)
get_filename_component(_TC_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(CMAKE_OBJCOPY NAMES ${TOOLCHAIN_PREFIX}objcopy HINTS "${_TC_BIN_DIR}" REQUIRED)
find_program(CMAKE_SIZE    NAMES ${TOOLCHAIN_PREFIX}size    HINTS "${_TC_BIN_DIR}" REQUIRED)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# ---------------------------------------------------------------------------
# MCU flags (STM32F446RE — Cortex-M4, FPU)
# ---------------------------------------------------------------------------
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
file(GLOB_RECURSE CORE_SOURCES
    "${CMAKE_SOURCE_DIR}/Core/Src/*.c"
)

file(GLOB_RECURSE HAL_SOURCES
    "${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c"
)

set(STARTUP_FILE
    "${CMAKE_SOURCE_DIR}/Core/Startup/startup_stm32f446retx.s"
)

# ---------------------------------------------------------------------------
# Sysroot / C library headers
# ---------------------------------------------------------------------------
# The Homebrew formula `arm-none-eabi-gcc` is built --without-headers and ships
# no newlib, so string.h and friends are missing.  Ask the compiler where its
# sysroot is; if it is non-empty and contains a usable include tree we add it.
# With ARM's official gcc-arm-embedded (cask) the sysroot is fully populated.
execute_process(
    COMMAND ${CMAKE_C_COMPILER}
            -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
            -print-sysroot
    OUTPUT_VARIABLE TOOLCHAIN_SYSROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(TOOLCHAIN_SYSROOT AND EXISTS "${TOOLCHAIN_SYSROOT}/include/string.h")
    message(STATUS "Toolchain sysroot: ${TOOLCHAIN_SYSROOT}")
    set(SYSROOT_FLAGS --sysroot=${TOOLCHAIN_SYSROOT})
else()
    # Sysroot is absent or empty (common with Homebrew arm-none-eabi-gcc).
    # Fall back to the GCC-internal include dir for compiler-provided headers
    # (stdint.h, stddef.h, …) and warn loudly that newlib is required.
    execute_process(
        COMMAND ${CMAKE_C_COMPILER}
                -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
                -print-file-name=include
        OUTPUT_VARIABLE GCC_INTERNAL_INCLUDE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(GCC_INTERNAL_INCLUDE AND EXISTS "${GCC_INTERNAL_INCLUDE}")
        message(STATUS "GCC internal include: ${GCC_INTERNAL_INCLUDE}")
        set(EXTRA_SYSTEM_INCLUDES "${GCC_INTERNAL_INCLUDE}")
    endif()

    set(SYSROOT_FLAGS "")
    message(WARNING
        "\n"
        "  arm-none-eabi-gcc has no sysroot with C library headers.\n"
        "  Standard headers such as string.h will be missing.\n"
        "\n"
        "  Fix: install ARM's official toolchain (includes newlib):\n"
        "    brew install --cask gcc-arm-embedded\n"
        "\n"
        "  Then re-run cmake pointing at that toolchain:\n"
        "    cmake .. -DTOOLCHAIN_DIR=\"/Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin\"\n"
    )
endif()

# ---------------------------------------------------------------------------
# Include directories
# ---------------------------------------------------------------------------
set(INCLUDE_DIRS
    "${CMAKE_SOURCE_DIR}/Core/Inc"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
    "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
    "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include"
)

if(DEFINED EXTRA_SYSTEM_INCLUDES)
    list(APPEND INCLUDE_DIRS "${EXTRA_SYSTEM_INCLUDES}")
endif()

# ---------------------------------------------------------------------------
# Compile definitions
# ---------------------------------------------------------------------------
set(COMPILE_DEFS
    USE_HAL_DRIVER
    STM32F446xx
)

# ---------------------------------------------------------------------------
# Linker script
# ---------------------------------------------------------------------------
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F446RETX_FLASH.ld")

# ---------------------------------------------------------------------------
# Executable target
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}.elf
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${STARTUP_FILE}
)

target_include_directories(${PROJECT_NAME}.elf PRIVATE ${INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE ${COMPILE_DEFS})

# Apply MCU + common flags to every language via generator expressions so they
# are always present regardless of build type or CMake version.
target_compile_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    ${SYSROOT_FLAGS}
    -fdata-sections
    -ffunction-sections
    $<$<COMPILE_LANGUAGE:C>:   -Wall -fstack-usage>
    $<$<COMPILE_LANGUAGE:CXX>: -Wall -fstack-usage>
    $<$<COMPILE_LANGUAGE:ASM>: -x assembler-with-cpp>
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    ${SYSROOT_FLAGS}
    -T${LINKER_SCRIPT}
    -lc -lm
    -specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map,--cref
    -Wl,--print-memory-usage
)

# ---------------------------------------------------------------------------
# Post-build: generate .bin and print size
# ---------------------------------------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating ${PROJECT_NAME}.bin"
)
