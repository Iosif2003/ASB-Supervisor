# Top-level wrapper Makefile for STM32CubeIDE generated Debug makefile.
# Usage (PowerShell): mingw32-make all | mingw32-make clean

PROJECT_DIR := $(abspath .)
DEBUG_DIR := $(PROJECT_DIR)/Debug

# You can override ARM_GCC_BIN from command line or environment:
# mingw32-make ARM_GCC_BIN="C:/path/to/bin" all
ARM_GCC_BIN ?= C:/ST/STM32CubeIDE_2.1.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin

ifeq ($(wildcard $(ARM_GCC_BIN)/arm-none-eabi-gcc.exe),)
$(warning arm-none-eabi-gcc.exe not found in ARM_GCC_BIN='$(ARM_GCC_BIN)')
$(warning Set ARM_GCC_BIN to your toolchain bin directory.)
endif

export PATH := $(ARM_GCC_BIN);$(PATH)

.PHONY: all clean rebuild size list

all:
	@echo Building firmware ELF only (no size/list post-steps)
	mingw32-make -C "$(DEBUG_DIR)" "EBS TEST Code.elf"

clean:
	mingw32-make -C "$(DEBUG_DIR)" clean

rebuild: clean all

size: all
	arm-none-eabi-size "$(DEBUG_DIR)/EBS TEST Code.elf"

list: all
	arm-none-eabi-objdump -h -S "$(DEBUG_DIR)/EBS TEST Code.elf" > "$(DEBUG_DIR)/EBS TEST Code.list"
