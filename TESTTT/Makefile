PROJECT := TESTTT
BUILD_DIR := build

CUBEIDE_ROOT ?= $(firstword $(wildcard C:/ST/STM32CubeIDE_*/STM32CubeIDE))
GCC_BIN_DIR ?= $(firstword $(wildcard $(CUBEIDE_ROOT)/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin))
MAKE_BIN_DIR ?= $(firstword $(wildcard $(CUBEIDE_ROOT)/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_*/tools/bin))

ifeq ($(strip $(GCC_BIN_DIR)),)
$(error STM32CubeIDE GNU toolchain not found. Set CUBEIDE_ROOT or GCC_BIN_DIR)
endif

CC := $(GCC_BIN_DIR)/arm-none-eabi-gcc
OBJCOPY := $(GCC_BIN_DIR)/arm-none-eabi-objcopy
OBJDUMP := $(GCC_BIN_DIR)/arm-none-eabi-objdump
SIZE := $(GCC_BIN_DIR)/arm-none-eabi-size

CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
C_DEFS := -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx
INCLUDES := \
	-ICore/Inc \
	-IDrivers/STM32F4xx_HAL_Driver/Inc \
	-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
	-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
	-IDrivers/CMSIS/Include

COMMON_FLAGS := $(CPUFLAGS) $(C_DEFS) $(INCLUDES) -ffunction-sections -fdata-sections -Wall
CFLAGS := $(COMMON_FLAGS) -std=gnu11 -g3 -O0 --specs=nano.specs
ASFLAGS := $(CPUFLAGS) $(C_DEFS) $(INCLUDES) -g3 -x assembler-with-cpp
LDFLAGS := $(CPUFLAGS) -TSTM32F446RETX_FLASH.ld --specs=nosys.specs --specs=nano.specs -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group

SRCS_C := \
	Core/Src/main.c \
	Core/Src/stm32f4xx_hal_msp.c \
	Core/Src/stm32f4xx_it.c \
	Core/Src/syscalls.c \
	Core/Src/sysmem.c \
	Core/Src/system_stm32f4xx.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c

SRCS_S := Core/Startup/startup_stm32f446retx.s

OBJS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRCS_C)) $(patsubst %.s,$(BUILD_DIR)/%.o,$(SRCS_S))
DEPS := $(OBJS:.o=.d)

.PHONY: all clean size list

all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).bin size list

$(BUILD_DIR)/$(PROJECT).elf: $(OBJS) STM32F446RETX_FLASH.ld | $(BUILD_DIR)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(PROJECT).hex: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary -S $< $@

size: $(BUILD_DIR)/$(PROJECT).elf
	$(SIZE) $<

list: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJDUMP) -h -S $< > $(BUILD_DIR)/$(PROJECT).list

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@if not exist "$(subst /,\,$(@D))" mkdir "$(subst /,\,$(@D))"
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@if not exist "$(subst /,\,$(@D))" mkdir "$(subst /,\,$(@D))"
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR):
	@if not exist "$(subst /,\,$@)" mkdir "$(subst /,\,$@)"

clean:
	@if exist "$(subst /,\,$(BUILD_DIR))" rmdir /s /q "$(subst /,\,$(BUILD_DIR))"

-include $(DEPS)
