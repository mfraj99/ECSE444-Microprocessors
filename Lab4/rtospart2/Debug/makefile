################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/Components/wm8994/subdir.mk
-include Core/Components/stmpe811/subdir.mk
-include Core/Components/stmpe1600/subdir.mk
-include Core/Components/st7789h2/subdir.mk
-include Core/Components/st7735/subdir.mk
-include Core/Components/st25dv/subdir.mk
-include Core/Components/ov9655/subdir.mk
-include Core/Components/mfxstm32l152/subdir.mk
-include Core/Components/m24sr/subdir.mk
-include Core/Components/lsm6dsl/subdir.mk
-include Core/Components/lsm303dlhc/subdir.mk
-include Core/Components/lsm303c/subdir.mk
-include Core/Components/ls016b8uy/subdir.mk
-include Core/Components/lps22hb/subdir.mk
-include Core/Components/lis3mdl/subdir.mk
-include Core/Components/l3gd20/subdir.mk
-include Core/Components/hx8347i/subdir.mk
-include Core/Components/hx8347g/subdir.mk
-include Core/Components/hts221/subdir.mk
-include Core/Components/ft6x06/subdir.mk
-include Core/Components/ft5336/subdir.mk
-include Core/Components/ft3x67/subdir.mk
-include Core/Components/cy8c4014lqi/subdir.mk
-include Core/Components/cs43l22/subdir.mk
-include Core/Components/cs42l51/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

BUILD_ARTIFACT_NAME := rtospart2
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
rtospart2.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
rtospart2.list \

OBJCOPY_BIN += \
rtospart2.bin \


# All Target
all: main-build

# Main-build Target
main-build: rtospart2.elf secondary-outputs

# Tool invocations
rtospart2.elf: $(OBJS) $(USER_OBJS) C:\Users\xx420mhfxx\STM32CubeIDE\workspace_1.5.0\rtospart2\STM32L4S5VITX_FLASH.ld
	arm-none-eabi-gcc -o "rtospart2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\xx420mhfxx\STM32CubeIDE\workspace_1.5.0\rtospart2\STM32L4S5VITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="rtospart2.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

rtospart2.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "rtospart2.list"
	@echo 'Finished building: $@'
	@echo ' '

rtospart2.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "rtospart2.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
