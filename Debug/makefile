################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include wiznet/Src/subdir.mk
-include libmodbus/Src/subdir.mk
-include datalink/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/w25qxx/subdir.mk
-include Core/Src/TimeStamp/subdir.mk
-include Core/Src/StepperMotor/subdir.mk
-include Core/Src/PID/subdir.mk
-include Core/Src/MyTimer/subdir.mk
-include Core/Src/MotorSafety/subdir.mk
-include Core/Src/MotorController/subdir.mk
-include Core/Src/Filters/subdir.mk
-include Core/Src/Encoder/subdir.mk
-include Core/Src/DM542T/subdir.mk
-include Core/Src/BTS7960/subdir.mk
-include Core/Src/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := GenIgniSys_FW
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
GenIgniSys_FW.elf \

MAP_FILES += \
GenIgniSys_FW.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
GenIgniSys_FW.list \

OBJCOPY_HEX += \
GenIgniSys_FW.hex \


# All Target
all: main-build

# Main-build Target
main-build: GenIgniSys_FW.elf secondary-outputs

# Tool invocations
GenIgniSys_FW.elf GenIgniSys_FW.map: $(OBJS) $(USER_OBJS) C:\Users\cfl20\shared_projects\GenIgniSys_FW\STM32F407VETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "GenIgniSys_FW.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\cfl20\shared_projects\GenIgniSys_FW\STM32F407VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="GenIgniSys_FW.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

GenIgniSys_FW.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "GenIgniSys_FW.list"
	@echo 'Finished building: $@'
	@echo ' '

GenIgniSys_FW.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "GenIgniSys_FW.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) GenIgniSys_FW.elf GenIgniSys_FW.hex GenIgniSys_FW.list GenIgniSys_FW.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
