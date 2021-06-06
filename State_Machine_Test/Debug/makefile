################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
State_Machine_Test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
State_Machine_Test.list \

OBJCOPY_BIN += \
State_Machine_Test.bin \


# All Target
all: State_Machine_Test.elf secondary-outputs

# Tool invocations
State_Machine_Test.elf: $(OBJS) $(USER_OBJS) C:\Users\Sasha123\Documents\GitHub\LCR-Meter\State_Machine_Test\STM32F412ZGTX_FLASH.ld
	arm-none-eabi-gcc -o "State_Machine_Test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Sasha123\Documents\GitHub\LCR-Meter\State_Machine_Test\STM32F412ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="State_Machine_Test.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

State_Machine_Test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "State_Machine_Test.list"
	@echo 'Finished building: $@'
	@echo ' '

State_Machine_Test.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "State_Machine_Test.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
