################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
SMART\ DRIVER\ INTERNAL\ CLOCK.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SMART\ DRIVER\ INTERNAL\ CLOCK.list \

OBJCOPY_BIN += \
SMART\ DRIVER\ INTERNAL\ CLOCK.bin \


# All Target
all:
	+@make --no-print-directory main-build && make --no-print-directory post-build

# Main-build Target
main-build: SMART\ DRIVER\ INTERNAL\ CLOCK.elf secondary-outputs

# Tool invocations
SMART\ DRIVER\ INTERNAL\ CLOCK.elf: $(OBJS) $(USER_OBJS) ../STM32F103C8_FLASH.ld
	arm-none-eabi-gcc -o "SMART DRIVER INTERNAL CLOCK.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"../STM32F103C8_FLASH.ld" --specs=nosys.specs -Wl,-Map="SMART DRIVER INTERNAL CLOCK.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SMART\ DRIVER\ INTERNAL\ CLOCK.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SMART DRIVER INTERNAL CLOCK.list"
	@echo 'Finished building: $@'
	@echo ' '

SMART\ DRIVER\ INTERNAL\ CLOCK.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SMART DRIVER INTERNAL CLOCK.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy -O ihex "SMART DRIVER INTERNAL CLOCK.elf" "SMART DRIVER INTERNAL CLOCK.hex"
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
