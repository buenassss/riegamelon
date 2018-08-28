################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SaveYourPlants.axf

# Tool invocations
SaveYourPlants.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/nuriafernandezmingo/Documents/Projects/FreeRTOSlib/Debug" -L"/Users/nuriafernandezmingo/Documents/Projects/lpc_board_nxp_lpcxpresso_1769/Debug" -L"/Users/nuriafernandezmingo/Documents/Projects/lpc_chip_175x_6x/Debug" -Xlinker -Map="SaveYourPlants.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m3 -mthumb -T "SaveYourPlants_Debug.ld" -o "SaveYourPlants.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) SaveYourPlants.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "SaveYourPlants.axf"; # arm-none-eabi-objcopy -v -O binary "SaveYourPlants.axf" "SaveYourPlants.bin" ; # checksum -p LPC1769 -d "SaveYourPlants.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
