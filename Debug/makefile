################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_Device/Target/subdir.mk
-include USB_Device/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
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
O4_DH_pinmux.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
O4_DH_pinmux.list \

OBJCOPY_BIN += \
O4_DH_pinmux.bin \


# All Target
all: O4_DH_pinmux.elf secondary-outputs

# Tool invocations
O4_DH_pinmux.elf: $(OBJS) $(USER_OBJS) D:\software\STM32CubeIDE\workspace_1.0.0\O4_DH_pinmux\STM32G471CETX_FLASH.ld
	arm-none-eabi-gcc -o "O4_DH_pinmux.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\software\STM32CubeIDE\workspace_1.0.0\O4_DH_pinmux\STM32G471CETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="O4_DH_pinmux.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

O4_DH_pinmux.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "O4_DH_pinmux.list"
	@echo 'Finished building: $@'
	@echo ' '

O4_DH_pinmux.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "O4_DH_pinmux.bin"
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
