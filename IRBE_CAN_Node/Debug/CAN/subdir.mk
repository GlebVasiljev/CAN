################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CAN/CANSPI.c \
../CAN/MCP2515.c 

OBJS += \
./CAN/CANSPI.o \
./CAN/MCP2515.o 

C_DEPS += \
./CAN/CANSPI.d \
./CAN/MCP2515.d 


# Each subdirectory must supply rules for building sources it contributes
CAN/%.o CAN/%.su CAN/%.cyclo: ../CAN/%.c CAN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CAN

clean-CAN:
	-$(RM) ./CAN/CANSPI.cyclo ./CAN/CANSPI.d ./CAN/CANSPI.o ./CAN/CANSPI.su ./CAN/MCP2515.cyclo ./CAN/MCP2515.d ./CAN/MCP2515.o ./CAN/MCP2515.su

.PHONY: clean-CAN

